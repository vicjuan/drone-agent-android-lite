package com.durendal.droneagent.lite

import android.util.Log
import dji.sdk.keyvalue.value.flightcontroller.FlightControlAuthorityChangeReason
import dji.sdk.keyvalue.value.flightcontroller.FlightCoordinateSystem
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode
import dji.sdk.keyvalue.value.flightcontroller.VerticalControlMode
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam
import dji.sdk.keyvalue.value.flightcontroller.YawControlMode
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.aircraft.virtualstick.VirtualStickManager
import dji.v5.manager.aircraft.virtualstick.VirtualStickState
import dji.v5.manager.aircraft.virtualstick.VirtualStickStateListener
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit

/** Which of the two Mode 2 sticks a sample came from. */
enum class StickSide { LEFT, RIGHT }

/** What the aircraft currently reports about virtual-stick control. */
data class VirtualStickStatus(
    val enabled: Boolean = false,
    val advancedMode: Boolean = false,
    val authority: String = "UNKNOWN",
)

/**
 * MSDK virtual-stick lifecycle plus the fixed-rate sender it requires.
 *
 * The aircraft treats virtual stick as a live control link: it expects a fresh
 * parameter frame at a steady cadence, and it stops honouring the link when the
 * frames stop arriving. Producing frames is therefore this class's job alone —
 * callers only move sticks, and a released stick is a zero, never a missing frame.
 *
 * Axis semantics (advanced mode, BODY frame) as measured on a Mini 4 Pro on
 * 2026-08-14: MSDK's `roll` drives the body X axis (forward/back) and `pitch`
 * drives the body Y axis (right/left) — the opposite of what the field names
 * suggest, and the opposite of the mapping in the main project's
 * DjiBodyVelocityMapping, which has never been flown. The first flight moved
 * sideways on the "forward" button, so the field names lose to the aircraft:
 *   roll = forward m/s        pitch = right m/s
 *   verticalThrottle = up m/s yaw   = clockwise deg/s
 */
class VirtualStickSession(
    private val onStatus: (VirtualStickStatus) -> Unit,
    private val onFrameSummary: (String) -> Unit = {},
) {

    private val manager = VirtualStickManager.getInstance()
    private val sender = Executors.newSingleThreadScheduledExecutor { runnable ->
        Thread(runnable, "LiteVirtualStick").apply { isDaemon = true }
    }

    @Volatile private var forward = 0.0
    @Volatile private var right = 0.0
    @Volatile private var up = 0.0
    @Volatile private var yawRate = 0.0
    private var sendTask: ScheduledFuture<*>? = null

    /** Frames the aircraft accepted since the stream started, and failures. */
    @Volatile private var frameCount = 0L
    @Volatile private var frameFailures = 0L

    private val stateListener = object : VirtualStickStateListener {
        override fun onVirtualStickStateUpdate(state: VirtualStickState) {
            val owner = state.currentFlightControlAuthorityOwner?.name ?: "UNKNOWN"
            // Frames follow authority, in both directions. Only the owner's frames
            // mean anything, so this app goes quiet the moment the aircraft names
            // someone else — and resumes as soon as it names MSDK again.
            //
            // The resume half is not optional: enableVirtualStick returns before the
            // aircraft has finished handing authority over, so the first state update
            // still says RC. A one-way stop there killed the frame stream for the
            // whole flight (2026-08-17): the sticks and the height loop produced
            // commands that were never sent, while takeoff and landing kept working
            // because those are action keys, not frames.
            if (state.isVirtualStickEnable) {
                if (owner == MSDK_AUTHORITY_OWNER) {
                    if (startSending()) Log.i(TAG, "authority is MSDK; frames running")
                } else if (stopSending()) {
                    Log.w(TAG, "authority owner is $owner; frames paused")
                }
            }
            onStatus(
                VirtualStickStatus(
                    enabled = state.isVirtualStickEnable,
                    advancedMode = state.isVirtualStickAdvancedModeEnabled,
                    authority = owner,
                ),
            )
        }

        override fun onChangeReasonUpdate(reason: FlightControlAuthorityChangeReason) {
            Log.i(TAG, "flight control authority changed: ${reason.name}")
        }
    }

    fun start() {
        manager.init()
        manager.setVirtualStickStateListener(stateListener)
    }

    /** Takes control: enables virtual stick, advanced mode, and the frame sender. */
    fun enable(onResult: (String?) -> Unit) {
        zeroAxes()
        manager.enableVirtualStick(
            object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    manager.setVirtualStickAdvancedModeEnabled(true)
                    startSending()
                    Log.i(TAG, "virtual stick enabled at ${FRAME_RATE_HZ}Hz")
                    onResult(null)
                }

                override fun onFailure(error: IDJIError) {
                    Log.e(TAG, "enable refused: $error")
                    onResult(error.description() ?: error.toString())
                }
            },
        )
    }

    /**
     * Returns control to the RC. A neutral frame is sent before the link drops so
     * the aircraft never inherits the last non-zero command as its final input.
     */
    fun disable(onResult: (String?) -> Unit) {
        zeroAxes()
        stopSending()
        runCatching { manager.sendVirtualStickAdvancedParam(currentParam()) }
        manager.setVirtualStickAdvancedModeEnabled(false)
        manager.disableVirtualStick(
            object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    Log.i(TAG, "virtual stick disabled")
                    onResult(null)
                }

                override fun onFailure(error: IDJIError) {
                    Log.e(TAG, "disable refused: $error")
                    onResult(error.description() ?: error.toString())
                }
            },
        )
    }

    /**
     * Mode 2 sample from one stick, in normalised [-1, 1] with y positive up:
     * left = (yaw, climb), right = (lateral, forward). MainActivity is the
     * fail-closed obstacle gate; this class only converts accepted input to
     * aircraft units.
     */
    fun setStick(side: StickSide, x: Double, y: Double) {
        when (side) {
            StickSide.LEFT -> {
                yawRate = x.coerceIn(-1.0, 1.0) * MAX_YAW_DEGREES_PER_SECOND
                up = y.coerceIn(-1.0, 1.0) * MAX_VERTICAL_MPS
            }
            StickSide.RIGHT -> {
                right = x.coerceIn(-1.0, 1.0) * MAX_HORIZONTAL_MPS
                forward = y.coerceIn(-1.0, 1.0) * MAX_HORIZONTAL_MPS
            }
        }
    }

    /**
     * Vertical command in m/s, for a closed loop that holds a height instead of
     * a human pushing the stick. It writes the same axis the left stick writes,
     * so exactly one of the two may be in charge at any moment — the caller
     * decides which, and the aircraft never receives two vertical intents.
     */
    fun setClimbRate(metersPerSecond: Double) {
        up = metersPerSecond.coerceIn(-MAX_VERTICAL_MPS, MAX_VERTICAL_MPS)
    }

    /** Yaw-only command for a closed-loop turn; positive is clockwise. */
    fun setYawRate(degreesPerSecond: Double) {
        yawRate = degreesPerSecond.coerceIn(-MAX_YAW_DEGREES_PER_SECOND, MAX_YAW_DEGREES_PER_SECOND)
    }

    /** Fixed body-forward speed used only by the obstacle-gated pulse. */
    fun setForwardOnly(metersPerSecond: Double) {
        forward = metersPerSecond.coerceIn(-MAX_HORIZONTAL_MPS, MAX_HORIZONTAL_MPS)
        right = 0.0
    }

    /** Body-frame horizontal velocity for autonomous tracking. */
    fun setHorizontalVelocity(
        forwardMetersPerSecond: Double,
        rightMetersPerSecond: Double,
    ) {
        forward = forwardMetersPerSecond.coerceIn(-MAX_HORIZONTAL_MPS, MAX_HORIZONTAL_MPS)
        right = rightMetersPerSecond.coerceIn(-MAX_HORIZONTAL_MPS, MAX_HORIZONTAL_MPS)
    }


    fun close() {
        stopSending()
        sender.shutdownNow()
        runCatching { manager.removeVirtualStickStateListener(stateListener) }
        runCatching { manager.destroy() }
    }

    /** Starts the frame stream; returns true when this call is what started it. */
    @Synchronized
    private fun startSending(): Boolean {
        if (sendTask != null) return false
        val periodMs = 1_000L / FRAME_RATE_HZ
        frameCount = 0
        frameFailures = 0
        sendTask = sender.scheduleAtFixedRate(
            {
                runCatching { manager.sendVirtualStickAdvancedParam(currentParam()) }
                    .onSuccess { frameCount += 1 }
                    .onFailure {
                        frameFailures += 1
                        Log.w(TAG, "frame send failed", it)
                    }
                // One line per second, and only while something is actually being
                // commanded: enough to prove the stream is alive and what it
                // carries, without the flood that destroyed earlier evidence.
                if (frameCount % FRAME_RATE_HZ == 0L) {
                    val moving = forward != 0.0 || right != 0.0 || up != 0.0 || yawRate != 0.0
                    if (moving || frameFailures > 0) {
                        onFrameSummary(
                            "frames=$frameCount fails=$frameFailures " +
                                "fwd=%.2f right=%.2f up=%.2f yaw=%.1f".format(forward, right, up, yawRate),
                        )
                    }
                }
            },
            0L,
            periodMs,
            TimeUnit.MILLISECONDS,
        )
        return true
    }

    /** Stops the frame stream; returns true when this call is what stopped it. */
    @Synchronized
    private fun stopSending(): Boolean {
        val task = sendTask ?: return false
        task.cancel(false)
        sendTask = null
        return true
    }

    private fun zeroAxes() {
        forward = 0.0
        right = 0.0
        up = 0.0
        yawRate = 0.0
    }

    private fun currentParam(): VirtualStickFlightControlParam =
        VirtualStickFlightControlParam().apply {
            roll = forward
            pitch = right
            verticalThrottle = up
            yaw = yawRate
            rollPitchControlMode = RollPitchControlMode.VELOCITY
            verticalControlMode = VerticalControlMode.VELOCITY
            yawControlMode = YawControlMode.ANGULAR_VELOCITY
            rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
        }

    companion object {
        /** MSDK's documented virtual-stick cadence; the main project uses the same rate. */
        const val FRAME_RATE_HZ = 20L

        /** Authority owner name that means this app's frames are the ones being flown. */
        const val MSDK_AUTHORITY_OWNER = "MSDK"

        /** Deliberately gentle limits for indoor control. */
        const val MAX_HORIZONTAL_MPS = 0.5
        const val MAX_VERTICAL_MPS = 0.3
        const val MAX_YAW_DEGREES_PER_SECOND = 20.0

        private const val TAG = "LiteVirtualStick"
    }
}
