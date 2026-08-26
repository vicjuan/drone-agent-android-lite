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

private sealed interface YawCommand {
    val value: Double
    val mode: YawControlMode

    data class Rate(override val value: Double) : YawCommand {
        override val mode = YawControlMode.ANGULAR_VELOCITY
    }

    data class Heading(override val value: Double) : YawCommand {
        override val mode = YawControlMode.ANGLE
    }
}

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
 *   verticalThrottle = up m/s yaw   = clockwise rate or ground-frame heading
 */
class VirtualStickSession(
    private val onStatus: (VirtualStickStatus) -> Unit,
    private val onFrameSummary: (String) -> Unit = {},
) {

    private val manager = VirtualStickManager.getInstance()
    private val sender = Executors.newSingleThreadScheduledExecutor { runnable ->
        Thread(runnable, "LiteVirtualStick").apply { isDaemon = true }
    }

    @Volatile private var commandedForwardMetersPerSecond = 0.0
    @Volatile private var commandedRightMetersPerSecond = 0.0
    @Volatile private var commandedClimbMetersPerSecond = 0.0
    @Volatile private var yawCommand: YawCommand = YawCommand.Rate(0.0)
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
        runCatching {
            manager.sendVirtualStickAdvancedParam(currentParam(YawCommand.Rate(0.0)))
        }
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
                yawCommand =
                    YawCommand.Rate(x.coerceIn(-1.0, 1.0) * MAX_YAW_DEGREES_PER_SECOND)
                commandedClimbMetersPerSecond = y.coerceIn(-1.0, 1.0) * MAX_VERTICAL_MPS
            }
            StickSide.RIGHT -> {
                commandedRightMetersPerSecond = x.coerceIn(-1.0, 1.0) * MAX_HORIZONTAL_MPS
                commandedForwardMetersPerSecond = y.coerceIn(-1.0, 1.0) * MAX_HORIZONTAL_MPS
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
        commandedClimbMetersPerSecond = metersPerSecond.coerceIn(-MAX_VERTICAL_MPS, MAX_VERTICAL_MPS)
    }

    /** Yaw-rate command for manual or autonomous motion; positive is clockwise. */
    fun setYawRate(degreesPerSecond: Double) {
        yawCommand =
            YawCommand.Rate(
                degreesPerSecond.coerceIn(
                    -AUTONOMOUS_MAX_YAW_DEGREES_PER_SECOND,
                    AUTONOMOUS_MAX_YAW_DEGREES_PER_SECOND,
                ),
            )
    }

    /**
     * Ground-frame heading setpoint for the isolated yaw-capability experiment.
     * Returns false instead of allowing an undefined command into the frame stream.
     */
    fun setYawHeading(degrees: Double): Boolean {
        if (!degrees.isFinite()) return false
        yawCommand =
            YawCommand.Heading(
                degrees.coerceIn(
                    -MAX_YAW_HEADING_DEGREES,
                    MAX_YAW_HEADING_DEGREES,
                ),
            )
        return true
    }

    fun speedLevel(): Double = manager.speedLevel

    /** Fixed body-forward speed used only by the obstacle-gated pulse. */
    fun setForwardOnly(metersPerSecond: Double) {
        commandedForwardMetersPerSecond = metersPerSecond.coerceIn(-MAX_HORIZONTAL_MPS, MAX_HORIZONTAL_MPS)
        commandedRightMetersPerSecond = 0.0
    }

    /** Body-frame horizontal velocity for autonomous tracking. */
    fun setHorizontalVelocity(
        forwardMetersPerSecond: Double,
        rightMetersPerSecond: Double,
    ) {
        commandedForwardMetersPerSecond =
            forwardMetersPerSecond.coerceIn(
                -AUTONOMOUS_MAX_HORIZONTAL_MPS,
                AUTONOMOUS_MAX_HORIZONTAL_MPS,
            )
        commandedRightMetersPerSecond =
            rightMetersPerSecond.coerceIn(
                -AUTONOMOUS_MAX_HORIZONTAL_MPS,
                AUTONOMOUS_MAX_HORIZONTAL_MPS,
            )
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
                val currentYawCommand = yawCommand
                runCatching { manager.sendVirtualStickAdvancedParam(currentParam(currentYawCommand)) }
                    .onSuccess { frameCount += 1 }
                    .onFailure {
                        frameFailures += 1
                        Log.w(TAG, "frame send failed", it)
                    }
                // One line per second, and only while something is actually being
                // commanded: enough to prove the stream is alive and what it
                // carries, without the flood that destroyed earlier evidence.
                if (frameCount % FRAME_RATE_HZ == 0L) {
                    val yawActive =
                        when (currentYawCommand) {
                            is YawCommand.Rate -> currentYawCommand.value != 0.0
                            is YawCommand.Heading -> true
                        }
                    val moving =
                        commandedForwardMetersPerSecond != 0.0 ||
                            commandedRightMetersPerSecond != 0.0 ||
                            commandedClimbMetersPerSecond != 0.0 ||
                            yawActive
                    if (moving || frameFailures > 0) {
                        onFrameSummary(
                            "frames=$frameCount fails=$frameFailures " +
                                "fwd=%.2f right=%.2f up=%.2f yawMode=%s yaw=%.1f".format(
                                    commandedForwardMetersPerSecond,
                                    commandedRightMetersPerSecond,
                                    commandedClimbMetersPerSecond,
                                    currentYawCommand.mode.name,
                                    currentYawCommand.value,
                                ),
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
        commandedForwardMetersPerSecond = 0.0
        commandedRightMetersPerSecond = 0.0
        commandedClimbMetersPerSecond = 0.0
        yawCommand = YawCommand.Rate(0.0)
    }

    private fun currentParam(currentYawCommand: YawCommand): VirtualStickFlightControlParam =
        VirtualStickFlightControlParam().apply {
            roll = commandedForwardMetersPerSecond
            pitch = commandedRightMetersPerSecond
            verticalThrottle = commandedClimbMetersPerSecond
            yaw = currentYawCommand.value
            rollPitchControlMode = RollPitchControlMode.VELOCITY
            verticalControlMode = VerticalControlMode.VELOCITY
            yawControlMode = currentYawCommand.mode
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

        /**
         * MSDK documents ±100°/s for yaw-rate control. Manual controls remain gentle;
         * autonomous experiments may explicitly exercise the full documented range.
         */
        const val AUTONOMOUS_MAX_HORIZONTAL_MPS = 1.0
        const val AUTONOMOUS_MAX_YAW_DEGREES_PER_SECOND = 100.0
        const val MAX_YAW_HEADING_DEGREES = 180.0

        private const val TAG = "LiteVirtualStick"
    }
}
