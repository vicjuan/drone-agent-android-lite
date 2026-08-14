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

/** The four body-frame axes a stick command can drive. */
enum class StickAxis { FORWARD, RIGHT, UP, YAW }

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
 * callers only move axes, and a released axis is a zero, never a missing frame.
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
class VirtualStickSession(private val onStatus: (VirtualStickStatus) -> Unit) {

    private val manager = VirtualStickManager.getInstance()
    private val sender = Executors.newSingleThreadScheduledExecutor { runnable ->
        Thread(runnable, "LiteVirtualStick").apply { isDaemon = true }
    }

    @Volatile private var forward = 0.0
    @Volatile private var right = 0.0
    @Volatile private var up = 0.0
    @Volatile private var yawRate = 0.0
    private var sendTask: ScheduledFuture<*>? = null

    private val stateListener = object : VirtualStickStateListener {
        override fun onVirtualStickStateUpdate(state: VirtualStickState) {
            onStatus(
                VirtualStickStatus(
                    enabled = state.isVirtualStickEnable,
                    advancedMode = state.isVirtualStickAdvancedModeEnabled,
                    authority = state.currentFlightControlAuthorityOwner?.name ?: "UNKNOWN",
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

    fun setAxis(axis: StickAxis, value: Double) {
        when (axis) {
            StickAxis.FORWARD -> forward = value
            StickAxis.RIGHT -> right = value
            StickAxis.UP -> up = value
            StickAxis.YAW -> yawRate = value
        }
    }

    fun close() {
        stopSending()
        sender.shutdownNow()
        runCatching { manager.removeVirtualStickStateListener(stateListener) }
        runCatching { manager.destroy() }
    }

    private fun startSending() {
        if (sendTask != null) return
        val periodMs = 1_000L / FRAME_RATE_HZ
        sendTask = sender.scheduleAtFixedRate(
            {
                runCatching { manager.sendVirtualStickAdvancedParam(currentParam()) }
                    .onFailure { Log.w(TAG, "frame send failed", it) }
            },
            0L,
            periodMs,
            TimeUnit.MILLISECONDS,
        )
    }

    private fun stopSending() {
        sendTask?.cancel(false)
        sendTask = null
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
        private const val TAG = "LiteVirtualStick"
    }
}
