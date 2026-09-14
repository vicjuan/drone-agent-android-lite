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
import dji.v5.manager.interfaces.IVirtualStickManager
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
) {
    /** UNKNOWN is accepted by operator policy, but never enables a disabled session. */
    val hasMsdkAuthority: Boolean
        get() = enabled && (authority == "MSDK" || authority == "UNKNOWN")

    /** An unknown owner only confirms release once virtual stick reports disabled. */
    val isReleased: Boolean
        get() = authority == "RC" || (!enabled && authority == "UNKNOWN")
}

internal enum class VirtualStickFrameRate(
    val hertz: Long,
) {
    BASELINE_20(20L),
    EXPERIMENT_40(40L),
    ;

    val periodMillis: Long
        get() = 1_000L / hertz

    fun toggled(): VirtualStickFrameRate =
        if (this == BASELINE_20) EXPERIMENT_40 else BASELINE_20
}


data class VirtualStickFrameProfile(
    val sentAtNanos: Long,
    val sendDurationNanos: Long,
    val horizontalCommandUpdatedAtNanos: Long,
    val configuredRateHz: Long,
    val succeeded: Boolean,
    val rollPitchMode: String,
    val forwardMetersPerSecond: Double?,
    val rightMetersPerSecond: Double?,
    val rollDegrees: Double?,
    val pitchDegrees: Double?,
    val climbMetersPerSecond: Double,
    val yawMode: String,
    val yawValue: Double,
)
/**
 * MSDK virtual-stick lifecycle plus the fixed-rate sender it requires.
 *
 * The aircraft treats virtual stick as a live control link: it expects a fresh
 * parameter frame at a steady cadence, and it stops honouring the link when the
 * frames stop arriving. Producing frames is therefore this class's job alone —
 * callers only move sticks, and a released stick is a zero, never a missing frame.
 *
 * VELOCITY axis semantics (advanced mode, BODY frame) as measured on a Mini 4 Pro on
 * 2026-08-14: MSDK's `roll` drives the body X axis (forward/back) and `pitch`
 * drives the body Y axis (right/left) — the opposite of what the field names
 * suggest, and the opposite of the mapping in the main project's
 * DjiBodyVelocityMapping, which has never been flown. The first flight moved
 * sideways on the "forward" button, so the field names lose to the aircraft:
 *   roll = forward m/s        pitch = right m/s
 *   verticalThrottle = up m/s yaw   = clockwise rate or ground-frame heading
 * ANGLE commands instead use raw DJI rotation fields in degrees: positive roll
 * tilts right and negative pitch tilts forward. They never use the velocity mapping.
 */
class VirtualStickSession(
    private val onStatus: (VirtualStickStatus) -> Unit,
    private val onFrameSummary: (String) -> Unit = {},
    private val onFrameSent: (VirtualStickFrameProfile) -> Unit = {},
    private val onBeforeFrame: (Long) -> Unit = {},
    private val manager: IVirtualStickManager = VirtualStickManager.getInstance(),
) {

    private val sender = Executors.newSingleThreadScheduledExecutor { runnable ->
        Thread(runnable, "LiteVirtualStick").apply { isDaemon = true }
    }

    // Mode and raw DJI horizontal fields are read/written together under this monitor.
    private var horizontalMode = RollPitchControlMode.VELOCITY
    private var commandedRoll = 0.0
    private var commandedPitch = 0.0
    @Volatile private var commandedClimbMetersPerSecond = 0.0
    @Volatile private var yawCommand: YawCommand = YawCommand.Rate(0.0)
    @Volatile private var horizontalCommandUpdatedAtNanos = 0L
    private var horizontalCommandValidUntilNanos = 0L
    private var sendTask: ScheduledFuture<*>? = null
    private var sendGeneration = 0L
    private var selectedFrameRate = VirtualStickFrameRate.EXPERIMENT_40
    @Volatile private var firstFrameSentAtNanos = 0L

    /** Frames the aircraft accepted since the stream started, and failures. */
    @Volatile private var frameCount = 0L
    @Volatile private var frameFailures = 0L

    private val stateListener = object : VirtualStickStateListener {
        override fun onVirtualStickStateUpdate(state: VirtualStickState) {
            val status = VirtualStickStatus(
                enabled = state.isVirtualStickEnable,
                advancedMode = state.isVirtualStickAdvancedModeEnabled,
                authority = state.currentFlightControlAuthorityOwner?.name ?: "UNKNOWN",
            )
            // Mini 3 Pro reported enabled + UNKNOWN after a successful enable.
            // Accept that owner like MSDK; explicit takeover or disable still stops frames.
            // Resume on later accepted states: enable may initially report RC.
            if (status.hasMsdkAuthority) {
                if (startSending()) Log.i(TAG, "authority owner is ${status.authority}; frames running")
            } else if (stopSending()) {
                Log.w(TAG, "enabled=${status.enabled} authority=${status.authority}; frames paused")
            }
            onStatus(status)
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
                    Log.i(TAG, "virtual stick enabled at ${frameRate().hertz}Hz")
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
    @Synchronized
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

    @Synchronized
    internal fun selectFrameRate(frameRate: VirtualStickFrameRate): Boolean {
        if (sendTask != null) return false
        selectedFrameRate = frameRate
        return true
    }

    @Synchronized
    internal fun frameRate(): VirtualStickFrameRate = selectedFrameRate

    /**
     * Mode 2 sample from one stick, in normalised [-1, 1] with y positive up:
     * left = (yaw, climb), right = (lateral, forward). MainActivity is the
     * fail-closed obstacle gate; this class only converts accepted input to
     * aircraft units.
     */
    @Synchronized
    fun setStick(side: StickSide, x: Double, y: Double) {
        when (side) {
            StickSide.LEFT -> {
                yawCommand =
                    YawCommand.Rate(x.coerceIn(-1.0, 1.0) * MAX_YAW_DEGREES_PER_SECOND)
                commandedClimbMetersPerSecond = y.coerceIn(-1.0, 1.0) * MAX_VERTICAL_MPS
            }
            StickSide.RIGHT -> {
                horizontalMode = RollPitchControlMode.VELOCITY
                horizontalCommandValidUntilNanos = 0L
                commandedPitch = x.coerceIn(-1.0, 1.0) * MAX_HORIZONTAL_MPS
                commandedRoll = y.coerceIn(-1.0, 1.0) * MAX_HORIZONTAL_MPS
                horizontalCommandUpdatedAtNanos = System.nanoTime()
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
    @Synchronized
    fun setForwardOnly(metersPerSecond: Double) {
        horizontalMode = RollPitchControlMode.VELOCITY
        horizontalCommandValidUntilNanos = 0L
        commandedRoll = metersPerSecond.coerceIn(-MAX_HORIZONTAL_MPS, MAX_HORIZONTAL_MPS)
        commandedPitch = 0.0
        horizontalCommandUpdatedAtNanos = System.nanoTime()
    }

    /**
     * Body-frame horizontal velocity for autonomous tracking. A positive deadline
     * opts into sender-thread neutralisation even when the caller stops ticking.
     * Zero keeps the legacy unbounded command lifetime.
     * A positive magnitude limit opts into the bounded stage-two envelope for
     * this command only; zero retains the legacy per-axis 2.0 m/s clamp.
     */
    @Synchronized
    fun setHorizontalVelocity(
        forwardMetersPerSecond: Double,
        rightMetersPerSecond: Double,
        validUntilNanos: Long = 0L,
        maximumMagnitudeMetersPerSecond: Double = 0.0,
    ) {
        require(validUntilNanos >= 0L)
        require(
            maximumMagnitudeMetersPerSecond.isFinite() &&
                maximumMagnitudeMetersPerSecond in 0.0..MAX_EXPLICIT_HORIZONTAL_MPS,
        )
        if (maximumMagnitudeMetersPerSecond > 0.0) {
            require(forwardMetersPerSecond.isFinite() && rightMetersPerSecond.isFinite())
        }
        horizontalMode = RollPitchControlMode.VELOCITY
        horizontalCommandValidUntilNanos = validUntilNanos
        if (maximumMagnitudeMetersPerSecond == 0.0) {
            commandedRoll =
                forwardMetersPerSecond.coerceIn(-AUTONOMOUS_MAX_HORIZONTAL_MPS, AUTONOMOUS_MAX_HORIZONTAL_MPS)
            commandedPitch =
                rightMetersPerSecond.coerceIn(-AUTONOMOUS_MAX_HORIZONTAL_MPS, AUTONOMOUS_MAX_HORIZONTAL_MPS)
        } else {
            val magnitude = kotlin.math.hypot(forwardMetersPerSecond, rightMetersPerSecond)
            val scale =
                if (magnitude > maximumMagnitudeMetersPerSecond) {
                    maximumMagnitudeMetersPerSecond / magnitude
                } else {
                    1.0
                }
            commandedRoll = forwardMetersPerSecond * scale
            commandedPitch = rightMetersPerSecond * scale
        }
        horizontalCommandUpdatedAtNanos = System.nanoTime()
    }

    /**
     * Raw DJI ANGLE rotation fields, not the measured VELOCITY field mapping.
     * Every accepted command has at most a 150 ms lease; rejection clears any
     * prior tilt to VELOCITY zero. Commands cannot queue while authority is paused.
     */
    @Synchronized
    fun setHorizontalAngles(
        rollDegrees: Double,
        pitchDegrees: Double,
        validUntilNanos: Long,
    ): Boolean {
        val nowNanos = System.nanoTime()
        if (
            !rollDegrees.isFinite() || !pitchDegrees.isFinite() ||
            kotlin.math.hypot(rollDegrees, pitchDegrees) > MAX_HORIZONTAL_ANGLE_DEGREES ||
            validUntilNanos <= 0L || validUntilNanos <= nowNanos || sendTask == null
        ) {
            zeroHorizontal(nowNanos)
            return false
        }
        horizontalMode = RollPitchControlMode.ANGLE
        commandedRoll = rollDegrees
        commandedPitch = pitchDegrees
        horizontalCommandValidUntilNanos =
            minOf(validUntilNanos, nowNanos + MAX_HORIZONTAL_ANGLE_LEASE_NANOS)
        horizontalCommandUpdatedAtNanos = nowNanos
        return true
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
        if (sender.isShutdown || sendTask != null) return false
        val activeFrameRate = selectedFrameRate
        val generation = ++sendGeneration
        frameCount = 0
        frameFailures = 0
        firstFrameSentAtNanos = 0L
        sendTask = sender.scheduleAtFixedRate(
            {
                synchronized(this@VirtualStickSession) {
                    if (sendTask == null || generation != sendGeneration) return@scheduleAtFixedRate
                }
                // Prepare time-sensitive phase changes before sampling the frame,
                // outside the transport monitor to avoid caller/session lock inversion.
                try {
                    onBeforeFrame(System.nanoTime())
                } catch (error: Throwable) {
                    zeroAxes()
                    Log.w(TAG, "frame preparation failed; sending velocity zero", error)
                }
                val currentYawCommand: YawCommand
                val currentHorizontalCommandUpdatedAtNanos: Long
                val param: VirtualStickFlightControlParam
                val sendStartedAtNanos: Long
                val sendCompletedAtNanos: Long
                val sendResult: Result<Unit>
                synchronized(this@VirtualStickSession) {
                    if (sendTask == null || generation != sendGeneration) return@scheduleAtFixedRate
                    currentYawCommand = yawCommand
                    param = currentParam(currentYawCommand)
                    currentHorizontalCommandUpdatedAtNanos = horizontalCommandUpdatedAtNanos
                    sendStartedAtNanos = System.nanoTime()
                    if (firstFrameSentAtNanos == 0L) firstFrameSentAtNanos = sendStartedAtNanos
                    // A neutral update or cancellation must not overtake an older submission.
                    sendResult = runCatching { manager.sendVirtualStickAdvancedParam(param) }
                    sendCompletedAtNanos = System.nanoTime()
                }
                sendResult
                    .onSuccess { frameCount += 1 }
                    .onFailure {
                        frameFailures += 1
                        Log.w(TAG, "frame send failed", it)
                    }
                try {
                    onFrameSent(
                        VirtualStickFrameProfile(
                            sentAtNanos = sendStartedAtNanos,
                            sendDurationNanos = sendCompletedAtNanos - sendStartedAtNanos,
                            horizontalCommandUpdatedAtNanos =
                                currentHorizontalCommandUpdatedAtNanos,
                            configuredRateHz = activeFrameRate.hertz,
                            succeeded = sendResult.isSuccess,
                            rollPitchMode = param.rollPitchControlMode.name,
                            forwardMetersPerSecond =
                                if (param.rollPitchControlMode == RollPitchControlMode.VELOCITY) param.roll else null,
                            rightMetersPerSecond =
                                if (param.rollPitchControlMode == RollPitchControlMode.VELOCITY) param.pitch else null,
                            rollDegrees =
                                if (param.rollPitchControlMode == RollPitchControlMode.ANGLE) param.roll else null,
                            pitchDegrees =
                                if (param.rollPitchControlMode == RollPitchControlMode.ANGLE) param.pitch else null,
                            climbMetersPerSecond = param.verticalThrottle,
                            yawMode = currentYawCommand.mode.name,
                            yawValue = currentYawCommand.value,
                        ),
                    )
                } catch (error: Throwable) {
                    Log.w(TAG, "profiling callback failed", error)
                }
                // One line per second, and only while something is actually being
                // commanded: enough to prove the stream is alive and what it
                // carries, without the flood that destroyed earlier evidence.
                val attemptedFrames = frameCount + frameFailures
                if (attemptedFrames % activeFrameRate.hertz == 0L) {
                    val yawActive =
                        when (currentYawCommand) {
                            is YawCommand.Rate -> currentYawCommand.value != 0.0
                            is YawCommand.Heading -> true
                        }
                    val moving =
                        param.roll != 0.0 ||
                            param.pitch != 0.0 ||
                            param.verticalThrottle != 0.0 ||
                            yawActive
                    if (moving || frameFailures > 0) {
                        val elapsedNanos = sendStartedAtNanos - firstFrameSentAtNanos
                        val actualRateHz =
                            if (attemptedFrames > 1L && elapsedNanos > 0L) {
                                (attemptedFrames - 1L) * NANOS_PER_SECOND / elapsedNanos
                            } else {
                                0.0
                            }
                        val horizontalSummary =
                            if (param.rollPitchControlMode == RollPitchControlMode.ANGLE) {
                                "rollPitchMode=ANGLE rawRollDeg=%.2f rawPitchDeg=%.2f ".format(param.roll, param.pitch)
                            } else {
                                "rollPitchMode=VELOCITY fwdMps=%.2f rightMps=%.2f ".format(param.roll, param.pitch)
                            }
                        onFrameSummary(
                            "frames=$frameCount fails=$frameFailures " +
                                "configuredHz=${activeFrameRate.hertz} actualHz=%.3f ".format(actualRateHz) +
                                horizontalSummary +
                                "upMps=%.2f yawMode=%s yaw=%.1f".format(
                                    param.verticalThrottle,
                                    currentYawCommand.mode.name,
                                    currentYawCommand.value,
                                ),
                        )
                    }
                }
            },
            0L,
            activeFrameRate.periodMillis,
            TimeUnit.MILLISECONDS,
        )
        return true
    }

    /** Stops the frame stream; returns true when this call is what stopped it. */
    @Synchronized
    private fun stopSending(): Boolean {
        // Do not revive an opted-in leg when aircraft authority later returns.
        if (horizontalMode == RollPitchControlMode.ANGLE || horizontalCommandValidUntilNanos != 0L) {
            zeroHorizontal(System.nanoTime())
        }
        val task = sendTask ?: return false
        task.cancel(false)
        sendTask = null
        return true
    }

    @Synchronized
    private fun zeroAxes() {
        zeroHorizontal(System.nanoTime())
        commandedClimbMetersPerSecond = 0.0
        yawCommand = YawCommand.Rate(0.0)
    }

    /** Caller holds the session monitor; no second horizontal command snapshot. */
    private fun zeroHorizontal(nowNanos: Long) {
        horizontalMode = RollPitchControlMode.VELOCITY
        commandedRoll = 0.0
        commandedPitch = 0.0
        horizontalCommandValidUntilNanos = 0L
        horizontalCommandUpdatedAtNanos = nowNanos
    }

    @Synchronized
    private fun currentParam(currentYawCommand: YawCommand): VirtualStickFlightControlParam {
        val nowNanos = System.nanoTime()
        if (horizontalCommandValidUntilNanos != 0L && nowNanos >= horizontalCommandValidUntilNanos) {
            zeroHorizontal(nowNanos)
        }
        return VirtualStickFlightControlParam().apply {
            roll = commandedRoll
            pitch = commandedPitch
            verticalThrottle = commandedClimbMetersPerSecond
            yaw = currentYawCommand.value
            rollPitchControlMode = horizontalMode
            verticalControlMode = VerticalControlMode.VELOCITY
            yawControlMode = currentYawCommand.mode
            rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
        }
    }

    companion object {
        private const val NANOS_PER_SECOND = 1_000_000_000.0
        /** Deliberately gentle limits for indoor control. */
        const val MAX_HORIZONTAL_MPS = 0.5
        const val MAX_VERTICAL_MPS = 0.3
        const val MAX_YAW_DEGREES_PER_SECOND = 20.0

        /**
         * Autonomous body-velocity authority. Manual controls remain deliberately
         * gentle; fixed-heading profiles may explicitly use up to 2.0 m/s while the
         * path controller retains confidence, lookahead, offset, and loss gates.
         */
        const val AUTONOMOUS_MAX_HORIZONTAL_MPS = 2.0
        private const val MAX_EXPLICIT_HORIZONTAL_MPS = 2.35
        private const val MAX_HORIZONTAL_ANGLE_DEGREES = 5.0
        private const val MAX_HORIZONTAL_ANGLE_LEASE_NANOS = 150_000_000L
        const val AUTONOMOUS_MAX_YAW_DEGREES_PER_SECOND = 100.0
        const val MAX_YAW_HEADING_DEGREES = 180.0

        private const val TAG = "LiteVirtualStick"
    }
}
