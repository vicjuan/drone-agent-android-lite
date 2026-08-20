package com.durendal.droneagent.lite

import dji.sdk.keyvalue.key.GimbalKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.gimbal.CtrlInfo
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotation
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotationMode
import dji.sdk.keyvalue.value.gimbal.GimbalSpeedRotation
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean

internal fun gimbalPitchRateForInput(input: Double, maxRateDegreesPerSecond: Double): Double =
    input.coerceIn(-1.0, 1.0) * maxRateDegreesPerSecond

/**
 * Two-axis camera-gimbal control: a fixed-rate speed stream for the analogue pad,
 * plus [rotateTo] for the absolute poses autonomous tracking needs.
 */
class GimbalSession(
    private val onFailure: (String) -> Unit,
) {
    @Volatile private var pitchRate = 0.0
    @Volatile private var yawRate = 0.0
    @Volatile private var stopPending = false
    private val commandInFlight = AtomicBoolean(false)
    private val failureReported = AtomicBoolean(false)
    private val sender = Executors.newSingleThreadScheduledExecutor { runnable ->
        Thread(runnable, "LiteGimbal").apply { isDaemon = true }
    }
    private var sendTask: ScheduledFuture<*>? = null

    /** Input is normalized to [-1, 1], with positive y tilting the camera up. */
    fun setInput(x: Double, y: Double) {
        val nextYaw = x.coerceIn(-1.0, 1.0) * MAX_RATE_DEGREES_PER_SECOND
        val nextPitch = gimbalPitchRateForInput(y, MAX_RATE_DEGREES_PER_SECOND)
        val wasMoving = pitchRate != 0.0 || yawRate != 0.0
        pitchRate = nextPitch
        yawRate = nextYaw
        val moving = nextPitch != 0.0 || nextYaw != 0.0
        if (moving) {
            stopPending = false
            startSending()
        } else if (wasMoving) {
            stopPending = true
            startSending()
        }
    }

    /**
     * Moves the gimbal to an aircraft-relative pose. Pitch is negative downward;
     * yaw is positive right. Any manual speed command is stopped first.
     */
    fun rotateTo(
        pitchDegrees: Double,
        yawDegrees: Double,
        durationSeconds: Double,
        onDone: (String?) -> Unit,
    ) {
        pitchRate = 0.0
        yawRate = 0.0
        stopPending = false
        val rotation = GimbalAngleRotation().apply {
            mode = GimbalAngleRotationMode.ABSOLUTE_ANGLE
            pitch = pitchDegrees
            roll = 0.0
            yaw = yawDegrees
            pitchIgnored = false
            rollIgnored = true
            yawIgnored = false
            duration = durationSeconds
            jointReferenceUsed = false
            timeout = ((durationSeconds + COMMAND_TIMEOUT_MARGIN_SECONDS) * 1_000).toInt()
        }
        KeyManager.getInstance().performAction(
            KeyTools.createKey(GimbalKey.KeyRotateByAngle),
            rotation,
            object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(result: EmptyMsg) {
                    failureReported.set(false)
                    onDone(null)
                }

                override fun onFailure(error: IDJIError) {
                    val detail = error.description() ?: error.toString()
                    if (failureReported.compareAndSet(false, true)) onFailure(detail)
                    onDone(detail)
                }
            },
        )
    }

    fun close() {
        pitchRate = 0.0
        yawRate = 0.0
        stopPending = false
        sendTask?.cancel(false)
        sendTask = null
        sender.shutdownNow()
        // Do not let an in-flight non-zero callback suppress the final stop.
        KeyManager.getInstance().performAction(
            KeyTools.createKey(GimbalKey.KeyRotateBySpeed),
            speedRotation(0.0, 0.0),
            object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(result: EmptyMsg) = Unit
                override fun onFailure(error: IDJIError) = Unit
            },
        )
    }

    @Synchronized
    private fun startSending() {
        if (sendTask != null) return
        sendTask = sender.scheduleAtFixedRate(
            ::sendDesiredCommand,
            0L,
            1_000L / FRAME_RATE_HZ,
            TimeUnit.MILLISECONDS,
        )
    }

    private fun sendDesiredCommand() {
        val pitch = pitchRate
        val yaw = yawRate
        if (pitch == 0.0 && yaw == 0.0 && !stopPending) return
        sendCommand(pitch, yaw)
    }

    private fun sendCommand(pitch: Double, yaw: Double) {
        if (!commandInFlight.compareAndSet(false, true)) return
        val isStop = pitch == 0.0 && yaw == 0.0
        val rotation = speedRotation(pitch, yaw)
        KeyManager.getInstance().performAction(
            KeyTools.createKey(GimbalKey.KeyRotateBySpeed),
            rotation,
            object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(result: EmptyMsg) {
                    if (isStop) stopPending = false
                    failureReported.set(false)
                    commandInFlight.set(false)
                }

                override fun onFailure(error: IDJIError) {
                    commandInFlight.set(false)
                    if (failureReported.compareAndSet(false, true)) {
                        onFailure(error.description() ?: error.toString())
                    }
                }
            },
        )
    }

    private fun speedRotation(pitch: Double, yaw: Double) =
        GimbalSpeedRotation(
            pitch,
            yaw,
            0.0,
            CtrlInfo(false, false),
        )

    companion object {
        const val MAX_RATE_DEGREES_PER_SECOND = 30.0
        const val FRAME_RATE_HZ = 10L
        private const val COMMAND_TIMEOUT_MARGIN_SECONDS = 1.0
    }
}
