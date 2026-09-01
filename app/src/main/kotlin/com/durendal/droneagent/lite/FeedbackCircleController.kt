package com.durendal.droneagent.lite

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Local-position feedback for a camera-independent circle.
 *
 * DJI's fused earth-frame velocity is integrated from the launch point. The
 * controller follows a time-indexed circle with velocity feed-forward plus
 * bounded position-error feedback, then closes the final residual before
 * declaring completion. It does not pretend elapsed time alone is a physical lap.
 */
internal class FeedbackCircleController(
    diameterMeters: Double = DIAMETER_METERS,
    secondsPerLap: Double = SECONDS_PER_LAP,
    val lapCount: Int = LAP_COUNT,
) {
    val radiusMeters = diameterMeters / 2.0
    val tangentialSpeedMetersPerSecond = PI * diameterMeters / secondsPerLap
    val angularRateDegreesPerSecond = 360.0 / secondsPerLap
    val scheduledDurationNanos = (secondsPerLap * lapCount * NANOS_PER_SECOND).toLong()

    private val lapDurationNanos = (secondsPerLap * NANOS_PER_SECOND).toLong()
    private var startedAtNanos = 0L
    private var previousAtNanos = 0L
    private var initialHeadingDegrees = 0.0
    private var estimatedEarthX = 0.0
    private var estimatedEarthY = 0.0
    private var previousVelocityX = 0.0
    private var previousVelocityY = 0.0

    init {
        require(diameterMeters.isFinite() && diameterMeters > 0.0)
        require(secondsPerLap.isFinite() && secondsPerLap > 0.0)
        require(lapCount > 0)
    }

    fun start(
        nowNanos: Long,
        headingDegrees: Double,
        velocityX: Double,
        velocityY: Double,
    ): FeedbackCircleCommand {
        require(nowNanos > 0L)
        require(headingDegrees.isFinite() && velocityX.isFinite() && velocityY.isFinite())
        check(startedAtNanos == 0L) { "feedback circle already started" }
        startedAtNanos = nowNanos
        previousAtNanos = nowNanos
        initialHeadingDegrees = headingDegrees
        previousVelocityX = velocityX
        previousVelocityY = velocityY
        return command(nowNanos, headingDegrees, velocityX, velocityY)
    }

    fun command(
        nowNanos: Long,
        headingDegrees: Double,
        velocityX: Double,
        velocityY: Double,
    ): FeedbackCircleCommand {
        check(startedAtNanos > 0L) { "feedback circle has not started" }
        require(nowNanos >= previousAtNanos)
        require(headingDegrees.isFinite() && velocityX.isFinite() && velocityY.isFinite())

        val deltaSeconds = (nowNanos - previousAtNanos) / NANOS_PER_SECOND
        estimatedEarthX += (previousVelocityX + velocityX) * 0.5 * deltaSeconds
        estimatedEarthY += (previousVelocityY + velocityY) * 0.5 * deltaSeconds
        previousAtNanos = nowNanos
        previousVelocityX = velocityX
        previousVelocityY = velocityY

        val elapsedNanos = nowNanos - startedAtNanos
        val scheduledComplete = elapsedNanos >= scheduledDurationNanos
        val progressDegrees =
            if (scheduledComplete) {
                lapCount * 360.0
            } else {
                elapsedNanos.toDouble() / lapDurationNanos * 360.0
            }
        val phaseRadians = Math.toRadians(progressDegrees % 360.0)
        val desiredForward =
            if (scheduledComplete) 0.0 else tangentialSpeedMetersPerSecond * cos(phaseRadians)
        val desiredRight =
            if (scheduledComplete) 0.0 else -tangentialSpeedMetersPerSecond * sin(phaseRadians)
        val desiredPositionForward =
            if (scheduledComplete) 0.0 else radiusMeters * sin(phaseRadians)
        val desiredPositionRight =
            if (scheduledComplete) 0.0 else radiusMeters * (cos(phaseRadians) - 1.0)

        val initialHeadingRadians = Math.toRadians(initialHeadingDegrees)
        val desiredEarthX =
            desiredPositionForward * cos(initialHeadingRadians) -
                desiredPositionRight * sin(initialHeadingRadians)
        val desiredEarthY =
            desiredPositionForward * sin(initialHeadingRadians) +
                desiredPositionRight * cos(initialHeadingRadians)
        val feedForwardEarthX =
            desiredForward * cos(initialHeadingRadians) - desiredRight * sin(initialHeadingRadians)
        val feedForwardEarthY =
            desiredForward * sin(initialHeadingRadians) + desiredRight * cos(initialHeadingRadians)
        val errorEarthX = desiredEarthX - estimatedEarthX
        val errorEarthY = desiredEarthY - estimatedEarthY
        val positionErrorMeters = hypot(errorEarthX, errorEarthY)
        val correctionScale =
            if (positionErrorMeters == 0.0) {
                0.0
            } else {
                minOf(
                    POSITION_GAIN_PER_SECOND,
                    MAX_POSITION_CORRECTION_METERS_PER_SECOND / positionErrorMeters,
                )
            }
        var commandEarthX = feedForwardEarthX + errorEarthX * correctionScale
        var commandEarthY = feedForwardEarthY + errorEarthY * correctionScale
        val commandSpeed = hypot(commandEarthX, commandEarthY)
        if (commandSpeed > MAX_COMMAND_SPEED_METERS_PER_SECOND) {
            val scale = MAX_COMMAND_SPEED_METERS_PER_SECOND / commandSpeed
            commandEarthX *= scale
            commandEarthY *= scale
        }

        val closureTimedOut =
            scheduledComplete &&
                elapsedNanos >= scheduledDurationNanos + MAX_CLOSURE_DURATION_NANOS
        val completed =
            scheduledComplete &&
                (
                    closureTimedOut ||
                        (
                            positionErrorMeters <= COMPLETION_POSITION_ERROR_METERS &&
                                hypot(velocityX, velocityY) <= COMPLETION_SPEED_METERS_PER_SECOND
                            )
                    )
        if (completed) {
            commandEarthX = 0.0
            commandEarthY = 0.0
        }

        val currentHeadingRadians = Math.toRadians(headingDegrees)
        return FeedbackCircleCommand(
            lap =
                if (scheduledComplete) {
                    lapCount
                } else {
                    (elapsedNanos / lapDurationNanos).toInt() + 1
                },
            totalProgressDegrees = progressDegrees,
            forwardMetersPerSecond =
                commandEarthX * cos(currentHeadingRadians) +
                    commandEarthY * sin(currentHeadingRadians),
            rightMetersPerSecond =
                -commandEarthX * sin(currentHeadingRadians) +
                    commandEarthY * cos(currentHeadingRadians),
            tangentHeadingDegrees = wrapToSignedHeading(initialHeadingDegrees - progressDegrees),
            estimatedEarthX = estimatedEarthX,
            estimatedEarthY = estimatedEarthY,
            positionErrorMeters = positionErrorMeters,
            closing = scheduledComplete && !completed,
            completed = completed,
            closureTimedOut = closureTimedOut,
        )
    }

    companion object {
        const val DIAMETER_METERS = 2.0
        const val SECONDS_PER_LAP = 7.5
        const val LAP_COUNT = 3
        const val POSITION_GAIN_PER_SECOND = 0.8
        const val MAX_POSITION_CORRECTION_METERS_PER_SECOND = 0.45
        const val MAX_COMMAND_SPEED_METERS_PER_SECOND = 1.35
        const val COMPLETION_POSITION_ERROR_METERS = 0.12
        const val COMPLETION_SPEED_METERS_PER_SECOND = 0.15
        const val MAX_CLOSURE_SECONDS = 4.0
        private const val NANOS_PER_SECOND = 1_000_000_000.0
        private const val MAX_CLOSURE_DURATION_NANOS =
            (MAX_CLOSURE_SECONDS * NANOS_PER_SECOND).toLong()
    }
}

internal data class FeedbackCircleCommand(
    val lap: Int,
    val totalProgressDegrees: Double,
    val forwardMetersPerSecond: Double,
    val rightMetersPerSecond: Double,
    val tangentHeadingDegrees: Double,
    val estimatedEarthX: Double,
    val estimatedEarthY: Double,
    val positionErrorMeters: Double,
    val closing: Boolean,
    val completed: Boolean,
    val closureTimedOut: Boolean,
)

/**
 * Tracks active MSDK velocity reads independently of change-driven key-listener callbacks.
 *
 * A stationary aircraft may not emit another listener callback for seconds. Only successful
 * explicit reads refresh this gate; a request that never completes still expires safely.
 */
internal class FeedbackVelocityReadState(
    private val pollIntervalNanos: Long,
    private val timeoutNanos: Long,
) {
    private var requestPending = false
    private var lastRequestedAtNanos = 0L
    private var lastSucceededAtNanos = 0L

    init {
        require(pollIntervalNanos > 0L)
        require(timeoutNanos >= pollIntervalNanos)
    }

    @Synchronized
    fun beginRequest(nowNanos: Long): Boolean {
        require(nowNanos > 0L)
        if (requestPending) return false
        if (
            lastRequestedAtNanos > 0L &&
            nowNanos - lastRequestedAtNanos < pollIntervalNanos
        ) {
            return false
        }
        requestPending = true
        lastRequestedAtNanos = nowNanos
        return true
    }

    @Synchronized
    fun recordSuccess(nowNanos: Long) {
        require(nowNanos > 0L)
        requestPending = false
        lastSucceededAtNanos = nowNanos
    }

    @Synchronized
    fun recordFailure() {
        requestPending = false
    }

    @Synchronized
    fun hasExpired(nowNanos: Long): Boolean =
        lastSucceededAtNanos == 0L ||
            nowNanos - lastSucceededAtNanos > timeoutNanos

    @Synchronized
    fun reset() {
        requestPending = false
        lastRequestedAtNanos = 0L
        lastSucceededAtNanos = 0L
    }
}
