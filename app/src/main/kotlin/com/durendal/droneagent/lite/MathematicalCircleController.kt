package com.durendal.droneagent.lite

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * Camera- and position-independent circular velocity schedule.
 *
 * Elapsed time defines the ideal tangent velocity of a circle in the earth frame. The
 * current aircraft heading is used only to express that vector in body coordinates.
 * No measured velocity or integrated position changes the commanded trajectory.
 */
internal class MathematicalCircleController(
    diameterMeters: Double = DIAMETER_METERS,
    val secondsPerLap: Double = SECONDS_PER_LAP,
    val lapCount: Int = LAP_COUNT,
) {
    val radiusMeters = diameterMeters / 2.0
    val tangentialSpeedMetersPerSecond = PI * diameterMeters / secondsPerLap
    val angularRateDegreesPerSecond = 360.0 / secondsPerLap
    val scheduledDurationNanos = (secondsPerLap * lapCount * NANOS_PER_SECOND).toLong()

    private val lapDurationNanos = (secondsPerLap * NANOS_PER_SECOND).toLong()
    private var startedAtNanos = 0L
    private var initialHeadingDegrees = 0.0

    init {
        require(diameterMeters.isFinite() && diameterMeters > 0.0)
        require(secondsPerLap.isFinite() && secondsPerLap > 0.0)
        require(lapCount > 0)
    }

    fun start(nowNanos: Long, headingDegrees: Double): MathematicalCircleCommand {
        require(nowNanos > 0L)
        require(headingDegrees.isFinite())
        check(startedAtNanos == 0L) { "mathematical circle already started" }
        startedAtNanos = nowNanos
        initialHeadingDegrees = headingDegrees
        return command(nowNanos, headingDegrees)
    }

    fun command(nowNanos: Long, headingDegrees: Double): MathematicalCircleCommand {
        check(startedAtNanos > 0L) { "mathematical circle has not started" }
        require(nowNanos >= startedAtNanos)
        require(headingDegrees.isFinite())

        val elapsedNanos = nowNanos - startedAtNanos
        val completed = elapsedNanos >= scheduledDurationNanos
        val progressDegrees =
            if (completed) {
                lapCount * 360.0
            } else {
                elapsedNanos.toDouble() / lapDurationNanos * 360.0
            }
        val phaseRadians = Math.toRadians(progressDegrees % 360.0)
        val desiredForward =
            if (completed) 0.0 else tangentialSpeedMetersPerSecond * cos(phaseRadians)
        val desiredRight =
            if (completed) 0.0 else -tangentialSpeedMetersPerSecond * sin(phaseRadians)

        val initialHeadingRadians = Math.toRadians(initialHeadingDegrees)
        val commandEarthX =
            desiredForward * cos(initialHeadingRadians) - desiredRight * sin(initialHeadingRadians)
        val commandEarthY =
            desiredForward * sin(initialHeadingRadians) + desiredRight * cos(initialHeadingRadians)
        val currentHeadingRadians = Math.toRadians(headingDegrees)
        return MathematicalCircleCommand(
            lap =
                if (completed) {
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
            completed = completed,
        )
    }

    companion object {
        const val DIAMETER_METERS = 1.5
        const val SECONDS_PER_LAP = 7.5
        const val RACING_SECONDS_PER_LAP = 6.75
        const val LAP_COUNT = 3
        private const val NANOS_PER_SECOND = 1_000_000_000.0
    }
}

internal data class MathematicalCircleCommand(
    val lap: Int,
    val totalProgressDegrees: Double,
    val forwardMetersPerSecond: Double,
    val rightMetersPerSecond: Double,
    val tangentHeadingDegrees: Double,
    val completed: Boolean,
)
