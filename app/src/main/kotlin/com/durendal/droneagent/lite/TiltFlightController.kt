package com.durendal.droneagent.lite

import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

internal enum class TiltFlightMode { STRAIGHT, CIRCLE, SKATING_CIRCLE }

internal enum class TiltFlightPhase { SETTLE, STRAIGHT, STARTUP, CIRCLE, BRAKE, COMPLETE }

/**
 * Ideal no-drag feedforward experiment, not a measured physical flight path.
 *
 * Velocity-zero settling precedes body-forward tilt. The circle first builds its ideal
 * tangent speed from rest; applying only the rotating inward acceleration from rest
 * would instead produce a drifting cycloid. The startup displacement is outside the circle.
 * Circle acceleration is scheduled in the initial-heading frame and expressed in the
 * actual body frame; heading only changes coordinates, not the scheduled trajectory.
 * Racing follows the tangent heading; skating holds the initial heading.
 * There is no position/velocity feedback or assumed drag compensation. Physical geometry,
 * speed and lap time remain to be measured, including the Mini 4 Pro's raw ANGLE signs.
 */
internal class TiltFlightController(val mode: TiltFlightMode) {
    private val forwardAcceleration = GRAVITY_METERS_PER_SECOND_SQUARED * tan(Math.toRadians(STRAIGHT_TILT_DEGREES))
    val targetSpeedMetersPerSecond =
        if (mode == TiltFlightMode.STRAIGHT) forwardAcceleration * STRAIGHT_DURATION_NANOS / NANOS_PER_SECOND
        else PI * DIAMETER_METERS / SECONDS_PER_LAP
    val startupDurationNanos =
        if (mode == TiltFlightMode.STRAIGHT) {
            0L
        } else {
            (targetSpeedMetersPerSecond / forwardAcceleration * NANOS_PER_SECOND).toLong()
        }
    private val activeDurationNanos =
        if (mode == TiltFlightMode.STRAIGHT) STRAIGHT_DURATION_NANOS
        else startupDurationNanos + CIRCLE_DURATION_NANOS
    val scheduledDurationNanos = SETTLE_DURATION_NANOS + activeDurationNanos + BRAKE_DURATION_NANOS

    private var startedAtNanos = 0L
    private var initialHeadingDegrees = 0.0

    fun start(nowNanos: Long, headingDegrees: Double): TiltFlightCommand {
        require(nowNanos > 0L && nowNanos <= Long.MAX_VALUE - scheduledDurationNanos)
        require(headingDegrees.isFinite())
        check(startedAtNanos == 0L) { "tilt flight already started" }
        startedAtNanos = nowNanos
        initialHeadingDegrees = wrapToSignedHeading(headingDegrees)
        return command(nowNanos, headingDegrees)
    }

    fun command(nowNanos: Long, headingDegrees: Double): TiltFlightCommand {
        check(startedAtNanos > 0L) { "tilt flight has not started" }
        require(nowNanos >= startedAtNanos)
        require(headingDegrees.isFinite())

        val elapsedNanos = nowNanos - startedAtNanos
        val activeEndsAt = SETTLE_DURATION_NANOS + activeDurationNanos
        if (elapsedNanos < SETTLE_DURATION_NANOS) {
            return zeroCommand(TiltFlightPhase.SETTLE, SETTLE_DURATION_NANOS)
        }
        if (elapsedNanos >= scheduledDurationNanos) {
            return zeroCommand(TiltFlightPhase.COMPLETE, scheduledDurationNanos)
        }
        if (elapsedNanos >= activeEndsAt) {
            return zeroCommand(TiltFlightPhase.BRAKE, scheduledDurationNanos)
        }
        if (mode == TiltFlightMode.STRAIGHT || elapsedNanos < SETTLE_DURATION_NANOS + startupDurationNanos) {
            return TiltFlightCommand(
                phase = if (mode == TiltFlightMode.STRAIGHT) TiltFlightPhase.STRAIGHT else TiltFlightPhase.STARTUP,
                rollDegrees = 0.0,
                pitchDegrees = -STRAIGHT_TILT_DEGREES,
                yawHeadingDegrees = initialHeadingDegrees,
                lap = 0,
                progressDegrees = 0.0,
                phaseEndsAtNanos = startedAtNanos + SETTLE_DURATION_NANOS +
                    if (mode == TiltFlightMode.STRAIGHT) STRAIGHT_DURATION_NANOS else startupDurationNanos,
            )
        }

        val circleElapsedNanos = elapsedNanos - SETTLE_DURATION_NANOS - startupDurationNanos
        val progressDegrees = circleElapsedNanos.toDouble() / LAP_DURATION_NANOS * 360.0
        val phaseRadians = Math.toRadians(progressDegrees % 360.0)
        val inwardAcceleration = targetSpeedMetersPerSecond * targetSpeedMetersPerSecond / (DIAMETER_METERS / 2.0)
        // Initial-frame tangent is (v cos phase, -v sin phase), so its LEFT normal
        // acceleration is (-a sin phase, -a cos phase), not the tangent itself.
        val initialForwardAcceleration = -inwardAcceleration * sin(phaseRadians)
        val initialRightAcceleration = -inwardAcceleration * cos(phaseRadians)
        val headingDelta = Math.toRadians(initialHeadingDegrees - wrapToSignedHeading(headingDegrees))
        val forwardAcceleration = initialForwardAcceleration * cos(headingDelta) - initialRightAcceleration * sin(headingDelta)
        val rightAcceleration = initialForwardAcceleration * sin(headingDelta) + initialRightAcceleration * cos(headingDelta)
        val gravity = GRAVITY_METERS_PER_SECOND_SQUARED
        return TiltFlightCommand(
            phase = TiltFlightPhase.CIRCLE,
            rollDegrees = Math.toDegrees(atan2(rightAcceleration, sqrt(gravity * gravity + forwardAcceleration * forwardAcceleration))),
            pitchDegrees = -Math.toDegrees(atan2(forwardAcceleration, gravity)),
            yawHeadingDegrees = if (mode == TiltFlightMode.SKATING_CIRCLE) initialHeadingDegrees
                else wrapToSignedHeading(initialHeadingDegrees - progressDegrees),
            lap = (circleElapsedNanos / LAP_DURATION_NANOS).toInt() + 1,
            progressDegrees = progressDegrees,
            phaseEndsAtNanos = startedAtNanos + activeEndsAt,
        )
    }

    private fun zeroCommand(phase: TiltFlightPhase, phaseEndElapsedNanos: Long): TiltFlightCommand {
        val finishedCircle = mode != TiltFlightMode.STRAIGHT && phase != TiltFlightPhase.SETTLE
        return TiltFlightCommand(
            phase = phase,
            rollDegrees = 0.0,
            pitchDegrees = 0.0,
            yawHeadingDegrees = initialHeadingDegrees,
            lap = if (finishedCircle) LAP_COUNT else 0,
            progressDegrees = if (finishedCircle) LAP_COUNT * 360.0 else 0.0,
            phaseEndsAtNanos = startedAtNanos + phaseEndElapsedNanos,
        )
    }

    companion object {
        const val DIAMETER_METERS = 1.5
        const val SECONDS_PER_LAP = 7.0
        const val LAP_COUNT = 3
        const val STRAIGHT_TILT_DEGREES = 3.0
        const val STRAIGHT_DURATION_NANOS = 2_000_000_000L
        const val SETTLE_DURATION_NANOS = 2_000_000_000L
        const val BRAKE_DURATION_NANOS = 2_000_000_000L
        private const val GRAVITY_METERS_PER_SECOND_SQUARED = 9.80665
        private const val NANOS_PER_SECOND = 1_000_000_000.0
        private const val LAP_DURATION_NANOS = 7_000_000_000L
        private const val CIRCLE_DURATION_NANOS = LAP_DURATION_NANOS * LAP_COUNT
    }
}

internal data class TiltFlightCommand(
    val phase: TiltFlightPhase,
    val rollDegrees: Double,
    val pitchDegrees: Double,
    val yawHeadingDegrees: Double,
    val lap: Int,
    val progressDegrees: Double,
    val phaseEndsAtNanos: Long,
) {
    val usesAngle: Boolean
        get() = phase == TiltFlightPhase.STRAIGHT || phase == TiltFlightPhase.STARTUP || phase == TiltFlightPhase.CIRCLE
    val completed: Boolean
        get() = phase == TiltFlightPhase.COMPLETE
}
