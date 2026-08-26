package com.durendal.droneagent.lite

internal fun boundedControlIntervalSeconds(
    nowNanos: Long,
    previousNanos: Long,
    initialSeconds: Double,
    maximumSeconds: Double,
): Double =
    if (previousNanos == 0L) {
        initialSeconds
    } else {
        ((nowNanos - previousNanos) / 1_000_000_000.0)
            .coerceIn(0.0, maximumSeconds)
    }

internal fun wrapToSignedHeading(degrees: Double): Double {
    require(degrees.isFinite()) { "heading must be finite" }
    var wrapped = degrees % 360.0
    if (wrapped > 180.0) wrapped -= 360.0
    if (wrapped < -180.0) wrapped += 360.0
    return wrapped
}

internal fun headingTargetForYawRate(
    currentHeadingDegrees: Double,
    yawRateDegreesPerSecond: Double,
    maximumYawRateDegreesPerSecond: Double,
    maximumHeadingLeadDegrees: Double,
): Double? {
    if (!currentHeadingDegrees.isFinite() || !yawRateDegreesPerSecond.isFinite()) return null
    require(maximumYawRateDegreesPerSecond > 0.0 && maximumYawRateDegreesPerSecond.isFinite()) {
        "maximum yaw rate must be finite and positive"
    }
    require(maximumHeadingLeadDegrees in 0.0..<180.0) {
        "maximum heading lead must be in [0, 180)"
    }
    val headingLead =
        yawRateDegreesPerSecond
            .coerceIn(-maximumYawRateDegreesPerSecond, maximumYawRateDegreesPerSecond) /
            maximumYawRateDegreesPerSecond *
            maximumHeadingLeadDegrees
    return wrapToSignedHeading(currentHeadingDegrees + headingLead)
}

/** Tracks clockwise yaw progress across the -180/180-degree heading boundary. */
internal class HeadingTurn(
    initialHeadingDegrees: Double,
    val targetDegrees: Double = DEFAULT_TARGET_DEGREES,
) {
    private var directedDisplacementDegrees = 0.0
    private var previousHeadingDegrees = initialHeadingDegrees

    init {
        require(initialHeadingDegrees.isFinite()) { "initial heading must be finite" }
        require(targetDegrees.isFinite() && targetDegrees > 0.0) {
            "target turn must be finite and positive"
        }
    }

    val progressDegrees: Double
        get() = directedDisplacementDegrees.coerceAtLeast(0.0)

    fun update(headingDegrees: Double): Double {
        require(headingDegrees.isFinite()) { "heading must be finite" }
        val signedDelta = shortestAngularDelta(previousHeadingDegrees, headingDegrees)
        previousHeadingDegrees = headingDegrees
        directedDisplacementDegrees += signedDelta
        return (targetDegrees - progressDegrees).coerceAtLeast(0.0)
    }

    companion object {
        const val DEFAULT_TARGET_DEGREES = 180.0

        internal fun shortestAngularDelta(fromDegrees: Double, toDegrees: Double): Double {
            var delta = (toDegrees - fromDegrees + 180.0) % 360.0
            if (delta < 0.0) delta += 360.0
            return delta - 180.0
        }
    }
}

internal data class QuarterArcCommand(
    val forwardSpeedMetersPerSecond: Double,
    val yawRateDegreesPerSecond: Double,
)

/**
 * Commands a clockwise constant-radius quarter arc without using camera data.
 * Heading closes only the 90° endpoint; position remains deliberately open-loop.
 */
internal class QuarterArcController(
    initialHeadingDegrees: Double,
    private val radiusMeters: Double = RADIUS_METERS,
    private val maximumForwardSpeedMetersPerSecond: Double = MAXIMUM_FORWARD_SPEED_METERS_PER_SECOND,
    private val forwardAccelerationMetersPerSecondSquared: Double =
        FORWARD_ACCELERATION_METERS_PER_SECOND_SQUARED,
) {
    private val headingTurn = HeadingTurn(initialHeadingDegrees, TARGET_DEGREES)
    private var appliedForwardSpeedMetersPerSecond = 0.0
    private var lastCommandAtNanos = 0L

    init {
        require(radiusMeters.isFinite() && radiusMeters > 0.0) {
            "arc radius must be finite and positive"
        }
        require(
            maximumForwardSpeedMetersPerSecond.isFinite() &&
                maximumForwardSpeedMetersPerSecond > 0.0,
        ) {
            "maximum forward speed must be finite and positive"
        }
        require(
            forwardAccelerationMetersPerSecondSquared.isFinite() &&
                forwardAccelerationMetersPerSecondSquared > 0.0,
        ) {
            "forward acceleration must be finite and positive"
        }
    }

    val progressDegrees: Double
        get() = headingTurn.progressDegrees

    val remainingDegrees: Double
        get() = (TARGET_DEGREES - progressDegrees).coerceAtLeast(0.0)

    fun updateHeading(headingDegrees: Double) {
        headingTurn.update(headingDegrees)
    }

    fun command(nowNanos: Long): QuarterArcCommand {
        val elapsedSeconds =
            boundedControlIntervalSeconds(
                nowNanos = nowNanos,
                previousNanos = lastCommandAtNanos,
                initialSeconds = INITIAL_COMMAND_INTERVAL_SECONDS,
                maximumSeconds = MAX_COMMAND_INTERVAL_SECONDS,
            )
        lastCommandAtNanos = nowNanos
        appliedForwardSpeedMetersPerSecond =
            (appliedForwardSpeedMetersPerSecond +
                forwardAccelerationMetersPerSecondSquared * elapsedSeconds)
                .coerceAtMost(maximumForwardSpeedMetersPerSecond)
        return QuarterArcCommand(
            forwardSpeedMetersPerSecond = appliedForwardSpeedMetersPerSecond,
            yawRateDegreesPerSecond =
                Math.toDegrees(appliedForwardSpeedMetersPerSecond / radiusMeters),
        )
    }

    companion object {
        const val TARGET_DEGREES = 90.0
        const val RADIUS_METERS = 0.75
        const val MAXIMUM_FORWARD_SPEED_METERS_PER_SECOND = 0.12
        const val FORWARD_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.02
        private const val INITIAL_COMMAND_INTERVAL_SECONDS = 0.05
        private const val MAX_COMMAND_INTERVAL_SECONDS = 0.10
    }
}
