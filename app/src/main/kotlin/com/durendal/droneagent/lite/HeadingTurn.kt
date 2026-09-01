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

/**
 * Converts a camera-relative path bearing into a bounded DJI earth-frame heading setpoint.
 *
 * ANGLE mode is position control: its input must be an absolute heading, never a yaw rate
 * disguised as a heading lead. The bearing comes from the directed look-ahead point, not
 * the undirected near-field tape tangent, so its sign identifies the upcoming turn.
 */
internal class AngleHeadingController(
    private val maximumSetpointRateDegreesPerSecond: Double =
        DEFAULT_MAXIMUM_SETPOINT_RATE_DEGREES_PER_SECOND,
    private val maximumHeadingLeadDegrees: Double = DEFAULT_MAXIMUM_HEADING_LEAD_DEGREES,
) {
    private var commandedHeadingDegrees: Double? = null
    private var lastUpdateAtNanos = 0L

    init {
        require(
            maximumSetpointRateDegreesPerSecond.isFinite() &&
                maximumSetpointRateDegreesPerSecond > 0.0,
        ) {
            "maximum heading setpoint rate must be finite and positive"
        }
        require(maximumHeadingLeadDegrees.isFinite() && maximumHeadingLeadDegrees in 0.0..<180.0) {
            "maximum heading lead must be finite and in [0, 180)"
        }
    }

    fun reset() {
        commandedHeadingDegrees = null
        lastUpdateAtNanos = 0L
    }

    fun update(
        currentHeadingDegrees: Double,
        relativePathBearingDegrees: Double?,
        pathTracking: Boolean,
        nowNanos: Long,
    ): Double? {
        if (
            !currentHeadingDegrees.isFinite() ||
            relativePathBearingDegrees?.isFinite() == false ||
            nowNanos < 0L
        ) {
            return null
        }
        if (!pathTracking || relativePathBearingDegrees == null) {
            val heldHeading = wrapToSignedHeading(currentHeadingDegrees)
            commandedHeadingDegrees = heldHeading
            lastUpdateAtNanos = nowNanos
            return heldHeading
        }

        val desiredHeading =
            wrapToSignedHeading(currentHeadingDegrees + relativePathBearingDegrees)
        val previousHeading = commandedHeadingDegrees ?: currentHeadingDegrees
        val intervalSeconds =
            boundedControlIntervalSeconds(
                nowNanos = nowNanos,
                previousNanos = lastUpdateAtNanos,
                initialSeconds = INITIAL_UPDATE_INTERVAL_SECONDS,
                maximumSeconds = MAXIMUM_UPDATE_INTERVAL_SECONDS,
            )
        val maximumStep = maximumSetpointRateDegreesPerSecond * intervalSeconds
        val slewedHeading =
            wrapToSignedHeading(
                previousHeading +
                    shortestAngularDelta(previousHeading, desiredHeading)
                        .coerceIn(-maximumStep, maximumStep),
            )
        val boundedHeading =
            wrapToSignedHeading(
                currentHeadingDegrees +
                    shortestAngularDelta(currentHeadingDegrees, slewedHeading)
                        .coerceIn(-maximumHeadingLeadDegrees, maximumHeadingLeadDegrees),
            )
        commandedHeadingDegrees = boundedHeading
        lastUpdateAtNanos = nowNanos
        return boundedHeading
    }

    companion object {
        const val DEFAULT_MAXIMUM_SETPOINT_RATE_DEGREES_PER_SECOND = 10.0
        const val DEFAULT_MAXIMUM_HEADING_LEAD_DEGREES = 5.0
        private const val INITIAL_UPDATE_INTERVAL_SECONDS = 0.1
        private const val MAXIMUM_UPDATE_INTERVAL_SECONDS = 0.2
    }
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
