package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.sign

internal enum class TapeTrackingPhase {
    DISABLED,
    RECENTERING,
    TRACKING,
    VERIFYING_ENDPOINT,
    TURNING,
}

internal enum class TapeTrackingMode {
    STRAIGHT,
    CIRCULAR,
}


internal enum class TrackingGimbalTarget {
    DOWN_CENTER,
}

internal data class TapeTrackingObservation(
    val angleFromVerticalDegrees: Double,
    val longSideFraction: Double,
    val nearFieldOffsetFraction: Double,
    val bounds: NormalizedRect,
) {
    init {
        require(angleFromVerticalDegrees in -90.0..90.0)
        require(longSideFraction > 0.0 && longSideFraction.isFinite())
        require(nearFieldOffsetFraction in -0.5..0.5)
    }
}

internal data class TapeTrackingDecision(
    val phase: TapeTrackingPhase,
    val yawRateDegreesPerSecond: Double,
    val forwardSpeedMetersPerSecond: Double = 0.0,
    val rightSpeedMetersPerSecond: Double = 0.0,
    val gimbalTarget: TrackingGimbalTarget? = null,
    val endpointReached: Boolean = false,
    val rawAngleDegrees: Double? = null,
    val controlledAngleDegrees: Double? = null,
    val rawOffsetFraction: Double? = null,
    val controlledOffsetFraction: Double? = null,
    val offsetRatePerSecond: Double = 0.0,
)

/**
 * Pure timing and steering policy for black-tape tracking. Aircraft and gimbal
 * commands remain in MainActivity, so this class can be tested without DJI SDK.
 */
internal class TapeTrackingController {
    var enabled: Boolean = false
        private set

    var phase: TapeTrackingPhase = TapeTrackingPhase.DISABLED
        private set


    private var lastDetectionAtNanos = 0L
    private var lastControlObservationAtNanos = 0L
    private var rawAngleDegrees: Double? = null
    private var rawHorizontalOffsetFraction: Double? = null
    private var controlledAngleDegrees: Double? = null
    private var controlledHorizontalOffsetFraction: Double? = null
    private var offsetRatePerSecond = 0.0
    private var recenterUntilNanos = 0L
    private var consecutiveEndpointDetections = 0
    private var endpointCandidateSinceNanos = 0L
    private var longestObservedTapeFraction = 0.0
    private var endpointQualificationArmed = true
    private var endpointPending = false
    private var pendingGimbalTarget: TrackingGimbalTarget? = null
    private var awaitingPostTurnDetection = false
    private var postTurnRecoveryUntilNanos = 0L
    private var endpointVerificationStartedAtNanos = 0L
    private var consecutiveEndpointMisses = 0
    private var endpointReferenceBounds: NormalizedRect? = null
    private var lateralCorrectionActive = false
    private var appliedYawRateDegreesPerSecond = 0.0
    private var appliedRightSpeedMetersPerSecond = 0.0
    private var lastCommandAtNanos = 0L
    var mode: TapeTrackingMode = TapeTrackingMode.STRAIGHT
        private set
    private var circularTrackCandidateSinceNanos = 0L
    private var consecutiveCircularTrackDetections = 0
    private var circularTrackConfirmed = false
    private var circularReferenceBounds: NormalizedRect? = null


    fun start(nowNanos: Long, mode: TapeTrackingMode = TapeTrackingMode.STRAIGHT) {
        enabled = true
        this.mode = mode
        resetLeg()
        awaitingPostTurnDetection = false
        beginRecentering(nowNanos)
    }

    fun stop() {
        enabled = false
        phase = TapeTrackingPhase.DISABLED
        consecutiveEndpointDetections = 0
        endpointCandidateSinceNanos = 0L
        endpointPending = false
        awaitingPostTurnDetection = false
        postTurnRecoveryUntilNanos = 0L
        endpointVerificationStartedAtNanos = 0L
        endpointReferenceBounds = null
        consecutiveEndpointMisses = 0
        pendingGimbalTarget = null
        resetControlState()
        mode = TapeTrackingMode.STRAIGHT
    }

    fun resumeAfterTurn(nowNanos: Long) {
        check(enabled && mode == TapeTrackingMode.STRAIGHT && phase == TapeTrackingPhase.TURNING)
        resetLeg()
        awaitingPostTurnDetection = true
        postTurnRecoveryUntilNanos =
            nowNanos + RECENTER_DURATION_NANOS + POST_TURN_RECOVERY_NANOS
        beginRecentering(nowNanos)
    }

    fun observe(observation: TapeTrackingObservation?, nowNanos: Long) {
        if (!enabled || phase == TapeTrackingPhase.TURNING) return
        if (mode == TapeTrackingMode.CIRCULAR) {
            observeCircularPath(observation, nowNanos)
            return
        }
        if (observation == null) {
            clearControlMeasurements()
            if (phase == TapeTrackingPhase.TRACKING) {
                // A real endpoint shortens continuously. After a detection gap, a short contour
                // may be only one glare-separated tape segment; require a long reacquisition.
                endpointQualificationArmed = false
                endpointCandidateSinceNanos = 0L
                consecutiveEndpointDetections = 0
                endpointReferenceBounds = null
            } else if (phase == TapeTrackingPhase.VERIFYING_ENDPOINT) {
                registerEndpointMiss(nowNanos)
            }
            return
        }
        when (phase) {
            TapeTrackingPhase.TRACKING -> {
                updateControlMeasurements(
                    observation.angleFromVerticalDegrees,
                    observation.nearFieldOffsetFraction,
                    nowNanos,
                )
                lastDetectionAtNanos = nowNanos
                awaitingPostTurnDetection = false
                longestObservedTapeFraction =
                    maxOf(longestObservedTapeFraction, observation.longSideFraction)
                if (
                    !endpointQualificationArmed &&
                    observation.longSideFraction >= ENDPOINT_REFERENCE_MIN_FRACTION &&
                    observation.longSideFraction >=
                    longestObservedTapeFraction * ENDPOINT_REARM_LENGTH_RATIO
                ) {
                    endpointQualificationArmed = true
                }
                if (
                    !endpointQualificationArmed &&
                    abs(observation.angleFromVerticalDegrees) > ALIGNMENT_TOLERANCE_DEGREES
                ) {
                    clearControlMeasurements()
                    endpointCandidateSinceNanos = 0L
                    consecutiveEndpointDetections = 0
                    endpointReferenceBounds = null
                    return
                }
                val endpointCandidate =
                    endpointQualificationArmed &&
                        abs(observation.angleFromVerticalDegrees) <= ALIGNMENT_TOLERANCE_DEGREES &&
                        longestObservedTapeFraction >= ENDPOINT_REFERENCE_MIN_FRACTION &&
                        observation.longSideFraction <=
                        longestObservedTapeFraction * ENDPOINT_LENGTH_RATIO
                if (endpointCandidate) {
                    if (endpointCandidateSinceNanos == 0L) {
                        endpointCandidateSinceNanos = nowNanos
                    }
                    consecutiveEndpointDetections += 1
                    endpointReferenceBounds = observation.bounds
                } else {
                    endpointCandidateSinceNanos = 0L
                    consecutiveEndpointDetections = 0
                    endpointReferenceBounds = null
                }
                beginEndpointVerificationIfReady(nowNanos)
            }

            TapeTrackingPhase.VERIFYING_ENDPOINT -> {
                val referenceBounds = endpointReferenceBounds
                val matchesTrackedTape =
                    referenceBounds != null &&
                        abs(observation.angleFromVerticalDegrees) <=
                        ALIGNMENT_TOLERANCE_DEGREES &&
                        observation.bounds.bottom >= ENDPOINT_NEAR_EDGE_MIN_FRACTION &&
                        overlapOfSmallerArea(observation.bounds, referenceBounds) >=
                        ENDPOINT_TRACK_MIN_OVERLAP
                if (!matchesTrackedTape) {
                    clearControlMeasurements()
                    registerEndpointMiss(nowNanos)
                    return
                }
                endpointReferenceBounds = observation.bounds
                consecutiveEndpointMisses = 0
                updateControlMeasurements(
                    observation.angleFromVerticalDegrees,
                    observation.nearFieldOffsetFraction,
                    nowNanos,
                )
                lastDetectionAtNanos = nowNanos
                if (
                    observation.longSideFraction >=
                    longestObservedTapeFraction * ENDPOINT_EXIT_LENGTH_RATIO
                ) {
                    phase = TapeTrackingPhase.TRACKING
                    endpointVerificationStartedAtNanos = 0L
                    endpointCandidateSinceNanos = 0L
                    consecutiveEndpointDetections = 0
                    endpointReferenceBounds = null
                }
            }

            TapeTrackingPhase.DISABLED,
            TapeTrackingPhase.RECENTERING,
            TapeTrackingPhase.TURNING,
            -> Unit
        }
    }

    fun tick(nowNanos: Long): TapeTrackingDecision {
        if (!enabled) return decision(TapeTrackingPhase.DISABLED, 0.0)
        if (mode == TapeTrackingMode.STRAIGHT) beginEndpointVerificationIfReady(nowNanos)
        if (endpointPending) {
            endpointPending = false
            resetAppliedCommands()
            return decision(
                phase = TapeTrackingPhase.TURNING,
                yawRateDegreesPerSecond = 0.0,
                endpointReached = true,
            )
        }
        if (phase == TapeTrackingPhase.TURNING) {
            resetAppliedCommands()
            return decision(TapeTrackingPhase.TURNING, 0.0)
        }

        if (phase == TapeTrackingPhase.RECENTERING && nowNanos >= recenterUntilNanos) {
            phase = TapeTrackingPhase.TRACKING
            lastDetectionAtNanos = nowNanos
            clearControlMeasurements()
        }

        val target = pendingGimbalTarget
        pendingGimbalTarget = null
        if (
            (phase != TapeTrackingPhase.TRACKING &&
                phase != TapeTrackingPhase.VERIFYING_ENDPOINT) ||
            nowNanos - lastDetectionAtNanos > DETECTION_COMMAND_STALE_NANOS ||
            controlledAngleDegrees == null ||
            controlledHorizontalOffsetFraction == null
        ) {
            resetAppliedCommands()
            return decision(
                phase = phase,
                yawRateDegreesPerSecond = 0.0,
                forwardSpeedMetersPerSecond = desiredForwardSpeed(nowNanos),
                gimbalTarget = target,
            )
        }

        val targetYawRate = desiredYawRate()
        val targetRightSpeed = desiredRightSpeed()
        val (yawRate, rightSpeed) = applyOutputLimits(
            targetYawRate,
            targetRightSpeed,
            nowNanos,
        )
        return decision(
            phase = phase,
            yawRateDegreesPerSecond = yawRate,
            forwardSpeedMetersPerSecond = desiredForwardSpeed(nowNanos),
            rightSpeedMetersPerSecond = rightSpeed,
            gimbalTarget = target,
        )
    }

    private fun observeCircularPath(
        observation: TapeTrackingObservation?,
        nowNanos: Long,
    ) {
        if (observation == null) {
            clearControlMeasurements()
            if (phase == TapeTrackingPhase.TRACKING && circularTrackConfirmed) {
                phase = TapeTrackingPhase.VERIFYING_ENDPOINT
                endpointVerificationStartedAtNanos = nowNanos
                consecutiveEndpointMisses = 1
                resetAppliedCommands()
            } else if (phase == TapeTrackingPhase.VERIFYING_ENDPOINT) {
                registerCircularEndpointMiss(nowNanos)
            }
            return
        }

        if (phase == TapeTrackingPhase.TRACKING) {
            updateControlMeasurements(
                observation.angleFromVerticalDegrees,
                observation.nearFieldOffsetFraction,
                nowNanos,
            )
            lastDetectionAtNanos = nowNanos
            circularReferenceBounds = observation.bounds
            if (observation.longSideFraction >= CIRCULAR_MIN_TRACK_FRACTION) {
                if (circularTrackCandidateSinceNanos == 0L) {
                    circularTrackCandidateSinceNanos = nowNanos
                }
                consecutiveCircularTrackDetections += 1
                if (
                    consecutiveCircularTrackDetections >= CIRCULAR_TRACK_CONFIRMATION_COUNT &&
                    nowNanos - circularTrackCandidateSinceNanos >=
                    CIRCULAR_TRACK_CONFIRMATION_NANOS
                ) {
                    circularTrackConfirmed = true
                }
            } else {
                circularTrackCandidateSinceNanos = 0L
                consecutiveCircularTrackDetections = 0
            }
            return
        }

        if (phase != TapeTrackingPhase.VERIFYING_ENDPOINT) return
        val reference = circularReferenceBounds
        val matchesTrackedPath =
            reference != null &&
                overlapOfSmallerArea(observation.bounds, reference) >=
                CIRCULAR_REACQUISITION_MIN_OVERLAP
        if (!matchesTrackedPath) {
            clearControlMeasurements()
            registerCircularEndpointMiss(nowNanos)
            return
        }
        phase = TapeTrackingPhase.TRACKING
        endpointVerificationStartedAtNanos = 0L
        consecutiveEndpointMisses = 0
        circularReferenceBounds = observation.bounds
        updateControlMeasurements(
            observation.angleFromVerticalDegrees,
            observation.nearFieldOffsetFraction,
            nowNanos,
        )
        lastDetectionAtNanos = nowNanos
        resetAppliedCommands()
    }

    private fun registerCircularEndpointMiss(nowNanos: Long) {
        consecutiveEndpointMisses += 1
        if (
            consecutiveEndpointMisses >= CIRCULAR_ENDPOINT_MISS_COUNT &&
            nowNanos - endpointVerificationStartedAtNanos >=
            CIRCULAR_ENDPOINT_CONFIRMATION_NANOS
        ) {
            phase = TapeTrackingPhase.TURNING
            endpointPending = true
        }
    }

    private fun beginEndpointVerificationIfReady(nowNanos: Long) {
        if (
            phase != TapeTrackingPhase.TRACKING ||
            endpointCandidateSinceNanos == 0L ||
            consecutiveEndpointDetections < ENDPOINT_CONFIRMATION_COUNT ||
            nowNanos - endpointCandidateSinceNanos < ENDPOINT_CONFIRMATION_NANOS
        ) {
            return
        }
        phase = TapeTrackingPhase.VERIFYING_ENDPOINT
        endpointReferenceBounds = checkNotNull(endpointReferenceBounds)
        endpointCandidateSinceNanos = 0L
        consecutiveEndpointDetections = 0
        endpointVerificationStartedAtNanos = nowNanos
        consecutiveEndpointMisses = 0
        resetAppliedCommands()
    }

    private fun registerEndpointMiss(nowNanos: Long) {
        consecutiveEndpointMisses += 1
        if (
            consecutiveEndpointMisses >= ENDPOINT_DISAPPEARANCE_COUNT &&
            nowNanos - endpointVerificationStartedAtNanos >= ENDPOINT_PROBE_DURATION_NANOS
        ) {
            phase = TapeTrackingPhase.TURNING
            endpointPending = true
        }
    }

    private fun beginRecentering(nowNanos: Long) {
        phase = TapeTrackingPhase.RECENTERING
        clearControlMeasurements()
        pendingGimbalTarget = TrackingGimbalTarget.DOWN_CENTER
        recenterUntilNanos = nowNanos + RECENTER_DURATION_NANOS
        resetAppliedCommands()
    }

    private fun resetLeg() {
        lastDetectionAtNanos = 0L
        consecutiveEndpointDetections = 0
        endpointCandidateSinceNanos = 0L
        longestObservedTapeFraction = 0.0
        endpointQualificationArmed = true
        endpointPending = false
        endpointVerificationStartedAtNanos = 0L
        consecutiveEndpointMisses = 0
        endpointReferenceBounds = null
        circularTrackCandidateSinceNanos = 0L
        consecutiveCircularTrackDetections = 0
        circularTrackConfirmed = false
        circularReferenceBounds = null
        lateralCorrectionActive = false
        resetControlState()
    }

    private fun updateControlMeasurements(
        angleDegrees: Double,
        horizontalOffsetFraction: Double,
        nowNanos: Long,
    ) {
        rawAngleDegrees = angleDegrees
        rawHorizontalOffsetFraction = horizontalOffsetFraction
        val previousOffset = controlledHorizontalOffsetFraction
        val nextAngle = controlledAngleDegrees?.let {
            exponentialAverage(it, angleDegrees, STABILIZED_FILTER_ALPHA)
        } ?: angleDegrees
        val nextOffset = previousOffset?.let {
            exponentialAverage(it, horizontalOffsetFraction, STABILIZED_FILTER_ALPHA)
        } ?: horizontalOffsetFraction
        val elapsedNanos = nowNanos - lastControlObservationAtNanos
        offsetRatePerSecond =
            if (previousOffset != null && lastControlObservationAtNanos != 0L && elapsedNanos > 0L) {
                ((nextOffset - previousOffset) / (elapsedNanos / NANOS_PER_SECOND))
                    .coerceIn(-MAX_OFFSET_RATE_PER_SECOND, MAX_OFFSET_RATE_PER_SECOND)
            } else {
                0.0
            }
        controlledAngleDegrees = nextAngle
        controlledHorizontalOffsetFraction = nextOffset
        lastControlObservationAtNanos = nowNanos
    }

    private fun clearControlMeasurements() {
        rawAngleDegrees = null
        rawHorizontalOffsetFraction = null
        controlledAngleDegrees = null
        controlledHorizontalOffsetFraction = null
        offsetRatePerSecond = 0.0
        lastControlObservationAtNanos = 0L
        lateralCorrectionActive = false
    }

    private fun resetControlState() {
        clearControlMeasurements()
        resetAppliedCommands()
    }

    private fun resetAppliedCommands() {
        appliedYawRateDegreesPerSecond = 0.0
        appliedRightSpeedMetersPerSecond = 0.0
        lastCommandAtNanos = 0L
    }

    private fun desiredYawRate(): Double {
        val angle = controlledAngleDegrees ?: return 0.0
        val deadZone =
            if (mode == TapeTrackingMode.CIRCULAR) CIRCULAR_YAW_DEAD_ZONE_DEGREES
            else YAW_DEAD_ZONE_DEGREES
        if (abs(angle) <= deadZone) return 0.0
        val maximumYawRate = when {
            mode == TapeTrackingMode.CIRCULAR ->
                CIRCULAR_MAX_YAW_RATE_DEGREES_PER_SECOND
            phase == TapeTrackingPhase.VERIFYING_ENDPOINT ->
                ENDPOINT_MAX_YAW_RATE_DEGREES_PER_SECOND
            else -> MAX_TRACKING_YAW_RATE_DEGREES_PER_SECOND
        }
        val gain =
            if (mode == TapeTrackingMode.CIRCULAR) CIRCULAR_YAW_PROPORTIONAL_GAIN
            else YAW_PROPORTIONAL_GAIN
        return (angle * gain).coerceIn(-maximumYawRate, maximumYawRate)
    }

    private fun desiredRightSpeed(): Double {
        val angle = controlledAngleDegrees ?: return 0.0
        val offset = controlledHorizontalOffsetFraction ?: return 0.0
        lateralCorrectionActive = when {
            lateralCorrectionActive && abs(offset) <= CENTERING_STOP_FRACTION -> false
            !lateralCorrectionActive && abs(offset) >= CENTERING_START_FRACTION -> true
            else -> lateralCorrectionActive
        }
        if (!lateralCorrectionActive || abs(angle) >= MAX_LATERAL_CORRECTION_ANGLE_DEGREES) {
            return 0.0
        }
        val angleScale =
            (1.0 - abs(angle) / MAX_LATERAL_CORRECTION_ANGLE_DEGREES).coerceIn(0.0, 1.0)
        val maximumCenteringSpeed =
            if (mode == TapeTrackingMode.CIRCULAR) {
                CIRCULAR_MAX_CENTERING_SPEED_METERS_PER_SECOND
            } else {
                MAX_CENTERING_SPEED_METERS_PER_SECOND
            }
        val correction =
            (
                offset * LATERAL_PROPORTIONAL_GAIN +
                    offsetRatePerSecond * LATERAL_DERIVATIVE_GAIN
                )
                .coerceIn(-maximumCenteringSpeed, maximumCenteringSpeed) * angleScale
        return if (phase == TapeTrackingPhase.VERIFYING_ENDPOINT) {
            correction.coerceIn(
                -ENDPOINT_MAX_CENTERING_SPEED_METERS_PER_SECOND,
                ENDPOINT_MAX_CENTERING_SPEED_METERS_PER_SECOND,
            )
        } else {
            correction
        }
    }

    private fun desiredForwardSpeed(nowNanos: Long): Double {
        if (mode == TapeTrackingMode.CIRCULAR) {
            if (phase != TapeTrackingPhase.TRACKING) return 0.0
            val angle = controlledAngleDegrees ?: return 0.0
            val offset = controlledHorizontalOffsetFraction ?: return 0.0
            if (
                abs(angle) > CIRCULAR_MAX_MOVING_ANGLE_DEGREES ||
                abs(offset) > CIRCULAR_MAX_MOVING_OFFSET_FRACTION
            ) {
                return 0.0
            }
            return if (
                abs(angle) <= CIRCULAR_STABLE_ANGLE_DEGREES &&
                abs(offset) <= CENTERING_START_FRACTION
            ) {
                CIRCULAR_TRACKING_FORWARD_SPEED_METERS_PER_SECOND
            } else {
                CIRCULAR_CORRECTION_FORWARD_SPEED_METERS_PER_SECOND
            }
        }
        if (phase == TapeTrackingPhase.VERIFYING_ENDPOINT) {
            return if (
                nowNanos - endpointVerificationStartedAtNanos <= ENDPOINT_PROBE_DURATION_NANOS
            ) {
                ENDPOINT_PROBE_SPEED_METERS_PER_SECOND
            } else {
                0.0
            }
        }
        if (awaitingPostTurnDetection && phase == TapeTrackingPhase.TRACKING) {
            return if (nowNanos <= postTurnRecoveryUntilNanos) {
                POST_TURN_RECOVERY_SPEED_METERS_PER_SECOND
            } else {
                0.0
            }
        }
        if (phase != TapeTrackingPhase.TRACKING || consecutiveEndpointDetections > 0) return 0.0
        val angle = controlledAngleDegrees ?: return 0.0
        val offset = controlledHorizontalOffsetFraction ?: return 0.0
        if (
            abs(angle) > ALIGNMENT_TOLERANCE_DEGREES ||
            abs(offset) > CENTERING_START_FRACTION
        ) {
            return 0.0
        }
        if (!endpointQualificationArmed) {
            return STABILIZED_CORRECTION_FORWARD_SPEED_METERS_PER_SECOND
        }
        return if (
            abs(angle) <= STABLE_ANGLE_DEGREES &&
            abs(offset) <= CENTERING_STOP_FRACTION &&
            !lateralCorrectionActive
        ) {
            TRACKING_FORWARD_SPEED_METERS_PER_SECOND
        } else {
            STABILIZED_CORRECTION_FORWARD_SPEED_METERS_PER_SECOND
        }
    }

    private fun applyOutputLimits(
        targetYawRate: Double,
        targetRightSpeed: Double,
        nowNanos: Long,
    ): Pair<Double, Double> {
        val elapsedSeconds =
            if (lastCommandAtNanos == 0L) {
                INITIAL_COMMAND_INTERVAL_SECONDS
            } else {
                ((nowNanos - lastCommandAtNanos) / NANOS_PER_SECOND)
                    .coerceIn(0.0, MAX_COMMAND_INTERVAL_SECONDS)
            }
        appliedYawRateDegreesPerSecond = moveToward(
            appliedYawRateDegreesPerSecond,
            targetYawRate,
            MAX_YAW_ACCELERATION_DEGREES_PER_SECOND_SQUARED * elapsedSeconds,
        )
        appliedRightSpeedMetersPerSecond = moveToward(
            appliedRightSpeedMetersPerSecond,
            targetRightSpeed,
            MAX_LATERAL_ACCELERATION_METERS_PER_SECOND_SQUARED * elapsedSeconds,
        )
        lastCommandAtNanos = nowNanos
        return appliedYawRateDegreesPerSecond to appliedRightSpeedMetersPerSecond
    }

    private fun decision(
        phase: TapeTrackingPhase,
        yawRateDegreesPerSecond: Double,
        forwardSpeedMetersPerSecond: Double = 0.0,
        rightSpeedMetersPerSecond: Double = 0.0,
        gimbalTarget: TrackingGimbalTarget? = null,
        endpointReached: Boolean = false,
    ) = TapeTrackingDecision(
        phase = phase,
        yawRateDegreesPerSecond = yawRateDegreesPerSecond,
        forwardSpeedMetersPerSecond = forwardSpeedMetersPerSecond,
        rightSpeedMetersPerSecond = rightSpeedMetersPerSecond,
        gimbalTarget = gimbalTarget,
        endpointReached = endpointReached,
        rawAngleDegrees = rawAngleDegrees,
        controlledAngleDegrees = controlledAngleDegrees,
        rawOffsetFraction = rawHorizontalOffsetFraction,
        controlledOffsetFraction = controlledHorizontalOffsetFraction,
        offsetRatePerSecond = offsetRatePerSecond,
    )

    private fun exponentialAverage(previous: Double, sample: Double, alpha: Double): Double =
        previous + alpha * (sample - previous)

    private fun moveToward(current: Double, target: Double, maximumDelta: Double): Double =
        when {
            target > current -> minOf(target, current + maximumDelta)
            target < current -> maxOf(target, current - maximumDelta)
            else -> current
        }

    private fun overlapOfSmallerArea(first: NormalizedRect, second: NormalizedRect): Double {
        val left = maxOf(first.left, second.left)
        val top = maxOf(first.top, second.top)
        val right = minOf(first.right, second.right)
        val bottom = minOf(first.bottom, second.bottom)
        if (left >= right || top >= bottom) return 0.0
        val intersection = (right - left) * (bottom - top)
        val firstArea = (first.right - first.left) * (first.bottom - first.top)
        val secondArea = (second.right - second.left) * (second.bottom - second.top)
        return intersection / minOf(firstArea, secondArea)
    }

    internal companion object {
        const val ALIGNMENT_TOLERANCE_DEGREES = 10.0
        const val YAW_DEAD_ZONE_DEGREES = 2.0
        const val YAW_PROPORTIONAL_GAIN = 0.3
        const val MAX_TRACKING_YAW_RATE_DEGREES_PER_SECOND = 5.0
        const val TRACKING_FORWARD_SPEED_METERS_PER_SECOND = 0.15
        const val STABILIZED_CORRECTION_FORWARD_SPEED_METERS_PER_SECOND = 0.10
        const val CENTERING_STOP_FRACTION = 0.04
        const val CENTERING_START_FRACTION = 0.08
        const val LATERAL_PROPORTIONAL_GAIN = 0.35
        const val LATERAL_DERIVATIVE_GAIN = 0.10
        const val MAX_CENTERING_SPEED_METERS_PER_SECOND = 0.10
        const val MAX_LATERAL_CORRECTION_ANGLE_DEGREES = 45.0
        const val STABLE_ANGLE_DEGREES = 5.0
        const val STABILIZED_FILTER_ALPHA = 0.4
        const val MAX_OFFSET_RATE_PER_SECOND = 1.0
        const val MAX_YAW_ACCELERATION_DEGREES_PER_SECOND_SQUARED = 10.0
        const val MAX_LATERAL_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.20
        const val ENDPOINT_REFERENCE_MIN_FRACTION = 0.60
        const val ENDPOINT_LENGTH_RATIO = 0.75
        const val ENDPOINT_REARM_LENGTH_RATIO = 0.65
        const val ENDPOINT_EXIT_LENGTH_RATIO = 0.85
        const val ENDPOINT_CONFIRMATION_COUNT = 3
        const val ENDPOINT_CONFIRMATION_NANOS = 750_000_000L
        const val ENDPOINT_DISAPPEARANCE_COUNT = 3
        const val ENDPOINT_PROBE_SPEED_METERS_PER_SECOND = 0.10
        const val ENDPOINT_PROBE_DURATION_NANOS = 1_500_000_000L
        const val RECENTER_DURATION_NANOS = 2_000_000_000L
        const val POST_TURN_RECOVERY_SPEED_METERS_PER_SECOND = 0.15
        const val POST_TURN_RECOVERY_NANOS = 2_000_000_000L
        const val DETECTION_COMMAND_STALE_NANOS = 1_000_000_000L
        const val ENDPOINT_NEAR_EDGE_MIN_FRACTION = 0.90
        const val ENDPOINT_TRACK_MIN_OVERLAP = 0.20
        const val ENDPOINT_MAX_YAW_RATE_DEGREES_PER_SECOND = 2.0
        const val ENDPOINT_MAX_CENTERING_SPEED_METERS_PER_SECOND = 0.03
        const val CIRCULAR_MIN_TRACK_FRACTION = 0.20
        const val CIRCULAR_TRACK_CONFIRMATION_COUNT = 4
        const val CIRCULAR_TRACK_CONFIRMATION_NANOS = 750_000_000L
        const val CIRCULAR_ENDPOINT_MISS_COUNT = 4
        const val CIRCULAR_ENDPOINT_CONFIRMATION_NANOS = 750_000_000L
        const val CIRCULAR_REACQUISITION_MIN_OVERLAP = 0.10
        const val CIRCULAR_YAW_DEAD_ZONE_DEGREES = 1.5
        const val CIRCULAR_YAW_PROPORTIONAL_GAIN = 0.70
        const val CIRCULAR_MAX_YAW_RATE_DEGREES_PER_SECOND = 6.0
        const val CIRCULAR_TRACKING_FORWARD_SPEED_METERS_PER_SECOND = 0.05
        const val CIRCULAR_CORRECTION_FORWARD_SPEED_METERS_PER_SECOND = 0.03
        const val CIRCULAR_MAX_CENTERING_SPEED_METERS_PER_SECOND = 0.05
        const val CIRCULAR_STABLE_ANGLE_DEGREES = 8.0
        const val CIRCULAR_MAX_MOVING_ANGLE_DEGREES = 45.0
        const val CIRCULAR_MAX_MOVING_OFFSET_FRACTION = 0.25
        private const val NANOS_PER_SECOND = 1_000_000_000.0
        private const val INITIAL_COMMAND_INTERVAL_SECONDS = 0.1
        private const val MAX_COMMAND_INTERVAL_SECONDS = 0.25
    }
}
