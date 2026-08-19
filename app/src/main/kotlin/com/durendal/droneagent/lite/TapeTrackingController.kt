package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.sqrt
import kotlin.math.tan
import kotlin.math.sign

internal enum class TapeTrackingPhase {
    DISABLED,
    RECENTERING,
    RECOVERING_AFTER_TURN,
    TRACKING,
    ALIGNING_CURVE,
    REACQUIRING_PATH,
    VERIFYING_ENDPOINT,
    TURNING,
}

internal enum class TapeTrackingMode {
    STRAIGHT,
    CIRCULAR,
}



internal data class TapeTrackingObservation(
    val angleFromVerticalDegrees: Double,
    val longSideFraction: Double,
    val nearFieldOffsetFraction: Double,
    val bounds: NormalizedRect,
    val lookaheadXFraction: Double,
    val lookaheadYFraction: Double,
    val frameWidthPixels: Int,
    val frameHeightPixels: Int,
    val heightAboveGroundMeters: Double?,
) {
    init {
        require(angleFromVerticalDegrees in -90.0..90.0)
        require(longSideFraction > 0.0 && longSideFraction.isFinite())
        require(nearFieldOffsetFraction in -0.5..0.5)
        require(lookaheadXFraction in 0.0..1.0)
        require(lookaheadYFraction in 0.0..1.0)
        require(frameWidthPixels > 0 && frameHeightPixels > 0)
        require(heightAboveGroundMeters == null || (
            heightAboveGroundMeters.isFinite() && heightAboveGroundMeters > 0.0
        ))
    }
}

internal data class TapeTrackingDecision(
    val phase: TapeTrackingPhase,
    val yawRateDegreesPerSecond: Double,
    val forwardSpeedMetersPerSecond: Double = 0.0,
    val rightSpeedMetersPerSecond: Double = 0.0,
    val endpointReached: Boolean = false,
    val rawAngleDegrees: Double? = null,
    val controlledAngleDegrees: Double? = null,
    val rawOffsetFraction: Double? = null,
    val controlledOffsetFraction: Double? = null,
    val offsetRatePerSecond: Double = 0.0,
    val purePursuitYawRateDegreesPerSecond: Double = 0.0,
)

/**
 * Timing and steering policy for black-tape tracking: it owns the tracking phase,
 * the smoothed measurements and the output slew limits, and answers every tick with
 * a [TapeTrackingDecision]. Aircraft commands remain in MainActivity, so this state machine
 * can be tested without the DJI SDK.
 */
internal class TapeTrackingController {
    var enabled: Boolean = false
        private set

    private var phase: TapeTrackingPhase = TapeTrackingPhase.DISABLED

    private var lastDetectionAtNanos = 0L
    private var lastControlObservationAtNanos = 0L
    private var rawAngleDegrees: Double? = null
    private var rawHorizontalOffsetFraction: Double? = null
    private var controlledAngleDegrees: Double? = null
    private var controlledHorizontalOffsetFraction: Double? = null
    private var controlledLookaheadXFraction: Double? = null
    private var controlledLookaheadYFraction: Double? = null
    private var lookaheadFrameAspectRatio: Double? = null
    private var heightAboveGroundMeters: Double? = null
    private var offsetRatePerSecond = 0.0
    private var recenterUntilNanos = 0L
    private var consecutiveEndpointDetections = 0
    private var endpointCandidateSinceNanos = 0L
    private var longestObservedTapeFraction = 0.0
    private var endpointQualificationArmed = true
    private var endpointPending = false
    private var awaitingPostTurnDetection = false
    private var postTurnRecoveryUntilNanos = 0L
    private var endpointVerificationStartedAtNanos = 0L
    private var consecutiveEndpointMisses = 0
    private var endpointReferenceBounds: NormalizedRect? = null
    private var lateralCorrectionActive = false
    private var consecutiveCurveAlignmentDetections = 0
    private var lastAcceptedCircularObservation: TapeTrackingObservation? = null
    private var circularDetectionGap = false
    private var circularReacquisitionCandidate: TapeTrackingObservation? = null
    private var consecutiveCircularReacquisitionDetections = 0
    private var consecutiveCircularEndpointReadyDetections = 0
    private var circularEndpointQualified = false
    private var appliedYawRateDegreesPerSecond = 0.0
    private var appliedRightSpeedMetersPerSecond = 0.0
    private var lastCommandAtNanos = 0L
    private var mode: TapeTrackingMode = TapeTrackingMode.STRAIGHT

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
        resetControlState()
        mode = TapeTrackingMode.STRAIGHT
    }

    fun resumeAfterTurn(nowNanos: Long) {
        check(enabled && phase == TapeTrackingPhase.TURNING)
        resetLeg()
        awaitingPostTurnDetection = true
        postTurnRecoveryUntilNanos =
            if (mode == TapeTrackingMode.STRAIGHT) {
                nowNanos + RECENTER_DURATION_NANOS + POST_TURN_RECOVERY_NANOS
            } else {
                0L
            }
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
                updateControlMeasurements(observation, nowNanos)
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
                updateControlMeasurements(observation, nowNanos)
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
            TapeTrackingPhase.RECOVERING_AFTER_TURN,
            TapeTrackingPhase.ALIGNING_CURVE,
            TapeTrackingPhase.REACQUIRING_PATH,
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
        if (
            mode == TapeTrackingMode.CIRCULAR &&
            phase == TapeTrackingPhase.VERIFYING_ENDPOINT &&
            endpointVerificationStartedAtNanos != 0L &&
            nowNanos - endpointVerificationStartedAtNanos >=
            CIRCULAR_ENDPOINT_VERIFICATION_TIMEOUT_NANOS
        ) {
            beginCircularReacquisition()
        }

        if (phase == TapeTrackingPhase.RECENTERING && nowNanos >= recenterUntilNanos) {
            phase =
                if (mode == TapeTrackingMode.CIRCULAR && awaitingPostTurnDetection) {
                    postTurnRecoveryUntilNanos =
                        nowNanos + CIRCULAR_POST_TURN_RECOVERY_NANOS
                    TapeTrackingPhase.RECOVERING_AFTER_TURN
                } else {
                    TapeTrackingPhase.TRACKING
                }
            lastDetectionAtNanos = nowNanos
            clearControlMeasurements()
        }
        if (
            phase == TapeTrackingPhase.RECOVERING_AFTER_TURN &&
            nowNanos >= postTurnRecoveryUntilNanos
        ) {
            awaitingPostTurnDetection = false
            beginCircularReacquisition()
        }

        if (
            (phase != TapeTrackingPhase.TRACKING &&
                phase != TapeTrackingPhase.ALIGNING_CURVE &&
                phase != TapeTrackingPhase.RECOVERING_AFTER_TURN &&
                phase != TapeTrackingPhase.VERIFYING_ENDPOINT) ||
            nowNanos - lastDetectionAtNanos > DETECTION_COMMAND_STALE_NANOS ||
            controlledAngleDegrees == null ||
            controlledHorizontalOffsetFraction == null
        ) {
            resetAppliedCommands()
            return decision(
                phase = phase,
                yawRateDegreesPerSecond = 0.0,
                forwardSpeedMetersPerSecond =
                    if (
                        phase == TapeTrackingPhase.VERIFYING_ENDPOINT ||
                        phase == TapeTrackingPhase.RECOVERING_AFTER_TURN
                    ) {
                        desiredForwardSpeed(nowNanos)
                    } else if (mode == TapeTrackingMode.CIRCULAR) {
                        0.0
                    } else {
                        desiredForwardSpeed(nowNanos)
                    },
            )
        }
        if (phase == TapeTrackingPhase.ALIGNING_CURVE) {
            val targetYawRate = desiredCurveAlignmentYawRate()
            val (yawRate, _) = applyOutputLimits(
                targetYawRate,
                targetRightSpeed = 0.0,
                nowNanos,
            )
            return decision(
                phase = phase,
                yawRateDegreesPerSecond = yawRate,
            )
        }


        // Establish the anchor correction state before yaw selection so a
        // displaced rail translates first instead of spending the frame spinning.
        val targetRightSpeed = desiredRightSpeed()
        val targetForwardSpeed = desiredForwardSpeed(nowNanos)
        val purePursuitYawRate = desiredPurePursuitYawRate(targetForwardSpeed)
        val targetYawRate = desiredYawRate(purePursuitYawRate)
        val (yawRate, rightSpeed) = applyOutputLimits(
            targetYawRate,
            targetRightSpeed,
            nowNanos,
        )
        return decision(
            phase = phase,
            yawRateDegreesPerSecond = yawRate,
            forwardSpeedMetersPerSecond = targetForwardSpeed,
            rightSpeedMetersPerSecond = rightSpeed,
            purePursuitYawRateDegreesPerSecond = purePursuitYawRate,
        )
    }

    private fun observeCircularPath(
        observation: TapeTrackingObservation?,
        nowNanos: Long,
    ) {
        if (
            phase != TapeTrackingPhase.TRACKING &&
            phase != TapeTrackingPhase.ALIGNING_CURVE &&
            phase != TapeTrackingPhase.RECOVERING_AFTER_TURN &&
            phase != TapeTrackingPhase.REACQUIRING_PATH &&
            phase != TapeTrackingPhase.VERIFYING_ENDPOINT
        ) {
            return
        }
        if (phase == TapeTrackingPhase.RECOVERING_AFTER_TURN) {
            observeCircularPostTurnRecovery(observation, nowNanos)
            return
        }
        if (phase == TapeTrackingPhase.VERIFYING_ENDPOINT) {
            observeCircularEndpointVerification(observation, nowNanos)
            return
        }
        if (observation == null) {
            circularDetectionGap = true
            resetCircularReacquisitionCandidate()
            when (phase) {
                TapeTrackingPhase.TRACKING -> {
                    if (circularEndpointQualified) {
                        consecutiveEndpointMisses += 1
                        if (
                            consecutiveEndpointMisses >=
                            CIRCULAR_ENDPOINT_ENTRY_MISS_COUNT
                        ) {
                            beginCircularEndpointVerification(nowNanos)
                        }
                    }
                }

                TapeTrackingPhase.ALIGNING_CURVE -> {
                    consecutiveCurveAlignmentDetections = 0
                    clearControlMeasurements()
                }

                TapeTrackingPhase.REACQUIRING_PATH -> {
                    resetCircularReacquisitionCandidate()
                }

                else -> Unit
            }
            return
        }

        if (phase == TapeTrackingPhase.REACQUIRING_PATH) {
            observeCircularReacquisitionCandidate(observation, nowNanos)
            return
        }

        val previousObservation = lastAcceptedCircularObservation
        if (
            previousObservation == null &&
            abs(observation.nearFieldOffsetFraction) >
            REACQUISITION_ENTRY_OFFSET_FRACTION
        ) {
            beginCircularReacquisition()
            observeCircularReacquisitionCandidate(observation, nowNanos)
            return
        }
        if (circularDetectionGap && previousObservation != null) {
            if (!isPlausibleCircularContinuation(observation, previousObservation)) {
                if (
                    circularEndpointQualified &&
                    !isCredibleCircularReacquisitionPath(observation)
                ) {
                    beginCircularEndpointVerification(nowNanos)
                    registerEndpointMiss(nowNanos)
                } else {
                    beginCircularReacquisition()
                    observeCircularReacquisitionCandidate(observation, nowNanos)
                }
            } else {
                observeCircularGapContinuation(observation, nowNanos)
            }
            return
        }

        acceptCircularObservation(observation, nowNanos)
        updateCircularTrackingPhase()
    }

    private fun observeCircularPostTurnRecovery(
        observation: TapeTrackingObservation?,
        nowNanos: Long,
    ) {
        val centeredPath =
            observation != null &&
                observation.longSideFraction >= CIRCULAR_POST_TURN_MIN_PATH_FRACTION &&
                abs(observation.angleFromVerticalDegrees) <=
                CIRCULAR_POST_TURN_MAX_ANGLE_DEGREES &&
                abs(observation.nearFieldOffsetFraction) <=
                CIRCULAR_POST_TURN_MAX_OFFSET_FRACTION
        if (!centeredPath) {
            resetCircularReacquisitionCandidate()
            clearControlMeasurements()
            return
        }

        val trackedObservation = checkNotNull(observation)
        val previousCandidate = circularReacquisitionCandidate
        if (
            previousCandidate != null &&
            isConsistentCircularReacquisition(trackedObservation, previousCandidate)
        ) {
            consecutiveCircularReacquisitionDetections += 1
        } else {
            consecutiveCircularReacquisitionDetections = 1
        }
        circularReacquisitionCandidate = trackedObservation
        if (
            consecutiveCircularReacquisitionDetections <
            CIRCULAR_POST_TURN_CONFIRMATION_COUNT
        ) {
            clearControlMeasurements()
            return
        }

        awaitingPostTurnDetection = false
        phase = TapeTrackingPhase.TRACKING
        resetCircularReacquisitionCandidate()
        acceptCircularObservation(trackedObservation, nowNanos)
        resetAppliedCommands()
        updateCircularTrackingPhase()
    }

    private fun observeCircularEndpointVerification(
        observation: TapeTrackingObservation?,
        nowNanos: Long,
    ) {
        val referenceBounds = endpointReferenceBounds
        val previousObservation = lastAcceptedCircularObservation
        val matchesTrackedPath =
            observation != null &&
                referenceBounds != null &&
                previousObservation != null &&
                isPlausibleCircularContinuation(observation, previousObservation) &&
                overlapOfSmallerArea(observation.bounds, referenceBounds) >=
                ENDPOINT_TRACK_MIN_OVERLAP
        if (!matchesTrackedPath) {
            clearControlMeasurements()
            registerEndpointMiss(nowNanos)
            return
        }

        val trackedObservation = checkNotNull(observation)
        endpointReferenceBounds = trackedObservation.bounds
        consecutiveEndpointMisses = 0
        acceptCircularObservation(trackedObservation, nowNanos)
        if (
            trackedObservation.longSideFraction >=
            CIRCULAR_ENDPOINT_RECOVERY_MIN_FRACTION &&
            trackedObservation.bounds.bottom >= ENDPOINT_NEAR_EDGE_MIN_FRACTION
        ) {
            phase = TapeTrackingPhase.TRACKING
            endpointVerificationStartedAtNanos = 0L
            endpointReferenceBounds = null
            resetAppliedCommands()
            updateCircularTrackingPhase()
        }
    }

    private fun beginCircularEndpointVerification(nowNanos: Long) {
        phase = TapeTrackingPhase.VERIFYING_ENDPOINT
        endpointReferenceBounds = lastAcceptedCircularObservation?.bounds
        endpointVerificationStartedAtNanos = nowNanos
        consecutiveEndpointMisses = 0
        consecutiveCircularEndpointReadyDetections = 0
        circularEndpointQualified = false
        clearControlMeasurements()
        resetAppliedCommands()
    }

    private fun observeCircularGapContinuation(
        observation: TapeTrackingObservation,
        nowNanos: Long,
    ) {
        val previousCandidate = circularReacquisitionCandidate
        if (
            previousCandidate != null &&
            isConsistentCircularReacquisition(observation, previousCandidate)
        ) {
            consecutiveCircularReacquisitionDetections += 1
        } else {
            consecutiveCircularReacquisitionDetections = 1
        }
        circularReacquisitionCandidate = observation
        clearControlMeasurements()
        resetAppliedCommands()
        if (
            consecutiveCircularReacquisitionDetections <
            CIRCULAR_GAP_CONTINUATION_CONFIRMATION_COUNT
        ) {
            return
        }
        resetCircularReacquisitionCandidate()
        acceptCircularObservation(observation, nowNanos)
        updateCircularTrackingPhase()
    }

    private fun observeCircularReacquisitionCandidate(
        observation: TapeTrackingObservation,
        nowNanos: Long,
    ) {
        if (!isCredibleCircularReacquisitionPath(observation)) {
            resetCircularReacquisitionCandidate()
            return
        }

        val previousCandidate = circularReacquisitionCandidate
        if (
            previousCandidate != null &&
            isConsistentCircularReacquisition(observation, previousCandidate)
        ) {
            consecutiveCircularReacquisitionDetections += 1
        } else {
            circularReacquisitionCandidate = observation
            consecutiveCircularReacquisitionDetections = 1
        }
        if (
            consecutiveCircularReacquisitionDetections >=
            REACQUISITION_CONFIRMATION_COUNT
        ) {
            completeCircularReacquisition(observation, nowNanos)
        } else {
            circularReacquisitionCandidate = observation
        }
    }

    private fun beginCircularReacquisition() {
        phase = TapeTrackingPhase.REACQUIRING_PATH
        consecutiveCurveAlignmentDetections = 0
        consecutiveCircularEndpointReadyDetections = 0
        circularEndpointQualified = false
        consecutiveEndpointMisses = 0
        resetCircularReacquisitionCandidate()
        clearControlMeasurements()
        resetAppliedCommands()
    }

    private fun completeCircularReacquisition(
        observation: TapeTrackingObservation,
        nowNanos: Long,
    ) {
        phase = TapeTrackingPhase.TRACKING
        resetCircularReacquisitionCandidate()
        acceptCircularObservation(observation, nowNanos)
        resetAppliedCommands()
        updateCircularTrackingPhase()
    }

    private fun acceptCircularObservation(
        observation: TapeTrackingObservation,
        nowNanos: Long,
    ) {
        updateControlMeasurements(observation, nowNanos)
        lastDetectionAtNanos = nowNanos
        lastAcceptedCircularObservation = observation
        circularDetectionGap = false
        consecutiveEndpointMisses = 0
        val endpointReady =
            observation.longSideFraction >= CIRCULAR_ENDPOINT_READY_MIN_FRACTION &&
                observation.bounds.bottom >= ENDPOINT_NEAR_EDGE_MIN_FRACTION &&
                abs(observation.angleFromVerticalDegrees) <=
                CIRCULAR_ENDPOINT_READY_MAX_ANGLE_DEGREES &&
                abs(observation.nearFieldOffsetFraction) <=
                CIRCULAR_ENDPOINT_READY_MAX_OFFSET_FRACTION
        if (endpointReady) {
            consecutiveCircularEndpointReadyDetections += 1
            if (
                consecutiveCircularEndpointReadyDetections >=
                CIRCULAR_ENDPOINT_READY_CONFIRMATION_COUNT
            ) {
                circularEndpointQualified = true
            }
        } else {
            consecutiveCircularEndpointReadyDetections = 0
            circularEndpointQualified = false
        }
    }

    private fun updateCircularTrackingPhase() {
        val angle = controlledAngleDegrees ?: return
        when (phase) {
            TapeTrackingPhase.TRACKING -> {
                if (abs(angle) >= CURVE_ALIGNMENT_ENTER_ANGLE_DEGREES) {
                    phase = TapeTrackingPhase.ALIGNING_CURVE
                    consecutiveCurveAlignmentDetections = 0
                    lateralCorrectionActive = false
                    resetAppliedCommands()
                }
            }

            TapeTrackingPhase.ALIGNING_CURVE -> {
                if (abs(angle) <= CURVE_ALIGNMENT_EXIT_ANGLE_DEGREES) {
                    consecutiveCurveAlignmentDetections += 1
                    if (
                        consecutiveCurveAlignmentDetections >=
                        CURVE_ALIGNMENT_CONFIRMATION_COUNT
                    ) {
                        phase = TapeTrackingPhase.TRACKING
                        consecutiveCurveAlignmentDetections = 0
                        lateralCorrectionActive = false
                        resetAppliedCommands()
                    }
                } else {
                    consecutiveCurveAlignmentDetections = 0
                }
            }

            else -> Unit
        }
    }

    private fun isPlausibleCircularContinuation(
        observation: TapeTrackingObservation,
        reference: TapeTrackingObservation,
    ): Boolean =
        abs(observation.nearFieldOffsetFraction - reference.nearFieldOffsetFraction) <=
            REACQUISITION_MAX_OFFSET_JUMP_FRACTION &&
            axialAngleDifferenceDegrees(
                observation.angleFromVerticalDegrees,
                reference.angleFromVerticalDegrees,
            ) <= REACQUISITION_MAX_ANGLE_JUMP_DEGREES

    private fun isCredibleCircularReacquisitionPath(
        observation: TapeTrackingObservation,
    ): Boolean {
        if (observation.longSideFraction < CIRCULAR_REACQUISITION_MIN_PATH_FRACTION) {
            return false
        }
        return abs(observation.nearFieldOffsetFraction) <=
            REACQUISITION_ENTRY_OFFSET_FRACTION ||
            observation.bounds.bottom >= ENDPOINT_NEAR_EDGE_MIN_FRACTION
    }

    private fun isConsistentCircularReacquisition(
        observation: TapeTrackingObservation,
        previousCandidate: TapeTrackingObservation,
    ): Boolean =
        abs(observation.nearFieldOffsetFraction - previousCandidate.nearFieldOffsetFraction) <=
            REACQUISITION_CANDIDATE_OFFSET_TOLERANCE_FRACTION &&
            axialAngleDifferenceDegrees(
                observation.angleFromVerticalDegrees,
                previousCandidate.angleFromVerticalDegrees,
            ) <= REACQUISITION_CANDIDATE_ANGLE_TOLERANCE_DEGREES

    private fun resetCircularReacquisitionCandidate() {
        circularReacquisitionCandidate = null
        consecutiveCircularReacquisitionDetections = 0
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
        if (endpointReferenceBounds == null) {
            endpointCandidateSinceNanos = 0L
            consecutiveEndpointDetections = 0
            endpointQualificationArmed = false
            return
        }
        phase = TapeTrackingPhase.VERIFYING_ENDPOINT
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
        resetControlState()
    }

    private fun updateControlMeasurements(
        observation: TapeTrackingObservation,
        nowNanos: Long,
    ) {
        rawAngleDegrees = observation.angleFromVerticalDegrees
        rawHorizontalOffsetFraction = observation.nearFieldOffsetFraction
        val previousOffset = controlledHorizontalOffsetFraction
        val nextAngle = controlledAngleDegrees?.let {
            axialExponentialAverage(it, observation.angleFromVerticalDegrees, STABILIZED_FILTER_ALPHA)
        } ?: observation.angleFromVerticalDegrees
        val nextOffset = previousOffset?.let {
            exponentialAverage(it, observation.nearFieldOffsetFraction, STABILIZED_FILTER_ALPHA)
        } ?: observation.nearFieldOffsetFraction
        controlledLookaheadXFraction = controlledLookaheadXFraction?.let {
            exponentialAverage(it, observation.lookaheadXFraction, STABILIZED_FILTER_ALPHA)
        } ?: observation.lookaheadXFraction
        controlledLookaheadYFraction = controlledLookaheadYFraction?.let {
            exponentialAverage(it, observation.lookaheadYFraction, STABILIZED_FILTER_ALPHA)
        } ?: observation.lookaheadYFraction
        lookaheadFrameAspectRatio =
            observation.frameWidthPixels.toDouble() / observation.frameHeightPixels
        heightAboveGroundMeters = observation.heightAboveGroundMeters
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
        controlledLookaheadXFraction = null
        controlledLookaheadYFraction = null
        lookaheadFrameAspectRatio = null
        heightAboveGroundMeters = null
        offsetRatePerSecond = 0.0
        lastControlObservationAtNanos = 0L
        lateralCorrectionActive = false
    }

    private fun resetControlState() {
        consecutiveCurveAlignmentDetections = 0
        lastAcceptedCircularObservation = null
        circularDetectionGap = false
        resetCircularReacquisitionCandidate()
        consecutiveCircularEndpointReadyDetections = 0
        circularEndpointQualified = false
        clearControlMeasurements()
        resetAppliedCommands()
    }

    private fun resetAppliedCommands() {
        appliedYawRateDegreesPerSecond = 0.0
        appliedRightSpeedMetersPerSecond = 0.0
        lastCommandAtNanos = 0L
    }

    private fun desiredYawRate(purePursuitYawRate: Double): Double {
        val angle = controlledAngleDegrees ?: return 0.0
        val deadZone =
            if (mode == TapeTrackingMode.CIRCULAR) CIRCULAR_YAW_DEAD_ZONE_DEGREES
            else YAW_DEAD_ZONE_DEGREES
        val modeMaximumYawRate = when {
            phase == TapeTrackingPhase.VERIFYING_ENDPOINT ->
                ENDPOINT_MAX_YAW_RATE_DEGREES_PER_SECOND
            mode == TapeTrackingMode.CIRCULAR ->
                CIRCULAR_MAX_YAW_RATE_DEGREES_PER_SECOND
            else -> MAX_TRACKING_YAW_RATE_DEGREES_PER_SECOND
        }
        val maximumYawRate =
            if (lateralCorrectionActive) {
                minOf(modeMaximumYawRate, ANCHOR_ACQUISITION_MAX_YAW_RATE_DEGREES_PER_SECOND)
            } else {
                modeMaximumYawRate
            }
        val gain =
            if (mode == TapeTrackingMode.CIRCULAR) CIRCULAR_YAW_PROPORTIONAL_GAIN
            else YAW_PROPORTIONAL_GAIN
        val angleFeedback = if (abs(angle) <= deadZone) 0.0 else angle * gain
        return (purePursuitYawRate + angleFeedback)
            .coerceIn(-maximumYawRate, maximumYawRate)
    }
    private fun desiredCurveAlignmentYawRate(): Double {
        val angle = controlledAngleDegrees ?: return 0.0
        return (angle * CIRCULAR_YAW_PROPORTIONAL_GAIN).coerceIn(
            -CIRCULAR_MAX_YAW_RATE_DEGREES_PER_SECOND,
            CIRCULAR_MAX_YAW_RATE_DEGREES_PER_SECOND,
        )
    }


    /**
     * Ground-plane Pure Pursuit from the detected target point. The white overlay cross is the
     * image-space aircraft reference; image-up is forward and image-right is positive yaw.
     *
     * Height and the Mini 4 Pro camera's diagonal FOV provide the metric scale. If either the
     * height or a forward target is unavailable, the angle feedback remains the safe fallback.
     */
    private fun desiredPurePursuitYawRate(forwardSpeedMetersPerSecond: Double): Double {
        if (forwardSpeedMetersPerSecond <= 0.0) return 0.0
        val lookaheadX = controlledLookaheadXFraction ?: return 0.0
        val lookaheadY = controlledLookaheadYFraction ?: return 0.0
        val aspectRatio = lookaheadFrameAspectRatio ?: return 0.0
        val height = heightAboveGroundMeters?.takeIf {
            it >= PURE_PURSUIT_MIN_HEIGHT_METERS
        } ?: return 0.0
        val verticalHalfFovTangent =
            CAMERA_DIAGONAL_HALF_FOV_TANGENT / sqrt(1.0 + aspectRatio * aspectRatio)
        val groundFrameHeightMeters = 2.0 * height * verticalHalfFovTangent
        val lateralMeters =
            (lookaheadX - PURE_PURSUIT_TARGET_X_FRACTION) *
                aspectRatio * groundFrameHeightMeters
        val forwardMeters =
            (PURE_PURSUIT_TARGET_Y_FRACTION - lookaheadY) * groundFrameHeightMeters
        if (forwardMeters <= 0.0) return 0.0
        val lookaheadDistanceMeters = hypot(lateralMeters, forwardMeters)
        if (lookaheadDistanceMeters < PURE_PURSUIT_MIN_LOOKAHEAD_METERS) return 0.0
        val curvaturePerMeter =
            2.0 * lateralMeters / (lookaheadDistanceMeters * lookaheadDistanceMeters)
        return Math.toDegrees(forwardSpeedMetersPerSecond * curvaturePerMeter)
    }

    private fun desiredRightSpeed(): Double {
        val offset = controlledHorizontalOffsetFraction ?: return 0.0
        lateralCorrectionActive = when {
            lateralCorrectionActive && abs(offset) <= CENTERING_STOP_FRACTION -> false
            !lateralCorrectionActive && abs(offset) >= CENTERING_START_FRACTION -> true
            else -> lateralCorrectionActive
        }
        if (!lateralCorrectionActive) return 0.0
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
                ).coerceIn(-maximumCenteringSpeed, maximumCenteringSpeed)
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
        if (phase == TapeTrackingPhase.RECOVERING_AFTER_TURN) {
            return if (nowNanos < postTurnRecoveryUntilNanos) {
                CIRCULAR_POST_TURN_RECOVERY_SPEED_METERS_PER_SECOND
            } else {
                0.0
            }
        }
        if (phase == TapeTrackingPhase.VERIFYING_ENDPOINT) {
            return if (
                nowNanos - endpointVerificationStartedAtNanos <= ENDPOINT_PROBE_DURATION_NANOS
            ) {
                if (mode == TapeTrackingMode.CIRCULAR) {
                    CIRCULAR_ENDPOINT_PROBE_SPEED_METERS_PER_SECOND
                } else {
                    ENDPOINT_PROBE_SPEED_METERS_PER_SECOND
                }
            } else {
                0.0
            }
        }
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
        endpointReached: Boolean = false,
        purePursuitYawRateDegreesPerSecond: Double = 0.0,
    ): TapeTrackingDecision {
        check(!endpointReached || phase == TapeTrackingPhase.TURNING) {
            "A physical endpoint can only be reported while turning around"
        }
        return TapeTrackingDecision(
            phase = phase,
            yawRateDegreesPerSecond = yawRateDegreesPerSecond,
            forwardSpeedMetersPerSecond = forwardSpeedMetersPerSecond,
            rightSpeedMetersPerSecond = rightSpeedMetersPerSecond,
            endpointReached = endpointReached,
            rawAngleDegrees = rawAngleDegrees,
            controlledAngleDegrees = controlledAngleDegrees,
            rawOffsetFraction = rawHorizontalOffsetFraction,
            controlledOffsetFraction = controlledHorizontalOffsetFraction,
            offsetRatePerSecond = offsetRatePerSecond,
            purePursuitYawRateDegreesPerSecond = purePursuitYawRateDegreesPerSecond,
        )
    }

    private fun exponentialAverage(previous: Double, sample: Double, alpha: Double): Double =
        previous + alpha * (sample - previous)

    /**
     * Tape orientation is axial: +89° and -89° describe nearly the same line.
     * Interpolate across that seam instead of averaging both samples toward 0°.
     */
    private fun axialExponentialAverage(previous: Double, sample: Double, alpha: Double): Double {
        var delta = sample - previous
        while (delta > 90.0) delta -= 180.0
        while (delta < -90.0) delta += 180.0
        var averaged = previous + alpha * delta
        while (averaged > 90.0) averaged -= 180.0
        while (averaged < -90.0) averaged += 180.0
        return averaged
    }
    private fun axialAngleDifferenceDegrees(first: Double, second: Double): Double {
        var difference = abs(first - second)
        if (difference > 90.0) difference = 180.0 - difference
        return difference
    }

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
        const val ANCHOR_ACQUISITION_MAX_YAW_RATE_DEGREES_PER_SECOND = 2.0
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
        const val CIRCULAR_YAW_DEAD_ZONE_DEGREES = 1.5
        const val CIRCULAR_YAW_PROPORTIONAL_GAIN = 0.70
        const val CIRCULAR_MAX_YAW_RATE_DEGREES_PER_SECOND = 6.0
        const val CIRCULAR_TRACKING_FORWARD_SPEED_METERS_PER_SECOND = 0.05
        const val CIRCULAR_CORRECTION_FORWARD_SPEED_METERS_PER_SECOND = 0.03
        const val CIRCULAR_MAX_CENTERING_SPEED_METERS_PER_SECOND = 0.10
        const val CIRCULAR_STABLE_ANGLE_DEGREES = 8.0
        const val CURVE_ALIGNMENT_ENTER_ANGLE_DEGREES = 45.0
        const val CURVE_ALIGNMENT_EXIT_ANGLE_DEGREES = 20.0
        const val CURVE_ALIGNMENT_CONFIRMATION_COUNT = 3
        const val REACQUISITION_MAX_OFFSET_JUMP_FRACTION = 0.25
        const val REACQUISITION_MAX_ANGLE_JUMP_DEGREES = 45.0
        const val REACQUISITION_ENTRY_OFFSET_FRACTION = 0.30
        const val CIRCULAR_REACQUISITION_MIN_PATH_FRACTION = 0.60
        const val REACQUISITION_CANDIDATE_OFFSET_TOLERANCE_FRACTION = 0.08
        const val REACQUISITION_CANDIDATE_ANGLE_TOLERANCE_DEGREES = 20.0
        const val REACQUISITION_CONFIRMATION_COUNT = 3
        const val CIRCULAR_GAP_CONTINUATION_CONFIRMATION_COUNT = 2
        const val CIRCULAR_ENDPOINT_READY_MIN_FRACTION = 0.60
        const val CIRCULAR_ENDPOINT_READY_MAX_ANGLE_DEGREES = 30.0
        const val CIRCULAR_ENDPOINT_READY_MAX_OFFSET_FRACTION = 0.20
        const val CIRCULAR_ENDPOINT_READY_CONFIRMATION_COUNT = 3
        const val CIRCULAR_ENDPOINT_ENTRY_MISS_COUNT = 2
        const val CIRCULAR_ENDPOINT_RECOVERY_MIN_FRACTION = 0.60
        const val CIRCULAR_ENDPOINT_PROBE_SPEED_METERS_PER_SECOND = 0.02
        const val CIRCULAR_ENDPOINT_VERIFICATION_TIMEOUT_NANOS = 6_000_000_000L
        const val CIRCULAR_POST_TURN_RECOVERY_SPEED_METERS_PER_SECOND = 0.03
        const val CIRCULAR_POST_TURN_RECOVERY_NANOS = 6_000_000_000L
        const val CIRCULAR_POST_TURN_MIN_PATH_FRACTION = 0.20
        const val CIRCULAR_POST_TURN_MAX_ANGLE_DEGREES = 45.0
        const val CIRCULAR_POST_TURN_MAX_OFFSET_FRACTION = 0.20
        const val CIRCULAR_POST_TURN_CONFIRMATION_COUNT = 3
        const val CIRCULAR_MAX_MOVING_ANGLE_DEGREES = 45.0
        const val CIRCULAR_MAX_MOVING_OFFSET_FRACTION = 0.25
        const val PURE_PURSUIT_TARGET_X_FRACTION = 0.5
        const val PURE_PURSUIT_TARGET_Y_FRACTION = 0.94
        const val PURE_PURSUIT_MIN_HEIGHT_METERS = 0.30
        const val PURE_PURSUIT_MIN_LOOKAHEAD_METERS = 0.10
        const val CAMERA_VIDEO_DIAGONAL_FOV_DEGREES = 75.0
        val CAMERA_DIAGONAL_HALF_FOV_TANGENT =
            tan(Math.toRadians(CAMERA_VIDEO_DIAGONAL_FOV_DEGREES) / 2.0)
        private const val NANOS_PER_SECOND = 1_000_000_000.0
        private const val INITIAL_COMMAND_INTERVAL_SECONDS = 0.1
        private const val MAX_COMMAND_INTERVAL_SECONDS = 0.25
    }
}
