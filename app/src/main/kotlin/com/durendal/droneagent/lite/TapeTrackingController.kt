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

internal enum class TapeTrackingMode(val followsCurvedPath: Boolean) {
    STRAIGHT(false),
    CIRCULAR(true),
    CURVED_OUT_AND_BACK(true),
}



internal data class TapeTrackingObservation(
    val angleFromVerticalDegrees: Double,
    val longSideFraction: Double,
    val nearFieldOffsetFraction: Double,
    val bounds: NormalizedRect,
    val lookahead: TapeLookahead?,
    /**
     * What the geometry stage concluded this frame. The controller reads this
     * rather than inferring authority from whether a look-ahead point happens to
     * be present: one field, one meaning, decided where the evidence is.
     */
    val quality: PathQuality,
    /**
     * The chain's far end stops inside the frame. A chain cut off at the border
     * says the tape left the field of view, which shortening alone cannot tell
     * apart from an approaching end — and mistaking one for the other is a
     * 180-degree turn in the middle of the tape.
     */
    val endpointCandidate: Boolean,
    /** A loop has no end to reach, so it may never trigger a turnaround. */
    val closedLoop: Boolean,
    val frameWidthPixels: Int,
    val frameHeightPixels: Int,
    val heightAboveGroundMeters: Double?,
) {
    init {
        require(angleFromVerticalDegrees in -90.0..90.0)
        require(longSideFraction > 0.0 && longSideFraction.isFinite())
        require(nearFieldOffsetFraction in -0.5..0.5)
        require(frameWidthPixels > 0 && frameHeightPixels > 0)
        require(quality != PathQuality.LOST) { "a LOST path is reported as no observation" }
        require((quality == PathQuality.FULL_PATH) == (lookahead != null)) {
            "only FULL_PATH carries a look-ahead point"
        }
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
    val stopRequested: Boolean = false,
    val rawAngleDegrees: Double? = null,
    val controlledAngleDegrees: Double? = null,
    val rawOffsetFraction: Double? = null,
    val controlledOffsetFraction: Double? = null,
    val offsetRatePerSecond: Double = 0.0,
    val purePursuitYawRateDegreesPerSecond: Double = 0.0,
    val pathQuality: PathQuality = PathQuality.LOST,
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
    // One candidate sequence is shared by three mutually exclusive contexts:
    // post-turn recovery, REACQUIRING_PATH, and a TRACKING gap. Context transitions
    // reset it; full reacquisition alone may retain it across a brief detector flicker.
    private var circularReacquisitionCandidate: TapeTrackingObservation? = null
    private var circularReacquisitionCandidateAtNanos = 0L
    private var consecutiveCircularReacquisitionDetections = 0
    private var consecutiveReliableCircularPathDetections = 0
    private var reliableCircularPathEstablished = false
    private var appliedYawRateDegreesPerSecond = 0.0
    private var appliedRightSpeedMetersPerSecond = 0.0
    private var appliedForwardSpeedMetersPerSecond = 0.0
    private var lastCommandAtNanos = 0L
    private var mode: TapeTrackingMode = TapeTrackingMode.STRAIGHT
    private var pathQuality: PathQuality = PathQuality.LOST
    private var endpointTurnEnabled = true

    fun start(
        nowNanos: Long,
        mode: TapeTrackingMode = TapeTrackingMode.STRAIGHT,
        endpointTurnEnabled: Boolean = true,
    ) {
        enabled = true
        this.mode = mode
        this.endpointTurnEnabled = endpointTurnEnabled
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
        endpointTurnEnabled = true
    }

    fun resumeAfterTurn(nowNanos: Long) {
        check(enabled && phase == TapeTrackingPhase.TURNING)
        resetLeg()
        awaitingPostTurnDetection = true
        postTurnRecoveryUntilNanos = 0L
        beginRecentering(nowNanos)
    }


    fun observe(observation: TapeTrackingObservation?, nowNanos: Long) {
        if (!enabled || phase == TapeTrackingPhase.TURNING) return
        if (mode.followsCurvedPath) {
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
                    endpointCandidateSinceNanos = 0L
                    consecutiveEndpointDetections = 0
                    endpointReferenceBounds = null
                    if (
                        abs(observation.angleFromVerticalDegrees) >
                        STRAIGHT_REACQUISITION_MAX_ALIGNMENT_ANGLE_DEGREES
                    ) {
                        // A near-horizontal fragment is likely a glare edge, not a route.
                        clearControlMeasurements()
                    }
                    return
                }
                // Length alone is not evidence of an end: only a terminus that
                // stops inside the frame is, and a loop never is.
                val endpointCandidate =
                    observation.endpointCandidate &&
                        !observation.closedLoop &&
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

            TapeTrackingPhase.RECOVERING_AFTER_TURN -> {
                // The bounded search found tape again: resume following it from
                // this frame's measurement, not from anything held over the turn.
                awaitingPostTurnDetection = false
                phase = TapeTrackingPhase.TRACKING
                updateControlMeasurements(observation, nowNanos)
                lastDetectionAtNanos = nowNanos
                longestObservedTapeFraction = observation.longSideFraction
                resetAppliedCommands()
            }

            TapeTrackingPhase.DISABLED,
            TapeTrackingPhase.RECENTERING,
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
            phase == TapeTrackingPhase.VERIFYING_ENDPOINT &&
            endpointVerificationStartedAtNanos != 0L &&
            nowNanos - endpointVerificationStartedAtNanos >=
            ENDPOINT_VERIFICATION_TIMEOUT_NANOS
        ) {
            if (mode.followsCurvedPath) {
                beginCircularReacquisition()
            } else {
                resetAppliedCommands()
                return decision(
                    phase = phase,
                    yawRateDegreesPerSecond = 0.0,
                    stopRequested = true,
                )
            }
        }

        if (phase == TapeTrackingPhase.RECENTERING && nowNanos >= recenterUntilNanos) {
            // Both modes reacquire in an explicit, bounded, low-speed phase.
            // Advancing inside TRACKING with nothing detected was the one place
            // the controller moved on evidence it did not have.
            phase =
                if (awaitingPostTurnDetection) {
                    postTurnRecoveryUntilNanos = nowNanos + postTurnRecoveryDurationNanos()
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
            if (mode.followsCurvedPath) {
                beginCircularReacquisition()
            } else {
                // The bounded search is over and nothing was found. Hovering in
                // TRACKING with no path is a LOST state, and LOST commands zero.
                phase = TapeTrackingPhase.TRACKING
                clearControlMeasurements()
                resetAppliedCommands()
            }
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
                        // The bounded searches meter their own advance and stop on
                        // their own clocks; a detection gap is their normal state.
                        desiredForwardSpeed(nowNanos)
                    } else {
                        // A stale or missing measurement is no measurement in either
                        // mode: keeping speed here advances into unmeasured space.
                        0.0
                    },
            )
        }
        if (
            !mode.followsCurvedPath &&
            !endpointQualificationArmed &&
            abs(controlledAngleDegrees ?: 0.0) > ALIGNMENT_TOLERANCE_DEGREES
        ) {
            // Reacquisition may need an in-place yaw before the route is long enough
            // to re-arm endpoint tracking. Translation remains forbidden.
            appliedForwardSpeedMetersPerSecond = 0.0
            val (yawRate, _) = applyOutputLimits(
                targetYawRate = desiredYawRate(purePursuitYawRate = null),
                targetRightSpeed = 0.0,
                nowNanos = nowNanos,
            )
            return decision(
                phase = phase,
                yawRateDegreesPerSecond = yawRate,
            )
        }
        if (phase == TapeTrackingPhase.ALIGNING_CURVE) {
            appliedForwardSpeedMetersPerSecond = 0.0
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
        val profileForwardSpeed = desiredForwardSpeed(nowNanos)
        val purePursuitCurvature = desiredPurePursuitCurvaturePerMeter()
        val curvatureLimitedForwardSpeed =
            limitForwardSpeedForYaw(profileForwardSpeed, purePursuitCurvature)
        val forwardSpeed =
            applyForwardAccelerationLimit(curvatureLimitedForwardSpeed, nowNanos)
        val purePursuitYawRate =
            desiredPurePursuitYawRate(forwardSpeed, purePursuitCurvature)
        val targetYawRate = desiredYawRate(purePursuitYawRate)
        val (yawRate, rightSpeed) = applyOutputLimits(
            targetYawRate,
            targetRightSpeed,
            nowNanos,
        )
        return decision(
            phase = phase,
            yawRateDegreesPerSecond = yawRate,
            forwardSpeedMetersPerSecond = forwardSpeed,
            rightSpeedMetersPerSecond = rightSpeed,
            purePursuitYawRateDegreesPerSecond = purePursuitYawRate ?: 0.0,
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
            disableCircularLateralCorrection()
            if (!reliableCircularPathEstablished) {
                consecutiveReliableCircularPathDetections = 0
            }
            if (phase == TapeTrackingPhase.REACQUIRING_PATH) {
                expireCircularReacquisitionCandidateIfStale(nowNanos)
            } else {
                resetCircularReacquisitionCandidate()
            }
            when (phase) {
                TapeTrackingPhase.TRACKING -> {
                    if (reliableCircularPathEstablished) {
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
                    reliableCircularPathEstablished &&
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
        // Endpoint inference is based on reliable-route absence. Any matching full path
        // proves the route is present again, even when its far end also looks terminal.
        if (
            trackedObservation.quality == PathQuality.FULL_PATH &&
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
        consecutiveReliableCircularPathDetections = 0
        reliableCircularPathEstablished = false
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
        val commandStillFresh =
            nowNanos - lastDetectionAtNanos <= DETECTION_COMMAND_STALE_NANOS
        if (!commandStillFresh) {
            clearControlMeasurements()
            resetAppliedCommands()
        }
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
        expireCircularReacquisitionCandidateIfStale(nowNanos)
        if (!isCredibleCircularReacquisitionPath(observation)) {
            return
        }

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
        circularReacquisitionCandidateAtNanos = nowNanos
        if (
            consecutiveCircularReacquisitionDetections >=
            REACQUISITION_CONFIRMATION_COUNT
        ) {
            completeCircularReacquisition(observation, nowNanos)
        }
    }

    private fun beginCircularReacquisition() {
        phase = TapeTrackingPhase.REACQUIRING_PATH
        consecutiveCurveAlignmentDetections = 0
        consecutiveReliableCircularPathDetections = 0
        reliableCircularPathEstablished = false
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
        if (!endpointTurnEnabled) {
            consecutiveReliableCircularPathDetections = 0
            reliableCircularPathEstablished = false
            return
        }
        val reliablePath =
            observation.quality == PathQuality.FULL_PATH &&
                !observation.closedLoop &&
                observation.longSideFraction >= CIRCULAR_RELIABLE_PATH_MIN_FRACTION &&
                observation.bounds.bottom >= ENDPOINT_NEAR_EDGE_MIN_FRACTION
        if (reliablePath) {
            consecutiveReliableCircularPathDetections += 1
            if (
                consecutiveReliableCircularPathDetections >=
                CIRCULAR_RELIABLE_PATH_CONFIRMATION_COUNT
            ) {
                reliableCircularPathEstablished = true
            }
        } else if (!reliableCircularPathEstablished) {
            consecutiveReliableCircularPathDetections = 0
        }
    }

    private fun updateCircularTrackingPhase() {
        val angle = controlledAngleDegrees ?: return
        when (phase) {
            TapeTrackingPhase.TRACKING -> {
                if (
                    abs(angle) >= CURVE_ALIGNMENT_ENTER_ANGLE_DEGREES &&
                    !hasUsablePurePursuitTarget()
                ) {
                    phase = TapeTrackingPhase.ALIGNING_CURVE
                    consecutiveCurveAlignmentDetections = 0
                    disableCircularLateralCorrection()
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
                        disableCircularLateralCorrection()
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

    private fun expireCircularReacquisitionCandidateIfStale(nowNanos: Long) {
        if (
            circularReacquisitionCandidateAtNanos != 0L &&
            nowNanos - circularReacquisitionCandidateAtNanos >
            REACQUISITION_CANDIDATE_MAX_GAP_NANOS
        ) {
            resetCircularReacquisitionCandidate()
        }
    }

    private fun resetCircularReacquisitionCandidate() {
        circularReacquisitionCandidate = null
        circularReacquisitionCandidateAtNanos = 0L
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
        setPathQuality(observation.quality)
        rawAngleDegrees = observation.angleFromVerticalDegrees
        rawHorizontalOffsetFraction = observation.nearFieldOffsetFraction
        val previousOffset = controlledHorizontalOffsetFraction
        val nextAngle = controlledAngleDegrees?.let {
            axialExponentialAverage(it, observation.angleFromVerticalDegrees, STABILIZED_FILTER_ALPHA)
        } ?: observation.angleFromVerticalDegrees
        val nextOffset = previousOffset?.let {
            exponentialAverage(it, observation.nearFieldOffsetFraction, STABILIZED_FILTER_ALPHA)
        } ?: observation.nearFieldOffsetFraction
        // A path the detector could not follow far enough reports no look-ahead.
        // Holding the previous one would steer from evidence this frame does not
        // have, so Pure Pursuit loses its target instead of inheriting a stale one.
        val lookahead = observation.lookahead
        if (lookahead == null) {
            controlledLookaheadXFraction = null
            controlledLookaheadYFraction = null
        } else {
            controlledLookaheadXFraction = controlledLookaheadXFraction?.let {
                exponentialAverage(it, lookahead.xFraction, STABILIZED_FILTER_ALPHA)
            } ?: lookahead.xFraction
            controlledLookaheadYFraction = controlledLookaheadYFraction?.let {
                exponentialAverage(it, lookahead.yFraction, STABILIZED_FILTER_ALPHA)
            } ?: lookahead.yFraction
        }
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

    /**
     * Records this frame's quality. LOST is the resting state, so a frame that
     * produced no observation leaves the controller with no authority rather
     * than with the authority the previous frame had.
     */
    private fun setPathQuality(value: PathQuality) {
        pathQuality = value
    }

    /**
     * LOST is expressed by clearing the filtered measurements, not by a separate
     * branch in every command: with no angle, offset or look-ahead there is
     * nothing for a command to be computed from, and no path by which a previous
     * frame's value could survive into this one.
     */

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
        disableCircularLateralCorrection()
        pathQuality = PathQuality.LOST
    }

    private fun resetControlState() {
        consecutiveCurveAlignmentDetections = 0
        lastAcceptedCircularObservation = null
        circularDetectionGap = false
        resetCircularReacquisitionCandidate()
        consecutiveReliableCircularPathDetections = 0
        reliableCircularPathEstablished = false
        clearControlMeasurements()
        resetAppliedCommands()
    }

    private fun resetAppliedCommands() {
        appliedYawRateDegreesPerSecond = 0.0
        appliedRightSpeedMetersPerSecond = 0.0
        appliedForwardSpeedMetersPerSecond = 0.0
        lastCommandAtNanos = 0L
    }

    private fun desiredYawRate(purePursuitYawRate: Double?): Double {
        val angle = controlledAngleDegrees ?: return 0.0
        val deadZone =
            if (mode.followsCurvedPath) CIRCULAR_YAW_DEAD_ZONE_DEGREES
            else YAW_DEAD_ZONE_DEGREES
        val modeMaximumYawRate = when {
            phase == TapeTrackingPhase.VERIFYING_ENDPOINT ->
                ENDPOINT_MAX_YAW_RATE_DEGREES_PER_SECOND
            mode.followsCurvedPath ->
                CIRCULAR_MAX_YAW_RATE_DEGREES_PER_SECOND
            else -> MAX_TRACKING_YAW_RATE_DEGREES_PER_SECOND
        }
        val maximumYawRate =
            if (pathQuality == PathQuality.NEAR_FIELD_ONLY) {
                // In-place alignment only, and bounded: a near-field-only path is
                // enough to point at, never enough to chase.
                minOf(modeMaximumYawRate, NEAR_FIELD_MAX_YAW_RATE_DEGREES_PER_SECOND)
            } else if (lateralCorrectionActive && !mode.followsCurvedPath) {
                minOf(modeMaximumYawRate, ANCHOR_ACQUISITION_MAX_YAW_RATE_DEGREES_PER_SECOND)
            } else {
                modeMaximumYawRate
            }
        val gain =
            if (mode.followsCurvedPath) CIRCULAR_YAW_PROPORTIONAL_GAIN
            else YAW_PROPORTIONAL_GAIN
        val angleFeedback = if (abs(angle) <= deadZone) 0.0 else angle * gain
        return (purePursuitYawRate ?: angleFeedback)
            .coerceIn(-maximumYawRate, maximumYawRate)
    }
    private fun desiredCurveAlignmentYawRate(): Double {
        val angle = controlledAngleDegrees ?: return 0.0
        val maximumYawRate =
            if (pathQuality == PathQuality.NEAR_FIELD_ONLY) {
                NEAR_FIELD_MAX_YAW_RATE_DEGREES_PER_SECOND
            } else {
                CIRCULAR_MAX_YAW_RATE_DEGREES_PER_SECOND
            }
        return (angle * CIRCULAR_YAW_PROPORTIONAL_GAIN).coerceIn(
            -maximumYawRate,
            maximumYawRate,
        )
    }

    private fun hasUsablePurePursuitTarget(): Boolean {
        if (pathQuality != PathQuality.FULL_PATH) return false
        val lookaheadX = controlledLookaheadXFraction ?: return false
        val lookaheadY = controlledLookaheadYFraction ?: return false
        val aspectRatio = lookaheadFrameAspectRatio ?: return false
        val height = heightAboveGroundMeters?.takeIf {
            it >= PURE_PURSUIT_MIN_HEIGHT_METERS
        } ?: return false
        val normalizedLateralDistance =
            (lookaheadX - TRACKING_TARGET_X_FRACTION) * aspectRatio
        val normalizedForwardDistance = TRACKING_TARGET_Y_FRACTION - lookaheadY
        if (normalizedForwardDistance <= 0.0) return false
        val verticalHalfFovTangent =
            CAMERA_DIAGONAL_HALF_FOV_TANGENT / sqrt(1.0 + aspectRatio * aspectRatio)
        val groundFrameHeightMeters = 2.0 * height * verticalHalfFovTangent
        return groundFrameHeightMeters *
            hypot(normalizedLateralDistance, normalizedForwardDistance) >=
            PURE_PURSUIT_MIN_LOOKAHEAD_METERS
    }
    /**
     * Ground-plane Pure Pursuit curvature from the detected target point. The white overlay
     * cross is the image-space aircraft reference; image-up is forward. Meeting-room flight
     * evidence confirms that image-right maps to positive DJI angular velocity: the opposite
     * mapping drove a centered route monotonically out through the right frame edge.
     *
     * Height and the Mini 4 Pro camera's diagonal FOV provide the metric scale. A path without
     * a look-ahead cannot reach here: only FULL_PATH carries one, and an observation without one
     * clears the filtered target.
     */
    private fun desiredPurePursuitCurvaturePerMeter(): Double? {
        if (!hasUsablePurePursuitTarget()) return null
        val lookaheadX = checkNotNull(controlledLookaheadXFraction)
        val lookaheadY = checkNotNull(controlledLookaheadYFraction)
        val aspectRatio = checkNotNull(lookaheadFrameAspectRatio)
        val height = checkNotNull(heightAboveGroundMeters)
        val verticalHalfFovTangent =
            CAMERA_DIAGONAL_HALF_FOV_TANGENT / sqrt(1.0 + aspectRatio * aspectRatio)
        val groundFrameHeightMeters = 2.0 * height * verticalHalfFovTangent
        val lateralMeters =
            (lookaheadX - TRACKING_TARGET_X_FRACTION) *
                aspectRatio * groundFrameHeightMeters
        val forwardMeters =
            (TRACKING_TARGET_Y_FRACTION - lookaheadY) * groundFrameHeightMeters
        val lookaheadDistanceMeters = hypot(lateralMeters, forwardMeters)
        return 2.0 * lateralMeters / (lookaheadDistanceMeters * lookaheadDistanceMeters)
    }

    private fun desiredPurePursuitYawRate(
        forwardSpeedMetersPerSecond: Double,
        curvaturePerMeter: Double?,
    ): Double? {
        if (forwardSpeedMetersPerSecond <= 0.0 || curvaturePerMeter == null) return null
        return Math.toDegrees(forwardSpeedMetersPerSecond * curvaturePerMeter)
    }

    private fun limitForwardSpeedForYaw(
        profileForwardSpeedMetersPerSecond: Double,
        curvaturePerMeter: Double?,
    ): Double {
        if (!mode.followsCurvedPath || phase != TapeTrackingPhase.TRACKING) {
            return profileForwardSpeedMetersPerSecond
        }
        if (curvaturePerMeter == null) {
            // Full speed needs a metric look-ahead. Missing height or a target
            // too close to project safely keeps the conservative profile.
            return minOf(
                profileForwardSpeedMetersPerSecond,
                CIRCULAR_CORRECTION_FORWARD_SPEED_METERS_PER_SECOND,
            )
        }
        if (abs(curvaturePerMeter) <= PURE_PURSUIT_STRAIGHT_CURVATURE_PER_METER) {
            return profileForwardSpeedMetersPerSecond
        }
        val maximumForwardSpeed =
            Math.toRadians(CIRCULAR_YAW_SPEED_BUDGET_DEGREES_PER_SECOND) /
                abs(curvaturePerMeter)
        return minOf(profileForwardSpeedMetersPerSecond, maximumForwardSpeed)
    }

    private fun applyForwardAccelerationLimit(
        targetForwardSpeedMetersPerSecond: Double,
        nowNanos: Long,
    ): Double {
        if (!mode.followsCurvedPath) {
            appliedForwardSpeedMetersPerSecond = targetForwardSpeedMetersPerSecond
            return appliedForwardSpeedMetersPerSecond
        }
        if (targetForwardSpeedMetersPerSecond <= appliedForwardSpeedMetersPerSecond) {
            // Curvature, quality and stop paths take speed away immediately. Only
            // acceleration is gradual; safety deceleration is never delayed.
            appliedForwardSpeedMetersPerSecond = targetForwardSpeedMetersPerSecond
            return appliedForwardSpeedMetersPerSecond
        }
        appliedForwardSpeedMetersPerSecond = moveToward(
            appliedForwardSpeedMetersPerSecond,
            targetForwardSpeedMetersPerSecond,
            MAX_FORWARD_ACCELERATION_METERS_PER_SECOND_SQUARED *
                outputIntervalSeconds(nowNanos),
        )
        return appliedForwardSpeedMetersPerSecond
    }


    private fun disableCircularLateralCorrection() {
        lateralCorrectionActive = false
        if (mode.followsCurvedPath) {
            appliedRightSpeedMetersPerSecond = 0.0
        }
    }

    private fun desiredRightSpeed(): Double {
        // Translation is the command a wrong path turns into a crash, so it is
        // the first thing quality takes away.
        if (pathQuality != PathQuality.FULL_PATH) return 0.0
        val offset = controlledHorizontalOffsetFraction ?: return 0.0
        if (mode.followsCurvedPath) {
            if (
                phase != TapeTrackingPhase.TRACKING ||
                circularDetectionGap
            ) {
                return 0.0
            }
            val effectiveOffset = when {
                offset > CIRCULAR_CENTERING_DEAD_ZONE_FRACTION ->
                    offset - CIRCULAR_CENTERING_DEAD_ZONE_FRACTION
                offset < -CIRCULAR_CENTERING_DEAD_ZONE_FRACTION ->
                    offset + CIRCULAR_CENTERING_DEAD_ZONE_FRACTION
                else -> 0.0
            }
            return (effectiveOffset * CIRCULAR_LATERAL_PROPORTIONAL_GAIN).coerceIn(
                -CIRCULAR_MAX_CENTERING_SPEED_METERS_PER_SECOND,
                CIRCULAR_MAX_CENTERING_SPEED_METERS_PER_SECOND,
            )
        }
        lateralCorrectionActive = when {
            lateralCorrectionActive && abs(offset) <= CENTERING_STOP_FRACTION -> false
            !lateralCorrectionActive && abs(offset) >= CENTERING_START_FRACTION -> true
            else -> lateralCorrectionActive
        }
        if (!lateralCorrectionActive) return 0.0
        val correction =
            (
                offset * LATERAL_PROPORTIONAL_GAIN +
                    offsetRatePerSecond * LATERAL_DERIVATIVE_GAIN
                ).coerceIn(
                -MAX_CENTERING_SPEED_METERS_PER_SECOND,
                MAX_CENTERING_SPEED_METERS_PER_SECOND,
            )
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
        // NEAR_FIELD_ONLY knows which way the tape under the aircraft runs and
        // nothing about what is ahead. Aligning in place is safe; advancing is
        // advancing into unmeasured space.
        if (pathQuality != PathQuality.FULL_PATH && phase == TapeTrackingPhase.TRACKING) return 0.0
        if (phase == TapeTrackingPhase.RECOVERING_AFTER_TURN) {
            return if (nowNanos < postTurnRecoveryUntilNanos) {
                postTurnRecoverySpeedMetersPerSecond()
            } else {
                0.0
            }
        }
        if (phase == TapeTrackingPhase.VERIFYING_ENDPOINT) {
            return if (
                nowNanos - endpointVerificationStartedAtNanos <= ENDPOINT_PROBE_DURATION_NANOS
            ) {
                if (mode.followsCurvedPath) {
                    CIRCULAR_ENDPOINT_PROBE_SPEED_METERS_PER_SECOND
                } else {
                    ENDPOINT_PROBE_SPEED_METERS_PER_SECOND
                }
            } else {
                0.0
            }
        }
        if (mode.followsCurvedPath) {
            if (phase != TapeTrackingPhase.TRACKING) return 0.0
            val angle = controlledAngleDegrees ?: return 0.0
            val offset = controlledHorizontalOffsetFraction ?: return 0.0
            val hasUsableLookahead = hasUsablePurePursuitTarget()
            if (
                !hasUsableLookahead &&
                (
                    abs(offset) > CIRCULAR_MAX_MOVING_OFFSET_FRACTION ||
                        abs(angle) > CIRCULAR_MAX_MOVING_ANGLE_DEGREES
                )
            ) {
                // A large error is recoverable by an interception arc only while
                // the detector supplies a metric target ahead. Without one,
                // advancing would merely chase the local fragment out of frame.
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

    private fun postTurnRecoveryDurationNanos(): Long =
        if (mode.followsCurvedPath) {
            CIRCULAR_POST_TURN_RECOVERY_NANOS
        } else {
            POST_TURN_RECOVERY_NANOS
        }

    private fun postTurnRecoverySpeedMetersPerSecond(): Double =
        if (mode.followsCurvedPath) {
            CIRCULAR_POST_TURN_RECOVERY_SPEED_METERS_PER_SECOND
        } else {
            POST_TURN_RECOVERY_SPEED_METERS_PER_SECOND
        }

    private fun applyOutputLimits(
        targetYawRate: Double,
        targetRightSpeed: Double,
        nowNanos: Long,
    ): Pair<Double, Double> {
        val elapsedSeconds = outputIntervalSeconds(nowNanos)
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

    private fun outputIntervalSeconds(nowNanos: Long): Double =
        if (lastCommandAtNanos == 0L) {
            INITIAL_COMMAND_INTERVAL_SECONDS
        } else {
            ((nowNanos - lastCommandAtNanos) / NANOS_PER_SECOND)
                .coerceIn(0.0, MAX_COMMAND_INTERVAL_SECONDS)
        }

    private fun decision(
        phase: TapeTrackingPhase,
        yawRateDegreesPerSecond: Double,
        forwardSpeedMetersPerSecond: Double = 0.0,
        rightSpeedMetersPerSecond: Double = 0.0,
        endpointReached: Boolean = false,
        stopRequested: Boolean = false,
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
            stopRequested = stopRequested,
            rawAngleDegrees = rawAngleDegrees,
            controlledAngleDegrees = controlledAngleDegrees,
            rawOffsetFraction = rawHorizontalOffsetFraction,
            controlledOffsetFraction = controlledHorizontalOffsetFraction,
            offsetRatePerSecond = offsetRatePerSecond,
            purePursuitYawRateDegreesPerSecond = purePursuitYawRateDegreesPerSecond,
            pathQuality = pathQuality,
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
        /** Reject near-horizontal glare fragments while allowing in-place route realignment. */
        const val STRAIGHT_REACQUISITION_MAX_ALIGNMENT_ANGLE_DEGREES = 45.0
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
        const val MAX_YAW_ACCELERATION_DEGREES_PER_SECOND_SQUARED = 15.0
        const val MAX_FORWARD_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.02
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
        // The detector runs at about 3.8 Hz in flight. Keep four missed sample windows plus
        // callback jitter before hovering; at 0.12 m/s the added 250 ms is at most 3 cm.
        const val DETECTION_COMMAND_STALE_NANOS = 1_250_000_000L
        const val ENDPOINT_NEAR_EDGE_MIN_FRACTION = 0.90
        const val ENDPOINT_TRACK_MIN_OVERLAP = 0.20
        const val ENDPOINT_MAX_YAW_RATE_DEGREES_PER_SECOND = 2.0
        const val ENDPOINT_MAX_CENTERING_SPEED_METERS_PER_SECOND = 0.03

        /** Bounded in-place alignment allowed when only the near field is credible. */
        const val NEAR_FIELD_MAX_YAW_RATE_DEGREES_PER_SECOND = 3.0
        const val CIRCULAR_YAW_DEAD_ZONE_DEGREES = 1.5
        const val CIRCULAR_YAW_PROPORTIONAL_GAIN = 0.70
        const val CIRCULAR_MAX_YAW_RATE_DEGREES_PER_SECOND = 15.0
        const val CIRCULAR_YAW_SPEED_BUDGET_DEGREES_PER_SECOND = 13.0
        const val CIRCULAR_TRACKING_FORWARD_SPEED_METERS_PER_SECOND = 0.12
        // The three-lap flight used this profile without the aggressive yaw saturation seen
        // in the 55-second attempt. Brief plausible detector gaps now preserve its smooth
        // acceleration instead of forcing a stop-and-relaunch.
        const val CIRCULAR_CORRECTION_FORWARD_SPEED_METERS_PER_SECOND = 0.11
        const val CIRCULAR_CENTERING_DEAD_ZONE_FRACTION = 0.05
        const val CIRCULAR_LATERAL_PROPORTIONAL_GAIN = 0.07
        const val CIRCULAR_MAX_CENTERING_SPEED_METERS_PER_SECOND = 0.02
        const val CIRCULAR_STABLE_ANGLE_DEGREES = 8.0
        const val CURVE_ALIGNMENT_ENTER_ANGLE_DEGREES = 45.0
        const val CURVE_ALIGNMENT_EXIT_ANGLE_DEGREES = 20.0
        const val CURVE_ALIGNMENT_CONFIRMATION_COUNT = 3
        const val REACQUISITION_MAX_OFFSET_JUMP_FRACTION = 0.25
        const val REACQUISITION_MAX_ANGLE_JUMP_DEGREES = 45.0
        const val REACQUISITION_ENTRY_OFFSET_FRACTION = 0.30
        // A lost-path reacquisition must be a substantial route, not a short floor feature.
        // Flight evidence after the detector hardening showed that waiting for a third
        // mutually consistent frame only prolonged a stationary tail reacquisition.
        const val CIRCULAR_REACQUISITION_MIN_PATH_FRACTION = 0.60
        const val REACQUISITION_CANDIDATE_OFFSET_TOLERANCE_FRACTION = 0.08
        const val REACQUISITION_CANDIDATE_ANGLE_TOLERANCE_DEGREES = 20.0
        const val REACQUISITION_CONFIRMATION_COUNT = 2
        // Detector flicker on wrinkled tape may insert null or short-fragment frames.
        const val REACQUISITION_CANDIDATE_MAX_GAP_NANOS = 500_000_000L
        const val CIRCULAR_GAP_CONTINUATION_CONFIRMATION_COUNT = 2
        // In out-and-back mode the operational endpoint signal is not a visible tape tip:
        // the latest flights end where tape passes under the mat, so no trustworthy
        // in-frame terminus exists. Establish a route from several full, near-aircraft
        // observations, then let sustained disappearance enter the existing low-speed
        // probe. A brief gap cannot arm this state, and a returning full path cancels it.
        const val CIRCULAR_RELIABLE_PATH_MIN_FRACTION = 0.60
        const val CIRCULAR_RELIABLE_PATH_CONFIRMATION_COUNT = 3
        const val CIRCULAR_ENDPOINT_ENTRY_MISS_COUNT = 2
        const val CIRCULAR_ENDPOINT_RECOVERY_MIN_FRACTION = 0.60
        const val CIRCULAR_ENDPOINT_PROBE_SPEED_METERS_PER_SECOND = 0.02
        const val ENDPOINT_VERIFICATION_TIMEOUT_NANOS = 6_000_000_000L
        const val CIRCULAR_POST_TURN_RECOVERY_SPEED_METERS_PER_SECOND = 0.03
        const val CIRCULAR_POST_TURN_RECOVERY_NANOS = 6_000_000_000L
        // Immediately after a half-turn the camera may see only the near fragment; three
        // centered, mutually consistent frames compensate for this deliberately lower length.
        const val CIRCULAR_POST_TURN_MIN_PATH_FRACTION = 0.20
        const val CIRCULAR_POST_TURN_MAX_ANGLE_DEGREES = 45.0
        const val CIRCULAR_POST_TURN_MAX_OFFSET_FRACTION = 0.20
        const val CIRCULAR_POST_TURN_CONFIRMATION_COUNT = 3
        const val CIRCULAR_MAX_MOVING_ANGLE_DEGREES = 45.0
        // Do not accept a path into tracking and then forbid that same path from advancing.
        const val CIRCULAR_MAX_MOVING_OFFSET_FRACTION = REACQUISITION_ENTRY_OFFSET_FRACTION
        const val PURE_PURSUIT_MIN_HEIGHT_METERS = 0.30
        const val PURE_PURSUIT_MIN_LOOKAHEAD_METERS = 0.10
        private const val PURE_PURSUIT_STRAIGHT_CURVATURE_PER_METER = 1e-6
        const val CAMERA_VIDEO_DIAGONAL_FOV_DEGREES = 75.0
        val CAMERA_DIAGONAL_HALF_FOV_TANGENT =
            tan(Math.toRadians(CAMERA_VIDEO_DIAGONAL_FOV_DEGREES) / 2.0)
        private const val NANOS_PER_SECOND = 1_000_000_000.0
        private const val INITIAL_COMMAND_INTERVAL_SECONDS = 0.1
        private const val MAX_COMMAND_INTERVAL_SECONDS = 0.25
    }
}
