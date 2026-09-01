package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sign
import kotlin.math.sqrt
import kotlin.math.tan

internal enum class TapeTrackingPhase {
    DISABLED,
    RECENTERING,
    RECOVERING_AFTER_TURN,
    RECOVERING_FRAME_STREAM,
    TRACKING,
    ALIGNING_CURVE,
    REACQUIRING_PATH,
    VERIFYING_ENDPOINT,
    TURNING,
}

internal enum class TapeTrackingMode(
    val followsCurvedPath: Boolean,
    val automaticallyCapturesEvidence: Boolean = false,
) {
    STRAIGHT(false),
    CIRCULAR(true, automaticallyCapturesEvidence = true),
    FIXED_HEADING(true, automaticallyCapturesEvidence = true),
    CURVED_OUT_AND_BACK(true),
}

internal val TapeTrackingMode.detectionMode: TapeDetectionMode
    get() = if (followsCurvedPath) TapeDetectionMode.PATH else TapeDetectionMode.STRAIGHT
internal enum class CircularTrackingSpeed(
    val targetMetersPerSecond: Double,
    val latencyCompensatedLookahead: Boolean,
) {
    ANGLE(0.10, latencyCompensatedLookahead = true),
    SLOW(0.50, latencyCompensatedLookahead = false),
    FAST(0.85, latencyCompensatedLookahead = true),
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
    /** Source-frame capture time, not detector completion time. Zero is allowed in tests. */
    val capturedAtNanos: Long = 0L,
    val heightAboveGroundMeters: Double?,
    val confidence: Double = 0.0,
    val centerline: TapeCenterlinePath? = null,
    val actualTravelDirectionDegrees: Double? = null,
) {
    init {
        require(angleFromVerticalDegrees in -90.0..90.0)
        require(longSideFraction > 0.0 && longSideFraction.isFinite())
        require(nearFieldOffsetFraction in -0.5..0.5)
        require(frameWidthPixels > 0 && frameHeightPixels > 0)
        require(capturedAtNanos >= 0L)
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
    val pathBearingDegrees: Double? = null,
    val rawCurvaturePerMeter: Double? = null,
    val predictedCurvaturePerMeter: Double? = null,
    val trustedCurvaturePerMeter: Double? = null,
    val instantaneousCurvatureSpeedCapMetersPerSecond: Double? = null,
    val trustedCurvatureSpeedCapMetersPerSecond: Double? = null,
    val profileForwardSpeedMetersPerSecond: Double = 0.0,
    val accelerationLimitedForwardSpeedMetersPerSecond: Double = 0.0,
    val pathQuality: PathQuality = PathQuality.LOST,
)
private data class PredictedLookahead(
    val xFraction: Double,
    val yFraction: Double,
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
    private var rawLookaheadXFraction: Double? = null
    private var rawLookaheadYFraction: Double? = null
    private var lookaheadXRatePerSecond = 0.0
    private var lookaheadYRatePerSecond = 0.0
    private var measurementCapturedAtNanos = 0L
    private var lookaheadFrameAspectRatio: Double? = null
    private var heightAboveGroundMeters: Double? = null
    private var offsetRatePerSecond = 0.0
    private var measuredOffsetRatePerSecond = 0.0
    private var rawCurvaturePerMeter: Double? = null
    private var predictedCurvaturePerMeter: Double? = null
    private var trustedCurvaturePerMeter: Double? = null
    private var instantaneousCurvatureSpeedCapMetersPerSecond: Double? = null
    private var trustedCurvatureSpeedCapMetersPerSecond =
        CIRCULAR_CORRECTION_FORWARD_SPEED_METERS_PER_SECOND
    private val curvatureSamplesPerMeter = DoubleArray(CURVATURE_MEDIAN_SAMPLE_COUNT)
    private var curvatureSampleCount = 0
    private var curvatureSampleWriteIndex = 0
    private var lastCurvatureSampleAtNanos = 0L
    private var profileForwardSpeedMetersPerSecond = 0.0
    private var accelerationLimitedForwardSpeedMetersPerSecond = 0.0
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
    private var endpointMissSinceNanos = 0L
    private var endpointReferenceBounds: NormalizedRect? = null
    private var lateralCorrectionActive = false
    private var curveAlignmentSinceNanos = 0L
    private var curveMisalignmentSinceNanos = 0L
    private var lastAcceptedCircularObservation: TapeTrackingObservation? = null
    private var circularDetectionGap = false
    // One candidate sequence is shared by three mutually exclusive contexts:
    // post-turn recovery, REACQUIRING_PATH, and a TRACKING gap. Context transitions
    // reset it; full reacquisition alone may retain it across a brief detector flicker.
    private var circularReacquisitionCandidate: TapeTrackingObservation? = null
    private var circularReacquisitionCandidateAtNanos = 0L
    private var circularCandidateSequenceSinceNanos = 0L
    private var reliableCircularPathSinceNanos = 0L
    private var reliableCircularPathEstablished = false
    private var appliedYawRateDegreesPerSecond = 0.0
    private var appliedRightSpeedMetersPerSecond = 0.0
    private var appliedForwardSpeedMetersPerSecond = 0.0
    private var appliedForwardAccelerationMetersPerSecondSquared = 0.0
    private var lastCommandAtNanos = 0L
    private var mode: TapeTrackingMode = TapeTrackingMode.STRAIGHT
    private var circularTrackingSpeed = CircularTrackingSpeed.FAST
    private var pathQuality: PathQuality = PathQuality.LOST
    private var endpointTurnEnabled = true
    private val fixedHeadingLapController = FixedHeadingLapController()

    fun start(
        nowNanos: Long,
        mode: TapeTrackingMode = TapeTrackingMode.STRAIGHT,
        endpointTurnEnabled: Boolean = true,
        circularTrackingSpeed: CircularTrackingSpeed = CircularTrackingSpeed.FAST,
        fixedHeadingActuationPhaseLead: FixedHeadingActuationPhaseLead =
            FixedHeadingActuationPhaseLead.DEGREES_0,
    ) {
        enabled = true
        this.mode = mode
        this.endpointTurnEnabled = endpointTurnEnabled
        this.circularTrackingSpeed = circularTrackingSpeed
        resetLeg()
        awaitingPostTurnDetection = false
        if (mode == TapeTrackingMode.FIXED_HEADING) {
            phase = TapeTrackingPhase.RECENTERING
            fixedHeadingLapController.start(
                nowNanos,
                fixedHeadingActuationPhaseLead,
            )
        } else {
            beginRecentering(nowNanos)
        }
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
        fixedHeadingLapController.stop()
        resetControlState()
        mode = TapeTrackingMode.STRAIGHT
        endpointTurnEnabled = true
        circularTrackingSpeed = CircularTrackingSpeed.FAST
    }

    fun resumeAfterTurn(nowNanos: Long) {
        check(enabled && phase == TapeTrackingPhase.TURNING)
        resetLeg()
        awaitingPostTurnDetection = true
        postTurnRecoveryUntilNanos = 0L
        beginRecentering(nowNanos)
    }

    fun beginFrameStreamRecovery(nowNanos: Long): Boolean {
        if (!enabled || mode != TapeTrackingMode.FIXED_HEADING) return false
        fixedHeadingLapController.beginFrameStreamRecovery(nowNanos)
        phase = TapeTrackingPhase.RECOVERING_FRAME_STREAM
        return true
    }


    fun observe(observation: TapeTrackingObservation?, nowNanos: Long) {
        if (!enabled || phase == TapeTrackingPhase.TURNING) return
        if (mode == TapeTrackingMode.FIXED_HEADING) {
            fixedHeadingLapController.observe(
                centerline = observation?.centerline,
                heightMeters = observation?.heightAboveGroundMeters,
                confidence = observation?.confidence ?: 0.0,
                nowNanos = nowNanos,
                capturedAtNanos = observation?.capturedAtNanos ?: 0L,
            )
            return
        }
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
                endpointMissSinceNanos = 0L
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
            TapeTrackingPhase.RECOVERING_FRAME_STREAM,
            TapeTrackingPhase.REACQUIRING_PATH,
            TapeTrackingPhase.TURNING,
            -> Unit
        }
    }

    fun tick(nowNanos: Long): TapeTrackingDecision {
        if (mode == TapeTrackingMode.FIXED_HEADING) {
            val fixedDecision = fixedHeadingLapController.tick(nowNanos)
            phase = when (fixedDecision.phase) {
                FixedHeadingLapPhase.DISABLED, FixedHeadingLapPhase.STOPPED ->
                    TapeTrackingPhase.DISABLED
                FixedHeadingLapPhase.ACQUIRING -> TapeTrackingPhase.RECENTERING
                FixedHeadingLapPhase.RECOVERING_FRAME_STREAM ->
                    TapeTrackingPhase.RECOVERING_FRAME_STREAM
                FixedHeadingLapPhase.TRACKING, FixedHeadingLapPhase.COASTING ->
                    TapeTrackingPhase.TRACKING
            }
            return TapeTrackingDecision(
                phase = phase,
                yawRateDegreesPerSecond = 0.0,
                forwardSpeedMetersPerSecond = fixedDecision.forwardMetersPerSecond,
                rightSpeedMetersPerSecond = fixedDecision.rightMetersPerSecond,
                stopRequested = fixedDecision.stopRequested,
                endpointReached = fixedDecision.endpointReached,
                rawAngleDegrees = fixedDecision.tangentDegrees,
                controlledAngleDegrees = fixedDecision.virtualHeadingDegrees,
                rawOffsetFraction = fixedDecision.lateralOffsetMeters,
                purePursuitYawRateDegreesPerSecond =
                    fixedDecision.curvaturePerMeter?.let {
                        Math.toDegrees(
                            hypot(
                                fixedDecision.forwardMetersPerSecond,
                                fixedDecision.rightMetersPerSecond,
                            ) * it,
                        )
                    } ?: 0.0,
                pathQuality =
                    if (fixedDecision.tangentDegrees == null) PathQuality.LOST else PathQuality.FULL_PATH,
            )
        }
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
            appliedForwardAccelerationMetersPerSecondSquared = 0.0
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
            appliedForwardAccelerationMetersPerSecondSquared = 0.0
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
        profileForwardSpeedMetersPerSecond = desiredForwardSpeed(nowNanos)
        val pathBearingDegrees = desiredLookaheadBearingDegrees(nowNanos)
        val purePursuitCurvature = desiredPurePursuitCurvaturePerMeter(nowNanos)
        val yawLimitedTargetSpeed =
            if (mode == TapeTrackingMode.CIRCULAR && purePursuitCurvature != null) {
                profileForwardSpeedMetersPerSecond
            } else {
                limitForwardSpeedForYaw(profileForwardSpeedMetersPerSecond, purePursuitCurvature)
            }
        val ordinaryTargetSpeed = applyCurvatureSpeedCaps(yawLimitedTargetSpeed)
        accelerationLimitedForwardSpeedMetersPerSecond =
            applyForwardAccelerationLimit(ordinaryTargetSpeed, nowNanos)
        val forwardSpeed =
            applyEmergencyCurvatureSpeedCap(accelerationLimitedForwardSpeedMetersPerSecond)
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
            pathBearingDegrees = pathBearingDegrees,
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
            curveMisalignmentSinceNanos = 0L
            circularDetectionGap = true
            disableCircularLateralCorrection()
            if (!reliableCircularPathEstablished) {
                reliableCircularPathSinceNanos = 0L
            }
            if (phase == TapeTrackingPhase.REACQUIRING_PATH) {
                expireCircularReacquisitionCandidateIfStale(nowNanos)
            } else {
                resetCircularReacquisitionCandidate()
            }
            when (phase) {
                TapeTrackingPhase.TRACKING -> {
                    if (reliableCircularPathEstablished) {
                        if (endpointMissSinceNanos == 0L) {
                            endpointMissSinceNanos = nowNanos
                        }
                        if (
                            nowNanos - endpointMissSinceNanos >=
                            CIRCULAR_ENDPOINT_ENTRY_DURATION_NANOS
                        ) {
                            beginCircularEndpointVerification(nowNanos)
                        }
                    }
                }

                TapeTrackingPhase.ALIGNING_CURVE -> {
                    curveAlignmentSinceNanos = 0L
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
        updateCircularTrackingPhase(nowNanos)
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
        val consistentDurationNanos =
            recordConsistentCircularCandidate(trackedObservation, nowNanos)
        if (consistentDurationNanos < CIRCULAR_POST_TURN_CONFIRMATION_NANOS) {
            clearControlMeasurements()
            return
        }

        awaitingPostTurnDetection = false
        phase = TapeTrackingPhase.TRACKING
        resetCircularReacquisitionCandidate()
        acceptCircularObservation(trackedObservation, nowNanos)
        resetAppliedCommands()
        updateCircularTrackingPhase(nowNanos)
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
        endpointMissSinceNanos = 0L
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
            updateCircularTrackingPhase(nowNanos)
        }
    }

    private fun beginCircularEndpointVerification(nowNanos: Long) {
        phase = TapeTrackingPhase.VERIFYING_ENDPOINT
        endpointReferenceBounds = lastAcceptedCircularObservation?.bounds
        endpointVerificationStartedAtNanos = nowNanos
        consecutiveEndpointMisses = 0
        endpointMissSinceNanos = 0L
        reliableCircularPathSinceNanos = 0L
        reliableCircularPathEstablished = false
        clearControlMeasurements()
        resetAppliedCommands()
    }

    private fun observeCircularGapContinuation(
        observation: TapeTrackingObservation,
        nowNanos: Long,
    ) {
        val consistentDurationNanos =
            recordConsistentCircularCandidate(observation, nowNanos)
        val commandStillFresh =
            nowNanos - lastDetectionAtNanos <= DETECTION_COMMAND_STALE_NANOS
        if (!commandStillFresh) {
            clearControlMeasurements()
            resetAppliedCommands()
        }
        if (consistentDurationNanos < CIRCULAR_GAP_CONTINUATION_CONFIRMATION_NANOS) {
            return
        }
        resetCircularReacquisitionCandidate()
        acceptCircularObservation(observation, nowNanos)
        updateCircularTrackingPhase(nowNanos)
    }

    private fun observeCircularReacquisitionCandidate(
        observation: TapeTrackingObservation,
        nowNanos: Long,
    ) {
        expireCircularReacquisitionCandidateIfStale(nowNanos)
        if (!isCredibleCircularReacquisitionPath(observation)) {
            return
        }

        val consistentDurationNanos =
            recordConsistentCircularCandidate(observation, nowNanos)
        if (consistentDurationNanos >= REACQUISITION_CONFIRMATION_NANOS) {
            completeCircularReacquisition(observation, nowNanos)
        }
    }

    private fun beginCircularReacquisition() {
        phase = TapeTrackingPhase.REACQUIRING_PATH
        curveAlignmentSinceNanos = 0L
        reliableCircularPathSinceNanos = 0L
        reliableCircularPathEstablished = false
        consecutiveEndpointMisses = 0
        endpointMissSinceNanos = 0L
        resetCircularReacquisitionCandidate()
        clearControlMeasurements()
        resetAppliedCommands()
        curveMisalignmentSinceNanos = 0L
    }

    private fun completeCircularReacquisition(
        observation: TapeTrackingObservation,
        nowNanos: Long,
    ) {
        phase = TapeTrackingPhase.TRACKING
        resetCircularReacquisitionCandidate()
        acceptCircularObservation(observation, nowNanos)
        resetAppliedCommands()
        updateCircularTrackingPhase(nowNanos)
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
        endpointMissSinceNanos = 0L
        if (!endpointTurnEnabled) {
            reliableCircularPathSinceNanos = 0L
            reliableCircularPathEstablished = false
            return
        }
        val reliablePath =
            observation.quality == PathQuality.FULL_PATH &&
                !observation.closedLoop &&
                observation.longSideFraction >= CIRCULAR_RELIABLE_PATH_MIN_FRACTION &&
                observation.bounds.bottom >= ENDPOINT_NEAR_EDGE_MIN_FRACTION
        if (reliablePath) {
            if (reliableCircularPathSinceNanos == 0L) {
                reliableCircularPathSinceNanos = nowNanos
            }
            if (
                nowNanos - reliableCircularPathSinceNanos >=
                CIRCULAR_RELIABLE_PATH_CONFIRMATION_NANOS
            ) {
                reliableCircularPathEstablished = true
            }
        } else if (!reliableCircularPathEstablished) {
            reliableCircularPathSinceNanos = 0L
        }
    }

    private fun updateCircularTrackingPhase(nowNanos: Long) {
        val angle = controlledAngleDegrees ?: return
        when (phase) {
            TapeTrackingPhase.TRACKING -> {
                val requiresInPlaceAlignment =
                    abs(angle) >= CURVE_ALIGNMENT_ENTER_ANGLE_DEGREES &&
                        !hasUsablePurePursuitTarget(nowNanos)
                if (!requiresInPlaceAlignment) {
                    curveMisalignmentSinceNanos = 0L
                } else if (curveMisalignmentSinceNanos == 0L) {
                    curveMisalignmentSinceNanos = nowNanos
                } else if (
                    nowNanos - curveMisalignmentSinceNanos >=
                    CURVE_MISALIGNMENT_CONFIRMATION_NANOS
                ) {
                    phase = TapeTrackingPhase.ALIGNING_CURVE
                    curveMisalignmentSinceNanos = 0L
                    curveAlignmentSinceNanos = 0L
                    disableCircularLateralCorrection()
                    resetAppliedCommands()
                }
            }

            TapeTrackingPhase.ALIGNING_CURVE -> {
                if (abs(angle) <= CURVE_ALIGNMENT_EXIT_ANGLE_DEGREES) {
                    if (curveAlignmentSinceNanos == 0L) {
                        curveAlignmentSinceNanos = nowNanos
                    }
                    if (
                        nowNanos - curveAlignmentSinceNanos >=
                        CURVE_ALIGNMENT_CONFIRMATION_NANOS
                    ) {
                        phase = TapeTrackingPhase.TRACKING
                        curveAlignmentSinceNanos = 0L
                        disableCircularLateralCorrection()
                        resetAppliedCommands()
                    }
                } else {
                    curveAlignmentSinceNanos = 0L
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

    private fun recordConsistentCircularCandidate(
        observation: TapeTrackingObservation,
        nowNanos: Long,
    ): Long {
        val previousCandidate = circularReacquisitionCandidate
        if (
            previousCandidate == null ||
            !isConsistentCircularReacquisition(observation, previousCandidate)
        ) {
            circularCandidateSequenceSinceNanos = nowNanos
        }
        circularReacquisitionCandidate = observation
        circularReacquisitionCandidateAtNanos = nowNanos
        return nowNanos - circularCandidateSequenceSinceNanos
    }

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
        circularCandidateSequenceSinceNanos = 0L
    }


    private fun beginEndpointVerificationIfReady(nowNanos: Long) {
        if (
            phase != TapeTrackingPhase.TRACKING ||
            endpointCandidateSinceNanos == 0L ||
            consecutiveEndpointDetections < ENDPOINT_MIN_CONFIRMATION_SAMPLES ||
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
        endpointMissSinceNanos = 0L
        resetAppliedCommands()
    }

    private fun registerEndpointMiss(nowNanos: Long) {
        consecutiveEndpointMisses += 1
        if (endpointMissSinceNanos == 0L) {
            endpointMissSinceNanos = nowNanos
        }
        if (
            consecutiveEndpointMisses >= ENDPOINT_MIN_DISAPPEARANCE_SAMPLES &&
            nowNanos - endpointMissSinceNanos >= ENDPOINT_DISAPPEARANCE_CONFIRMATION_NANOS &&
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
        endpointMissSinceNanos = 0L
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
        val sampleAtNanos = observation.capturedAtNanos.takeIf { it > 0L } ?: nowNanos
        val elapsedNanos = sampleAtNanos - measurementCapturedAtNanos
        val elapsedSeconds = elapsedNanos / NANOS_PER_SECOND
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
            rawLookaheadXFraction = null
            rawLookaheadYFraction = null
            controlledLookaheadXFraction = null
            controlledLookaheadYFraction = null
            lookaheadXRatePerSecond = 0.0
            lookaheadYRatePerSecond = 0.0
        } else {
            val previousRawX = rawLookaheadXFraction
            val previousRawY = rawLookaheadYFraction
            if (
                previousRawX != null &&
                previousRawY != null &&
                measurementCapturedAtNanos != 0L &&
                elapsedSeconds > 0.0
            ) {
                val measuredXRate = (
                    (lookahead.xFraction - previousRawX) / elapsedSeconds
                    ).coerceIn(-MAX_LOOKAHEAD_RATE_PER_SECOND, MAX_LOOKAHEAD_RATE_PER_SECOND)
                val measuredYRate = (
                    (lookahead.yFraction - previousRawY) / elapsedSeconds
                    ).coerceIn(-MAX_LOOKAHEAD_RATE_PER_SECOND, MAX_LOOKAHEAD_RATE_PER_SECOND)
                lookaheadXRatePerSecond = exponentialAverage(
                    lookaheadXRatePerSecond,
                    measuredXRate,
                    LOOKAHEAD_RATE_FILTER_ALPHA,
                )
                lookaheadYRatePerSecond = exponentialAverage(
                    lookaheadYRatePerSecond,
                    measuredYRate,
                    LOOKAHEAD_RATE_FILTER_ALPHA,
                )
            } else {
                lookaheadXRatePerSecond = 0.0
                lookaheadYRatePerSecond = 0.0
            }
            rawLookaheadXFraction = lookahead.xFraction
            rawLookaheadYFraction = lookahead.yFraction
            if (mode == TapeTrackingMode.CIRCULAR && circularTrackingSpeed.latencyCompensatedLookahead) {
                // Latency-sensitive profiles use the current target plus bounded motion
                // prediction instead of aiming at an EMA-delayed pixel.
                controlledLookaheadXFraction = lookahead.xFraction
                controlledLookaheadYFraction = lookahead.yFraction
            } else {
                controlledLookaheadXFraction = controlledLookaheadXFraction?.let {
                    exponentialAverage(it, lookahead.xFraction, STABILIZED_FILTER_ALPHA)
                } ?: lookahead.xFraction
                controlledLookaheadYFraction = controlledLookaheadYFraction?.let {
                    exponentialAverage(it, lookahead.yFraction, STABILIZED_FILTER_ALPHA)
                } ?: lookahead.yFraction
            }
        }
        lookaheadFrameAspectRatio =
            observation.frameWidthPixels.toDouble() / observation.frameHeightPixels
        heightAboveGroundMeters = observation.heightAboveGroundMeters
        measuredOffsetRatePerSecond =
            if (previousOffset != null && measurementCapturedAtNanos != 0L && elapsedSeconds > 0.0) {
                ((nextOffset - previousOffset) / elapsedSeconds)
                    .coerceIn(-MAX_OFFSET_RATE_PER_SECOND, MAX_OFFSET_RATE_PER_SECOND)
            } else {
                0.0
            }
        offsetRatePerSecond =
            if (previousOffset != null && measurementCapturedAtNanos != 0L && elapsedSeconds > 0.0) {
                exponentialAverage(
                    offsetRatePerSecond,
                    measuredOffsetRatePerSecond,
                    OFFSET_RATE_FILTER_ALPHA,
                )
            } else {
                0.0
            }
        controlledAngleDegrees = nextAngle
        controlledHorizontalOffsetFraction = nextOffset
        measurementCapturedAtNanos = sampleAtNanos
        lastControlObservationAtNanos = nowNanos
        rawCurvaturePerMeter = lookahead?.let {
            curvaturePerMeter(
                xFraction = it.xFraction,
                yFraction = it.yFraction,
                aspectRatio = checkNotNull(lookaheadFrameAspectRatio),
                heightMeters = heightAboveGroundMeters,
            )
        }
        predictedCurvaturePerMeter = desiredPurePursuitCurvaturePerMeter(nowNanos)
        instantaneousCurvatureSpeedCapMetersPerSecond =
            if (mode == TapeTrackingMode.CIRCULAR) {
                predictedCurvaturePerMeter?.let(::curvatureSpeedCapMetersPerSecond)
            } else {
                null
            }
        if (mode == TapeTrackingMode.CIRCULAR && predictedCurvaturePerMeter != null) {
            recordCurvatureSample(
                curvaturePerMeter = checkNotNull(predictedCurvaturePerMeter),
                sampleAtNanos = sampleAtNanos,
            )
        } else if (mode == TapeTrackingMode.CIRCULAR) {
            resetTrustedCurvatureState()
        }
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
        rawLookaheadXFraction = null
        rawLookaheadYFraction = null
        lookaheadXRatePerSecond = 0.0
        lookaheadYRatePerSecond = 0.0
        measurementCapturedAtNanos = 0L
        lookaheadFrameAspectRatio = null
        heightAboveGroundMeters = null
        measuredOffsetRatePerSecond = 0.0
        offsetRatePerSecond = 0.0
        rawCurvaturePerMeter = null
        predictedCurvaturePerMeter = null
        resetTrustedCurvatureState()
        lastControlObservationAtNanos = 0L
        disableCircularLateralCorrection()
        pathQuality = PathQuality.LOST
    }

    private fun resetTrustedCurvatureState() {
        trustedCurvaturePerMeter = null
        instantaneousCurvatureSpeedCapMetersPerSecond = null
        trustedCurvatureSpeedCapMetersPerSecond =
            CIRCULAR_CORRECTION_FORWARD_SPEED_METERS_PER_SECOND
        curvatureSampleCount = 0
        curvatureSampleWriteIndex = 0
        lastCurvatureSampleAtNanos = 0L
    }

    private fun resetControlState() {
        curveAlignmentSinceNanos = 0L
        curveMisalignmentSinceNanos = 0L
        lastAcceptedCircularObservation = null
        circularDetectionGap = false
        resetCircularReacquisitionCandidate()
        reliableCircularPathSinceNanos = 0L
        reliableCircularPathEstablished = false
        clearControlMeasurements()
        resetAppliedCommands()
    }

    private fun resetAppliedCommands() {
        appliedYawRateDegreesPerSecond = 0.0
        appliedRightSpeedMetersPerSecond = 0.0
        appliedForwardSpeedMetersPerSecond = 0.0
        appliedForwardAccelerationMetersPerSecondSquared = 0.0
        profileForwardSpeedMetersPerSecond = 0.0
        accelerationLimitedForwardSpeedMetersPerSecond = 0.0
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
            mode == TapeTrackingMode.CIRCULAR ->
                CIRCULAR_FAST_MAX_YAW_RATE_DEGREES_PER_SECOND
            mode.followsCurvedPath ->
                CIRCULAR_MAX_YAW_RATE_DEGREES_PER_SECOND
            else -> MAX_TRACKING_YAW_RATE_DEGREES_PER_SECOND
        }
        val maximumYawRate =
            if (pathQuality == PathQuality.NEAR_FIELD_ONLY) {
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
        val maximumYawRate = when {
            pathQuality == PathQuality.NEAR_FIELD_ONLY ->
                NEAR_FIELD_MAX_YAW_RATE_DEGREES_PER_SECOND
            mode == TapeTrackingMode.CIRCULAR ->
                CIRCULAR_FAST_MAX_YAW_RATE_DEGREES_PER_SECOND
            else ->
                CIRCULAR_MAX_YAW_RATE_DEGREES_PER_SECOND
        }
        return (angle * CIRCULAR_YAW_PROPORTIONAL_GAIN).coerceIn(
            -maximumYawRate,
            maximumYawRate,
        )
    }

    private fun predictedLookahead(nowNanos: Long): PredictedLookahead? {
        val lookaheadX = controlledLookaheadXFraction ?: return null
        val lookaheadY = controlledLookaheadYFraction ?: return null
        if (
            mode != TapeTrackingMode.CIRCULAR ||
            measurementCapturedAtNanos == 0L
        ) {
            return PredictedLookahead(lookaheadX, lookaheadY)
        }
        val measurementAgeSeconds = (
            (nowNanos - measurementCapturedAtNanos).coerceAtLeast(0L) / NANOS_PER_SECOND
            ).coerceAtMost(MAX_MEASUREMENT_AGE_COMPENSATION_SECONDS)
        val predictionSeconds = measurementAgeSeconds + COMMAND_RESPONSE_PREVIEW_SECONDS
        val predictedX = lookaheadX + (
            lookaheadXRatePerSecond * predictionSeconds
            ).coerceIn(-MAX_LOOKAHEAD_PREDICTION_FRACTION, MAX_LOOKAHEAD_PREDICTION_FRACTION)
        val predictedY = lookaheadY + (
            lookaheadYRatePerSecond * predictionSeconds
            ).coerceIn(-MAX_LOOKAHEAD_PREDICTION_FRACTION, MAX_LOOKAHEAD_PREDICTION_FRACTION)
        return PredictedLookahead(
            xFraction = predictedX.coerceIn(0.0, 1.0),
            yFraction = predictedY.coerceIn(0.0, 1.0),
        )
    }

    /**
     * Bearing from the aircraft reference to the predicted point ahead on the path.
     * Unlike the undirected near-field tape tangent, this preserves which side the
     * upcoming route actually occupies.
     */
    private fun desiredLookaheadBearingDegrees(nowNanos: Long): Double? {
        if (pathQuality != PathQuality.FULL_PATH) return null
        val lookahead = predictedLookahead(nowNanos) ?: return null
        val aspectRatio = lookaheadFrameAspectRatio ?: return null
        val lateralDistance =
            (lookahead.xFraction - TRACKING_TARGET_X_FRACTION) * aspectRatio
        val forwardDistance = TRACKING_TARGET_Y_FRACTION - lookahead.yFraction
        if (forwardDistance <= 0.0) return null
        return Math.toDegrees(atan2(lateralDistance, forwardDistance))
    }

    private fun hasUsablePurePursuitTarget(nowNanos: Long): Boolean =
        desiredPurePursuitCurvaturePerMeter(nowNanos) != null

    /**
     * Ground-plane Pure Pursuit curvature from the detected target point. Circular
     * mode projects the target through detector and command latency.
     */
    private fun desiredPurePursuitCurvaturePerMeter(nowNanos: Long): Double? {
        if (pathQuality != PathQuality.FULL_PATH) return null
        val lookahead = predictedLookahead(nowNanos) ?: return null
        val aspectRatio = lookaheadFrameAspectRatio ?: return null
        return curvaturePerMeter(
            xFraction = lookahead.xFraction,
            yFraction = lookahead.yFraction,
            aspectRatio = aspectRatio,
            heightMeters = heightAboveGroundMeters,
        )
    }

    private fun curvaturePerMeter(
        xFraction: Double,
        yFraction: Double,
        aspectRatio: Double,
        heightMeters: Double?,
    ): Double? {
        val height = heightMeters?.takeIf {
            it >= PURE_PURSUIT_MIN_HEIGHT_METERS
        } ?: return null
        val normalizedLateralDistance =
            (xFraction - TRACKING_TARGET_X_FRACTION) * aspectRatio
        val normalizedForwardDistance = TRACKING_TARGET_Y_FRACTION - yFraction
        if (normalizedForwardDistance <= 0.0) return null
        val verticalHalfFovTangent =
            CAMERA_DIAGONAL_HALF_FOV_TANGENT / sqrt(1.0 + aspectRatio * aspectRatio)
        val groundFrameHeightMeters = 2.0 * height * verticalHalfFovTangent
        val lateralMeters = normalizedLateralDistance * groundFrameHeightMeters
        val forwardMeters = normalizedForwardDistance * groundFrameHeightMeters
        val lookaheadDistanceMeters = hypot(lateralMeters, forwardMeters)
        if (lookaheadDistanceMeters < PURE_PURSUIT_MIN_LOOKAHEAD_METERS) return null
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
        val yawBudgetDegreesPerSecond =
            if (mode == TapeTrackingMode.CIRCULAR) {
                CIRCULAR_FAST_YAW_SPEED_BUDGET_DEGREES_PER_SECOND
            } else {
                CIRCULAR_MAX_YAW_RATE_DEGREES_PER_SECOND -
                    CIRCULAR_YAW_CONTROL_RESERVE_DEGREES_PER_SECOND
            }
        val maximumForwardSpeed =
            Math.toRadians(yawBudgetDegreesPerSecond) / abs(curvaturePerMeter)
        return minOf(profileForwardSpeedMetersPerSecond, maximumForwardSpeed)
    }

    private fun recordCurvatureSample(
        curvaturePerMeter: Double,
        sampleAtNanos: Long,
    ) {
        if (sampleAtNanos <= lastCurvatureSampleAtNanos) return
        val elapsedSeconds =
            if (lastCurvatureSampleAtNanos == 0L) {
                0.0
            } else {
                ((sampleAtNanos - lastCurvatureSampleAtNanos) / NANOS_PER_SECOND)
                    .coerceAtMost(MAX_CURVATURE_CAP_UPDATE_INTERVAL_SECONDS)
            }
        lastCurvatureSampleAtNanos = sampleAtNanos
        curvatureSamplesPerMeter[curvatureSampleWriteIndex] = curvaturePerMeter
        curvatureSampleWriteIndex =
            (curvatureSampleWriteIndex + 1) % CURVATURE_MEDIAN_SAMPLE_COUNT
        curvatureSampleCount =
            minOf(curvatureSampleCount + 1, CURVATURE_MEDIAN_SAMPLE_COUNT)
        if (curvatureSampleCount < CURVATURE_MEDIAN_SAMPLE_COUNT) return

        val first = curvatureSamplesPerMeter[0]
        val second = curvatureSamplesPerMeter[1]
        val third = curvatureSamplesPerMeter[2]
        val median = first + second + third -
            minOf(first, second, third) -
            maxOf(first, second, third)
        trustedCurvaturePerMeter = median
        val trustedTargetCap = curvatureSpeedCapMetersPerSecond(median)
        trustedCurvatureSpeedCapMetersPerSecond =
            if (trustedTargetCap <= trustedCurvatureSpeedCapMetersPerSecond) {
                trustedTargetCap
            } else {
                moveToward(
                    trustedCurvatureSpeedCapMetersPerSecond,
                    trustedTargetCap,
                    CIRCULAR_CURVATURE_CAP_RISE_METERS_PER_SECOND_SQUARED * elapsedSeconds,
                )
            }
    }

    private fun curvatureSpeedCapMetersPerSecond(curvaturePerMeter: Double): Double {
        val targetSpeed = circularTrackingSpeed.targetMetersPerSecond
        val magnitude = abs(curvaturePerMeter)
        if (magnitude <= PURE_PURSUIT_STRAIGHT_CURVATURE_PER_METER) return targetSpeed
        return minOf(
            targetSpeed,
            Math.toRadians(CIRCULAR_FAST_YAW_SPEED_BUDGET_DEGREES_PER_SECOND) / magnitude,
        )
    }

    private fun applyCurvatureSpeedCaps(speedMetersPerSecond: Double): Double {
        if (
            mode != TapeTrackingMode.CIRCULAR ||
            (curvatureSampleCount == 0 && trustedCurvaturePerMeter == null)
        ) {
            return speedMetersPerSecond
        }
        return minOf(
            speedMetersPerSecond,
            trustedCurvatureSpeedCapMetersPerSecond,
            instantaneousCurvatureSpeedCapMetersPerSecond ?: speedMetersPerSecond,
        )
    }

    /**
     * Ordinary curve planning is jerk-limited before reaching this guard. A late
     * curvature discovery may still require an immediate clamp so commanded
     * speed never exceeds the yaw authority proven by the current image.
     */
    private fun applyEmergencyCurvatureSpeedCap(speedMetersPerSecond: Double): Double {
        val safeSpeed = applyCurvatureSpeedCaps(speedMetersPerSecond)
        if (safeSpeed < speedMetersPerSecond) {
            appliedForwardSpeedMetersPerSecond = safeSpeed
            appliedForwardAccelerationMetersPerSecondSquared = 0.0
        }
        return safeSpeed
    }

    private fun applyForwardAccelerationLimit(
        targetForwardSpeedMetersPerSecond: Double,
        nowNanos: Long,
    ): Double {
        if (!mode.followsCurvedPath) {
            appliedForwardSpeedMetersPerSecond = targetForwardSpeedMetersPerSecond
            return appliedForwardSpeedMetersPerSecond
        }
        val elapsedSeconds = outputIntervalSeconds(nowNanos)
        if (mode != TapeTrackingMode.CIRCULAR) {
            val maximumRate =
                if (targetForwardSpeedMetersPerSecond >= appliedForwardSpeedMetersPerSecond) {
                    MAX_FORWARD_ACCELERATION_METERS_PER_SECOND_SQUARED
                } else {
                    CIRCULAR_MAX_FORWARD_DECELERATION_METERS_PER_SECOND_SQUARED
                }
            appliedForwardSpeedMetersPerSecond = moveToward(
                appliedForwardSpeedMetersPerSecond,
                targetForwardSpeedMetersPerSecond,
                maximumRate * elapsedSeconds,
            )
            return appliedForwardSpeedMetersPerSecond
        }
        if (
            targetForwardSpeedMetersPerSecond <= 0.0 &&
            controlledHorizontalOffsetFraction?.let {
                projectedCrossTrackMagnitude(it) >= CIRCULAR_FORWARD_STOP_OFFSET_FRACTION
            } == true
        ) {
            appliedForwardSpeedMetersPerSecond = 0.0
            appliedForwardAccelerationMetersPerSecondSquared = 0.0
            return 0.0
        }
        if (
            targetForwardSpeedMetersPerSecond < appliedForwardSpeedMetersPerSecond &&
            (circularDetectionGap || pathQuality == PathQuality.LOST)
        ) {
            appliedForwardSpeedMetersPerSecond = targetForwardSpeedMetersPerSecond
            appliedForwardAccelerationMetersPerSecondSquared = 0.0
            return appliedForwardSpeedMetersPerSecond
        }

        val speedError =
            targetForwardSpeedMetersPerSecond - appliedForwardSpeedMetersPerSecond
        val desiredAcceleration = (
            speedError * CIRCULAR_SPEED_ERROR_RESPONSE_PER_SECOND
            ).coerceIn(
            -CIRCULAR_MAX_FORWARD_DECELERATION_METERS_PER_SECOND_SQUARED,
            CIRCULAR_FAST_MAX_FORWARD_ACCELERATION_METERS_PER_SECOND_SQUARED,
        )
        appliedForwardAccelerationMetersPerSecondSquared = moveToward(
            appliedForwardAccelerationMetersPerSecondSquared,
            desiredAcceleration,
            CIRCULAR_MAX_FORWARD_JERK_METERS_PER_SECOND_CUBED * elapsedSeconds,
        )
        val nextSpeed = (
            appliedForwardSpeedMetersPerSecond +
                appliedForwardAccelerationMetersPerSecondSquared * elapsedSeconds
            ).coerceAtLeast(0.0)
        val reachesTarget =
            abs(speedError) <= CIRCULAR_SPEED_SETTLED_TOLERANCE_METERS_PER_SECOND ||
                speedError > 0.0 && nextSpeed >= targetForwardSpeedMetersPerSecond ||
                speedError < 0.0 && nextSpeed <= targetForwardSpeedMetersPerSecond
        if (reachesTarget) {
            appliedForwardSpeedMetersPerSecond = targetForwardSpeedMetersPerSecond
            appliedForwardAccelerationMetersPerSecondSquared = 0.0
        } else {
            appliedForwardSpeedMetersPerSecond = nextSpeed
        }
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
            if (phase != TapeTrackingPhase.TRACKING || circularDetectionGap) return 0.0
            val effectiveOffset = when {
                offset > CIRCULAR_CENTERING_DEAD_ZONE_FRACTION ->
                    offset - CIRCULAR_CENTERING_DEAD_ZONE_FRACTION
                offset < -CIRCULAR_CENTERING_DEAD_ZONE_FRACTION ->
                    offset + CIRCULAR_CENTERING_DEAD_ZONE_FRACTION
                else -> 0.0
            }
            return (
                effectiveOffset * CIRCULAR_LATERAL_PROPORTIONAL_GAIN +
                    offsetRatePerSecond * CIRCULAR_LATERAL_DERIVATIVE_GAIN
                ).coerceIn(
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
            val hasUsableLookahead = hasUsablePurePursuitTarget(nowNanos)
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
            val trackingSpeed =
                if (mode == TapeTrackingMode.CIRCULAR) {
                    circularTrackingSpeed.targetMetersPerSecond
                } else {
                    CIRCULAR_TRACKING_FORWARD_SPEED_METERS_PER_SECOND
                }
            // Both complete-lap profiles use the same cross-track safety gate.
            // A low current offset cannot cancel a high crossing rate and reopen
            // full speed while the aircraft is sweeping through the centerline.
            if (mode == TapeTrackingMode.CIRCULAR && hasUsableLookahead) {
                return circularSpeedForProjectedOffset(trackingSpeed, offset)
            }
            return if (
                abs(angle) <= CIRCULAR_STABLE_ANGLE_DEGREES &&
                abs(offset) <= CENTERING_START_FRACTION
            ) {
                trackingSpeed
            } else {
                minOf(trackingSpeed, CIRCULAR_CORRECTION_FORWARD_SPEED_METERS_PER_SECOND)
            }
        }

        if (phase != TapeTrackingPhase.TRACKING || endpointCandidateSinceNanos != 0L) return 0.0
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
    private fun circularSpeedForProjectedOffset(
        trackingSpeedMetersPerSecond: Double,
        offsetFraction: Double,
    ): Double {
        val projectedMagnitude = projectedCrossTrackMagnitude(offsetFraction)
        val forwardAuthority = (
            (CIRCULAR_FORWARD_STOP_OFFSET_FRACTION - projectedMagnitude) /
                (
                    CIRCULAR_FORWARD_STOP_OFFSET_FRACTION -
                        CIRCULAR_FULL_SPEED_OFFSET_FRACTION
                    )
            ).coerceIn(0.0, 1.0)
        return trackingSpeedMetersPerSecond * forwardAuthority
    }

    private fun projectedCrossTrackMagnitude(offsetFraction: Double): Double {
        val projectedDelta =
            offsetRatePerSecond * CROSS_TRACK_RESPONSE_PREVIEW_SECONDS
        return maxOf(
            abs(offsetFraction),
            abs(projectedDelta),
            abs(offsetFraction + projectedDelta),
        )
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
        boundedControlIntervalSeconds(
            nowNanos = nowNanos,
            previousNanos = lastCommandAtNanos,
            initialSeconds = INITIAL_COMMAND_INTERVAL_SECONDS,
            maximumSeconds = MAX_COMMAND_INTERVAL_SECONDS,
        )




    private fun decision(
        phase: TapeTrackingPhase,
        yawRateDegreesPerSecond: Double,
        forwardSpeedMetersPerSecond: Double = 0.0,
        rightSpeedMetersPerSecond: Double = 0.0,
        endpointReached: Boolean = false,
        stopRequested: Boolean = false,
        purePursuitYawRateDegreesPerSecond: Double = 0.0,
        pathBearingDegrees: Double? = null,
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
            pathBearingDegrees = pathBearingDegrees,
            rawCurvaturePerMeter = rawCurvaturePerMeter,
            predictedCurvaturePerMeter = predictedCurvaturePerMeter,
            trustedCurvaturePerMeter = trustedCurvaturePerMeter,
            instantaneousCurvatureSpeedCapMetersPerSecond =
                instantaneousCurvatureSpeedCapMetersPerSecond,
            trustedCurvatureSpeedCapMetersPerSecond =
                if (mode == TapeTrackingMode.CIRCULAR) {
                    trustedCurvatureSpeedCapMetersPerSecond
                } else {
                    null
                },
            profileForwardSpeedMetersPerSecond = profileForwardSpeedMetersPerSecond,
            accelerationLimitedForwardSpeedMetersPerSecond =
                accelerationLimitedForwardSpeedMetersPerSecond,
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
        const val OFFSET_RATE_FILTER_ALPHA = 0.35
        const val MAX_YAW_ACCELERATION_DEGREES_PER_SECOND_SQUARED = 30.0
        const val MAX_FORWARD_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.10
        const val MAX_LATERAL_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.20
        const val ENDPOINT_REFERENCE_MIN_FRACTION = 0.60
        const val ENDPOINT_LENGTH_RATIO = 0.75
        const val ENDPOINT_REARM_LENGTH_RATIO = 0.65
        const val ENDPOINT_EXIT_LENGTH_RATIO = 0.85
        // Duration is the primary debounce at 10 Hz; three samples prevent sparse
        // callbacks from satisfying it with only one or two observations.
        const val ENDPOINT_MIN_CONFIRMATION_SAMPLES = 3
        const val ENDPOINT_MIN_DISAPPEARANCE_SAMPLES = 3
        const val ENDPOINT_CONFIRMATION_NANOS = 750_000_000L
        const val ENDPOINT_DISAPPEARANCE_CONFIRMATION_NANOS = 500_000_000L
        const val ENDPOINT_PROBE_SPEED_METERS_PER_SECOND = 0.10
        const val ENDPOINT_PROBE_DURATION_NANOS = 1_500_000_000L
        const val RECENTER_DURATION_NANOS = 2_000_000_000L
        const val POST_TURN_RECOVERY_SPEED_METERS_PER_SECOND = 0.15
        const val POST_TURN_RECOVERY_NANOS = 2_000_000_000L
        // The detector ran at about 3.8 Hz in the successful flight. Keep four missed
        // sample windows plus callback jitter before hovering.
        const val DETECTION_COMMAND_STALE_NANOS = 1_250_000_000L
        const val ENDPOINT_NEAR_EDGE_MIN_FRACTION = 0.90
        const val ENDPOINT_TRACK_MIN_OVERLAP = 0.20
        const val ENDPOINT_MAX_YAW_RATE_DEGREES_PER_SECOND = 2.0
        const val ENDPOINT_MAX_CENTERING_SPEED_METERS_PER_SECOND = 0.03

        /** Bounded in-place alignment allowed when only the near field is credible. */
        const val NEAR_FIELD_MAX_YAW_RATE_DEGREES_PER_SECOND = 3.0
        const val CIRCULAR_YAW_DEAD_ZONE_DEGREES = 1.5
        const val CIRCULAR_YAW_PROPORTIONAL_GAIN = 0.70
        const val CIRCULAR_MAX_YAW_RATE_DEGREES_PER_SECOND = 30.0
        const val CIRCULAR_YAW_CONTROL_RESERVE_DEGREES_PER_SECOND = 2.0
        const val CIRCULAR_FAST_MAX_YAW_RATE_DEGREES_PER_SECOND = 50.0
        const val CIRCULAR_FAST_YAW_CONTROL_RESERVE_DEGREES_PER_SECOND = 5.0
        const val CIRCULAR_FAST_YAW_SPEED_BUDGET_DEGREES_PER_SECOND =
            CIRCULAR_FAST_MAX_YAW_RATE_DEGREES_PER_SECOND -
                CIRCULAR_FAST_YAW_CONTROL_RESERVE_DEGREES_PER_SECOND
        const val CIRCULAR_TRACKING_FORWARD_SPEED_METERS_PER_SECOND = 0.24
        const val CIRCULAR_CORRECTION_FORWARD_SPEED_METERS_PER_SECOND = 0.20
        const val CIRCULAR_CENTERING_DEAD_ZONE_FRACTION = 0.04
        const val CIRCULAR_LATERAL_PROPORTIONAL_GAIN = 0.18
        const val CIRCULAR_LATERAL_DERIVATIVE_GAIN = 0.05
        const val CIRCULAR_MAX_CENTERING_SPEED_METERS_PER_SECOND = 0.06
        const val CIRCULAR_FAST_MAX_FORWARD_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.60
        const val CIRCULAR_MAX_FORWARD_DECELERATION_METERS_PER_SECOND_SQUARED = 1.20
        const val CIRCULAR_MAX_FORWARD_JERK_METERS_PER_SECOND_CUBED = 2.0
        const val CIRCULAR_SPEED_ERROR_RESPONSE_PER_SECOND = 3.0
        const val CIRCULAR_SPEED_SETTLED_TOLERANCE_METERS_PER_SECOND = 0.000_01
        const val CIRCULAR_CURVATURE_CAP_RISE_METERS_PER_SECOND_SQUARED = 0.25
        const val CURVATURE_MEDIAN_SAMPLE_COUNT = 3
        const val MAX_CURVATURE_CAP_UPDATE_INTERVAL_SECONDS = 0.25
        const val CIRCULAR_FULL_SPEED_OFFSET_FRACTION = 0.08
        const val CIRCULAR_FORWARD_STOP_OFFSET_FRACTION = 0.30
        const val CROSS_TRACK_RESPONSE_PREVIEW_SECONDS = 0.35
        const val MAX_LOOKAHEAD_RATE_PER_SECOND = 2.0
        const val LOOKAHEAD_RATE_FILTER_ALPHA = 0.35
        const val MAX_MEASUREMENT_AGE_COMPENSATION_SECONDS = 0.15
        const val COMMAND_RESPONSE_PREVIEW_SECONDS = 0.10
        const val MAX_LOOKAHEAD_PREDICTION_FRACTION = 0.12
        const val CIRCULAR_STABLE_ANGLE_DEGREES = 8.0
        const val CURVE_ALIGNMENT_ENTER_ANGLE_DEGREES = 45.0
        const val CURVE_ALIGNMENT_EXIT_ANGLE_DEGREES = 20.0
        const val CURVE_ALIGNMENT_CONFIRMATION_NANOS = 500_000_000L
        const val CURVE_MISALIGNMENT_CONFIRMATION_NANOS = 250_000_000L
        const val REACQUISITION_MAX_OFFSET_JUMP_FRACTION = 0.25
        const val REACQUISITION_MAX_ANGLE_JUMP_DEGREES = 45.0
        const val REACQUISITION_ENTRY_OFFSET_FRACTION =
            CIRCULAR_FORWARD_STOP_OFFSET_FRACTION
        // A lost-path reacquisition must be a substantial route, not a short floor feature.
        // Flight evidence after the detector hardening showed that waiting for a third
        // mutually consistent frame only prolonged a stationary tail reacquisition.
        const val CIRCULAR_REACQUISITION_MIN_PATH_FRACTION = 0.60
        const val REACQUISITION_CANDIDATE_OFFSET_TOLERANCE_FRACTION = 0.08
        const val REACQUISITION_CANDIDATE_ANGLE_TOLERANCE_DEGREES = 20.0
        const val REACQUISITION_CONFIRMATION_NANOS = 200_000_000L
        // Detector flicker on wrinkled tape may insert null or short-fragment frames.
        const val REACQUISITION_CANDIDATE_MAX_GAP_NANOS = 500_000_000L
        const val CIRCULAR_GAP_CONTINUATION_CONFIRMATION_NANOS = 250_000_000L
        // In out-and-back mode the operational endpoint signal is not a visible tape tip:
        // the latest flights end where tape passes under the mat, so no trustworthy
        // in-frame terminus exists. Establish a route from several full, near-aircraft
        // observations, then let sustained disappearance enter the existing low-speed
        // probe. A brief gap cannot arm this state, and a returning full path cancels it.
        const val CIRCULAR_RELIABLE_PATH_MIN_FRACTION = 0.60
        const val CIRCULAR_RELIABLE_PATH_CONFIRMATION_NANOS = 500_000_000L
        const val CIRCULAR_ENDPOINT_ENTRY_DURATION_NANOS = 250_000_000L
        const val CIRCULAR_ENDPOINT_RECOVERY_MIN_FRACTION = 0.60
        const val CIRCULAR_ENDPOINT_PROBE_SPEED_METERS_PER_SECOND = 0.02
        const val ENDPOINT_VERIFICATION_TIMEOUT_NANOS = 6_000_000_000L
        const val CIRCULAR_POST_TURN_RECOVERY_SPEED_METERS_PER_SECOND = 0.03
        const val CIRCULAR_POST_TURN_RECOVERY_NANOS = 6_000_000_000L
        // Immediately after a half-turn the camera may see only the near fragment;
        // roughly 500 ms of centered, mutually consistent frames compensate for
        // this deliberately lower length.
        const val CIRCULAR_POST_TURN_MIN_PATH_FRACTION = 0.20
        const val CIRCULAR_POST_TURN_MAX_ANGLE_DEGREES = 45.0
        const val CIRCULAR_POST_TURN_MAX_OFFSET_FRACTION = 0.20
        const val CIRCULAR_POST_TURN_CONFIRMATION_NANOS = 500_000_000L
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
