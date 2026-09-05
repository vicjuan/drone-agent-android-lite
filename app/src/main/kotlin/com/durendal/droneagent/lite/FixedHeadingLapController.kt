package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt

internal enum class FixedHeadingLapPhase {
    DISABLED,
    ACQUIRING,
    TRACKING,
    COASTING,
    RECOVERING_FRAME_STREAM,
    STOPPED,
}

internal enum class FixedHeadingActuationPhaseLead(
    val degrees: Double,
    val targetSpeedMetersPerSecond: Double,
    val desiredAlongTrackSpeedMetersPerSecond: Double? = null,
    val maximumCommandSpeedMetersPerSecond: Double = targetSpeedMetersPerSecond,
    val speedSlewRateMetersPerSecondSquared: Double? = null,
    val usesCurvatureFeedforward: Boolean = false,
) {
    DEGREES_0(0.0, 1.25),
    DEGREES_14(14.0, 1.25),
    DEGREES_16(
        degrees = 16.0,
        targetSpeedMetersPerSecond = 1.60,
        desiredAlongTrackSpeedMetersPerSecond = 0.70,
        maximumCommandSpeedMetersPerSecond = 1.80,
    ),
    CURVATURE_FEEDFORWARD_16(
        degrees = 16.0,
        targetSpeedMetersPerSecond = 1.60,
        desiredAlongTrackSpeedMetersPerSecond = 0.70,
        maximumCommandSpeedMetersPerSecond = 1.90,
        speedSlewRateMetersPerSecondSquared = 1.60,
        usesCurvatureFeedforward = true,
    ),
    CURVATURE_FEEDFORWARD_16_FAST(
        degrees = 16.0,
        targetSpeedMetersPerSecond = 1.90,
        desiredAlongTrackSpeedMetersPerSecond = 0.70,
        maximumCommandSpeedMetersPerSecond = 2.00,
        speedSlewRateMetersPerSecondSquared = 1.60,
        usesCurvatureFeedforward = true,
    ),
}


internal data class OmniCenterlineMeasurement(
    val tangentDegrees: Double,
    val lateralOffsetMeters: Double,
    val curvaturePerMeter: Double,
    val lookaheadXFraction: Double,
    val lookaheadYFraction: Double,
    val lookaheadRightMeters: Double,
    val lookaheadForwardMeters: Double,
    val lookaheadDistanceMeters: Double,
    val projectionXFraction: Double,
    val projectionYFraction: Double,
    val endpointDistanceMeters: Double? = null,
)

internal data class FixedHeadingLapDecision(
    val phase: FixedHeadingLapPhase,
    val forwardMetersPerSecond: Double = 0.0,
    val rightMetersPerSecond: Double = 0.0,
    val virtualHeadingDegrees: Double = 0.0,
    val tangentDegrees: Double? = null,
    val lateralOffsetMeters: Double? = null,
    val curvaturePerMeter: Double? = null,
    val endpointReached: Boolean = false,
    val stopRequested: Boolean = false,
    val measuredAlongTrackSpeedMetersPerSecond: Double? = null,
    val speedFeedbackBoostMetersPerSecond: Double = 0.0,
    val commandTargetSpeedMetersPerSecond: Double = 0.0,
    val lateralCorrectionMetersPerSecond: Double = 0.0,
)

/**
 * Fixed-aircraft-heading path follower. The aircraft yaw stays zero while a body-frame
 * velocity vector turns around the detected tape. It deliberately owns no DJI state.
 */
internal class FixedHeadingLapController {
    var enabled: Boolean = false
        private set

    private var phase = FixedHeadingLapPhase.DISABLED
    private var actuationPhaseLead = FixedHeadingActuationPhaseLead.DEGREES_0
    private var targetSpeedMetersPerSecond = TARGET_SPEED_METERS_PER_SECOND
    private var virtualHeadingDegrees = 0.0
    private var pathSpeedMetersPerSecond = 0.0
    private var pathAccelerationMetersPerSecondSquared = 0.0
    private var latestMeasurement: OmniCenterlineMeasurement? = null
    private var latestConfidence = 0.0
    private var lastDetectionAtNanos = 0L
    private var startedAtNanos = 0L
    private var lastTickAtNanos = 0L
    private var latestMeasurementCapturedAtNanos = 0L
    private var lastVirtualTurnRateDegreesPerSecond = 0.0
    private var pendingRecoveryMeasurement: OmniCenterlineMeasurement? = null
    private var lossObservedAtNanos = 0L
    private var frameStreamRecoveryStartedAtNanos = 0L
    private var measuredAlongTrackSpeedMetersPerSecond: Double? = null
    private var speedFeedbackBoostMetersPerSecond = 0.0
    private var commandTargetSpeedMetersPerSecond = 0.0
    private var filteredCurvaturePerMeter = 0.0
    private var filteredLateralOffsetMeters = 0.0

    fun start(
        nowNanos: Long,
        actuationPhaseLead: FixedHeadingActuationPhaseLead,
    ) {
        enabled = true
        this.actuationPhaseLead = actuationPhaseLead
        targetSpeedMetersPerSecond = actuationPhaseLead.targetSpeedMetersPerSecond
        phase = FixedHeadingLapPhase.ACQUIRING
        virtualHeadingDegrees = 0.0
        pathSpeedMetersPerSecond = 0.0
        pathAccelerationMetersPerSecondSquared = 0.0
        latestMeasurement = null
        latestConfidence = 0.0
        lastDetectionAtNanos = 0L
        startedAtNanos = nowNanos
        lastTickAtNanos = nowNanos
        latestMeasurementCapturedAtNanos = 0L
        lastVirtualTurnRateDegreesPerSecond = 0.0
        pendingRecoveryMeasurement = null
        lossObservedAtNanos = 0L
        frameStreamRecoveryStartedAtNanos = 0L
        measuredAlongTrackSpeedMetersPerSecond = null
        speedFeedbackBoostMetersPerSecond = 0.0
        commandTargetSpeedMetersPerSecond = 0.0
        filteredCurvaturePerMeter = 0.0
        filteredLateralOffsetMeters = 0.0
    }

    fun stop() {
        enabled = false
        phase = FixedHeadingLapPhase.DISABLED
        pathSpeedMetersPerSecond = 0.0
        pathAccelerationMetersPerSecondSquared = 0.0
        latestMeasurement = null
        lastTickAtNanos = 0L
        latestMeasurementCapturedAtNanos = 0L
        lastVirtualTurnRateDegreesPerSecond = 0.0
        pendingRecoveryMeasurement = null
        lossObservedAtNanos = 0L
        frameStreamRecoveryStartedAtNanos = 0L
        measuredAlongTrackSpeedMetersPerSecond = null
        speedFeedbackBoostMetersPerSecond = 0.0
        commandTargetSpeedMetersPerSecond = 0.0
        filteredCurvaturePerMeter = 0.0
        filteredLateralOffsetMeters = 0.0
    }

    /**
     * Holds position while MSDK rebuilds its decoded-frame callback. This is a
     * separate timeout from ordinary path loss: extending path-loss tolerance
     * would permit blind motion, while stream recovery always commands zero.
     */
    fun beginFrameStreamRecovery(nowNanos: Long) {
        if (!enabled || phase == FixedHeadingLapPhase.STOPPED) return
        phase = FixedHeadingLapPhase.RECOVERING_FRAME_STREAM
        frameStreamRecoveryStartedAtNanos = nowNanos
        pathSpeedMetersPerSecond = 0.0
        pathAccelerationMetersPerSecondSquared = 0.0
        latestMeasurement = null
        latestConfidence = 0.0
        pendingRecoveryMeasurement = null
        lossObservedAtNanos = nowNanos
        filteredCurvaturePerMeter = 0.0
        lastTickAtNanos = nowNanos
    }

    fun observe(
        centerline: TapeCenterlinePath?,
        heightMeters: Double?,
        confidence: Double,
        nowNanos: Long,
        capturedAtNanos: Long = 0L,
        actualTravelDirectionDegrees: Double? = null,
        actualGroundSpeedMetersPerSecond: Double? = null,
    ) {
        if (!enabled) return
        updateMeasuredAlongTrackSpeed(
            actualTravelDirectionDegrees = actualTravelDirectionDegrees,
            actualGroundSpeedMetersPerSecond = actualGroundSpeedMetersPerSecond,
        )
        val measurementAtNanos = capturedAtNanos.takeIf { it > 0L } ?: nowNanos
        val measurementAgeSeconds = (
            (nowNanos - measurementAtNanos).coerceAtLeast(0L) / NANOS_PER_SECOND
            ).coerceAtMost(MAX_MEASUREMENT_AGE_COMPENSATION_SECONDS)
        val virtualHeadingAtCapture = wrapDegrees(
            virtualHeadingDegrees -
                lastVirtualTurnRateDegreesPerSecond * measurementAgeSeconds,
        )
        val measurement =
            if (centerline == null || heightMeters == null) {
                null
            } else if (phase == FixedHeadingLapPhase.RECOVERING_FRAME_STREAM) {
                recoveryMeasurement(
                    centerline = centerline,
                    heightMeters = heightMeters,
                    travelDirectionDegrees = virtualHeadingAtCapture,
                    confidence = confidence,
                )
            } else {
                val continuous = OmniCenterline.measure(
                    path = centerline,
                    heightMeters = heightMeters,
                    travelDirectionDegrees = virtualHeadingAtCapture,
                    lookaheadMeters = configuredLookaheadMeters(),
                    previousMeasurement = latestMeasurement,
                )
                continuous ?: recoveryMeasurement(
                    centerline = centerline,
                    heightMeters = heightMeters,
                    travelDirectionDegrees = virtualHeadingAtCapture,
                    confidence = confidence,
                )
            }
        if (measurement != null) {
            latestConfidence = confidence
            latestMeasurement = measurement
            pendingRecoveryMeasurement = null
            lastDetectionAtNanos = nowNanos
            latestMeasurementCapturedAtNanos = measurementAtNanos
            lossObservedAtNanos = 0L
        } else {
            if (phase == FixedHeadingLapPhase.TRACKING) {
                phase = FixedHeadingLapPhase.COASTING
                lossObservedAtNanos = nowNanos
                pathAccelerationMetersPerSecondSquared = 0.0
            }
            return
        }
        if (phase == FixedHeadingLapPhase.ACQUIRING) {
            // The feedforward profile owns the tangent state; the lookahead chord
            // remains the residual correction target.
            virtualHeadingDegrees =
                if (
                    actuationPhaseLead.usesCurvatureFeedforward &&
                    measuredAlongTrackSpeedMetersPerSecond != null
                ) {
                    measurement.tangentDegrees
                } else {
                    directionToLookahead(measurement)
                }
            phase = FixedHeadingLapPhase.TRACKING
            lastTickAtNanos = nowNanos
        } else if (
            phase == FixedHeadingLapPhase.COASTING ||
            phase == FixedHeadingLapPhase.RECOVERING_FRAME_STREAM
        ) {
            phase = FixedHeadingLapPhase.TRACKING
            frameStreamRecoveryStartedAtNanos = 0L
            lastTickAtNanos = nowNanos
        }
    }

    fun tick(nowNanos: Long): FixedHeadingLapDecision {
        if (!enabled) return FixedHeadingLapDecision(FixedHeadingLapPhase.DISABLED)
        if (phase == FixedHeadingLapPhase.ACQUIRING) {
            if (nowNanos - startedAtNanos >= ACQUISITION_TIMEOUT_NANOS) {
                phase = FixedHeadingLapPhase.STOPPED
                return decision(stopRequested = true)
            }
            return decision()
        }
        if (phase == FixedHeadingLapPhase.STOPPED) {
            return decision(stopRequested = true)
        }
        if (phase == FixedHeadingLapPhase.RECOVERING_FRAME_STREAM) {
            if (
                nowNanos - frameStreamRecoveryStartedAtNanos >=
                FRAME_STREAM_RECOVERY_TIMEOUT_NANOS
            ) {
                phase = FixedHeadingLapPhase.STOPPED
                return decision(stopRequested = true)
            }
            lastTickAtNanos = nowNanos
            return decision()
        }

        val elapsedSeconds =
            ((nowNanos - lastTickAtNanos).coerceIn(0L, MAX_TICK_NANOS)) / NANOS_PER_SECOND
        lastTickAtNanos = nowNanos

        val detectionAgeNanos = nowNanos - lastDetectionAtNanos
        if (
            latestMeasurement == null ||
            detectionAgeNanos > REACQUISITION_TIMEOUT_NANOS
        ) {
            pathSpeedMetersPerSecond = 0.0
            pathAccelerationMetersPerSecondSquared = 0.0
            phase = FixedHeadingLapPhase.STOPPED
            return decision(stopRequested = true)
        }
        val speedHoldWindowNanos = staleWindowNanos()
        val withinExplicitLossHold =
            phase == FixedHeadingLapPhase.COASTING &&
                lossObservedAtNanos > 0L &&
                nowNanos - lossObservedAtNanos <= speedHoldWindowNanos
        if (
            phase == FixedHeadingLapPhase.COASTING ||
            detectionAgeNanos > speedHoldWindowNanos
        ) {
            phase = FixedHeadingLapPhase.COASTING
            filteredCurvaturePerMeter = 0.0
            filteredLateralOffsetMeters = 0.0
            if (!withinExplicitLossHold) {
                pathAccelerationMetersPerSecondSquared = 0.0
                pathSpeedMetersPerSecond = moveToward(
                    pathSpeedMetersPerSecond,
                    0.0,
                    MAX_BLIND_DECELERATION_METERS_PER_SECOND_SQUARED * elapsedSeconds,
                )
            }
            val headingRadians = Math.toRadians(virtualHeadingDegrees)
            return decision(
                forward = pathSpeedMetersPerSecond * cos(headingRadians),
                right = pathSpeedMetersPerSecond * sin(headingRadians),
            )
        }

        val measurement = checkNotNull(latestMeasurement)
        val desiredHeadingDegrees = directionToLookahead(measurement)
        val guidanceError =
            shortestAngularDelta(virtualHeadingDegrees, desiredHeadingDegrees)
        if (abs(guidanceError) > MAX_TRACKING_ANGLE_DEGREES) {
            pathSpeedMetersPerSecond = 0.0
            pathAccelerationMetersPerSecondSquared = 0.0
            phase = FixedHeadingLapPhase.STOPPED
            return decision(stopRequested = true)
        }

        val curvatureFeedforwardActive =
            actuationPhaseLead.usesCurvatureFeedforward &&
                measuredAlongTrackSpeedMetersPerSecond != null
        if (!curvatureFeedforwardActive) {
            filteredCurvaturePerMeter = 0.0
            filteredLateralOffsetMeters = 0.0
        }
        val curvature =
            if (curvatureFeedforwardActive) {
                updateFilteredCurvature(measurement.curvaturePerMeter)
            } else {
                0.0
            }
        val expectedGuidanceLeadDegrees =
            Math.toDegrees(curvature * configuredLookaheadMeters() / 2.0)
        val guidanceResidualDegrees = guidanceError - expectedGuidanceLeadDegrees
        val expectedLateralOffsetMeters =
            abs(curvature) * configuredLookaheadMeters() * configuredLookaheadMeters() / 2.0
        val lateralOffsetResidualMeters =
            (abs(measurement.lateralOffsetMeters) - expectedLateralOffsetMeters)
                .coerceAtLeast(0.0)

        commandTargetSpeedMetersPerSecond = feedbackCompensatedCommandSpeed()
        val confidenceLimitedSpeed =
            if (latestConfidence >= MIN_CONFIDENCE) {
                commandTargetSpeedMetersPerSecond
            } else {
                DEGRADED_SPEED_METERS_PER_SECOND
            }
        val trackingAuthority =
            minOf(
                authorityAfterMargin(
                    magnitude = abs(guidanceResidualDegrees),
                    fullAuthorityLimit = HIGH_SPEED_FULL_AUTHORITY_GUIDANCE_ERROR_DEGREES,
                    zeroAuthorityLimit = SLOWDOWN_GUIDANCE_ERROR_DEGREES,
                ),
                authorityAfterMargin(
                    magnitude = lateralOffsetResidualMeters,
                    fullAuthorityLimit = HIGH_SPEED_FULL_AUTHORITY_LATERAL_OFFSET_METERS,
                    zeroAuthorityLimit = SLOWDOWN_LATERAL_OFFSET_METERS,
                ),
            )
        val authorityLimitedSpeed = confidenceLimitedSpeed * trackingAuthority
        val lookaheadLimitedSpeed =
            commandTargetSpeedMetersPerSecond *
                (measurement.lookaheadDistanceMeters / configuredLookaheadMeters())
                    .coerceIn(0.0, 1.0)
        val targetSpeed = minOf(
            commandTargetSpeedMetersPerSecond,
            authorityLimitedSpeed,
            lookaheadLimitedSpeed,
        )
        updatePathSpeed(
            targetSpeed = targetSpeed,
            emergencyBrake = trackingAuthority == 0.0,
            elapsedSeconds = elapsedSeconds,
        )
        val maximumTurnDegrees =
            MAX_VIRTUAL_TURN_RATE_DEGREES_PER_SECOND * elapsedSeconds
        val feedforwardTurnDegrees =
            Math.toDegrees(
                (measuredAlongTrackSpeedMetersPerSecond ?: 0.0) * curvature,
            ) * elapsedSeconds
        val residualTurnDegrees =
            guidanceResidualDegrees * GUIDANCE_RESIDUAL_RESPONSE_PER_SECOND * elapsedSeconds
        val turnDegrees =
            (feedforwardTurnDegrees + residualTurnDegrees)
                .coerceIn(-maximumTurnDegrees, maximumTurnDegrees)
        virtualHeadingDegrees = wrapDegrees(virtualHeadingDegrees + turnDegrees)
        lastVirtualTurnRateDegreesPerSecond =
            if (elapsedSeconds > 0.0) turnDegrees / elapsedSeconds else 0.0

        val phaseLeadDegrees = appliedActuationPhaseLeadDegrees()
        val commandHeadingDegrees = wrapDegrees(virtualHeadingDegrees + phaseLeadDegrees)
        val commandHeadingRadians = Math.toRadians(commandHeadingDegrees)
        val lateralCorrectionSourceMeters =
            when {
                pathSpeedMetersPerSecond == 0.0 -> measurement.lateralOffsetMeters
                curvatureFeedforwardActive ->
                    updateFilteredLateralOffset(measurement.lateralOffsetMeters)
                else -> 0.0
            }
        val correction =
            (LATERAL_FEEDBACK_GAIN_PER_SECOND * lateralCorrectionSourceMeters)
                .coerceIn(
                    -MAX_LATERAL_CORRECTION_METERS_PER_SECOND,
                    MAX_LATERAL_CORRECTION_METERS_PER_SECOND,
                )
        val correctionHeadingRadians =
            Math.toRadians(measurement.tangentDegrees + phaseLeadDegrees)
        val forward =
            pathSpeedMetersPerSecond * cos(commandHeadingRadians) -
                correction * sin(correctionHeadingRadians)
        val right =
            pathSpeedMetersPerSecond * sin(commandHeadingRadians) +
                correction * cos(correctionHeadingRadians)
        val resultantSpeed = hypot(forward, right)
        val resultantScale =
            if (resultantSpeed > actuationPhaseLead.maximumCommandSpeedMetersPerSecond) {
                actuationPhaseLead.maximumCommandSpeedMetersPerSecond / resultantSpeed
            } else {
                1.0
            }
        phase = FixedHeadingLapPhase.TRACKING
        return decision(
            forward = forward * resultantScale,
            right = right * resultantScale,
            lateralCorrectionMetersPerSecond = correction * resultantScale,
        )
    }


    /**
     * A specular highlight can redraw the same physical tape far enough to fail
     * the previous-frame jump limits. Rebase only after two mutually continuous
     * samples that still point along the commanded travel direction and remain
     * close to the aircraft. One isolated reflection frame can therefore slow
     * the aircraft, but cannot redirect it.
     */
    private fun recoveryMeasurement(
        centerline: TapeCenterlinePath,
        heightMeters: Double,
        travelDirectionDegrees: Double,
        confidence: Double,
    ): OmniCenterlineMeasurement? {
        if (phase == FixedHeadingLapPhase.ACQUIRING || confidence < MIN_CONFIDENCE) {
            pendingRecoveryMeasurement = null
            return null
        }
        val previousRecovery = pendingRecoveryMeasurement
        if (previousRecovery == null) {
            pendingRecoveryMeasurement =
                OmniCenterline.measure(
                    path = centerline,
                    heightMeters = heightMeters,
                    travelDirectionDegrees = travelDirectionDegrees,
                    lookaheadMeters = configuredLookaheadMeters(),
                )?.takeIf {
                    isSafeRecoveryMeasurement(it, travelDirectionDegrees)
                }
            return null
        }
        val candidate = OmniCenterline.measure(
            path = centerline,
            heightMeters = heightMeters,
            travelDirectionDegrees = travelDirectionDegrees,
            lookaheadMeters = configuredLookaheadMeters(),
            previousMeasurement = previousRecovery,
        )
        if (candidate == null || !isSafeRecoveryMeasurement(candidate, travelDirectionDegrees)) {
            pendingRecoveryMeasurement = null
            return null
        }
        return candidate
    }

    private fun isSafeRecoveryMeasurement(
        measurement: OmniCenterlineMeasurement,
        travelDirectionDegrees: Double,
    ): Boolean =
        abs(shortestAngularDelta(
            travelDirectionDegrees,
            directionToLookahead(measurement),
        )) <= MAX_RECOVERY_GUIDANCE_ERROR_DEGREES &&
            abs(measurement.lateralOffsetMeters) <= MAX_RECOVERY_LATERAL_OFFSET_METERS

    private fun directionToLookahead(measurement: OmniCenterlineMeasurement): Double =
        wrapDegrees(
            Math.toDegrees(
                atan2(
                    measurement.lookaheadRightMeters,
                    measurement.lookaheadForwardMeters,
                ),
            ),
        )

    private fun staleWindowNanos(): Long {
        val speed = pathSpeedMetersPerSecond.coerceAtLeast(0.05)
        val committedHighSpeed =
            latestConfidence >= HIGH_SPEED_COMMIT_CONFIDENCE
        val maximumBlindDistanceMeters =
            if (committedHighSpeed) HIGH_SPEED_MAX_BLIND_DISTANCE_METERS else MAX_BLIND_DISTANCE_METERS
        val maximumStaleNanos =
            if (committedHighSpeed) HIGH_SPEED_MAX_STALE_NANOS else MAX_STALE_NANOS
        return (maximumBlindDistanceMeters / speed * 1_000_000_000.0).toLong()
            .coerceIn(MIN_STALE_NANOS, maximumStaleNanos)
    }

    private fun configuredLookaheadMeters(): Double = LOOKAHEAD_METERS

    private fun appliedActuationPhaseLeadDegrees(): Double {
        if (
            phase != FixedHeadingLapPhase.TRACKING ||
            pathSpeedMetersPerSecond == 0.0
        ) {
            return 0.0
        }
        return when {
            lastVirtualTurnRateDegreesPerSecond > 0.0 -> actuationPhaseLead.degrees
            lastVirtualTurnRateDegreesPerSecond < 0.0 -> -actuationPhaseLead.degrees
            else -> 0.0
        }
    }
    private fun updateMeasuredAlongTrackSpeed(
        actualTravelDirectionDegrees: Double?,
        actualGroundSpeedMetersPerSecond: Double?,
    ) {
        if (
            actualTravelDirectionDegrees == null ||
            actualGroundSpeedMetersPerSecond == null ||
            !actualTravelDirectionDegrees.isFinite() ||
            !actualGroundSpeedMetersPerSecond.isFinite() ||
            actualGroundSpeedMetersPerSecond < 0.0
        ) {
            measuredAlongTrackSpeedMetersPerSecond = null
            return
        }
        val travelDirectionErrorDegrees =
            shortestAngularDelta(virtualHeadingDegrees, actualTravelDirectionDegrees)
        measuredAlongTrackSpeedMetersPerSecond =
            if (abs(travelDirectionErrorDegrees) <= MAX_SPEED_FEEDBACK_DIRECTION_ERROR_DEGREES) {
                (
                    actualGroundSpeedMetersPerSecond *
                        cos(Math.toRadians(travelDirectionErrorDegrees))
                    ).coerceAtLeast(0.0)
            } else {
                null
            }
    }

    private fun feedbackCompensatedCommandSpeed(): Double {
        val desiredSpeed = actuationPhaseLead.desiredAlongTrackSpeedMetersPerSecond
        val measuredSpeed = measuredAlongTrackSpeedMetersPerSecond
        if (desiredSpeed == null || measuredSpeed == null) {
            speedFeedbackBoostMetersPerSecond = 0.0
            return targetSpeedMetersPerSecond
        }
        speedFeedbackBoostMetersPerSecond =
            (
                SPEED_FEEDBACK_GAIN *
                    (desiredSpeed - measuredSpeed).coerceAtLeast(0.0)
                ).coerceAtMost(
                actuationPhaseLead.maximumCommandSpeedMetersPerSecond -
                    targetSpeedMetersPerSecond,
            )
        return targetSpeedMetersPerSecond + speedFeedbackBoostMetersPerSecond
    }


    private fun updateFilteredCurvature(measuredCurvaturePerMeter: Double): Double {
        val boundedMeasurement =
            measuredCurvaturePerMeter.coerceIn(
                -MAX_FEEDFORWARD_CURVATURE_PER_METER,
                MAX_FEEDFORWARD_CURVATURE_PER_METER,
            )
        filteredCurvaturePerMeter +=
            CURVATURE_FILTER_ALPHA * (boundedMeasurement - filteredCurvaturePerMeter)
        return filteredCurvaturePerMeter
    }
    private fun updateFilteredLateralOffset(measuredOffsetMeters: Double): Double {
        filteredLateralOffsetMeters +=
            LATERAL_OFFSET_FILTER_ALPHA *
                (measuredOffsetMeters - filteredLateralOffsetMeters)
        return filteredLateralOffsetMeters
    }


    private fun authorityAfterMargin(
        magnitude: Double,
        fullAuthorityLimit: Double,
        zeroAuthorityLimit: Double,
    ): Double =
        (
            1.0 -
                (magnitude - fullAuthorityLimit) /
                (zeroAuthorityLimit - fullAuthorityLimit)
            ).coerceIn(0.0, 1.0)

    private fun decision(
        forward: Double = 0.0,
        right: Double = 0.0,
        stopRequested: Boolean = false,
        lateralCorrectionMetersPerSecond: Double = 0.0,
    ): FixedHeadingLapDecision = FixedHeadingLapDecision(
        phase = phase,
        forwardMetersPerSecond = forward,
        rightMetersPerSecond = right,
        virtualHeadingDegrees = virtualHeadingDegrees,
        tangentDegrees = latestMeasurement?.tangentDegrees,
        lateralOffsetMeters = latestMeasurement?.lateralOffsetMeters,
        curvaturePerMeter = latestMeasurement?.curvaturePerMeter,
        measuredAlongTrackSpeedMetersPerSecond = measuredAlongTrackSpeedMetersPerSecond,
        speedFeedbackBoostMetersPerSecond = speedFeedbackBoostMetersPerSecond,
        commandTargetSpeedMetersPerSecond = commandTargetSpeedMetersPerSecond,
        lateralCorrectionMetersPerSecond = lateralCorrectionMetersPerSecond,
        stopRequested = stopRequested,
    )

    private fun updatePathSpeed(
        targetSpeed: Double,
        emergencyBrake: Boolean,
        elapsedSeconds: Double,
    ) {
        if (emergencyBrake) {
            pathAccelerationMetersPerSecondSquared = 0.0
            pathSpeedMetersPerSecond = moveToward(
                pathSpeedMetersPerSecond,
                targetSpeed,
                MAX_BLIND_DECELERATION_METERS_PER_SECOND_SQUARED * elapsedSeconds,
            )
            return
        }
        actuationPhaseLead.speedSlewRateMetersPerSecondSquared?.let { speedSlewRate ->
            val previousSpeed = pathSpeedMetersPerSecond
            pathSpeedMetersPerSecond = moveToward(
                current = previousSpeed,
                target = targetSpeed,
                maximumDelta = speedSlewRate * elapsedSeconds,
            )
            pathAccelerationMetersPerSecondSquared =
                if (elapsedSeconds > 0.0) {
                    (pathSpeedMetersPerSecond - previousSpeed) / elapsedSeconds
                } else {
                    0.0
                }
            return
        }

        val speedError = targetSpeed - pathSpeedMetersPerSecond
        val desiredAcceleration = (
            speedError * SPEED_ERROR_RESPONSE_PER_SECOND
            ).coerceIn(
            -MAX_DECELERATION_METERS_PER_SECOND_SQUARED,
            HIGH_SPEED_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
        )
        pathAccelerationMetersPerSecondSquared = moveToward(
            pathAccelerationMetersPerSecondSquared,
            desiredAcceleration,
            HIGH_SPEED_MAX_JERK_METERS_PER_SECOND_CUBED * elapsedSeconds,
        )
        val nextSpeed = (
            pathSpeedMetersPerSecond +
                pathAccelerationMetersPerSecondSquared * elapsedSeconds
            ).coerceAtLeast(0.0)
        val reachesTarget =
            speedError == 0.0 ||
                speedError > 0.0 && nextSpeed >= targetSpeed ||
                speedError < 0.0 && nextSpeed <= targetSpeed
        if (reachesTarget) {
            pathSpeedMetersPerSecond = targetSpeed
            pathAccelerationMetersPerSecondSquared = 0.0
        } else {
            pathSpeedMetersPerSecond = nextSpeed
        }
    }

    private fun moveToward(current: Double, target: Double, maximumDelta: Double): Double =
        when {
            target > current -> minOf(target, current + maximumDelta)
            target < current -> maxOf(target, current - maximumDelta)
            else -> current
        }

    internal companion object {
        const val TARGET_SPEED_METERS_PER_SECOND = 1.25
        const val FASTER_TARGET_SPEED_METERS_PER_SECOND = 1.60
        const val LOOKAHEAD_METERS = 0.45
        const val DEGRADED_SPEED_METERS_PER_SECOND = 0.22
        const val HIGH_SPEED_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1.00
        const val MAX_DECELERATION_METERS_PER_SECOND_SQUARED = 1.20
        const val MAX_BLIND_DECELERATION_METERS_PER_SECOND_SQUARED = 1.50
        const val HIGH_SPEED_MAX_JERK_METERS_PER_SECOND_CUBED = 4.00
        const val SPEED_ERROR_RESPONSE_PER_SECOND = 3.00
        const val SPEED_FEEDBACK_GAIN = 1.0
        const val MAX_SPEED_FEEDBACK_DIRECTION_ERROR_DEGREES = 45.0
        const val MAX_VIRTUAL_TURN_RATE_DEGREES_PER_SECOND = 60.0
        const val MAX_FEEDFORWARD_CURVATURE_PER_METER = 1.20
        const val GUIDANCE_RESIDUAL_RESPONSE_PER_SECOND = 4.0
        const val CURVATURE_FILTER_ALPHA = 0.35
        const val LATERAL_FEEDBACK_GAIN_PER_SECOND = 0.8
        const val LATERAL_OFFSET_FILTER_ALPHA = 0.35
        const val MAX_LATERAL_CORRECTION_METERS_PER_SECOND = 0.25
        const val MIN_CONFIDENCE = 0.60
        const val HIGH_SPEED_COMMIT_CONFIDENCE = 0.75
        const val MAX_TRACKING_ANGLE_DEGREES = 70.0
        const val ACQUISITION_TIMEOUT_NANOS = 8_000_000_000L
        const val MAX_BLIND_DISTANCE_METERS = 0.10
        const val HIGH_SPEED_MAX_BLIND_DISTANCE_METERS = 0.30
        const val MIN_STALE_NANOS = 80_000_000L
        const val MAX_STALE_NANOS = 150_000_000L
        const val HIGH_SPEED_MAX_STALE_NANOS = 180_000_000L
        const val REACQUISITION_TIMEOUT_NANOS = 4_000_000_000L
        const val FRAME_STREAM_RECOVERY_TIMEOUT_NANOS = 4_000_000_000L
        const val MAX_TICK_NANOS = 250_000_000L
        const val SLOWDOWN_GUIDANCE_ERROR_DEGREES = 35.0
        const val HIGH_SPEED_FULL_AUTHORITY_GUIDANCE_ERROR_DEGREES = 10.0
        const val MAX_RECOVERY_GUIDANCE_ERROR_DEGREES = 35.0
        const val MAX_RECOVERY_LATERAL_OFFSET_METERS = 0.20
        const val SLOWDOWN_LATERAL_OFFSET_METERS = 0.25
        const val HIGH_SPEED_FULL_AUTHORITY_LATERAL_OFFSET_METERS = 0.08
        const val MAX_MEASUREMENT_AGE_COMPENSATION_SECONDS = 0.20
        private const val NANOS_PER_SECOND = 1_000_000_000.0
    }
}

internal object OmniCenterline {
    fun measure(
        path: TapeCenterlinePath,
        heightMeters: Double,
        travelDirectionDegrees: Double,
        lookaheadMeters: Double = LOOKAHEAD_METERS,
        endpointCandidate: Boolean = false,
        previousMeasurement: OmniCenterlineMeasurement? = null,
    ): OmniCenterlineMeasurement? {
        if (!heightMeters.isFinite() || heightMeters <= 0.0 || path.pointCount < MIN_POINT_COUNT) return null
        val aspectRatio = path.sourceWidth.toDouble() / path.sourceHeight
        val verticalHalfFovTangent =
            TapeTrackingController.CAMERA_DIAGONAL_HALF_FOV_TANGENT / sqrt(1.0 + aspectRatio * aspectRatio)
        val frameHeightMeters = 2.0 * heightMeters * verticalHalfFovTangent
        val frameWidthMeters = frameHeightMeters * aspectRatio
        val targetXFraction = FRAME_CENTER_FRACTION
        val targetYFraction = FRAME_CENTER_FRACTION
        val points = List(path.pointCount) { index ->
            GroundPoint(
                rightMeters = (path.xFractions[index] - targetXFraction) * frameWidthMeters,
                forwardMeters = (targetYFraction - path.yFractions[index]) * frameHeightMeters,
                xFraction = path.xFractions[index].toDouble(),
                yFraction = path.yFractions[index].toDouble(),
            )
        }
        // Every segment is a possible projection. Prefix arc lengths make each
        // candidate's remaining distance O(1) and its lookahead lookup O(log n)
        // instead of walking the rest of the centerline again.
        val cumulativeDistances = DoubleArray(points.size)
        for (index in 1 until points.size) {
            val previous = points[index - 1]
            val current = points[index]
            cumulativeDistances[index] =
                cumulativeDistances[index - 1] +
                    hypot(
                        current.rightMeters - previous.rightMeters,
                        current.forwardMeters - previous.forwardMeters,
                    )
        }
        val groundPath = GroundPath(points, cumulativeDistances)

        var bestCandidate: MeasurementCandidate? = null
        for (index in 0 until points.lastIndex) {
            val projection = projectOntoSegment(points, index) ?: continue
            val measurement = measurementForProjection(
                path = groundPath,
                projection = projection,
                travelDirectionDegrees = travelDirectionDegrees,
                requestedLookaheadMeters = lookaheadMeters,
                endpointCandidate = endpointCandidate,
            ) ?: continue
            val score = continuityScore(
                measurement = measurement,
                projectionDistanceSquared = projection.distanceSquared,
                previousMeasurement = previousMeasurement,
                frameWidthMeters = frameWidthMeters,
                frameHeightMeters = frameHeightMeters,
            ) ?: continue
            if (bestCandidate == null || score < bestCandidate.score) {
                bestCandidate = MeasurementCandidate(measurement, score)
            }
        }
        return bestCandidate?.measurement
    }

    private fun projectOntoSegment(
        points: List<GroundPoint>,
        segmentIndex: Int,
    ): Projection? {
        val start = points[segmentIndex]
        val end = points[segmentIndex + 1]
        val deltaRight = end.rightMeters - start.rightMeters
        val deltaForward = end.forwardMeters - start.forwardMeters
        val lengthSquared = deltaRight * deltaRight + deltaForward * deltaForward
        if (lengthSquared <= 1e-9) return null
        val fraction = (
            -(start.rightMeters * deltaRight + start.forwardMeters * deltaForward) / lengthSquared
            ).coerceIn(0.0, 1.0)
        val projected = interpolate(start, end, fraction)
        return Projection(
            segmentIndex = segmentIndex,
            segmentFraction = fraction,
            point = projected,
            distanceSquared =
                projected.rightMeters * projected.rightMeters +
                    projected.forwardMeters * projected.forwardMeters,
        )
    }

    private fun measurementForProjection(
        path: GroundPath,
        projection: Projection,
        travelDirectionDegrees: Double,
        requestedLookaheadMeters: Double,
        endpointCandidate: Boolean,
    ): OmniCenterlineMeasurement? {
        val points = path.points
        val tangentStart =
            points[(projection.segmentIndex - TANGENT_POINT_SPAN).coerceAtLeast(0)]
        val tangentEnd =
            points[
                (projection.segmentIndex + 1 + TANGENT_POINT_SPAN)
                    .coerceAtMost(points.lastIndex)
            ]
        val baseTangent = directionDegrees(
            tangentEnd.rightMeters - tangentStart.rightMeters,
            tangentEnd.forwardMeters - tangentStart.forwardMeters,
        ) ?: return null
        val reverseTangent = wrapDegrees(baseTangent + 180.0)
        val followsIncreasingIndices =
            abs(shortestAngularDelta(travelDirectionDegrees, baseTangent)) <=
                abs(shortestAngularDelta(travelDirectionDegrees, reverseTangent))
        val tangent = if (followsIncreasingIndices) baseTangent else reverseTangent
        if (
            abs(shortestAngularDelta(travelDirectionDegrees, tangent)) >
            MAX_DIRECTION_CHANGE_DEGREES
        ) {
            return null
        }

        val remainingPathMeters =
            remainingPathDistance(path, projection, followsIncreasingIndices)
        val tangentRadians = Math.toRadians(tangent)
        val pathRightNormalRight = cos(tangentRadians)
        val pathRightNormalForward = -sin(tangentRadians)
        val lateralOffset =
            projection.point.rightMeters * pathRightNormalRight +
                projection.point.forwardMeters * pathRightNormalForward
        if (endpointCandidate && remainingPathMeters < MIN_LOOKAHEAD_METERS) {
            val endpoint = if (followsIncreasingIndices) points.last() else points.first()
            return OmniCenterlineMeasurement(
                tangentDegrees = tangent,
                lateralOffsetMeters = lateralOffset,
                curvaturePerMeter = 0.0,
                lookaheadXFraction = endpoint.xFraction,
                lookaheadYFraction = endpoint.yFraction,
                lookaheadRightMeters = endpoint.rightMeters,
                lookaheadForwardMeters = endpoint.forwardMeters,
                lookaheadDistanceMeters = remainingPathMeters,
                projectionXFraction = projection.point.xFraction,
                projectionYFraction = projection.point.yFraction,
                endpointDistanceMeters = remainingPathMeters,
            )
        }
        val lookahead = lookaheadPoint(
            path = path,
            projection = projection,
            increasing = followsIncreasingIndices,
            targetDistance = requestedLookaheadMeters,
        ) ?: return null
        val headingRadians = Math.toRadians(travelDirectionDegrees)
        val rightNormalRight = cos(headingRadians)
        val rightNormalForward = -sin(headingRadians)
        val lateralLookahead =
            lookahead.point.rightMeters * rightNormalRight +
                lookahead.point.forwardMeters * rightNormalForward
        val lookaheadDistanceSquared =
            lookahead.point.rightMeters * lookahead.point.rightMeters +
                lookahead.point.forwardMeters * lookahead.point.forwardMeters
        if (lookaheadDistanceSquared < MIN_LOOKAHEAD_METERS * MIN_LOOKAHEAD_METERS) return null
        return OmniCenterlineMeasurement(
            tangentDegrees = tangent,
            lateralOffsetMeters = lateralOffset,
            curvaturePerMeter = 2.0 * lateralLookahead / lookaheadDistanceSquared,
            lookaheadXFraction = lookahead.point.xFraction,
            lookaheadYFraction = lookahead.point.yFraction,
            lookaheadRightMeters = lookahead.point.rightMeters,
            lookaheadForwardMeters = lookahead.point.forwardMeters,
            lookaheadDistanceMeters = lookahead.distanceMeters,
            projectionXFraction = projection.point.xFraction,
            projectionYFraction = projection.point.yFraction,
            endpointDistanceMeters = if (endpointCandidate) remainingPathMeters else null,
        )
    }

    private fun continuityScore(
        measurement: OmniCenterlineMeasurement,
        projectionDistanceSquared: Double,
        previousMeasurement: OmniCenterlineMeasurement?,
        frameWidthMeters: Double,
        frameHeightMeters: Double,
    ): Double? {
        if (previousMeasurement == null) return projectionDistanceSquared
        val projectionJumpMeters = groundDistance(
            firstXFraction = previousMeasurement.projectionXFraction,
            firstYFraction = previousMeasurement.projectionYFraction,
            secondXFraction = measurement.projectionXFraction,
            secondYFraction = measurement.projectionYFraction,
            frameWidthMeters = frameWidthMeters,
            frameHeightMeters = frameHeightMeters,
        )
        if (projectionJumpMeters > MAX_PROJECTION_JUMP_METERS) return null
        val lookaheadJumpMeters = groundDistance(
            firstXFraction = previousMeasurement.lookaheadXFraction,
            firstYFraction = previousMeasurement.lookaheadYFraction,
            secondXFraction = measurement.lookaheadXFraction,
            secondYFraction = measurement.lookaheadYFraction,
            frameWidthMeters = frameWidthMeters,
            frameHeightMeters = frameHeightMeters,
        )
        if (lookaheadJumpMeters > MAX_LOOKAHEAD_JUMP_METERS) return null
        return projectionDistanceSquared +
            CONTINUITY_PROJECTION_WEIGHT * projectionJumpMeters * projectionJumpMeters +
            CONTINUITY_LOOKAHEAD_WEIGHT * lookaheadJumpMeters * lookaheadJumpMeters
    }

    private fun groundDistance(
        firstXFraction: Double,
        firstYFraction: Double,
        secondXFraction: Double,
        secondYFraction: Double,
        frameWidthMeters: Double,
        frameHeightMeters: Double,
    ): Double = hypot(
        (secondXFraction - firstXFraction) * frameWidthMeters,
        (secondYFraction - firstYFraction) * frameHeightMeters,
    )

    private fun lookaheadPoint(
        path: GroundPath,
        projection: Projection,
        increasing: Boolean,
        targetDistance: Double,
    ): Lookahead? {
        val projectionDistance = path.distanceAt(projection)
        val endDistance =
            if (increasing) {
                minOf(projectionDistance + targetDistance, path.totalDistance)
            } else {
                maxOf(projectionDistance - targetDistance, 0.0)
            }
        val availableDistance = abs(endDistance - projectionDistance)
        if (availableDistance < MIN_LOOKAHEAD_METERS) return null
        return Lookahead(
            point = path.pointAt(endDistance),
            distanceMeters = availableDistance,
        )
    }

    private fun remainingPathDistance(
        path: GroundPath,
        projection: Projection,
        increasing: Boolean,
    ): Double {
        val projectionDistance = path.distanceAt(projection)
        return if (increasing) {
            path.totalDistance - projectionDistance
        } else {
            projectionDistance
        }
    }

    private fun interpolate(first: GroundPoint, second: GroundPoint, fraction: Double): GroundPoint =
        GroundPoint(
            rightMeters = first.rightMeters + (second.rightMeters - first.rightMeters) * fraction,
            forwardMeters = first.forwardMeters + (second.forwardMeters - first.forwardMeters) * fraction,
            xFraction = first.xFraction + (second.xFraction - first.xFraction) * fraction,
            yFraction = first.yFraction + (second.yFraction - first.yFraction) * fraction,
        )

    private fun directionDegrees(right: Double, forward: Double): Double? {
        if (right == 0.0 && forward == 0.0) return null
        return wrapDegrees(Math.toDegrees(atan2(right, forward)))
    }

    private data class GroundPath(
        val points: List<GroundPoint>,
        val cumulativeDistances: DoubleArray,
    ) {
        val totalDistance: Double
            get() = cumulativeDistances.last()

        fun distanceAt(projection: Projection): Double {
            val segmentStart = cumulativeDistances[projection.segmentIndex]
            val segmentEnd = cumulativeDistances[projection.segmentIndex + 1]
            return segmentStart + (segmentEnd - segmentStart) * projection.segmentFraction
        }

        fun pointAt(distance: Double): GroundPoint {
            if (distance <= 0.0) return points.first()
            if (distance >= totalDistance) return points.last()
            var low = 1
            var high = cumulativeDistances.lastIndex
            while (low < high) {
                val middle = (low + high) ushr 1
                if (cumulativeDistances[middle] < distance) {
                    low = middle + 1
                } else {
                    high = middle
                }
            }
            val segmentEndIndex = low
            val segmentStartIndex = segmentEndIndex - 1
            val segmentStartDistance = cumulativeDistances[segmentStartIndex]
            val segmentLength = cumulativeDistances[segmentEndIndex] - segmentStartDistance
            if (segmentLength <= 0.0) return points[segmentEndIndex]
            return interpolate(
                points[segmentStartIndex],
                points[segmentEndIndex],
                (distance - segmentStartDistance) / segmentLength,
            )
        }
    }

    private data class GroundPoint(
        val rightMeters: Double,
        val forwardMeters: Double,
        val xFraction: Double,
        val yFraction: Double,
    )

    private data class Projection(
        val segmentIndex: Int,
        val segmentFraction: Double,
        val point: GroundPoint,
        val distanceSquared: Double,
    )

    private data class Lookahead(
        val point: GroundPoint,
        val distanceMeters: Double,
    )

    private data class MeasurementCandidate(
        val measurement: OmniCenterlineMeasurement,
        val score: Double,
    )

    private const val LOOKAHEAD_METERS = 0.35
    private const val FRAME_CENTER_FRACTION = 0.5
    private const val MIN_LOOKAHEAD_METERS = 0.15
    private const val MIN_POINT_COUNT = 5
    private const val TANGENT_POINT_SPAN = 3
    private const val MAX_DIRECTION_CHANGE_DEGREES = 80.0
    private const val MAX_PROJECTION_JUMP_METERS = 0.30
    private const val MAX_LOOKAHEAD_JUMP_METERS = 0.45
    private const val CONTINUITY_PROJECTION_WEIGHT = 2.0
    private const val CONTINUITY_LOOKAHEAD_WEIGHT = 0.5
}

/**
 * Keeps scheme B's aircraft nose on the heading captured at launch. A zero yaw-rate
 * command only asks MSDK not to rotate; it does not correct heading disturbed by
 * translation, prop wash, or controller bias.
 */
internal fun fixedHeadingHoldYawRate(
    currentHeadingDegrees: Double,
    targetHeadingDegrees: Double,
): Double =
    (
        shortestAngularDelta(currentHeadingDegrees, targetHeadingDegrees) *
            FIXED_HEADING_HOLD_PROPORTIONAL_GAIN
        ).coerceIn(
        -FIXED_HEADING_HOLD_MAX_YAW_RATE_DEGREES_PER_SECOND,
        FIXED_HEADING_HOLD_MAX_YAW_RATE_DEGREES_PER_SECOND,
    )

private const val FIXED_HEADING_HOLD_PROPORTIONAL_GAIN = 0.8
private const val FIXED_HEADING_HOLD_MAX_YAW_RATE_DEGREES_PER_SECOND = 10.0


internal fun shortestAngularDelta(fromDegrees: Double, toDegrees: Double): Double {
    var delta = toDegrees - fromDegrees
    while (delta > 180.0) delta -= 360.0
    while (delta < -180.0) delta += 360.0
    return delta
}

internal fun wrapDegrees(degrees: Double): Double {
    var wrapped = degrees % 360.0
    if (wrapped < 0.0) wrapped += 360.0
    return wrapped
}
