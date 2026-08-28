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
    STOPPED,
}

internal enum class FixedHeadingTrackingSpeed(
    val targetMetersPerSecond: Double,
) {
    SLOW(0.20),
    FAST(0.90),
    BOOST(1.25),
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
)

/**
 * Fixed-aircraft-heading path follower. The aircraft yaw stays zero while a body-frame
 * velocity vector turns around the detected tape. It deliberately owns no DJI state.
 */
internal class FixedHeadingLapController {
    var enabled: Boolean = false
        private set

    private var phase = FixedHeadingLapPhase.DISABLED
    private var virtualHeadingDegrees = 0.0
    private var pathSpeedMetersPerSecond = 0.0
    private var pathAccelerationMetersPerSecondSquared = 0.0
    private var trackingSpeed = FixedHeadingTrackingSpeed.FAST
    private var latestMeasurement: OmniCenterlineMeasurement? = null
    private var latestConfidence = 0.0
    private var lastDetectionAtNanos = 0L
    private var startedAtNanos = 0L
    private var lastTickAtNanos = 0L
    private var latestMeasurementCapturedAtNanos = 0L
    private var lastVirtualTurnRateDegreesPerSecond = 0.0
    private var pendingRecoveryMeasurement: OmniCenterlineMeasurement? = null
    private var lossObservedAtNanos = 0L

    fun start(
        nowNanos: Long,
        trackingSpeed: FixedHeadingTrackingSpeed = FixedHeadingTrackingSpeed.FAST,
    ) {
        enabled = true
        phase = FixedHeadingLapPhase.ACQUIRING
        virtualHeadingDegrees = 0.0
        pathSpeedMetersPerSecond = 0.0
        pathAccelerationMetersPerSecondSquared = 0.0
        this.trackingSpeed = trackingSpeed
        latestMeasurement = null
        latestConfidence = 0.0
        lastDetectionAtNanos = 0L
        startedAtNanos = nowNanos
        lastTickAtNanos = nowNanos
        latestMeasurementCapturedAtNanos = 0L
        lastVirtualTurnRateDegreesPerSecond = 0.0
        pendingRecoveryMeasurement = null
        lossObservedAtNanos = 0L
    }

    fun stop() {
        enabled = false
        phase = FixedHeadingLapPhase.DISABLED
        pathSpeedMetersPerSecond = 0.0
        pathAccelerationMetersPerSecondSquared = 0.0
        trackingSpeed = FixedHeadingTrackingSpeed.FAST
        latestMeasurement = null
        lastTickAtNanos = 0L
        latestMeasurementCapturedAtNanos = 0L
        lastVirtualTurnRateDegreesPerSecond = 0.0
        pendingRecoveryMeasurement = null
        lossObservedAtNanos = 0L
    }

    fun observe(
        centerline: TapeCenterlinePath?,
        heightMeters: Double?,
        confidence: Double,
        nowNanos: Long,
        capturedAtNanos: Long = 0L,
    ) {
        if (!enabled) return
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
        }
        if (measurement == null) {
            if (centerline == null || heightMeters == null) {
                pendingRecoveryMeasurement = null
            }
            if (phase == FixedHeadingLapPhase.TRACKING) {
                phase = FixedHeadingLapPhase.COASTING
                lossObservedAtNanos = nowNanos
                pathAccelerationMetersPerSecondSquared = 0.0
            }
            return
        }
        if (phase == FixedHeadingLapPhase.ACQUIRING) {
            // The first valid lookahead vector is already a closed-loop steering
            // target: it contains both path direction and cross-track correction.
            virtualHeadingDegrees = directionToLookahead(measurement)
            phase = FixedHeadingLapPhase.TRACKING
            lastTickAtNanos = nowNanos
        } else if (phase == FixedHeadingLapPhase.COASTING) {
            phase = FixedHeadingLapPhase.TRACKING
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

        val confidenceLimitedSpeed =
            if (latestConfidence >= MIN_CONFIDENCE) {
                trackingSpeed.targetMetersPerSecond
            } else {
                DEGRADED_SPEED_METERS_PER_SECOND
            }
        val trackingAuthority =
            if (trackingSpeed == FixedHeadingTrackingSpeed.BOOST) {
                minOf(
                    authorityAfterMargin(
                        magnitude = abs(guidanceError),
                        fullAuthorityLimit = BOOST_FULL_SPEED_GUIDANCE_ERROR_DEGREES,
                        zeroAuthorityLimit = SLOWDOWN_GUIDANCE_ERROR_DEGREES,
                    ),
                    authorityAfterMargin(
                        magnitude = abs(measurement.lateralOffsetMeters),
                        fullAuthorityLimit = BOOST_FULL_SPEED_LATERAL_OFFSET_METERS,
                        zeroAuthorityLimit = SLOWDOWN_LATERAL_OFFSET_METERS,
                    ),
                )
            } else {
                minOf(
                    1.0 - abs(guidanceError) / SLOWDOWN_GUIDANCE_ERROR_DEGREES,
                    1.0 - abs(measurement.lateralOffsetMeters) / SLOWDOWN_LATERAL_OFFSET_METERS,
                ).coerceIn(0.0, 1.0)
            }
        val authorityLimitedSpeed = confidenceLimitedSpeed * trackingAuthority
        val lookaheadLimitedSpeed =
            trackingSpeed.targetMetersPerSecond *
                (measurement.lookaheadDistanceMeters / configuredLookaheadMeters())
                    .coerceIn(0.0, 1.0)
        val targetSpeed = minOf(
            trackingSpeed.targetMetersPerSecond,
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
        val turnDegrees =
            guidanceError.coerceIn(-maximumTurnDegrees, maximumTurnDegrees)
        virtualHeadingDegrees = wrapDegrees(virtualHeadingDegrees + turnDegrees)
        lastVirtualTurnRateDegreesPerSecond =
            if (elapsedSeconds > 0.0) turnDegrees / elapsedSeconds else 0.0

        val headingRadians = Math.toRadians(virtualHeadingDegrees)
        val correction =
            if (pathSpeedMetersPerSecond == 0.0) {
                (
                    LATERAL_FEEDBACK_GAIN_PER_SECOND * measurement.lateralOffsetMeters
                    ).coerceIn(
                    -MAX_LATERAL_CORRECTION_METERS_PER_SECOND,
                    MAX_LATERAL_CORRECTION_METERS_PER_SECOND,
                )
            } else {
                0.0
            }
        val correctionHeadingRadians = Math.toRadians(measurement.tangentDegrees)
        val forward =
            pathSpeedMetersPerSecond * cos(headingRadians) -
                correction * sin(correctionHeadingRadians)
        val right =
            pathSpeedMetersPerSecond * sin(headingRadians) +
                correction * cos(correctionHeadingRadians)
        phase = FixedHeadingLapPhase.TRACKING
        return decision(forward = forward, right = right)
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
        val committedBoost =
            trackingSpeed == FixedHeadingTrackingSpeed.BOOST &&
                latestConfidence >= BOOST_COMMIT_CONFIDENCE
        val maximumBlindDistanceMeters =
            if (committedBoost) BOOST_MAX_BLIND_DISTANCE_METERS else MAX_BLIND_DISTANCE_METERS
        val maximumStaleNanos =
            if (committedBoost) BOOST_MAX_STALE_NANOS else MAX_STALE_NANOS
        return (maximumBlindDistanceMeters / speed * 1_000_000_000.0).toLong()
            .coerceIn(MIN_STALE_NANOS, maximumStaleNanos)
    }

    private fun configuredLookaheadMeters(): Double =
        when (trackingSpeed) {
            FixedHeadingTrackingSpeed.SLOW -> SLOW_LOOKAHEAD_METERS
            FixedHeadingTrackingSpeed.FAST -> FAST_LOOKAHEAD_METERS
            FixedHeadingTrackingSpeed.BOOST -> BOOST_LOOKAHEAD_METERS
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
    ): FixedHeadingLapDecision = FixedHeadingLapDecision(
        phase = phase,
        forwardMetersPerSecond = forward,
        rightMetersPerSecond = right,
        virtualHeadingDegrees = virtualHeadingDegrees,
        tangentDegrees = latestMeasurement?.tangentDegrees,
        lateralOffsetMeters = latestMeasurement?.lateralOffsetMeters,
        curvaturePerMeter = latestMeasurement?.curvaturePerMeter,
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

        val speedError = targetSpeed - pathSpeedMetersPerSecond
        val maximumAcceleration =
            if (trackingSpeed == FixedHeadingTrackingSpeed.BOOST) {
                BOOST_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
            } else {
                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
            }
        val maximumJerk =
            if (trackingSpeed == FixedHeadingTrackingSpeed.BOOST) {
                BOOST_MAX_JERK_METERS_PER_SECOND_CUBED
            } else {
                MAX_JERK_METERS_PER_SECOND_CUBED
            }
        val desiredAcceleration = (
            speedError * SPEED_ERROR_RESPONSE_PER_SECOND
            ).coerceIn(
            -MAX_DECELERATION_METERS_PER_SECOND_SQUARED,
            maximumAcceleration,
        )
        pathAccelerationMetersPerSecondSquared = moveToward(
            pathAccelerationMetersPerSecondSquared,
            desiredAcceleration,
            maximumJerk * elapsedSeconds,
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
        const val DEGRADED_SPEED_METERS_PER_SECOND = 0.22
        const val MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.60
        const val BOOST_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1.00
        const val MAX_DECELERATION_METERS_PER_SECOND_SQUARED = 1.20
        const val MAX_BLIND_DECELERATION_METERS_PER_SECOND_SQUARED = 1.50
        const val MAX_JERK_METERS_PER_SECOND_CUBED = 2.00
        const val BOOST_MAX_JERK_METERS_PER_SECOND_CUBED = 4.00
        const val SPEED_ERROR_RESPONSE_PER_SECOND = 3.00
        const val MAX_VIRTUAL_TURN_RATE_DEGREES_PER_SECOND = 60.0
        const val LATERAL_FEEDBACK_GAIN_PER_SECOND = 0.8
        const val MAX_LATERAL_CORRECTION_METERS_PER_SECOND = 0.25
        const val MIN_CONFIDENCE = 0.60
        const val BOOST_COMMIT_CONFIDENCE = 0.75
        const val MAX_TRACKING_ANGLE_DEGREES = 70.0
        const val ACQUISITION_TIMEOUT_NANOS = 8_000_000_000L
        const val MAX_BLIND_DISTANCE_METERS = 0.10
        const val BOOST_MAX_BLIND_DISTANCE_METERS = 0.22
        const val MIN_STALE_NANOS = 80_000_000L
        const val MAX_STALE_NANOS = 150_000_000L
        const val BOOST_MAX_STALE_NANOS = 180_000_000L
        const val REACQUISITION_TIMEOUT_NANOS = 1_200_000_000L
        const val MAX_TICK_NANOS = 250_000_000L
        const val FAST_LOOKAHEAD_METERS = 0.35
        const val BOOST_LOOKAHEAD_METERS = 0.45
        const val SLOW_LOOKAHEAD_METERS = 0.25
        const val SLOWDOWN_GUIDANCE_ERROR_DEGREES = 35.0
        const val BOOST_FULL_SPEED_GUIDANCE_ERROR_DEGREES = 10.0
        const val MAX_RECOVERY_GUIDANCE_ERROR_DEGREES = 35.0
        const val MAX_RECOVERY_LATERAL_OFFSET_METERS = 0.20
        const val SLOWDOWN_LATERAL_OFFSET_METERS = 0.25
        const val BOOST_FULL_SPEED_LATERAL_OFFSET_METERS = 0.08
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

        var bestCandidate: MeasurementCandidate? = null
        for (index in 0 until points.lastIndex) {
            val projection = projectOntoSegment(points, index) ?: continue
            val measurement = measurementForProjection(
                points = points,
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
        points: List<GroundPoint>,
        projection: Projection,
        travelDirectionDegrees: Double,
        requestedLookaheadMeters: Double,
        endpointCandidate: Boolean,
    ): OmniCenterlineMeasurement? {
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
            remainingPathDistance(points, projection, followsIncreasingIndices)
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
            points = points,
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
        points: List<GroundPoint>,
        projection: Projection,
        increasing: Boolean,
        targetDistance: Double,
    ): Lookahead? {
        var current = projection.point
        var accumulated = 0.0
        var index = if (increasing) projection.segmentIndex + 1 else projection.segmentIndex
        while (index in points.indices) {
            val next = points[index]
            val segmentDistance = hypot(
                next.rightMeters - current.rightMeters,
                next.forwardMeters - current.forwardMeters,
            )
            if (segmentDistance > 0.0 && accumulated + segmentDistance >= targetDistance) {
                return Lookahead(
                    point = interpolate(
                        current,
                        next,
                        (targetDistance - accumulated) / segmentDistance,
                    ),
                    distanceMeters = targetDistance,
                )
            }
            accumulated += segmentDistance
            current = next
            index += if (increasing) 1 else -1
        }
        return if (accumulated >= MIN_LOOKAHEAD_METERS) {
            Lookahead(point = current, distanceMeters = accumulated)
        } else {
            null
        }
    }

    private fun remainingPathDistance(
        points: List<GroundPoint>,
        projection: Projection,
        increasing: Boolean,
    ): Double {
        var current = projection.point
        var distance = 0.0
        var index = if (increasing) projection.segmentIndex + 1 else projection.segmentIndex
        while (index in points.indices) {
            val next = points[index]
            distance += hypot(
                next.rightMeters - current.rightMeters,
                next.forwardMeters - current.forwardMeters,
            )
            current = next
            index += if (increasing) 1 else -1
        }
        return distance
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
