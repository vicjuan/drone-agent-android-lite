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
}

internal data class OmniCenterlineMeasurement(
    val tangentDegrees: Double,
    val lateralOffsetMeters: Double,
    val curvaturePerMeter: Double,
    val lookaheadXFraction: Double,
    val lookaheadYFraction: Double,
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
    private var trackingSpeed = FixedHeadingTrackingSpeed.FAST
    private var latestMeasurement: OmniCenterlineMeasurement? = null
    private var latestConfidence = 0.0
    private var lastDetectionAtNanos = 0L
    private var endpointCandidateSinceNanos = 0L
    private var endpointReached = false
    private var accumulatedTurnDegrees = 0.0
    private var startedAtNanos = 0L
    private var lastTickAtNanos = 0L
    private var latestMeasurementCapturedAtNanos = 0L
    private var lastVirtualTurnRateDegreesPerSecond = 0.0

    fun start(
        nowNanos: Long,
        trackingSpeed: FixedHeadingTrackingSpeed = FixedHeadingTrackingSpeed.FAST,
    ) {
        enabled = true
        phase = FixedHeadingLapPhase.ACQUIRING
        virtualHeadingDegrees = 0.0
        pathSpeedMetersPerSecond = 0.0
        this.trackingSpeed = trackingSpeed
        latestMeasurement = null
        latestConfidence = 0.0
        endpointCandidateSinceNanos = 0L
        endpointReached = false
        lastDetectionAtNanos = 0L
        accumulatedTurnDegrees = 0.0
        startedAtNanos = nowNanos
        lastTickAtNanos = nowNanos
        latestMeasurementCapturedAtNanos = 0L
        lastVirtualTurnRateDegreesPerSecond = 0.0
    }

    fun stop() {
        enabled = false
        phase = FixedHeadingLapPhase.DISABLED
        pathSpeedMetersPerSecond = 0.0
        trackingSpeed = FixedHeadingTrackingSpeed.FAST
        latestMeasurement = null
        endpointCandidateSinceNanos = 0L
        endpointReached = false
        accumulatedTurnDegrees = 0.0
        lastTickAtNanos = 0L
        latestMeasurementCapturedAtNanos = 0L
        lastVirtualTurnRateDegreesPerSecond = 0.0
    }

    fun observe(
        centerline: TapeCenterlinePath?,
        heightMeters: Double?,
        confidence: Double,
        endpointCandidate: Boolean,
        closedLoop: Boolean,
        nowNanos: Long,
        capturedAtNanos: Long = 0L,
    ) {
        if (!enabled) return
        val hasPhysicalEndpoint = endpointCandidate && !closedLoop
        val measurementAtNanos = capturedAtNanos.takeIf { it > 0L } ?: nowNanos
        val measurementAgeSeconds = (
            (nowNanos - measurementAtNanos).coerceAtLeast(0L) / NANOS_PER_SECOND
            ).coerceAtMost(MAX_MEASUREMENT_AGE_COMPENSATION_SECONDS)
        val virtualHeadingAtCapture = wrapDegrees(
            virtualHeadingDegrees -
                lastVirtualTurnRateDegreesPerSecond * measurementAgeSeconds,
        )
        val measurement = if (centerline == null || heightMeters == null) {
            null
        } else {
            OmniCenterline.measure(
                path = centerline,
                heightMeters = heightMeters,
                travelDirectionDegrees = virtualHeadingAtCapture,
                lookaheadMeters =
                    if (trackingSpeed == FixedHeadingTrackingSpeed.FAST) {
                        FAST_LOOKAHEAD_METERS
                    } else {
                        SLOW_LOOKAHEAD_METERS
                    },
                endpointCandidate = hasPhysicalEndpoint,
            )
        }
        latestConfidence = confidence
        if (measurement != null) {
            latestMeasurement = measurement
            lastDetectionAtNanos = nowNanos
            latestMeasurementCapturedAtNanos = measurementAtNanos
        }
        val endpointAtAircraft =
            hasPhysicalEndpoint &&
                measurement?.endpointDistanceMeters?.let {
                    it <= ENDPOINT_STOP_TOLERANCE_METERS
                } == true
        updateEndpointCandidate(endpointAtAircraft, nowNanos)
        if (phase == FixedHeadingLapPhase.STOPPED || endpointCandidateSinceNanos != 0L) return
        if (measurement == null) {
            if (phase == FixedHeadingLapPhase.TRACKING) {
                phase = FixedHeadingLapPhase.COASTING
            }
            return
        }
        if (phase == FixedHeadingLapPhase.ACQUIRING) {
            // The first valid route sample defines the initial travel direction. Waiting
            // for tape to align with image-up made an already valid circular path time out
            // before issuing its first command.
            virtualHeadingDegrees = measurement.tangentDegrees
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
            return decision(
                stopRequested = !endpointReached,
                endpointReached = endpointReached,
            )
        }

        val detectionAgeNanos = nowNanos - lastDetectionAtNanos
        if (latestMeasurement == null || detectionAgeNanos > staleWindowNanos()) {
            pathSpeedMetersPerSecond = 0.0
            phase = FixedHeadingLapPhase.STOPPED
            return decision(stopRequested = true)
        }
        if (phase == FixedHeadingLapPhase.COASTING) {
            pathSpeedMetersPerSecond = 0.0
            return decision()
        }

        val measurement = checkNotNull(latestMeasurement)
        val elapsedSeconds =
            ((nowNanos - lastTickAtNanos).coerceIn(0L, MAX_TICK_NANOS)) / 1_000_000_000.0
        lastTickAtNanos = nowNanos
        val measurementAgeSeconds = (
            (nowNanos - latestMeasurementCapturedAtNanos).coerceAtLeast(0L) /
                NANOS_PER_SECOND
            ).coerceAtMost(MAX_MEASUREMENT_AGE_COMPENSATION_SECONDS)
        val predictedTangentDegrees = wrapDegrees(
            measurement.tangentDegrees +
                Math.toDegrees(
                    pathSpeedMetersPerSecond *
                        measurement.curvaturePerMeter *
                        measurementAgeSeconds,
                ),
        )
        val tangentError = shortestAngularDelta(virtualHeadingDegrees, predictedTangentDegrees)
        if (abs(tangentError) > MAX_TRACKING_ANGLE_DEGREES) {
            pathSpeedMetersPerSecond = 0.0
            phase = FixedHeadingLapPhase.STOPPED
            return decision(stopRequested = true)
        }

        val confidenceLimitedSpeed =
            if (latestConfidence >= MIN_CONFIDENCE) {
                TARGET_SPEED_METERS_PER_SECOND
            } else {
                DEGRADED_SPEED_METERS_PER_SECOND
            }
        val trackingAuthority = minOf(
            1.0 - abs(tangentError) / SLOWDOWN_TANGENT_ERROR_DEGREES,
            1.0 - abs(measurement.lateralOffsetMeters) / SLOWDOWN_LATERAL_OFFSET_METERS,
        ).coerceIn(0.0, 1.0)
        val authorityLimitedSpeed =
            DEGRADED_SPEED_METERS_PER_SECOND +
                (confidenceLimitedSpeed - DEGRADED_SPEED_METERS_PER_SECOND) *
                    trackingAuthority
        val targetSpeed =
            minOf(trackingSpeed.targetMetersPerSecond, authorityLimitedSpeed)
        pathSpeedMetersPerSecond =
            if (targetSpeed <= pathSpeedMetersPerSecond) {
                // Route authority may disappear within one camera frame. Acceleration is
                // gradual; evidence-driven deceleration is immediate.
                targetSpeed
            } else {
                moveToward(
                    pathSpeedMetersPerSecond,
                    targetSpeed,
                    MAX_ACCELERATION_METERS_PER_SECOND_SQUARED * elapsedSeconds,
                )
            }
        val virtualTurnRate = (
            Math.toDegrees(pathSpeedMetersPerSecond * measurement.curvaturePerMeter) +
                TANGENT_FEEDBACK_GAIN * tangentError
            ).coerceIn(
            -MAX_VIRTUAL_TURN_RATE_DEGREES_PER_SECOND,
            MAX_VIRTUAL_TURN_RATE_DEGREES_PER_SECOND,
        )
        val turnDegrees = virtualTurnRate * elapsedSeconds
        virtualHeadingDegrees = wrapDegrees(virtualHeadingDegrees + turnDegrees)
        lastVirtualTurnRateDegreesPerSecond = virtualTurnRate
        accumulatedTurnDegrees += turnDegrees
        if (hasCompletedFixedHeadingLap(accumulatedTurnDegrees)) {
            pathSpeedMetersPerSecond = 0.0
            endpointReached = true
            phase = FixedHeadingLapPhase.STOPPED
            return decision(endpointReached = true)
        }

        val headingRadians = Math.toRadians(virtualHeadingDegrees)
        val correction = (
            LATERAL_FEEDBACK_GAIN_PER_SECOND * measurement.lateralOffsetMeters
            ).coerceIn(
            -MAX_LATERAL_CORRECTION_METERS_PER_SECOND,
            MAX_LATERAL_CORRECTION_METERS_PER_SECOND,
        )
        val forward =
            pathSpeedMetersPerSecond * cos(headingRadians) - correction * sin(headingRadians)
        val right =
            pathSpeedMetersPerSecond * sin(headingRadians) + correction * cos(headingRadians)
        phase = FixedHeadingLapPhase.TRACKING
        return decision(forward = forward, right = right)
    }

    private fun updateEndpointCandidate(candidate: Boolean, nowNanos: Long) {
        if (phase != FixedHeadingLapPhase.TRACKING && phase != FixedHeadingLapPhase.COASTING) {
            endpointCandidateSinceNanos = 0L
            return
        }
        if (!candidate) {
            endpointCandidateSinceNanos = 0L
            return
        }
        pathSpeedMetersPerSecond = 0.0
        phase = FixedHeadingLapPhase.COASTING
        if (endpointCandidateSinceNanos == 0L) {
            endpointCandidateSinceNanos = nowNanos
        } else if (nowNanos - endpointCandidateSinceNanos >= ENDPOINT_CONFIRMATION_NANOS) {
            endpointReached = true
            phase = FixedHeadingLapPhase.STOPPED
        }
    }

    private fun staleWindowNanos(): Long {
        val speed = pathSpeedMetersPerSecond.coerceAtLeast(0.05)
        return (MAX_BLIND_DISTANCE_METERS / speed * 1_000_000_000.0).toLong()
            .coerceIn(MIN_STALE_NANOS, MAX_STALE_NANOS)
    }

    private fun decision(
        forward: Double = 0.0,
        right: Double = 0.0,
        stopRequested: Boolean = false,
        endpointReached: Boolean = false,
    ): FixedHeadingLapDecision = FixedHeadingLapDecision(
        phase = phase,
        forwardMetersPerSecond = forward,
        rightMetersPerSecond = right,
        virtualHeadingDegrees = virtualHeadingDegrees,
        tangentDegrees = latestMeasurement?.tangentDegrees,
        lateralOffsetMeters = latestMeasurement?.lateralOffsetMeters,
        curvaturePerMeter = latestMeasurement?.curvaturePerMeter,
        stopRequested = stopRequested,
        endpointReached = endpointReached,
    )

    private fun moveToward(current: Double, target: Double, maximumDelta: Double): Double =
        when {
            target > current -> minOf(target, current + maximumDelta)
            target < current -> maxOf(target, current - maximumDelta)
            else -> current
        }

    internal companion object {
        const val TARGET_SPEED_METERS_PER_SECOND = 0.90
        const val DEGRADED_SPEED_METERS_PER_SECOND = 0.22
        const val MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.60
        const val MAX_VIRTUAL_TURN_RATE_DEGREES_PER_SECOND = 90.0
        const val TANGENT_FEEDBACK_GAIN = 0.45
        const val LATERAL_FEEDBACK_GAIN_PER_SECOND = 0.8
        const val MAX_LATERAL_CORRECTION_METERS_PER_SECOND = 0.10
        const val MIN_CONFIDENCE = 0.60
        const val MAX_TRACKING_ANGLE_DEGREES = 70.0
        const val COMPLETE_LAP_DEGREES = 360.0
        const val ENDPOINT_CONFIRMATION_NANOS = 500_000_000L
        const val ENDPOINT_STOP_TOLERANCE_METERS = 0.08
        const val ACQUISITION_TIMEOUT_NANOS = 8_000_000_000L
        const val MAX_BLIND_DISTANCE_METERS = 0.10
        const val MIN_STALE_NANOS = 200_000_000L
        const val MAX_STALE_NANOS = 400_000_000L
        const val MAX_TICK_NANOS = 250_000_000L
        const val FAST_LOOKAHEAD_METERS = 0.55
        const val SLOW_LOOKAHEAD_METERS = 0.35
        const val SLOWDOWN_TANGENT_ERROR_DEGREES = 35.0
        const val SLOWDOWN_LATERAL_OFFSET_METERS = 0.25
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
    ): OmniCenterlineMeasurement? {
        if (!heightMeters.isFinite() || heightMeters <= 0.0 || path.pointCount < MIN_POINT_COUNT) return null
        val aspectRatio = path.sourceWidth.toDouble() / path.sourceHeight
        val verticalHalfFovTangent =
            TapeTrackingController.CAMERA_DIAGONAL_HALF_FOV_TANGENT / sqrt(1.0 + aspectRatio * aspectRatio)
        val frameHeightMeters = 2.0 * heightMeters * verticalHalfFovTangent
        val frameWidthMeters = frameHeightMeters * aspectRatio
        val directionRadians = Math.toRadians(travelDirectionDegrees)
        val entryOffsetMeters = ENTRY_OFFSET_FRAME_HEIGHT_FRACTION * frameHeightMeters
        val targetXFraction =
            FRAME_CENTER_FRACTION -
                sin(directionRadians) * entryOffsetMeters / frameWidthMeters
        val targetYFraction =
            FRAME_CENTER_FRACTION +
                cos(directionRadians) * entryOffsetMeters / frameHeightMeters
        val points = List(path.pointCount) { index ->
            GroundPoint(
                rightMeters = (path.xFractions[index] - targetXFraction) * frameWidthMeters,
                forwardMeters = (targetYFraction - path.yFractions[index]) * frameHeightMeters,
                xFraction = path.xFractions[index].toDouble(),
                yFraction = path.yFractions[index].toDouble(),
            )
        }

        var nearest: Projection? = null
        for (index in 0 until points.lastIndex) {
            val start = points[index]
            val end = points[index + 1]
            val deltaRight = end.rightMeters - start.rightMeters
            val deltaForward = end.forwardMeters - start.forwardMeters
            val lengthSquared = deltaRight * deltaRight + deltaForward * deltaForward
            if (lengthSquared <= 1e-9) continue
            val fraction = (
                -(start.rightMeters * deltaRight + start.forwardMeters * deltaForward) / lengthSquared
                ).coerceIn(0.0, 1.0)
            val projected = interpolate(start, end, fraction)
            val distanceSquared =
                projected.rightMeters * projected.rightMeters + projected.forwardMeters * projected.forwardMeters
            if (nearest == null || distanceSquared < nearest.distanceSquared) {
                nearest = Projection(index, fraction, projected, distanceSquared)
            }
        }
        val projection = nearest ?: return null
        val tangentStart = points[(projection.segmentIndex - TANGENT_POINT_SPAN).coerceAtLeast(0)]
        val tangentEnd = points[(projection.segmentIndex + 1 + TANGENT_POINT_SPAN).coerceAtMost(points.lastIndex)]
        val baseTangent = directionDegrees(
            tangentEnd.rightMeters - tangentStart.rightMeters,
            tangentEnd.forwardMeters - tangentStart.forwardMeters,
        ) ?: return null
        val reverseTangent = wrapDegrees(baseTangent + 180.0)
        val followsIncreasingIndices =
            abs(shortestAngularDelta(travelDirectionDegrees, baseTangent)) <=
                abs(shortestAngularDelta(travelDirectionDegrees, reverseTangent))
        val tangent = if (followsIncreasingIndices) baseTangent else reverseTangent
        if (abs(shortestAngularDelta(travelDirectionDegrees, tangent)) > MAX_DIRECTION_CHANGE_DEGREES) return null

        val remainingPathMeters =
            remainingPathDistance(points, projection, followsIncreasingIndices)
        val lookahead =
            lookaheadPoint(
                points = points,
                projection = projection,
                increasing = followsIncreasingIndices,
                targetDistance = lookaheadMeters,
                acceptPathEnd = endpointCandidate,
            ) ?: return null
        val headingRadians = Math.toRadians(travelDirectionDegrees)
        val rightNormalRight = cos(headingRadians)
        val rightNormalForward = -sin(headingRadians)
        val lateralLookahead =
            lookahead.rightMeters * rightNormalRight + lookahead.forwardMeters * rightNormalForward
        val lookaheadDistanceSquared =
            lookahead.rightMeters * lookahead.rightMeters + lookahead.forwardMeters * lookahead.forwardMeters
        if (lookaheadDistanceSquared < MIN_LOOKAHEAD_METERS * MIN_LOOKAHEAD_METERS) return null
        val tangentRadians = Math.toRadians(tangent)
        val pathRightNormalRight = cos(tangentRadians)
        val pathRightNormalForward = -sin(tangentRadians)
        val lateralOffset =
            projection.point.rightMeters * pathRightNormalRight +
                projection.point.forwardMeters * pathRightNormalForward
        return OmniCenterlineMeasurement(
            tangentDegrees = tangent,
            lateralOffsetMeters = lateralOffset,
            curvaturePerMeter = 2.0 * lateralLookahead / lookaheadDistanceSquared,
            lookaheadXFraction = lookahead.xFraction,
            lookaheadYFraction = lookahead.yFraction,
            endpointDistanceMeters =
                if (endpointCandidate) remainingPathMeters - entryOffsetMeters else null,
        )
    }

    private fun lookaheadPoint(
        points: List<GroundPoint>,
        projection: Projection,
        increasing: Boolean,
        targetDistance: Double,
        acceptPathEnd: Boolean,
    ): GroundPoint? {
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
                return interpolate(current, next, (targetDistance - accumulated) / segmentDistance)
            }
            accumulated += segmentDistance
            current = next
            index += if (increasing) 1 else -1
        }
        return if (acceptPathEnd) current else null
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

    private const val LOOKAHEAD_METERS = 0.35
    private const val FRAME_CENTER_FRACTION = 0.5
    private const val ENTRY_OFFSET_FRAME_HEIGHT_FRACTION = 0.44
    private const val MIN_LOOKAHEAD_METERS = 0.15
    private const val MIN_POINT_COUNT = 5
    private const val TANGENT_POINT_SPAN = 3
    private const val MAX_DIRECTION_CHANGE_DEGREES = 80.0
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

internal fun hasCompletedFixedHeadingLap(accumulatedTurnDegrees: Double): Boolean =
    abs(accumulatedTurnDegrees) >= FixedHeadingLapController.COMPLETE_LAP_DEGREES

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
