package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Exact-frame geometry/VO adjunct to the original racer, never a flight controller.
 * Unavailable evidence withdraws only the optional tangent. Confirmation is measured
 * in source time; callback delays and prediction queries cannot move its barrier.
 */
internal class RacingCircleGuidance(private val config: CircleModelConfig = CircleModelConfig()) {
    private val fitter = FixedRadiusCircleFitter(config)
    private val estimator = CircleStateEstimator(config)
    private var startedAtNanos = -1L
    private var lastSourceAtNanos = 0L
    private var lastReceiptAtNanos = 0L
    private var confirmationBarrierAtNanos = 0L
    private var blockedStatus = "WAITING"
    private var blockedReason: String? = "not_started"
    private var pendingInvalidationReason: String? = null
    private var pendingInvalidationAtNanos = 0L
    private var fit: CircleArcObservation? = null
    private var fitReason: String? = null
    private var scale: CircleImageScale? = null
    private var posterior = CircleEstimateResult()
    private var priorAcceptedVisionCount = 0
    private var priorRejectedVisionCount = 0
    private var priorAcceptedVelocityCount = 0
    private var priorLateReplayCount = 0
    private var estimatorResetCount = 0
    private var lastRejectionReason: String? = null
    private var lastRejectionSourceAtNanos = 0L
    private var rejectionCount = 0

    fun reset(startedAtNanos: Long) {
        estimator.reset(startedAtNanos)
        this.startedAtNanos = startedAtNanos
        lastSourceAtNanos = 0L
        lastReceiptAtNanos = startedAtNanos
        confirmationBarrierAtNanos = startedAtNanos
        blockedStatus = "WAITING"
        blockedReason = "confirming_circle"
        pendingInvalidationReason = null
        pendingInvalidationAtNanos = 0L
        fit = null
        fitReason = null
        scale = null
        posterior = CircleEstimateResult()
        priorAcceptedVisionCount = 0
        priorRejectedVisionCount = 0
        priorAcceptedVelocityCount = 0
        priorLateReplayCount = 0
        estimatorResetCount = 0
        lastRejectionReason = null
        lastRejectionSourceAtNanos = 0L
        rejectionCount = 0
    }

    fun observe(
        observation: TapeTrackingObservation,
        forwardMetersPerSecond: Double,
        rightMetersPerSecond: Double,
        nowNanos: Long,
    ) {
        resolveInvalidation(nowNanos)
        val source = observation.capturedAtNanos
        val timestampReason = when {
            startedAtNanos < 0L -> "not_started"
            nowNanos < lastReceiptAtNanos -> "reversed_receipt_time"
            source <= 0L || source <= startedAtNanos -> "frame_before_attempt"
            source > nowNanos -> "future_frame"
            source <= lastSourceAtNanos -> "replayed_or_out_of_order_frame"
            else -> null
        }
        if (timestampReason != null) {
            recordRejection(timestampReason, source)
            return
        }
        lastSourceAtNanos = source
        lastReceiptAtNanos = nowNanos
        scale = observation.circleImageScale
        fit = null
        fitReason = when {
            nowNanos - source > config.maximumVisionAgeNanos -> "stale_image"
            nowNanos - source > config.maximumVelocityAgeNanos -> "stale_velocity"
            else -> untrustedObservationReason(observation)
                ?: if (scale == null) "missing_image_scale" else null
        }
        if (fitReason != null) {
            reject(if (fitReason!!.startsWith("stale_")) "STALE" else "FIT_REJECTED", fitReason!!, source)
            return
        }
        val frameScale = checkNotNull(scale)
        val fitted = fitter.fit(checkNotNull(observation.centerline), frameScale, source, nowNanos)
        fit = fitted.observation
        fitReason = fitted.reason
        val arc = fitted.observation
        if (arc == null) {
            reject("FIT_REJECTED", fitted.reason ?: "circle_fit_rejected", source)
            return
        }
        if (!forwardMetersPerSecond.isFinite() || !rightMetersPerSecond.isFinite()) {
            reject("EKF_REJECTED", "invalid_velocity", source)
            return
        }

        // A retained stale posterior must not gate every future circle forever.
        // Restart at the NEW source frame, not at receipt time. Thus delayed but
        // fresh frames remain admissible and no acquisition deadline exists.
        val oldVision = posterior.lastVisionAtNanos
        val oldVelocity = posterior.lastVelocityAtNanos
        if ((oldVision > 0L && source - oldVision > config.maximumVisionAgeNanos) ||
            (oldVelocity > 0L && source - oldVelocity > config.maximumVelocityAgeNanos) ||
            posterior.reason == "history_overflow" || posterior.reason == "invalid_covariance"
        ) {
            priorAcceptedVisionCount += posterior.acceptedVisionCount
            priorRejectedVisionCount += posterior.rejectedVisionCount
            priorAcceptedVelocityCount += posterior.acceptedVelocityCount
            priorLateReplayCount += posterior.lateReplayCount
            estimator.reset(source)
            confirmationBarrierAtNanos = source - 1L
            estimatorResetCount += 1
        }
        val heading = Math.toRadians(frameScale.headingDegrees % 360.0)
        val c = cos(heading)
        val s = sin(heading)
        val velocityAccepted = estimator.updateVelocity(
            c * forwardMetersPerSecond - s * rightMetersPerSecond,
            s * forwardMetersPerSecond + c * rightMetersPerSecond,
            source,
            nowNanos,
        )
        val visionAccepted = estimator.observe(arc, nowNanos)
        posterior = estimator.estimate(nowNanos, frameScale.headingDegrees, frameScale.headingAtNanos)
        if (!velocityAccepted || !visionAccepted) {
            reject("EKF_REJECTED", posterior.reason ?: if (!velocityAccepted) "velocity_innovation_rejected" else "vision_innovation_rejected", source)
            return
        }
        blockedReason = null
        blockedStatus = "WAITING"
    }

    /** No receipt clock is invented here; validate a pending source against the next real clock. */
    fun invalidate(reason: String, sourceAtNanos: Long) {
        if (pendingInvalidationReason == null || sourceAtNanos >= pendingInvalidationAtNanos) {
            pendingInvalidationReason = reason
            pendingInvalidationAtNanos = sourceAtNanos
        }
    }

    fun predict(
        nowNanos: Long,
        headingDegrees: Double?,
        headingAtNanos: Long,
        turnDirection: Double,
    ): RacingCircleGuidanceResult {
        resolveInvalidation(nowNanos)
        val result = estimator.estimate(nowNanos, headingDegrees ?: Double.NaN, headingAtNanos)
        val estimate = result.estimate
        val distance = estimate?.let { hypot(it.centerForwardMeters, it.centerRightMeters) }
        val directionValid = turnDirection == -1.0 || turnDirection == 1.0
        val tangent = if (estimate != null && directionValid && distance != null && distance > 0.0) {
            Math.toDegrees(atan2(-turnDirection * estimate.centerForwardMeters, turnDirection * estimate.centerRightMeters))
        } else null
        val alongTrack = if (estimate != null && directionValid && distance != null && distance > 0.0) {
            turnDirection * (estimate.forwardVelocityMetersPerSecond * estimate.centerRightMeters -
                estimate.rightVelocityMetersPerSecond * estimate.centerForwardMeters) / distance
        } else null
        val reason = when {
            startedAtNanos < 0L -> "not_started"
            nowNanos < lastReceiptAtNanos -> "reversed_query_time"
            blockedReason != null -> blockedReason
            headingDegrees == null || !headingDegrees.isFinite() || headingAtNanos <= 0L -> "missing_heading"
            estimate == null -> result.reason ?: "estimate_unavailable"
            !directionValid -> "turn_direction_unestablished"
            distance == null || !distance.isFinite() || abs(distance - config.radiusMeters) > MAXIMUM_RADIAL_ERROR_METERS -> "circle_radial_bound"
            tangent == null || !tangent.isFinite() || abs(tangent) > MAXIMUM_TANGENT_ERROR_DEGREES -> "circle_tangent_bound"
            !estimator.hasConfirmedVisionSince(confirmationBarrierAtNanos, MINIMUM_CONFIRMATION_FRAMES, MINIMUM_CONFIRMATION_NANOS) -> "confirming_circle"
            else -> null
        }
        val status = when {
            reason == null -> "AVAILABLE"
            blockedReason != null -> blockedStatus
            reason.endsWith("_expired") || reason.startsWith("stale_") -> "STALE"
            reason == "not_started" || reason == "confirming_circle" || reason == "turn_direction_unestablished" ||
                reason == "missing_heading" || reason.endsWith("_unavailable") -> "WAITING"
            else -> "EKF_REJECTED"
        }
        return RacingCircleGuidanceResult(
            tangentDegrees = if (reason == null) tangent else null,
            diagnostics = CircleModelDiagnostics(
                guidanceActive = false,
                status = status,
                fitResidualRmsMeters = fit?.residualRmsMeters,
                fitInlierCount = fit?.inlierCount ?: 0,
                fitArcSpanRadians = fit?.arcSpanRadians,
                imageScaleAtNanos = scale?.sampleAtNanos ?: 0L,
                metersPerPixel = scale?.metersPerPixel?.takeIf { it.isFinite() && it > 0.0 },
                lastVisionAtNanos = result.lastVisionAtNanos,
                lastVelocityAtNanos = result.lastVelocityAtNanos,
                estimateAtNanos = estimate?.estimateAtNanos ?: 0L,
                centerForwardMeters = estimate?.centerForwardMeters,
                centerRightMeters = estimate?.centerRightMeters,
                radialErrorMeters = distance?.minus(config.radiusMeters),
                tangentErrorDegrees = tangent,
                alongTrackVelocityMetersPerSecond = alongTrack,
                positionStdMeters = estimate?.positionStdMeters,
                velocityStdMetersPerSecond = estimate?.velocityStdMetersPerSecond,
                predictionSeconds = estimate?.predictionSeconds ?: 0.0,
                acceptedVisionCount = priorAcceptedVisionCount + result.acceptedVisionCount,
                rejectedVisionCount = priorRejectedVisionCount + result.rejectedVisionCount,
                acceptedVelocityCount = priorAcceptedVelocityCount + result.acceptedVelocityCount,
                lateReplayCount = priorLateReplayCount + result.lateReplayCount,
                lastInnovationSquared = result.lastInnovationSquared,
                fitReason = fitReason,
                estimatorReason = reason ?: result.reason,
                sourceAtNanos = lastSourceAtNanos,
                receivedAtNanos = lastReceiptAtNanos,
                lastRejectionReason = lastRejectionReason,
                lastRejectionSourceAtNanos = lastRejectionSourceAtNanos,
                rejectionCount = rejectionCount,
                estimatorResetCount = estimatorResetCount,
            ),
        )
    }

    private fun resolveInvalidation(nowNanos: Long) {
        val reason = pendingInvalidationReason ?: return
        val source = pendingInvalidationAtNanos
        pendingInvalidationReason = null
        when {
            source <= 0L || source > nowNanos -> recordRejection("invalid_or_future_invalidation", source)
            startedAtNanos < 0L || source <= startedAtNanos || source < lastSourceAtNanos ->
                recordRejection("old_invalidation:$reason", source)
            else -> {
                lastSourceAtNanos = source
                lastReceiptAtNanos = maxOf(lastReceiptAtNanos, nowNanos)
                fitReason = reason
                reject("FIT_REJECTED", reason, source)
            }
        }
    }

    private fun reject(status: String, reason: String, sourceAtNanos: Long) {
        confirmationBarrierAtNanos = maxOf(confirmationBarrierAtNanos, sourceAtNanos)
        blockedStatus = status
        blockedReason = reason
        recordRejection(reason, sourceAtNanos)
    }

    private fun recordRejection(reason: String, sourceAtNanos: Long) {
        if (reason == lastRejectionReason && sourceAtNanos == lastRejectionSourceAtNanos) return
        lastRejectionReason = reason
        lastRejectionSourceAtNanos = sourceAtNanos
        rejectionCount += 1
    }

    internal companion object {
        fun untrustedObservationReason(observation: TapeTrackingObservation?): String? = when {
            observation == null -> "NO_CIRCLE_OBSERVATION"
            !observation.confidence.isFinite() || observation.confidence < MINIMUM_CONFIDENCE -> "LOW_CIRCLE_CONFIDENCE"
            observation.centerline == null -> "NO_CIRCLE_CENTERLINE"
            else -> FixedRadiusCircleFitter.untrustedPathReason(observation.centerline)
        }

        private const val MINIMUM_CONFIDENCE = 0.60
        private const val MINIMUM_CONFIRMATION_FRAMES = 3
        private const val MINIMUM_CONFIRMATION_NANOS = 100_000_000L
        private const val MAXIMUM_RADIAL_ERROR_METERS = 0.25
        private const val MAXIMUM_TANGENT_ERROR_DEGREES = 45.0
    }
}
