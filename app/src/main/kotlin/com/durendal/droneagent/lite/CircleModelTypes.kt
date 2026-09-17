package com.durendal.droneagent.lite

/** Exact-frame VO scale; no height/FOV fallback or calibrated camera-pose claim. */
internal data class CircleImageScale(
    val metersPerPixel: Double,
    val analysisWidth: Int,
    val analysisHeight: Int,
    val sampleAtNanos: Long,
    val headingDegrees: Double,
    val headingAtNanos: Long,
)

/** Fixed-radius fit in horizontal aircraft-heading axes: forward/right, metres. */
internal data class CircleArcObservation(
    val centerForwardMeters: Double,
    val centerRightMeters: Double,
    val centerForwardVariance: Double,
    val centerRightVariance: Double,
    val centerCovariance: Double,
    val residualRmsMeters: Double,
    val inlierCount: Int,
    val pointCount: Int,
    val arcSpanRadians: Double,
    val sampleAtNanos: Long,
    val headingDegrees: Double,
    val headingAtNanos: Long,
)

internal data class CircleFitResult(
    val observation: CircleArcObservation? = null,
    val reason: String? = null,
)

/** Predicted relative centre and velocity, rotated into the requested heading axes. */
internal data class CircleStateEstimate(
    val centerForwardMeters: Double,
    val centerRightMeters: Double,
    val forwardVelocityMetersPerSecond: Double,
    val rightVelocityMetersPerSecond: Double,
    val positionStdMeters: Double,
    val velocityStdMetersPerSecond: Double,
    val estimateAtNanos: Long,
    val lastVisionAtNanos: Long,
    val lastVelocityAtNanos: Long,
    val predictionSeconds: Double,
)

internal data class CircleEstimateResult(
    val estimate: CircleStateEstimate? = null,
    val reason: String? = null,
    val acceptedVisionCount: Int = 0,
    val rejectedVisionCount: Int = 0,
    val acceptedVelocityCount: Int = 0,
    val lateReplayCount: Int = 0,
    val lastInnovationSquared: Double? = null,
    val lastVisionAtNanos: Long = 0L,
    val lastVelocityAtNanos: Long = 0L,
)

/** Provisional estimator/geometry configuration, not a flight mode or calibration. */
internal data class CircleModelConfig(
    val radiusMeters: Double = 0.75,
    val maximumVisionAgeNanos: Long = 250_000_000L,
    val maximumVelocityAgeNanos: Long = 250_000_000L,
    val maximumHeadingAgeNanos: Long = 250_000_000L,
    val historyDurationNanos: Long = 600_000_000L,
    val historyCapacity: Int = 128,
    val commandPredictionSeconds: Double = 0.05,
    val maximumPredictionSeconds: Double = 0.10,
    val maximumPositionStdMeters: Double = 0.08,
    /** sqrt(.10² + .80² × (.25s maximum sample age + .05s prediction)), rounded up. */
    val maximumVelocityStdMetersPerSecond: Double = 0.45,
    /** Std of acceleration averaged over 1s; white-noise PSD = sigma² × 1s (m²/s³). */
    val accelerationNoiseMetersPerSecondSquared: Double = 0.8,
    val velocityMeasurementStdMetersPerSecond: Double = 0.10,
    val innovationGateSquared: Double = 9.21,
) {
    init {
        require(radiusMeters.isFinite() && radiusMeters > 0.0)
        require(maximumVisionAgeNanos > 0L && maximumVelocityAgeNanos > 0L && maximumHeadingAgeNanos > 0L)
        require(historyDurationNanos >= maxOf(maximumVisionAgeNanos, maximumVelocityAgeNanos))
        require(historyCapacity >= 8)
        require(commandPredictionSeconds.isFinite() && maximumPredictionSeconds.isFinite())
        require(commandPredictionSeconds in 0.0..maximumPredictionSeconds && maximumPredictionSeconds <= 0.25)
        require(maximumPositionStdMeters.isFinite() && maximumPositionStdMeters > 0.0)
        require(maximumVelocityStdMetersPerSecond.isFinite() && maximumVelocityStdMetersPerSecond > 0.0)
        require(accelerationNoiseMetersPerSecondSquared.isFinite() && accelerationNoiseMetersPerSecondSquared > 0.0)
        require(velocityMeasurementStdMetersPerSecond.isFinite() && velocityMeasurementStdMetersPerSecond > 0.0)
        require(innovationGateSquared.isFinite() && innovationGateSquared > 0.0)
    }
}

/** Model provenance is distinct from detector quality and parent control authority. */
internal data class CircleModelDiagnostics(
    val guidanceActive: Boolean,
    val status: String,
    val fitResidualRmsMeters: Double? = null,
    val fitInlierCount: Int = 0,
    val fitArcSpanRadians: Double? = null,
    val imageScaleAtNanos: Long = 0L,
    val metersPerPixel: Double? = null,
    val lastVisionAtNanos: Long = 0L,
    val lastVelocityAtNanos: Long = 0L,
    val estimateAtNanos: Long = 0L,
    val centerForwardMeters: Double? = null,
    val centerRightMeters: Double? = null,
    val radialErrorMeters: Double? = null,
    val tangentErrorDegrees: Double? = null,
    val alongTrackVelocityMetersPerSecond: Double? = null,
    val positionStdMeters: Double? = null,
    val velocityStdMetersPerSecond: Double? = null,
    val predictionSeconds: Double = 0.0,
    val acceptedVisionCount: Int = 0,
    val rejectedVisionCount: Int = 0,
    val acceptedVelocityCount: Int = 0,
    val lateReplayCount: Int = 0,
    val lastInnovationSquared: Double? = null,
    val fitReason: String? = null,
    val estimatorReason: String? = null,
    val sourceAtNanos: Long = 0L,
    val receivedAtNanos: Long = 0L,
    val lastRejectionReason: String? = null,
    val lastRejectionSourceAtNanos: Long = 0L,
    val rejectionCount: Int = 0,
    val estimatorResetCount: Int = 0,
)

internal data class RacingCircleGuidanceResult(
    val tangentDegrees: Double?,
    val diagnostics: CircleModelDiagnostics,
)
