package com.durendal.droneagent.lite

import kotlin.math.atan2
/** A black-tape candidate expressed in source-frame proportions. */
data class TapeDetection(
    val sourceWidth: Int,
    val sourceHeight: Int,
    val bounds: NormalizedRect,
    val confidence: Double,
    val angleFromVerticalDegrees: Double,
) {
    init {
        require(sourceWidth > 0 && sourceHeight > 0)
        require(confidence in 0.0..1.0)
        require(angleFromVerticalDegrees in -90.0..90.0)
    }
}

data class NormalizedRect(
    val left: Double,
    val top: Double,
    val right: Double,
    val bottom: Double,
) {
    init {
        require(left in 0.0..1.0 && right in 0.0..1.0 && left < right)
        require(top in 0.0..1.0 && bottom in 0.0..1.0 && top < bottom)
    }
}


internal object TapeOrientation {
    /**
     * Returns the signed angle from image-up to an undirected line. Positive
     * means the line leans toward image-right as it extends forward.
     */
    fun deviationFromVerticalDegrees(deltaX: Double, deltaY: Double): Double {
        require(deltaX.isFinite() && deltaY.isFinite())
        require(deltaX != 0.0 || deltaY != 0.0)
        val pointsUp = deltaY < 0.0 || (deltaY == 0.0 && deltaX >= 0.0)
        val upwardX = if (pointsUp) deltaX else -deltaX
        val upwardY = if (pointsUp) deltaY else -deltaY
        return Math.toDegrees(atan2(upwardX, -upwardY)).coerceIn(-90.0, 90.0)
    }
}
internal data class TapeCandidateMetrics(
    val areaFraction: Double,
    val aspectRatio: Double,
    val shortSideFraction: Double,
    val longSideFraction: Double,
    val orientedFill: Double,
    val surroundingBrown: Double,
    val touchesHorizontalFrameEdge: Boolean = false,
    val overlapsPreviousDetection: Boolean = false,
)

internal enum class TapeCandidateRejection {
    SHAPE,
    HORIZONTAL_FRAME_EDGE,
    ORIENTED_FILL,
    BROWN_CONTEXT,
}

internal object TapeLuminancePolicy {
    fun effectiveThreshold(otsuThreshold: Double): Double =
        otsuThreshold.coerceAtMost(MAX_TAPE_LUMINANCE)

    private const val MAX_TAPE_LUMINANCE = 105.0
}

internal object TapeCandidatePolicy {
    fun hasPlausibleShape(
        areaFraction: Double,
        aspectRatio: Double,
        shortSideFraction: Double,
        longSideFraction: Double,
    ): Boolean =
        areaFraction in MIN_AREA_FRACTION..MAX_AREA_FRACTION &&
            aspectRatio >= MIN_ASPECT_RATIO &&
            shortSideFraction <= MAX_SHORT_SIDE_FRACTION &&
            longSideFraction >= MIN_LONG_SIDE_FRACTION

    fun rejectionReason(metrics: TapeCandidateMetrics): TapeCandidateRejection? {
        if (
            !hasPlausibleShape(
                metrics.areaFraction,
                metrics.aspectRatio,
                metrics.shortSideFraction,
                metrics.longSideFraction,
            )
        ) {
            return TapeCandidateRejection.SHAPE
        }
        if (metrics.touchesHorizontalFrameEdge && !metrics.overlapsPreviousDetection) {
            return TapeCandidateRejection.HORIZONTAL_FRAME_EDGE
        }
        if (metrics.orientedFill < MIN_ORIENTED_FILL) {
            return TapeCandidateRejection.ORIENTED_FILL
        }
        if (metrics.surroundingBrown < MIN_SURROUNDING_BROWN) {
            return TapeCandidateRejection.BROWN_CONTEXT
        }
        return null
    }

    fun score(metrics: TapeCandidateMetrics): Double? {
        if (rejectionReason(metrics) != null) return null
        val aspectConfidence = (metrics.aspectRatio / IDEAL_ASPECT_RATIO).coerceIn(0.0, 1.0)
        return (
            metrics.orientedFill * 0.40 +
                metrics.surroundingBrown * 0.35 +
                aspectConfidence * 0.25
            ).coerceIn(0.0, 1.0)
    }

    private const val MIN_AREA_FRACTION = 0.0008
    private const val MAX_AREA_FRACTION = 0.18
    private const val MIN_ASPECT_RATIO = 2.2
    private const val MAX_SHORT_SIDE_FRACTION = 0.14
    private const val MIN_LONG_SIDE_FRACTION = 0.25
    private const val MIN_ORIENTED_FILL = 0.30
    private const val MIN_SURROUNDING_BROWN = 0.22
    private const val IDEAL_ASPECT_RATIO = 8.0
}
