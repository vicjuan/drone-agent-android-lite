package com.durendal.droneagent.lite

import kotlin.math.atan2

/**
 * PATH follows a sampled tape centerline and is the passive preview default, so
 * curved tape is outlined before the operator enables autonomous tracking.
 */
internal enum class TapeDetectionMode {
    STRAIGHT,
    PATH,
}

/** A black-tape candidate expressed in source-frame proportions. */
data class TapeDetection(
    val sourceWidth: Int,
    val sourceHeight: Int,
    val bounds: NormalizedRect,
    val confidence: Double,
    val angleFromVerticalDegrees: Double,
    val longSideFraction: Double,
    val nearFieldOffsetFraction: Double,
    val anchorXFraction: Double = bounds.centerX,
    val anchorYFraction: Double = bounds.bottom,
    val lookaheadXFraction: Double = bounds.centerX,
    val lookaheadYFraction: Double = bounds.top,
) {
    init {
        require(sourceWidth > 0 && sourceHeight > 0)
        require(confidence in 0.0..1.0)
        require(angleFromVerticalDegrees in -90.0..90.0)
        require(longSideFraction > 0.0 && longSideFraction.isFinite())
        require(nearFieldOffsetFraction in -0.5..0.5)
        require(anchorXFraction in 0.0..1.0 && anchorYFraction in 0.0..1.0)
        require(lookaheadXFraction in 0.0..1.0 && lookaheadYFraction in 0.0..1.0)
    }
}

data class NormalizedRect(
    val left: Double,
    val top: Double,
    val right: Double,
    val bottom: Double,
) {
    val centerX: Double
        get() = (left + right) / 2.0
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
    val surroundingFloor: Double,
    val minimumSideFloor: Double = 1.0,
    val touchesHorizontalFrameEdge: Boolean = false,
    val overlapsPreviousDetection: Boolean = false,
)

internal enum class TapeCandidateRejection {
    INVALID_GEOMETRY,
    AREA,
    ASPECT,
    LENGTH,
    CURVATURE,
    DIRECTION_CONTINUITY,
    WIDTH,
    HORIZONTAL_FRAME_EDGE,
    ORIENTED_FILL,
    CHROMA,
    FLOOR_CONTEXT,
}

internal object TapeLuminancePolicy {
    fun effectiveThreshold(otsuThreshold: Double): Double =
        otsuThreshold.coerceAtMost(MAX_TAPE_LUMINANCE)

    private const val MAX_TAPE_LUMINANCE = 105.0
}

internal object TapeCandidatePolicy {
    fun rejectionReason(metrics: TapeCandidateMetrics): TapeCandidateRejection? {
        if (metrics.areaFraction !in MIN_AREA_FRACTION..MAX_AREA_FRACTION) {
            return TapeCandidateRejection.AREA
        }
        val minimumAspectRatio =
            if (metrics.overlapsPreviousDetection) MIN_TRACKED_ASPECT_RATIO else MIN_ASPECT_RATIO
        if (metrics.aspectRatio < minimumAspectRatio) {
            return TapeCandidateRejection.ASPECT
        }
        val minimumLongSideFraction =
            if (metrics.overlapsPreviousDetection) {
                MIN_TRACKED_LONG_SIDE_FRACTION
            } else {
                MIN_LONG_SIDE_FRACTION
            }
        if (metrics.longSideFraction < minimumLongSideFraction) {
            return TapeCandidateRejection.LENGTH
        }
        if (metrics.shortSideFraction !in MIN_SHORT_SIDE_FRACTION..MAX_SHORT_SIDE_FRACTION) {
            return TapeCandidateRejection.WIDTH
        }
        if (metrics.touchesHorizontalFrameEdge && !metrics.overlapsPreviousDetection) {
            return TapeCandidateRejection.HORIZONTAL_FRAME_EDGE
        }
        if (metrics.orientedFill < MIN_ORIENTED_FILL) {
            return TapeCandidateRejection.ORIENTED_FILL
        }
        if (
            metrics.surroundingFloor < MIN_SURROUNDING_FLOOR ||
            metrics.minimumSideFloor < MIN_SIDE_FLOOR
        ) {
            return TapeCandidateRejection.FLOOR_CONTEXT
        }
        return null
    }

    fun score(metrics: TapeCandidateMetrics): Double? {
        if (rejectionReason(metrics) != null) return null
        val aspectConfidence = (metrics.aspectRatio / IDEAL_ASPECT_RATIO).coerceIn(0.0, 1.0)
        val floorConfidence = (metrics.surroundingFloor + metrics.minimumSideFloor) / 2.0
        return (
            metrics.orientedFill * 0.40 +
                floorConfidence * 0.35 +
                aspectConfidence * 0.25
            ).coerceIn(0.0, 1.0)
    }

    private const val MIN_AREA_FRACTION = 0.0008
    private const val MAX_AREA_FRACTION = 0.18
    private const val MIN_ASPECT_RATIO = 2.2
    private const val MIN_TRACKED_ASPECT_RATIO = 1.5
    private const val MIN_SHORT_SIDE_FRACTION = 0.035
    private const val MAX_SHORT_SIDE_FRACTION = 0.20
    private const val MIN_LONG_SIDE_FRACTION = 0.25
    private const val MIN_TRACKED_LONG_SIDE_FRACTION = 0.15
    private const val MIN_ORIENTED_FILL = 0.30
    private const val MIN_SURROUNDING_FLOOR = 0.22
    private const val MIN_SIDE_FLOOR = 0.30
    private const val IDEAL_ASPECT_RATIO = 8.0
}
