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

internal enum class TapeCandidateRejection {
    INVALID_GEOMETRY,
    AREA,
    LENGTH,
    CURVATURE,
    DIRECTION_CONTINUITY,
    HORIZONTAL_FRAME_EDGE,
    CHROMA,
    FLOOR_CONTEXT,
}

internal object TapeLuminancePolicy {
    fun effectiveThreshold(otsuThreshold: Double): Double =
        otsuThreshold.coerceAtMost(MAX_TAPE_LUMINANCE)

    private const val MAX_TAPE_LUMINANCE = 105.0
}

