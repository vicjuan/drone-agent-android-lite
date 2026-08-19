package com.durendal.droneagent.lite

import kotlin.math.atan2

/**
 * How much of the tape path this frame's image evidence actually supports.
 *
 * [FULL_PATH] is the only verdict that may drive Pure Pursuit: a credible
 * near-field centerline that also carries a usable arc-length lookahead.
 * [NEAR_FIELD_ONLY] means the aircraft still knows which way the tape under it
 * runs but has no trustworthy path ahead, so translation must stop and only a
 * bounded in-place alignment is allowed. [LOST] means even the near field is
 * not credible and every motion command must be zero.
 */
enum class PathQuality {
    FULL_PATH,
    NEAR_FIELD_ONLY,
    LOST,
}

/**
 * PATH follows a sampled tape centerline and is the passive preview default, so
 * curved tape is outlined before the operator enables autonomous tracking.
 */
internal enum class TapeDetectionMode {
    STRAIGHT,
    PATH,
}

/**
 * A black-tape candidate expressed in source-frame proportions.
 *
 * [lookaheadXFraction] and [lookaheadYFraction] are null unless [quality] is
 * [PathQuality.FULL_PATH]: a near-field-only path must never hand a lookahead
 * point to the controller. [directEvidenceFraction] is the share of the
 * accepted path supported by raw segmentation pixels rather than by glare
 * bridging, so a mostly-repaired path can never look as credible as an
 * observed one.
 */
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
    val lookaheadXFraction: Double? = bounds.centerX,
    val lookaheadYFraction: Double? = bounds.top,
    val quality: PathQuality = PathQuality.FULL_PATH,
    val curvatureNormalized: Double? = null,
    val directEvidenceFraction: Double = 1.0,
) {
    init {
        require(sourceWidth > 0 && sourceHeight > 0)
        require(confidence in 0.0..1.0)
        require(angleFromVerticalDegrees in -90.0..90.0)
        require(longSideFraction > 0.0 && longSideFraction.isFinite())
        require(nearFieldOffsetFraction in -0.5..0.5)
        require(anchorXFraction in 0.0..1.0 && anchorYFraction in 0.0..1.0)
        require(lookaheadXFraction == null || lookaheadXFraction in 0.0..1.0)
        require(lookaheadYFraction == null || lookaheadYFraction in 0.0..1.0)
        require(directEvidenceFraction in 0.0..1.0)
        require(quality != PathQuality.LOST) { "a LOST path is reported as no detection" }
        require(
            quality == PathQuality.FULL_PATH ||
                (lookaheadXFraction == null && lookaheadYFraction == null),
        ) { "only FULL_PATH may carry a lookahead point" }
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
    NO_CENTERLINE,
    INSUFFICIENT_LOOKAHEAD,
    TEMPORAL_DISCONTINUITY,
    BRIDGE_REJECTED,
}

internal object TapeLuminancePolicy {
    fun effectiveThreshold(otsuThreshold: Double): Double =
        otsuThreshold.coerceAtMost(MAX_TAPE_LUMINANCE)

    private const val MAX_TAPE_LUMINANCE = 105.0
}

