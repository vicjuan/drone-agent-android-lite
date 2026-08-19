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
 *
 * [LOST] is deliberately never a property of a [TapeDetection]: a lost path is
 * reported as no detection at all. Keeping it in this enum is what lets the
 * controller answer every frame — detected or not — with one quality value.
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
 * The point on the tape the aircraft should steer toward, in source-frame
 * proportions.
 *
 * The two coordinates travel together because half a look-ahead point is not a
 * weaker look-ahead, it is a bug: a path is either followed far enough ahead to
 * aim at, or it is not followed far enough at all.
 */
data class TapeLookahead(
    val xFraction: Double,
    val yFraction: Double,
) {
    init {
        require(xFraction in 0.0..1.0) { "lookahead x must be a frame fraction" }
        require(yFraction in 0.0..1.0) { "lookahead y must be a frame fraction" }
    }
}

/**
 * A black-tape candidate expressed in source-frame proportions.
 *
 * [quality] is derived from [lookahead] rather than stored, and [lookahead] has
 * no default, so a caller cannot claim [PathQuality.FULL_PATH] for a path it
 * could not follow ahead, nor arrive at one by omission. That
 * equivalence is the contract itself: a trustworthy look-ahead point is exactly
 * what separates a path the controller may pursue from one it may only align to.
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
    /**
     * Deliberately has no default. A default derived from [bounds] would hand
     * every forgetful caller a made-up look-ahead point and, with it,
     * [PathQuality.FULL_PATH] — the exact silent promotion this contract exists
     * to prevent. Whether the path was followed far enough to aim at is a fact
     * the measurement stage knows and must state.
     */
    val lookahead: TapeLookahead?,
    /**
     * Whether the far end of the chain stops inside the frame. Only then may the
     * mission layer treat shortening as an approaching end of tape; a chain cut
     * off by the border says the tape left the view, which is the opposite.
     */
    val endpointCandidate: Boolean = false,
    /** A closed loop has no end to reach and must never trigger a turnaround. */
    val closedLoop: Boolean = false,
    val centerline: TapeCenterlinePath? = null,
) {
    val quality: PathQuality
        get() = if (lookahead == null) PathQuality.NEAR_FIELD_ONLY else PathQuality.FULL_PATH

    init {
        require(sourceWidth > 0 && sourceHeight > 0)
        require(confidence in 0.0..1.0)
        require(angleFromVerticalDegrees in -90.0..90.0)
        require(longSideFraction > 0.0 && longSideFraction.isFinite())
        require(nearFieldOffsetFraction in -0.5..0.5)
        require(anchorXFraction in 0.0..1.0 && anchorYFraction in 0.0..1.0)
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
    NO_NEAR_FIELD_COMPONENT,
    INSUFFICIENT_LOOKAHEAD,
    AMBIGUOUS_BRANCH,
    TEMPORAL_DISCONTINUITY,
    BRIDGE_REJECTED,
}

internal object TapeLuminancePolicy {
    fun effectiveThreshold(otsuThreshold: Double): Double =
        otsuThreshold.coerceAtMost(MAX_TAPE_LUMINANCE)

    private const val MAX_TAPE_LUMINANCE = 105.0
}

