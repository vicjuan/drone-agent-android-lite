package com.durendal.droneagent.lite

/**
 * The replacement centerline for one frame, in source-frame proportions, ready
 * to draw.
 *
 * Coordinates are fractions rather than pixels because the analysis frame is a
 * scaled copy of the preview: the same numbers place the path correctly on any
 * view size, and the overlay never has to know the working resolution.
 *
 * This is the path the controller actually follows, so what an operator sees on
 * the preview and what drives forward, right and yaw are the same geometry. An
 * overlay that could disagree with the controller would be worse than no overlay.
 */
class TapeCenterlinePath internal constructor(
    val sourceWidth: Int,
    val sourceHeight: Int,
    val xFractions: FloatArray,
    val yFractions: FloatArray,
    val anchorXFraction: Float,
    val anchorYFraction: Float,
    val lookaheadXFraction: Float?,
    val lookaheadYFraction: Float?,
    val quality: PathQuality,
    val rejection: String?,
    val branchCount: Int = 0,
    val closedLoop: Boolean = false,
    val endpointCandidate: Boolean = false,
) {
    init {
        require(sourceWidth > 0 && sourceHeight > 0) { "source dimensions must be positive" }
        require(xFractions.size == yFractions.size) { "centerline coordinates must pair up" }
        require((lookaheadXFraction == null) == (lookaheadYFraction == null)) {
            "a look-ahead point is either fully known or absent"
        }
    }

    val pointCount: Int get() = xFractions.size

    /** What the operator reads on the preview: the verdict, and why if not full. */
    val label: String
        get() = buildString {
            append("CENTERLINE ")
            append(
                when (quality) {
                    PathQuality.FULL_PATH -> "FULL"
                    PathQuality.NEAR_FIELD_ONLY -> "NEAR ONLY"
                    PathQuality.LOST -> "LOST"
                },
            )
            append("  ").append(pointCount).append("pt")
            if (branchCount > 0) append("  BRANCH:").append(branchCount)
            if (closedLoop) append("  LOOP")
            if (endpointCandidate) append("  END")
            rejection?.let { append("  ").append(it) }
        }
}

