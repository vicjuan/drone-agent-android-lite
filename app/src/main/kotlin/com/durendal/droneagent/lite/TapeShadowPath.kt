package com.durendal.droneagent.lite

/**
 * The replacement centerline for one frame, in source-frame proportions, ready
 * to draw.
 *
 * Coordinates are fractions rather than pixels because the analysis frame is a
 * scaled copy of the preview: the same numbers place the path correctly on any
 * view size, and the overlay never has to know the working resolution.
 *
 * This is the shadow result made visible. It carries no authority over the
 * aircraft — an operator seeing this line is seeing what the new geometry found,
 * not what the controller is following.
 */
internal class TapeShadowPath(
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
            rejection?.let { append("  ").append(it) }
        }
}

/**
 * One frame's shadow output: the line for the flight log and the path for the
 * screen, built together so the two can never disagree about the same frame.
 */
internal class TapeShadowResult(
    val logLine: String,
    val path: TapeShadowPath?,
)
