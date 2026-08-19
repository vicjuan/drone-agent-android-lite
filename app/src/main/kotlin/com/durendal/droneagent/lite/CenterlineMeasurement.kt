package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.min

/**
 * The steering quantities read off one ordered centerline, in source-frame
 * proportions so a caller never has to know the working resolution.
 *
 * [lookahead] is null when the chain is too short to offer a target the
 * aircraft could aim at. That absence is the measurement's own verdict, not a
 * missing value to be filled in downstream: it is exactly what separates a path
 * that may drive Pure Pursuit from one that may only be aligned to.
 */
internal data class CenterlinePathMeasurement(
    val anchorXFraction: Double,
    val anchorYFraction: Double,
    val nearFieldAngleFromVerticalDegrees: Double,
    val lookahead: TapeLookahead?,
    val lookaheadAngleFromVerticalDegrees: Double?,
    val curvatureDegrees: Double,
    val arcLengthFraction: Double,
    val medianWidthFraction: Double,
)

/**
 * Turns an ordered centerline into steering quantities.
 *
 * Deliberately the same arc-length rule the existing estimator uses — a fixed
 * fraction of the traced arc, capped so a visible hairpin cannot place the
 * target on the returning leg — so that a shadow comparison between the two
 * measures the geometry, not two different look-ahead conventions.
 */
internal object CenterlineMeasurement {

    fun measure(
        estimate: CenterlineEstimate,
        frameWidth: Int,
        frameHeight: Int,
    ): CenterlinePathMeasurement? {
        require(frameWidth > 0 && frameHeight > 0) { "frame dimensions must be positive" }
        val points = estimate.points
        if (points.size < MIN_POINT_COUNT) return null

        val frameShortSide = min(frameWidth, frameHeight).toDouble()
        val cumulativeArc = DoubleArray(points.size)
        for (index in 1 until points.size) {
            cumulativeArc[index] = cumulativeArc[index - 1] +
                hypot(points[index].x - points[index - 1].x, points[index].y - points[index - 1].y)
        }
        val arcLength = cumulativeArc.last()
        if (arcLength <= 0.0) return null

        val nearFieldIndex = min(NEAR_FIELD_TANGENT_INDEX, points.size - 1)
        val nearFieldAngle = tangentAngle(points, nearFieldIndex) ?: return null

        val targetArc = min(
            arcLength * LOOKAHEAD_ARC_FRACTION,
            frameShortSide * MAX_LOOKAHEAD_FRAME_FRACTION,
        )
        var lookaheadIndex = 0
        while (lookaheadIndex < points.size - 1 && cumulativeArc[lookaheadIndex] < targetArc) {
            lookaheadIndex++
        }
        // A target that has not travelled a usable distance from the anchor is
        // not a look-ahead point, it is the anchor with noise on it.
        val lookaheadArc = cumulativeArc[lookaheadIndex]
        val lookaheadUsable =
            lookaheadArc >= frameShortSide * MIN_LOOKAHEAD_FRAME_FRACTION &&
                lookaheadIndex > nearFieldIndex
        val lookaheadAngle = if (lookaheadUsable) tangentAngle(points, lookaheadIndex) else null
        val lookahead = if (lookaheadUsable && lookaheadAngle != null) {
            TapeLookahead(
                xFraction = (points[lookaheadIndex].x / frameWidth).coerceIn(0.0, 1.0),
                yFraction = (points[lookaheadIndex].y / frameHeight).coerceIn(0.0, 1.0),
            )
        } else {
            null
        }

        val anchor = points.first()
        return CenterlinePathMeasurement(
            anchorXFraction = (anchor.x / frameWidth).coerceIn(0.0, 1.0),
            anchorYFraction = (anchor.y / frameHeight).coerceIn(0.0, 1.0),
            nearFieldAngleFromVerticalDegrees = nearFieldAngle,
            lookahead = lookahead,
            lookaheadAngleFromVerticalDegrees = if (lookahead == null) null else lookaheadAngle,
            curvatureDegrees = if (lookaheadAngle == null) {
                0.0
            } else {
                axialAngleDifferenceDegrees(nearFieldAngle, lookaheadAngle)
            },
            arcLengthFraction = arcLength / frameShortSide,
            medianWidthFraction = medianWidthFraction(points, frameShortSide),
        )
    }

    private fun tangentAngle(points: List<CenterlinePoint>, centerIndex: Int): Double? {
        val halfSpan = (points.size / TANGENT_SPAN_DIVISOR)
            .coerceIn(MIN_TANGENT_HALF_SPAN, MAX_TANGENT_HALF_SPAN)
        val behind = (centerIndex - halfSpan).coerceAtLeast(0)
        val ahead = (centerIndex + halfSpan).coerceAtMost(points.size - 1)
        val deltaX = points[ahead].x - points[behind].x
        val deltaY = points[ahead].y - points[behind].y
        if (deltaX == 0.0 && deltaY == 0.0) return null
        return TapeOrientation.deviationFromVerticalDegrees(deltaX, deltaY)
    }

    private fun medianWidthFraction(points: List<CenterlinePoint>, frameShortSide: Double): Double {
        val widths = points.map { it.widthPixels }.sorted()
        return widths[widths.size / 2] / frameShortSide
    }

    private fun axialAngleDifferenceDegrees(first: Double, second: Double): Double {
        var difference = abs(first - second)
        if (difference > 90.0) difference = 180.0 - difference
        return difference
    }

    /** Matches TapePathDirectionEstimator so a shadow comparison is like for like. */
    const val LOOKAHEAD_ARC_FRACTION = 0.40
    const val MAX_LOOKAHEAD_FRAME_FRACTION = 0.40
    private const val MIN_LOOKAHEAD_FRAME_FRACTION = 0.08
    private const val MIN_POINT_COUNT = 5
    private const val NEAR_FIELD_TANGENT_INDEX = 4
    private const val TANGENT_SPAN_DIVISOR = 20
    private const val MIN_TANGENT_HALF_SPAN = 2
    private const val MAX_TANGENT_HALF_SPAN = 12
}
