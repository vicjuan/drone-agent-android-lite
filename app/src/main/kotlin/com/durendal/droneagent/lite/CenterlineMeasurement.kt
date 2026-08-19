package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.min

/**
 * The steering quantities read off one ordered centerline, in source-frame
 * proportions so a caller never has to know the working resolution.
 *
 * [lookahead] is null when the chain is too short to offer a target the aircraft
 * could aim at. That absence is the measurement's own verdict, not a missing
 * value to be filled in downstream: it is exactly what separates a path that may
 * drive Pure Pursuit from one that may only be aligned to.
 *
 * The two turn quantities answer different questions and must not be conflated:
 * [lookaheadHeadingChangeDegrees] is how much the tape bends over the stretch
 * the aircraft is about to fly, which is a steering input; [totalPathTurnDegrees]
 * is how much the whole visible chain turns, which is what says "this is an arc
 * and not a wall edge". A long gentle arc and a short sharp kink can share one
 * of these numbers while differing completely in the other.
 */
internal data class CenterlinePathMeasurement(
    val anchorXFraction: Double,
    val anchorYFraction: Double,
    val nearFieldAngleFromVerticalDegrees: Double,
    val lookahead: TapeLookahead?,
    val lookaheadAngleFromVerticalDegrees: Double?,
    val lookaheadHeadingChangeDegrees: Double,
    val totalPathTurnDegrees: Double,
    val turnConsistency: Double,
    val arcLengthFraction: Double,
    val medianWidthFraction: Double,
)

/**
 * Turns an ordered centerline into steering quantities.
 *
 * The arc-length look-ahead rule is a fixed fraction of the traced arc, capped so
 * a visible hairpin cannot place the target on the returning leg, and floored so
 * a target that has not travelled a usable distance is reported as no target at
 * all rather than as the anchor with noise on it.
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
        val lookaheadUsable =
            cumulativeArc[lookaheadIndex] >= frameShortSide * MIN_LOOKAHEAD_FRAME_FRACTION &&
                lookaheadIndex > nearFieldIndex
        val lookaheadAngle = if (lookaheadUsable) tangentAngle(points, lookaheadIndex) else null
        val lookahead = if (lookaheadAngle == null) {
            null
        } else {
            TapeLookahead(
                xFraction = (points[lookaheadIndex].x / frameWidth).coerceIn(0.0, 1.0),
                yFraction = (points[lookaheadIndex].y / frameHeight).coerceIn(0.0, 1.0),
            )
        }

        val turn = pathTurn(points)
        val anchor = points.first()
        return CenterlinePathMeasurement(
            anchorXFraction = (anchor.x / frameWidth).coerceIn(0.0, 1.0),
            anchorYFraction = (anchor.y / frameHeight).coerceIn(0.0, 1.0),
            nearFieldAngleFromVerticalDegrees = nearFieldAngle,
            lookahead = lookahead,
            lookaheadAngleFromVerticalDegrees = lookaheadAngle,
            lookaheadHeadingChangeDegrees = if (lookaheadAngle == null) {
                0.0
            } else {
                axialAngleDifferenceDegrees(nearFieldAngle, lookaheadAngle)
            },
            totalPathTurnDegrees = turn.totalDegrees,
            turnConsistency = turn.consistency,
            arcLengthFraction = arcLength / frameShortSide,
            medianWidthFraction = medianWidthFraction(points, frameShortSide),
        )
    }

    /**
     * Net heading change along the chain, and whether the chain turns one way.
     *
     * Consistency is measured over a few coarse segments rather than sample to
     * sample. Real tape has pixel-level jitter in its skeleton, and a per-sample
     * ratio charges every wobble against the path: the recorded cardboard scene
     * scores 0.24 that way despite turning 71 degrees in one direction. Netting
     * each segment first lets jitter cancel where it belongs while leaving a
     * genuine reversal — an S bend — plainly visible as opposing segment signs.
     */
    private fun pathTurn(points: List<CenterlinePoint>): PathTurn {
        val stride = (points.size / TURN_SAMPLE_COUNT).coerceAtLeast(1)
        val headings = ArrayList<Double>(TURN_SAMPLE_COUNT + 1)
        var index = 0
        while (index + stride < points.size) {
            val deltaX = points[index + stride].x - points[index].x
            val deltaY = points[index + stride].y - points[index].y
            if (deltaX != 0.0 || deltaY != 0.0) {
                headings += Math.toDegrees(atan2(deltaX, -deltaY))
            }
            index += stride
        }
        if (headings.size < 2) return PathTurn(0.0, 0.0)

        val deltas = DoubleArray(headings.size - 1) { position ->
            var delta = headings[position + 1] - headings[position]
            while (delta > 180.0) delta -= 360.0
            while (delta < -180.0) delta += 360.0
            delta
        }
        val signedTotal = deltas.sum()

        val segmentSize = (deltas.size / TURN_SEGMENT_COUNT).coerceAtLeast(1)
        var segmentAbsoluteTotal = 0.0
        var start = 0
        while (start < deltas.size) {
            val end = min(start + segmentSize, deltas.size)
            var segmentNet = 0.0
            for (position in start until end) segmentNet += deltas[position]
            segmentAbsoluteTotal += abs(segmentNet)
            start = end
        }
        return PathTurn(
            totalDegrees = abs(signedTotal),
            consistency = if (segmentAbsoluteTotal <= 0.0) {
                0.0
            } else {
                abs(signedTotal) / segmentAbsoluteTotal
            },
        )
    }

    private class PathTurn(val totalDegrees: Double, val consistency: Double)

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

    const val LOOKAHEAD_ARC_FRACTION = 0.40
    const val MAX_LOOKAHEAD_FRAME_FRACTION = 0.40
    const val MIN_LOOKAHEAD_FRAME_FRACTION = 0.08
    const val MIN_POINT_COUNT = 5
    private const val NEAR_FIELD_TANGENT_INDEX = 4
    private const val TANGENT_SPAN_DIVISOR = 20
    private const val MIN_TANGENT_HALF_SPAN = 2
    private const val MAX_TANGENT_HALF_SPAN = 12

    /** Enough samples to see a bend develop, few enough to ignore pixel staircases. */
    private const val TURN_SAMPLE_COUNT = 12

    /** Coarse enough that skeleton jitter cancels inside a segment. */
    private const val TURN_SEGMENT_COUNT = 4
}
