package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.roundToInt

/** One locally connected black-tape path, ordered from the camera-near end upward. */
internal data class TapePathEstimate(
    val nearFieldCenterX: Double,
    val nearFieldCenterY: Double,
    val nearFieldAngleFromVerticalDegrees: Double,
    val lookaheadCenterX: Double,
    val lookaheadCenterY: Double,
    val lookaheadAngleFromVerticalDegrees: Double,
    val arcLengthFraction: Double,
    val medianWidthFraction: Double,
    val widthConsistency: Double,
    val curvatureDegrees: Double,
    val curvatureSmoothness: Double,
    val sampleCount: Int,
    val bounds: TapePathBounds,
    val horizontalFallback: Boolean = false,
)

internal data class TapePathBounds(
    val left: Int,
    val top: Int,
    val right: Int,
    val bottom: Int,
)

/**
 * Extracts the locally-followable part of a curved ribbon from one isolated contour mask.
 *
 * The aircraft keeps the camera-down path roughly vertical while following it, so tracing the
 * connected run nearest the previous row gives a cheap local centerline without mistaking the
 * contour's bounding-box chord for its direction. The near-field tangent controls current
 * alignment. The Pure Pursuit target stays ahead on the same trace, but its arc distance is
 * capped so a visible hairpin cannot move the target onto the returning leg.
 *
 * [estimateVerticalPath] walks rows upward and fails closed once the path turns horizontal
 * enough to stop producing a bounded local run. [estimateHorizontalFallback] then walks
 * columns instead, so an arc lying across the frame is still followable rather than lost.
 */
internal class TapePathDirectionEstimator {
    fun estimateVerticalPath(
        mask: ByteArray,
        frameWidth: Int,
        frameHeight: Int,
        left: Int,
        top: Int,
        right: Int,
        bottom: Int,
        initialCenterHint: Double? = null,
        preferRightmostInitialRun: Boolean = false,
    ): TapePathEstimate? {
        require(mask.size >= frameWidth * frameHeight)
        if (left !in 0 until right || top !in 0 until bottom) return null
        if (right > frameWidth || bottom > frameHeight) return null

        val frameShortSide = minOf(frameWidth, frameHeight).toDouble()
        val maximumLocalWidth = frameShortSide * MAX_LOCAL_WIDTH_FRACTION
        val pointsX = DoubleArray(bottom - top)
        val pointsY = DoubleArray(bottom - top)
        val widths = DoubleArray(bottom - top)
        var pointCount = 0
        var previousCenter = Double.NaN
        var previousWidth = Double.NaN
        var missingRows = 0

        for (y in bottom - 1 downTo top) {
            val run =
                nearestRun(
                    mask,
                    frameWidth,
                    y,
                    left,
                    right,
                    previousCenter,
                    previousWidth,
                    maximumLocalWidth,
                    initialCenterHint,
                    preferRightmostInitialRun,
                )
            if (run == null) {
                if (pointCount > 0 && ++missingRows > MAX_CONSECUTIVE_MISSING_ROWS) break
                continue
            }
            missingRows = 0
            previousCenter = run.center
            previousWidth = run.width
            pointsX[pointCount] = run.center
            pointsY[pointCount] = y.toDouble()
            widths[pointCount] = run.width
            pointCount++
        }
        if (pointCount < MIN_SAMPLE_COUNT) return null

        val cumulativeArc = DoubleArray(pointCount)
        for (index in 1 until pointCount) {
            cumulativeArc[index] = cumulativeArc[index - 1] + hypot(
                pointsX[index] - pointsX[index - 1],
                pointsY[index] - pointsY[index - 1],
            )
        }
        val arcLength = cumulativeArc[pointCount - 1]
        val arcLengthFraction = arcLength / frameShortSide
        if (arcLengthFraction < MIN_ARC_LENGTH_FRACTION) return null

        val sortedWidths = widths.copyOf(pointCount).apply { sort() }
        val medianWidth = sortedWidths[pointCount / 2]
        val medianWidthFraction = medianWidth / frameShortSide
        if (medianWidthFraction !in MIN_LOCAL_WIDTH_FRACTION..MAX_LOCAL_WIDTH_FRACTION) return null
        val deviations = DoubleArray(pointCount) { abs(widths[it] - medianWidth) }.apply { sort() }
        val medianDeviation = deviations[pointCount / 2]
        val medianConsistency = (1.0 - medianDeviation / medianWidth).coerceIn(0.0, 1.0)
        val lowerWidth = sortedWidths[(pointCount * WIDTH_LOWER_QUANTILE).toInt()]
        val upperWidth =
            sortedWidths[(pointCount * WIDTH_UPPER_QUANTILE).toInt().coerceAtMost(pointCount - 1)]
        val rangeConsistency = (lowerWidth / upperWidth).coerceIn(0.0, 1.0)
        val widthConsistency = minOf(medianConsistency, rangeConsistency)
        if (widthConsistency < MIN_WIDTH_CONSISTENCY) return null

        val targetArc = lookaheadArcLength(arcLength, frameShortSide)
        var lookaheadIndex = 0
        while (lookaheadIndex < pointCount - 1 && cumulativeArc[lookaheadIndex] < targetArc) {
            lookaheadIndex++
        }
        val nearFieldTangentIndex = minOf(NEAR_FIELD_SAMPLE_COUNT / 2, pointCount - 1)
        val nearFieldAngle =
            tangentAngle(pointsX, pointsY, pointCount, nearFieldTangentIndex) ?: return null
        val lookaheadAngle =
            tangentAngle(pointsX, pointsY, pointCount, lookaheadIndex) ?: return null

        val anchorSamples = minOf(NEAR_FIELD_SAMPLE_COUNT, pointCount)
        var nearFieldCenterX = 0.0
        var nearFieldCenterY = 0.0
        for (index in 0 until anchorSamples) {
            nearFieldCenterX += pointsX[index]
            nearFieldCenterY += pointsY[index]
        }
        nearFieldCenterX /= anchorSamples
        nearFieldCenterY /= anchorSamples
        var pathLeft = frameWidth
        var pathRight = 0
        for (index in 0 until pointCount) {
            val runStart = (pointsX[index] - (widths[index] - 1.0) / 2.0).roundToInt()
            pathLeft = minOf(pathLeft, runStart)
            pathRight = maxOf(pathRight, runStart + widths[index].toInt())
        }
        val pathBounds = TapePathBounds(
            left = pathLeft.coerceIn(left, right - 1),
            top = pointsY[pointCount - 1].toInt(),
            right = pathRight.coerceIn(left + 1, right),
            bottom = pointsY[0].toInt() + 1,
        )

        return TapePathEstimate(
            nearFieldCenterX = nearFieldCenterX,
            nearFieldCenterY = nearFieldCenterY,
            lookaheadCenterX = pointsX[lookaheadIndex],
            lookaheadCenterY = pointsY[lookaheadIndex],
            nearFieldAngleFromVerticalDegrees = nearFieldAngle,
            lookaheadAngleFromVerticalDegrees = lookaheadAngle,
            arcLengthFraction = arcLengthFraction,
            medianWidthFraction = medianWidthFraction,
            widthConsistency = widthConsistency,
            curvatureDegrees = curvatureDegrees(pointsX, pointsY, pointCount),
            curvatureSmoothness = curvatureSmoothness(pointsX, pointsY, pointCount),
            sampleCount = pointCount,
            bounds = pathBounds,
        )
    }
    /**
     * Traces a ribbon that is predominantly horizontal in the camera frame.
     *
     * A prior vertical trace narrows the accepted width during tracking. Without
     * one, the normal tape-width bounds still allow initial acquisition while
     * excluding thin floor seams.
     */
    fun estimateHorizontalFallback(
        mask: ByteArray,
        frameWidth: Int,
        frameHeight: Int,
        left: Int,
        top: Int,
        right: Int,
        bottom: Int,
        expectedMedianWidthFraction: Double? = null,
        preferredNearFieldX: Double? = null,
        preferredNearFieldY: Double? = null,
        preferRightToLeft: Boolean = false,
    ): TapePathEstimate? {
        require(mask.size >= frameWidth * frameHeight)
        if (left !in 0 until right || top !in 0 until bottom) return null
        if (right > frameWidth || bottom > frameHeight) return null

        val frameShortSide = minOf(frameWidth, frameHeight).toDouble()
        val expectedWidth = expectedMedianWidthFraction?.times(frameShortSide)
        if (
            expectedWidth != null &&
            expectedWidth !in
                frameShortSide * MIN_LOCAL_WIDTH_FRACTION..
                frameShortSide * MAX_LOCAL_WIDTH_FRACTION
        ) {
            return null
        }
        val minimumTrackedWidth =
            expectedWidth?.times(MIN_TRACKED_WIDTH_RATIO)
                ?: frameShortSide * MIN_LOCAL_WIDTH_FRACTION
        val maximumTrackedWidth =
            expectedWidth?.times(MAX_TRACKED_WIDTH_RATIO)
                ?: frameShortSide * MAX_LOCAL_WIDTH_FRACTION
        val leftToRight = traceColumns(
            mask = mask,
            frameWidth = frameWidth,
            left = left,
            top = top,
            right = right,
            bottom = bottom,
            step = 1,
            minimumTrackedWidth = minimumTrackedWidth,
            maximumTrackedWidth = maximumTrackedWidth,
            expectedWidth = expectedWidth,
        )
        val rightToLeft = traceColumns(
            mask = mask,
            frameWidth = frameWidth,
            left = left,
            top = top,
            right = right,
            bottom = bottom,
            step = -1,
            minimumTrackedWidth = minimumTrackedWidth,
            maximumTrackedWidth = maximumTrackedWidth,
            expectedWidth = expectedWidth,
        )
        val trace =
            selectHorizontalTrace(
                leftToRight,
                rightToLeft,
                preferredNearFieldX,
                preferredNearFieldY,
                preferRightToLeft,
                frameShortSide,
            ) ?: return null
        val arcLengthFraction = trace.arcLength / frameShortSide
        if (arcLengthFraction < MIN_ARC_LENGTH_FRACTION) return null

        val sortedWidths = trace.widths.copyOf(trace.pointCount).apply { sort() }
        val medianWidth = sortedWidths[trace.pointCount / 2]
        if (medianWidth !in minimumTrackedWidth..maximumTrackedWidth) return null
        val deviations =
            DoubleArray(trace.pointCount) { abs(trace.widths[it] - medianWidth) }.apply { sort() }
        val medianConsistency =
            (1.0 - deviations[trace.pointCount / 2] / medianWidth).coerceIn(0.0, 1.0)
        val lowerWidth =
            sortedWidths[(trace.pointCount * WIDTH_LOWER_QUANTILE).toInt()]
        val upperWidth =
            sortedWidths[
                (trace.pointCount * WIDTH_UPPER_QUANTILE).toInt()
                    .coerceAtMost(trace.pointCount - 1)
            ]
        val widthConsistency = minOf(
            medianConsistency,
            (lowerWidth / upperWidth).coerceIn(0.0, 1.0),
        )
        if (widthConsistency < MIN_WIDTH_CONSISTENCY) return null

        val targetArc = lookaheadArcLength(trace.arcLength, frameShortSide)
        var lookaheadIndex = 0
        while (
            lookaheadIndex < trace.pointCount - 1 &&
            trace.cumulativeArc[lookaheadIndex] < targetArc
        ) {
            lookaheadIndex++
        }
        val nearFieldTangentIndex =
            minOf(NEAR_FIELD_SAMPLE_COUNT / 2, trace.pointCount - 1)
        val nearFieldAngle =
            tangentAngle(
                trace.pointsX,
                trace.pointsY,
                trace.pointCount,
                nearFieldTangentIndex,
            ) ?: return null
        val lookaheadAngle =
            tangentAngle(
                trace.pointsX,
                trace.pointsY,
                trace.pointCount,
                lookaheadIndex,
            ) ?: return null

        var pathTop = frameHeight
        var pathBottom = 0
        for (index in 0 until trace.pointCount) {
            val runStart =
                (trace.pointsY[index] - (trace.widths[index] - 1.0) / 2.0).roundToInt()
            pathTop = minOf(pathTop, runStart)
            pathBottom = maxOf(pathBottom, runStart + trace.widths[index].toInt())
        }
        val firstX = trace.pointsX[0].toInt()
        val lastX = trace.pointsX[trace.pointCount - 1].toInt()
        return TapePathEstimate(
            nearFieldCenterX = trace.nearFieldCenterX,
            nearFieldCenterY = trace.nearFieldCenterY,
            lookaheadCenterX = trace.pointsX[lookaheadIndex],
            lookaheadCenterY = trace.pointsY[lookaheadIndex],
            nearFieldAngleFromVerticalDegrees = nearFieldAngle,
            lookaheadAngleFromVerticalDegrees = lookaheadAngle,
            arcLengthFraction = arcLengthFraction,
            medianWidthFraction = medianWidth / frameShortSide,
            widthConsistency = widthConsistency,
            curvatureDegrees =
                curvatureDegrees(trace.pointsX, trace.pointsY, trace.pointCount),
            curvatureSmoothness =
                curvatureSmoothness(trace.pointsX, trace.pointsY, trace.pointCount),
            sampleCount = trace.pointCount,
            bounds = TapePathBounds(
                left = minOf(firstX, lastX).coerceIn(left, right - 1),
                top = pathTop.coerceIn(top, bottom - 1),
                right = (maxOf(firstX, lastX) + 1).coerceIn(left + 1, right),
                bottom = pathBottom.coerceIn(top + 1, bottom),
            ),
            horizontalFallback = true,
        )
    }

    private fun selectHorizontalTrace(
        leftToRight: HorizontalTrace?,
        rightToLeft: HorizontalTrace?,
        preferredNearFieldX: Double?,
        preferredNearFieldY: Double?,
        preferRightToLeft: Boolean,
        frameShortSide: Double,
    ): HorizontalTrace? {
        val available = listOfNotNull(leftToRight, rightToLeft)
        if (available.isEmpty()) return null
        if (preferredNearFieldX != null && preferredNearFieldY != null) {
            return available.minByOrNull {
                hypot(
                    it.nearFieldCenterX - preferredNearFieldX,
                    it.nearFieldCenterY - preferredNearFieldY,
                )
            }
        }
        if (preferRightToLeft && rightToLeft != null) return rightToLeft
        return available.maxByOrNull {
            it.nearFieldCenterY + it.arcLength / frameShortSide
        }
    }

    private fun traceColumns(
        mask: ByteArray,
        frameWidth: Int,
        left: Int,
        top: Int,
        right: Int,
        bottom: Int,
        step: Int,
        minimumTrackedWidth: Double,
        maximumTrackedWidth: Double,
        expectedWidth: Double?,
    ): HorizontalTrace? {
        val capacity = right - left
        val pointsX = DoubleArray(capacity)
        val pointsY = DoubleArray(capacity)
        val widths = DoubleArray(capacity)
        var pointCount = 0
        var previousCenter = Double.NaN
        var previousWidth = Double.NaN
        var missingColumns = 0
        var x = if (step > 0) left else right - 1
        while (x in left until right) {
            val run = nearestColumnRun(
                mask = mask,
                frameWidth = frameWidth,
                x = x,
                top = top,
                bottom = bottom,
                previousCenter = previousCenter,
                previousWidth = previousWidth,
                minimumTrackedWidth = minimumTrackedWidth,
                maximumTrackedWidth = maximumTrackedWidth,
                expectedWidth = expectedWidth,
            )
            if (run == null) {
                if (pointCount > 0 && ++missingColumns > MAX_CONSECUTIVE_MISSING_COLUMNS) break
            } else {
                missingColumns = 0
                previousCenter = run.center
                previousWidth = run.width
                pointsX[pointCount] = x.toDouble()
                pointsY[pointCount] = run.center
                widths[pointCount] = run.width
                pointCount++
            }
            x += step
        }
        if (pointCount < MIN_SAMPLE_COUNT) return null

        val cumulativeArc = DoubleArray(pointCount)
        for (index in 1 until pointCount) {
            cumulativeArc[index] = cumulativeArc[index - 1] + hypot(
                pointsX[index] - pointsX[index - 1],
                pointsY[index] - pointsY[index - 1],
            )
        }
        val arcLength = cumulativeArc[pointCount - 1]
        val anchorSamples = minOf(NEAR_FIELD_SAMPLE_COUNT, pointCount)
        var nearFieldCenterX = 0.0
        var nearFieldCenterY = 0.0
        for (index in 0 until anchorSamples) {
            nearFieldCenterX += pointsX[index]
            nearFieldCenterY += pointsY[index]
        }
        return HorizontalTrace(
            pointsX = pointsX,
            pointsY = pointsY,
            widths = widths,
            cumulativeArc = cumulativeArc,
            pointCount = pointCount,
            arcLength = arcLength,
            nearFieldCenterX = nearFieldCenterX / anchorSamples,
            nearFieldCenterY = nearFieldCenterY / anchorSamples,
        )
    }

    private fun nearestColumnRun(
        mask: ByteArray,
        frameWidth: Int,
        x: Int,
        top: Int,
        bottom: Int,
        previousCenter: Double,
        previousWidth: Double,
        minimumTrackedWidth: Double,
        maximumTrackedWidth: Double,
        expectedWidth: Double?,
    ): PixelRun? {
        var best: PixelRun? = null
        var y = top
        while (y < bottom) {
            while (y < bottom && mask[y * frameWidth + x].toInt() == 0) y++
            if (y >= bottom) break
            val runStart = y
            while (y < bottom && mask[y * frameWidth + x].toInt() != 0) y++
            val candidate = PixelRun(runStart, y)
            if (candidate.width !in minimumTrackedWidth..maximumTrackedWidth) continue
            val candidateCost =
                if (previousCenter.isNaN()) {
                    expectedWidth?.let { abs(candidate.width - it) } ?: -candidate.width
                } else {
                    abs(candidate.center - previousCenter) +
                        abs(candidate.width - previousWidth) * WIDTH_CHANGE_COST
                }
            val bestCost = best?.let {
                if (previousCenter.isNaN()) {
                    expectedWidth?.let { expected -> abs(it.width - expected) } ?: -it.width
                } else {
                    abs(it.center - previousCenter) +
                        abs(it.width - previousWidth) * WIDTH_CHANGE_COST
                }
            }
            if (
                bestCost == null ||
                candidateCost < bestCost ||
                (candidateCost == bestCost && candidate.center > checkNotNull(best).center)
            ) {
                best = candidate
            }
        }
        return best
    }

    private data class HorizontalTrace(
        val pointsX: DoubleArray,
        val pointsY: DoubleArray,
        val widths: DoubleArray,
        val cumulativeArc: DoubleArray,
        val pointCount: Int,
        val arcLength: Double,
        val nearFieldCenterX: Double,
        val nearFieldCenterY: Double,
    )

    private fun lookaheadArcLength(arcLength: Double, frameShortSide: Double): Double =
        minOf(
            arcLength * LOOKAHEAD_ARC_FRACTION,
            frameShortSide * MAX_LOOKAHEAD_FRAME_FRACTION,
        )

    private fun tangentAngle(
        pointsX: DoubleArray,
        pointsY: DoubleArray,
        pointCount: Int,
        centerIndex: Int,
    ): Double? {
        val halfSpan =
            (pointCount / 20).coerceIn(MIN_TANGENT_HALF_SPAN, MAX_TANGENT_HALF_SPAN)
        val behind = (centerIndex - halfSpan).coerceAtLeast(0)
        val ahead = (centerIndex + halfSpan).coerceAtMost(pointCount - 1)
        val deltaX = pointsX[ahead] - pointsX[behind]
        val deltaY = pointsY[ahead] - pointsY[behind]
        if (deltaX == 0.0 && deltaY == 0.0) return null
        return TapeOrientation.deviationFromVerticalDegrees(deltaX, deltaY)
    }

    private fun curvatureDegrees(
        pointsX: DoubleArray,
        pointsY: DoubleArray,
        pointCount: Int,
    ): Double {
        val tangentSpan = (pointCount / CURVATURE_TANGENT_DIVISOR)
            .coerceIn(MIN_TANGENT_HALF_SPAN, MAX_CURVATURE_TANGENT_SPAN)
        val nearAngle = TapeOrientation.deviationFromVerticalDegrees(
            pointsX[tangentSpan] - pointsX[0],
            pointsY[tangentSpan] - pointsY[0],
        )
        val farAngle = TapeOrientation.deviationFromVerticalDegrees(
            pointsX[pointCount - 1] - pointsX[pointCount - 1 - tangentSpan],
            pointsY[pointCount - 1] - pointsY[pointCount - 1 - tangentSpan],
        )
        val difference = abs(nearAngle - farAngle)
        return minOf(difference, 180.0 - difference)
    }

    private fun curvatureSmoothness(
        pointsX: DoubleArray,
        pointsY: DoubleArray,
        pointCount: Int,
    ): Double {
        val segmentSize = pointCount / CURVATURE_SEGMENT_COUNT
        if (segmentSize < 2) return 0.0
        var previousAngle = segmentAngle(pointsX, pointsY, start = 0, segmentSize)
        var signedTurn = 0.0
        var absoluteTurn = 0.0
        var largestTurn = 0.0
        for (segmentIndex in 1 until CURVATURE_SEGMENT_COUNT) {
            val start = minOf(segmentIndex * segmentSize, pointCount - segmentSize)
            val angle = segmentAngle(pointsX, pointsY, start, segmentSize)
            var turn = angle - previousAngle
            if (turn > 90.0) turn -= 180.0
            if (turn < -90.0) turn += 180.0
            signedTurn += turn
            absoluteTurn += abs(turn)
            largestTurn = maxOf(largestTurn, abs(turn))
            previousAngle = angle
        }
        if (absoluteTurn == 0.0) return 0.0
        val directionConsistency = abs(signedTurn) / absoluteTurn
        val distribution = 1.0 - largestTurn / absoluteTurn
        return directionConsistency * distribution
    }

    private fun segmentAngle(
        pointsX: DoubleArray,
        pointsY: DoubleArray,
        start: Int,
        segmentSize: Int,
    ): Double {
        val end = start + segmentSize - 1
        return TapeOrientation.deviationFromVerticalDegrees(
            pointsX[end] - pointsX[start],
            pointsY[end] - pointsY[start],
        )
    }



    private fun nearestRun(
        mask: ByteArray,
        frameWidth: Int,
        y: Int,
        left: Int,
        right: Int,
        previousCenter: Double,
        previousWidth: Double,
        maximumLocalWidth: Double,
        initialCenterHint: Double?,
        preferRightmostInitialRun: Boolean,
    ): PixelRun? {
        var best: PixelRun? = null
        var x = left
        while (x < right) {
            while (x < right && mask[y * frameWidth + x].toInt() == 0) x++
            if (x >= right) break
            val runStart = x
            while (x < right && mask[y * frameWidth + x].toInt() != 0) x++
            val candidate = PixelRun(runStart, x)
            if (candidate.width > maximumLocalWidth) continue
            if (previousCenter.isNaN()) {
                val candidateCost =
                    initialCenterHint?.let { abs(candidate.center - it) } ?: -candidate.width
                val bestCost = best?.let {
                    initialCenterHint?.let { hint -> abs(it.center - hint) } ?: -it.width
                }
                if (
                    bestCost == null ||
                    candidateCost < bestCost ||
                    (
                        candidateCost == bestCost &&
                            preferRightmostInitialRun &&
                            candidate.center > checkNotNull(best).center
                        )
                ) {
                    best = candidate
                }
                continue
            }
            val candidateCost =
                abs(candidate.center - previousCenter) +
                    abs(candidate.width - previousWidth) * WIDTH_CHANGE_COST
            val bestCost = best?.let {
                abs(it.center - previousCenter) +
                    abs(it.width - previousWidth) * WIDTH_CHANGE_COST
            }
            if (bestCost == null || candidateCost < bestCost) best = candidate
        }
        return best
    }

    private data class PixelRun(val start: Int, val endExclusive: Int) {
        val width: Double get() = (endExclusive - start).toDouble()
        val center: Double get() = (start + endExclusive - 1) / 2.0
    }

    private companion object {
        const val LOOKAHEAD_ARC_FRACTION = 0.40
        const val MAX_LOOKAHEAD_FRAME_FRACTION = 0.40
        const val MIN_SAMPLE_COUNT = 12
        const val MIN_ARC_LENGTH_FRACTION = 0.12
        const val MIN_LOCAL_WIDTH_FRACTION = 0.025
        const val MAX_LOCAL_WIDTH_FRACTION = 0.20
        const val MIN_WIDTH_CONSISTENCY = 0.40
        const val WIDTH_LOWER_QUANTILE = 0.10
        const val WIDTH_UPPER_QUANTILE = 0.90
        const val WIDTH_CHANGE_COST = 0.75
        const val MAX_CONSECUTIVE_MISSING_ROWS = 3
        const val MAX_CONSECUTIVE_MISSING_COLUMNS = 3
        const val MIN_TRACKED_WIDTH_RATIO = 0.60
        const val MAX_TRACKED_WIDTH_RATIO = 1.60
        const val NEAR_FIELD_SAMPLE_COUNT = 8
        const val MIN_TANGENT_HALF_SPAN = 3
        const val CURVATURE_TANGENT_DIVISOR = 8
        const val CURVATURE_SEGMENT_COUNT = 5
        const val MAX_CURVATURE_TANGENT_SPAN = 24
        const val MAX_TANGENT_HALF_SPAN = 12
    }
}
