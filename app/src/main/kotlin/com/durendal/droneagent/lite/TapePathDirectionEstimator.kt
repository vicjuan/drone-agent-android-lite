package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.roundToInt

/** One locally connected black-tape path, ordered from the camera-near end upward. */
internal data class TapePathEstimate(
    val nearFieldCenterX: Double,
    val lookaheadAngleFromVerticalDegrees: Double,
    val arcLengthFraction: Double,
    val medianWidthFraction: Double,
    val widthConsistency: Double,
    val sampleCount: Int,
    val bounds: TapePathBounds,
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
 * contour's bounding-box chord for its direction. The steering tangent is measured 40% of the
 * traced arc ahead, matching the look-ahead convention proven in the full drone-agent vision
 * pipeline. A path that turns horizontal enough to stop producing a bounded local run fails
 * closed instead of inventing a direction.
 */
internal class TapePathDirectionEstimator {
    fun estimate(
        mask: ByteArray,
        frameWidth: Int,
        frameHeight: Int,
        left: Int,
        top: Int,
        right: Int,
        bottom: Int,
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

        val targetArc = arcLength * LOOKAHEAD_ARC_FRACTION
        var lookaheadIndex = 0
        while (lookaheadIndex < pointCount - 1 && cumulativeArc[lookaheadIndex] < targetArc) {
            lookaheadIndex++
        }
        val tangentSpan = (pointCount / 20).coerceIn(MIN_TANGENT_HALF_SPAN, MAX_TANGENT_HALF_SPAN)
        val behind = (lookaheadIndex - tangentSpan).coerceAtLeast(0)
        val ahead = (lookaheadIndex + tangentSpan).coerceAtMost(pointCount - 1)
        val deltaX = pointsX[ahead] - pointsX[behind]
        val deltaY = pointsY[ahead] - pointsY[behind]
        if (deltaX == 0.0 && deltaY == 0.0) return null

        val anchorSamples = minOf(NEAR_FIELD_SAMPLE_COUNT, pointCount)
        var nearFieldCenterX = 0.0
        for (index in 0 until anchorSamples) nearFieldCenterX += pointsX[index]
        nearFieldCenterX /= anchorSamples
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
            lookaheadAngleFromVerticalDegrees =
                TapeOrientation.deviationFromVerticalDegrees(deltaX, deltaY),
            arcLengthFraction = arcLengthFraction,
            medianWidthFraction = medianWidthFraction,
            widthConsistency = widthConsistency,
            sampleCount = pointCount,
            bounds = pathBounds,
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
                if (best == null || candidate.width > best.width) best = candidate
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
        const val MIN_SAMPLE_COUNT = 12
        const val MIN_ARC_LENGTH_FRACTION = 0.20
        const val MIN_LOCAL_WIDTH_FRACTION = 0.025
        const val MAX_LOCAL_WIDTH_FRACTION = 0.20
        const val MIN_WIDTH_CONSISTENCY = 0.40
        const val WIDTH_LOWER_QUANTILE = 0.10
        const val WIDTH_UPPER_QUANTILE = 0.90
        const val WIDTH_CHANGE_COST = 0.75
        const val MAX_CONSECUTIVE_MISSING_ROWS = 3
        const val NEAR_FIELD_SAMPLE_COUNT = 8
        const val MIN_TANGENT_HALF_SPAN = 3
        const val MAX_TANGENT_HALF_SPAN = 12
    }
}
