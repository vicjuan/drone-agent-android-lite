package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.roundToInt

/** One locally connected black-tape path, ordered from the camera-near end forward. */
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
 * Extracts the locally-followable centerline of one isolated tape contour.
 *
 * The normal path is sampled from the camera-near bottom edge upward. During a tight turn the
 * ribbon can become almost horizontal, so the same run tracker is also applied from both sides.
 * The side trace whose first samples are nearer the bottom is the physically useful direction.
 * Choosing the strongest of these three traces keeps the tangent observable while the aircraft
 * yaws back into alignment, without accepting a different contour or extrapolating through gaps.
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

        val vertical = trace(
            mask = mask,
            frameWidth = frameWidth,
            frameHeight = frameHeight,
            left = left,
            top = top,
            right = right,
            bottom = bottom,
            axis = TraceAxis.VERTICAL,
            reverse = true,
        )
        val fromLeft = trace(
            mask = mask,
            frameWidth = frameWidth,
            frameHeight = frameHeight,
            left = left,
            top = top,
            right = right,
            bottom = bottom,
            axis = TraceAxis.HORIZONTAL,
            reverse = false,
        )
        val fromRight = trace(
            mask = mask,
            frameWidth = frameWidth,
            frameHeight = frameHeight,
            left = left,
            top = top,
            right = right,
            bottom = bottom,
            axis = TraceAxis.HORIZONTAL,
            reverse = true,
        )
        val horizontal = when {
            fromLeft == null -> fromRight
            fromRight == null -> fromLeft
            fromLeft.nearFieldCenterY >= fromRight.nearFieldCenterY -> fromLeft
            else -> fromRight
        }
        return stronger(vertical, horizontal)?.estimate
    }

    private fun trace(
        mask: ByteArray,
        frameWidth: Int,
        frameHeight: Int,
        left: Int,
        top: Int,
        right: Int,
        bottom: Int,
        axis: TraceAxis,
        reverse: Boolean,
    ): TraceResult? {
        val frameShortSide = minOf(frameWidth, frameHeight).toDouble()
        val maximumLocalWidth = frameShortSide * MAX_LOCAL_WIDTH_FRACTION
        val majorStart: Int
        val majorLimit: Int
        val majorStep: Int
        val minorStart: Int
        val minorLimit: Int
        val capacity: Int
        if (axis == TraceAxis.VERTICAL) {
            majorStart = if (reverse) bottom - 1 else top
            majorLimit = if (reverse) top - 1 else bottom
            majorStep = if (reverse) -1 else 1
            minorStart = left
            minorLimit = right
            capacity = bottom - top
        } else {
            majorStart = if (reverse) right - 1 else left
            majorLimit = if (reverse) left - 1 else right
            majorStep = if (reverse) -1 else 1
            minorStart = top
            minorLimit = bottom
            capacity = right - left
        }

        val pointsX = DoubleArray(capacity)
        val pointsY = DoubleArray(capacity)
        val widths = DoubleArray(capacity)
        var pointCount = 0
        var previousCenter = Double.NaN
        var previousWidth = Double.NaN
        var missingSlices = 0
        var major = majorStart
        while (major != majorLimit) {
            val run = nearestRun(
                mask = mask,
                frameWidth = frameWidth,
                major = major,
                minorStart = minorStart,
                minorLimit = minorLimit,
                previousCenter = previousCenter,
                previousWidth = previousWidth,
                maximumLocalWidth = maximumLocalWidth,
                axis = axis,
            )
            if (run == null) {
                if (pointCount > 0 && ++missingSlices > MAX_CONSECUTIVE_MISSING_SLICES) break
                major += majorStep
                continue
            }
            missingSlices = 0
            previousCenter = run.center
            previousWidth = run.width
            if (axis == TraceAxis.VERTICAL) {
                pointsX[pointCount] = run.center
                pointsY[pointCount] = major.toDouble()
            } else {
                pointsX[pointCount] = major.toDouble()
                pointsY[pointCount] = run.center
            }
            widths[pointCount] = run.width
            pointCount++
            major += majorStep
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
        var nearFieldCenterY = 0.0
        for (index in 0 until anchorSamples) {
            nearFieldCenterX += pointsX[index]
            nearFieldCenterY += pointsY[index]
        }
        nearFieldCenterX /= anchorSamples
        nearFieldCenterY /= anchorSamples

        val pathBounds = pathBounds(
            pointsX = pointsX,
            pointsY = pointsY,
            widths = widths,
            pointCount = pointCount,
            axis = axis,
            left = left,
            top = top,
            right = right,
            bottom = bottom,
        )
        val estimate = TapePathEstimate(
            nearFieldCenterX = nearFieldCenterX,
            lookaheadAngleFromVerticalDegrees =
                TapeOrientation.deviationFromVerticalDegrees(deltaX, deltaY),
            arcLengthFraction = arcLengthFraction,
            medianWidthFraction = medianWidthFraction,
            widthConsistency = widthConsistency,
            sampleCount = pointCount,
            bounds = pathBounds,
        )
        return TraceResult(
            estimate = estimate,
            nearFieldCenterY = nearFieldCenterY,
            quality = arcLengthFraction * widthConsistency,
        )
    }

    private fun pathBounds(
        pointsX: DoubleArray,
        pointsY: DoubleArray,
        widths: DoubleArray,
        pointCount: Int,
        axis: TraceAxis,
        left: Int,
        top: Int,
        right: Int,
        bottom: Int,
    ): TapePathBounds {
        var pathLeft = right
        var pathTop = bottom
        var pathRight = left
        var pathBottom = top
        for (index in 0 until pointCount) {
            if (axis == TraceAxis.VERTICAL) {
                val runStart = (pointsX[index] - (widths[index] - 1.0) / 2.0).roundToInt()
                pathLeft = minOf(pathLeft, runStart)
                pathRight = maxOf(pathRight, runStart + widths[index].toInt())
                pathTop = minOf(pathTop, pointsY[index].toInt())
                pathBottom = maxOf(pathBottom, pointsY[index].toInt() + 1)
            } else {
                val runStart = (pointsY[index] - (widths[index] - 1.0) / 2.0).roundToInt()
                pathLeft = minOf(pathLeft, pointsX[index].toInt())
                pathRight = maxOf(pathRight, pointsX[index].toInt() + 1)
                pathTop = minOf(pathTop, runStart)
                pathBottom = maxOf(pathBottom, runStart + widths[index].toInt())
            }
        }
        return TapePathBounds(
            left = pathLeft.coerceIn(left, right - 1),
            top = pathTop.coerceIn(top, bottom - 1),
            right = pathRight.coerceIn(left + 1, right),
            bottom = pathBottom.coerceIn(top + 1, bottom),
        )
    }

    private fun nearestRun(
        mask: ByteArray,
        frameWidth: Int,
        major: Int,
        minorStart: Int,
        minorLimit: Int,
        previousCenter: Double,
        previousWidth: Double,
        maximumLocalWidth: Double,
        axis: TraceAxis,
    ): PixelRun? {
        var best: PixelRun? = null
        var minor = minorStart
        while (minor < minorLimit) {
            while (minor < minorLimit && !isSet(mask, frameWidth, major, minor, axis)) minor++
            if (minor >= minorLimit) break
            val runStart = minor
            while (minor < minorLimit && isSet(mask, frameWidth, major, minor, axis)) minor++
            val candidate = PixelRun(runStart, minor)
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

    private fun isSet(
        mask: ByteArray,
        frameWidth: Int,
        major: Int,
        minor: Int,
        axis: TraceAxis,
    ): Boolean {
        val index =
            if (axis == TraceAxis.VERTICAL) major * frameWidth + minor
            else minor * frameWidth + major
        return mask[index].toInt() != 0
    }

    private fun stronger(first: TraceResult?, second: TraceResult?): TraceResult? = when {
        first == null -> second
        second == null -> first
        first.quality >= second.quality -> first
        else -> second
    }

    private enum class TraceAxis { VERTICAL, HORIZONTAL }

    private data class TraceResult(
        val estimate: TapePathEstimate,
        val nearFieldCenterY: Double,
        val quality: Double,
    )

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
        const val MAX_CONSECUTIVE_MISSING_SLICES = 3
        const val NEAR_FIELD_SAMPLE_COUNT = 8
        const val MIN_TANGENT_HALF_SPAN = 3
        const val MAX_TANGENT_HALF_SPAN = 12
    }
}
