package com.durendal.droneagent.lite

import java.util.Arrays
import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

/**
 * Updates an established centerline by measuring the current binary mask along
 * normals to the preceding path. Acquisition, branches, loops, and periodic
 * topology validation remain the responsibility of [CenterlineExtractor].
 */
internal class TemporalCenterlineTracker(
    private val config: TemporalCenterlineTrackerConfig = TemporalCenterlineTrackerConfig(),
) {
    private var sampledIndices = IntArray(0)
    private var widthScratch = DoubleArray(0)
    fun track(
        mask: CenterlineMask,
        previous: CenterlineEstimate,
        maskOriginX: Int = 0,
        maskOriginY: Int = 0,
    ): CenterlineEstimate? {
        if (
            previous.points.size < config.minimumTrackedPoints ||
            previous.topology.branchCount > 0 ||
            previous.topology.closedLoop
        ) {
            return null
        }

        val sampledCount = sampleIndices(previous.points)
        if (sampledCount < config.minimumTrackedPoints) return null

        val tracked = ArrayList<CenterlinePoint>(sampledCount)
        var supportedSamples = 0
        var consecutiveMisses = 0
        var maximumConsecutiveMisses = 0
        for (samplePosition in 0 until sampledCount) {
            val pointIndex = sampledIndices[samplePosition]
            val point = previous.points[pointIndex]
            val before = previous.points[max(0, pointIndex - config.tangentPointRadius)]
            val after = previous.points[min(previous.points.lastIndex, pointIndex + config.tangentPointRadius)]
            val tangentX = after.x - before.x
            val tangentY = after.y - before.y
            val tangentLength = hypot(tangentX, tangentY)
            if (tangentLength < MINIMUM_TANGENT_LENGTH) return null
            val trackedPoint = scanNormal(
                mask = mask,
                previousX = point.x - maskOriginX,
                previousY = point.y - maskOriginY,
                previousWidth = point.widthPixels,
                tangentX = tangentX / tangentLength,
                tangentY = tangentY / tangentLength,
            )
            if (trackedPoint == null) {
                consecutiveMisses++
                maximumConsecutiveMisses = max(maximumConsecutiveMisses, consecutiveMisses)
                continue
            }
            if (consecutiveMisses > 0) {
                // A missing cross-section is allowed only as a short glare gap.
                // The resulting direct segment is still bounded below by the
                // same maximum-gap check used for every pair of tracked points.
                consecutiveMisses = 0
            }
            if (
                tracked.isNotEmpty() &&
                distance(tracked.last(), trackedPoint) > config.maximumTrackedPointGapPixels
            ) {
                return null
            }
            tracked += trackedPoint
            supportedSamples++
        }

        val support = supportedSamples.toDouble() / sampledCount
        if (
            tracked.size < config.minimumTrackedPoints ||
            support < config.minimumSupportFraction ||
            maximumConsecutiveMisses > config.maximumConsecutiveMisses
        ) {
            return null
        }

        val smoothed = smooth(tracked)
        val widthConsistency = widthConsistency(smoothed)
        if (widthConsistency < config.minimumWidthConsistency) return null
        val continuity = continuity(smoothed)
        if (continuity < config.minimumContinuity) return null
        val fitResidual = fitResidual(smoothed)
        val aggregate = weightedGeometricMean(support, widthConsistency, continuity, fitResidual)
        val components = CenterlineConfidence(
            support = support,
            widthConsistency = widthConsistency,
            continuity = continuity,
            fitResidual = fitResidual,
            aggregate = aggregate,
        )
        return CenterlineEstimate(
            points = smoothed,
            confidence = aggregate,
            components = components,
        )
    }

    private fun sampleIndices(points: List<CenterlinePoint>): Int {
        if (sampledIndices.size < points.size) sampledIndices = IntArray(points.size)
        var count = 0
        var accumulated = 0.0
        sampledIndices[count++] = 0
        for (index in 1 until points.lastIndex) {
            accumulated += distance(points[index - 1], points[index])
            if (accumulated >= config.sampleSpacingPixels) {
                sampledIndices[count++] = index
                accumulated = 0.0
            }
        }
        if (points.lastIndex > 0 && sampledIndices[count - 1] != points.lastIndex) {
            sampledIndices[count++] = points.lastIndex
        }
        return count
    }

    private fun scanNormal(
        mask: CenterlineMask,
        previousX: Double,
        previousY: Double,
        previousWidth: Double,
        tangentX: Double,
        tangentY: Double,
    ): CenterlinePoint? {
        val normalX = -tangentY
        val normalY = tangentX
        val searchRadius = max(
            config.minimumSearchRadiusPixels,
            previousWidth * config.searchRadiusWidthFactor,
        ).coerceAtMost(config.maximumSearchRadiusPixels)
        val maximumDisplacement = max(
            config.minimumMaximumDisplacementPixels,
            previousWidth * config.maximumDisplacementWidthFactor,
        )
        val minimumWidth = max(1.0, previousWidth * config.minimumWidthRatio)
        val maximumWidth = previousWidth * config.maximumWidthRatio
        val radiusSteps = searchRadius.roundToInt()

        var bestMidpoint = Double.NaN
        var bestWidth = 0.0
        var bestDistance = Double.POSITIVE_INFINITY
        var runStart = 0
        var inRun = false
        var previousPixelX = Int.MIN_VALUE
        var previousPixelY = Int.MIN_VALUE
        for (step in -radiusSteps..radiusSteps + 1) {
            val inside = if (step <= radiusSteps) {
                val pixelX = (previousX + normalX * step).roundToInt()
                val pixelY = (previousY + normalY * step).roundToInt()
                if (pixelX == previousPixelX && pixelY == previousPixelY) {
                    continue
                }
                previousPixelX = pixelX
                previousPixelY = pixelY
                pixelX in 0 until mask.width &&
                    pixelY in 0 until mask.height &&
                    mask.hasTapeInRectangle(pixelX, pixelY, pixelX + 1, pixelY + 1)
            } else {
                false
            }
            if (inside && !inRun) {
                runStart = step
                inRun = true
            } else if (!inside && inRun) {
                val runEnd = step - 1
                val width = (runEnd - runStart + 1).toDouble()
                val midpoint = (runStart + runEnd) / 2.0
                val midpointDistance = abs(midpoint)
                if (
                    width in minimumWidth..maximumWidth &&
                    midpointDistance <= maximumDisplacement &&
                    midpointDistance < bestDistance
                ) {
                    bestMidpoint = midpoint
                    bestWidth = width
                    bestDistance = midpointDistance
                }
                inRun = false
            }
        }
        if (!bestMidpoint.isFinite()) return null
        return CenterlinePoint(
            x = previousX + normalX * bestMidpoint,
            y = previousY + normalY * bestMidpoint,
            widthPixels = bestWidth,
        )
    }

    private fun smooth(points: List<CenterlinePoint>): List<CenterlinePoint> {
        if (points.size < 3) return points
        return List(points.size) { index ->
            if (index == 0 || index == points.lastIndex) {
                points[index]
            } else {
                val before = points[index - 1]
                val current = points[index]
                val after = points[index + 1]
                CenterlinePoint(
                    x = (before.x + current.x * 2.0 + after.x) / 4.0,
                    y = (before.y + current.y * 2.0 + after.y) / 4.0,
                    widthPixels = current.widthPixels,
                )
            }
        }
    }

    private fun widthConsistency(points: List<CenterlinePoint>): Double {
        if (widthScratch.size < points.size) widthScratch = DoubleArray(points.size)
        for (index in points.indices) widthScratch[index] = points[index].widthPixels
        Arrays.sort(widthScratch, 0, points.size)
        val median = widthScratch[points.size / 2].coerceAtLeast(1.0)
        var relativeDeviation = 0.0
        for (index in points.indices) relativeDeviation += abs(widthScratch[index] - median) / median
        val meanRelativeDeviation = relativeDeviation / points.size
        return exp(-meanRelativeDeviation * WIDTH_DEVIATION_PENALTY).coerceIn(0.0, 1.0)
    }

    private fun continuity(points: List<CenterlinePoint>): Double {
        if (points.size < 2) return 0.0
        var longest = 0.0
        var shortest = Double.POSITIVE_INFINITY
        for (index in 1 until points.size) {
            val segment = distance(points[index - 1], points[index])
            longest = max(longest, segment)
            shortest = min(shortest, segment)
        }
        if (longest <= 0.0) return 0.0
        return (shortest / longest).coerceIn(0.0, 1.0)
    }

    private fun fitResidual(points: List<CenterlinePoint>): Double {
        if (points.size < 3) return 1.0
        var normalizedResidual = 0.0
        for (index in 1 until points.lastIndex) {
            val previous = points[index - 1]
            val current = points[index]
            val next = points[index + 1]
            val midpointX = (previous.x + next.x) / 2.0
            val midpointY = (previous.y + next.y) / 2.0
            normalizedResidual +=
                hypot(current.x - midpointX, current.y - midpointY) /
                current.widthPixels.coerceAtLeast(1.0)
        }
        return exp(-normalizedResidual / (points.size - 2)).coerceIn(0.0, 1.0)
    }

    private fun weightedGeometricMean(
        support: Double,
        widthConsistency: Double,
        continuity: Double,
        fitResidual: Double,
    ): Double = exp(
        SUPPORT_WEIGHT * kotlin.math.ln(support.coerceAtLeast(MINIMUM_CONFIDENCE_COMPONENT)) +
            WIDTH_WEIGHT * kotlin.math.ln(widthConsistency.coerceAtLeast(MINIMUM_CONFIDENCE_COMPONENT)) +
            CONTINUITY_WEIGHT * kotlin.math.ln(continuity.coerceAtLeast(MINIMUM_CONFIDENCE_COMPONENT)) +
            FIT_WEIGHT * kotlin.math.ln(fitResidual.coerceAtLeast(MINIMUM_CONFIDENCE_COMPONENT)),
    ).coerceIn(0.0, 1.0)

    private fun distance(first: CenterlinePoint, second: CenterlinePoint): Double =
        hypot(first.x - second.x, first.y - second.y)

    private companion object {
        const val MINIMUM_TANGENT_LENGTH = 1e-6
        const val WIDTH_DEVIATION_PENALTY = 2.5
        const val MINIMUM_CONFIDENCE_COMPONENT = 1e-6
        const val SUPPORT_WEIGHT = 0.35
        const val WIDTH_WEIGHT = 0.30
        const val CONTINUITY_WEIGHT = 0.20
        const val FIT_WEIGHT = 0.15
    }
}

internal data class TemporalCenterlineTrackerConfig(
    val sampleSpacingPixels: Double = 6.0,
    val tangentPointRadius: Int = 3,
    val minimumTrackedPoints: Int = 8,
    val minimumSupportFraction: Double = 0.72,
    val maximumConsecutiveMisses: Int = 2,
    val minimumSearchRadiusPixels: Double = 14.0,
    val maximumSearchRadiusPixels: Double = 64.0,
    val searchRadiusWidthFactor: Double = 2.0,
    val minimumMaximumDisplacementPixels: Double = 8.0,
    val maximumDisplacementWidthFactor: Double = 1.25,
    val minimumWidthRatio: Double = 0.45,
    val maximumWidthRatio: Double = 2.0,
    val maximumTrackedPointGapPixels: Double = 24.0,
    val minimumWidthConsistency: Double = 0.45,
    val minimumContinuity: Double = 0.20,
) {
    init {
        require(sampleSpacingPixels > 0.0)
        require(tangentPointRadius > 0)
        require(minimumTrackedPoints >= 3)
        require(minimumSupportFraction in 0.0..1.0)
        require(maximumConsecutiveMisses >= 0)
        require(minimumSearchRadiusPixels > 0.0)
        require(maximumSearchRadiusPixels >= minimumSearchRadiusPixels)
        require(searchRadiusWidthFactor > 0.0)
        require(minimumMaximumDisplacementPixels > 0.0)
        require(maximumDisplacementWidthFactor > 0.0)
        require(minimumWidthRatio > 0.0)
        require(maximumWidthRatio >= minimumWidthRatio)
        require(maximumTrackedPointGapPixels > 0.0)
        require(minimumWidthConsistency in 0.0..1.0)
        require(minimumContinuity in 0.0..1.0)
    }
}
