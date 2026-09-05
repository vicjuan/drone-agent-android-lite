package com.durendal.droneagent.lite

import java.util.Arrays
import kotlin.math.PI
import kotlin.math.acos
import kotlin.math.atan2
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/** One ordered centerline sample in full-resolution image coordinates. */
internal data class CenterlinePoint(
    val x: Double,
    val y: Double,
    val widthPixels: Double,
)

/** Observable, direction-independent contributors to centerline confidence. */
internal data class CenterlineConfidence(
    val support: Double,
    val widthConsistency: Double,
    val continuity: Double,
    val fitResidual: Double,
    val aggregate: Double,
)

/** Why the far end of the chain stops where it does. */
internal enum class CenterlineTerminus { INSIDE_FRAME, AT_FRAME_BORDER, NONE }

/**
 * Topology facts about the extracted chain, additive and observation-only.
 * [distalTerminus] is the discriminator the mission layer needs: a chain that
 * ends inside the frame may be a real end of tape; one truncated at a border
 * only means the tape left the field of view. [branchCount] counts credible
 * skeleton divergences from the reported chain so a junction can fail safe
 * instead of silently trusting the extractor's guessed continuation.
 */
internal data class CenterlineTopology(
    val distalTerminus: CenterlineTerminus = CenterlineTerminus.NONE,
    val distalBorderDistancePixels: Double = 0.0,
    val branchCount: Int = 0,
    val closedLoop: Boolean = false,
)

/**
 * Extracted tape centerline. Points follow the skeleton path, beginning at the
 * endpoint nearest the bottom of the image. [confidence] is exactly
 * [CenterlineConfidence.aggregate]. [topology] is additive context and never
 * influences the extracted points.
 */
internal data class CenterlineEstimate(
    val points: List<CenterlinePoint>,
    val confidence: Double,
    val components: CenterlineConfidence,
    val topology: CenterlineTopology = CenterlineTopology(),
)

/**
 * Plausible tape full-width in full-resolution image pixels.
 *
 * A single range instance is shared by segmentation and centerline extraction
 * for a frame, so the two stages cannot apply contradictory width priors.
 */
internal data class TapeWidthRangePixels(
    val minimum: Double,
    val maximum: Double,
) {
    init {
        require(minimum.isFinite() && minimum > 0.0) { "minimum must be finite and > 0" }
        require(maximum.isFinite() && maximum >= minimum) {
            "maximum must be finite and >= minimum"
        }
    }
}

/**
 * Conservative fallback when no caller-projected width prior is available.
 * A camera-geometry-aware caller replaces this with a per-frame range
 * projected from intrinsics, pose, and the physical tape width.
 */
private val DEFAULT_TAPE_WIDTH_RANGE_PIXELS = TapeWidthRangePixels(minimum = 6.0, maximum = 160.0)

internal interface CenterlineMask {
    val width: Int
    val height: Int
    val tapePixelCount: Int

    fun hasTapeInRectangle(
        left: Int,
        top: Int,
        rightExclusive: Int,
        bottomExclusive: Int,
    ): Boolean
}

/**
 * Binary tape candidate mask, row-major, same dimensions as the source frame.
 * `true` marks a pixel classified as tape. The array is caller-owned and never
 * copied, so a detector can hand its per-frame scratch mask straight in.
 */
internal class SegmentationMask(
    override val width: Int,
    override val height: Int,
    val tape: BooleanArray,
) : CenterlineMask {
    init {
        require(width > 0 && height > 0) { "mask dimensions must be positive" }
        require(tape.size == width * height) {
            "mask size ${tape.size} does not match $width x $height"
        }
    }

    /** Eager because every consumer gates on emptiness before touching pixels. */
    override val tapePixelCount: Int = tape.count { it }

    /** Fraction of the frame classified as tape, in 0.0..1.0. */
    val tapeFraction: Double get() = tapePixelCount.toDouble() / (width * height)

    /** Returns whether a non-empty full-resolution rectangle contains tape. */
    override fun hasTapeInRectangle(
        left: Int,
        top: Int,
        rightExclusive: Int,
        bottomExclusive: Int,
    ): Boolean {
        for (y in top until bottomExclusive) {
            var index = y * width + left
            val end = y * width + rightExclusive
            while (index < end) {
                if (tape[index]) return true
                index++
            }
        }
        return false
    }

    companion object {
        fun empty(width: Int, height: Int): SegmentationMask =
            SegmentationMask(width, height, BooleanArray(width * height))
    }
}

/**
 * OpenCV binary mask adapter. Non-zero bytes are tape pixels, including signed
 * 0xFF values; the detector's reusable Mat buffer can therefore feed the
 * extractor without a second full-region BooleanArray conversion.
 */
internal class ByteSegmentationMask(
    override val width: Int,
    override val height: Int,
    private val tape: ByteArray,
) : CenterlineMask {
    init {
        require(width > 0 && height > 0) { "mask dimensions must be positive" }
        require(tape.size == width * height) {
            "mask size ${tape.size} does not match $width x $height"
        }
    }

    override val tapePixelCount: Int = tape.count { it != 0.toByte() }

    override fun hasTapeInRectangle(
        left: Int,
        top: Int,
        rightExclusive: Int,
        bottomExclusive: Int,
    ): Boolean {
        for (y in top until bottomExclusive) {
            var index = y * width + left
            val end = y * width + rightExclusive
            while (index < end) {
                if (tape[index] != 0.toByte()) return true
                index++
            }
        }
        return false
    }
}

/**
 * Tape geometry and continuity limits accepted by [CenterlineExtractor].
 *
 * [defaultTapeWidthRangePixels] is the local skeleton safety envelope. The
 * minimum is softened because perspective and thinning make local centerline
 * width narrower than a component-wide area/length estimate. The maximum gets
 * bounded headroom because close working distances and curved joins make local
 * distance-transform width wider than that component-wide estimate.
 *
 * [minDistalBorderMarginPixels] and [distalBorderMarginWidthFactor] bound the
 * distal-terminus classification margin. Thinning retracts the medial axis of
 * a border-cut ribbon by half the local tape width (the centerline of a
 * rectangle stops half a width short of each end), so a terminus nearer to a
 * border than that is geometrically indistinguishable from frame truncation.
 * The factor keeps headroom above that 0.5-width bound for diagonal cuts and
 * thinning asymmetry; the pixel floor covers working-grid quantization when
 * the tape is thin (working coordinates are factor-quantized, factor =
 * ceil(height / 270)).
 *
 * [minBranchArcLengthPixels] and [branchArcLengthWidthFactor] separate genuine
 * tape divergences from thinning spurs. A bump or staircase corner grows a
 * medial spur of at most about 0.7 local widths (the corner bisector of a
 * right-angled ribbon end is half-width times sqrt(2)), while a real tape arm
 * carries many widths of arc, so 1.5 widths cleanly separates the two.
 */
internal data class CenterlineExtractorConfig(
    val defaultTapeWidthRangePixels: TapeWidthRangePixels = DEFAULT_TAPE_WIDTH_RANGE_PIXELS,
    val maxChainGapPixels: Double = 24.0,
    val minPointCount: Int = 5,
    val minDistalBorderMarginPixels: Double = 12.0,
    val distalBorderMarginWidthFactor: Double = 0.75,
    val minBranchArcLengthPixels: Double = 12.0,
    val branchArcLengthWidthFactor: Double = 1.5,
) {
    init {
        require(maxChainGapPixels > 0.0) { "maxChainGapPixels must be > 0" }
        require(minPointCount > 0) { "minPointCount must be > 0" }
        require(minDistalBorderMarginPixels > 0.0) { "minDistalBorderMarginPixels must be > 0" }
        require(distalBorderMarginWidthFactor > 0.0) { "distalBorderMarginWidthFactor must be > 0" }
        require(minBranchArcLengthPixels > 0.0) { "minBranchArcLengthPixels must be > 0" }
        require(branchArcLengthWidthFactor > 0.0) { "branchArcLengthWidthFactor must be > 0" }
    }
}

/** Per-extraction timings aggregated by the detector's frame profile. */
internal data class CenterlineStageProfile(
    val downsampleNanos: Long,
    val gapFillNanos: Long,
    val distanceNanos: Long,
    val thinningNanos: Long,
    val routeNanos: Long,
    val qualityNanos: Long,
    val topologyNanos: Long,
)


/**
 * Direction-independent centerline extraction.
 *
 * The mask is reduced to at most [TARGET_WORKING_HEIGHT] rows, transformed by
 * a 3-4 chamfer distance pass, thinned with Zhang-Suen, then walked as an
 * 8-connected skeleton. This removes the former row-scan bias that rewarded
 * vertical objects and discarded horizontal or diagonal tape.
 */
internal class CenterlineExtractor(
    private val config: CenterlineExtractorConfig = CenterlineExtractorConfig(),
    private val onProfile: (CenterlineStageProfile) -> Unit = {},
) {
    /** Analyzer gate; exposed so the stage machine does not duplicate config. */
    val minPointCount: Int get() = config.minPointCount

    // Geometry-sized primitive workspaces. The extractor is single-consumer
    // and reuses these arrays so a 1080p frame does not allocate several MiB
    // before producing its intentionally-owned CenterlinePoint list.
    private var workspaceSize = 0
    private var foreground = BooleanArray(0)
    private var visited = BooleanArray(0)
    private var skeleton = BooleanArray(0)
    private var indexBuffer = IntArray(0)
    private var distances = IntArray(0)
    private var removalBuffer = IntArray(0)
    private var thinningCandidates = BooleanArray(0)
    private var routeCost = IntArray(0)
    private var routeLength = IntArray(0)
    private var routeStrength = IntArray(0)
    private var predecessor = IntArray(0)
    private var heapNodes = IntArray(0)
    private var heapCosts = IntArray(0)
    private var heapPositions = IntArray(0)

    // Whether the last route fell back to the lowest skeleton point because
    // the skeleton offered no degree-<=1 endpoint at all, i.e. it closes on
    // itself. Valid for the extract() call in flight; the extractor is
    // single-consumer by contract.
    private var routeClosedLoop = false

    fun extract(
        mask: CenterlineMask,
        tapeWidthRangePixels: TapeWidthRangePixels = config.defaultTapeWidthRangePixels,
        fillEnclosedGaps: Boolean = true,
    ): CenterlineEstimate {
        var downsampleNanos = 0L
        var gapFillNanos = 0L
        var distanceNanos = 0L
        var thinningNanos = 0L
        var routeNanos = 0L
        var qualityNanos = 0L
        var topologyNanos = 0L
        fun profiled(estimate: CenterlineEstimate): CenterlineEstimate {
            onProfile(
                CenterlineStageProfile(
                    downsampleNanos = downsampleNanos,
                    gapFillNanos = gapFillNanos,
                    distanceNanos = distanceNanos,
                    thinningNanos = thinningNanos,
                    routeNanos = routeNanos,
                    qualityNanos = qualityNanos,
                    topologyNanos = topologyNanos,
                ),
            )
            return estimate
        }

        if (mask.tapePixelCount == 0) return profiled(emptyEstimate())

        var stageStartedAtNanos = System.nanoTime()
        val factor = max(1, ceil(mask.height.toDouble() / TARGET_WORKING_HEIGHT).toInt())
        val workingWidth = (mask.width + factor - 1) / factor
        val workingHeight = (mask.height + factor - 1) / factor
        val paddedWidth = workingWidth + 2
        val paddedHeight = workingHeight + 2
        ensureWorkspace(paddedWidth * paddedHeight)
        val minimumLocalWidth =
            tapeWidthRangePixels.minimum * MINIMUM_WIDTH_TOLERANCE
        val maximumLocalWidth = max(
            config.defaultTapeWidthRangePixels.maximum,
            tapeWidthRangePixels.maximum * MAXIMUM_WIDTH_HEADROOM,
        )
        Arrays.fill(foreground, false)

        var foregroundCount = 0
        for (wy in 0 until workingHeight) {
            val yStart = wy * factor
            val yEnd = min(yStart + factor, mask.height)
            for (wx in 0 until workingWidth) {
                val xStart = wx * factor
                val xEnd = min(xStart + factor, mask.width)
                if (mask.hasTapeInRectangle(xStart, yStart, xEnd, yEnd)) {
                    foreground[(wy + 1) * paddedWidth + wx + 1] = true
                    foregroundCount++
                }
            }
        }
        downsampleNanos = System.nanoTime() - stageStartedAtNanos
        if (foregroundCount == 0) return profiled(emptyEstimate())

        stageStartedAtNanos = System.nanoTime()
        if (fillEnclosedGaps) {
            fillEnclosedGaps(foreground, paddedWidth, paddedHeight, factor)
        }
        gapFillNanos = System.nanoTime() - stageStartedAtNanos

        stageStartedAtNanos = System.nanoTime()
        chamferDistance(foreground, paddedWidth, paddedHeight)
        distanceNanos = System.nanoTime() - stageStartedAtNanos

        stageStartedAtNanos = System.nanoTime()
        System.arraycopy(foreground, 0, skeleton, 0, workspaceSize)
        thinZhangSuen(skeleton, paddedWidth, paddedHeight, indexBuffer)
        var skeletonCount = 0
        for (py in 1 until paddedHeight - 1) {
            for (px in 1 until paddedWidth - 1) {
                val index = py * paddedWidth + px
                if (!skeleton[index]) continue
                val widthPixels = widthPixels(distances[index], factor)
                if (widthPixels in minimumLocalWidth..maximumLocalWidth) {
                    skeletonCount++
                } else {
                    skeleton[index] = false
                }
            }
        }
        thinningNanos = System.nanoTime() - stageStartedAtNanos
        if (skeletonCount == 0) return profiled(emptyEstimate())

        stageStartedAtNanos = System.nanoTime()
        val minimumRouteHalfWidthDistance = ceil(
            tapeWidthRangePixels.minimum * ORTHOGONAL_COST / (2.0 * factor),
        ).toInt().coerceAtLeast(1)
        var selectedCount = longestPathFromBottom(
            skeleton = skeleton,
            widthDistances = distances,
            width = paddedWidth,
            height = paddedHeight,
            minimumRouteHalfWidthDistance = minimumRouteHalfWidthDistance,
        )
        if (selectedCount == 0) {
            routeNanos = System.nanoTime() - stageStartedAtNanos
            return profiled(emptyEstimate())
        }
        Arrays.fill(visited, false)
        for (position in 0 until selectedCount) visited[indexBuffer[position]] = true
        selectedCount = retainLongestContinuousSegment(
            selectedCount = selectedCount,
            paddedWidth = paddedWidth,
            factor = factor,
            frameWidth = mask.width,
            frameHeight = mask.height,
        )

        val points = ArrayList<CenterlinePoint>(selectedCount)
        for (position in 0 until selectedCount) {
            val index = indexBuffer[position]
            val px = index % paddedWidth - 1
            val py = index / paddedWidth - 1
            points.add(
                CenterlinePoint(
                    x = fullResolutionCoordinate(px, factor, mask.width),
                    y = fullResolutionCoordinate(py, factor, mask.height),
                    widthPixels = widthPixels(distances[index], factor),
                ),
            )
        }
        routeNanos = System.nanoTime() - stageStartedAtNanos

        stageStartedAtNanos = System.nanoTime()
        val support = computeSupport(
            indexBuffer,
            selectedCount,
            foreground,
            paddedWidth,
            paddedHeight,
        )
        val widthConsistency = computeWidthConsistency(points)
        val continuity = computeContinuity(points, selectedCount, skeletonCount)
        val fitResidual = computeFitResidual(points)
        val aggregate = weightedGeometricMean(
            support = support,
            widthConsistency = widthConsistency,
            continuity = continuity,
            fitResidual = fitResidual,
        )
        val components = CenterlineConfidence(
            support = support,
            widthConsistency = widthConsistency,
            continuity = continuity,
            fitResidual = fitResidual,
            aggregate = aggregate,
        )
        qualityNanos = System.nanoTime() - stageStartedAtNanos

        stageStartedAtNanos = System.nanoTime()
        val topology = computeTopology(
            points = points,
            selectedCount = selectedCount,
            paddedWidth = paddedWidth,
            factor = factor,
            frameWidth = mask.width,
            frameHeight = mask.height,
        )
        topologyNanos = System.nanoTime() - stageStartedAtNanos
        return profiled(
            CenterlineEstimate(
                points = points,
                confidence = aggregate,
                components = components,
                topology = topology,
            ),
        )
    }

    /**
     * Enforces [CenterlineExtractorConfig.maxChainGapPixels] after mapping the
     * working skeleton back to full-resolution coordinates. Downsampling can
     * otherwise turn adjacent working pixels into an output chain whose every
     * step exceeds the caller's configured gap. The longest valid segment wins;
     * ties retain the earlier segment nearest the route's bottom endpoint.
     */
    private fun retainLongestContinuousSegment(
        selectedCount: Int,
        paddedWidth: Int,
        factor: Int,
        frameWidth: Int,
        frameHeight: Int,
    ): Int {
        if (selectedCount < 2) return selectedCount
        var bestStart = 0
        var bestCount = 1
        var currentStart = 0
        for (position in 1 until selectedCount) {
            val previous = indexBuffer[position - 1]
            val current = indexBuffer[position]
            val previousX = fullResolutionCoordinate(
                previous % paddedWidth - 1,
                factor,
                frameWidth,
            )
            val previousY = fullResolutionCoordinate(
                previous / paddedWidth - 1,
                factor,
                frameHeight,
            )
            val currentX = fullResolutionCoordinate(
                current % paddedWidth - 1,
                factor,
                frameWidth,
            )
            val currentY = fullResolutionCoordinate(
                current / paddedWidth - 1,
                factor,
                frameHeight,
            )
            if (hypot(currentX - previousX, currentY - previousY) > config.maxChainGapPixels) {
                val currentCount = position - currentStart
                if (currentCount > bestCount) {
                    bestStart = currentStart
                    bestCount = currentCount
                }
                currentStart = position
            }
        }
        val finalCount = selectedCount - currentStart
        if (finalCount > bestCount) {
            bestStart = currentStart
            bestCount = finalCount
        }
        if (bestStart > 0) {
            System.arraycopy(indexBuffer, bestStart, indexBuffer, 0, bestCount)
        }
        return bestCount
    }

    private fun fullResolutionCoordinate(
        workingCoordinate: Int,
        factor: Int,
        frameExtent: Int,
    ): Double = ((workingCoordinate + 0.5) * factor - 0.5)
        .coerceIn(0.0, frameExtent - 1.0)

    private fun ensureWorkspace(size: Int) {
        if (size == workspaceSize) return
        workspaceSize = size
        foreground = BooleanArray(size)
        visited = BooleanArray(size)
        skeleton = BooleanArray(size)
        indexBuffer = IntArray(size)
        distances = IntArray(size)
        removalBuffer = IntArray(size)
        thinningCandidates = BooleanArray(size)
        routeCost = IntArray(size)
        routeLength = IntArray(size)
        routeStrength = IntArray(size)
        predecessor = IntArray(size)
        heapNodes = IntArray(size)
        heapCosts = IntArray(size)
        heapPositions = IntArray(size)
    }

    /**
     * Closes bounded occlusion holes whose narrow dimension fits the configured
     * chain gap. The unbounded image background is never filled.
     */
    private fun fillEnclosedGaps(
        foreground: BooleanArray,
        width: Int,
        height: Int,
        downsampleFactor: Int,
    ) {
        Arrays.fill(visited, false)
        val queue = indexBuffer
        for (seedY in 0 until height) {
            for (seedX in 0 until width) {
                val seed = seedY * width + seedX
                if (foreground[seed] || visited[seed]) continue

                var head = 0
                var tail = 1
                queue[0] = seed
                visited[seed] = true
                var minX = seedX
                var maxX = seedX
                var minY = seedY
                var maxY = seedY
                var touchesBorder = false

                while (head < tail) {
                    val current = queue[head++]
                    val x = current % width
                    val y = current / width
                    minX = min(minX, x)
                    maxX = max(maxX, x)
                    minY = min(minY, y)
                    maxY = max(maxY, y)
                    if (x == 0 || x == width - 1 || y == 0 || y == height - 1) {
                        touchesBorder = true
                    }

                    if (x > 0) tail = enqueueBackground(current - 1, foreground, visited, queue, tail)
                    if (x + 1 < width) {
                        tail = enqueueBackground(current + 1, foreground, visited, queue, tail)
                    }
                    if (y > 0) tail = enqueueBackground(current - width, foreground, visited, queue, tail)
                    if (y + 1 < height) {
                        tail = enqueueBackground(current + width, foreground, visited, queue, tail)
                    }
                }

                val narrowSpanPixels =
                    min(maxX - minX + 1, maxY - minY + 1) * downsampleFactor
                if (!touchesBorder && narrowSpanPixels <= config.maxChainGapPixels) {
                    for (index in 0 until tail) foreground[queue[index]] = true
                }
            }
        }
    }

    private fun enqueueBackground(
        index: Int,
        foreground: BooleanArray,
        visited: BooleanArray,
        queue: IntArray,
        tail: Int,
    ): Int {
        if (foreground[index] || visited[index]) return tail
        visited[index] = true
        queue[tail] = index
        return tail + 1
    }

    private fun chamferDistance(
        foreground: BooleanArray,
        width: Int,
        height: Int,
    ) {
        val distance = distances
        for (index in 0 until workspaceSize) {
            distance[index] = if (foreground[index]) INF_DISTANCE else 0
        }
        for (y in 1 until height - 1) {
            for (x in 1 until width - 1) {
                val i = y * width + x
                if (!foreground[i]) continue
                var nearest = distance[i]
                nearest = min(nearest, distance[i - 1] + ORTHOGONAL_COST)
                nearest = min(nearest, distance[i - width] + ORTHOGONAL_COST)
                nearest = min(nearest, distance[i - width - 1] + DIAGONAL_COST)
                distance[i] = min(nearest, distance[i - width + 1] + DIAGONAL_COST)
            }
        }
        for (y in height - 2 downTo 1) {
            for (x in width - 2 downTo 1) {
                val i = y * width + x
                if (!foreground[i]) continue
                var nearest = distance[i]
                nearest = min(nearest, distance[i + 1] + ORTHOGONAL_COST)
                nearest = min(nearest, distance[i + width] + ORTHOGONAL_COST)
                nearest = min(nearest, distance[i + width - 1] + DIAGONAL_COST)
                distance[i] = min(nearest, distance[i + width + 1] + DIAGONAL_COST)
            }
        }
    }

    private fun collectBoundaryIndices(
        pixels: BooleanArray,
        width: Int,
        height: Int,
        indices: IntArray,
    ): Int {
        Arrays.fill(thinningCandidates, false)
        var count = 0
        for (y in 1 until height - 1) {
            for (x in 1 until width - 1) {
                val index = y * width + x
                if (!pixels[index]) continue
                if (
                    !pixels[index - width] ||
                    !pixels[index - width + 1] ||
                    !pixels[index + 1] ||
                    !pixels[index + width + 1] ||
                    !pixels[index + width] ||
                    !pixels[index + width - 1] ||
                    !pixels[index - 1] ||
                    !pixels[index - width - 1]
                ) {
                    indices[count++] = index
                    thinningCandidates[index] = true
                }
            }
        }
        return count
    }

    private fun thinZhangSuen(
        pixels: BooleanArray,
        width: Int,
        height: Int,
        activeIndices: IntArray,
    ) {
        var activeCount = collectBoundaryIndices(pixels, width, height, activeIndices)
        var changed: Boolean
        var iteration = 0
        val iterationLimit = max(width, height)
        do {
            val firstRemoveCount = thinningPass(
                pixels = pixels,
                width = width,
                firstPass = true,
                activeIndices = activeIndices,
                activeCount = activeCount,
                remove = removalBuffer,
            )
            activeCount = applyThinningRemovals(
                pixels,
                width,
                activeIndices,
                activeCount,
                removalBuffer,
                firstRemoveCount,
            )
            val secondRemoveCount = thinningPass(
                pixels = pixels,
                width = width,
                firstPass = false,
                activeIndices = activeIndices,
                activeCount = activeCount,
                remove = removalBuffer,
            )
            activeCount = applyThinningRemovals(
                pixels,
                width,
                activeIndices,
                activeCount,
                removalBuffer,
                secondRemoveCount,
            )
            activeCount = compactThinningCandidates(pixels, activeIndices, activeCount)
            changed = firstRemoveCount > 0 || secondRemoveCount > 0
            iteration++
        } while (changed && iteration < iterationLimit)
    }

    private fun thinningPass(
        pixels: BooleanArray,
        width: Int,
        firstPass: Boolean,
        activeIndices: IntArray,
        activeCount: Int,
        remove: IntArray,
    ): Int {
        var removeCount = 0
        for (position in 0 until activeCount) {
            val i = activeIndices[position]
            if (!pixels[i]) continue
            val p2 = pixels[i - width]
            val p3 = pixels[i - width + 1]
            val p4 = pixels[i + 1]
            val p5 = pixels[i + width + 1]
            val p6 = pixels[i + width]
            val p7 = pixels[i + width - 1]
            val p8 = pixels[i - 1]
            val p9 = pixels[i - width - 1]
            val neighborCount =
                (if (p2) 1 else 0) +
                    (if (p3) 1 else 0) +
                    (if (p4) 1 else 0) +
                    (if (p5) 1 else 0) +
                    (if (p6) 1 else 0) +
                    (if (p7) 1 else 0) +
                    (if (p8) 1 else 0) +
                    (if (p9) 1 else 0)
            if (neighborCount !in 2..6) continue
            val transitions =
                (if (!p2 && p3) 1 else 0) +
                    (if (!p3 && p4) 1 else 0) +
                    (if (!p4 && p5) 1 else 0) +
                    (if (!p5 && p6) 1 else 0) +
                    (if (!p6 && p7) 1 else 0) +
                    (if (!p7 && p8) 1 else 0) +
                    (if (!p8 && p9) 1 else 0) +
                    (if (!p9 && p2) 1 else 0)
            if (transitions != 1) continue
            val keepForTriples =
                if (firstPass) {
                    !(p2 && p4 && p6) && !(p4 && p6 && p8)
                } else {
                    !(p2 && p4 && p8) && !(p2 && p6 && p8)
                }
            if (keepForTriples) remove[removeCount++] = i
        }
        return removeCount
    }

    private fun applyThinningRemovals(
        pixels: BooleanArray,
        width: Int,
        activeIndices: IntArray,
        activeCount: Int,
        remove: IntArray,
        removeCount: Int,
    ): Int {
        for (position in 0 until removeCount) pixels[remove[position]] = false
        var nextCount = activeCount
        for (position in 0 until removeCount) {
            val removed = remove[position]
            for (dy in -1..1) {
                for (dx in -1..1) {
                    if (dx == 0 && dy == 0) continue
                    val neighbor = removed + dy * width + dx
                    if (pixels[neighbor] && !thinningCandidates[neighbor]) {
                        thinningCandidates[neighbor] = true
                        activeIndices[nextCount++] = neighbor
                    }
                }
            }
        }
        return nextCount
    }

    private fun compactThinningCandidates(
        pixels: BooleanArray,
        activeIndices: IntArray,
        activeCount: Int,
    ): Int {
        var retainedCount = 0
        for (position in 0 until activeCount) {
            val index = activeIndices[position]
            if (pixels[index]) {
                activeIndices[retainedCount++] = index
            } else {
                thinningCandidates[index] = false
            }
        }
        return retainedCount
    }

    /**
     * Returns the strongest route from the endpoint nearest the image bottom.
     * Strength is geometric length weighted by width only until the projected
     * minimum tape width. Equal-width and wider paths therefore retain the old
     * longest-path ordering, while a narrow board seam is penalized instead of
     * winning merely because it reaches farther across the frame. Dijkstra also
     * chooses the wider continuation when paths rejoin. Starting at an endpoint
     * preserves both sides of a U-shaped or bowed tape; a closed loop alone
     * falls back to its lowest skeleton point, a fact recorded in
     * [routeClosedLoop].
     */
    private fun longestPathFromBottom(
        skeleton: BooleanArray,
        widthDistances: IntArray,
        width: Int,
        height: Int,
        minimumRouteHalfWidthDistance: Int,
    ): Int {
        var lowestPoint = -1
        var lowestEndpoint = -1
        for (y in 1 until height - 1) {
            for (x in 1 until width - 1) {
                val index = y * width + x
                if (!skeleton[index]) continue
                if (lowestPoint < 0 || y > lowestPoint / width) lowestPoint = index
                var neighborCount = 0
                for (dy in -1..1) {
                    for (dx in -1..1) {
                        if ((dx != 0 || dy != 0) && skeleton[index + dy * width + dx]) {
                            neighborCount++
                        }
                    }
                }
                if (neighborCount <= 1 && (lowestEndpoint < 0 || y > lowestEndpoint / width)) {
                    lowestEndpoint = index
                }
            }
        }
        routeClosedLoop = lowestPoint >= 0 && lowestEndpoint < 0
        val start = if (lowestEndpoint >= 0) lowestEndpoint else lowestPoint
        if (start < 0) return 0

        Arrays.fill(routeCost, -1)
        Arrays.fill(predecessor, -1)
        Arrays.fill(heapPositions, HEAP_UNSEEN)
        var heapSize = 1
        var farthest = start
        routeCost[start] = 0
        routeLength[start] = 0
        routeStrength[start] = 0
        heapNodes[0] = start
        heapCosts[0] = 0
        heapPositions[start] = 0

        while (heapSize > 0) {
            val current = heapNodes[0]
            heapSize--
            if (heapSize > 0) {
                val movedNode = heapNodes[heapSize]
                heapNodes[0] = movedNode
                heapCosts[0] = heapCosts[heapSize]
                heapPositions[movedNode] = 0
                var position = 0
                while (true) {
                    val left = position * 2 + 1
                    if (left >= heapSize) break
                    val right = left + 1
                    val smaller =
                        if (right < heapSize && heapCosts[right] < heapCosts[left]) right else left
                    if (heapCosts[position] <= heapCosts[smaller]) break
                    val parentNode = heapNodes[position]
                    val parentCost = heapCosts[position]
                    heapNodes[position] = heapNodes[smaller]
                    heapCosts[position] = heapCosts[smaller]
                    heapPositions[heapNodes[position]] = position
                    heapNodes[smaller] = parentNode
                    heapCosts[smaller] = parentCost
                    heapPositions[parentNode] = smaller
                    position = smaller
                }
            }
            heapPositions[current] = HEAP_FINALIZED
            if (hasStrongerRoute(current, farthest)) farthest = current

            val currentX = current % width
            val currentY = current / width
            for (dy in -1..1) {
                for (dx in -1..1) {
                    if (dx == 0 && dy == 0) continue
                    val next = (currentY + dy) * width + currentX + dx
                    if (!skeleton[next] || heapPositions[next] == HEAP_FINALIZED) continue
                    val geometricCost =
                        if (dx == 0 || dy == 0) ORTHOGONAL_COST else DIAGONAL_COST
                    val localHalfWidth = max(1, widthDistances[next])
                    val edgeCost =
                        geometricCost * (WIDTH_ROUTE_BASE + WIDTH_ROUTE_BONUS / localHalfWidth)
                    val candidateCost = routeCost[current] + edgeCost
                    if (routeCost[next] >= 0 && candidateCost >= routeCost[next]) continue
                    routeCost[next] = candidateCost
                    routeLength[next] = routeLength[current] + geometricCost
                    routeStrength[next] =
                        routeStrength[current] +
                            geometricCost * min(localHalfWidth, minimumRouteHalfWidthDistance)
                    predecessor[next] = current

                    var position = heapPositions[next]
                    if (position == HEAP_UNSEEN) {
                        position = heapSize
                        heapNodes[position] = next
                        heapCosts[position] = candidateCost
                        heapPositions[next] = position
                        heapSize++
                    } else {
                        heapCosts[position] = candidateCost
                    }
                    while (position > 0) {
                        val parentPosition = (position - 1) / 2
                        if (heapCosts[parentPosition] <= heapCosts[position]) break
                        val parentNode = heapNodes[parentPosition]
                        val parentCost = heapCosts[parentPosition]
                        heapNodes[parentPosition] = heapNodes[position]
                        heapCosts[parentPosition] = heapCosts[position]
                        heapPositions[heapNodes[parentPosition]] = parentPosition
                        heapNodes[position] = parentNode
                        heapCosts[position] = parentCost
                        heapPositions[parentNode] = position
                        position = parentPosition
                    }
                }
            }
        }

        var pathCount = 0
        var current = farthest
        while (current >= 0) {
            indexBuffer[pathCount++] = current
            current = predecessor[current]
        }
        var left = 0
        var right = pathCount - 1
        while (left < right) {
            val swap = indexBuffer[left]
            indexBuffer[left] = indexBuffer[right]
            indexBuffer[right] = swap
            left++
            right--
        }
        return pathCount
    }

    private fun hasStrongerRoute(candidate: Int, selected: Int): Boolean {
        val candidateStrength = routeStrength[candidate]
        val selectedStrength = routeStrength[selected]
        return candidateStrength > selectedStrength ||
            (
                candidateStrength == selectedStrength &&
                    routeLength[candidate] > routeLength[selected]
                )
    }

    /**
     * Observation-only topology facts for the reported chain, in
     * full-resolution pixels like [CenterlinePoint]. A closed loop reports a
     * [CenterlineTerminus.NONE] terminus because a cycle has no distal end to
     * classify, and its off-route skeleton remainder is the tape rejoining
     * itself, never a divergence, so its branch count is zero by definition.
     * Cyclic tape graphs (a loop with a crossing bar) are outside the
     * single-path contract and are not detected here.
     */
    private fun computeTopology(
        points: List<CenterlinePoint>,
        selectedCount: Int,
        paddedWidth: Int,
        factor: Int,
        frameWidth: Int,
        frameHeight: Int,
    ): CenterlineTopology {
        if (points.isEmpty()) return CenterlineTopology()
        val distal = points.last()
        val distalBorderDistancePixels = min(
            min(distal.x, frameWidth - 1.0 - distal.x),
            min(distal.y, frameHeight - 1.0 - distal.y),
        )
        val borderMarginPixels = max(
            config.minDistalBorderMarginPixels,
            distal.widthPixels * config.distalBorderMarginWidthFactor,
        )
        val distalTerminus = when {
            routeClosedLoop -> CenterlineTerminus.NONE
            distalBorderDistancePixels <= borderMarginPixels -> CenterlineTerminus.AT_FRAME_BORDER
            else -> CenterlineTerminus.INSIDE_FRAME
        }
        return CenterlineTopology(
            distalTerminus = distalTerminus,
            distalBorderDistancePixels = distalBorderDistancePixels,
            branchCount = if (routeClosedLoop) {
                0
            } else {
                countCredibleBranches(selectedCount, paddedWidth, factor)
            },
            closedLoop = routeClosedLoop,
        )
    }

    /**
     * Counts credible skeleton divergences from the reported chain. A branch
     * is credible only when its off-route arc, measured from the chain pixel
     * it attaches to, exceeds the local-width-scaled threshold, so thinning
     * spurs at bumps and staircase corners are rejected while a genuine tape
     * arm always qualifies. Each walk consumes what it visits, so a branch
     * reachable from several adjacent chain pixels (a thick junction) counts
     * exactly once.
     */
    private fun countCredibleBranches(
        selectedCount: Int,
        paddedWidth: Int,
        factor: Int,
    ): Int {
        // Dijkstra scratch is free once the route is extracted; reuse it so
        // branch detection allocates nothing per frame. Zero marks unvisited.
        val branchVisited = predecessor
        val walkCosts = routeCost
        val queue = heapNodes
        Arrays.fill(branchVisited, 0)
        var branchCount = 0
        for (position in 0 until selectedCount) {
            val junction = indexBuffer[position]
            val thresholdArcPixels = max(
                config.minBranchArcLengthPixels,
                widthPixels(distances[junction], factor) * config.branchArcLengthWidthFactor,
            )
            // Walk costs accumulate in working-grid chamfer units; convert the
            // full-resolution threshold into the same units instead of scaling
            // every visited pixel back up.
            val thresholdCost = thresholdArcPixels * ORTHOGONAL_COST / factor
            if (branchWalkReaches(junction, thresholdCost, branchVisited, walkCosts, queue, paddedWidth)) {
                branchCount++
            }
        }
        return branchCount
    }

    /**
     * Floods the off-route skeleton 8-connected to [junction] and reports
     * whether any pixel lies at least [thresholdCost] of chamfer arc away.
     * The flood keeps consuming past the threshold so a later junction pixel
     * cannot recount the same branch.
     */
    private fun branchWalkReaches(
        junction: Int,
        thresholdCost: Double,
        branchVisited: IntArray,
        walkCosts: IntArray,
        queue: IntArray,
        paddedWidth: Int,
    ): Boolean {
        var tail = enqueueBranchNeighbors(
            center = junction,
            baseCost = 0,
            tail = 0,
            branchVisited = branchVisited,
            walkCosts = walkCosts,
            queue = queue,
            paddedWidth = paddedWidth,
        )
        if (tail == 0) return false
        var reached = false
        var head = 0
        while (head < tail) {
            val current = queue[head++]
            if (walkCosts[current] >= thresholdCost) reached = true
            tail = enqueueBranchNeighbors(
                center = current,
                baseCost = walkCosts[current],
                tail = tail,
                branchVisited = branchVisited,
                walkCosts = walkCosts,
                queue = queue,
                paddedWidth = paddedWidth,
            )
        }
        return reached
    }

    private fun enqueueBranchNeighbors(
        center: Int,
        baseCost: Int,
        tail: Int,
        branchVisited: IntArray,
        walkCosts: IntArray,
        queue: IntArray,
        paddedWidth: Int,
    ): Int {
        var nextTail = tail
        for (dy in -1..1) {
            for (dx in -1..1) {
                if (dx == 0 && dy == 0) continue
                val neighbor = center + dy * paddedWidth + dx
                // visited holds the route marks laid down in extract(); the
                // walk must neither seed from nor cross the route.
                if (!skeleton[neighbor] || visited[neighbor] || branchVisited[neighbor] != 0) continue
                branchVisited[neighbor] = 1
                walkCosts[neighbor] = baseCost +
                    if (dx == 0 || dy == 0) ORTHOGONAL_COST else DIAGONAL_COST
                queue[nextTail++] = neighbor
            }
        }
        return nextTail
    }

    private fun computeSupport(
        chain: IntArray,
        chainSize: Int,
        foreground: BooleanArray,
        width: Int,
        height: Int,
    ): Double {
        if (chainSize < 2) return 0.0
        var meanX = 0.0
        var meanY = 0.0
        var count = 0
        for (y in 1 until height - 1) {
            for (x in 1 until width - 1) {
                if (!foreground[y * width + x]) continue
                meanX += x
                meanY += y
                count++
            }
        }
        if (count < 2) return 0.0
        meanX /= count
        meanY /= count
        var covarianceXX = 0.0
        var covarianceXY = 0.0
        var covarianceYY = 0.0
        for (y in 1 until height - 1) {
            for (x in 1 until width - 1) {
                if (!foreground[y * width + x]) continue
                val dx = x - meanX
                val dy = y - meanY
                covarianceXX += dx * dx
                covarianceXY += dx * dy
                covarianceYY += dy * dy
            }
        }
        val angle = 0.5 * atan2(2.0 * covarianceXY, covarianceXX - covarianceYY)
        val axisX = cos(angle)
        val axisY = sin(angle)
        var minProjection = Double.POSITIVE_INFINITY
        var maxProjection = Double.NEGATIVE_INFINITY
        for (y in 1 until height - 1) {
            for (x in 1 until width - 1) {
                if (!foreground[y * width + x]) continue
                val projection = x * axisX + y * axisY
                minProjection = min(minProjection, projection)
                maxProjection = max(maxProjection, projection)
            }
        }
        val principalExtent = maxProjection - minProjection
        if (principalExtent <= 0.0) return 0.0

        var chainMinProjection = Double.POSITIVE_INFINITY
        var chainMaxProjection = Double.NEGATIVE_INFINITY
        for (position in 0 until chainSize) {
            val index = chain[position]
            val projection = (index % width) * axisX + (index / width) * axisY
            chainMinProjection = min(chainMinProjection, projection)
            chainMaxProjection = max(chainMaxProjection, projection)
        }
        val representedExtent = chainMaxProjection - chainMinProjection
        return (representedExtent / principalExtent).coerceIn(0.0, 1.0)
    }

    private fun computeWidthConsistency(points: List<CenterlinePoint>): Double {
        if (points.isEmpty()) return 0.0
        val mean = points.sumOf { it.widthPixels } / points.size
        if (mean <= 0.0) return 0.0
        val variance = points.sumOf {
            val delta = it.widthPixels - mean
            delta * delta
        } / points.size
        val coefficientOfVariation = sqrt(variance) / mean
        return (1.0 - coefficientOfVariation).coerceIn(0.0, 1.0)
    }

    private fun computeContinuity(
        points: List<CenterlinePoint>,
        selectedSkeletonCount: Int,
        totalSkeletonCount: Int,
    ): Double {
        if (points.size < 2 || totalSkeletonCount == 0) return 0.0
        var continuousPairs = 0
        for (i in 1 until points.size) {
            val distance = hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y)
            if (distance <= config.maxChainGapPixels) continuousPairs++
        }
        val adjacentRatio = continuousPairs.toDouble() / (points.size - 1)
        val selectedCoverage = selectedSkeletonCount.toDouble() / totalSkeletonCount
        return (adjacentRatio * selectedCoverage).coerceIn(0.0, 1.0)
    }

    private fun computeFitResidual(points: List<CenterlinePoint>): Double {
        if (points.size < 3) return if (points.size >= 2) 1.0 else 0.0
        val chordOffset = min(FIT_CHORD_POINT_OFFSET, points.lastIndex / 2)
        if (chordOffset == 0) return 0.0
        var normalizedResidual = 0.0
        var samples = 0
        for (i in chordOffset..points.lastIndex - chordOffset) {
            val ax = points[i].x - points[i - chordOffset].x
            val ay = points[i].y - points[i - chordOffset].y
            val bx = points[i + chordOffset].x - points[i].x
            val by = points[i + chordOffset].y - points[i].y
            val aLength = hypot(ax, ay)
            val bLength = hypot(bx, by)
            if (aLength == 0.0 || bLength == 0.0) continue
            val cosine = ((ax * bx + ay * by) / (aLength * bLength)).coerceIn(-1.0, 1.0)
            normalizedResidual += acos(cosine) / PI
            samples++
        }
        if (samples == 0) return 0.0
        return (1.0 - normalizedResidual / samples).coerceIn(0.0, 1.0)
    }

    /** Weights are corpus-calibrated and deliberately independent of image axes. */
    private fun weightedGeometricMean(
        support: Double,
        widthConsistency: Double,
        continuity: Double,
        fitResidual: Double,
    ): Double {
        if (support <= 0.0 || widthConsistency <= 0.0 || continuity <= 0.0 || fitResidual <= 0.0) {
            return 0.0
        }
        val weightedLog =
            SUPPORT_WEIGHT * ln(support) +
                WIDTH_CONSISTENCY_WEIGHT * ln(widthConsistency) +
                CONTINUITY_WEIGHT * ln(continuity) +
                FIT_RESIDUAL_WEIGHT * ln(fitResidual)
        return exp(weightedLog).coerceIn(0.0, 1.0)
    }

    private fun widthPixels(chamferDistance: Int, factor: Int): Double =
        2.0 * chamferDistance / ORTHOGONAL_COST * factor

    private fun emptyEstimate(): CenterlineEstimate = CenterlineEstimate(
        points = emptyList(),
        confidence = 0.0,
        components = CenterlineConfidence(
            support = 0.0,
            widthConsistency = 0.0,
            continuity = 0.0,
            fitResidual = 0.0,
            aggregate = 0.0,
        ),
    )

    private companion object {
        const val TARGET_WORKING_HEIGHT = 270
        const val ORTHOGONAL_COST = 3
        const val DIAGONAL_COST = 4
        const val INF_DISTANCE = 1_000_000
        const val HEAP_UNSEEN = -1
        const val HEAP_FINALIZED = -2
        const val WIDTH_ROUTE_BASE = 256
        const val WIDTH_ROUTE_BONUS = 4_096

        /** Chord span that suppresses single-pixel 8-connected staircase turns. */
        const val FIT_CHORD_POINT_OFFSET = 4
        const val SUPPORT_WEIGHT = 0.35
        const val WIDTH_CONSISTENCY_WEIGHT = 0.25
        const val CONTINUITY_WEIGHT = 0.25
        const val FIT_RESIDUAL_WEIGHT = 0.15
        const val MINIMUM_WIDTH_TOLERANCE = 0.50
        const val MAXIMUM_WIDTH_HEADROOM = 2.0
    }
}
