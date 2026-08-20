package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.sin

class CenterlineExtractorTest {
    private val extractor = CenterlineExtractor()

    @Test
    fun `vertical horizontal and diagonal masks yield long accurate chains`() {
        val paths = listOf(
            listOf(Point(100.0, 185.0), Point(100.0, 15.0)),
            listOf(Point(15.0, 100.0), Point(185.0, 100.0)),
            listOf(Point(20.0, 180.0), Point(180.0, 20.0)),
            listOf(Point(20.0, 20.0), Point(180.0, 180.0)),
        )

        paths.forEachIndexed { index, path ->
            val estimate = extractor.extract(maskAlongPath(200, 200, path, halfWidth = { 5.0 }))
            val truthLength = polylineLength(path)
            assertTrue("path $index has ${estimate.points.size} points", estimate.points.size >= truthLength * 0.60)
            assertTrue(
                "path $index left its truth corridor",
                estimate.points.all { point -> distanceToPath(Point(point.x, point.y), path) <= 3.0 },
            )
            assertTrue("path $index confidence ${estimate.confidence}", estimate.confidence >= 0.75)
            assertEquals(estimate.components.aggregate, estimate.confidence, 0.0)
        }
    }

    @Test
    fun `arc and closed loop retain most of their truth length`() {
        val arc = (0..48).map { step ->
            val angle = PI * (0.15 + 0.70 * step / 48.0)
            Point(100.0 + 70.0 * cos(angle), 105.0 + 70.0 * sin(angle))
        }
        val loop = (0..95).map { step ->
            val angle = 2.0 * PI * step / 96.0
            Point(100.0 + 62.0 * cos(angle), 100.0 + 62.0 * sin(angle))
        }

        val arcEstimate = extractor.extract(maskAlongPath(200, 200, arc, halfWidth = { 5.0 }))
        val loopEstimate = extractor.extract(maskAlongPath(200, 200, loop, halfWidth = { 5.0 }))

        assertTrue(pathLength(arcEstimate.points) >= polylineLength(arc) * 0.60)
        assertTrue(pathLength(loopEstimate.points) >= polylineLength(loop) * 0.50)
        assertTrue(loopEstimate.points.first().y >= loopEstimate.points.last().y)
    }

    @Test
    fun `symmetric U shaped tape starts at an endpoint and retains both arms`() {
        val uShape = listOf(
            Point(30.0, 35.0),
            Point(30.0, 145.0),
            Point(65.0, 175.0),
            Point(100.0, 185.0),
            Point(135.0, 175.0),
            Point(170.0, 145.0),
            Point(170.0, 35.0),
        )

        val estimate = extractor.extract(maskAlongPath(200, 200, uShape, halfWidth = { 5.0 }))

        assertTrue(
            "extracted ${pathLength(estimate.points)} of ${polylineLength(uShape)}",
            pathLength(estimate.points) >= polylineLength(uShape) * 0.75,
        )
        assertTrue(estimate.points.first().y < 60.0)
        assertTrue(estimate.points.last().y < 60.0)
    }

    @Test
    fun `long curved tape wins over a shorter locally straight branch`() {
        val tapePath = listOf(
            Point(120.0, 210.0),
            Point(150.0, 160.0),
            Point(120.0, 130.0),
            Point(80.0, 90.0),
            Point(30.0, 35.0),
        )
        val shortBranch = listOf(
            Point(150.0, 160.0),
            Point(180.0, 110.0),
        )
        val tapeMask = maskAlongPath(240, 220, tapePath, halfWidth = { 6.0 })
        val branchMask = maskAlongPath(240, 220, shortBranch, halfWidth = { 6.0 })
        val combined = BooleanArray(240 * 220) { index ->
            tapeMask.tape[index] || branchMask.tape[index]
        }

        val estimate = extractor.extract(SegmentationMask(240, 220, combined))
        val corridorFraction = estimate.points.count { point ->
            distanceToPath(Point(point.x, point.y), tapePath) <= 4.0
        }.toDouble() / estimate.points.size

        assertTrue("path ended on the short branch: ${estimate.points.last()}", estimate.points.last().x < 60.0)
        assertTrue("only $corridorFraction of the path followed the tape", corridorFraction >= 0.90)
        assertTrue(pathLength(estimate.points) >= polylineLength(tapePath) * 0.75)
    }

    @Test
    fun `branch selection compares physical path length instead of pixel step count`() {
        val width = 100
        val tape = BooleanArray(width * 100)
        for (y in 80..90) tape[y * width + 50] = true
        for (x in 40..50) tape[80 * width + x] = true
        for (step in 0..8) tape[(80 - step) * width + 50 + step] = true
        val estimate = extractor.extract(
            SegmentationMask(width, 100, tape),
            TapeWidthRangePixels(minimum = 1.0, maximum = 3.0),
        )

        assertTrue("weighted geodesic should end on the longer diagonal branch", estimate.points.last().x > 55.0)
    }

    @Test
    fun `rotating the same tape ninety degrees preserves aggregate confidence`() {
        val vertical = listOf(Point(90.0, 185.0), Point(95.0, 15.0))
        val horizontal = vertical.map { point -> Point(200.0 - point.y, point.x) }

        val verticalConfidence = extractor
            .extract(maskAlongPath(200, 200, vertical, halfWidth = { 6.0 }))
            .confidence
        val horizontalConfidence = extractor
            .extract(maskAlongPath(200, 200, horizontal, halfWidth = { 6.0 }))
            .confidence

        assertTrue(
            "vertical=$verticalConfidence horizontal=$horizontalConfidence",
            abs(verticalConfidence - horizontalConfidence) <= 0.15,
        )
    }

    @Test
    fun `working distance sweep preserves support as apparent tape width grows`() {
        val curvedTape = listOf(
            Point(24.0, 166.0),
            Point(104.0, 160.0),
            Point(184.0, 177.0),
            Point(252.0, 221.0),
            Point(300.0, 282.0),
            Point(326.0, 352.0),
        )

        listOf(8.0, 16.0, 32.0, 64.0, 96.0, 160.0).forEach { apparentWidthPixels ->
            val estimate = extractor.extract(
                maskAlongPath(
                    width = 640,
                    height = 360,
                    path = curvedTape,
                    halfWidth = { apparentWidthPixels / 2.0 },
                ),
            )
            println(
                "ISSUE196_WIDTH_SWEEP apparent_width_px=$apparentWidthPixels " +
                    "points=${estimate.points.size} support=${estimate.components.support} " +
                    "confidence=${estimate.confidence}",
            )
            assertTrue(
                "width=$apparentWidthPixels support=${estimate.components.support}",
                estimate.components.support >= 0.50,
            )
            assertTrue(
                "width=$apparentWidthPixels confidence=${estimate.confidence}",
                estimate.confidence >= 0.60,
            )
        }
    }

    @Test
    fun `pixel staircases do not penalize diagonal straight tape`() {
        val axisAligned = extractor.extract(
            maskAlongPath(
                200,
                200,
                listOf(Point(100.0, 185.0), Point(100.0, 15.0)),
                halfWidth = { 6.0 },
            ),
        )
        val diagonal = extractor.extract(
            maskAlongPath(
                200,
                200,
                listOf(Point(25.0, 175.0), Point(175.0, 25.0)),
                halfWidth = { 6.0 },
            ),
        )

        assertTrue(axisAligned.components.fitResidual >= 0.95)
        assertTrue(diagonal.components.fitResidual >= 0.95)
        assertTrue(
            "axis=${axisAligned.components.fitResidual} diagonal=${diagonal.components.fitResidual}",
            abs(axisAligned.components.fitResidual - diagonal.components.fitResidual) <= 0.03,
        )
    }

    // The upstream suite also asserts that a same-size extraction reuses its
    // primitive workspaces, measured with com.sun.management.ThreadMXBean.
    // Android's unit-test android.jar carries no java.lang.management, so that
    // assertion cannot compile here. Workspace reuse is instead a device
    // measurement: per-frame allocation and native heap growth are checked on
    // the aircraft stream, per the redesign document's on-device verification.

    @Test
    fun `full resolution chain honors a gap smaller than the downsample step`() {
        val width = 200
        val height = 1_080
        val tape = BooleanArray(width * height)
        for (y in 0 until height) {
            for (x in 80..119) tape[y * width + x] = true
        }
        val maxGapPixels = 2.0
        val strictExtractor = CenterlineExtractor(
            CenterlineExtractorConfig(
                maxChainGapPixels = maxGapPixels,
                minPointCount = 5,
            ),
        )

        val estimate = strictExtractor.extract(SegmentationMask(width, height, tape))

        assertTrue("the mask must still yield a geometric sample", estimate.points.isNotEmpty())
        assertTrue(
            "a chain violating the configured gap must not remain command-ready",
            estimate.points.size < strictExtractor.minPointCount,
        )
        assertTrue(
            estimate.points.zipWithNext().all { (first, second) ->
                hypot(second.x - first.x, second.y - first.y) <= maxGapPixels
            },
        )
    }

    @Test
    fun `varying width lowers width consistency`() {
        val path = listOf(Point(100.0, 185.0), Point(100.0, 15.0))
        val uniform = extractor.extract(maskAlongPath(200, 200, path, halfWidth = { 6.0 }))
        val varying = extractor.extract(
            maskAlongPath(200, 200, path) { yFraction -> 3.5 + 9.0 * yFraction },
        )

        assertTrue(
            "uniform=${uniform.components.widthConsistency} varying=${varying.components.widthConsistency}",
            varying.components.widthConsistency < uniform.components.widthConsistency - 0.10,
        )
    }

    @Test
    fun `fragmented mask lowers continuity`() {
        val tape = BooleanArray(200 * 200)
        fun paint(yRange: IntRange) {
            for (y in yRange) for (x in 94..106) tape[y * 200 + x] = true
        }
        paint(15..75)
        paint(125..185)

        val estimate = extractor.extract(SegmentationMask(200, 200, tape))

        assertTrue("continuity ${estimate.components.continuity}", estimate.components.continuity <= 0.60)
    }

    @Test
    fun `empty mask returns an empty chain and all zero components`() {
        val estimate = extractor.extract(SegmentationMask.empty(120, 80))

        assertTrue(estimate.points.isEmpty())
        assertEquals(0.0, estimate.confidence, 0.0)
        assertEquals(
            CenterlineConfidence(0.0, 0.0, 0.0, 0.0, 0.0),
            estimate.components,
        )
    }

    @Test
    fun `tape ending inside the frame reports an inside terminus`() {
        val estimate = extractor.extract(
            maskAlongPath(
                200,
                200,
                listOf(Point(100.0, 185.0), Point(100.0, 100.0)),
                halfWidth = { 5.0 },
            ),
        )

        assertEquals(CenterlineTerminus.INSIDE_FRAME, estimate.topology.distalTerminus)
        assertTrue(
            "distal border distance ${estimate.topology.distalBorderDistancePixels}",
            estimate.topology.distalBorderDistancePixels > 50.0,
        )
        assertFalse(estimate.topology.closedLoop)
        assertEquals(0, estimate.topology.branchCount)
    }

    @Test
    fun `the same tape truncated at the frame border reports a border terminus`() {
        val estimate = extractor.extract(
            maskAlongPath(
                200,
                200,
                listOf(Point(100.0, 185.0), Point(100.0, 0.0)),
                halfWidth = { 5.0 },
            ),
        )

        assertEquals(CenterlineTerminus.AT_FRAME_BORDER, estimate.topology.distalTerminus)
        assertTrue(
            "distal border distance ${estimate.topology.distalBorderDistancePixels}",
            estimate.topology.distalBorderDistancePixels < 15.0,
        )
        assertFalse(estimate.topology.closedLoop)
    }

    @Test
    fun `closed loop reports closed topology and no distal terminus`() {
        val loop = (0..95).map { step ->
            val angle = 2.0 * PI * step / 96.0
            Point(100.0 + 62.0 * cos(angle), 100.0 + 62.0 * sin(angle))
        }

        val estimate = extractor.extract(maskAlongPath(200, 200, loop, halfWidth = { 5.0 }))

        assertTrue(estimate.topology.closedLoop)
        assertEquals(CenterlineTerminus.NONE, estimate.topology.distalTerminus)
        assertEquals(0, estimate.topology.branchCount)
    }

    @Test
    fun `T junction arm diverging from the route is a credible branch`() {
        val trunk = maskAlongPath(
            200,
            200,
            listOf(Point(100.0, 185.0), Point(100.0, 20.0)),
            halfWidth = { 5.0 },
        )
        val arm = maskAlongPath(
            200,
            200,
            listOf(Point(100.0, 100.0), Point(160.0, 100.0)),
            halfWidth = { 5.0 },
        )
        val combined = BooleanArray(200 * 200) { index -> trunk.tape[index] || arm.tape[index] }

        val estimate = extractor.extract(SegmentationMask(200, 200, combined))

        assertTrue(
            "branchCount ${estimate.topology.branchCount}",
            estimate.topology.branchCount >= 1,
        )
        assertEquals(CenterlineTerminus.INSIDE_FRAME, estimate.topology.distalTerminus)
    }

    @Test
    fun `short thinning spur is not counted while a lax threshold sees it`() {
        val trunk = maskAlongPath(
            200,
            200,
            listOf(Point(100.0, 185.0), Point(100.0, 20.0)),
            halfWidth = { 5.0 },
        )
        val spur = maskAlongPath(
            200,
            200,
            listOf(Point(100.0, 120.0), Point(110.0, 120.0)),
            halfWidth = { 3.5 },
        )
        val combined = BooleanArray(200 * 200) { index -> trunk.tape[index] || spur.tape[index] }
        val mask = SegmentationMask(200, 200, combined)

        val strict = extractor.extract(mask)
        // Same mask with a spur-sized threshold: proves the default rejection
        // comes from the arc-length gate, not from the spur being invisible.
        val lax = CenterlineExtractor(
            CenterlineExtractorConfig(
                minBranchArcLengthPixels = 2.0,
                branchArcLengthWidthFactor = 0.05,
            ),
        ).extract(mask)

        assertEquals(0, strict.topology.branchCount)
        assertTrue("lax branchCount ${lax.topology.branchCount}", lax.topology.branchCount >= 1)
    }

    private fun maskAlongPath(
        width: Int,
        height: Int,
        path: List<Point>,
        halfWidth: (pathFraction: Double) -> Double,
    ): SegmentationMask {
        val tape = BooleanArray(width * height)
        for (y in 0 until height) {
            for (x in 0 until width) {
                val point = Point(x.toDouble(), y.toDouble())
                val nearest = nearestPathFraction(point, path)
                if (nearest.distance <= halfWidth(nearest.fraction)) tape[y * width + x] = true
            }
        }
        return SegmentationMask(width, height, tape)
    }

    private data class Point(val x: Double, val y: Double)
    private data class Nearest(val distance: Double, val fraction: Double)

    private fun nearestPathFraction(point: Point, path: List<Point>): Nearest {
        val segmentLengths = path.zipWithNext(::distance)
        val totalLength = segmentLengths.sum().coerceAtLeast(1e-9)
        var traversed = 0.0
        var bestDistance = Double.POSITIVE_INFINITY
        var bestFraction = 0.0
        path.zipWithNext().forEachIndexed { index, (start, end) ->
            val dx = end.x - start.x
            val dy = end.y - start.y
            val lengthSquared = dx * dx + dy * dy
            val t = if (lengthSquared == 0.0) 0.0 else {
                ((point.x - start.x) * dx + (point.y - start.y) * dy) / lengthSquared
            }.coerceIn(0.0, 1.0)
            val projected = Point(start.x + t * dx, start.y + t * dy)
            val candidateDistance = distance(point, projected)
            if (candidateDistance < bestDistance) {
                bestDistance = candidateDistance
                bestFraction = (traversed + t * segmentLengths[index]) / totalLength
            }
            traversed += segmentLengths[index]
        }
        return Nearest(bestDistance, bestFraction)
    }

    private fun distanceToPath(point: Point, path: List<Point>): Double =
        nearestPathFraction(point, path).distance

    private fun distance(a: Point, b: Point): Double = hypot(a.x - b.x, a.y - b.y)

    private fun polylineLength(path: List<Point>): Double = path.zipWithNext(::distance).sum()

    private fun pathLength(points: List<CenterlinePoint>): Double = points.zipWithNext { a, b ->
        hypot(a.x - b.x, a.y - b.y)
    }.sum()

}
