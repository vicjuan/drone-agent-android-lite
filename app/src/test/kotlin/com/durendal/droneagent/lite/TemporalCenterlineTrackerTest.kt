package com.durendal.droneagent.lite

import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

class TemporalCenterlineTrackerTest {
    private val extractor = CenterlineExtractor()
    private val tracker = TemporalCenterlineTracker()

    @Test
    fun `normal slices follow a translated curved ribbon`() {
        val previousPath = arcPath(shiftX = 0.0)
        val currentPath = arcPath(shiftX = 5.0)
        val previous = extractor.extract(maskAlongPath(previousPath))

        val tracked = tracker.track(maskAlongPath(currentPath), previous)

        assertTrue("translated path should be tracked", tracked != null)
        checkNotNull(tracked)
        assertTrue("tracked path needs enough control geometry", tracked.points.size >= 20)
        assertTrue(
            "tracked samples should remain on the current ribbon",
            tracked.points.all { point ->
                distanceToPath(Point(point.x, point.y), currentPath) <= 2.5
            },
        )
        val meanShift = tracked.points.zip(previous.points.pointsAtSimilarFractions(tracked.points.size))
            .sumOf { (current, old) -> current.x - old.x } / tracked.points.size
        assertTrue("mean horizontal update was $meanShift", meanShift in 3.0..7.0)
    }

    @Test
    fun `nearest compatible run wins over a parallel dark distractor`() {
        val previousPath = verticalPath(x = 100.0)
        val currentTape = maskAlongPath(verticalPath(x = 105.0))
        val distractor = maskAlongPath(verticalPath(x = 72.0), halfWidth = 4.0)
        val combined = SegmentationMask(
            width = WIDTH,
            height = HEIGHT,
            tape = BooleanArray(WIDTH * HEIGHT) { index ->
                currentTape.tape[index] || distractor.tape[index]
            },
        )
        val previous = extractor.extract(maskAlongPath(previousPath))

        val tracked = tracker.track(combined, previous)

        assertTrue(tracked != null)
        checkNotNull(tracked)
        assertTrue(
            "tracker jumped to the distractor",
            tracked.points.all { point -> point.x in 102.0..108.0 },
        )
    }

    @Test
    fun `large inter-frame displacement falls back to full extraction`() {
        val previous = extractor.extract(maskAlongPath(verticalPath(x = 100.0)))

        val tracked = tracker.track(maskAlongPath(verticalPath(x = 135.0)), previous)

        assertNull(tracked)
    }

    @Test
    fun `long unsupported section falls back instead of bridging guessed geometry`() {
        val previous = extractor.extract(maskAlongPath(verticalPath(x = 100.0)))
        val interrupted = maskAlongPath(verticalPath(x = 103.0))
        for (y in 75..125) {
            for (x in 0 until WIDTH) interrupted.tape[y * WIDTH + x] = false
        }

        val tracked = tracker.track(interrupted, previous)

        assertNull(tracked)
    }

    @Test
    fun `known branch topology requires full extraction`() {
        val points = extractor.extract(maskAlongPath(verticalPath(x = 100.0))).points
        val branched = CenterlineEstimate(
            points = points,
            confidence = 1.0,
            components = CenterlineConfidence(1.0, 1.0, 1.0, 1.0, 1.0),
            topology = CenterlineTopology(branchCount = 1),
        )

        assertNull(tracker.track(maskAlongPath(verticalPath(x = 102.0)), branched))
    }

    @Test
    fun `closed loop topology requires full extraction`() {
        val points = extractor.extract(maskAlongPath(verticalPath(x = 100.0))).points
        val closedLoop = CenterlineEstimate(
            points = points,
            confidence = 1.0,
            components = CenterlineConfidence(1.0, 1.0, 1.0, 1.0, 1.0),
            topology = CenterlineTopology(closedLoop = true),
        )

        assertNull(tracker.track(maskAlongPath(verticalPath(x = 102.0)), closedLoop))
    }

    private fun arcPath(shiftX: Double): List<Point> = (0..64).map { step ->
        val angle = PI * (0.12 + 0.76 * step / 64.0)
        Point(
            x = 110.0 + shiftX + 72.0 * cos(angle),
            y = 105.0 + 72.0 * sin(angle),
        )
    }

    private fun verticalPath(x: Double): List<Point> =
        listOf(Point(x, 190.0), Point(x, 10.0))

    private fun maskAlongPath(
        path: List<Point>,
        halfWidth: Double = 6.0,
    ): SegmentationMask {
        val tape = BooleanArray(WIDTH * HEIGHT)
        for (y in 0 until HEIGHT) {
            for (x in 0 until WIDTH) {
                if (distanceToPath(Point(x.toDouble(), y.toDouble()), path) <= halfWidth) {
                    tape[y * WIDTH + x] = true
                }
            }
        }
        return SegmentationMask(WIDTH, HEIGHT, tape)
    }

    private fun List<CenterlinePoint>.pointsAtSimilarFractions(count: Int): List<CenterlinePoint> =
        List(count) { index ->
            this[(index.toDouble() * lastIndex / (count - 1).coerceAtLeast(1)).toInt()]
        }

    private fun distanceToPath(point: Point, path: List<Point>): Double =
        path.zipWithNext().minOf { (start, end) ->
            val dx = end.x - start.x
            val dy = end.y - start.y
            val lengthSquared = dx * dx + dy * dy
            val fraction = if (lengthSquared == 0.0) {
                0.0
            } else {
                ((point.x - start.x) * dx + (point.y - start.y) * dy) / lengthSquared
            }.coerceIn(0.0, 1.0)
            hypot(
                point.x - (start.x + fraction * dx),
                point.y - (start.y + fraction * dy),
            )
        }

    private data class Point(val x: Double, val y: Double)

    private companion object {
        const val WIDTH = 220
        const val HEIGHT = 210
    }
}
