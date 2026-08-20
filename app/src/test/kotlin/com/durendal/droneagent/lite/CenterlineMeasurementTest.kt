package com.durendal.droneagent.lite

import kotlin.math.abs
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The measurement layer decides whether a centerline may be pursued, so these
 * pin what a caller downstream cannot check for itself: that a short chain
 * yields no look-ahead point, that the angles describe the tape rather than the
 * image axes, and that the two turn quantities stay separate.
 */
class CenterlineMeasurementTest {

    @Test
    fun `a straight vertical chain measures zero angle and a look-ahead ahead of the anchor`() {
        val measurement = requireNotNull(measure(verticalChain(pointCount = 120)))

        assertEquals(0.0, measurement.nearFieldAngleFromVerticalDegrees, 1.0)
        assertEquals(0.0, measurement.lookaheadHeadingChangeDegrees, 1.0)
        assertEquals(0.0, measurement.totalPathTurnDegrees, 2.0)
        val lookahead = requireNotNull(measurement.lookahead)
        assertTrue(
            "look-ahead ${lookahead.yFraction} must sit ahead of anchor ${measurement.anchorYFraction}",
            lookahead.yFraction < measurement.anchorYFraction,
        )
    }

    @Test
    fun `control anchor projects onto the path nearest the aircraft instead of using its endpoint`() {
        val path = buildList {
            for (x in 140..320 step 20) {
                add(CenterlinePoint(x.toDouble(), 359.0, WIDTH_PIXELS))
            }
            for (y in 357 downTo 101 step 2) {
                add(CenterlinePoint(320.0, y.toDouble(), WIDTH_PIXELS))
            }
        }

        val measurement = requireNotNull(measure(path))

        assertEquals(TRACKING_TARGET_X_FRACTION, measurement.anchorXFraction, 0.001)
        assertEquals(TRACKING_TARGET_Y_FRACTION, measurement.anchorYFraction, 0.001)
        assertEquals(0.0, measurement.nearFieldAngleFromVerticalDegrees, 1.0)
        val lookahead = requireNotNull(measurement.lookahead)
        assertEquals(TRACKING_TARGET_X_FRACTION, lookahead.xFraction, 0.001)
        assertTrue(lookahead.yFraction < measurement.anchorYFraction)
    }

    @Test
    fun `a horizontal chain is measured the same way as a vertical one`() {
        val vertical = requireNotNull(measure(verticalChain(pointCount = 120)))
        val horizontal = requireNotNull(measure(horizontalChain(pointCount = 120)))

        // Ninety degrees apart in image space, identical in every quality the
        // controller uses: direction independence is the whole point of the
        // replacement geometry.
        assertEquals(90.0, abs(horizontal.nearFieldAngleFromVerticalDegrees), 1.0)
        assertEquals(vertical.arcLengthFraction, horizontal.arcLengthFraction, 0.02)
        assertEquals(vertical.medianWidthFraction, horizontal.medianWidthFraction, 0.001)
        assertEquals(vertical.totalPathTurnDegrees, horizontal.totalPathTurnDegrees, 2.0)
        assertNotNull(horizontal.lookahead)
    }

    @Test
    fun `a chain too short to aim at yields no look-ahead point`() {
        val measurement = requireNotNull(measure(verticalChain(pointCount = 12, stepPixels = 1.0)))

        assertNull("a stub of tape must not offer a look-ahead point", measurement.lookahead)
    }

    @Test
    fun `a chain below the minimum point count is not measurable at all`() {
        assertNull(measure(verticalChain(pointCount = 4)))
    }

    @Test
    fun `a left and a right curve turn the same amount in opposite directions`() {
        val left = requireNotNull(measure(arcChain(displacement = -160.0)))
        val right = requireNotNull(measure(arcChain(displacement = 160.0)))

        // Total turn is unsigned magnitude, so mirrored arcs must agree on it.
        assertEquals(left.totalPathTurnDegrees, right.totalPathTurnDegrees, 3.0)
        assertTrue("a curve must register turn", right.totalPathTurnDegrees > 20.0)
        // The near-field angle is signed, and that is what tells them apart.
        assertTrue(
            "left=${left.nearFieldAngleFromVerticalDegrees} right=${right.nearFieldAngleFromVerticalDegrees}",
            left.nearFieldAngleFromVerticalDegrees < right.nearFieldAngleFromVerticalDegrees,
        )
        assertTrue("a steady arc turns one way: ${'$'}{right.turnConsistency}", right.turnConsistency > 0.9)
        assertTrue("a steady arc turns one way: ${'$'}{left.turnConsistency}", left.turnConsistency > 0.9)
    }

    @Test
    fun `an S bend turns a lot in total but not consistently`() {
        val sBend = requireNotNull(measure(sBendChain()))

        assertTrue(
            "an S bend reverses, so consistency must be low: ${sBend.turnConsistency}",
            sBend.turnConsistency < 0.5,
        )
        assertTrue(
            "an S bend is not a credible arc",
            !TapePathQualityPolicy.isCredibleArc(sBend),
        )
    }

    @Test
    fun `a hairpin turns far and stays consistent`() {
        val hairpin = requireNotNull(measure(hairpinChain()))

        assertTrue(
            "a hairpin should register a large total turn: ${hairpin.totalPathTurnDegrees}",
            hairpin.totalPathTurnDegrees > 90.0,
        )
        assertTrue("a hairpin turns one way", hairpin.turnConsistency > 0.8)
    }

    @Test
    fun `local look-ahead bend and whole-path turn are different quantities`() {
        val gentleLongArc = requireNotNull(measure(arcChain(displacement = 160.0, pointCount = 200)))

        // The look-ahead only reaches part way along, so the local bend must be
        // strictly smaller than the turn of the whole visible chain. Collapsing
        // them into one number is what made a wall edge look like an arc.
        assertTrue(
            "local=${gentleLongArc.lookaheadHeadingChangeDegrees} " +
                "total=${gentleLongArc.totalPathTurnDegrees}",
            gentleLongArc.lookaheadHeadingChangeDegrees < gentleLongArc.totalPathTurnDegrees,
        )
    }

    @Test
    fun `a straight chain is not a credible arc`() {
        val straight = requireNotNull(measure(verticalChain(pointCount = 120)))

        assertTrue(
            "a straight run must fail the arc test",
            !TapePathQualityPolicy.isCredibleArc(straight),
        )
    }

    private fun measure(points: List<CenterlinePoint>) = CenterlineMeasurement.measure(
        estimate = CenterlineEstimate(
            points = points,
            confidence = 1.0,
            components = CenterlineConfidence(1.0, 1.0, 1.0, 1.0, 1.0),
        ),
        frameWidth = FRAME_WIDTH,
        frameHeight = FRAME_HEIGHT,
    )

    /** Ordered from the image bottom upward, as the extractor emits them. */
    private fun verticalChain(pointCount: Int, stepPixels: Double = 2.0) =
        (0 until pointCount).map { index ->
            CenterlinePoint(
                x = FRAME_WIDTH / 2.0,
                y = FRAME_HEIGHT - 1.0 - index * stepPixels,
                widthPixels = WIDTH_PIXELS,
            )
        }

    private fun horizontalChain(pointCount: Int, stepPixels: Double = 2.0) =
        (0 until pointCount).map { index ->
            CenterlinePoint(
                x = 200.0 + index * stepPixels,
                y = FRAME_HEIGHT / 2.0,
                widthPixels = WIDTH_PIXELS,
            )
        }

    /** A steady parabolic arc, positive displacement bending toward image-right. */
    private fun arcChain(displacement: Double, pointCount: Int = 120) =
        (0 until pointCount).map { index ->
            val progress = index / (pointCount - 1.0)
            CenterlinePoint(
                x = FRAME_WIDTH / 2.0 + displacement * progress * progress,
                y = FRAME_HEIGHT - 1.0 - index * 2.0,
                widthPixels = WIDTH_PIXELS,
            )
        }

    /** Right then left: large accumulated motion, near-zero net turn. */
    private fun sBendChain() = (0 until 160).map { index ->
        val progress = index / 159.0
        CenterlinePoint(
            x = FRAME_WIDTH / 2.0 + 90.0 * kotlin.math.sin(progress * 2.0 * Math.PI),
            y = FRAME_HEIGHT - 1.0 - index * 2.0,
            widthPixels = WIDTH_PIXELS,
        )
    }

    /** A half circle: the tape doubles back on itself inside the frame. */
    private fun hairpinChain() = (0 until 140).map { index ->
        val angle = Math.PI * index / 139.0
        CenterlinePoint(
            x = FRAME_WIDTH / 2.0 + 90.0 * (1.0 - kotlin.math.cos(angle)),
            y = FRAME_HEIGHT - 40.0 - 90.0 * kotlin.math.sin(angle),
            widthPixels = WIDTH_PIXELS,
        )
    }

    private companion object {
        const val FRAME_WIDTH = 640
        const val FRAME_HEIGHT = 360
        const val WIDTH_PIXELS = 24.0
    }
}
