package com.durendal.droneagent.lite

import kotlin.math.abs
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * The measurement layer decides whether a centerline may be pursued, so these
 * pin the two things a caller downstream cannot check for itself: that a short
 * chain yields no look-ahead point, and that the angles describe the tape rather
 * than the image axes.
 */
class CenterlineMeasurementTest {

    @Test
    fun `a straight vertical chain measures zero angle and a look-ahead ahead of the anchor`() {
        val measurement = measure(verticalChain(pointCount = 120))

        assertNotNull(measurement)
        requireNotNull(measurement)
        assertEquals(0.0, measurement.nearFieldAngleFromVerticalDegrees, 1.0)
        assertEquals(0.0, measurement.curvatureDegrees, 1.0)
        val lookahead = requireNotNull(measurement.lookahead)
        assertTrue(
            "look-ahead ${lookahead.yFraction} must sit ahead of anchor ${measurement.anchorYFraction}",
            lookahead.yFraction < measurement.anchorYFraction,
        )
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
        assertNotNull(horizontal.lookahead)
    }

    @Test
    fun `a chain too short to aim at yields no look-ahead point`() {
        val measurement = measure(verticalChain(pointCount = 12, stepPixels = 1.0))

        assertNotNull(measurement)
        assertNull(
            "a stub of tape must not offer a look-ahead point",
            requireNotNull(measurement).lookahead,
        )
    }

    @Test
    fun `a chain below the minimum point count is not measurable at all`() {
        assertNull(measure(verticalChain(pointCount = 4)))
    }

    @Test
    fun `a bend is reported as curvature between the near field and the look-ahead`() {
        val measurement = requireNotNull(measure(bentChain()))

        assertTrue(
            "curvature=${measurement.curvatureDegrees} should register the bend",
            measurement.curvatureDegrees > 15.0,
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
                x = index * stepPixels,
                y = FRAME_HEIGHT / 2.0,
                widthPixels = WIDTH_PIXELS,
            )
        }

    /**
     * A steady arc from the anchor onward, so the bend is inside the arc-length
     * window the look-ahead actually reaches rather than beyond it.
     */
    private fun bentChain() = (0 until 120).map { index ->
        val progress = index / 119.0
        CenterlinePoint(
            x = FRAME_WIDTH / 2.0 + 160.0 * progress * progress,
            y = FRAME_HEIGHT - 1.0 - index * 2.0,
            widthPixels = WIDTH_PIXELS,
        )
    }

    private companion object {
        const val FRAME_WIDTH = 640
        const val FRAME_HEIGHT = 360
        const val WIDTH_PIXELS = 24.0
    }
}
