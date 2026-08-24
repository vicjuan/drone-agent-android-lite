package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Assert.assertThrows
import org.junit.Test

/**
 * The quality contract is only worth anything if an under-specified detection
 * cannot present itself as one the controller may pursue. These tests pin the
 * two halves of that: quality follows the look-ahead point, and a look-ahead
 * point is never half-present.
 */
class TapeDetectionContractTest {

    @Test
    fun `a detection with a look-ahead point is a full path`() {
        val detection = detection(TapeLookahead(xFraction = 0.5, yFraction = 0.2))

        assertEquals(PathQuality.FULL_PATH, detection.quality)
    }

    @Test
    fun `a detection without a look-ahead point is near field only`() {
        val detection = detection(lookahead = null)

        assertEquals(PathQuality.NEAR_FIELD_ONLY, detection.quality)
    }

    @Test
    fun `a look-ahead point outside the frame is rejected`() {
        assertThrows(IllegalArgumentException::class.java) {
            TapeLookahead(xFraction = 1.4, yFraction = 0.2)
        }
        assertThrows(IllegalArgumentException::class.java) {
            TapeLookahead(xFraction = 0.5, yFraction = -0.1)
        }
    }

    private fun detection(lookahead: TapeLookahead?) = TapeDetection(
        sourceWidth = 640,
        sourceHeight = 360,
        capturedAtNanos = 0L,
        bounds = NormalizedRect(left = 0.4, top = 0.1, right = 0.6, bottom = 0.9),
        confidence = 0.8,
        angleFromVerticalDegrees = 0.0,
        longSideFraction = 0.8,
        nearFieldOffsetFraction = 0.0,
        lookahead = lookahead,
    )
}
