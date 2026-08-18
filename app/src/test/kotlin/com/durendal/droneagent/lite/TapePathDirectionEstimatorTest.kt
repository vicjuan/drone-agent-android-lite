package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

class TapePathDirectionEstimatorTest {
    private val estimator = TapePathDirectionEstimator()

    @Test
    fun `lookahead tangent points right along a right bending ribbon`() {
        val estimate = estimateRibbon(curveDirection = 1.0)

        assertNotNull(estimate)
        checkNotNull(estimate)
        assertEquals(FRAME_WIDTH / 2.0, estimate.nearFieldCenterX, 2.0)
        assertTrue(estimate.lookaheadAngleFromVerticalDegrees > 10.0)
        assertTrue(estimate.arcLengthFraction > 0.9)
    }

    @Test
    fun `lookahead tangent points left along a left bending ribbon`() {
        val estimate = checkNotNull(estimateRibbon(curveDirection = -1.0))

        assertTrue(estimate.lookaheadAngleFromVerticalDegrees < -10.0)
    }

    @Test
    fun `straight ribbon has a neutral lookahead tangent`() {
        val estimate = checkNotNull(estimateRibbon(curveDirection = 0.0))

        assertEquals(0.0, estimate.lookaheadAngleFromVerticalDegrees, 0.1)
    }

    @Test
    fun `wide tape wins over a connected thin floor seam near image center`() {
        val mask = ByteArray(FRAME_WIDTH * FRAME_HEIGHT)
        for (y in 0 until FRAME_HEIGHT) {
            val nearFraction = y / (FRAME_HEIGHT - 1.0)
            val junctionX = 360.0
            fillRun(mask, y, junctionX + 80.0 * nearFraction, halfWidth = 15)
            fillRun(mask, y, junctionX + 40.0 * nearFraction, halfWidth = 3)
        }

        val estimate = checkNotNull(
            estimator.estimate(
                mask = mask,
                frameWidth = FRAME_WIDTH,
                frameHeight = FRAME_HEIGHT,
                left = 340,
                top = 0,
                right = 456,
                bottom = FRAME_HEIGHT,
            ),
        )

        assertTrue(estimate.nearFieldCenterX > 425.0)
        assertTrue(estimate.medianWidthFraction > 0.07)
    }

    private fun estimateRibbon(curveDirection: Double): TapePathEstimate? {
        val mask = ByteArray(FRAME_WIDTH * FRAME_HEIGHT)
        for (y in 0 until FRAME_HEIGHT) {
            val forwardFraction = (FRAME_HEIGHT - 1 - y) / (FRAME_HEIGHT - 1.0)
            val center =
                FRAME_WIDTH / 2.0 + curveDirection * CURVE_DISPLACEMENT *
                    forwardFraction * forwardFraction
            val left = (center - HALF_TAPE_WIDTH).toInt().coerceAtLeast(0)
            val right = (center + HALF_TAPE_WIDTH).toInt().coerceAtMost(FRAME_WIDTH)
            for (x in left until right) mask[y * FRAME_WIDTH + x] = 0xFF.toByte()
        }
        return estimator.estimate(
            mask = mask,
            frameWidth = FRAME_WIDTH,
            frameHeight = FRAME_HEIGHT,
            left = 0,
            top = 0,
            right = FRAME_WIDTH,
            bottom = FRAME_HEIGHT,
        )
    }

    private fun fillRun(mask: ByteArray, y: Int, centerX: Double, halfWidth: Int) {
        val left = (centerX - halfWidth).toInt().coerceAtLeast(0)
        val right = (centerX + halfWidth).toInt().coerceAtMost(FRAME_WIDTH)
        for (x in left until right) mask[y * FRAME_WIDTH + x] = 0xFF.toByte()
    }

    private companion object {
        const val FRAME_WIDTH = 640
        const val FRAME_HEIGHT = 360
        const val HALF_TAPE_WIDTH = 15.0
        const val CURVE_DISPLACEMENT = 140.0
    }
}
