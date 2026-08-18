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
        assertTrue(estimate.nearFieldCenterY > FRAME_HEIGHT * 0.95)
        assertTrue(estimate.lookaheadCenterY < estimate.nearFieldCenterY)
        assertTrue(estimate.arcLengthFraction > 0.9)
        assertTrue(estimate.curvatureDegrees > 8.0)
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
        assertEquals(0.0, estimate.curvatureDegrees, 0.1)
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
    @Test
    fun `horizontal fallback follows tracked tape width instead of a thin floor seam`() {
        val mask = ByteArray(FRAME_WIDTH * FRAME_HEIGHT)
        for (x in 100 until 550) {
            fillColumnRun(mask, x, centerY = 275.0, halfWidth = 15)
        }
        for (x in 0 until FRAME_WIDTH) {
            fillColumnRun(mask, x, centerY = 325.0, halfWidth = 3)
        }

        val unanchored = estimator.estimate(
            mask = mask,
            frameWidth = FRAME_WIDTH,
            frameHeight = FRAME_HEIGHT,
            left = 0,
            top = 0,
            right = FRAME_WIDTH,
            bottom = FRAME_HEIGHT,
        )
        val tracked = checkNotNull(
            estimator.estimateHorizontalFallback(
                mask = mask,
                frameWidth = FRAME_WIDTH,
                frameHeight = FRAME_HEIGHT,
                left = 0,
                top = 0,
                right = FRAME_WIDTH,
                bottom = FRAME_HEIGHT,
                expectedMedianWidthFraction = 30.0 / FRAME_HEIGHT,
            ),
        )
        val acquired = checkNotNull(
            estimator.estimateHorizontalFallback(
                mask = mask,
                frameWidth = FRAME_WIDTH,
                frameHeight = FRAME_HEIGHT,
                left = 0,
                top = 0,
                right = FRAME_WIDTH,
                bottom = FRAME_HEIGHT,
            ),
        )


        assertEquals(null, unanchored)
        assertTrue(tracked.horizontalFallback)
        assertTrue(kotlin.math.abs(tracked.lookaheadAngleFromVerticalDegrees) > 80.0)
        assertEquals(30.0 / FRAME_HEIGHT, tracked.medianWidthFraction, 0.01)
        assertTrue(tracked.bounds.bottom < 310)
        assertTrue(acquired.horizontalFallback)
        assertEquals(30.0 / FRAME_HEIGHT, acquired.medianWidthFraction, 0.01)
    }

    @Test
    fun `horizontal fallback anchors the endpoint nearest the image bottom`() {
        val mask = ByteArray(FRAME_WIDTH * FRAME_HEIGHT)
        for (x in 100 until 550) {
            val centerY = 220.0 + (x - 100) * 0.20
            fillColumnRun(mask, x, centerY = centerY, halfWidth = 15)
        }

        val estimate = checkNotNull(
            estimator.estimateHorizontalFallback(
                mask = mask,
                frameWidth = FRAME_WIDTH,
                frameHeight = FRAME_HEIGHT,
                left = 90,
                top = 180,
                right = 560,
                bottom = 340,
                expectedMedianWidthFraction = 30.0 / FRAME_HEIGHT,
            ),
        )

        assertTrue(estimate.nearFieldCenterX > 500.0)
        assertTrue(estimate.nearFieldCenterY > 290.0)
        assertTrue(estimate.lookaheadCenterX < estimate.nearFieldCenterX)
    }

    @Test
    fun `horizontal fallback rejects a ribbon whose width no longer matches tracking`() {
        val mask = ByteArray(FRAME_WIDTH * FRAME_HEIGHT)
        for (x in 100 until 550) {
            fillColumnRun(mask, x, centerY = 275.0, halfWidth = 15)
        }

        val estimate = estimator.estimateHorizontalFallback(
            mask = mask,
            frameWidth = FRAME_WIDTH,
            frameHeight = FRAME_HEIGHT,
            left = 0,
            top = 0,
            right = FRAME_WIDTH,
            bottom = FRAME_HEIGHT,
            expectedMedianWidthFraction = 10.0 / FRAME_HEIGHT,
        )

        assertEquals(null, estimate)
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
    private fun fillColumnRun(mask: ByteArray, x: Int, centerY: Double, halfWidth: Int) {
        val top = (centerY - halfWidth).toInt().coerceAtLeast(0)
        val bottom = (centerY + halfWidth).toInt().coerceAtMost(FRAME_HEIGHT)
        for (y in top until bottom) mask[y * FRAME_WIDTH + x] = 0xFF.toByte()
    }

    private companion object {
        const val FRAME_WIDTH = 640
        const val FRAME_HEIGHT = 360
        const val HALF_TAPE_WIDTH = 15.0
        const val CURVE_DISPLACEMENT = 140.0
    }
}
