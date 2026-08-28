package com.durendal.droneagent.lite

import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class BoardChromaPolicyTest {
    @Test
    fun `cardboard on both sides with sufficient coverage is accepted`() {
        assertTrue(
            BilateralBoardPolicy.accepts(
                pairCount = 40,
                sampleCoverageFraction = 0.90,
                compatiblePairFraction = 0.90,
                leftMatchFraction = 0.90,
                rightMatchFraction = 0.88,
                bothSidesMatchFraction = 0.84,
            ),
        )
    }

    @Test
    fun `cardboard on only one side is rejected`() {
        assertFalse(
            BilateralBoardPolicy.accepts(
                pairCount = 40,
                sampleCoverageFraction = 0.90,
                compatiblePairFraction = 0.90,
                leftMatchFraction = 0.95,
                rightMatchFraction = 0.50,
                bothSidesMatchFraction = 0.48,
            ),
        )
    }

    @Test
    fun `different bad sections on each side cannot pass through averages`() {
        assertFalse(
            BilateralBoardPolicy.accepts(
                pairCount = 40,
                sampleCoverageFraction = 0.90,
                compatiblePairFraction = 0.90,
                leftMatchFraction = 0.90,
                rightMatchFraction = 0.90,
                bothSidesMatchFraction = 0.75,
            ),
        )
    }

    @Test
    fun `samples that mostly fall outside the image or on black pixels are rejected`() {
        assertFalse(
            BilateralBoardPolicy.accepts(
                pairCount = 40,
                sampleCoverageFraction = 0.60,
                compatiblePairFraction = 0.90,
                leftMatchFraction = 0.95,
                rightMatchFraction = 0.95,
                bothSidesMatchFraction = 0.90,
            ),
        )
    }

    @Test
    fun `two differently coloured sides are rejected`() {
        assertFalse(
            BilateralBoardPolicy.accepts(
                pairCount = 40,
                sampleCoverageFraction = 0.90,
                compatiblePairFraction = 0.60,
                leftMatchFraction = 0.95,
                rightMatchFraction = 0.95,
                bothSidesMatchFraction = 0.90,
            ),
        )
    }
}
