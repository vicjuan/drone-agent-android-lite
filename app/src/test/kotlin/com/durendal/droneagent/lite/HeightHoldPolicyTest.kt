package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class HeightHoldPolicyTest {
    @Test
    fun `one meter target climbs proportionally from lower heights`() {
        assertEquals(
            0.12,
            HeightHoldPolicy.climbRateMetersPerSecond(
                targetHeightMeters = 1.0,
                currentHeightMeters = 0.8,
                maximumRateMetersPerSecond = 0.3,
            ),
            1e-9,
        )
    }

    @Test
    fun `one meter target descends proportionally from takeoff hover`() {
        assertEquals(
            -0.12,
            HeightHoldPolicy.climbRateMetersPerSecond(
                targetHeightMeters = 1.0,
                currentHeightMeters = 1.2,
                maximumRateMetersPerSecond = 0.3,
            ),
            1e-9,
        )
    }

    @Test
    fun `vertical command is bounded and target band matches altitude resolution`() {
        assertEquals(
            0.3,
            HeightHoldPolicy.climbRateMetersPerSecond(1.0, 0.2, 0.3),
            0.0,
        )
        assertEquals(
            -0.3,
            HeightHoldPolicy.climbRateMetersPerSecond(0.7, 1.4, 0.3),
            0.0,
        )
        assertTrue(HeightHoldPolicy.isWithinTarget(1.0, 0.9))
        assertTrue(HeightHoldPolicy.isWithinTarget(1.0, 1.1))
        assertFalse(HeightHoldPolicy.isWithinTarget(1.0, 0.8))
    }
}
