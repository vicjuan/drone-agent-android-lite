package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Test

class TapeOrientationTest {
    @Test
    fun `orientation is signed from image-up and ignores endpoint order`() {
        assertEquals(0.0, TapeOrientation.deviationFromVerticalDegrees(0.0, -10.0), 0.001)
        assertEquals(45.0, TapeOrientation.deviationFromVerticalDegrees(10.0, -10.0), 0.001)
        assertEquals(45.0, TapeOrientation.deviationFromVerticalDegrees(-10.0, 10.0), 0.001)
        assertEquals(-45.0, TapeOrientation.deviationFromVerticalDegrees(-10.0, -10.0), 0.001)
    }
}
