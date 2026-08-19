package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Test

class GimbalInputMappingTest {
    @Test
    fun `downward pad input produces negative Mini 4 Pro pitch rate`() {
        assertEquals(-24.0, gimbalPitchRateForInput(-0.8, 30.0), 0.0)
    }

    @Test
    fun `upward pad input produces positive Mini 4 Pro pitch rate`() {
        assertEquals(24.0, gimbalPitchRateForInput(0.8, 30.0), 0.0)
    }

    @Test
    fun `pitch input is clamped before conversion`() {
        assertEquals(-30.0, gimbalPitchRateForInput(-2.0, 30.0), 0.0)
        assertEquals(30.0, gimbalPitchRateForInput(2.0, 30.0), 0.0)
    }
}
