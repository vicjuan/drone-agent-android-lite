package com.durendal.droneagent.lite

import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class BatteryLandingGateTest {
    @Test
    fun `thirteen percent triggers once while flying`() {
        val gate = BatteryLandingGate()

        assertFalse(gate.evaluate(percent = 14, flying = true, connected = true, landingRequested = false))
        assertTrue(gate.evaluate(percent = 13, flying = true, connected = true, landingRequested = false))
        assertFalse(gate.evaluate(percent = 12, flying = true, connected = true, landingRequested = false))
    }

    @Test
    fun `grounded disconnected or already landing does not trigger`() {
        val gate = BatteryLandingGate()

        assertFalse(gate.evaluate(percent = 13, flying = false, connected = true, landingRequested = false))
        assertFalse(gate.evaluate(percent = 13, flying = true, connected = false, landingRequested = false))
        assertFalse(gate.evaluate(percent = 13, flying = true, connected = true, landingRequested = true))
    }

    @Test
    fun `landing gate rearms only after returning to ground`() {
        val gate = BatteryLandingGate()
        assertTrue(gate.evaluate(percent = 13, flying = true, connected = true, landingRequested = false))
        assertFalse(gate.evaluate(percent = 13, flying = true, connected = true, landingRequested = false))

        assertFalse(gate.evaluate(percent = 13, flying = false, connected = true, landingRequested = false))
        assertTrue(gate.evaluate(percent = 13, flying = true, connected = true, landingRequested = false))
    }
}
