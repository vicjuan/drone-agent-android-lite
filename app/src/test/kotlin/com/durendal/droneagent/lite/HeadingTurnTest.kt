package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Test

class HeadingTurnTest {
    @Test
    fun `right turn accumulates clockwise progress across 180 boundary`() {
        val turn = HeadingTurn(170.0)

        assertEquals(160.0, turn.update(-170.0), 0.0)
        assertEquals(90.0, turn.update(-100.0), 0.0)
        assertEquals(0.0, turn.update(-10.0), 0.0)
        assertEquals(180.0, turn.progressDegrees, 0.0)
    }


    @Test
    fun `opposite drift does not create directed progress`() {
        val turn = HeadingTurn(0.0)

        assertEquals(180.0, turn.update(-3.0), 0.0)
        assertEquals(178.0, turn.update(2.0), 0.0)
    }
}
