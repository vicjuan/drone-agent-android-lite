package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Test

class ShuttleSequenceTest {
    @Test
    fun `sequence alternates forward and right half-turn forever`() {
        val steps = generateSequence(ShuttleStep.FORWARD) { it.next() }
            .take(6)
            .toList()

        assertEquals(
            listOf(
                ShuttleStep.FORWARD,
                ShuttleStep.TURN_RIGHT,
                ShuttleStep.FORWARD,
                ShuttleStep.TURN_RIGHT,
                ShuttleStep.FORWARD,
                ShuttleStep.TURN_RIGHT,
            ),
            steps,
        )
        assertEquals(listOf(true, false, true, false, true, false), steps.map { it.isForward })
    }
}
