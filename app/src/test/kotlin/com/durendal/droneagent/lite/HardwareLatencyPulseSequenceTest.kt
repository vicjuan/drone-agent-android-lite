package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test

class HardwareLatencyPulseSequenceTest {

    @Test
    fun `two cycles preserve command order and exact phase boundaries`() {
        val sequence = HardwareLatencyPulseSequence(
            cycleCount = 2,
            speedMetersPerSecond = 0.5,
            baselineNanos = 3L,
            pulseNanos = 1L,
            settleNanos = 2L,
        )

        assertStep(
            sequence.start(0L),
            HardwareLatencyPulsePhase.BASELINE,
            cycle = 0,
            forwardMetersPerSecond = 0.0,
            markerWhite = false,
        )
        assertNull(sequence.advance(2L))
        assertStep(sequence.advance(3L), HardwareLatencyPulsePhase.FORWARD, 1, 0.5, true)
        assertStep(sequence.advance(4L), HardwareLatencyPulsePhase.SETTLE_AFTER_FORWARD, 1, 0.0, false)
        assertNull(sequence.advance(5L))
        assertStep(sequence.advance(6L), HardwareLatencyPulsePhase.BACKWARD, 1, -0.5, true)
        assertStep(sequence.advance(7L), HardwareLatencyPulsePhase.SETTLE_AFTER_BACKWARD, 1, 0.0, false)
        assertStep(sequence.advance(9L), HardwareLatencyPulsePhase.FORWARD, 2, 0.5, true)
        assertStep(sequence.advance(10L), HardwareLatencyPulsePhase.SETTLE_AFTER_FORWARD, 2, 0.0, false)
        assertStep(sequence.advance(12L), HardwareLatencyPulsePhase.BACKWARD, 2, -0.5, true)
        assertStep(sequence.advance(13L), HardwareLatencyPulsePhase.SETTLE_AFTER_BACKWARD, 2, 0.0, false)
        assertStep(sequence.advance(15L), HardwareLatencyPulsePhase.COMPLETE, 2, 0.0, false)
        assertNull(sequence.advance(100L))
    }

    @Test
    fun `delayed tick starts the full next phase at actual application time`() {
        val sequence = HardwareLatencyPulseSequence(
            cycleCount = 1,
            baselineNanos = 3L,
            pulseNanos = 5L,
            settleNanos = 7L,
        )

        sequence.start(0L)
        assertStep(sequence.advance(30L), HardwareLatencyPulsePhase.FORWARD, 1, 0.5, true)
        assertNull(sequence.advance(34L))
        assertStep(sequence.advance(35L), HardwareLatencyPulsePhase.SETTLE_AFTER_FORWARD, 1, 0.0, false)
        assertNull(sequence.advance(41L))
        assertStep(sequence.advance(42L), HardwareLatencyPulsePhase.BACKWARD, 1, -0.5, true)
    }

    @Test
    fun `invalid experiment parameters are rejected`() {
        assertThrows(IllegalArgumentException::class.java) {
            HardwareLatencyPulseSequence(cycleCount = 0)
        }
        assertThrows(IllegalArgumentException::class.java) {
            HardwareLatencyPulseSequence(speedMetersPerSecond = Double.NaN)
        }
        assertThrows(IllegalArgumentException::class.java) {
            HardwareLatencyPulseSequence(pulseNanos = 0L)
        }
    }

    private fun assertStep(
        actual: HardwareLatencyPulseStep?,
        phase: HardwareLatencyPulsePhase,
        cycle: Int,
        forwardMetersPerSecond: Double,
        markerWhite: Boolean,
    ) {
        requireNotNull(actual)
        assertEquals(phase, actual.phase)
        assertEquals(cycle, actual.cycle)
        assertEquals(forwardMetersPerSecond, actual.forwardMetersPerSecond, 0.0)
        if (markerWhite) {
            assertTrue(actual.markerWhite)
        } else {
            assertFalse(actual.markerWhite)
        }
    }
}
