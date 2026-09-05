package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test

class DirectionalVelocityPulseSequenceTest {

    @Test
    fun `each direction emits one bounded pulse between neutral intervals`() {
        DirectionalVelocityPulseDirection.entries.forEach { direction ->
            val sequence = DirectionalVelocityPulseSequence(
                direction = direction,
                speedMetersPerSecond = 0.8,
                baselineNanos = 2L,
                pulseNanos = 1L,
                settleNanos = 3L,
            )

            assertStep(
                sequence.start(0L),
                DirectionalVelocityPulsePhase.BASELINE,
                direction,
                forwardMetersPerSecond = 0.0,
                rightMetersPerSecond = 0.0,
                markerWhite = false,
            )
            assertNull(sequence.advance(1L))
            assertStep(
                sequence.advance(2L),
                DirectionalVelocityPulsePhase.PULSE,
                direction,
                forwardMetersPerSecond = 0.8 * direction.forwardSign,
                rightMetersPerSecond = 0.8 * direction.rightSign,
                markerWhite = true,
            )
            assertStep(
                sequence.advance(3L),
                DirectionalVelocityPulsePhase.SETTLE,
                direction,
                forwardMetersPerSecond = 0.0,
                rightMetersPerSecond = 0.0,
                markerWhite = false,
            )
            assertNull(sequence.advance(5L))
            val complete = sequence.advance(6L)
            assertStep(
                complete,
                DirectionalVelocityPulsePhase.COMPLETE,
                direction,
                forwardMetersPerSecond = 0.0,
                rightMetersPerSecond = 0.0,
                markerWhite = false,
            )
            assertTrue(requireNotNull(complete).complete)
            assertNull(sequence.advance(100L))
        }
    }

    @Test
    fun `fixed forward speed protocol ramps up holds then ramps down`() {
        val sequence = FixedDirectionSpeedSequence(
            direction = DirectionalVelocityPulseDirection.FORWARD,
            speedMetersPerSecond = 1.6,
            baselineNanos = 3L,
            rampUpNanos = 2L,
            holdNanos = 3L,
            rampDownNanos = 2L,
            settleNanos = 2L,
        )

        assertFixedStep(sequence.start(10L), FixedDirectionSpeedPhase.BASELINE, 0.0, false)
        assertNull(sequence.advance(12L))
        assertFixedStep(sequence.advance(13L), FixedDirectionSpeedPhase.RAMP_UP, 0.0, true)
        assertFixedStep(sequence.advance(14L), FixedDirectionSpeedPhase.RAMP_UP, 0.8, true)
        assertFixedStep(sequence.advance(15L), FixedDirectionSpeedPhase.HOLD, 1.6, true)
        assertNull(sequence.advance(17L))
        assertFixedStep(sequence.advance(18L), FixedDirectionSpeedPhase.RAMP_DOWN, 1.6, true)
        assertFixedStep(sequence.advance(19L), FixedDirectionSpeedPhase.RAMP_DOWN, 0.8, true)
        assertFixedStep(sequence.advance(20L), FixedDirectionSpeedPhase.SETTLE, 0.0, false)
        assertNull(sequence.advance(21L))
        val complete = sequence.advance(22L)
        assertFixedStep(complete, FixedDirectionSpeedPhase.COMPLETE, 0.0, false)
        assertTrue(requireNotNull(complete).complete)
        assertNull(sequence.advance(100L))
    }

    @Test
    fun `delayed fixed speed tick preserves every phase duration`() {
        val sequence = FixedDirectionSpeedSequence(
            direction = DirectionalVelocityPulseDirection.FORWARD,
            speedMetersPerSecond = 1.6,
            baselineNanos = 3L,
            rampUpNanos = 2L,
            holdNanos = 3L,
            rampDownNanos = 2L,
            settleNanos = 2L,
        )

        sequence.start(0L)
        assertFixedStep(sequence.advance(30L), FixedDirectionSpeedPhase.RAMP_UP, 0.0, true)
        assertFixedStep(sequence.advance(31L), FixedDirectionSpeedPhase.RAMP_UP, 0.8, true)
        assertFixedStep(sequence.advance(32L), FixedDirectionSpeedPhase.HOLD, 1.6, true)
        assertNull(sequence.advance(34L))
        assertFixedStep(sequence.advance(35L), FixedDirectionSpeedPhase.RAMP_DOWN, 1.6, true)
    }

    @Test
    fun `direction order follows forward right backward left`() {
        var direction = DirectionalVelocityPulseDirection.FORWARD
        val observed = buildList {
            repeat(DirectionalVelocityPulseDirection.entries.size) {
                add(direction)
                direction = direction.next()
            }
        }

        assertEquals(
            listOf(
                DirectionalVelocityPulseDirection.FORWARD,
                DirectionalVelocityPulseDirection.RIGHT,
                DirectionalVelocityPulseDirection.BACKWARD,
                DirectionalVelocityPulseDirection.LEFT,
            ),
            observed,
        )
        assertEquals(DirectionalVelocityPulseDirection.FORWARD, direction)
    }

    @Test
    fun `delayed tick preserves the full pulse duration`() {
        val sequence = DirectionalVelocityPulseSequence(
            direction = DirectionalVelocityPulseDirection.RIGHT,
            baselineNanos = 2L,
            pulseNanos = 5L,
            settleNanos = 7L,
        )

        sequence.start(0L)
        assertStep(
            sequence.advance(20L),
            DirectionalVelocityPulsePhase.PULSE,
            DirectionalVelocityPulseDirection.RIGHT,
            0.0,
            0.8,
            true,
        )
        assertNull(sequence.advance(24L))
        assertStep(
            sequence.advance(25L),
            DirectionalVelocityPulsePhase.SETTLE,
            DirectionalVelocityPulseDirection.RIGHT,
            0.0,
            0.0,
            false,
        )
    }

    @Test
    fun `invalid pulse parameters are rejected`() {
        assertThrows(IllegalArgumentException::class.java) {
            DirectionalVelocityPulseSequence(
                direction = DirectionalVelocityPulseDirection.FORWARD,
                speedMetersPerSecond = Double.NaN,
            )
        }
        assertThrows(IllegalArgumentException::class.java) {
            DirectionalVelocityPulseSequence(
                direction = DirectionalVelocityPulseDirection.FORWARD,
                pulseNanos = 0L,
            )
        }
    }

    private fun assertStep(
        actual: DirectionalVelocityPulseStep?,
        phase: DirectionalVelocityPulsePhase,
        direction: DirectionalVelocityPulseDirection,
        forwardMetersPerSecond: Double,
        rightMetersPerSecond: Double,
        markerWhite: Boolean,
    ) {
        requireNotNull(actual)
        assertEquals(phase, actual.phase)
        assertEquals(direction, actual.direction)
        assertEquals(forwardMetersPerSecond, actual.forwardMetersPerSecond, 0.0)
        assertEquals(rightMetersPerSecond, actual.rightMetersPerSecond, 0.0)
        if (markerWhite) {
            assertTrue(actual.markerWhite)
        } else {
            assertFalse(actual.markerWhite)
        }
    }

    private fun assertFixedStep(
        actual: FixedDirectionSpeedStep?,
        phase: FixedDirectionSpeedPhase,
        forwardMetersPerSecond: Double,
        markerWhite: Boolean,
    ) {
        requireNotNull(actual)
        assertEquals(phase, actual.phase)
        assertEquals(DirectionalVelocityPulseDirection.FORWARD, actual.direction)
        assertEquals(forwardMetersPerSecond, actual.forwardMetersPerSecond, 0.0)
        assertEquals(0.0, actual.rightMetersPerSecond, 0.0)
        assertEquals(markerWhite, actual.markerWhite)
    }
}
