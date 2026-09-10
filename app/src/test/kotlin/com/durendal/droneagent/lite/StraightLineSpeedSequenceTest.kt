package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test

class StraightLineSpeedSequenceTest {
    @Test
    fun `zero baseline starts forward immediately and can stop on the same tick`() {
        val sequence = StraightLineSpeedSequence(
            direction = DirectionalVelocityPulseDirection.FORWARD,
            speedMetersPerSecond = 0.3,
            maximumCruiseNanos = 10L,
            baselineNanos = 0L,
            brakingNanos = 3L,
            maximumTickGapNanos = 20L,
        )
        val first = sequence.start(100L)
        assertEquals(StraightLineSpeedPhase.CRUISE, first.phase)
        assertEquals(0.3, first.forwardMetersPerSecond, 0.0)
        assertEquals(0.0, first.rightMetersPerSecond, 0.0)
        assertEquals(110L, sequence.commandDeadlineNanos)

        assertNeutral(sequence.requestStop(100L), StraightLineSpeedPhase.BRAKING)
        assertEquals(StraightLineStopReason.OPERATOR, sequence.stopReason)
        assertNull(sequence.requestStop(101L))
        assertNeutral(sequence.advance(103L), StraightLineSpeedPhase.COMPLETE)
        assertNull(sequence.advance(104L))
    }

    @Test
    fun `zero baseline retains the automatic cruise time limit`() {
        val sequence = sequence(baselineNanos = 0L)
        sequence.start(100L)
        assertNull(sequence.advance(109L))
        assertNeutral(sequence.advance(110L), StraightLineSpeedPhase.BRAKING)
        assertEquals(StraightLineStopReason.TIME_LIMIT, sequence.stopReason)
        assertEquals(0L, sequence.commandDeadlineNanos)
        assertNeutral(sequence.advance(113L), StraightLineSpeedPhase.COMPLETE)
    }

    @Test
    fun `stopping at baseline deadline prevents cruise and repeated stops do not extend braking`() {
        val sequence = sequence()
        assertNeutral(sequence.start(10L), StraightLineSpeedPhase.BASELINE)
        assertEquals(0L, sequence.commandDeadlineNanos)

        assertNeutral(sequence.requestStop(12L), StraightLineSpeedPhase.BRAKING)
        assertEquals(StraightLineStopReason.CANCELLED_BEFORE_CRUISE, sequence.stopReason)
        assertNull(sequence.requestStop(14L))
        assertNull(sequence.advance(14L))
        assertNeutral(sequence.advance(15L), StraightLineSpeedPhase.COMPLETE)
        assertNull(sequence.advance(100L))
        assertNull(sequence.requestStop(100L))
        assertThrows(IllegalStateException::class.java) { sequence.start(100L) }
    }

    @Test
    fun `absolute cruise deadline starts at emission and a late timeout preserves real braking interval`() {
        val sequence = sequence(maximumTickGapNanos = 20L)
        sequence.start(0L)
        assertEquals(StraightLineSpeedPhase.CRUISE, sequence.advance(3L)?.phase)
        assertEquals(13L, sequence.commandDeadlineNanos)
        assertNull(sequence.advance(12L))
        assertEquals(13L, sequence.commandDeadlineNanos)

        assertNeutral(sequence.advance(100L), StraightLineSpeedPhase.BRAKING)
        assertEquals(StraightLineStopReason.TIME_LIMIT, sequence.stopReason)
        assertEquals(0L, sequence.commandDeadlineNanos)
        assertNull(sequence.requestStop(100L))
        assertNull(sequence.advance(102L))
        assertNeutral(sequence.advance(103L), StraightLineSpeedPhase.COMPLETE)
        assertEquals(StraightLineStopReason.TIME_LIMIT, sequence.stopReason)
        assertNull(sequence.advance(200L))
    }

    @Test
    fun `operator stop at cruise deadline remains a timeout`() {
        val sequence = sequence()
        sequence.start(0L)
        sequence.advance(2L)
        assertNeutral(sequence.requestStop(12L), StraightLineSpeedPhase.BRAKING)
        assertEquals(StraightLineStopReason.TIME_LIMIT, sequence.stopReason)
        assertNeutral(sequence.advance(15L), StraightLineSpeedPhase.COMPLETE)
    }

    @Test
    fun `stalled baseline never emits a cruise command`() {
        val sequence = sequence(maximumTickGapNanos = 5L)
        sequence.start(10L)
        assertNeutral(sequence.advance(15L), StraightLineSpeedPhase.BRAKING)
        assertEquals(StraightLineStopReason.CONTROL_STALL, sequence.stopReason)
        assertEquals(0L, sequence.commandDeadlineNanos)
        assertNull(sequence.advance(17L))
        assertNeutral(sequence.advance(18L), StraightLineSpeedPhase.COMPLETE)
        assertNull(sequence.advance(100L))
    }

    @Test
    fun `cruise tick gap expires at the sender deadline and never resumes`() {
        val sequence = sequence(maximumTickGapNanos = 5L)
        sequence.start(0L)
        sequence.advance(2L)
        assertNeutral(sequence.advance(7L), StraightLineSpeedPhase.BRAKING)
        assertEquals(StraightLineStopReason.CONTROL_STALL, sequence.stopReason)
        assertEquals(0L, sequence.commandDeadlineNanos)
        assertNull(sequence.requestStop(8L))
        assertNull(sequence.advance(9L))
        assertNeutral(sequence.advance(10L), StraightLineSpeedPhase.COMPLETE)
        assertEquals(StraightLineStopReason.CONTROL_STALL, sequence.stopReason)
        assertNull(sequence.advance(100L))
    }

    @Test
    fun `each selected direction holds its sign until neutral and never automatically reverses`() {
        listOf(DirectionalVelocityPulseDirection.FORWARD, DirectionalVelocityPulseDirection.BACKWARD)
            .forEach { direction ->
                val sequence = sequence(direction = direction)
                assertNeutral(sequence.start(0L), StraightLineSpeedPhase.BASELINE)
                val cruise = requireNotNull(sequence.advance(2L))
                assertEquals(direction, cruise.direction)
                assertEquals(StraightLineSpeedPhase.CRUISE, cruise.phase)
                assertEquals(0.5 * direction.forwardSign, cruise.forwardMetersPerSecond, 0.0)
                assertEquals(0.0, cruise.rightMetersPerSecond, 0.0)
                assertTrue(cruise.markerWhite)
                assertFalse(cruise.complete)
                assertNull(sequence.advance(3L))
                assertNeutral(sequence.requestStop(4L), StraightLineSpeedPhase.BRAKING)
                assertNull(sequence.advance(6L))
                assertNeutral(sequence.advance(7L), StraightLineSpeedPhase.COMPLETE)
                assertNull(sequence.advance(100L))
            }
    }

    private fun sequence(
        direction: DirectionalVelocityPulseDirection = DirectionalVelocityPulseDirection.FORWARD,
        maximumTickGapNanos: Long = 20L,
        baselineNanos: Long = 2L,
    ) = StraightLineSpeedSequence(
        direction = direction,
        speedMetersPerSecond = 0.5,
        maximumCruiseNanos = 10L,
        baselineNanos = baselineNanos,
        brakingNanos = 3L,
        maximumTickGapNanos = maximumTickGapNanos,
    )

    private fun assertNeutral(actual: StraightLineSpeedStep?, phase: StraightLineSpeedPhase) {
        requireNotNull(actual)
        assertEquals(phase, actual.phase)
        assertEquals(0.0, actual.forwardMetersPerSecond, 0.0)
        assertEquals(0.0, actual.rightMetersPerSecond, 0.0)
        assertFalse(actual.markerWhite)
        assertEquals(phase == StraightLineSpeedPhase.COMPLETE, actual.complete)
    }
}
