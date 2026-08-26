package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
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
    fun `target angle can close a clockwise quarter turn across heading boundary`() {
        val turn = HeadingTurn(170.0, targetDegrees = 90.0)

        assertEquals(70.0, turn.update(-170.0), 0.0)
        assertEquals(0.0, turn.update(-100.0), 0.0)
        assertEquals(90.0, turn.progressDegrees, 0.0)
    }

    @Test
    fun `full turn accumulates 360 degrees across both heading seams`() {
        val turn = HeadingTurn(170.0, targetDegrees = 360.0)

        assertEquals(340.0, turn.update(-170.0), 0.0)
        assertEquals(180.0, turn.update(-10.0), 0.0)
        assertEquals(20.0, turn.update(150.0), 0.0)
        assertEquals(0.0, turn.update(170.0), 0.0)
        assertEquals(360.0, turn.progressDegrees, 0.0)
    }

    @Test
    fun `heading lead remains in MSDK signed angle range across both seams`() {
        assertEquals(-170.0, wrapToSignedHeading(190.0), 0.0)
        assertEquals(170.0, wrapToSignedHeading(-190.0), 0.0)
        assertEquals(180.0, wrapToSignedHeading(180.0), 0.0)
        assertEquals(-180.0, wrapToSignedHeading(-180.0), 0.0)
    }

    @Test
    fun `yaw rate maps proportionally to bounded heading lead`() {
        assertEquals(
            -170.0,
            checkNotNull(
                headingTargetForYawRate(
                    currentHeadingDegrees = 160.0,
                    yawRateDegreesPerSecond = 25.0,
                    maximumYawRateDegreesPerSecond = 50.0,
                    maximumHeadingLeadDegrees = 60.0,
                ),
            ),
            0.0,
        )
        assertEquals(
            130.0,
            checkNotNull(
                headingTargetForYawRate(
                    currentHeadingDegrees = 160.0,
                    yawRateDegreesPerSecond = -25.0,
                    maximumYawRateDegreesPerSecond = 50.0,
                    maximumHeadingLeadDegrees = 60.0,
                ),
            ),
            0.0,
        )
        assertEquals(
            -140.0,
            checkNotNull(
                headingTargetForYawRate(
                    currentHeadingDegrees = 160.0,
                    yawRateDegreesPerSecond = 100.0,
                    maximumYawRateDegreesPerSecond = 50.0,
                    maximumHeadingLeadDegrees = 60.0,
                ),
            ),
            0.0,
        )
        assertNull(
            headingTargetForYawRate(
                currentHeadingDegrees = Double.NaN,
                yawRateDegreesPerSecond = 25.0,
                maximumYawRateDegreesPerSecond = 50.0,
                maximumHeadingLeadDegrees = 60.0,
            ),
        )
    }


    @Test
    fun `opposite drift does not create directed progress`() {
        val turn = HeadingTurn(0.0)

        assertEquals(180.0, turn.update(-3.0), 0.0)
        assertEquals(178.0, turn.update(2.0), 0.0)
    }

    @Test
    fun `quarter arc ramps forward speed and derives yaw from fixed radius`() {
        val controller = QuarterArcController(initialHeadingDegrees = 0.0)
        var nowNanos = 1_000_000_000L
        var command = controller.command(nowNanos)

        repeat(119) {
            nowNanos += 50_000_000L
            command = controller.command(nowNanos)
        }

        assertEquals(0.12, command.forwardSpeedMetersPerSecond, 1e-12)
        assertEquals(
            Math.toDegrees(0.12 / 0.75),
            command.yawRateDegreesPerSecond,
            1e-12,
        )
    }

    @Test
    fun `quarter arc heading closes at 90 degrees across heading boundary`() {
        val controller = QuarterArcController(initialHeadingDegrees = 170.0)

        controller.updateHeading(-170.0)
        assertEquals(20.0, controller.progressDegrees, 0.0)
        assertEquals(70.0, controller.remainingDegrees, 0.0)

        controller.updateHeading(-100.0)
        assertEquals(90.0, controller.progressDegrees, 0.0)
        assertEquals(0.0, controller.remainingDegrees, 0.0)
    }

    @Test
    fun `quarter arc speed cannot jump after a delayed control tick`() {
        val controller = QuarterArcController(initialHeadingDegrees = 0.0)
        controller.command(1_000_000_000L)

        val command = controller.command(11_000_000_000L)

        assertEquals(0.003, command.forwardSpeedMetersPerSecond, 1e-12)
    }
}
