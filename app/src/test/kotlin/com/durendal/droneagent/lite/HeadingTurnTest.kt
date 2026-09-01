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
    fun `angle heading follows directed path bearing at bounded rate`() {
        val controller = AngleHeadingController()

        assertEquals(
            161.0,
            checkNotNull(
                controller.update(
                    currentHeadingDegrees = 160.0,
                    relativePathBearingDegrees = 40.0,
                    pathTracking = true,
                    nowNanos = 1_000_000_000L,
                ),
            ),
            0.0,
        )
        assertEquals(
            162.0,
            checkNotNull(
                controller.update(
                    currentHeadingDegrees = 160.5,
                    relativePathBearingDegrees = 39.5,
                    pathTracking = true,
                    nowNanos = 1_100_000_000L,
                ),
            ),
            1e-9,
        )
    }

    @Test
    fun `angle heading crosses signed heading seam without jumping`() {
        val controller = AngleHeadingController()

        assertEquals(
            180.0,
            checkNotNull(
                controller.update(
                    currentHeadingDegrees = 179.0,
                    relativePathBearingDegrees = 20.0,
                    pathTracking = true,
                    nowNanos = 1_000_000_000L,
                ),
            ),
            0.0,
        )
        assertEquals(
            -179.0,
            checkNotNull(
                controller.update(
                    currentHeadingDegrees = 179.5,
                    relativePathBearingDegrees = 19.5,
                    pathTracking = true,
                    nowNanos = 1_100_000_000L,
                ),
            ),
            1e-9,
        )
    }

    @Test
    fun `angle heading cannot outrun aircraft by more than five degrees`() {
        val controller = AngleHeadingController()
        var command = 0.0

        repeat(20) { index ->
            command =
                checkNotNull(
                    controller.update(
                        currentHeadingDegrees = 0.0,
                        relativePathBearingDegrees = 90.0,
                        pathTracking = true,
                        nowNanos = 1_000_000_000L + index * 100_000_000L,
                    ),
                )
        }

        assertEquals(5.0, command, 0.0)
    }

    @Test
    fun `angle heading stops at current heading when path is unavailable`() {
        val controller = AngleHeadingController()
        controller.update(0.0, 45.0, pathTracking = true, nowNanos = 1_000_000_000L)

        assertEquals(
            0.4,
            checkNotNull(
                controller.update(
                    currentHeadingDegrees = 0.4,
                    relativePathBearingDegrees = null,
                    pathTracking = false,
                    nowNanos = 1_100_000_000L,
                ),
            ),
            0.0,
        )
        assertEquals(
            1.4,
            checkNotNull(
                controller.update(
                    currentHeadingDegrees = 0.4,
                    relativePathBearingDegrees = 45.0,
                    pathTracking = true,
                    nowNanos = 1_200_000_000L,
                ),
            ),
            1e-9,
        )
    }

    @Test
    fun `angle heading rejects invalid aircraft heading`() {
        val controller = AngleHeadingController()

        assertEquals(
            null,
            controller.update(
                currentHeadingDegrees = Double.NaN,
                relativePathBearingDegrees = 10.0,
                pathTracking = true,
                nowNanos = 1_000_000_000L,
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
