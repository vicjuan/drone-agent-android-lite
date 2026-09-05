package com.durendal.droneagent.lite

import kotlin.math.PI
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test

class MathematicalCircleControllerTest {
    @Test
    fun `circle starts with the requested radius speed and tangent`() {
        val controller = MathematicalCircleController()

        val command = controller.start(1_000_000_000L, headingDegrees = 30.0)

        assertEquals(PI * 1.5 / 7.5, controller.tangentialSpeedMetersPerSecond, 1e-12)
        assertEquals(0.75, controller.radiusMeters, 0.0)
        assertEquals(48.0, controller.angularRateDegreesPerSecond, 0.0)
        assertEquals(controller.tangentialSpeedMetersPerSecond, command.forwardMetersPerSecond, 1e-12)
        assertEquals(0.0, command.rightMetersPerSecond, 1e-12)
        assertEquals(30.0, command.tangentHeadingDegrees, 0.0)
        assertFalse(command.completed)
    }

    @Test
    fun `racing schedule is faster without changing the skating default`() {
        val skating = MathematicalCircleController()
        val racing = MathematicalCircleController(
            secondsPerLap = MathematicalCircleController.RACING_SECONDS_PER_LAP,
        )

        assertEquals(7.5, skating.secondsPerLap, 0.0)
        assertEquals(PI * 1.5 / 7.5, skating.tangentialSpeedMetersPerSecond, 1e-12)
        assertEquals(6.75, racing.secondsPerLap, 0.0)
        assertEquals(PI * 1.5 / 6.75, racing.tangentialSpeedMetersPerSecond, 1e-12)
        assertEquals(53.333333333333336, racing.angularRateDegreesPerSecond, 1e-12)
        assertEquals(20_250_000_000L, racing.scheduledDurationNanos)
    }

    @Test
    fun `elapsed time alone produces the quarter circle tangent`() {
        val controller = MathematicalCircleController()
        val startNanos = 2_000_000_000L
        controller.start(startNanos, headingDegrees = 0.0)

        val command =
            controller.command(
                nowNanos = startNanos + 1_875_000_000L,
                headingDegrees = 0.0,
            )

        assertEquals(90.0, command.totalProgressDegrees, 1e-9)
        assertEquals(0.0, command.forwardMetersPerSecond, 1e-9)
        assertEquals(-controller.tangentialSpeedMetersPerSecond, command.rightMetersPerSecond, 1e-9)
        assertEquals(-90.0, command.tangentHeadingDegrees, 1e-9)
    }

    @Test
    fun `racing heading keeps the mathematical tangent command forward`() {
        val controller = MathematicalCircleController()
        val startNanos = 3_000_000_000L
        controller.start(startNanos, headingDegrees = 0.0)

        val command =
            controller.command(
                nowNanos = startNanos + 1_875_000_000L,
                headingDegrees = -90.0,
            )

        assertEquals(controller.tangentialSpeedMetersPerSecond, command.forwardMetersPerSecond, 1e-9)
        assertEquals(0.0, command.rightMetersPerSecond, 1e-9)
    }

    @Test
    fun `three scheduled laps stop exactly from elapsed time`() {
        val controller = MathematicalCircleController()
        val startNanos = 4_000_000_000L
        controller.start(startNanos, headingDegrees = 0.0)

        val command =
            controller.command(
                nowNanos = startNanos + controller.scheduledDurationNanos,
                headingDegrees = 0.0,
            )

        assertEquals(3, command.lap)
        assertEquals(1_080.0, command.totalProgressDegrees, 0.0)
        assertTrue(command.completed)
        assertEquals(0.0, command.forwardMetersPerSecond, 0.0)
        assertEquals(0.0, command.rightMetersPerSecond, 0.0)
    }

    @Test
    fun `controller rejects a second start`() {
        val controller = MathematicalCircleController()
        controller.start(1_000_000_000L, headingDegrees = 0.0)

        assertThrows(IllegalStateException::class.java) {
            controller.start(2_000_000_000L, headingDegrees = 0.0)
        }
    }

    @Test
    fun `armed log identifies the mathematical schedule`() {
        val message =
            mathematicalCircleArmedLogMessage(
                mode = "RACING",
                diameterMeters = 2.0,
                secondsPerLap = 7.5,
                laps = 3,
                speedMetersPerSecond = 0.838,
                angularRateDegreesPerSecond = 48.0,
                headingDegrees = -12.5,
            )

        assertEquals(
            "mathematical circle armed mode=RACING diameter=2.00 secondsPerLap=7.5 " +
                "laps=3 speed=0.838 angularRate=48.0 heading=-12.5",
            message,
        )
    }
}
