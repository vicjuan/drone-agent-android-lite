package com.durendal.droneagent.lite

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test

class FeedbackCircleControllerTest {
    @Test
    fun `circle starts with the requested radius speed and tangent`() {
        val controller = FeedbackCircleController()
        val startNanos = 1_000_000_000L

        val command =
            controller.start(
                nowNanos = startNanos,
                headingDegrees = 30.0,
                velocityX = 0.0,
                velocityY = 0.0,
            )

        assertEquals(PI * 2.0 / 7.5, controller.tangentialSpeedMetersPerSecond, 1e-12)
        assertEquals(48.0, controller.angularRateDegreesPerSecond, 0.0)
        assertEquals(controller.tangentialSpeedMetersPerSecond, command.forwardMetersPerSecond, 1e-12)
        assertEquals(0.0, command.rightMetersPerSecond, 1e-12)
        assertEquals(30.0, command.tangentHeadingDegrees, 0.0)
        assertEquals(0.0, command.positionErrorMeters, 0.0)
    }

    @Test
    fun `ideal measured motion stays on the circle and racing yaw follows its tangent`() {
        val controller = FeedbackCircleController()
        val startNanos = 2_000_000_000L
        val speed = controller.tangentialSpeedMetersPerSecond
        controller.start(startNanos, 0.0, speed, 0.0)

        var command: FeedbackCircleCommand? = null
        for (step in 1..75) {
            val elapsedSeconds = step * 0.025
            val phase = 2.0 * PI * elapsedSeconds / FeedbackCircleController.SECONDS_PER_LAP
            command =
                controller.command(
                    nowNanos = startNanos + step * 25_000_000L,
                    headingDegrees = -Math.toDegrees(phase),
                    velocityX = speed * cos(phase),
                    velocityY = -speed * sin(phase),
                )
        }

        val quarterCircle = checkNotNull(command)
        assertEquals(-90.0, quarterCircle.tangentHeadingDegrees, 1e-9)
        assertEquals(speed, quarterCircle.forwardMetersPerSecond, 0.02)
        assertEquals(0.0, quarterCircle.rightMetersPerSecond, 0.02)
        assertTrue(quarterCircle.positionErrorMeters < 0.01)
    }

    @Test
    fun `position feedback commands recovery when aircraft falls behind`() {
        val controller = FeedbackCircleController()
        val startNanos = 3_000_000_000L
        controller.start(startNanos, 0.0, 0.0, 0.0)

        val command =
            controller.command(
                nowNanos = startNanos + 1_000_000_000L,
                headingDegrees = 0.0,
                velocityX = 0.0,
                velocityY = 0.0,
            )

        val phase = 2.0 * PI / FeedbackCircleController.SECONDS_PER_LAP
        val feedForward = controller.tangentialSpeedMetersPerSecond * cos(phase)
        assertTrue(command.forwardMetersPerSecond > feedForward)
        assertTrue(command.positionErrorMeters > 0.75)
    }

    @Test
    fun `three physically followed laps close at the launch point`() {
        val controller = FeedbackCircleController()
        val startNanos = 4_000_000_000L
        val speed = controller.tangentialSpeedMetersPerSecond
        controller.start(startNanos, 0.0, speed, 0.0)

        var command: FeedbackCircleCommand? = null
        val finalStep =
            (FeedbackCircleController.SECONDS_PER_LAP * FeedbackCircleController.LAP_COUNT / 0.025)
                .toInt()
        for (step in 1..finalStep) {
            val elapsedSeconds = step * 0.025
            val scheduledComplete =
                elapsedSeconds >=
                    FeedbackCircleController.SECONDS_PER_LAP * FeedbackCircleController.LAP_COUNT
            val phase = 2.0 * PI * elapsedSeconds / FeedbackCircleController.SECONDS_PER_LAP
            command =
                controller.command(
                    nowNanos = startNanos + step * 25_000_000L,
                    headingDegrees = 0.0,
                    velocityX = if (scheduledComplete) 0.0 else speed * cos(phase),
                    velocityY = if (scheduledComplete) 0.0 else -speed * sin(phase),
                )
        }

        val complete = checkNotNull(command)
        assertEquals(3, complete.lap)
        assertEquals(1_080.0, complete.totalProgressDegrees, 0.0)
        assertTrue(complete.positionErrorMeters < FeedbackCircleController.COMPLETION_POSITION_ERROR_METERS)
        assertTrue(complete.completed)
        assertFalse(complete.closing)
        assertEquals(0.0, complete.forwardMetersPerSecond, 0.0)
        assertEquals(0.0, complete.rightMetersPerSecond, 0.0)
    }

    @Test
    fun `controller rejects a second start so origin cannot reset in flight`() {
        val controller = FeedbackCircleController()
        controller.start(1_000_000_000L, 0.0, 0.0, 0.0)

        assertThrows(IllegalStateException::class.java) {
            controller.start(2_000_000_000L, 0.0, 0.0, 0.0)
        }
    }

    @Test
    fun `active velocity reads stay valid without listener value changes`() {
        val state =
            FeedbackVelocityReadState(
                pollIntervalNanos = 100_000_000L,
                timeoutNanos = 500_000_000L,
            )
        val startedAtNanos = 1_000_000_000L

        assertTrue(state.beginRequest(startedAtNanos))
        state.recordSuccess(startedAtNanos + 20_000_000L)

        assertFalse(state.beginRequest(startedAtNanos + 50_000_000L))
        assertTrue(state.beginRequest(startedAtNanos + 100_000_000L))
        assertFalse(state.hasExpired(startedAtNanos + 500_000_000L))
    }

    @Test
    fun `pending velocity read cannot hide a lost data source`() {
        val state =
            FeedbackVelocityReadState(
                pollIntervalNanos = 100_000_000L,
                timeoutNanos = 500_000_000L,
            )
        val startedAtNanos = 2_000_000_000L

        assertTrue(state.beginRequest(startedAtNanos))
        state.recordSuccess(startedAtNanos)
        assertTrue(state.beginRequest(startedAtNanos + 100_000_000L))

        assertTrue(state.hasExpired(startedAtNanos + 500_000_001L))
    }

    @Test
    fun `failed velocity read may retry at the next poll interval`() {
        val state =
            FeedbackVelocityReadState(
                pollIntervalNanos = 100_000_000L,
                timeoutNanos = 500_000_000L,
            )
        val startedAtNanos = 3_000_000_000L

        assertTrue(state.beginRequest(startedAtNanos))
        state.recordFailure()

        assertFalse(state.beginRequest(startedAtNanos + 99_999_999L))
        assertTrue(state.beginRequest(startedAtNanos + 100_000_000L))
        assertTrue(state.hasExpired(startedAtNanos + 100_000_000L))
    }
    @Test
    fun `armed log formats decimal settings before integer lap count`() {
        val message =
            feedbackCircleArmedLogMessage(
                mode = "RACING",
                diameterMeters = 2.0,
                secondsPerLap = 7.5,
                laps = 3,
                speedMetersPerSecond = 0.838,
                angularRateDegreesPerSecond = 48.0,
                headingDegrees = -12.5,
            )

        assertEquals(
            "feedback circle armed mode=RACING diameter=2.00 secondsPerLap=7.5 " +
                "laps=3 speed=0.838 angularRate=48.0 heading=-12.5",
            message,
        )
    }

    @Test
    fun `large tracking error remains observable without invalidating the command`() {
        val controller = FeedbackCircleController()
        val startNanos = 5_000_000_000L
        controller.start(startNanos, 0.0, 0.0, 0.0)

        val command =
            controller.command(
                nowNanos = startNanos + 2_500_000_000L,
                headingDegrees = 0.0,
                velocityX = 0.0,
                velocityY = 0.0,
            )

        assertTrue(command.positionErrorMeters > 0.90)
        assertTrue(
            kotlin.math.hypot(
                command.forwardMetersPerSecond,
                command.rightMetersPerSecond,
            ) <= FeedbackCircleController.MAX_COMMAND_SPEED_METERS_PER_SECOND,
        )
        assertFalse(command.completed)
        assertFalse(command.closing)
    }

}
