package com.durendal.droneagent.lite

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class TiltFlightControllerTest {
    @Test
    fun `circle tilts toward the left normal rather than the tangent`() {
        val controller = TiltFlightController(TiltFlightMode.CIRCLE)
        val startNanos = 1_000_000_000L
        controller.start(startNanos, headingDegrees = 0.0)
        val circleStart = startNanos + TiltFlightController.SETTLE_DURATION_NANOS + controller.startupDurationNanos

        val entry = controller.command(circleStart, headingDegrees = 0.0)
        assertTrue(entry.rollDegrees < 0.0)
        assertEquals(0.0, entry.pitchDegrees, 1e-12)
        val quarterWithRacingYaw = controller.command(circleStart + 1_750_000_000L, headingDegrees = -90.0)
        assertTrue(quarterWithRacingYaw.rollDegrees < 0.0)
        assertEquals(0.0, quarterWithRacingYaw.pitchDegrees, 1e-12)
        assertEquals(-90.0, quarterWithRacingYaw.yawHeadingDegrees, 1e-12)

        // With yaw still at the initial heading, the same world acceleration is backward.
        val quarterWithYawLag = controller.command(circleStart + 1_750_000_000L, headingDegrees = 0.0)
        assertEquals(0.0, quarterWithYawLag.rollDegrees, 1e-12)
        assertTrue(quarterWithYawLag.pitchDegrees > 0.0)
    }

    @Test
    fun `actual heading changes coordinates but never world acceleration`() {
        val controller = TiltFlightController(TiltFlightMode.CIRCLE)
        val startNanos = 1_000_000_000L
        val initialHeading = 170.0
        controller.start(startNanos, initialHeading)
        val circleStart = startNanos + TiltFlightController.SETTLE_DURATION_NANOS + controller.startupDurationNanos
        val acceleration = controller.targetSpeedMetersPerSecond * controller.targetSpeedMetersPerSecond / 0.75
        val initialHeadingRadians = Math.toRadians(initialHeading)

        for (phaseDegrees in 0..1075 step 5) {
            val elapsedNanos = (phaseDegrees / 360.0 * 7_000_000_000L).toLong()
            val actualPhase = elapsedNanos / 7_000_000_000.0 * 2.0 * PI
            val initialForward = -acceleration * sin(actualPhase)
            val initialRight = -acceleration * cos(actualPhase)
            val expectedX = initialForward * cos(initialHeadingRadians) - initialRight * sin(initialHeadingRadians)
            val expectedY = initialForward * sin(initialHeadingRadians) + initialRight * cos(initialHeadingRadians)
            for (heading in listOf(initialHeading, initialHeading - phaseDegrees, -179.0, 181.0, 721.0)) {
                val command = controller.command(circleStart + elapsedNanos, heading)
                val (worldX, worldY) = worldAcceleration(command, heading)
                assertEquals(expectedX, worldX, 1e-12)
                assertEquals(expectedY, worldY, 1e-12)
                assertTrue(command.rollDegrees.isFinite() && command.pitchDegrees.isFinite())
                assertTrue(hypot(command.rollDegrees, command.pitchDegrees) <= 5.0)
                assertTrue(command.yawHeadingDegrees in -180.0..180.0)
                // A wrapped yaw target must still point along the scheduled left-turn tangent.
                val yawRadians = Math.toRadians(command.yawHeadingDegrees)
                assertEquals(cos(initialHeadingRadians - actualPhase), cos(yawRadians), 1e-12)
                assertEquals(sin(initialHeadingRadians - actualPhase), sin(yawRadians), 1e-12)
            }
        }
    }

    @Test
    fun `integrating startup and all three circles closes a circle instead of a cycloid`() {
        val controller = TiltFlightController(TiltFlightMode.CIRCLE)
        val startNanos = 1_000_000_000L
        controller.start(startNanos, headingDegrees = 0.0)
        val startupStart = startNanos + TiltFlightController.SETTLE_DURATION_NANOS
        val startupSeconds = controller.startupDurationNanos / 1_000_000_000.0
        val startup = controller.command(startupStart + controller.startupDurationNanos / 2, headingDegrees = 0.0)
        val (startupAx, startupAy) = worldAcceleration(startup, headingDegrees = 0.0)
        var velocityX = startupAx * startupSeconds
        var velocityY = startupAy * startupSeconds
        var positionX = 0.5 * startupAx * startupSeconds * startupSeconds
        var positionY = 0.5 * startupAy * startupSeconds * startupSeconds
        val circleEntryX = positionX
        val radius = TiltFlightController.DIAMETER_METERS / 2.0
        assertEquals(controller.targetSpeedMetersPerSecond, velocityX, 1e-9)
        assertEquals(0.0, velocityY, 1e-12)

        val circleStart = startupStart + controller.startupDurationNanos
        val stepNanos = 10_000_000L
        val stepSeconds = stepNanos / 1_000_000_000.0
        for (step in 0 until 2100) {
            val midpointNanos = step * stepNanos + stepNanos / 2
            val phaseDegrees = midpointNanos / 7_000_000_000.0 * 360.0
            // Exercise body/world rotation while yaw lags the ideal racing tangent.
            val heading = -phaseDegrees + 25.0 * sin(Math.toRadians(phaseDegrees))
            val command = controller.command(circleStart + midpointNanos, heading)
            val (ax, ay) = worldAcceleration(command, heading)
            positionX += velocityX * stepSeconds + 0.5 * ax * stepSeconds * stepSeconds
            positionY += velocityY * stepSeconds + 0.5 * ay * stepSeconds * stepSeconds
            velocityX += ax * stepSeconds
            velocityY += ay * stepSeconds
            assertEquals(radius, hypot(positionX - circleEntryX, positionY + radius), 2e-4)
            assertEquals(controller.targetSpeedMetersPerSecond, hypot(velocityX, velocityY), 2e-5)
            if ((step + 1) % 700 == 0) {
                assertEquals(circleEntryX, positionX, 2e-4)
                assertEquals(0.0, positionY, 2e-4)
                assertEquals(controller.targetSpeedMetersPerSecond, velocityX, 1e-9)
                assertEquals(0.0, velocityY, 1e-9)
            }
        }
    }

    @Test
    fun `straight tilt has exact settling tilt and velocity brake boundaries`() {
        val controller = TiltFlightController(TiltFlightMode.STRAIGHT)
        val startNanos = 1_000_000_000L
        val initial = controller.start(startNanos, headingDegrees = 540.0)
        assertVelocityZero(initial, TiltFlightPhase.SETTLE)
        assertEquals(startNanos + TiltFlightController.SETTLE_DURATION_NANOS, initial.phaseEndsAtNanos)
        val tiltStart = initial.phaseEndsAtNanos
        assertVelocityZero(controller.command(tiltStart - 1, 180.0), TiltFlightPhase.SETTLE)
        val tilt = controller.command(tiltStart, 180.0)
        assertEquals(TiltFlightPhase.STRAIGHT, tilt.phase)
        assertTrue(tilt.usesAngle)
        assertEquals(-3.0, tilt.pitchDegrees, 0.0)
        assertEquals(0.0, tilt.rollDegrees, 0.0)
        assertEquals(tiltStart + TiltFlightController.STRAIGHT_DURATION_NANOS, tilt.phaseEndsAtNanos)
        assertTrue(controller.command(tilt.phaseEndsAtNanos - 1, 180.0).usesAngle)
        val brake = controller.command(tilt.phaseEndsAtNanos, 180.0)
        assertVelocityZero(brake, TiltFlightPhase.BRAKE)
        assertEquals(tilt.phaseEndsAtNanos + TiltFlightController.BRAKE_DURATION_NANOS, brake.phaseEndsAtNanos)
        assertVelocityZero(controller.command(brake.phaseEndsAtNanos - 1, 180.0), TiltFlightPhase.BRAKE)
        assertFinishedForever(controller, startNanos, 180.0)
    }

    @Test
    fun `circle phase and lap boundaries never extend the angle schedule`() {
        val controller = TiltFlightController(TiltFlightMode.CIRCLE)
        val startNanos = 1_000_000_000L
        val settle = controller.start(startNanos, headingDegrees = -170.0)
        assertVelocityZero(settle, TiltFlightPhase.SETTLE)
        assertVelocityZero(controller.command(settle.phaseEndsAtNanos - 1, -170.0), TiltFlightPhase.SETTLE)
        val startup = controller.command(settle.phaseEndsAtNanos, -170.0)
        assertEquals(TiltFlightPhase.STARTUP, startup.phase)
        assertTrue(startup.usesAngle)
        assertEquals(-3.0, startup.pitchDegrees, 0.0)
        assertEquals(0.0, startup.rollDegrees, 0.0)
        assertEquals(settle.phaseEndsAtNanos + controller.startupDurationNanos, startup.phaseEndsAtNanos)
        assertEquals(TiltFlightPhase.STARTUP, controller.command(startup.phaseEndsAtNanos - 1, -170.0).phase)
        val circle = controller.command(startup.phaseEndsAtNanos, -170.0)
        assertEquals(TiltFlightPhase.CIRCLE, circle.phase)
        assertEquals(startup.phaseEndsAtNanos + 21_000_000_000L, circle.phaseEndsAtNanos)
        for (lapBoundary in 1..2) {
            val boundaryNanos = startup.phaseEndsAtNanos + lapBoundary * 7_000_000_000L
            assertEquals(lapBoundary, controller.command(boundaryNanos - 1, -170.0).lap)
            val nextLap = controller.command(boundaryNanos, -170.0)
            assertEquals(lapBoundary + 1, nextLap.lap)
            assertEquals(lapBoundary * 360.0, nextLap.progressDegrees, 0.0)
        }
        assertTrue(controller.command(circle.phaseEndsAtNanos - 1, -170.0).usesAngle)
        val brake = controller.command(circle.phaseEndsAtNanos, -170.0)
        assertVelocityZero(brake, TiltFlightPhase.BRAKE)
        assertEquals(3, brake.lap)
        assertEquals(1080.0, brake.progressDegrees, 0.0)
        assertEquals(circle.phaseEndsAtNanos + TiltFlightController.BRAKE_DURATION_NANOS, brake.phaseEndsAtNanos)
        assertVelocityZero(controller.command(brake.phaseEndsAtNanos - 1, -170.0), TiltFlightPhase.BRAKE)
        assertFinishedForever(controller, startNanos, -170.0)
    }

    @Test
    fun `skating holds initial yaw while inward acceleration rotates through three laps`() {
        val controller = TiltFlightController(TiltFlightMode.SKATING_CIRCLE)
        val startNanos = 1_000_000_000L
        val initialHeading = 179.0
        val settle = controller.start(startNanos, initialHeading)
        assertVelocityZero(settle, TiltFlightPhase.SETTLE)
        val startup = controller.command(settle.phaseEndsAtNanos, initialHeading)
        assertEquals(TiltFlightPhase.STARTUP, startup.phase)
        assertEquals(initialHeading, startup.yawHeadingDegrees, 0.0)
        assertEquals(-3.0, startup.pitchDegrees, 0.0)
        val circleStart = startup.phaseEndsAtNanos
        val acceleration = controller.targetSpeedMetersPerSecond * controller.targetSpeedMetersPerSecond / 0.75
        for (quarter in 0 until 12) {
            // Include real-heading error across ±180°: hold the initial target, but
            // transform acceleration using the actual heading rather than the target.
            val actualHeading = if (quarter % 2 == 0) -179.0 else 176.0
            val command = controller.command(circleStart + quarter * 1_750_000_000L, actualHeading)
            assertEquals(TiltFlightPhase.CIRCLE, command.phase)
            assertEquals(initialHeading, command.yawHeadingDegrees, 0.0)
            assertEquals(quarter / 4 + 1, command.lap)
            val tangent = Math.toRadians(initialHeading - quarter * 90.0)
            val (worldX, worldY) = worldAcceleration(command, actualHeading)
            assertEquals(acceleration * sin(tangent), worldX, 1e-12)
            assertEquals(-acceleration * cos(tangent), worldY, 1e-12)
            assertTrue(hypot(command.rollDegrees, command.pitchDegrees) <= 5.0)
        }
        val brake = controller.command(circleStart + 21_000_000_000L, initialHeading)
        assertVelocityZero(brake, TiltFlightPhase.BRAKE)
        assertEquals(initialHeading, brake.yawHeadingDegrees, 0.0)
        assertEquals(3, brake.lap)
        assertEquals(1080.0, brake.progressDegrees, 0.0)
        assertFinishedForever(controller, startNanos, initialHeading)
    }

    private fun assertFinishedForever(controller: TiltFlightController, startNanos: Long, heading: Double) {
        val endNanos = startNanos + controller.scheduledDurationNanos
        for (nowNanos in listOf(endNanos, endNanos + 1, Long.MAX_VALUE)) {
            val command = controller.command(nowNanos, heading)
            assertVelocityZero(command, TiltFlightPhase.COMPLETE)
            assertTrue(command.completed)
            assertEquals(endNanos, command.phaseEndsAtNanos)
        }
    }

    private fun assertVelocityZero(command: TiltFlightCommand, phase: TiltFlightPhase) {
        assertEquals(phase, command.phase)
        assertFalse(command.usesAngle)
        assertEquals(0.0, command.rollDegrees, 0.0)
        assertEquals(0.0, command.pitchDegrees, 0.0)
        assertEquals(phase == TiltFlightPhase.COMPLETE, command.completed)
    }

    private fun worldAcceleration(command: TiltFlightCommand, headingDegrees: Double): Pair<Double, Double> {
        // Invert the specified Euler mapping, independently of controller internals.
        val gravity = 9.80665
        val forward = -gravity * tan(Math.toRadians(command.pitchDegrees))
        val right = sqrt(gravity * gravity + forward * forward) * tan(Math.toRadians(command.rollDegrees))
        val heading = Math.toRadians(headingDegrees)
        return Pair(forward * cos(heading) - right * sin(heading), forward * sin(heading) + right * cos(heading))
    }
}
