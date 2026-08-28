package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class FixedHeadingLapControllerTest {
    @Test
    fun `omni measurement follows a vertical path in the commanded direction`() {
        val measurement = OmniCenterline.measure(
            path = path(
                xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
                ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
            ),
            heightMeters = 1.0,
            travelDirectionDegrees = 0.0,
        )

        assertNotNull(measurement)
        assertEquals(0.0, checkNotNull(measurement).tangentDegrees, 1.0)
        assertEquals(0.0, measurement.lateralOffsetMeters, 1e-6)
        assertEquals(0.0, measurement.curvaturePerMeter, 1e-6)
    }

    @Test
    fun `omni measurement selects the reverse tangent without a 180 degree jump`() {
        val measurement = OmniCenterline.measure(
            path = path(
                xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
                ys = floatArrayOf(0.20f, 0.35f, 0.50f, 0.65f, 0.80f, 0.96f),
            ),
            heightMeters = 1.0,
            travelDirectionDegrees = 0.0,
        )

        assertNotNull(measurement)
        assertEquals(0.0, checkNotNull(measurement).tangentDegrees, 1.0)
    }

    @Test
    fun `omni measurement projects rightward travel from the camera centre`() {
        val measurement = OmniCenterline.measure(
            path = path(
                xs = floatArrayOf(0.20f, 0.34f, 0.48f, 0.62f, 0.76f, 0.95f),
                ys = floatArrayOf(0.50f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f),
            ),
            heightMeters = 1.0,
            travelDirectionDegrees = 90.0,
        )

        assertNotNull(measurement)
        assertEquals(90.0, checkNotNull(measurement).tangentDegrees, 1.0)
        assertEquals(0.0, measurement.lateralOffsetMeters, 1e-6)
        assertEquals(0.0, measurement.curvaturePerMeter, 1e-6)
        assertTrue(measurement.lookaheadXFraction > 0.45)
        assertEquals(0.5, measurement.lookaheadYFraction, 1e-6)
        assertEquals(0.5, measurement.projectionXFraction, 1e-6)
        assertEquals(0.5, measurement.projectionYFraction, 1e-6)
    }

    @Test
    fun `omni measurement keeps tracking a U turn with less than full fast lookahead`() {
        val measurement = OmniCenterline.measure(
            path = path(
                xs = floatArrayOf(
                    0.10f, 0.10f, 0.10f, 0.12f, 0.18f, 0.28f, 0.38f,
                    0.46f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f,
                ),
                ys = floatArrayOf(
                    0.10f, 0.40f, 0.70f, 0.84f, 0.92f, 0.96f, 0.96f,
                    0.92f, 0.84f, 0.65f, 0.50f, 0.35f, 0.20f,
                ),
            ),
            heightMeters = 1.0,
            travelDirectionDegrees = 0.0,
            lookaheadMeters = FixedHeadingLapController.FAST_LOOKAHEAD_METERS,
        )

        val tracked = checkNotNull(measurement)
        assertTrue(tracked.lookaheadDistanceMeters >= 0.15)
        assertTrue(
            tracked.lookaheadDistanceMeters <
                FixedHeadingLapController.FAST_LOOKAHEAD_METERS,
        )
    }

    @Test
    fun `omni measurement stays on the previous arm of an ambiguous U turn`() {
        val previous = OmniCenterline.measure(
            path = path(
                xs = floatArrayOf(0.60f, 0.60f, 0.60f, 0.60f, 0.60f, 0.60f),
                ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
            ),
            heightMeters = 1.0,
            travelDirectionDegrees = 0.0,
        )
        val measurement = OmniCenterline.measure(
            path = path(
                xs = floatArrayOf(
                    0.40f, 0.40f, 0.40f, 0.40f, 0.40f, 0.40f, 0.50f,
                    0.60f, 0.60f, 0.60f, 0.60f, 0.60f, 0.60f,
                ),
                ys = floatArrayOf(
                    0.10f, 0.30f, 0.50f, 0.70f, 0.90f, 0.96f, 0.98f,
                    0.96f, 0.90f, 0.70f, 0.50f, 0.30f, 0.10f,
                ),
            ),
            heightMeters = 1.0,
            travelDirectionDegrees = 0.0,
            previousMeasurement = checkNotNull(previous),
        )

        assertTrue(checkNotNull(measurement).projectionXFraction > 0.55)
    }

    @Test
    fun `fast fixed heading slows for a shortened visible lookahead`() {
        val controller = FixedHeadingLapController()
        val shortenedPath = path(
            xs = floatArrayOf(0.50f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.25f),
        )
        controller.start(1L)
        var decision = FixedHeadingLapDecision(FixedHeadingLapPhase.ACQUIRING)
        repeat(20) { index ->
            val now = 100_000_001L + index * 100_000_000L
            controller.observe(shortenedPath, 1.0, 0.9, now)
            decision = controller.tick(now)
        }
        val speed = kotlin.math.hypot(
            decision.forwardMetersPerSecond,
            decision.rightMetersPerSecond,
        )

        assertEquals(FixedHeadingLapPhase.TRACKING, decision.phase)
        assertTrue(speed > FixedHeadingLapController.DEGRADED_SPEED_METERS_PER_SECOND)
        assertTrue(speed < 0.5)
    }

    @Test
    fun `fixed heading starts moving on its first valid route sample`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        controller.start(1L)
        controller.observe(centerline, 1.0, 0.9, 1L)

        val decision = controller.tick(100_000_001L)

        assertEquals(FixedHeadingLapPhase.TRACKING, decision.phase)
        assertTrue(decision.forwardMetersPerSecond > 0.0)
        assertEquals(0.0, decision.rightMetersPerSecond, 1e-6)
        assertEquals(0.0, decision.virtualHeadingDegrees, 1e-6)
        assertFalse(decision.stopRequested)
    }

    @Test
    fun `fixed heading steers directly toward the measured lookahead`() {
        val controller = FixedHeadingLapController()
        controller.start(1L)
        controller.observe(
            centerline = curvedPath(tangentDegrees = 0.0),
            heightMeters = 1.2,
            confidence = 0.9,
            nowNanos = 1L,
        )

        val decision = controller.tick(100_000_001L)

        assertEquals(FixedHeadingLapPhase.TRACKING, decision.phase)
        assertTrue(decision.virtualHeadingDegrees > 5.0)
        assertTrue(decision.rightMetersPerSecond > 0.0)
        assertFalse(decision.stopRequested)
    }

    @Test
    fun `slow fixed heading profile caps body velocity at point two meters per second`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.10f),
        )
        controller.start(1L, FixedHeadingTrackingSpeed.SLOW)
        for (now in listOf(1L, 200_000_001L, 400_000_001L)) {
            controller.observe(centerline, 1.0, 0.9, now)
        }

        var decision = controller.tick(500_000_001L)
        for (now in listOf(700_000_001L, 900_000_001L, 1_100_000_001L)) {
            controller.observe(centerline, 1.0, 0.9, now)
            decision = controller.tick(now)
        }

        val speed = kotlin.math.hypot(
            decision.forwardMetersPerSecond,
            decision.rightMetersPerSecond,
        )
        assertTrue(speed <= FixedHeadingTrackingSpeed.SLOW.targetMetersPerSecond)
        assertTrue(speed > 0.19)
        assertEquals(0.0, decision.virtualHeadingDegrees, 1e-6)
    }

    @Test
    fun `boost fixed heading profile reaches one point two five meters per second on a centered route`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(1.00f, 0.80f, 0.60f, 0.40f, 0.20f, 0.00f),
        )
        controller.start(1L, FixedHeadingTrackingSpeed.BOOST)

        var decision = FixedHeadingLapDecision(FixedHeadingLapPhase.ACQUIRING)
        repeat(50) { index ->
            val now = (index + 1L) * 100_000_000L + 1L
            controller.observe(centerline, 1.2, 0.9, now)
            decision = controller.tick(now)
        }

        val speed = kotlin.math.hypot(
            decision.forwardMetersPerSecond,
            decision.rightMetersPerSecond,
        )
        assertTrue(speed > FixedHeadingTrackingSpeed.FAST.targetMetersPerSecond)
        assertEquals(FixedHeadingTrackingSpeed.BOOST.targetMetersPerSecond, speed, 0.001)
        assertTrue(
            FixedHeadingTrackingSpeed.BOOST.targetMetersPerSecond <=
                VirtualStickSession.AUTONOMOUS_MAX_HORIZONTAL_MPS,
        )
    }

    @Test
    fun `boost keeps full authority through a moderate centered correction`() {
        val centerline = path(
            xs = floatArrayOf(0.55f, 0.55f, 0.55f, 0.55f, 0.55f, 0.55f),
            ys = floatArrayOf(1.00f, 0.80f, 0.60f, 0.40f, 0.20f, 0.00f),
        )
        fun settledSpeed(trackingSpeed: FixedHeadingTrackingSpeed): Double {
            val controller = FixedHeadingLapController()
            controller.start(1L, trackingSpeed)
            var decision = FixedHeadingLapDecision(FixedHeadingLapPhase.ACQUIRING)
            repeat(50) { index ->
                val now = (index + 1L) * 100_000_000L + 1L
                controller.observe(centerline, 1.2, 0.9, now)
                decision = controller.tick(now)
            }
            return kotlin.math.hypot(
                decision.forwardMetersPerSecond,
                decision.rightMetersPerSecond,
            )
        }

        val fastSpeed = settledSpeed(FixedHeadingTrackingSpeed.FAST)
        val boostSpeed = settledSpeed(FixedHeadingTrackingSpeed.BOOST)

        assertTrue(fastSpeed < FixedHeadingTrackingSpeed.FAST.targetMetersPerSecond)
        assertTrue(boostSpeed > 1.10)
        assertTrue(boostSpeed > fastSpeed * 1.5)
    }

    @Test
    fun `committed boost bridges one detector period before braking`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(1.00f, 0.80f, 0.60f, 0.40f, 0.20f, 0.00f),
        )
        controller.start(1L, FixedHeadingTrackingSpeed.BOOST)
        var tracking = FixedHeadingLapDecision(FixedHeadingLapPhase.ACQUIRING)
        repeat(50) { index ->
            val now = (index + 1L) * 100_000_000L + 1L
            controller.observe(centerline, 1.2, 0.9, now)
            tracking = controller.tick(now)
        }
        val trackingSpeed = kotlin.math.hypot(
            tracking.forwardMetersPerSecond,
            tracking.rightMetersPerSecond,
        )

        controller.observe(null, 1.2, 0.0, 5_050_000_001L)
        val held = controller.tick(5_200_000_001L)
        val braking = controller.tick(5_250_000_001L)
        val heldSpeed = kotlin.math.hypot(
            held.forwardMetersPerSecond,
            held.rightMetersPerSecond,
        )
        val brakingSpeed = kotlin.math.hypot(
            braking.forwardMetersPerSecond,
            braking.rightMetersPerSecond,
        )

        assertEquals(FixedHeadingLapPhase.COASTING, held.phase)
        assertEquals(trackingSpeed, heldSpeed, 1e-6)
        assertTrue(brakingSpeed < heldSpeed)
        assertFalse(braking.stopRequested)
    }
    @Test
    fun `fixed heading recenters before advancing when route is far off axis`() {
        val controller = FixedHeadingLapController()
        val displacedPath = path(
            xs = floatArrayOf(0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        controller.start(1L)
        controller.observe(displacedPath, 1.0, 0.9, 1L)

        val decision = controller.tick(100_000_001L)

        assertEquals(0.0, decision.forwardMetersPerSecond, 1e-6)
        assertEquals(
            FixedHeadingLapController.MAX_LATERAL_CORRECTION_METERS_PER_SECOND,
            decision.rightMetersPerSecond,
            1e-6,
        )
        assertFalse(decision.stopRequested)
    }


    @Test
    fun `fixed heading smooths a confidence limited speed reduction`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.10f),
        )
        controller.start(1L)
        var cruising = FixedHeadingLapDecision(FixedHeadingLapPhase.ACQUIRING)
        repeat(20) { index ->
            val now = 100_000_001L + index * 100_000_000L
            controller.observe(centerline, 1.0, 0.9, now)
            cruising = controller.tick(now)
        }
        val cruisingSpeed = kotlin.math.hypot(
            cruising.forwardMetersPerSecond,
            cruising.rightMetersPerSecond,
        )

        controller.observe(centerline, 1.0, 0.5, 2_100_000_001L)
        val slowing = controller.tick(2_100_000_001L)
        val slowingSpeed = kotlin.math.hypot(
            slowing.forwardMetersPerSecond,
            slowing.rightMetersPerSecond,
        )

        val speedReduction = cruisingSpeed - slowingSpeed
        assertTrue(speedReduction > 0.0)
        assertTrue(
            speedReduction <=
                FixedHeadingLapController.MAX_JERK_METERS_PER_SECOND_CUBED *
                CONTROL_STEP_SECONDS * CONTROL_STEP_SECONDS,
        )
        assertTrue(slowingSpeed > FixedHeadingLapController.DEGRADED_SPEED_METERS_PER_SECOND)
    }

    @Test
    fun `fixed heading holds speed for one reflection frame before braking`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        controller.start(1L)
        var tracking = FixedHeadingLapDecision(FixedHeadingLapPhase.ACQUIRING)
        repeat(20) { index ->
            val now = 100_000_001L + index * 100_000_000L
            controller.observe(centerline, 1.0, 0.9, now)
            tracking = controller.tick(now)
        }
        val trackingSpeed = kotlin.math.hypot(
            tracking.forwardMetersPerSecond,
            tracking.rightMetersPerSecond,
        )
        controller.observe(null, 1.0, 0.0, 2_050_000_001L)

        val held = controller.tick(2_100_000_001L)
        val braking = controller.tick(2_250_000_001L)
        val waiting = controller.tick(2_500_000_001L)
        val stale = controller.tick(3_300_000_001L)
        val heldSpeed = kotlin.math.hypot(
            held.forwardMetersPerSecond,
            held.rightMetersPerSecond,
        )
        val brakingSpeed = kotlin.math.hypot(
            braking.forwardMetersPerSecond,
            braking.rightMetersPerSecond,
        )

        assertEquals(FixedHeadingLapPhase.COASTING, held.phase)
        assertEquals(trackingSpeed, heldSpeed, 1e-6)
        assertFalse(held.stopRequested)
        assertEquals(
            trackingSpeed -
                FixedHeadingLapController.MAX_BLIND_DECELERATION_METERS_PER_SECOND_SQUARED *
                0.15,
            brakingSpeed,
            1e-6,
        )
        assertFalse(braking.stopRequested)
        assertEquals(FixedHeadingLapPhase.COASTING, waiting.phase)
        assertFalse(waiting.stopRequested)
        assertTrue(stale.stopRequested)
    }

    @Test
    fun `fixed heading crosses one dropped reflection frame without slowing`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        controller.start(1L)
        var tracking = FixedHeadingLapDecision(FixedHeadingLapPhase.ACQUIRING)
        repeat(20) { index ->
            val now = 100_000_001L + index * 100_000_000L
            controller.observe(centerline, 1.0, 0.9, now)
            tracking = controller.tick(now)
        }
        val trackingSpeed = kotlin.math.hypot(
            tracking.forwardMetersPerSecond,
            tracking.rightMetersPerSecond,
        )

        controller.observe(null, 1.0, 0.0, 2_084_000_001L)
        val held = controller.tick(2_100_000_001L)
        controller.observe(centerline, 1.0, 0.9, 2_181_000_001L)
        val resumed = controller.tick(2_200_000_001L)
        val heldSpeed = kotlin.math.hypot(
            held.forwardMetersPerSecond,
            held.rightMetersPerSecond,
        )
        val resumedSpeed = kotlin.math.hypot(
            resumed.forwardMetersPerSecond,
            resumed.rightMetersPerSecond,
        )

        assertEquals(FixedHeadingLapPhase.COASTING, held.phase)
        assertEquals(trackingSpeed, heldSpeed, 1e-6)
        assertFalse(held.stopRequested)
        assertEquals(FixedHeadingLapPhase.TRACKING, resumed.phase)
        assertTrue(resumedSpeed >= heldSpeed)
        assertFalse(resumed.stopRequested)
    }

    @Test
    fun `fixed heading resumes after a half second reflection gap`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        controller.start(1L)
        repeat(20) { index ->
            val now = 100_000_001L + index * 100_000_000L
            controller.observe(centerline, 1.0, 0.9, now)
            controller.tick(now)
        }
        controller.observe(null, 1.0, 0.0, 2_050_000_001L)
        val waiting = controller.tick(2_500_000_001L)

        controller.observe(centerline, 1.0, 0.9, 2_550_000_001L)
        val resumed = controller.tick(2_600_000_001L)

        assertEquals(FixedHeadingLapPhase.COASTING, waiting.phase)
        assertFalse(waiting.stopRequested)
        assertEquals(FixedHeadingLapPhase.TRACKING, resumed.phase)
        assertFalse(resumed.stopRequested)
        assertTrue(resumed.forwardMetersPerSecond > 0.0)
    }

    @Test
    fun `fixed heading rebases only after two consistent recovery measurements`() {
        val controller = FixedHeadingLapController()
        val initialPath = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        val shiftedPath = path(
            xs = floatArrayOf(0.645f, 0.645f, 0.645f, 0.645f, 0.645f, 0.645f),
            ys = floatArrayOf(0.15f, 0.0f, -0.15f, -0.30f, -0.45f, -0.60f),
        )
        val shiftedMeasurement = checkNotNull(
            OmniCenterline.measure(
                path = shiftedPath,
                heightMeters = 1.0,
                travelDirectionDegrees = 0.0,
                lookaheadMeters = 0.35,
            ),
        )
        assertTrue(
            "lateral=${shiftedMeasurement.lateralOffsetMeters}",
            kotlin.math.abs(shiftedMeasurement.lateralOffsetMeters) <= 0.20,
        )
        assertTrue(
            "guidance=${Math.toDegrees(kotlin.math.atan2(
                shiftedMeasurement.lookaheadRightMeters,
                shiftedMeasurement.lookaheadForwardMeters,
            ))}",
            kotlin.math.abs(
                Math.toDegrees(kotlin.math.atan2(
                    shiftedMeasurement.lookaheadRightMeters,
                    shiftedMeasurement.lookaheadForwardMeters,
                )),
            ) <= 35.0,
        )
        assertNotNull(
            OmniCenterline.measure(
                path = shiftedPath,
                heightMeters = 1.0,
                travelDirectionDegrees = 0.0,
                lookaheadMeters = 0.35,
                previousMeasurement = shiftedMeasurement,
            ),
        )
        controller.start(1L)
        controller.observe(initialPath, 1.0, 0.9, 100_000_001L)
        controller.tick(100_000_001L)

        controller.observe(shiftedPath, 1.0, 0.9, 200_000_001L)
        val firstRecovery = controller.tick(200_000_001L)
        controller.observe(shiftedPath, 1.0, 0.9, 300_000_001L)
        val confirmedRecovery = controller.tick(300_000_001L)

        assertEquals(FixedHeadingLapPhase.COASTING, firstRecovery.phase)
        assertFalse(firstRecovery.stopRequested)
        assertEquals(FixedHeadingLapPhase.TRACKING, confirmedRecovery.phase)
        assertFalse(confirmedRecovery.stopRequested)
        assertTrue(confirmedRecovery.rightMetersPerSecond > 0.0)
    }

    @Test
    fun `nominal profile can cover a 2 point 5 meter diameter lap within ten seconds`() {
        val targetDistanceMeters = Math.PI * 2.5
        var speed = 0.0
        var distance = 0.0
        var elapsedSeconds = 0.0
        while (distance < targetDistanceMeters) {
            speed = minOf(
                FixedHeadingTrackingSpeed.FAST.targetMetersPerSecond,
                speed +
                    FixedHeadingLapController.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED *
                    CONTROL_STEP_SECONDS,
            )
            distance += speed * CONTROL_STEP_SECONDS
            elapsedSeconds += CONTROL_STEP_SECONDS
        }

        assertTrue(elapsedSeconds <= 10.0)
    }

    @Test
    fun `fixed heading continues tracking after a full virtual turn`() {
        val controller = FixedHeadingLapController()
        controller.start(1L)
        var decision = FixedHeadingLapDecision(FixedHeadingLapPhase.ACQUIRING)
        var accumulatedTurnDegrees = 0.0

        repeat(300) { index ->
            val now = 100_000_001L + index * 100_000_000L
            val previousHeading = decision.virtualHeadingDegrees
            controller.observe(
                centerline = curvedPath(previousHeading),
                heightMeters = 1.2,
                confidence = 0.9,
                nowNanos = now,
            )
            decision = controller.tick(now)
            accumulatedTurnDegrees += shortestAngularDelta(
                previousHeading,
                decision.virtualHeadingDegrees,
            )
        }

        assertTrue(accumulatedTurnDegrees >= 360.0)
        assertEquals(FixedHeadingLapPhase.TRACKING, decision.phase)
        assertFalse(decision.endpointReached)
        assertFalse(decision.stopRequested)
    }

    @Test
    fun `diagnostic turn cycle crosses the heading seam and ignores reverse motion`() {
        val timer = DiagnosticTurnCycleTimer()
        timer.arm(0L, 170.0)
        assertNull(timer.update(1_000_000_000L, -170.0))
        assertNull(timer.update(2_000_000_000L, -175.0))
        assertEquals(20.0, timer.progressDegrees, 1e-9)
        assertNull(timer.update(3_000_000_000L, 170.0))
        assertEquals(20.0, timer.progressDegrees, 1e-9)

        var angle = -170.0
        var event: DiagnosticTurnCycleEvent? = null
        repeat(18) { index ->
            angle = wrapDegrees(angle + 20.0)
            event = timer.update((index + 4L) * 1_000_000_000L, angle) ?: event
        }
        assertNotNull(event)
        assertEquals(1, checkNotNull(event).index)
        assertTrue(checkNotNull(event).turnDegrees >= 360.0)
    }

    @Test
    fun `diagnostic turn cycle requires movement gate and reports measured values`() {
        val timer = DiagnosticTurnCycleTimer(minimumDistanceMeters = 6.5)
        timer.arm(0L, 0.0)
        var angle = 0.0
        var event: DiagnosticTurnCycleEvent? = null

        repeat(18) { index ->
            angle = wrapDegrees(angle + 20.0)
            event =
                timer.update(
                    nowNanos = (index + 1L) * 683_333_333L,
                    angleDegrees = angle,
                    groundSpeedMetersPerSecond = 0.42,
                ) ?: event
        }

        assertNull(event)
        assertEquals(360.0, timer.progressDegrees, 1e-6)
        angle = wrapDegrees(angle + 20.0)
        event =
            timer.update(
                nowNanos = 16_000_000_000L,
                angleDegrees = angle,
                groundSpeedMetersPerSecond = 0.42,
            )
        val completedCycle = checkNotNull(event)
        assertEquals(1, completedCycle.index)
        assertEquals(16.0, completedCycle.elapsedSeconds, 1e-9)
        assertTrue(completedCycle.distanceMeters >= 6.5)
        assertEquals(380.0, completedCycle.turnDegrees, 1e-6)
    }

    @Test
    fun `fixed heading hold corrects drift across the heading seam`() {
        assertEquals(1.6, fixedHeadingHoldYawRate(179.0, -179.0), 1e-6)
        assertEquals(-1.6, fixedHeadingHoldYawRate(-179.0, 179.0), 1e-6)
        assertEquals(10.0, fixedHeadingHoldYawRate(0.0, 90.0), 1e-6)
        assertEquals(-10.0, fixedHeadingHoldYawRate(0.0, -90.0), 1e-6)
    }

    private fun curvedPath(tangentDegrees: Double): TapeCenterlinePath {
        val sourceWidth = 640
        val sourceHeight = 360
        val heightMeters = 1.2
        val aspectRatio = sourceWidth.toDouble() / sourceHeight
        val verticalHalfFovTangent =
            TapeTrackingController.CAMERA_DIAGONAL_HALF_FOV_TANGENT /
                kotlin.math.sqrt(1.0 + aspectRatio * aspectRatio)
        val frameHeightMeters = 2.0 * heightMeters * verticalHalfFovTangent
        val frameWidthMeters = frameHeightMeters * aspectRatio
        val tangentRadians = Math.toRadians(tangentDegrees)
        val samplesMeters = (-5..8).map { it * 0.05 }
        val xs = FloatArray(samplesMeters.size)
        val ys = FloatArray(samplesMeters.size)
        samplesMeters.forEachIndexed { index, distanceMeters ->
            val normalMeters = 0.5 * distanceMeters * distanceMeters
            val rightMeters =
                kotlin.math.sin(tangentRadians) * distanceMeters +
                    kotlin.math.cos(tangentRadians) * normalMeters
            val forwardMeters =
                kotlin.math.cos(tangentRadians) * distanceMeters -
                    kotlin.math.sin(tangentRadians) * normalMeters
            xs[index] = (0.5 + rightMeters / frameWidthMeters).toFloat()
            ys[index] = (0.5 - forwardMeters / frameHeightMeters).toFloat()
        }
        return path(xs, ys)
    }

    private fun path(xs: FloatArray, ys: FloatArray): TapeCenterlinePath =
        TapeCenterlinePath(
            sourceWidth = 640,
            sourceHeight = 360,
            xFractions = xs,
            yFractions = ys,
            anchorXFraction = 0.5f,
            anchorYFraction = 0.94f,
            lookaheadXFraction = null,
            lookaheadYFraction = null,
            quality = PathQuality.FULL_PATH,
            rejection = null,
        )

    private companion object {
        const val CONTROL_STEP_SECONDS = 0.1
    }
}
