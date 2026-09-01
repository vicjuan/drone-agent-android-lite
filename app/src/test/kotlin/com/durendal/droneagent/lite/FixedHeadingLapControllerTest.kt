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
            lookaheadMeters = FixedHeadingLapController.LOOKAHEAD_METERS,
        )

        val tracked = checkNotNull(measurement)
        assertTrue(tracked.lookaheadDistanceMeters >= 0.15)
        assertTrue(
            tracked.lookaheadDistanceMeters < FixedHeadingLapController.LOOKAHEAD_METERS,
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
        controller.start(1L, FixedHeadingActuationPhaseLead.DEGREES_0)
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
        assertTrue(speed < FixedHeadingLapController.TARGET_SPEED_METERS_PER_SECOND)
    }

    @Test
    fun `fixed heading starts moving on its first valid route sample`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        controller.start(1L, FixedHeadingActuationPhaseLead.DEGREES_0)
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
        controller.start(1L, FixedHeadingActuationPhaseLead.DEGREES_0)
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
    fun `fourteen degree phase lead follows turn direction without changing virtual heading`() {
        fun trackingDecision(
            phaseLead: FixedHeadingActuationPhaseLead,
            mirrored: Boolean,
        ): FixedHeadingLapDecision {
            val controller = FixedHeadingLapController()
            val centeredPath = path(
                xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
                ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
            )
            val turnPath = curvedPath(tangentDegrees = 0.0).let { curve ->
                if (!mirrored) {
                    curve
                } else {
                    path(
                        xs = FloatArray(curve.pointCount) { index ->
                            1.0f - curve.xFractions[index]
                        },
                        ys = curve.yFractions,
                    )
                }
            }
            controller.start(1L, phaseLead)
            controller.observe(centeredPath, 1.2, 0.9, 1L)
            controller.tick(100_000_001L)
            controller.observe(turnPath, 1.2, 0.9, 200_000_001L)
            return controller.tick(200_000_001L)
        }

        val zeroPositive = trackingDecision(FixedHeadingActuationPhaseLead.DEGREES_0, false)
        val leadPositive = trackingDecision(FixedHeadingActuationPhaseLead.DEGREES_14, false)
        val zeroNegative = trackingDecision(FixedHeadingActuationPhaseLead.DEGREES_0, true)
        val leadNegative = trackingDecision(FixedHeadingActuationPhaseLead.DEGREES_14, true)
        fun velocityHeading(decision: FixedHeadingLapDecision): Double =
            Math.toDegrees(
                kotlin.math.atan2(
                    decision.rightMetersPerSecond,
                    decision.forwardMetersPerSecond,
                ),
            )
        fun speed(decision: FixedHeadingLapDecision): Double =
            kotlin.math.hypot(
                decision.forwardMetersPerSecond,
                decision.rightMetersPerSecond,
            )

        assertEquals(zeroPositive.virtualHeadingDegrees, leadPositive.virtualHeadingDegrees, 1e-9)
        assertEquals(zeroNegative.virtualHeadingDegrees, leadNegative.virtualHeadingDegrees, 1e-9)
        assertEquals(speed(zeroPositive), speed(leadPositive), 1e-9)
        assertEquals(speed(zeroNegative), speed(leadNegative), 1e-9)
        assertTrue(speed(leadPositive) <= FixedHeadingLapController.TARGET_SPEED_METERS_PER_SECOND)
        assertTrue(speed(leadNegative) <= FixedHeadingLapController.TARGET_SPEED_METERS_PER_SECOND)
        assertEquals(zeroPositive.virtualHeadingDegrees, wrapDegrees(velocityHeading(zeroPositive)), 1e-9)
        assertEquals(zeroNegative.virtualHeadingDegrees, wrapDegrees(velocityHeading(zeroNegative)), 1e-9)
        assertEquals(
            14.0,
            shortestAngularDelta(velocityHeading(zeroPositive), velocityHeading(leadPositive)),
            1e-9,
        )
        assertEquals(
            -14.0,
            shortestAngularDelta(velocityHeading(zeroNegative), velocityHeading(leadNegative)),
            1e-9,
        )
    }

    @Test
    fun `fourteen degree phase lead is absent at zero speed and after stopping`() {
        val zeroSpeedController = FixedHeadingLapController()
        val displacedPath = path(
            xs = floatArrayOf(0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        zeroSpeedController.start(1L, FixedHeadingActuationPhaseLead.DEGREES_14)
        zeroSpeedController.observe(displacedPath, 1.0, 0.9, 1L)
        val zeroSpeedDecision = zeroSpeedController.tick(100_000_001L)

        assertEquals(FixedHeadingLapPhase.TRACKING, zeroSpeedDecision.phase)
        assertEquals(0.0, zeroSpeedDecision.forwardMetersPerSecond, 1e-9)
        assertEquals(
            FixedHeadingLapController.MAX_LATERAL_CORRECTION_METERS_PER_SECOND,
            zeroSpeedDecision.rightMetersPerSecond,
            1e-9,
        )

        val stoppedController = FixedHeadingLapController()
        stoppedController.start(1L, FixedHeadingActuationPhaseLead.DEGREES_14)
        val stoppedDecision =
            stoppedController.tick(1L + FixedHeadingLapController.ACQUISITION_TIMEOUT_NANOS)

        assertEquals(FixedHeadingLapPhase.STOPPED, stoppedDecision.phase)
        assertEquals(0.0, stoppedDecision.forwardMetersPerSecond, 1e-9)
        assertEquals(0.0, stoppedDecision.rightMetersPerSecond, 1e-9)
        assertTrue(stoppedDecision.stopRequested)
    }

    @Test
    fun `fixed heading reaches its 1 point 25 meter per second target on a centered route`() {
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(1.00f, 0.80f, 0.60f, 0.40f, 0.20f, 0.00f),
        )
        val controller = FixedHeadingLapController()
        controller.start(1L, FixedHeadingActuationPhaseLead.DEGREES_0)
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
        assertEquals(1.25, FixedHeadingLapController.TARGET_SPEED_METERS_PER_SECOND, 0.001)
        assertEquals(FixedHeadingLapController.TARGET_SPEED_METERS_PER_SECOND, speed, 0.001)
        assertTrue(speed <= VirtualStickSession.AUTONOMOUS_MAX_HORIZONTAL_MPS)
        assertEquals(0.0, decision.virtualHeadingDegrees, 1e-6)
    }

    @Test
    fun `sixteen degree profile reaches its 1 point 35 meter per second target`() {
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(1.00f, 0.80f, 0.60f, 0.40f, 0.20f, 0.00f),
        )
        val controller = FixedHeadingLapController()
        controller.start(1L, FixedHeadingActuationPhaseLead.DEGREES_16)
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
        assertEquals(
            FixedHeadingLapController.FASTER_TARGET_SPEED_METERS_PER_SECOND,
            speed,
            0.001,
        )
        assertTrue(speed <= VirtualStickSession.AUTONOMOUS_MAX_HORIZONTAL_MPS)
    }


    @Test
    fun `committed fixed heading bridges one detector period before braking`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(1.00f, 0.80f, 0.60f, 0.40f, 0.20f, 0.00f),
        )
        controller.start(1L, FixedHeadingActuationPhaseLead.DEGREES_0)
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
        controller.start(1L, FixedHeadingActuationPhaseLead.DEGREES_0)
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
        controller.start(1L, FixedHeadingActuationPhaseLead.DEGREES_0)
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
                FixedHeadingLapController.HIGH_SPEED_MAX_JERK_METERS_PER_SECOND_CUBED *
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
        controller.start(1L, FixedHeadingActuationPhaseLead.DEGREES_0)
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
        val stale = controller.tick(6_300_000_001L)
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
        controller.start(1L, FixedHeadingActuationPhaseLead.DEGREES_0)
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
    fun `fixed heading resumes after a two second reflection gap`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        controller.start(1L, FixedHeadingActuationPhaseLead.DEGREES_0)
        repeat(20) { index ->
            val now = 100_000_001L + index * 100_000_000L
            controller.observe(centerline, 1.0, 0.9, now)
            controller.tick(now)
        }
        controller.observe(null, 1.0, 0.0, 2_050_000_001L)
        val waiting = controller.tick(4_000_000_001L)

        controller.observe(centerline, 1.0, 0.9, 4_150_000_001L)
        val resumed = controller.tick(4_200_000_001L)

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
        controller.start(1L, FixedHeadingActuationPhaseLead.DEGREES_0)
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
                FixedHeadingLapController.TARGET_SPEED_METERS_PER_SECOND,
                speed +
                    FixedHeadingLapController.HIGH_SPEED_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED *
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
        controller.start(1L, FixedHeadingActuationPhaseLead.DEGREES_0)
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
    fun `frame stream recovery hovers beyond ordinary path timeout and resumes after two observations`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        controller.start(1L, FixedHeadingActuationPhaseLead.DEGREES_16)
        repeat(20) { index ->
            val now = 100_000_001L + index * 100_000_000L
            controller.observe(centerline, 1.0, 0.9, now)
            controller.tick(now)
        }
        val staleAtNanos = 4_000_000_001L
        controller.beginFrameStreamRecovery(staleAtNanos)

        val waiting = controller.tick(staleAtNanos + 3_500_000_000L)
        controller.observe(centerline, 1.0, 0.9, staleAtNanos + 3_600_000_000L)
        val firstObservation = controller.tick(staleAtNanos + 3_600_000_000L)
        controller.observe(centerline, 1.0, 0.9, staleAtNanos + 3_700_000_000L)
        val resumed = controller.tick(staleAtNanos + 3_800_000_000L)

        assertEquals(FixedHeadingLapPhase.RECOVERING_FRAME_STREAM, waiting.phase)
        assertEquals(0.0, waiting.forwardMetersPerSecond, 0.0)
        assertEquals(0.0, waiting.rightMetersPerSecond, 0.0)
        assertFalse(waiting.stopRequested)
        assertEquals(FixedHeadingLapPhase.RECOVERING_FRAME_STREAM, firstObservation.phase)
        assertEquals(0.0, firstObservation.forwardMetersPerSecond, 0.0)
        assertEquals(FixedHeadingLapPhase.TRACKING, resumed.phase)
        assertTrue(resumed.forwardMetersPerSecond > 0.0)
        assertFalse(resumed.stopRequested)
    }

    @Test
    fun `frame stream recovery stops after bounded hover timeout`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        controller.start(1L, FixedHeadingActuationPhaseLead.DEGREES_0)
        controller.observe(centerline, 1.0, 0.9, 100_000_001L)
        controller.tick(100_000_001L)
        val staleAtNanos = 200_000_001L
        controller.beginFrameStreamRecovery(staleAtNanos)

        val stopped =
            controller.tick(
                staleAtNanos + FixedHeadingLapController.FRAME_STREAM_RECOVERY_TIMEOUT_NANOS,
            )

        assertEquals(FixedHeadingLapPhase.STOPPED, stopped.phase)
        assertEquals(0.0, stopped.forwardMetersPerSecond, 0.0)
        assertEquals(0.0, stopped.rightMetersPerSecond, 0.0)
        assertTrue(stopped.stopRequested)
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
