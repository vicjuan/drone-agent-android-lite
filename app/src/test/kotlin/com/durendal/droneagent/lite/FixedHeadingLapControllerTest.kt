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
    fun `omni measurement moves the entry target to the left for rightward travel`() {
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
    }

    @Test
    fun `fixed heading starts moving on its first valid route sample`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        controller.start(1L)
        controller.observe(centerline, 1.0, 0.9, false, false, 1L)

        val decision = controller.tick(100_000_001L)

        assertEquals(FixedHeadingLapPhase.TRACKING, decision.phase)
        assertTrue(decision.forwardMetersPerSecond > 0.0)
        assertEquals(0.0, decision.rightMetersPerSecond, 1e-6)
        assertEquals(0.0, decision.virtualHeadingDegrees, 1e-6)
        assertFalse(decision.stopRequested)
    }

    @Test
    fun `slow fixed heading profile caps body velocity at point two meters per second`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        controller.start(1L, FixedHeadingTrackingSpeed.SLOW)
        for (now in listOf(1L, 200_000_001L, 400_000_001L)) {
            controller.observe(centerline, 1.0, 0.9, false, false, now)
        }

        var decision = controller.tick(500_000_001L)
        for (now in listOf(700_000_001L, 900_000_001L, 1_100_000_001L)) {
            controller.observe(centerline, 1.0, 0.9, false, false, now)
            decision = controller.tick(now)
        }

        assertEquals(
            FixedHeadingTrackingSpeed.SLOW.targetMetersPerSecond,
            kotlin.math.hypot(
                decision.forwardMetersPerSecond,
                decision.rightMetersPerSecond,
            ),
            1e-6,
        )
        assertEquals(0.0, decision.virtualHeadingDegrees, 1e-6)
    }
    @Test
    fun `fast fixed heading drops speed immediately when cross track authority degrades`() {
        val controller = FixedHeadingLapController()
        val centeredPath = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        val displacedPath = path(
            xs = floatArrayOf(0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        controller.start(1L)
        var decision = FixedHeadingLapDecision(FixedHeadingLapPhase.ACQUIRING)
        repeat(20) { index ->
            val now = 100_000_001L + index * 100_000_000L
            controller.observe(centeredPath, 1.0, 0.9, false, false, now)
            decision = controller.tick(now)
        }
        val stableSpeed = kotlin.math.hypot(
            decision.forwardMetersPerSecond,
            decision.rightMetersPerSecond,
        )

        val driftAt = 2_200_000_001L
        controller.observe(displacedPath, 1.0, 0.9, false, false, driftAt)
        val degraded = controller.tick(driftAt)
        val degradedSpeed = kotlin.math.hypot(
            degraded.forwardMetersPerSecond,
            degraded.rightMetersPerSecond,
        )

        assertEquals(FixedHeadingLapController.TARGET_SPEED_METERS_PER_SECOND, stableSpeed, 1e-6)
        assertTrue(degradedSpeed < stableSpeed)
    }


    @Test
    fun `fixed heading reports a confirmed physical tape endpoint as success`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        val endpointCenterline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.86f, 0.76f, 0.66f, 0.56f, 0.50f),
        )
        controller.start(1L, FixedHeadingTrackingSpeed.SLOW)
        for (now in listOf(1L, 200_000_001L, 400_000_001L)) {
            controller.observe(centerline, 1.0, 0.9, false, false, now)
        }
        controller.tick(500_000_001L)
        controller.observe(centerline, 1.0, 0.9, true, false, 550_000_001L)
        val approachingEndpoint = controller.tick(550_000_001L)
        assertFalse(approachingEndpoint.endpointReached)
        assertTrue(approachingEndpoint.forwardMetersPerSecond > 0.0)

        controller.observe(endpointCenterline, 1.0, 0.9, true, false, 600_000_001L)
        controller.observe(endpointCenterline, 1.0, 0.9, true, false, 1_100_000_001L)
        val decision = controller.tick(1_100_000_001L)

        assertEquals(FixedHeadingLapPhase.STOPPED, decision.phase)
        assertTrue(decision.endpointReached)
        assertFalse(decision.stopRequested)
        assertEquals(0.0, decision.forwardMetersPerSecond, 0.0)
        assertEquals(0.0, decision.rightMetersPerSecond, 0.0)
    }

    @Test
    fun `fixed heading zeros motion on a brief miss and stops when stale`() {
        val controller = FixedHeadingLapController()
        val centerline = path(
            xs = floatArrayOf(0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f),
            ys = floatArrayOf(0.96f, 0.82f, 0.68f, 0.54f, 0.40f, 0.20f),
        )
        controller.start(1L)
        for (now in listOf(1L, 200_000_001L, 400_000_001L)) {
            controller.observe(centerline, 1.0, 0.9, false, false, now)
        }
        controller.tick(500_000_001L)
        controller.observe(null, 1.0, 0.0, false, false, 550_000_001L)

        val coasting = controller.tick(600_000_001L)
        val stale = controller.tick(900_000_001L)

        assertEquals(FixedHeadingLapPhase.COASTING, coasting.phase)
        assertEquals(0.0, coasting.forwardMetersPerSecond, 0.0)
        assertEquals(0.0, coasting.rightMetersPerSecond, 0.0)
        assertFalse(coasting.stopRequested)
        assertTrue(stale.stopRequested)
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
                    FixedHeadingLapController.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED *
                    CONTROL_STEP_SECONDS,
            )
            distance += speed * CONTROL_STEP_SECONDS
            elapsedSeconds += CONTROL_STEP_SECONDS
        }

        assertTrue(elapsedSeconds <= 10.0)
    }

    @Test
    fun `fixed heading lap completes only after one full virtual turn`() {
        assertFalse(hasCompletedFixedHeadingLap(359.999))
        assertFalse(hasCompletedFixedHeadingLap(-359.999))
        assertTrue(hasCompletedFixedHeadingLap(360.0))
        assertTrue(hasCompletedFixedHeadingLap(-360.0))
    }

    @Test
    fun `lap timer crosses the heading seam and ignores reverse motion`() {
        val timer = LapTimer()
        timer.arm(0L, 170.0)
        assertNull(timer.update(1_000_000_000L, -170.0))
        assertNull(timer.update(2_000_000_000L, -175.0))
        assertEquals(20.0, timer.progressDegrees, 1e-9)
        assertNull(timer.update(3_000_000_000L, 170.0))
        assertEquals(20.0, timer.progressDegrees, 1e-9)

        var angle = -170.0
        var event: LapEvent? = null
        repeat(18) { index ->
            angle = wrapDegrees(angle + 20.0)
            event = timer.update((index + 4L) * 1_000_000_000L, angle) ?: event
        }
        assertNotNull(event)
        assertEquals(1, checkNotNull(event).lapIndex)
    }

    @Test
    fun `fixed heading hold corrects drift across the heading seam`() {
        assertEquals(1.6, fixedHeadingHoldYawRate(179.0, -179.0), 1e-6)
        assertEquals(-1.6, fixedHeadingHoldYawRate(-179.0, 179.0), 1e-6)
        assertEquals(10.0, fixedHeadingHoldYawRate(0.0, 90.0), 1e-6)
        assertEquals(-10.0, fixedHeadingHoldYawRate(0.0, -90.0), 1e-6)
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
