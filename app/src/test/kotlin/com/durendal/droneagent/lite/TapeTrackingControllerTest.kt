package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class TapeOrientationTest {
    @Test
    fun `orientation is signed from image-up and ignores endpoint order`() {
        assertEquals(0.0, TapeOrientation.deviationFromVerticalDegrees(0.0, -10.0), 0.001)
        assertEquals(45.0, TapeOrientation.deviationFromVerticalDegrees(10.0, -10.0), 0.001)
        assertEquals(45.0, TapeOrientation.deviationFromVerticalDegrees(-10.0, 10.0), 0.001)
        assertEquals(-45.0, TapeOrientation.deviationFromVerticalDegrees(-10.0, -10.0), 0.001)
    }
}

class TapeOverlayLabelTest {
    @Test
    fun `label renders percent and signed angle without formatter parsing`() {
        assertEquals("BLACK TAPE 77%  +43°", formatTapeDetectionLabel(0.779, 42.6))
        assertEquals("BLACK TAPE 80%  -18°", formatTapeDetectionLabel(0.80, -17.6))
        assertEquals("BLACK TAPE 90%  0°", formatTapeDetectionLabel(0.90, -0.2))
    }
}

class TapeTrackingControllerTest {

    @Test
    fun `aligned shortened tape enters a bounded endpoint probe before turning`() {
        val controller = trackingController()
        controller.observe(observation(0.0, 0.8), seconds(2))
        assertEquals(0.15, controller.tick(seconds(2)).forwardSpeedMetersPerSecond, 0.0)

        controller.observe(observation(0.0, 0.35), seconds(3))
        assertEquals(0.0, controller.tick(seconds(3)).forwardSpeedMetersPerSecond, 0.0)
        controller.observe(observation(0.0, 0.35), seconds(3) + 400_000_000L)
        assertFalse(controller.tick(seconds(3) + 400_000_000L).endpointReached)
        controller.observe(observation(0.0, 0.35), seconds(3) + 800_000_000L)

        val probe = controller.tick(seconds(3) + 800_000_000L)
        assertFalse(probe.endpointReached)
        assertEquals(TapeTrackingPhase.VERIFYING_ENDPOINT, probe.phase)
        assertEquals(0.10, probe.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, probe.yawRateDegreesPerSecond, 0.0)

        val probeTimeout = controller.tick(seconds(3) + 2_400_000_000L)
        assertEquals(TapeTrackingPhase.VERIFYING_ENDPOINT, probeTimeout.phase)
        assertEquals(0.0, probeTimeout.forwardSpeedMetersPerSecond, 0.0)

        val endpoint = confirmEndpointDisappearance(controller, seconds(6))
        assertTrue(endpoint.endpointReached)
        assertEquals(TapeTrackingPhase.TURNING, endpoint.phase)
    }

    @Test
    fun `latest low altitude endpoint sequence starts the probe`() {
        val controller = trackingController()
        controller.observe(observation(0.0, 1.0), seconds(2))
        controller.observe(observation(0.5, 0.676), seconds(3))
        controller.observe(observation(-3.1, 0.688), seconds(4))
        controller.observe(observation(4.5, 0.546), seconds(5))
        controller.observe(observation(6.0, 0.515), seconds(5) + 400_000_000L)
        controller.observe(observation(8.5, 0.484), seconds(5) + 800_000_000L)

        val probe = controller.tick(seconds(5) + 800_000_000L)

        assertFalse(probe.endpointReached)
        assertEquals(TapeTrackingPhase.VERIFYING_ENDPOINT, probe.phase)
        assertEquals(0.10, probe.forwardSpeedMetersPerSecond, 0.0)
    }

    @Test
    fun `gradual endpoint shrink enters probe before detector loses the tape`() {
        val controller = trackingController()
        controller.observe(observation(0.0, 1.0), seconds(2))
        controller.observe(observation(-5.2, 0.74), seconds(3))
        controller.observe(observation(-4.0, 0.72), seconds(3) + 250_000_000L)
        controller.observe(observation(-2.4, 0.70), seconds(3) + 500_000_000L)
        controller.observe(observation(0.0, 0.681), seconds(3) + 750_000_000L)

        val probe = controller.tick(seconds(3) + 750_000_000L)
        assertEquals(TapeTrackingPhase.VERIFYING_ENDPOINT, probe.phase)
        assertEquals(0.10, probe.forwardSpeedMetersPerSecond, 0.0)

        val endpoint = confirmEndpointDisappearance(controller, seconds(6))
        assertTrue(endpoint.endpointReached)
        assertEquals(TapeTrackingPhase.TURNING, endpoint.phase)
    }

    @Test
    fun `verified short tape turns only after three consecutive misses`() {
        val controller = trackingController()
        controller.observe(observation(0.0, 1.0), seconds(2))
        controller.observe(observation(3.2, 0.27), seconds(3))
        controller.observe(observation(3.2, 0.27), seconds(3) + 400_000_000L)
        controller.observe(observation(3.2, 0.27), seconds(3) + 800_000_000L)
        assertEquals(
            TapeTrackingPhase.VERIFYING_ENDPOINT,
            controller.tick(seconds(3) + 800_000_000L).phase,
        )

        controller.observe(null, seconds(5) + 300_000_000L)
        controller.observe(null, seconds(5) + 550_000_000L)
        assertFalse(controller.tick(seconds(5) + 550_000_000L).endpointReached)
        controller.observe(null, seconds(5) + 800_000_000L)

        val endpoint = controller.tick(seconds(5) + 800_000_000L)
        assertTrue(endpoint.endpointReached)
        assertEquals(TapeTrackingPhase.TURNING, endpoint.phase)
    }

    @Test
    fun `endpoint probe completes before unrelated candidates can turn`() {
        val controller = trackingController()
        enterEndpointVerification(controller)

        controller.observe(observation(86.8, 0.353, 0.223), seconds(4))
        controller.observe(observation(0.0, 0.258, 0.283), seconds(4) + 250_000_000L)
        controller.observe(observation(87.3, 0.294, 0.196), seconds(4) + 500_000_000L)

        val probing = controller.tick(seconds(4) + 500_000_000L)
        assertFalse(probing.endpointReached)
        assertEquals(TapeTrackingPhase.VERIFYING_ENDPOINT, probing.phase)
        assertEquals(0.10, probing.forwardSpeedMetersPerSecond, 0.0)

        controller.observe(observation(86.8, 0.353, 0.223), seconds(5) + 300_000_000L)
        val endpoint = controller.tick(seconds(5) + 300_000_000L)
        assertTrue(endpoint.endpointReached)
        assertEquals(TapeTrackingPhase.TURNING, endpoint.phase)
    }

    @Test
    fun `offset tracked tape remains authoritative throughout endpoint probe`() {
        val controller = trackingController()
        controller.observe(observation(0.0, 1.0), seconds(2))
        controller.observe(
            observation(
                angleDegrees = 6.1,
                longSideFraction = 0.724,
                nearFieldOffsetFraction = -0.069,
                bounds = NormalizedRect(0.39375, 0.283, 0.46875, 1.0),
            ),
            seconds(3),
        )
        controller.observe(
            observation(
                angleDegrees = 6.0,
                longSideFraction = 0.690,
                nearFieldOffsetFraction = -0.080,
                bounds = NormalizedRect(0.385, 0.310, 0.460, 1.0),
            ),
            seconds(3) + 400_000_000L,
        )
        controller.observe(
            observation(
                angleDegrees = 6.2,
                longSideFraction = 0.652,
                nearFieldOffsetFraction = -0.090,
                bounds = NormalizedRect(0.3734375, 0.356, 0.446875, 1.0),
            ),
            seconds(3) + 800_000_000L,
        )

        val probe = controller.tick(seconds(3) + 800_000_000L)
        assertEquals(TapeTrackingPhase.VERIFYING_ENDPOINT, probe.phase)
        assertEquals(0.10, probe.forwardSpeedMetersPerSecond, 0.0)

        controller.observe(
            observation(
                angleDegrees = 6.2,
                longSideFraction = 0.652,
                nearFieldOffsetFraction = -0.090,
                bounds = NormalizedRect(0.3734375, 0.356, 0.446875, 1.0),
            ),
            seconds(5) + 300_000_000L,
        )
        val stillTrackingTape = controller.tick(seconds(5) + 300_000_000L)
        assertFalse(stillTrackingTape.endpointReached)
        assertEquals(TapeTrackingPhase.VERIFYING_ENDPOINT, stillTrackingTape.phase)

        val endpoint = confirmEndpointDisappearance(controller, seconds(6))
        assertTrue(endpoint.endpointReached)
        assertEquals(TapeTrackingPhase.TURNING, endpoint.phase)
    }

    @Test
    fun `endpoint probe keeps limited lateral centering active`() {
        val controller = trackingController()
        controller.observe(observation(0.0, 0.8, 0.20), seconds(2))
        controller.observe(observation(0.0, 0.35, 0.20), seconds(3))
        controller.observe(observation(0.0, 0.35, 0.20), seconds(3) + 400_000_000L)
        controller.observe(observation(0.0, 0.35, 0.20), seconds(3) + 800_000_000L)

        val probe = controller.tick(seconds(3) + 800_000_000L)

        assertEquals(TapeTrackingPhase.VERIFYING_ENDPOINT, probe.phase)
        assertEquals(0.10, probe.forwardSpeedMetersPerSecond, 0.0)
        assertTrue(probe.rightSpeedMetersPerSecond > 0.0)
        assertTrue(
            probe.rightSpeedMetersPerSecond <=
                TapeTrackingController.ENDPOINT_MAX_CENTERING_SPEED_METERS_PER_SECOND,
        )
    }

    @Test
    fun `centered endpoint continuation resets the miss sequence`() {
        val controller = trackingController()
        enterEndpointVerification(controller)

        controller.observe(null, seconds(4))
        controller.observe(observation(0.0, 0.30), seconds(4) + 250_000_000L)
        controller.observe(null, seconds(4) + 500_000_000L)
        controller.observe(null, seconds(4) + 750_000_000L)

        val waiting = controller.tick(seconds(4) + 750_000_000L)
        assertFalse(waiting.endpointReached)
        assertEquals(TapeTrackingPhase.VERIFYING_ENDPOINT, waiting.phase)
    }

    @Test
    fun `short diagonal tape after search is not an endpoint`() {
        val controller = trackingController()
        controller.observe(observation(0.0, 1.0), seconds(2))

        controller.observe(observation(-69.9, 0.251), seconds(3))
        controller.observe(observation(-57.4, 0.478), seconds(4))
        controller.observe(observation(-46.7, 0.275), seconds(5))

        val decision = controller.tick(seconds(5))
        assertFalse(decision.endpointReached)
        assertEquals(TapeTrackingPhase.TRACKING, decision.phase)
    }

    @Test
    fun `turn completion resets endpoint baseline and recenters camera`() {
        val controller = trackingController()
        enterEndpointVerification(controller)
        confirmEndpointDisappearance(controller, seconds(6))

        controller.resumeAfterTurn(seconds(7))
        val recenter = controller.tick(seconds(7))
        assertEquals(TapeTrackingPhase.RECENTERING, recenter.phase)
        assertEquals(TrackingGimbalTarget.DOWN_CENTER, recenter.gimbalTarget)
        assertEquals(TapeTrackingPhase.TRACKING, controller.tick(seconds(9)).phase)
        controller.observe(observation(0.0, 0.35), seconds(9))
        val resumed = controller.tick(seconds(9))
        assertFalse(resumed.endpointReached)
        assertEquals(0.15, resumed.forwardSpeedMetersPerSecond, 0.0)
    }

    @Test
    fun `post turn recovery advances briefly then keeps detecting while hovering`() {
        val controller = trackingController()
        enterEndpointVerification(controller)
        assertTrue(confirmEndpointDisappearance(controller, seconds(6)).endpointReached)

        controller.resumeAfterTurn(seconds(7))
        assertEquals(0.0, controller.tick(seconds(7)).forwardSpeedMetersPerSecond, 0.0)
        val recovery = controller.tick(seconds(9))
        assertEquals(TapeTrackingPhase.TRACKING, recovery.phase)
        assertEquals(0.15, recovery.forwardSpeedMetersPerSecond, 0.0)

        val waiting = controller.tick(seconds(12))
        assertFalse(waiting.endpointReached)
        assertEquals(TapeTrackingPhase.TRACKING, waiting.phase)
        assertEquals(0.0, waiting.forwardSpeedMetersPerSecond, 0.0)
        assertTrue(controller.enabled)
    }

    @Test
    fun `temporary detection loss never requests an endpoint turn`() {
        val controller = trackingController()
        controller.observe(observation(0.0, 0.8), seconds(2))
        controller.observe(null, seconds(3))

        val waiting = controller.tick(seconds(8))

        assertFalse(waiting.endpointReached)
        assertEquals(TapeTrackingPhase.TRACKING, waiting.phase)
        assertEquals(0.0, waiting.yawRateDegreesPerSecond, 0.0)
        assertEquals(0.0, waiting.forwardSpeedMetersPerSecond, 0.0)
        assertTrue(controller.enabled)

        controller.observe(observation(0.0, 0.8), seconds(9))
        assertEquals(0.15, controller.tick(seconds(9)).forwardSpeedMetersPerSecond, 0.0)
    }

    @Test
    fun `reflection gap cannot turn a reacquired short tape into an endpoint`() {
        val controller = trackingController()
        controller.observe(observation(0.0, 1.0), seconds(2))
        controller.observe(null, seconds(3))

        controller.observe(observation(-4.9, 0.293), seconds(3) + 250_000_000L)
        controller.observe(observation(-4.4, 0.306), seconds(3) + 650_000_000L)
        controller.observe(observation(-3.3, 0.306), seconds(3) + 1_050_000_000L)

        val reflection = controller.tick(seconds(3) + 1_050_000_000L)
        assertFalse(reflection.endpointReached)
        assertEquals(TapeTrackingPhase.TRACKING, reflection.phase)
        assertEquals(0.10, reflection.forwardSpeedMetersPerSecond, 0.0)

        controller.observe(null, seconds(4) + 200_000_000L)
        controller.observe(observation(-90.0, 0.406), seconds(4) + 450_000_000L)
        val glareEdge = controller.tick(seconds(4) + 450_000_000L)
        assertEquals(0.0, glareEdge.yawRateDegreesPerSecond, 0.0)
        assertEquals(0.0, glareEdge.forwardSpeedMetersPerSecond, 0.0)

        controller.observe(observation(0.0, 0.8), seconds(5))
        controller.observe(observation(0.0, 0.35), seconds(6))
        controller.observe(observation(0.0, 0.35), seconds(6) + 400_000_000L)
        controller.observe(observation(0.0, 0.35), seconds(6) + 800_000_000L)

        assertEquals(
            TapeTrackingPhase.VERIFYING_ENDPOINT,
            controller.tick(seconds(6) + 800_000_000L).phase,
        )
    }

    @Test
    fun `long tape reappearing cancels endpoint probe`() {
        val controller = trackingController()
        enterEndpointVerification(controller)

        controller.observe(observation(0.0, 0.90), seconds(4))
        val resumed = controller.tick(seconds(4))

        assertEquals(TapeTrackingPhase.TRACKING, resumed.phase)
        assertFalse(resumed.endpointReached)
        assertEquals(0.15, resumed.forwardSpeedMetersPerSecond, 0.0)
    }

    @Test
    fun `controller filters offset and rate limits lateral commands`() {
        val controller = TapeTrackingController()
        controller.start(0L)
        controller.tick(0L)
        controller.tick(seconds(2))

        controller.observe(observation(0.0, 0.8, 0.20), seconds(2))
        val first = controller.tick(seconds(2))
        assertEquals(0.20, first.controlledOffsetFraction!!, 0.001)
        assertEquals(0.020, first.rightSpeedMetersPerSecond, 0.001)

        controller.observe(observation(0.0, 0.8, 0.0), seconds(2) + 250_000_000L)
        val braking = controller.tick(seconds(2) + 250_000_000L)
        assertEquals(0.12, braking.controlledOffsetFraction!!, 0.001)
        assertEquals(-0.32, braking.offsetRatePerSecond, 0.001)
        assertEquals(0.010, braking.rightSpeedMetersPerSecond, 0.001)

        val stale = controller.tick(seconds(4))
        assertEquals(0.0, stale.rightSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, stale.yawRateDegreesPerSecond, 0.0)
    }


    @Test
    fun `circular mode follows the lookahead tangent at experiment speed`() {
        val controller = circularTrackingController()

        controller.observe(observation(12.0, 0.8, 0.05), seconds(2))
        val decision = controller.tick(seconds(2))

        assertEquals(TapeTrackingPhase.TRACKING, decision.phase)
        assertTrue(decision.yawRateDegreesPerSecond > 0.0)
        assertTrue(
            decision.yawRateDegreesPerSecond <=
                TapeTrackingController.CIRCULAR_MAX_YAW_RATE_DEGREES_PER_SECOND,
        )
        assertEquals(
            TapeTrackingController.CIRCULAR_CORRECTION_FORWARD_SPEED_METERS_PER_SECOND,
            decision.forwardSpeedMetersPerSecond,
            0.0,
        )
    }

    @Test
    fun `circular mode hovers instead of landing when a confirmed path disappears`() {
        val controller = circularTrackingController()
        confirmCircularTrack(controller)

        repeat(12) { index ->
            controller.observe(null, seconds(3) + index * 250_000_000L)
        }
        val waiting = controller.tick(seconds(6))

        assertFalse(waiting.endpointReached)
        assertEquals(TapeTrackingPhase.VERIFYING_ENDPOINT, waiting.phase)
        assertEquals(0.0, waiting.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, waiting.rightSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, waiting.yawRateDegreesPerSecond, 0.0)
    }

    @Test
    fun `circular mode does not land when no path was ever confirmed`() {
        val controller = circularTrackingController()

        repeat(8) { index ->
            controller.observe(null, seconds(2) + index * 250_000_000L)
        }
        val waiting = controller.tick(seconds(5))

        assertFalse(waiting.endpointReached)
        assertEquals(TapeTrackingPhase.TRACKING, waiting.phase)
        assertEquals(0.0, waiting.forwardSpeedMetersPerSecond, 0.0)
    }

    @Test
    fun `circular path reacquisition resumes tracking after a miss`() {
        val controller = circularTrackingController()
        confirmCircularTrack(controller)

        controller.observe(null, seconds(3))
        assertEquals(
            TapeTrackingPhase.VERIFYING_ENDPOINT,
            controller.tick(seconds(3)).phase,
        )
        controller.observe(observation(9.0, 0.75, 0.03), seconds(3) + 250_000_000L)
        val resumed = controller.tick(seconds(3) + 250_000_000L)

        assertFalse(resumed.endpointReached)
        assertEquals(TapeTrackingPhase.TRACKING, resumed.phase)
        assertTrue(resumed.forwardSpeedMetersPerSecond > 0.0)
    }

    private fun confirmCircularTrack(controller: TapeTrackingController) {
        controller.observe(observation(8.0, 0.8), seconds(2))
        controller.observe(observation(9.0, 0.8), seconds(2) + 250_000_000L)
        controller.observe(observation(10.0, 0.8), seconds(2) + 500_000_000L)
        controller.observe(observation(11.0, 0.8), seconds(2) + 750_000_000L)
    }

    private fun circularTrackingController(): TapeTrackingController =
        TapeTrackingController().also {
            it.start(0L, TapeTrackingMode.CIRCULAR)
            assertEquals(TrackingGimbalTarget.DOWN_CENTER, it.tick(0L).gimbalTarget)
            assertEquals(TapeTrackingPhase.TRACKING, it.tick(seconds(2)).phase)
        }

    private fun enterEndpointVerification(controller: TapeTrackingController) {
        controller.observe(observation(0.0, 0.8), seconds(2))
        controller.observe(observation(0.0, 0.35), seconds(3))
        controller.observe(observation(0.0, 0.35), seconds(3) + 400_000_000L)
        controller.observe(observation(0.0, 0.35), seconds(3) + 800_000_000L)
        assertEquals(
            TapeTrackingPhase.VERIFYING_ENDPOINT,
            controller.tick(seconds(3) + 800_000_000L).phase,
        )
    }

    private fun confirmEndpointDisappearance(
        controller: TapeTrackingController,
        startNanos: Long,
    ): TapeTrackingDecision {
        controller.observe(null, startNanos)
        controller.observe(null, startNanos + 250_000_000L)
        controller.observe(null, startNanos + 500_000_000L)
        return controller.tick(startNanos + 500_000_000L)
    }

    private fun trackingController(): TapeTrackingController = TapeTrackingController().also {
        it.start(0L)
        assertEquals(TrackingGimbalTarget.DOWN_CENTER, it.tick(0L).gimbalTarget)
        assertEquals(TapeTrackingPhase.TRACKING, it.tick(seconds(2)).phase)
    }

    private fun observation(
        angleDegrees: Double,
        longSideFraction: Double,
        nearFieldOffsetFraction: Double = 0.0,
        bounds: NormalizedRect = verticalBounds(longSideFraction, nearFieldOffsetFraction),
    ) = TapeTrackingObservation(
        angleFromVerticalDegrees = angleDegrees,
        longSideFraction = longSideFraction,
        nearFieldOffsetFraction = nearFieldOffsetFraction,
        bounds = bounds,
    )

    private fun verticalBounds(
        longSideFraction: Double,
        nearFieldOffsetFraction: Double,
    ): NormalizedRect {
        val center = 0.5 + nearFieldOffsetFraction
        return NormalizedRect(
            left = center - 0.04,
            top = 1.0 - longSideFraction.coerceAtMost(1.0),
            right = center + 0.04,
            bottom = 1.0,
        )
    }

    private fun seconds(value: Long): Long = value * 1_000_000_000L
}
