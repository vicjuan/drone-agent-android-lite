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
        controller.observe(0.0, 0.8, seconds(2))
        assertEquals(0.15, controller.tick(seconds(2)).forwardSpeedMetersPerSecond, 0.0)

        controller.observe(0.0, 0.35, seconds(3))
        assertEquals(0.0, controller.tick(seconds(3)).forwardSpeedMetersPerSecond, 0.0)
        controller.observe(0.0, 0.35, seconds(3) + 400_000_000L)
        assertFalse(controller.tick(seconds(3) + 400_000_000L).endpointReached)
        controller.observe(0.0, 0.35, seconds(3) + 800_000_000L)

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
        controller.observe(0.0, 1.0, seconds(2))
        controller.observe(0.5, 0.676, seconds(3))
        controller.observe(-3.1, 0.688, seconds(4))
        controller.observe(4.5, 0.546, seconds(5))
        controller.observe(6.0, 0.515, seconds(5) + 400_000_000L)
        controller.observe(8.5, 0.484, seconds(5) + 800_000_000L)

        val probe = controller.tick(seconds(5) + 800_000_000L)

        assertFalse(probe.endpointReached)
        assertEquals(TapeTrackingPhase.VERIFYING_ENDPOINT, probe.phase)
        assertEquals(0.10, probe.forwardSpeedMetersPerSecond, 0.0)
    }

    @Test
    fun `gradual endpoint shrink enters probe before detector loses the tape`() {
        val controller = trackingController()
        controller.observe(0.0, 1.0, seconds(2))
        controller.observe(-5.2, 0.74, seconds(3))
        controller.observe(-4.0, 0.72, seconds(3) + 250_000_000L)
        controller.observe(-2.4, 0.70, seconds(3) + 500_000_000L)
        controller.observe(0.0, 0.681, seconds(3) + 750_000_000L)

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
        controller.observe(0.0, 1.0, seconds(2))
        controller.observe(3.2, 0.27, seconds(3))
        controller.observe(3.2, 0.27, seconds(3) + 400_000_000L)
        controller.observe(3.2, 0.27, seconds(3) + 800_000_000L)
        assertEquals(
            TapeTrackingPhase.VERIFYING_ENDPOINT,
            controller.tick(seconds(3) + 800_000_000L).phase,
        )

        controller.observe(null, null, seconds(5) + 300_000_000L)
        controller.observe(null, null, seconds(5) + 550_000_000L)
        assertFalse(controller.tick(seconds(5) + 550_000_000L).endpointReached)
        controller.observe(null, null, seconds(5) + 800_000_000L)

        val endpoint = controller.tick(seconds(5) + 800_000_000L)
        assertTrue(endpoint.endpointReached)
        assertEquals(TapeTrackingPhase.TURNING, endpoint.phase)
    }

    @Test
    fun `endpoint probe completes before unrelated candidates can turn`() {
        val controller = trackingController()
        enterEndpointVerification(controller)

        controller.observe(86.8, 0.353, seconds(4), horizontalOffsetFraction = 0.223)
        controller.observe(0.0, 0.258, seconds(4) + 250_000_000L, horizontalOffsetFraction = 0.283)
        controller.observe(87.3, 0.294, seconds(4) + 500_000_000L, horizontalOffsetFraction = 0.196)

        val probing = controller.tick(seconds(4) + 500_000_000L)
        assertFalse(probing.endpointReached)
        assertEquals(TapeTrackingPhase.VERIFYING_ENDPOINT, probing.phase)
        assertEquals(0.10, probing.forwardSpeedMetersPerSecond, 0.0)

        controller.observe(86.8, 0.353, seconds(5) + 300_000_000L, horizontalOffsetFraction = 0.223)
        val endpoint = controller.tick(seconds(5) + 300_000_000L)
        assertTrue(endpoint.endpointReached)
        assertEquals(TapeTrackingPhase.TURNING, endpoint.phase)
    }

    @Test
    fun `centered endpoint continuation resets the miss sequence`() {
        val controller = trackingController()
        enterEndpointVerification(controller)

        controller.observe(null, null, seconds(4))
        controller.observe(0.0, 0.30, seconds(4) + 250_000_000L)
        controller.observe(null, null, seconds(4) + 500_000_000L)
        controller.observe(null, null, seconds(4) + 750_000_000L)

        val waiting = controller.tick(seconds(4) + 750_000_000L)
        assertFalse(waiting.endpointReached)
        assertEquals(TapeTrackingPhase.VERIFYING_ENDPOINT, waiting.phase)
    }

    @Test
    fun `short diagonal tape after search is not an endpoint`() {
        val controller = trackingController()
        controller.observe(0.0, 1.0, seconds(2))

        controller.observe(-69.9, 0.251, seconds(3))
        controller.observe(-57.4, 0.478, seconds(4))
        controller.observe(-46.7, 0.275, seconds(5))

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
        controller.observe(0.0, 0.35, seconds(9))
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
        controller.observe(0.0, 0.8, seconds(2))
        controller.observe(null, null, seconds(3))

        val waiting = controller.tick(seconds(8))

        assertFalse(waiting.endpointReached)
        assertEquals(TapeTrackingPhase.TRACKING, waiting.phase)
        assertEquals(0.0, waiting.yawRateDegreesPerSecond, 0.0)
        assertEquals(0.0, waiting.forwardSpeedMetersPerSecond, 0.0)
        assertTrue(controller.enabled)

        controller.observe(0.0, 0.8, seconds(9))
        assertEquals(0.15, controller.tick(seconds(9)).forwardSpeedMetersPerSecond, 0.0)
    }

    @Test
    fun `reflection gap cannot turn a reacquired short tape into an endpoint`() {
        val controller = trackingController()
        controller.observe(0.0, 1.0, seconds(2))
        controller.observe(null, null, seconds(3))

        controller.observe(-4.9, 0.293, seconds(3) + 250_000_000L)
        controller.observe(-4.4, 0.306, seconds(3) + 650_000_000L)
        controller.observe(-3.3, 0.306, seconds(3) + 1_050_000_000L)

        val reflection = controller.tick(seconds(3) + 1_050_000_000L)
        assertFalse(reflection.endpointReached)
        assertEquals(TapeTrackingPhase.TRACKING, reflection.phase)
        assertEquals(0.10, reflection.forwardSpeedMetersPerSecond, 0.0)

        controller.observe(null, null, seconds(4) + 200_000_000L)
        controller.observe(-90.0, 0.406, seconds(4) + 450_000_000L)
        val glareEdge = controller.tick(seconds(4) + 450_000_000L)
        assertEquals(0.0, glareEdge.yawRateDegreesPerSecond, 0.0)
        assertEquals(0.0, glareEdge.forwardSpeedMetersPerSecond, 0.0)

        controller.observe(0.0, 0.8, seconds(5))
        controller.observe(0.0, 0.35, seconds(6))
        controller.observe(0.0, 0.35, seconds(6) + 400_000_000L)
        controller.observe(0.0, 0.35, seconds(6) + 800_000_000L)

        assertEquals(
            TapeTrackingPhase.VERIFYING_ENDPOINT,
            controller.tick(seconds(6) + 800_000_000L).phase,
        )
    }

    @Test
    fun `long tape reappearing cancels endpoint probe`() {
        val controller = trackingController()
        enterEndpointVerification(controller)

        controller.observe(0.0, 0.90, seconds(4))
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

        controller.observe(0.0, 0.8, seconds(2), horizontalOffsetFraction = 0.20)
        val first = controller.tick(seconds(2))
        assertEquals(0.20, first.controlledOffsetFraction!!, 0.001)
        assertEquals(0.020, first.rightSpeedMetersPerSecond, 0.001)

        controller.observe(
            0.0,
            0.8,
            seconds(2) + 250_000_000L,
            horizontalOffsetFraction = 0.0,
        )
        val braking = controller.tick(seconds(2) + 250_000_000L)
        assertEquals(0.12, braking.controlledOffsetFraction!!, 0.001)
        assertEquals(-0.32, braking.offsetRatePerSecond, 0.001)
        assertEquals(0.010, braking.rightSpeedMetersPerSecond, 0.001)

        val stale = controller.tick(seconds(4))
        assertEquals(0.0, stale.rightSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, stale.yawRateDegreesPerSecond, 0.0)
    }


    private fun enterEndpointVerification(controller: TapeTrackingController) {
        controller.observe(0.0, 0.8, seconds(2))
        controller.observe(0.0, 0.35, seconds(3))
        controller.observe(0.0, 0.35, seconds(3) + 400_000_000L)
        controller.observe(0.0, 0.35, seconds(3) + 800_000_000L)
        assertEquals(
            TapeTrackingPhase.VERIFYING_ENDPOINT,
            controller.tick(seconds(3) + 800_000_000L).phase,
        )
    }

    private fun confirmEndpointDisappearance(
        controller: TapeTrackingController,
        startNanos: Long,
    ): TapeTrackingDecision {
        controller.observe(null, null, startNanos)
        controller.observe(null, null, startNanos + 250_000_000L)
        controller.observe(null, null, startNanos + 500_000_000L)
        return controller.tick(startNanos + 500_000_000L)
    }

    private fun trackingController(): TapeTrackingController = TapeTrackingController().also {
        it.start(0L)
        assertEquals(TrackingGimbalTarget.DOWN_CENTER, it.tick(0L).gimbalTarget)
        assertEquals(TapeTrackingPhase.TRACKING, it.tick(seconds(2)).phase)
    }

    private fun seconds(value: Long): Long = value * 1_000_000_000L
}
