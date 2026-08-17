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
    fun `tracking turns outside tolerance and advances only while aligned`() {
        val controller = trackingController()

        controller.observe(45.0, 0.8, seconds(2))
        var decision = controller.tick(seconds(2))
        assertEquals(20.0, decision.yawRateDegreesPerSecond, 0.0)
        assertEquals(0.0, decision.forwardSpeedMetersPerSecond, 0.0)

        controller.observe(-50.0, 0.8, seconds(2))
        assertEquals(-20.0, controller.tick(seconds(2)).yawRateDegreesPerSecond, 0.0)
        controller.observe(10.0, 0.8, seconds(2))
        decision = controller.tick(seconds(2))
        assertEquals(0.0, decision.yawRateDegreesPerSecond, 0.0)
        assertEquals(0.3, decision.forwardSpeedMetersPerSecond, 0.0)
        controller.observe(10.1, 0.8, seconds(2))
        decision = controller.tick(seconds(2))
        assertEquals(20.0, decision.yawRateDegreesPerSecond, 0.0)
        assertEquals(0.0, decision.forwardSpeedMetersPerSecond, 0.0)
        controller.observe(null, null, seconds(2))
        decision = controller.tick(seconds(2))
        assertEquals(0.0, decision.yawRateDegreesPerSecond, 0.0)
        assertEquals(0.0, decision.forwardSpeedMetersPerSecond, 0.0)
    }

    @Test
    fun `aligned shortened tape must persist before endpoint turn`() {
        val controller = trackingController()
        controller.observe(0.0, 0.8, seconds(2))
        assertEquals(0.3, controller.tick(seconds(2)).forwardSpeedMetersPerSecond, 0.0)

        controller.observe(0.0, 0.35, seconds(3))
        assertEquals(0.0, controller.tick(seconds(3)).forwardSpeedMetersPerSecond, 0.0)
        controller.observe(0.0, 0.35, seconds(3) + 400_000_000L)
        assertFalse(controller.tick(seconds(3) + 400_000_000L).endpointReached)
        controller.observe(0.0, 0.35, seconds(3) + 800_000_000L)

        val endpoint = controller.tick(seconds(3) + 800_000_000L)
        assertTrue(endpoint.endpointReached)
        assertEquals(TapeTrackingPhase.TURNING, endpoint.phase)
        assertEquals(0.0, endpoint.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, endpoint.yawRateDegreesPerSecond, 0.0)
    }

    @Test
    fun `confirmed short tape remains an endpoint when it leaves the frame`() {
        val controller = trackingController()
        controller.observe(0.0, 1.0, seconds(2))
        controller.observe(3.2, 0.27, seconds(3))
        controller.observe(3.2, 0.27, seconds(3) + 50_000_000L)
        controller.observe(3.2, 0.27, seconds(3) + 100_000_000L)
        controller.observe(null, null, seconds(3) + 200_000_000L)

        val endpoint = controller.tick(seconds(3) + 800_000_000L)
        assertTrue(endpoint.endpointReached)
        assertEquals(TapeTrackingPhase.TURNING, endpoint.phase)
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
        assertEquals(-20.0, decision.yawRateDegreesPerSecond, 0.0)
    }

    @Test
    fun `turn completion resets endpoint baseline and recenters camera`() {
        val controller = trackingController()
        controller.observe(0.0, 0.8, seconds(2))
        controller.observe(0.0, 0.35, seconds(3))
        controller.observe(0.0, 0.35, seconds(3) + 400_000_000L)
        controller.observe(0.0, 0.35, seconds(3) + 800_000_000L)
        controller.tick(seconds(3) + 800_000_000L)

        controller.observe(0.0, 0.1, seconds(4))
        controller.resumeAfterTurn(seconds(4))
        val recenter = controller.tick(seconds(4))
        assertEquals(TapeTrackingPhase.RECENTERING, recenter.phase)
        assertEquals(TrackingGimbalTarget.DOWN_CENTER, recenter.gimbalTarget)
        assertEquals(TapeTrackingPhase.TRACKING, controller.tick(seconds(6)).phase)
        controller.observe(0.0, 0.35, seconds(6))
        assertFalse(controller.tick(seconds(6)).endpointReached)
    }

    @Test
    fun `five seconds without tape requests endpoint turn`() {
        val controller = trackingController()

        val loss = controller.tick(seconds(7))

        assertTrue(loss.endpointReached)
        assertEquals(TapeTrackingPhase.TURNING, loss.phase)
        assertEquals(0.0, loss.yawRateDegreesPerSecond, 0.0)
        assertEquals(0.0, loss.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(null, loss.gimbalTarget)
        assertTrue(controller.enabled)
    }

    private fun trackingController(): TapeTrackingController = TapeTrackingController().also {
        it.start(0L)
        assertEquals(TrackingGimbalTarget.DOWN_CENTER, it.tick(0L).gimbalTarget)
        assertEquals(TapeTrackingPhase.TRACKING, it.tick(seconds(2)).phase)
    }

    private fun seconds(value: Long): Long = value * 1_000_000_000L
}
