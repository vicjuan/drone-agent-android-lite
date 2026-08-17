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
    fun `three shortened detections stop motion and request endpoint turn`() {
        val controller = trackingController()
        controller.observe(0.0, 0.8, seconds(2))
        assertEquals(0.3, controller.tick(seconds(2)).forwardSpeedMetersPerSecond, 0.0)

        controller.observe(0.0, 0.35, seconds(3))
        assertEquals(0.0, controller.tick(seconds(3)).forwardSpeedMetersPerSecond, 0.0)
        controller.observe(0.0, 0.35, seconds(3) + 100L)
        controller.observe(0.0, 0.35, seconds(3) + 200L)

        val endpoint = controller.tick(seconds(3) + 200L)
        assertTrue(endpoint.endpointReached)
        assertEquals(TapeTrackingPhase.TURNING, endpoint.phase)
        assertEquals(0.0, endpoint.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, endpoint.yawRateDegreesPerSecond, 0.0)
    }

    @Test
    fun `turn completion resets endpoint baseline and recenters camera`() {
        val controller = trackingController()
        controller.observe(0.0, 0.8, seconds(2))
        repeat(3) { index ->
            controller.observe(0.0, 0.35, seconds(3) + index)
        }
        controller.tick(seconds(3) + 2L)

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
    fun `five seconds without tape starts alternating gimbal search`() {
        val controller = trackingController()

        val searchStart = controller.tick(seconds(7))
        assertEquals(TapeTrackingPhase.SEARCHING, searchStart.phase)
        assertEquals(TrackingGimbalTarget.SEARCH_LEFT, searchStart.gimbalTarget)
        assertEquals(TrackingGimbalTarget.SEARCH_RIGHT, controller.tick(seconds(10)).gimbalTarget)
        assertEquals(TrackingGimbalTarget.SEARCH_LEFT, controller.tick(seconds(13)).gimbalTarget)
    }

    @Test
    fun `three detections during search recenter before tracking resumes`() {
        val controller = trackingController()
        controller.tick(seconds(7))

        controller.observe(20.0, 0.8, seconds(8))
        controller.observe(21.0, 0.8, seconds(8) + 100L)
        controller.observe(19.0, 0.8, seconds(8) + 200L)
        val recenter = controller.tick(seconds(8) + 200L)
        assertEquals(TapeTrackingPhase.RECENTERING, recenter.phase)
        assertEquals(TrackingGimbalTarget.DOWN_CENTER, recenter.gimbalTarget)
        assertEquals(TapeTrackingPhase.TRACKING, controller.tick(seconds(10) + 200L).phase)
    }

    @Test
    fun `twenty second search timeout stops tracking and centers camera`() {
        val controller = trackingController()
        controller.tick(seconds(7))

        val timeout = controller.tick(seconds(27))
        assertTrue(timeout.searchTimedOut)
        assertEquals(TrackingGimbalTarget.DOWN_CENTER, timeout.gimbalTarget)
        assertEquals(0.0, timeout.yawRateDegreesPerSecond, 0.0)
        assertEquals(0.0, timeout.forwardSpeedMetersPerSecond, 0.0)
        assertFalse(controller.enabled)
    }

    private fun trackingController(): TapeTrackingController = TapeTrackingController().also {
        it.start(0L)
        assertEquals(TrackingGimbalTarget.DOWN_CENTER, it.tick(0L).gimbalTarget)
        assertEquals(TapeTrackingPhase.TRACKING, it.tick(seconds(2)).phase)
    }

    private fun seconds(value: Long): Long = value * 1_000_000_000L
}
