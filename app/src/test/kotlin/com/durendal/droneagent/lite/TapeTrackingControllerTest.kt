package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

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
    fun `persistent short tape does not strand straight endpoint verification`() {
        val controller = trackingController()
        enterEndpointVerification(controller)
        val persistentShortTape = observation(0.0, 0.35)
        controller.observe(persistentShortTape, seconds(5))
        controller.observe(persistentShortTape, seconds(8))

        val timedOut = controller.tick(seconds(10))

        assertTrue(timedOut.stopRequested)
        assertEquals(0.0, timedOut.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, timedOut.rightSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, timedOut.yawRateDegreesPerSecond, 0.0)
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
    fun `turn completion resets endpoint baseline before tracking resumes`() {
        val controller = trackingController()
        enterEndpointVerification(controller)
        confirmEndpointDisappearance(controller, seconds(6))

        controller.resumeAfterTurn(seconds(7))
        val recenter = controller.tick(seconds(7))
        assertEquals(TapeTrackingPhase.RECENTERING, recenter.phase)
        assertEquals(
            TapeTrackingPhase.RECOVERING_AFTER_TURN,
            controller.tick(seconds(9)).phase,
        )
        controller.observe(observation(0.0, 0.35), seconds(9))
        val resumed = controller.tick(seconds(9))
        assertEquals(TapeTrackingPhase.TRACKING, resumed.phase)
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
        // The reacquisition move is its own bounded phase now, so an advance with
        // nothing detected can never be mistaken for following a path.
        val recovery = controller.tick(seconds(9))
        assertEquals(TapeTrackingPhase.RECOVERING_AFTER_TURN, recovery.phase)
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
    fun `controller filters axial angle across the ninety degree seam`() {
        val controller = trackingController()
        controller.observe(observation(89.0, 0.8), seconds(2))
        controller.tick(seconds(2))

        controller.observe(observation(-89.0, 0.8), seconds(2) + 250_000_000L)
        val decision = controller.tick(seconds(2) + 250_000_000L)

        assertTrue(decision.controlledAngleDegrees!! > 80.0)
        assertTrue(decision.yawRateDegreesPerSecond > 0.0)
        assertEquals(0.0, decision.forwardSpeedMetersPerSecond, 0.0)
    }

    @Test
    fun `sharp curve alignment rotates in place without lateral translation`() {
        val controller = circularTrackingController()
        controller.observe(observation(-82.0, 0.8, 0.25), seconds(2))

        val first = controller.tick(seconds(2))
        val accelerated = controller.tick(seconds(2) + 250_000_000L)

        assertEquals(TapeTrackingPhase.ALIGNING_CURVE, first.phase)
        assertEquals(0.0, first.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, first.rightSpeedMetersPerSecond, 0.0)
        assertTrue(first.yawRateDegreesPerSecond < 0.0)
        assertTrue(
            kotlin.math.abs(accelerated.yawRateDegreesPerSecond) >
                TapeTrackingController.ANCHOR_ACQUISITION_MAX_YAW_RATE_DEGREES_PER_SECOND,
        )
        assertTrue(
            kotlin.math.abs(accelerated.yawRateDegreesPerSecond) <=
                TapeTrackingController.CIRCULAR_MAX_YAW_RATE_DEGREES_PER_SECOND,
        )
    }

    @Test
    fun `sharp curve alignment requires three stable detections before moving`() {
        val controller = circularTrackingController()
        controller.observe(observation(46.0, 0.8), seconds(2))
        assertEquals(TapeTrackingPhase.ALIGNING_CURVE, controller.tick(seconds(2)).phase)

        val samples = listOf(
            seconds(2) + 250_000_000L,
            seconds(2) + 500_000_000L,
            seconds(2) + 750_000_000L,
        )
        samples.forEach { now ->
            controller.observe(observation(0.0, 0.8), now)
            assertEquals(TapeTrackingPhase.ALIGNING_CURVE, controller.tick(now).phase)
        }

        val alignedAt = seconds(3)
        controller.observe(observation(0.0, 0.8), alignedAt)
        val resumed = controller.tick(alignedAt)

        assertEquals(TapeTrackingPhase.TRACKING, resumed.phase)
        assertEquals(
            TapeTrackingController.CIRCULAR_TRACKING_FORWARD_SPEED_METERS_PER_SECOND,
            resumed.forwardSpeedMetersPerSecond,
            0.0,
        )
    }

    @Test
    fun `sharp curve alignment hovers immediately when detection is lost`() {
        val controller = circularTrackingController()
        controller.observe(observation(55.0, 0.8), seconds(2))
        assertTrue(controller.tick(seconds(2)).yawRateDegreesPerSecond > 0.0)

        val missingAt = seconds(2) + 250_000_000L
        controller.observe(null, missingAt)
        val hovering = controller.tick(missingAt)

        assertEquals(TapeTrackingPhase.ALIGNING_CURVE, hovering.phase)
        assertEquals(0.0, hovering.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, hovering.rightSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, hovering.yawRateDegreesPerSecond, 0.0)
    }

    @Test
    fun `distant edge candidate cannot take control after a detection gap`() {
        val controller = circularTrackingController()
        controller.observe(observation(18.0, 0.8, 0.02), seconds(2))
        controller.observe(null, seconds(3))

        val edgeCandidates = listOf(
            rightEdgeObservation(67.4, 0.34),
            rightEdgeObservation(8.0, 0.21),
            rightEdgeObservation(-71.6, 0.21),
            rightEdgeObservation(-60.9, 1.56),
        )
        edgeCandidates.forEachIndexed { index, candidate ->
            val now = seconds(6) + index * 250_000_000L
            controller.observe(candidate, now)
            val decision = controller.tick(now)

            assertEquals(TapeTrackingPhase.REACQUIRING_PATH, decision.phase)
            assertEquals(0.0, decision.forwardSpeedMetersPerSecond, 0.0)
            assertEquals(0.0, decision.rightSpeedMetersPerSecond, 0.0)
            assertEquals(0.0, decision.yawRateDegreesPerSecond, 0.0)
        }
    }

    @Test
    fun `initial edge candidate cannot take circular control`() {
        val controller = circularTrackingController()

        controller.observe(rightEdgeObservation(67.4, 0.34), seconds(2))
        val decision = controller.tick(seconds(2))

        assertEquals(TapeTrackingPhase.REACQUIRING_PATH, decision.phase)
        assertEquals(0.0, decision.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, decision.rightSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, decision.yawRateDegreesPerSecond, 0.0)
    }

    @Test
    fun `three consistent centered candidates reacquire a new circular path`() {
        val controller = circularTrackingController()
        controller.observe(observation(80.0, 0.8, 0.02), seconds(2))
        controller.observe(null, seconds(3))
        controller.observe(rightEdgeObservation(67.4, 0.34), seconds(6))
        assertEquals(
            TapeTrackingPhase.REACQUIRING_PATH,
            controller.tick(seconds(6)).phase,
        )

        val centeredCandidates = listOf(
            observation(6.0, 0.75, 0.06),
            observation(5.0, 0.77, 0.04),
            observation(4.0, 0.79, 0.05),
        )
        var reacquired: TapeTrackingDecision? = null
        centeredCandidates.forEachIndexed { index, candidate ->
            val now = seconds(7) + index * 250_000_000L
            controller.observe(candidate, now)
            reacquired = controller.tick(now)
            if (index < centeredCandidates.lastIndex) {
                assertEquals(TapeTrackingPhase.REACQUIRING_PATH, reacquired?.phase)
            }
        }

        val decision = checkNotNull(reacquired)
        assertEquals(TapeTrackingPhase.TRACKING, decision.phase)
        assertEquals(
            TapeTrackingController.CIRCULAR_TRACKING_FORWARD_SPEED_METERS_PER_SECOND,
            decision.forwardSpeedMetersPerSecond,
            0.0,
        )
    }

    @Test
    fun `brief detector flicker does not strand circular path reacquisition`() {
        val controller = circularTrackingController()
        controller.observe(observation(80.0, 0.8, 0.02), seconds(2))
        controller.observe(null, seconds(3))
        controller.observe(rightEdgeObservation(67.4, 0.34), seconds(4))

        val firstCandidateAt = seconds(5)
        controller.observe(observation(-60.0, 0.90, -0.16), firstCandidateAt)
        controller.observe(null, firstCandidateAt + 200_000_000L)
        controller.observe(
            observation(-62.0, 0.95, -0.14),
            firstCandidateAt + 400_000_000L,
        )
        controller.observe(
            observation(-50.0, 0.35, -0.12),
            firstCandidateAt + 600_000_000L,
        )
        controller.observe(
            observation(-64.0, 0.99, -0.18),
            firstCandidateAt + 800_000_000L,
        )

        val reacquired = controller.tick(firstCandidateAt + 800_000_000L)
        assertEquals(TapeTrackingPhase.ALIGNING_CURVE, reacquired.phase)
        assertTrue(reacquired.yawRateDegreesPerSecond < 0.0)
        assertEquals(0.0, reacquired.forwardSpeedMetersPerSecond, 0.0)
    }

    @Test
    fun `stale circular reacquisition candidate still requires three fresh detections`() {
        val controller = circularTrackingController()
        controller.observe(observation(80.0, 0.8, 0.02), seconds(2))
        controller.observe(null, seconds(3))
        controller.observe(rightEdgeObservation(67.4, 0.34), seconds(4))

        val firstCandidateAt = seconds(5)
        controller.observe(observation(-60.0, 0.90, -0.16), firstCandidateAt)
        controller.observe(
            null,
            firstCandidateAt +
                TapeTrackingController.REACQUISITION_CANDIDATE_MAX_GAP_NANOS +
                1L,
        )
        repeat(2) { index ->
            controller.observe(
                observation(-62.0, 0.95, -0.14),
                seconds(6) + index * 200_000_000L,
            )
        }

        val awaitingThirdCandidate = controller.tick(seconds(6) + 200_000_000L)
        assertEquals(TapeTrackingPhase.REACQUIRING_PATH, awaitingThirdCandidate.phase)
        assertEquals(0.0, awaitingThirdCandidate.yawRateDegreesPerSecond, 0.0)
        assertEquals(0.0, awaitingThirdCandidate.forwardSpeedMetersPerSecond, 0.0)
    }

    @Test
    fun `stable near-edge path reacquires laterally without moving forward`() {
        val controller = circularTrackingController()
        controller.observe(observation(18.0, 0.8, 0.02), seconds(2))
        controller.observe(null, seconds(3))
        controller.observe(rightEdgeObservation(67.4, 0.34), seconds(4))
        assertEquals(
            TapeTrackingPhase.REACQUIRING_PATH,
            controller.tick(seconds(4)).phase,
        )

        val nearEdgePath = observation(
            angleDegrees = 20.0,
            longSideFraction = 1.10,
            nearFieldOffsetFraction = -0.48,
            bounds = NormalizedRect(0.0, 0.05, 0.90, 1.0),
            lookahead = TapeLookahead(xFraction = 0.75, yFraction = 0.30),
        )
        var reacquired: TapeTrackingDecision? = null
        repeat(3) { index ->
            val now = seconds(5) + index * 250_000_000L
            controller.observe(nearEdgePath, now)
            reacquired = controller.tick(now)
        }

        val decision = checkNotNull(reacquired)
        assertEquals(TapeTrackingPhase.TRACKING, decision.phase)
        assertEquals(0.0, decision.forwardSpeedMetersPerSecond, 0.0)
        assertTrue(decision.rightSpeedMetersPerSecond < 0.0)
    }

    @Test
    fun `short centered floor features cannot reacquire a circular path`() {
        val controller = circularTrackingController()
        controller.observe(observation(18.0, 0.8, 0.02), seconds(2))
        controller.observe(null, seconds(3))
        controller.observe(rightEdgeObservation(67.4, 0.34), seconds(4))

        val floorFeature = observation(
            angleDegrees = 50.0,
            longSideFraction = 0.35,
            nearFieldOffsetFraction = 0.01,
            bounds = NormalizedRect(0.46, 0.58, 0.56, 0.73),
            lookahead = TapeLookahead(xFraction = 0.51, yFraction = 0.67),
        )
        repeat(4) { index ->
            val now = seconds(5) + index * 250_000_000L
            controller.observe(floorFeature, now)
            val decision = controller.tick(now)
            assertEquals(TapeTrackingPhase.REACQUIRING_PATH, decision.phase)
            assertEquals(0.0, decision.forwardSpeedMetersPerSecond, 0.0)
            assertEquals(0.0, decision.rightSpeedMetersPerSecond, 0.0)
            assertEquals(0.0, decision.yawRateDegreesPerSecond, 0.0)
        }
    }

    @Test
    fun `long connected path after a gap cannot become a circular endpoint`() {
        val controller = circularTrackingController()
        repeat(3) { index ->
            controller.observe(
                observation(4.0, 0.8, 0.02),
                seconds(2) + index * 250_000_000L,
            )
        }
        controller.observe(null, seconds(3))

        val shiftedPath = observation(
            angleDegrees = 60.0,
            longSideFraction = 1.60,
            nearFieldOffsetFraction = 0.35,
            bounds = NormalizedRect(0.10, 0.51, 0.74, 1.0),
            lookahead = TapeLookahead(xFraction = 0.65, yFraction = 0.65),
        )
        controller.observe(shiftedPath, seconds(3) + 250_000_000L)
        val decision = controller.tick(seconds(3) + 250_000_000L)

        assertEquals(TapeTrackingPhase.REACQUIRING_PATH, decision.phase)
        assertEquals(0.0, decision.forwardSpeedMetersPerSecond, 0.0)
        assertFalse(decision.endpointReached)
    }

    @Test
    fun `plausible continuation requires two consistent detections after a gap`() {
        val controller = circularTrackingController()
        controller.observe(observation(18.0, 0.8, 0.02), seconds(2))
        controller.observe(null, seconds(3))

        controller.observe(observation(20.0, 0.82, 0.05), seconds(4))
        val awaitingConfirmation = controller.tick(seconds(4))
        assertEquals(TapeTrackingPhase.TRACKING, awaitingConfirmation.phase)
        assertEquals(0.0, awaitingConfirmation.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, awaitingConfirmation.rightSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, awaitingConfirmation.yawRateDegreesPerSecond, 0.0)

        controller.observe(observation(19.0, 0.81, 0.04), seconds(4) + 250_000_000L)
        val resumed = controller.tick(seconds(4) + 250_000_000L)
        assertEquals(TapeTrackingPhase.TRACKING, resumed.phase)
        assertTrue(resumed.forwardSpeedMetersPerSecond > 0.0)
    }

    @Test
    fun `detection loss restarts gap continuation confirmation`() {
        val controller = circularTrackingController()
        controller.observe(observation(18.0, 0.8, 0.02), seconds(2))
        controller.observe(null, seconds(3))
        controller.observe(observation(20.0, 0.82, 0.05), seconds(4))
        controller.observe(null, seconds(4) + 250_000_000L)

        controller.observe(observation(19.0, 0.81, 0.04), seconds(5))
        val restarted = controller.tick(seconds(5))
        assertEquals(0.0, restarted.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, restarted.rightSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, restarted.yawRateDegreesPerSecond, 0.0)

        controller.observe(observation(20.0, 0.82, 0.05), seconds(5) + 250_000_000L)
        assertTrue(
            controller.tick(seconds(5) + 250_000_000L).forwardSpeedMetersPerSecond > 0.0,
        )
    }


    @Test
    fun `circular mode follows the local tangent at experiment speed`() {
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
    fun `circular mode advances into a hairpin using pure pursuit`() {
        val controller = circularTrackingController()
        controller.observe(
            observation(
                angleDegrees = 0.0,
                longSideFraction = 1.8,
                lookahead = TapeLookahead(xFraction = 0.28, yFraction = 0.55),
                heightAboveGroundMeters = 0.5,
            ),
            seconds(2),
        )

        val decision = controller.tick(seconds(2))

        assertEquals(
            TapeTrackingController.CIRCULAR_TRACKING_FORWARD_SPEED_METERS_PER_SECOND,
            decision.forwardSpeedMetersPerSecond,
            0.0,
        )
        assertTrue(decision.purePursuitYawRateDegreesPerSecond < 0.0)
        assertTrue(decision.yawRateDegreesPerSecond < 0.0)
    }

    @Test
    fun `circular mode holds a brief miss then hovers when detection becomes stale`() {
        val controller = circularTrackingController()
        controller.observe(observation(12.0, 0.8, 0.05), seconds(2))
        controller.tick(seconds(2))

        controller.observe(null, seconds(2) + 250_000_000L)
        val briefMiss = controller.tick(seconds(2) + 500_000_000L)
        val stale = controller.tick(seconds(3) + 250_000_000L)

        assertFalse(briefMiss.endpointReached)
        assertEquals(TapeTrackingPhase.TRACKING, briefMiss.phase)
        assertTrue(briefMiss.forwardSpeedMetersPerSecond > 0.0)
        assertTrue(briefMiss.yawRateDegreesPerSecond > 0.0)
        assertEquals(TapeTrackingPhase.TRACKING, stale.phase)
        assertEquals(0.0, stale.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, stale.rightSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, stale.yawRateDegreesPerSecond, 0.0)
    }

    @Test
    fun `pure pursuit turns toward an offset forward target while tangent is aligned`() {
        val rightController = trackingController()
        rightController.observe(
            observation(
                angleDegrees = 0.0,
                longSideFraction = 0.8,
                lookahead = TapeLookahead(xFraction = 0.65, yFraction = 0.60),
                heightAboveGroundMeters = 0.5,
            ),
            seconds(2),
        )
        val right = rightController.tick(seconds(2))

        val leftController = trackingController()
        leftController.observe(
            observation(
                angleDegrees = 0.0,
                longSideFraction = 0.8,
                lookahead = TapeLookahead(xFraction = 0.35, yFraction = 0.60),
                heightAboveGroundMeters = 0.5,
            ),
            seconds(2),
        )
        val left = leftController.tick(seconds(2))

        assertTrue(right.purePursuitYawRateDegreesPerSecond > 0.0)
        assertTrue(right.yawRateDegreesPerSecond > 0.0)
        assertEquals(
            -right.purePursuitYawRateDegreesPerSecond,
            left.purePursuitYawRateDegreesPerSecond,
            1e-9,
        )
        assertTrue(left.yawRateDegreesPerSecond < 0.0)
    }

    @Test
    fun `pure pursuit is neutral for a centered target`() {
        val controller = trackingController()
        controller.observe(
            observation(
                angleDegrees = 0.0,
                longSideFraction = 0.8,
                lookahead = TapeLookahead(xFraction = 0.5, yFraction = 0.60),
                heightAboveGroundMeters = 0.5,
            ),
            seconds(2),
        )

        val decision = controller.tick(seconds(2))

        assertEquals(0.0, decision.purePursuitYawRateDegreesPerSecond, 0.0)
        assertEquals(0.0, decision.yawRateDegreesPerSecond, 0.0)
    }

    @Test
    fun `missing height falls back to tangent feedback without fake metric curvature`() {
        val controller = trackingController()
        controller.observe(
            observation(
                angleDegrees = 0.0,
                longSideFraction = 0.8,
                lookahead = TapeLookahead(xFraction = 0.65, yFraction = 0.60),
                heightAboveGroundMeters = null,
            ),
            seconds(2),
        )

        val decision = controller.tick(seconds(2))

        assertEquals(0.0, decision.purePursuitYawRateDegreesPerSecond, 0.0)
        assertEquals(0.0, decision.yawRateDegreesPerSecond, 0.0)
    }

    @Test
    fun `circular mode does not land when no path was ever detected`() {
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
    fun `circular mode ignores the straight tape endpoint signature`() {
        val controller = circularTrackingController()
        controller.observe(observation(0.0, 0.8), seconds(2))
        controller.observe(observation(0.0, 0.35), seconds(3))
        controller.observe(observation(0.0, 0.35), seconds(3) + 400_000_000L)
        controller.observe(observation(0.0, 0.35), seconds(3) + 800_000_000L)
        repeat(4) { index ->
            controller.observe(null, seconds(4) + index * 250_000_000L)
        }

        val decision = controller.tick(seconds(5))

        assertFalse(decision.endpointReached)
        assertEquals(TapeTrackingPhase.TRACKING, decision.phase)
    }

    @Test
    fun `stable circular path disappearance triggers a low speed endpoint probe and turnaround`() {
        val controller = circularTrackingController()
        repeat(3) { index ->
            controller.observe(
                observation(4.0, 0.8, 0.02),
                seconds(2) + index * 250_000_000L,
            )
        }
        repeat(3) { index ->
            controller.observe(null, seconds(3) + index * 250_000_000L)
        }

        val probing = controller.tick(seconds(3) + 500_000_000L)
        assertEquals(TapeTrackingPhase.VERIFYING_ENDPOINT, probing.phase)
        assertEquals(
            TapeTrackingController.CIRCULAR_ENDPOINT_PROBE_SPEED_METERS_PER_SECOND,
            probing.forwardSpeedMetersPerSecond,
            0.0,
        )

        repeat(3) { index ->
            controller.observe(null, seconds(5) + index * 250_000_000L)
        }
        val turning = controller.tick(seconds(5) + 500_000_000L)

        assertTrue(turning.endpointReached)
        assertEquals(TapeTrackingPhase.TURNING, turning.phase)

        controller.resumeAfterTurn(seconds(6))
        assertEquals(TapeTrackingPhase.RECENTERING, controller.tick(seconds(6)).phase)
        val recovery = controller.tick(seconds(8))
        assertEquals(TapeTrackingPhase.RECOVERING_AFTER_TURN, recovery.phase)
        assertEquals(
            TapeTrackingController.CIRCULAR_POST_TURN_RECOVERY_SPEED_METERS_PER_SECOND,
            recovery.forwardSpeedMetersPerSecond,
            0.0,
        )
    }

    @Test
    fun `post turn recovery advances past a false edge candidate`() {
        val controller = circularControllerAfterEndpointTurn()
        val recovery = controller.tick(seconds(8))
        assertEquals(TapeTrackingPhase.RECOVERING_AFTER_TURN, recovery.phase)

        controller.observe(rightEdgeObservation(67.4, 0.34), seconds(8) + 250_000_000L)
        val ignoringEdge = controller.tick(seconds(8) + 250_000_000L)

        assertEquals(TapeTrackingPhase.RECOVERING_AFTER_TURN, ignoringEdge.phase)
        assertEquals(
            TapeTrackingController.CIRCULAR_POST_TURN_RECOVERY_SPEED_METERS_PER_SECOND,
            ignoringEdge.forwardSpeedMetersPerSecond,
            0.0,
        )
        assertEquals(0.0, ignoringEdge.rightSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, ignoringEdge.yawRateDegreesPerSecond, 0.0)
    }

    @Test
    fun `three centered path observations end post turn recovery`() {
        val controller = circularControllerAfterEndpointTurn()
        controller.tick(seconds(8))
        val centeredPath = observation(
            angleDegrees = 5.0,
            longSideFraction = 0.40,
            nearFieldOffsetFraction = 0.02,
            bounds = NormalizedRect(0.45, 0.10, 0.58, 0.58),
            lookahead = TapeLookahead(xFraction = 0.52, yFraction = 0.25),
        )

        repeat(3) { index ->
            controller.observe(centeredPath, seconds(8) + (index + 1) * 250_000_000L)
        }
        val tracking = controller.tick(seconds(8) + 750_000_000L)

        assertEquals(TapeTrackingPhase.TRACKING, tracking.phase)
        assertEquals(
            TapeTrackingController.CIRCULAR_TRACKING_FORWARD_SPEED_METERS_PER_SECOND,
            tracking.forwardSpeedMetersPerSecond,
            0.0,
        )
    }

    @Test
    fun `post turn recovery stops after its bounded forward search`() {
        val controller = circularControllerAfterEndpointTurn()
        controller.tick(seconds(8))

        val stopped = controller.tick(
            seconds(8) + TapeTrackingController.CIRCULAR_POST_TURN_RECOVERY_NANOS,
        )

        assertEquals(TapeTrackingPhase.REACQUIRING_PATH, stopped.phase)
        assertEquals(0.0, stopped.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, stopped.rightSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, stopped.yawRateDegreesPerSecond, 0.0)
    }

    @Test
    fun `closed loop experiment ignores circular endpoint disappearance`() {
        val controller = TapeTrackingController()
        controller.start(
            nowNanos = 0L,
            mode = TapeTrackingMode.CIRCULAR,
            endpointTurnEnabled = false,
        )
        assertEquals(TapeTrackingPhase.TRACKING, controller.tick(seconds(2)).phase)
        repeat(3) { index ->
            controller.observe(
                observation(4.0, 0.8, 0.02),
                seconds(2) + index * 250_000_000L,
            )
        }
        repeat(4) { index ->
            controller.observe(null, seconds(3) + index * 250_000_000L)
        }

        val decision = controller.tick(seconds(4))

        assertFalse(decision.endpointReached)
        assertEquals(TapeTrackingPhase.TRACKING, decision.phase)
        assertEquals(0.0, decision.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, decision.rightSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, decision.yawRateDegreesPerSecond, 0.0)
    }

    @Test
    fun `false edge after a qualified circular path starts endpoint verification`() {
        val controller = circularTrackingController()
        repeat(3) { index ->
            controller.observe(
                observation(4.0, 0.8, 0.02),
                seconds(2) + index * 250_000_000L,
            )
        }
        controller.observe(null, seconds(3))

        controller.observe(rightEdgeObservation(67.4, 0.34), seconds(3) + 250_000_000L)
        val decision = controller.tick(seconds(3) + 250_000_000L)

        assertEquals(TapeTrackingPhase.VERIFYING_ENDPOINT, decision.phase)
        assertEquals(
            TapeTrackingController.CIRCULAR_ENDPOINT_PROBE_SPEED_METERS_PER_SECOND,
            decision.forwardSpeedMetersPerSecond,
            0.0,
        )
        assertEquals(0.0, decision.rightSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, decision.yawRateDegreesPerSecond, 0.0)
    }

    @Test
    fun `circular endpoint probe is cancelled when the tracked path returns`() {
        val controller = circularTrackingController()
        repeat(3) { index ->
            controller.observe(
                observation(4.0, 0.8, 0.02),
                seconds(2) + index * 250_000_000L,
            )
        }
        repeat(3) { index ->
            controller.observe(null, seconds(3) + index * 250_000_000L)
        }
        assertEquals(
            TapeTrackingPhase.VERIFYING_ENDPOINT,
            controller.tick(seconds(3) + 500_000_000L).phase,
        )

        controller.observe(observation(5.0, 0.82, 0.03), seconds(4))
        val resumed = controller.tick(seconds(4))

        assertFalse(resumed.endpointReached)
        assertEquals(TapeTrackingPhase.TRACKING, resumed.phase)
        assertTrue(resumed.forwardSpeedMetersPerSecond > 0.0)
    }

    @Test
    fun `circular endpoint verification times out to stationary reacquisition`() {
        val controller = circularTrackingController()
        repeat(3) { index ->
            controller.observe(
                observation(4.0, 0.8, 0.02),
                seconds(2) + index * 250_000_000L,
            )
        }
        val verificationStartedAt = seconds(3) + 250_000_000L
        controller.observe(null, seconds(3))
        controller.observe(null, verificationStartedAt)
        assertEquals(
            TapeTrackingPhase.VERIFYING_ENDPOINT,
            controller.tick(verificationStartedAt).phase,
        )

        val persistentShortPath = observation(4.0, 0.30, 0.02)
        controller.observe(persistentShortPath, seconds(4))
        controller.observe(persistentShortPath, seconds(6))
        val timedOut = controller.tick(
            verificationStartedAt +
                TapeTrackingController.ENDPOINT_VERIFICATION_TIMEOUT_NANOS,
        )

        assertFalse(timedOut.endpointReached)
        assertEquals(TapeTrackingPhase.REACQUIRING_PATH, timedOut.phase)
        assertEquals(0.0, timedOut.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, timedOut.rightSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, timedOut.yawRateDegreesPerSecond, 0.0)
    }


    private fun circularControllerAfterEndpointTurn(): TapeTrackingController {
        val controller = circularTrackingController()
        repeat(3) { index ->
            controller.observe(
                observation(4.0, 0.8, 0.02),
                seconds(2) + index * 250_000_000L,
            )
        }
        repeat(3) { index ->
            controller.observe(null, seconds(3) + index * 250_000_000L)
        }
        repeat(3) { index ->
            controller.observe(null, seconds(5) + index * 250_000_000L)
        }
        assertTrue(controller.tick(seconds(5) + 500_000_000L).endpointReached)
        controller.resumeAfterTurn(seconds(6))
        assertEquals(TapeTrackingPhase.RECENTERING, controller.tick(seconds(6)).phase)
        return controller
    }

    private fun circularTrackingController(): TapeTrackingController =
        TapeTrackingController().also {
            it.start(0L, TapeTrackingMode.CIRCULAR)
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
        assertEquals(TapeTrackingPhase.TRACKING, it.tick(seconds(2)).phase)
    }

    private fun observation(
        angleDegrees: Double,
        longSideFraction: Double,
        nearFieldOffsetFraction: Double = 0.0,
        bounds: NormalizedRect = verticalBounds(longSideFraction, nearFieldOffsetFraction),
        lookahead: TapeLookahead? = TapeLookahead(xFraction = 0.5, yFraction = 0.60),
        quality: PathQuality =
            if (lookahead == null) PathQuality.NEAR_FIELD_ONLY else PathQuality.FULL_PATH,
        endpointCandidate: Boolean = true,
        closedLoop: Boolean = false,
        heightAboveGroundMeters: Double? = null,
    ) = TapeTrackingObservation(
        angleFromVerticalDegrees = angleDegrees,
        longSideFraction = longSideFraction,
        nearFieldOffsetFraction = nearFieldOffsetFraction,
        bounds = bounds,
        lookahead = lookahead,
        quality = quality,
        endpointCandidate = endpointCandidate,
        closedLoop = closedLoop,
        frameWidthPixels = 1600,
        frameHeightPixels = 900,
        heightAboveGroundMeters = heightAboveGroundMeters,
    )

    private fun rightEdgeObservation(
        angleDegrees: Double,
        longSideFraction: Double,
    ): TapeTrackingObservation = observation(
        angleDegrees = angleDegrees,
        longSideFraction = longSideFraction,
        nearFieldOffsetFraction = 0.493,
        bounds = NormalizedRect(0.90, 0.64, 1.0, 0.83),
        lookahead = TapeLookahead(xFraction = 0.966, yFraction = 0.714),
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
