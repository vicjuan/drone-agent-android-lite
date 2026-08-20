package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * What each path quality is allowed to command.
 *
 * These assert the final [TapeTrackingDecision] — the numbers that reach the
 * aircraft — rather than an enum or a nullable field. A gate that is present in
 * the type system but absent from the output is not a gate.
 */
class TapeQualityCommandTest {

    @Test
    fun `a full path may translate and pursue`() {
        val controller = tracking()
        settle(controller, fullPath())

        val decision = controller.tick(seconds(6))

        assertEquals(PathQuality.FULL_PATH, decision.pathQuality)
        assertTrue(
            "a full path should be allowed to advance: ${decision.forwardSpeedMetersPerSecond}",
            decision.forwardSpeedMetersPerSecond > 0.0,
        )
    }

    @Test
    fun `a full path off centre corrects laterally`() {
        val controller = tracking()
        settle(controller, fullPath(offset = 0.20))

        val decision = controller.tick(seconds(6))

        assertNotEquals(
            "an offset full path should translate to recentre",
            0.0,
            decision.rightSpeedMetersPerSecond,
            0.0,
        )
    }

    @Test
    fun `near field only stops translating in the same frame`() {
        val controller = tracking()
        settle(controller, fullPath())
        val moving = controller.tick(seconds(6))
        assertTrue("precondition: the full path was advancing", moving.forwardSpeedMetersPerSecond > 0.0)

        // Offset far enough that a full path would be translating to recentre,
        // so a missing gate shows up as movement rather than as a coincidence.
        controller.observe(nearFieldOnly(offset = 0.25), seconds(7))
        val decision = controller.tick(seconds(7))

        assertEquals(PathQuality.NEAR_FIELD_ONLY, decision.pathQuality)
        assertEquals(0.0, decision.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, decision.rightSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, decision.purePursuitYawRateDegreesPerSecond, 0.0)
    }

    @Test
    fun `near field only may still align in place within a bound`() {
        val controller = tracking()
        settle(controller, fullPath())
        // A large angle: unbounded proportional yaw would exceed the near-field
        // cap, so the cap is what the assertion actually measures.
        var now = 6L
        var decision = controller.tick(seconds(now))
        repeat(12) {
            now++
            controller.observe(nearFieldOnly(angleDegrees = 40.0), seconds(now))
            decision = controller.tick(seconds(now))
        }

        assertEquals(0.0, decision.forwardSpeedMetersPerSecond, 0.0)
        assertTrue(
            "an in-place alignment is allowed: ${decision.yawRateDegreesPerSecond}",
            decision.yawRateDegreesPerSecond != 0.0,
        )
        assertTrue(
            "and it must stay bounded: ${decision.yawRateDegreesPerSecond}",
            kotlin.math.abs(decision.yawRateDegreesPerSecond) <=
                TapeTrackingController.NEAR_FIELD_MAX_YAW_RATE_DEGREES_PER_SECOND,
        )
    }

    @Test
    fun `losing the path zeroes every command`() {
        val controller = tracking()
        settle(controller, fullPath())
        assertTrue(controller.tick(seconds(6)).forwardSpeedMetersPerSecond > 0.0)

        controller.observe(null, seconds(7))
        val decision = controller.tick(seconds(7))

        assertEquals(PathQuality.LOST, decision.pathQuality)
        assertEquals(0.0, decision.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, decision.rightSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, decision.yawRateDegreesPerSecond, 0.0)
        assertEquals(0.0, decision.purePursuitYawRateDegreesPerSecond, 0.0)
    }

    @Test
    fun `near field only then lost is still all zero`() {
        val controller = tracking()
        settle(controller, fullPath())
        controller.observe(nearFieldOnly(angleDegrees = 8.0), seconds(7))
        controller.tick(seconds(7))

        controller.observe(null, seconds(8))
        val decision = controller.tick(seconds(8))

        assertEquals(PathQuality.LOST, decision.pathQuality)
        assertEquals(0.0, decision.yawRateDegreesPerSecond, 0.0)
        assertEquals(0.0, decision.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, decision.rightSpeedMetersPerSecond, 0.0)
    }

    @Test
    fun `a path regained after a loss does not reuse the stale look-ahead`() {
        val controller = tracking()
        settle(controller, fullPath())
        controller.observe(null, seconds(7))
        val lost = controller.tick(seconds(7))
        assertEquals(0.0, lost.purePursuitYawRateDegreesPerSecond, 0.0)

        // Regained on the other side of the frame: a controller that kept the old
        // filtered look-ahead would steer toward where the tape used to be.
        controller.observe(fullPath(offset = -0.20, lookaheadX = 0.20), seconds(8))
        val regained = controller.tick(seconds(8))

        assertEquals(PathQuality.FULL_PATH, regained.pathQuality)
        assertEquals(
            "the filtered offset must start from this frame, not the old one",
            -0.20,
            regained.controlledOffsetFraction ?: 0.0,
            0.001,
        )
    }

    @Test
    fun `a shortening in-frame terminus does confirm an endpoint`() {
        // The positive control for the border-truncation case below: without it,
        // that test would pass simply because nothing ever confirms an endpoint.
        assertTrue(
            "an in-frame terminus must be able to confirm",
            runEndpointSequence(endpointCandidate = true),
        )
    }

    @Test
    fun `a branch path never advances`() {
        val controller = tracking()
        settle(controller, fullPath())

        // A junction reports near-field-only with no look-ahead, which is exactly
        // how the quality policy encodes "the extractor picked an arm".
        controller.observe(nearFieldOnly(), seconds(7))
        val decision = controller.tick(seconds(7))

        assertEquals(0.0, decision.forwardSpeedMetersPerSecond, 0.0)
        assertEquals(0.0, decision.rightSpeedMetersPerSecond, 0.0)
    }

    @Test
    fun `a shortening path truncated by the frame border is not an endpoint`() {
        // Exactly the sequence that confirms an endpoint above, except the chain
        // leaves the frame: the tape ran out of view, not out of tape.
        assertTrue(
            "a border truncation must never request a turn",
            !runEndpointSequence(endpointCandidate = false),
        )
    }

    /** Shortens a tracked path, then loses it, and reports whether a turn was requested. */
    private fun runEndpointSequence(endpointCandidate: Boolean): Boolean {
        val controller = tracking()
        settle(controller, fullPath(longSideFraction = 1.0))
        var reached = false
        var now = 6L
        repeat(5) {
            now++
            controller.observe(
                fullPath(longSideFraction = 0.5, endpointCandidate = endpointCandidate),
                seconds(now),
            )
            if (controller.tick(seconds(now)).endpointReached) reached = true
        }
        repeat(6) {
            now++
            controller.observe(null, seconds(now))
            if (controller.tick(seconds(now)).endpointReached) reached = true
        }
        return reached
    }

    @Test
    fun `a closed loop in circular mode is never an endpoint`() {
        val controller = TapeTrackingController().apply {
            start(seconds(0), TapeTrackingMode.CIRCULAR)
        }
        var now = 3L
        repeat(10) {
            now++
            controller.observe(
                fullPath(longSideFraction = 0.9, endpointCandidate = false, closedLoop = true),
                seconds(now),
            )
            assertTrue(
                "a loop has no end to reach",
                !controller.tick(seconds(now)).endpointReached,
            )
        }
        repeat(6) {
            now++
            controller.observe(null, seconds(now))
            assertTrue(
                "losing sight of a loop is not reaching its end",
                !controller.tick(seconds(now)).endpointReached,
            )
        }
    }

    private fun tracking() = TapeTrackingController().apply { start(seconds(0)) }

    /** Runs past recentring so the controller is in TRACKING with filters settled. */
    private fun settle(controller: TapeTrackingController, observation: TapeTrackingObservation) {
        controller.tick(seconds(3))
        for (second in 3..5) {
            controller.observe(observation, seconds(second.toLong()))
            controller.tick(seconds(second.toLong()))
        }
        controller.observe(observation, seconds(6))
    }

    private fun fullPath(
        angleDegrees: Double = 0.0,
        offset: Double = 0.0,
        longSideFraction: Double = 0.9,
        lookaheadX: Double = 0.5,
        endpointCandidate: Boolean = true,
        closedLoop: Boolean = false,
    ) = observation(
        angleDegrees = angleDegrees,
        offset = offset,
        longSideFraction = longSideFraction,
        lookahead = TapeLookahead(xFraction = lookaheadX, yFraction = 0.55),
        quality = PathQuality.FULL_PATH,
        endpointCandidate = endpointCandidate,
        closedLoop = closedLoop,
    )

    private fun nearFieldOnly(
        angleDegrees: Double = 0.0,
        offset: Double = 0.0,
    ) = observation(
        angleDegrees = angleDegrees,
        offset = offset,
        longSideFraction = 0.4,
        lookahead = null,
        quality = PathQuality.NEAR_FIELD_ONLY,
        endpointCandidate = false,
        closedLoop = false,
    )

    private fun observation(
        angleDegrees: Double,
        offset: Double,
        longSideFraction: Double,
        lookahead: TapeLookahead?,
        quality: PathQuality,
        endpointCandidate: Boolean,
        closedLoop: Boolean,
    ) = TapeTrackingObservation(
        angleFromVerticalDegrees = angleDegrees,
        longSideFraction = longSideFraction,
        nearFieldOffsetFraction = offset,
        bounds = NormalizedRect(
            left = (0.5 + offset - 0.05).coerceIn(0.0, 0.9),
            top = 0.02,
            right = (0.5 + offset + 0.05).coerceIn(0.1, 1.0),
            bottom = 1.0,
        ),
        lookahead = lookahead,
        quality = quality,
        endpointCandidate = endpointCandidate,
        closedLoop = closedLoop,
        frameWidthPixels = 1600,
        frameHeightPixels = 900,
        heightAboveGroundMeters = 0.5,
    )

    private fun seconds(value: Long): Long = value * 1_000_000_000L
}
