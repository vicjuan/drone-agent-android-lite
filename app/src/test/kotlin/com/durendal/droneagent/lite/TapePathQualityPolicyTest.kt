package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Quality is the only thing standing between an ambiguous shape and forward
 * motion, so every downgrade needs a test that fails if the downgrade is lost.
 */
class TapePathQualityPolicyTest {

    @Test
    fun `a long clean chain with a look-ahead is a full path`() {
        val verdict = evaluate(points = 120)

        assertEquals(PathQuality.FULL_PATH, verdict.quality)
        assertTrue(verdict.lookahead != null)
        assertNull(verdict.rejection)
    }

    @Test
    fun `a junction can be aligned to but never pursued`() {
        val verdict = evaluate(points = 120, branchCount = 1)

        assertEquals(PathQuality.NEAR_FIELD_ONLY, verdict.quality)
        assertNull("a branch must not hand over a target", verdict.lookahead)
        assertEquals(TapeCandidateRejection.AMBIGUOUS_BRANCH, verdict.rejection)
    }

    @Test
    fun `a chain with no usable look-ahead is near field only`() {
        // Long enough for the near field to be credible (>= 0.10 of the short
        // side) but too short for the look-ahead to travel a usable distance.
        val verdict = evaluate(points = 20, stepPixels = 3.0)

        assertEquals(PathQuality.NEAR_FIELD_ONLY, verdict.quality)
        assertNull(verdict.lookahead)
        assertEquals(TapeCandidateRejection.INSUFFICIENT_LOOKAHEAD, verdict.rejection)
    }

    @Test
    fun `a chain too short to give a direction is lost`() {
        // Where the chain sits in the frame is a fact about the camera angle, so
        // it is not a credibility test. Length is: below this there is no
        // direction to align to, only a speck.
        val verdict = evaluate(points = 8, stepPixels = 1.0)

        assertEquals(PathQuality.LOST, verdict.quality)
        assertEquals(TapeCandidateRejection.NO_NEAR_FIELD_COMPONENT, verdict.rejection)
    }

    @Test
    fun `an inconsistent width chain is lost rather than followed`() {
        val verdict = evaluate(points = 120, widthConsistency = 0.1)

        assertEquals(PathQuality.LOST, verdict.quality)
        assertEquals(TapeCandidateRejection.NO_NEAR_FIELD_COMPONENT, verdict.rejection)
    }

    @Test
    fun `an empty chain is lost with no centerline`() {
        val verdict = TapePathQualityPolicy.evaluate(
            estimate = estimate(emptyList()),
            measurement = null,
            mode = TapeDetectionMode.PATH,
            frameHeight = FRAME_HEIGHT,
        )

        assertEquals(PathQuality.LOST, verdict.quality)
        assertEquals(TapeCandidateRejection.NO_CENTERLINE, verdict.rejection)
    }

    @Test
    fun `a terminus inside the frame is an endpoint candidate but a border one is not`() {
        assertTrue(
            TapePathQualityPolicy.isEndpointCandidate(
                CenterlineTopology(distalTerminus = CenterlineTerminus.INSIDE_FRAME),
                TapeDetectionMode.STRAIGHT,
            ),
        )
        assertFalse(
            "a chain cut off by the border means the tape left the view",
            TapePathQualityPolicy.isEndpointCandidate(
                CenterlineTopology(distalTerminus = CenterlineTerminus.AT_FRAME_BORDER),
                TapeDetectionMode.STRAIGHT,
            ),
        )
    }

    @Test
    fun `a closed loop is never an endpoint candidate`() {
        assertFalse(
            TapePathQualityPolicy.isEndpointCandidate(
                CenterlineTopology(
                    distalTerminus = CenterlineTerminus.NONE,
                    closedLoop = true,
                ),
                TapeDetectionMode.PATH,
            ),
        )
        assertTrue(
            TapePathQualityPolicy.isClosedLoopPath(
                CenterlineTopology(closedLoop = true),
            ),
        )
    }

    @Test
    fun `a terminus of NONE without a loop is not read as an endpoint`() {
        assertFalse(
            TapePathQualityPolicy.isEndpointCandidate(
                CenterlineTopology(distalTerminus = CenterlineTerminus.NONE),
                TapeDetectionMode.PATH,
            ),
        )
    }

    private fun evaluate(
        points: Int,
        stepPixels: Double = 3.0,
        branchCount: Int = 0,
        widthConsistency: Double = 1.0,
        anchorBottomOffset: Double = 1.0,
    ): TapePathVerdict {
        val chain = (0 until points).map { index ->
            CenterlinePoint(
                x = FRAME_WIDTH / 2.0,
                y = FRAME_HEIGHT - anchorBottomOffset - index * stepPixels,
                widthPixels = 24.0,
            )
        }
        val estimate = estimate(chain, branchCount, widthConsistency)
        return TapePathQualityPolicy.evaluate(
            estimate = estimate,
            measurement = CenterlineMeasurement.measure(estimate, FRAME_WIDTH, FRAME_HEIGHT),
            mode = TapeDetectionMode.STRAIGHT,
            frameHeight = FRAME_HEIGHT,
        )
    }

    private fun estimate(
        points: List<CenterlinePoint>,
        branchCount: Int = 0,
        widthConsistency: Double = 1.0,
    ) = CenterlineEstimate(
        points = points,
        confidence = 1.0,
        components = CenterlineConfidence(1.0, widthConsistency, 1.0, 1.0, 1.0),
        topology = CenterlineTopology(
            distalTerminus = CenterlineTerminus.INSIDE_FRAME,
            branchCount = branchCount,
        ),
    )

    private companion object {
        const val FRAME_WIDTH = 640
        const val FRAME_HEIGHT = 360
    }
}
