package com.durendal.droneagent.lite

import org.junit.Assert.assertNotNull
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test


class TapeLuminancePolicyTest {
    @Test
    fun `bright background cannot raise black threshold into cardboard range`() {
        assertEquals(105.0, TapeLuminancePolicy.effectiveThreshold(168.0), 0.0)
    }

    @Test
    fun `dim scene keeps its lower adaptive threshold`() {
        assertEquals(62.0, TapeLuminancePolicy.effectiveThreshold(62.0), 0.0)
    }
}
class TapeCandidatePolicyTest {
    @Test
    fun `dark rectangular strip surrounded by brown is accepted`() {
        val score = TapeCandidatePolicy.score(
            TapeCandidateMetrics(
                areaFraction = 0.03,
                aspectRatio = 4.0,
                shortSideFraction = 0.05,
                longSideFraction = 0.60,
                orientedFill = 0.90,
                surroundingBrown = 0.75,
            ),
        )

        assertNotNull(score)
    }

    @Test
    fun `thin cardboard seam shadow is rejected by tape width`() {
        val rejection = TapeCandidatePolicy.rejectionReason(
            TapeCandidateMetrics(
                areaFraction = 0.0065,
                aspectRatio = 18.5,
                shortSideFraction = 0.025,
                longSideFraction = 0.464,
                orientedFill = 0.90,
                surroundingBrown = 0.70,
            ),
        )

        assertEquals(TapeCandidateRejection.WIDTH, rejection)
    }

    @Test
    fun `dark strip without brown cardboard context is rejected`() {
        val score = TapeCandidatePolicy.score(
            TapeCandidateMetrics(
                areaFraction = 0.03,
                aspectRatio = 4.0,
                shortSideFraction = 0.05,
                longSideFraction = 0.60,
                orientedFill = 0.90,
                surroundingBrown = 0.10,
            ),
        )

        assertNull(score)
    }

    @Test
    fun `large dark scene region is not mistaken for tape`() {
        val score = TapeCandidatePolicy.score(
            TapeCandidateMetrics(
                areaFraction = 0.60,
                aspectRatio = 2.0,
                shortSideFraction = 0.20,
                longSideFraction = 0.60,
                orientedFill = 0.95,
                surroundingBrown = 0.80,
            ),
        )

        assertNull(score)
    }

    @Test
    fun `motion softened tape remains accepted`() {
        val score = TapeCandidatePolicy.score(
            TapeCandidateMetrics(
                areaFraction = 0.04,
                aspectRatio = 7.0,
                shortSideFraction = 0.06,
                longSideFraction = 0.70,
                orientedFill = 0.38,
                surroundingBrown = 0.32,
            ),
        )

        assertNotNull(score)
    }

    @Test
    fun `nearby tape remains accepted when its apparent width increases`() {
        val score = TapeCandidatePolicy.score(
            TapeCandidateMetrics(
                areaFraction = 0.12,
                aspectRatio = 5.5,
                shortSideFraction = 0.18,
                longSideFraction = 1.0,
                orientedFill = 0.80,
                surroundingBrown = 0.60,
            ),
        )

        assertNotNull(score)
    }

    @Test
    fun `broad dark edge is rejected even when surrounded by brown`() {
        val score = TapeCandidatePolicy.score(
            TapeCandidateMetrics(
                areaFraction = 0.12,
                aspectRatio = 3.0,
                shortSideFraction = 0.22,
                longSideFraction = 0.70,
                orientedFill = 0.90,
                surroundingBrown = 0.70,
            ),
        )

        assertNull(score)
    }

    @Test
    fun `short door detail is rejected even when narrow and dark`() {
        val score = TapeCandidatePolicy.score(
            TapeCandidateMetrics(
                areaFraction = 0.0025,
                aspectRatio = 9.0,
                shortSideFraction = 0.018,
                longSideFraction = 0.17,
                orientedFill = 0.90,
                surroundingBrown = 0.70,
            ),
        )

        assertNull(score)
    }

    @Test
    fun `angled tape remains valid using its oriented rectangle`() {
        val score = TapeCandidatePolicy.score(
            TapeCandidateMetrics(
                areaFraction = 0.04,
                aspectRatio = 8.0,
                shortSideFraction = 0.05,
                longSideFraction = 0.75,
                orientedFill = 0.82,
                surroundingBrown = 0.70,
            ),
        )

        assertNotNull(score)
    }

    @Test
    fun `untracked candidate touching horizontal frame edge is rejected`() {
        val score = TapeCandidatePolicy.score(
            TapeCandidateMetrics(
                areaFraction = 0.03,
                aspectRatio = 8.0,
                shortSideFraction = 0.05,
                longSideFraction = 0.70,
                orientedFill = 0.90,
                surroundingBrown = 0.70,
                touchesHorizontalFrameEdge = true,
            ),
        )

        assertNull(score)
    }

    @Test
    fun `tracked tape remains detectable as it reaches horizontal frame edge`() {
        val score = TapeCandidatePolicy.score(
            TapeCandidateMetrics(
                areaFraction = 0.03,
                aspectRatio = 8.0,
                shortSideFraction = 0.05,
                longSideFraction = 0.70,
                orientedFill = 0.90,
                surroundingBrown = 0.70,
                touchesHorizontalFrameEdge = true,
                overlapsPreviousDetection = true,
            ),
        )

        assertNotNull(score)
    }

    @Test
    fun `untracked short dark fragment is rejected as scenery`() {
        val rejection = TapeCandidatePolicy.rejectionReason(
            TapeCandidateMetrics(
                areaFraction = 0.025,
                aspectRatio = 1.8,
                shortSideFraction = 0.11,
                longSideFraction = 0.20,
                orientedFill = 0.80,
                surroundingBrown = 0.70,
            ),
        )

        assertEquals(TapeCandidateRejection.ASPECT, rejection)
    }

    @Test
    fun `overlapping terminal tape keeps continuity as its visible shape shortens`() {
        val score = TapeCandidatePolicy.score(
            TapeCandidateMetrics(
                areaFraction = 0.025,
                aspectRatio = 1.8,
                shortSideFraction = 0.11,
                longSideFraction = 0.20,
                orientedFill = 0.80,
                surroundingBrown = 0.70,
                overlapsPreviousDetection = true,
            ),
        )

        assertNotNull(score)
    }
}
