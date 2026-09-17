package com.durendal.droneagent.lite

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class RacingCircleGuidanceTest {
    @Test
    fun `opposite established directions give mirrored local tangents and never choose a direction`() {
        for (side in listOf(-1.0, 1.0)) {
            val guidance = started()
            for (millis in listOf(20L, 80L, 140L)) {
                guidance.observe(frame(millis, centerForward = 0.1, side = side), 0.0, 0.0, time(millis))
            }
            val result = predict(guidance, 140, side)
            assertEquals(Math.toDegrees(atan2(-side * 0.1, 0.75)), checkNotNull(result.tangentDegrees), 0.01)
            assertFalse(result.diagnostics.guidanceActive)
            assertNull(predict(guidance, 140, -side).tangentDegrees)
            assertNull(predict(guidance, 140, 0.0).tangentDegrees)
            assertEquals(result.tangentDegrees, predict(guidance, 140, side).tangentDegrees)
        }
    }

    @Test
    fun `delayed frames preserve source provenance and exact frame heading rotates VO`() {
        val guidance = started()
        for (millis in listOf(20L, 80L, 140L)) {
            guidance.observe(frame(millis, centerForward = 0.1 - 0.4 * millis / 1000.0, heading = 90.0), 0.4, 0.0, time(millis + 40))
        }
        val result = guidance.predict(time(180), 90.0, time(180), 1.0)
        assertNotNull(result.diagnostics.toString(), result.tangentDegrees)
        assertEquals(time(140), result.diagnostics.lastVisionAtNanos)
        assertEquals(time(140), result.diagnostics.lastVelocityAtNanos)
        assertEquals(time(180), result.diagnostics.receivedAtNanos)
        assertEquals(time(230), result.diagnostics.estimateAtNanos)
        assertEquals(0.1 - 0.4 * 0.230, checkNotNull(result.diagnostics.centerForwardMeters), 0.002)
        assertEquals(0.75, checkNotNull(result.diagnostics.centerRightMeters), 0.002)
        assertEquals(3, result.diagnostics.acceptedVisionCount)
    }

    @Test
    fun `recovery receipts at 76 228 and 306 ms confirm on third source frame without a 250 ms stop`() {
        val guidance = acquired()
        guidance.invalidate("missing_scale", time(200))
        assertNull(predict(guidance, 200).tangentDegrees)
        val sources = listOf(240L, 390L, 470L)
        val receipts = listOf(276L, 428L, 506L)
        for (index in sources.indices) {
            guidance.observe(frame(sources[index]), 0.0, 0.0, time(receipts[index]))
            val result = predict(guidance, receipts[index])
            if (index < 2) assertNull(result.tangentDegrees)
            else {
                assertNotNull(result.diagnostics.toString(), result.tangentDegrees)
                assertEquals(6, result.diagnostics.acceptedVisionCount)
                assertEquals(time(200), result.diagnostics.lastRejectionSourceAtNanos)
            }
        }
    }

    @Test
    fun `missing scale or same frame heading withdraws contribution until three new frames`() {
        for (missingScale in listOf(true, false)) {
            val guidance = acquired()
            val image = frame(180)
            val invalid = image.copy(circleImageScale = if (missingScale) null else image.circleImageScale!!.copy(headingAtNanos = 0L))
            guidance.observe(invalid, 0.0, 0.0, time(180))
            val rejected = predict(guidance, 180)
            assertNull(rejected.tangentDegrees)
            assertEquals("FIT_REJECTED", rejected.diagnostics.status)
            assertNotNull(rejected.diagnostics.fitReason)
            for (millis in listOf(220L, 280L)) {
                guidance.observe(frame(millis), 0.0, 0.0, time(millis))
                assertNull(predict(guidance, millis).tangentDegrees)
            }
            guidance.observe(frame(340), 0.0, 0.0, time(340))
            assertNotNull(predict(guidance, 340).tangentDegrees)
        }
    }

    @Test
    fun `current heading must be real fresh and nonfuture rather than synthesized by prediction`() {
        val guidance = acquired()
        assertNull(guidance.predict(time(140), null, time(140), 1.0).tangentDegrees)
        assertNull(guidance.predict(time(140), 0.0, 0L, 1.0).tangentDegrees)
        assertNull(guidance.predict(time(140), 0.0, time(141), 1.0).tangentDegrees)
        assertNull(guidance.predict(time(140), 0.0, time(-111), 1.0).tangentDegrees)
        val recoveredHeading = predict(guidance, 140)
        assertNotNull(recoveredHeading.tangentDegrees)
        assertEquals(time(140), recoveredHeading.diagnostics.lastVisionAtNanos)
    }

    @Test
    fun `attempt reset rejects previous attempt frames and clears accumulated confidence`() {
        val guidance = acquired()
        guidance.invalidate("old_pending_failure", time(180))
        guidance.reset(time(200))
        guidance.observe(frame(140), 0.0, 0.0, time(210))
        val reset = predict(guidance, 210)
        assertNull(reset.tangentDegrees)
        assertEquals(0, reset.diagnostics.acceptedVisionCount)
        assertEquals(0L, reset.diagnostics.lastVisionAtNanos)
        for (millis in listOf(220L, 280L, 340L)) {
            guidance.observe(frame(millis, side = -1.0), 0.0, 0.0, time(millis))
        }
        val recovered = predict(guidance, 340, -1.0)
        assertNotNull(recovered.tangentDegrees)
        assertEquals(3, recovered.diagnostics.acceptedVisionCount)
        assertEquals(0, recovered.diagnostics.estimatorResetCount)
    }

    @Test
    fun `duplicates reordered and future source events cannot count or poison subsequent acquisition`() {
        val guidance = started()
        guidance.observe(frame(20), 0.0, 0.0, time(20))
        guidance.observe(frame(20), 0.0, 0.0, time(40))
        guidance.observe(frame(10), 0.0, 0.0, time(40))
        guidance.observe(frame(5000), 0.0, 0.0, time(40))
        assertEquals(1, predict(guidance, 40).diagnostics.acceptedVisionCount)
        guidance.observe(frame(80), 0.0, 0.0, time(80))
        assertNull(predict(guidance, 80).tangentDegrees)
        guidance.observe(frame(140), 0.0, 0.0, time(140))
        assertNotNull(predict(guidance, 140).tangentDegrees)
        assertEquals(3, predict(guidance, 140).diagnostics.acceptedVisionCount)
    }

    @Test
    fun `confirmation requires source span rather than callback span or repeated predictions`() {
        val guidance = started()
        for (millis in listOf(20L, 30L, 40L)) {
            guidance.observe(frame(millis), 0.0, 0.0, time(millis + 100))
        }
        repeat(10) { assertNull(predict(guidance, 180).tangentDegrees) }
        guidance.observe(frame(150), 0.0, 0.0, time(190))
        assertNotNull(predict(guidance, 190).tangentDegrees)
    }

    @Test
    fun `stale posterior after a long gap permits a completely new circle without a moving receipt barrier`() {
        val guidance = acquired()
        assertEquals("STALE", predict(guidance, 500).diagnostics.status)
        for (millis in listOf(3000L, 3070L, 3140L)) {
            guidance.observe(frame(millis, side = -1.0), 0.0, 0.0, time(millis + 40))
            val result = predict(guidance, millis + 40, -1.0)
            if (millis < 3140) assertNull(result.tangentDegrees)
            else {
                assertNotNull(result.diagnostics.toString(), result.tangentDegrees)
                assertEquals(1, result.diagnostics.estimatorResetCount)
                assertEquals(6, result.diagnostics.acceptedVisionCount)
                assertEquals(time(3140), result.diagnostics.lastVisionAtNanos)
            }
        }
    }

    @Test
    fun `stale image cannot acquire or refresh freshness and repeated invalidation is idempotent`() {
        val guidance = acquired()
        guidance.observe(frame(180), 0.0, 0.0, time(440))
        val stale = predict(guidance, 440)
        assertNull(stale.tangentDegrees)
        assertEquals("STALE", stale.diagnostics.status)
        assertEquals(time(140), stale.diagnostics.lastVisionAtNanos)
        guidance.invalidate("bad_frame", time(450))
        val first = predict(guidance, 450)
        repeat(10) {
            guidance.invalidate("bad_frame", time(450))
            assertEquals(first.diagnostics.rejectionCount, predict(guidance, 450).diagnostics.rejectionCount)
        }
        assertEquals(time(450), first.diagnostics.lastRejectionSourceAtNanos)
    }

    @Test
    fun `late invalidations cannot revoke newer exact frame evidence or future poison its barrier`() {
        val guidance = acquired()
        guidance.invalidate("late_failure", time(80))
        assertNotNull(predict(guidance, 160).tangentDegrees)
        guidance.invalidate("future_failure", time(10000))
        assertNotNull(predict(guidance, 160).tangentDegrees)
        guidance.invalidate("missing_source", 0L)
        assertNotNull(predict(guidance, 160).tangentDegrees)
        assertEquals(3, predict(guidance, 160).diagnostics.acceptedVisionCount)
    }

    @Test
    fun `frame mismatch and EKF innovation reject without masquerading as detector loss`() {
        val guidance = acquired()
        val mismatch = frame(180)
        guidance.observe(mismatch.copy(circleImageScale = mismatch.circleImageScale!!.copy(sampleAtNanos = time(179))), 0.0, 0.0, time(180))
        val rejectedFit = predict(guidance, 180)
        assertEquals("FIT_REJECTED", rejectedFit.diagnostics.status)
        assertEquals("mismatched_image_scale", rejectedFit.diagnostics.fitReason)
        guidance.observe(frame(220, side = -1.0), 0.0, 0.0, time(220))
        val rejectedEkf = predict(guidance, 220)
        assertEquals("EKF_REJECTED", rejectedEkf.diagnostics.status)
        assertNull(rejectedEkf.tangentDegrees)
        assertEquals(1, rejectedEkf.diagnostics.rejectedVisionCount)
        assertTrue(checkNotNull(rejectedEkf.diagnostics.lastInnovationSquared) > 9.21)
        assertEquals(time(140), rejectedEkf.diagnostics.lastVisionAtNanos)
    }

    private fun started() = RacingCircleGuidance().also { it.reset(time(0)) }

    private fun acquired() = started().also { guidance ->
        for (millis in listOf(20L, 80L, 140L)) {
            guidance.observe(frame(millis), 0.0, 0.0, time(millis))
        }
        assertNotNull(predict(guidance, 140).tangentDegrees)
    }

    private fun predict(guidance: RacingCircleGuidance, millis: Long, side: Double = 1.0) =
        guidance.predict(time(millis), 0.0, time(millis), side)

    private fun frame(millis: Long, centerForward: Double = 0.0, side: Double = 1.0, heading: Double = 0.0): TapeTrackingObservation {
        val count = 41
        val xs = FloatArray(count)
        val ys = FloatArray(count)
        for (index in 0 until count) {
            val angle = -0.7 + 1.4 * index / (count - 1)
            xs[index] = (0.5 + side * (0.75 - 0.75 * cos(angle)) / 2.56).toFloat()
            ys[index] = (0.5 - (centerForward + 0.75 * sin(angle)) / 1.44).toFloat()
        }
        val path = TapeCenterlinePath(
            sourceWidth = 1280,
            sourceHeight = 720,
            xFractions = xs,
            yFractions = ys,
            anchorXFraction = xs.first(),
            anchorYFraction = ys.first(),
            lookaheadXFraction = xs.last(),
            lookaheadYFraction = ys.last(),
            quality = PathQuality.FULL_PATH,
            rejection = null,
        )
        return TapeTrackingObservation(
            angleFromVerticalDegrees = 0.0,
            longSideFraction = 0.7,
            nearFieldOffsetFraction = 0.0,
            bounds = NormalizedRect(0.2, 0.1, 0.8, 0.9),
            lookahead = TapeLookahead(xs.last().toDouble(), ys.last().toDouble()),
            quality = PathQuality.FULL_PATH,
            endpointCandidate = false,
            closedLoop = false,
            frameWidthPixels = 1280,
            frameHeightPixels = 720,
            capturedAtNanos = time(millis),
            heightAboveGroundMeters = null,
            confidence = 0.95,
            centerline = path,
            circleImageScale = CircleImageScale(0.004, 640, 360, time(millis), heading, time(millis)),
        )
    }

    private fun time(millis: Long): Long = 1_000_000_000L + millis * 1_000_000L
}
