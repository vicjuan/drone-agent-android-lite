package com.durendal.droneagent.lite

import kotlin.math.cos
import kotlin.math.sin
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Test

class CircleVisualFrameJoinerTest {
    @Test
    fun delayedVelocityFitsItsOwnGeometryNotTheNewestPathAndIsConsumedOnce() {
        val joiner = CircleVisualFrameJoiner()
        val first = 1_100_000_000L
        val second = first + 100_000_000L
        joiner.offer(rawCircle(first))
        assertNull(joiner.take(sample(second)))
        joiner.offer(rawCircle(second, radialError = 0.10))
        val joined = checkNotNull(joiner.take(sample(first)))
        assertEquals(0.75, fittedRightCenter(joined, second), 0.002)
        assertNull(joiner.take(sample(first)))
        joiner.offer(rawCircle(first))
        assertNull(joiner.take(sample(first)))
        assertEquals(0.85, fittedRightCenter(checkNotNull(joiner.take(sample(second))), second), 0.002)
    }

    @Test
    fun invalidVelocityCannotSupplyMetricAuthorityOrResurrectAnInvalidatedPair() {
        val joiner = CircleVisualFrameJoiner()
        val first = 1_100_000_000L
        joiner.offer(rawCircle(first))
        assertNull(joiner.take(sample(first).copy(reason = "NO_SCALE", metersPerPixel = null)))
        joiner.invalidate(first)
        assertNull(joiner.take(sample(first)))
        joiner.offer(rawCircle(first))
        assertNull(joiner.take(sample(first)))
        val next = first + 100_000_000L
        joiner.offer(rawCircle(next))
        assertNotNull(joiner.take(sample(next)))
    }

    @Test
    fun invalidDetectionDropsPendingPairsWithoutWaitingForVelocity() {
        val first = 1_100_000_000L
        val next = first + 100_000_000L
        val invalid = listOf(
            rawCircle(next).copy(confidence = 0.2),
            rawCircle(next).copy(centerline = null),
            rawCircle(next, ambiguousBranch = true),
            null,
        )
        for (observation in invalid) {
            val joiner = CircleVisualFrameJoiner()
            joiner.offer(rawCircle(first))
            assertFalse(joiner.offer(observation, next))
            assertNull(joiner.take(sample(first)))
            assertNull(joiner.take(sample(next)))
            joiner.offer(rawCircle(first))
            assertNull(joiner.take(sample(first)))
        }
    }

    @Test
    fun delayedInvalidationKeepsNewerPendingGeometry() {
        val joiner = CircleVisualFrameJoiner()
        val first = 1_100_000_000L
        val next = first + 100_000_000L
        joiner.offer(rawCircle(first))
        joiner.offer(rawCircle(next, radialError = 0.10))
        joiner.invalidate(first)
        joiner.invalidate(first)
        assertNull(joiner.take(sample(first)))
        assertEquals(0.85, fittedRightCenter(checkNotNull(joiner.take(sample(next))), next), 0.002)
    }

    @Test
    fun startingANewAttemptDiscardsOldPairsAndLateOldFailures() {
        val joiner = CircleVisualFrameJoiner()
        val old = 1_100_000_000L
        val started = old + 200_000_000L
        val next = started + 100_000_000L
        joiner.offer(rawCircle(old))
        joiner.clear(started)
        joiner.offer(rawCircle(old))
        joiner.offer(rawCircle(next))
        joiner.offer(null, old)
        joiner.invalidate(old)
        assertNull(joiner.take(sample(old)))
        assertNotNull(joiner.take(sample(next)))
    }

    @Test
    fun capacityEvictsOldestGeometryWithoutSubstitutingAnotherFrame() {
        val joiner = CircleVisualFrameJoiner()
        val first = 1_100_000_000L
        val interval = 10_000_000L
        repeat(17) { joiner.offer(rawCircle(first + it * interval)) }
        assertNull(joiner.take(sample(first)))
        assertNotNull(joiner.take(sample(first + interval)))
        assertNotNull(joiner.take(sample(first + 16 * interval)))
        assertNull(joiner.take(sample(first + 15 * interval)))
    }

    private fun fittedRightCenter(observation: TapeTrackingObservation, now: Long): Double {
        val result = FixedRadiusCircleFitter().fit(
            checkNotNull(observation.centerline), checkNotNull(observation.circleImageScale),
            observation.capturedAtNanos, now,
        )
        assertNotNull(result.reason, result.observation)
        return checkNotNull(result.observation).centerRightMeters
    }

    private fun sample(at: Long) = VisualVelocityDiagnostics.ControlSample(
        frameNanos = at,
        forwardMps = 0.5,
        rightMps = 0.0,
        aircraftHeadingDegrees = 0.0,
        reason = null,
        aircraftHeadingAtNanos = at,
        metersPerPixel = 0.0025,
        analysisWidth = 640,
        analysisHeight = 360,
    )

    /** Fixed-radius image geometry, deliberately without metric scale until the exact VO pair. */
    private fun rawCircle(at: Long, radialError: Double = 0.0, ambiguousBranch: Boolean = false): TapeTrackingObservation {
        val xs = FloatArray(25)
        val ys = FloatArray(25)
        for (index in xs.indices) {
            val theta = -0.5 + index.toDouble() / (xs.size - 1)
            xs[index] = (0.5 + (radialError + 0.75 * (1.0 - cos(theta))) / (640 * 0.0025)).toFloat()
            ys[index] = (0.5 - 0.75 * sin(theta) / (360 * 0.0025)).toFloat()
        }
        val quality = if (ambiguousBranch) PathQuality.NEAR_FIELD_ONLY else PathQuality.FULL_PATH
        val path = TapeCenterlinePath(
            640, 360, xs, ys, xs.first(), ys.first(),
            if (ambiguousBranch) null else xs.last(), if (ambiguousBranch) null else ys.last(),
            quality, if (ambiguousBranch) "AMBIGUOUS_BRANCH" else null,
            branchCount = if (ambiguousBranch) 1 else 0,
        )
        return TapeTrackingObservation(
            angleFromVerticalDegrees = 0.0,
            longSideFraction = 0.8,
            nearFieldOffsetFraction = 0.0,
            bounds = NormalizedRect(0.2, 0.1, 0.8, 0.9),
            lookahead = if (ambiguousBranch) null else TapeLookahead(xs.last().toDouble(), ys.last().toDouble()),
            quality = quality,
            endpointCandidate = false,
            closedLoop = false,
            frameWidthPixels = 640,
            frameHeightPixels = 360,
            capturedAtNanos = at,
            heightAboveGroundMeters = null,
            confidence = 0.95,
            centerline = path,
        )
    }
}
