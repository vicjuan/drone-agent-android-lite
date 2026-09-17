package com.durendal.droneagent.lite

import kotlin.math.sqrt
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class CircleStateEstimatorTest {
    @Test
    fun `metric velocity predicts relative centre with integrated acceleration covariance`() {
        val estimator = initialized(config = permissiveConfig())
        val prediction = checkNotNull(estimator.estimate(time(100), 0.0, time(100), 0.0).estimate)

        assertEquals(0.14, prediction.centerForwardMeters, 1e-12)
        assertEquals(0.74, prediction.centerRightMeters, 1e-12)
        assertEquals(0.6, prediction.forwardVelocityMetersPerSecond, 1e-12)
        assertEquals(0.1, prediction.rightVelocityMetersPerSecond, 1e-12)
        assertEquals(sqrt(0.0004 + 0.01 * 0.1 * 0.1 + 0.64 * 0.1 * 0.1 * 0.1 / 3.0), prediction.positionStdMeters, 1e-12)
        assertEquals(sqrt(0.01 + 0.64 * 0.1), prediction.velocityStdMetersPerSecond, 1e-12)
        assertEquals(time(100), prediction.estimateAtNanos)
        assertEquals(time(0), prediction.lastVisionAtNanos)
        assertEquals(time(0), prediction.lastVelocityAtNanos)

        val horizon = checkNotNull(estimator.estimate(time(0), 0.0, time(0)).estimate)
        assertEquals(0.17, horizon.centerForwardMeters, 1e-12)
        assertEquals(0.745, horizon.centerRightMeters, 1e-12)
        assertEquals(time(50), horizon.estimateAtNanos)
        assertEquals(0.05, horizon.predictionSeconds, 0.0)
    }

    @Test
    fun `vision conditions velocity through cross covariance rather than resetting speed`() {
        val estimator = initialized()
        val before = checkNotNull(estimator.estimate(time(100), 0.0, time(100), 0.0).estimate)

        assertTrue(estimator.observe(observation(100, north = 0.142, east = 0.74), time(100)))
        val after = estimator.estimate(time(100), 0.0, time(100), 0.0)
        val state = checkNotNull(after.estimate)
        assertTrue(state.centerForwardMeters > before.centerForwardMeters)
        assertTrue(state.centerForwardMeters < 0.142)
        assertTrue(state.forwardVelocityMetersPerSecond in 0.5..0.6)
        assertTrue(state.forwardVelocityMetersPerSecond < before.forwardVelocityMetersPerSecond)
        assertTrue(state.positionStdMeters < before.positionStdMeters)
        assertEquals(1, after.acceptedVelocityCount)
        assertEquals(2, after.acceptedVisionCount)
    }

    @Test
    fun `late vision replays every intervening velocity and vision in chronological order`() {
        val chronological = initialized()
        velocity(chronological, 60, 0.62, 0.09)
        assertTrue(chronological.observe(observation(90, north = 0.147, east = 0.742), time(90)))
        velocity(chronological, 120, 0.58, 0.12)
        assertTrue(chronological.observe(observation(150, north = 0.109, east = 0.733), time(150)))
        velocity(chronological, 180, 0.61, 0.1)
        assertTrue(chronological.observe(observation(210, north = 0.075, east = 0.729), time(210)))
        velocity(chronological, 240, 0.6, 0.11)

        val delayed = initialized()
        velocity(delayed, 60, 0.62, 0.09)
        velocity(delayed, 120, 0.58, 0.12)
        velocity(delayed, 180, 0.61, 0.1)
        assertTrue(delayed.observe(observation(150, north = 0.109, east = 0.733), time(180)))
        velocity(delayed, 240, 0.6, 0.11)
        assertTrue(delayed.observe(observation(210, north = 0.075, east = 0.729), time(240)))
        assertTrue(delayed.observe(observation(90, north = 0.147, east = 0.742), time(240)))

        val expected = chronological.estimate(time(240), 13.0, time(240))
        val actual = delayed.estimate(time(240), 13.0, time(240))
        assertEquivalent(expected, actual)
        assertEquals(3, actual.lateReplayCount)
        assertEquals(4, actual.acceptedVisionCount)
        assertEquals(5, actual.acceptedVelocityCount)
    }

    @Test
    fun `checkpoint trimming preserves posterior for a late retained image`() {
        val config = permissiveConfig().copy(
            maximumVisionAgeNanos = 300_000_000L,
            maximumVelocityAgeNanos = 300_000_000L,
            historyDurationNanos = 350_000_000L,
            historyCapacity = 32,
        )
        val chronological = CircleStateEstimator(config).also { it.reset(time(0)) }
        val delayed = CircleStateEstimator(config).also { it.reset(time(0)) }
        for (index in 0..20) {
            val velocityMillis = index * 50L
            val visionMillis = velocityMillis + 20L
            velocity(chronological, velocityMillis, 0.3, 0.04)
            velocity(delayed, velocityMillis, 0.3, 0.04)
            val frame = observation(
                visionMillis,
                north = 0.5 - 0.3 * visionMillis / 1000.0 + if (index % 2 == 0) 0.001 else -0.001,
                east = 0.75 - 0.04 * visionMillis / 1000.0,
            )
            assertTrue(chronological.observe(frame, time(visionMillis)))
            if (index != 16) assertTrue(delayed.observe(frame, time(visionMillis)))
        }
        val lateFrame = observation(820, north = 0.5 - 0.3 * 0.82 + 0.001, east = 0.75 - 0.04 * 0.82)
        assertTrue(delayed.observe(lateFrame, time(1020)))
        val expected = chronological.estimate(time(1020), -30.0, time(1020))
        val actual = delayed.estimate(time(1020), -30.0, time(1020))
        assertEquivalent(expected, actual)
        assertEquals(21, actual.acceptedVisionCount)
        assertEquals(21, actual.acceptedVelocityCount)
        assertEquals(1, actual.lateReplayCount)

        assertFalse(delayed.observe(lateFrame, time(1020)))
        assertFalse(delayed.observe(observation(650), time(1020)))
        assertEquals(actual, delayed.estimate(time(1020), -30.0, time(1020)))
    }

    @Test
    fun `replay can accept a previously gated frame without double counting it`() {
        val chronological = CircleStateEstimator(permissiveConfig()).also { it.reset(time(0)) }
        val delayed = CircleStateEstimator(permissiveConfig()).also { it.reset(time(0)) }
        for (estimator in listOf(chronological, delayed)) {
            velocity(estimator, 0, 0.0, 0.0)
            assertTrue(estimator.observe(observation(0), time(0)))
        }
        val early = observation(50, north = 0.27)
        val later = observation(100, north = 0.31)
        assertTrue(chronological.observe(early, time(50)))
        assertTrue(chronological.observe(later, time(100)))
        assertFalse(delayed.observe(later, time(100)))
        assertEquals(1, delayed.estimate(time(100), 0.0, time(100)).rejectedVisionCount)
        assertFalse(delayed.hasConfirmedVisionSince(time(0) - 1L, 3, 100_000_000L))
        assertTrue(delayed.observe(early, time(100)))
        val replayed = delayed.estimate(time(100), 0.0, time(100))
        assertEquivalent(chronological.estimate(time(100), 0.0, time(100)), replayed)
        assertEquals(3, replayed.acceptedVisionCount)
        assertEquals(0, replayed.rejectedVisionCount)
        assertEquals(time(100), checkNotNull(replayed.estimate).lastVisionAtNanos)
        assertTrue(delayed.hasConfirmedVisionSince(time(0) - 1L, 3, 100_000_000L))
    }

    @Test
    fun `equal timestamp measurements are independent of callback order`() {
        val velocityFirst = initialized()
        val visionFirst = CircleStateEstimator().also { it.reset(time(0)) }
        assertTrue(visionFirst.observe(observation(0), time(0)))
        assertNull(visionFirst.estimate(time(0), 0.0, time(0)).estimate)
        velocity(visionFirst, 0, 0.6, 0.1)
        assertEquivalent(
            velocityFirst.estimate(time(0), 0.0, time(0)),
            visionFirst.estimate(time(0), 0.0, time(0)),
        )
    }

    @Test
    fun `vision before first velocity measurements can be replayed without a fabricated velocity sample`() {
        val chronological = CircleStateEstimator().also { it.reset(time(0)) }
        assertTrue(chronological.observe(observation(0), time(0)))
        assertEquals("velocity_unavailable", chronological.estimate(time(25), 0.0, time(25)).reason)
        velocity(chronological, 50, 0.6, 0.1)

        val delayed = CircleStateEstimator().also { it.reset(time(0)) }
        velocity(delayed, 50, 0.6, 0.1)
        assertEquals("vision_unavailable", delayed.estimate(time(50), 0.0, time(50)).reason)
        assertTrue(delayed.observe(observation(0), time(50)))
        assertEquivalent(
            chronological.estimate(time(50), 0.0, time(50)),
            delayed.estimate(time(50), 0.0, time(50)),
        )
        assertEquals(time(50), checkNotNull(delayed.estimate(time(50), 0.0, time(50)).estimate).lastVelocityAtNanos)
    }

    @Test
    fun `prediction queries neither integrate twice nor renew velocity measurements timestamps`() {
        val estimator = initialized(config = permissiveConfig())
        val first = estimator.estimate(time(100), 0.0, time(100))
        repeat(10) { assertEquals(first, estimator.estimate(time(100), 0.0, time(100))) }
        assertNotNull(estimator.estimate(time(200), 0.0, time(200)).estimate)
        assertEquals(first, estimator.estimate(time(100), 0.0, time(100)))
        assertEquals(time(0), checkNotNull(first.estimate).lastVelocityAtNanos)
        assertEquals(time(0), first.estimate?.lastVisionAtNanos)
        assertEquals("vision_expired", estimator.estimate(time(251), 0.0, time(251)).reason)
    }

    @Test
    fun `image heading rotates observation and current heading rotates output without changing earth velocity`() {
        val estimator = CircleStateEstimator().also { it.reset(time(0)) }
        velocity(estimator, 0, 0.6, -0.2)
        assertTrue(
            estimator.observe(
                observation(0).copy(
                    centerForwardMeters = 0.75,
                    centerRightMeters = -0.2,
                    centerForwardVariance = 0.0001,
                    centerRightVariance = 0.0004,
                    centerCovariance = 0.00005,
                    headingDegrees = 90.0,
                ),
                time(0),
            ),
        )
        val north = checkNotNull(estimator.estimate(time(0), 0.0, time(0), 0.0).estimate)
        val east = checkNotNull(estimator.estimate(time(0), 90.0, time(0), 0.0).estimate)
        val wrapped = checkNotNull(estimator.estimate(time(0), 450.0, time(0), 0.0).estimate)
        assertEquals(0.2, north.centerForwardMeters, 1e-12)
        assertEquals(0.75, north.centerRightMeters, 1e-12)
        assertEquals(0.75, east.centerForwardMeters, 1e-12)
        assertEquals(-0.2, east.centerRightMeters, 1e-12)
        assertEquals(-0.2, east.forwardVelocityMetersPerSecond, 1e-12)
        assertEquals(-0.6, east.rightVelocityMetersPerSecond, 1e-12)
        assertEquals(north.positionStdMeters, east.positionStdMeters, 0.0)
        assertEquals(east, wrapped)
    }

    @Test
    fun `bearing innovation crosses negative north seam rather than appearing as a full turn`() {
        val estimator = CircleStateEstimator().also { it.reset(time(0)) }
        velocity(estimator, 0, 0.0, 0.0)
        assertTrue(estimator.observe(observation(0, north = -0.75, east = 0.002), time(0)))
        assertTrue(estimator.observe(observation(50, north = -0.75, east = -0.002), time(50)))
        val result = estimator.estimate(time(50), 0.0, time(50), 0.0)
        assertNotNull(result.estimate)
        assertTrue(checkNotNull(result.lastInnovationSquared) < 1.0)
        assertEquals(2, result.acceptedVisionCount)
    }

    @Test
    fun `outliers do not rejuvenate image or velocity provenance and repeated images are unique`() {
        val estimator = initialized(config = permissiveConfig())
        val outlier = observation(50, north = 3.0, east = 3.0)
        assertFalse(estimator.observe(outlier, time(50)))
        assertFalse(estimator.observe(outlier, time(50)))
        assertFalse(estimator.updateVelocity(100.0, -100.0, time(60), time(60)))
        assertFalse(estimator.updateVelocity(0.6, 0.1, time(60), time(60)))
        val gated = estimator.estimate(time(60), 0.0, time(60), 0.0)
        assertEquals(1, gated.acceptedVisionCount)
        assertEquals(1, gated.rejectedVisionCount)
        assertEquals(1, gated.acceptedVelocityCount)
        assertTrue(checkNotNull(gated.lastInnovationSquared) > 9.21)
        assertEquals(time(0), checkNotNull(gated.estimate).lastVisionAtNanos)
        assertEquals(time(0), gated.estimate?.lastVelocityAtNanos)

        velocity(estimator, 70, 0.6, 0.1)
        assertTrue(estimator.observe(observation(80, north = 0.152, east = 0.742), time(80)))
        assertEquals(time(80), checkNotNull(estimator.estimate(time(80), 0.0, time(80)).estimate).lastVisionAtNanos)
    }

    @Test
    fun `rejected event boundaries do not inject extra process noise`() {
        val uninterrupted = initialized(config = permissiveConfig())
        val split = initialized(config = permissiveConfig())
        assertFalse(split.observe(observation(80, north = 30.0, east = 30.0), time(80)))
        val expected = checkNotNull(uninterrupted.estimate(time(200), 0.0, time(200), 0.0).estimate)
        val actual = checkNotNull(split.estimate(time(200), 0.0, time(200), 0.0).estimate)
        assertStateEquivalent(expected, actual)
    }

    @Test
    fun `heading velocity and vision expire independently with inclusive age limits`() {
        val config = permissiveConfig().copy(
            maximumVisionAgeNanos = 200_000_000L,
            maximumVelocityAgeNanos = 100_000_000L,
            maximumHeadingAgeNanos = 50_000_000L,
        )
        val estimator = initialized(config)
        assertNotNull(estimator.estimate(time(50), 0.0, time(0), 0.0).estimate)
        assertEquals("heading_expired", estimator.estimate(time(50) + 1L, 0.0, time(0), 0.0).reason)
        assertNotNull(estimator.estimate(time(100), 0.0, time(100), 0.0).estimate)
        assertEquals("velocity_expired", estimator.estimate(time(100) + 1L, 0.0, time(100) + 1L, 0.0).reason)
        velocity(estimator, 150, 0.6, 0.1)
        assertNotNull(estimator.estimate(time(200), 0.0, time(200), 0.0).estimate)
        assertEquals("vision_expired", estimator.estimate(time(200) + 1L, 0.0, time(200) + 1L, 0.0).reason)
    }

    @Test
    fun `position and velocity uncertainty gate the predicted horizon independently`() {
        val positionLimited = initialized(permissiveConfig().copy(maximumPositionStdMeters = 0.02))
        assertNotNull(positionLimited.estimate(time(0), 0.0, time(0), 0.0).estimate)
        assertEquals("position_uncertainty", positionLimited.estimate(time(0), 0.0, time(0), 0.05).reason)
        assertNotNull(positionLimited.estimate(time(0), 0.0, time(0), 0.0).estimate)

        val velocityLimited = initialized(permissiveConfig().copy(maximumVelocityStdMetersPerSecond = 0.1))
        assertNotNull(velocityLimited.estimate(time(0), 0.0, time(0), 0.0).estimate)
        assertEquals("velocity_uncertainty", velocityLimited.estimate(time(100), 0.0, time(100), 0.0).reason)
    }

    @Test
    fun `malformed geometry covariance and image heading never update the posterior`() {
        val estimator = initialized()
        assertFalse(estimator.observe(observation(10).copy(centerForwardMeters = Double.NaN), time(10)))
        assertFalse(estimator.observe(observation(20).copy(centerCovariance = 0.001), time(20)))
        assertFalse(estimator.observe(observation(30).copy(headingAtNanos = time(31)), time(30)))
        assertFalse(estimator.observe(observation(40).copy(headingAtNanos = time(40) - 250_000_001L), time(40)))
        val result = estimator.estimate(time(40), 0.0, time(40), 0.0)
        assertEquals(1, result.acceptedVisionCount)
        assertEquals(4, result.rejectedVisionCount)
        assertEquals(time(0), checkNotNull(result.estimate).lastVisionAtNanos)
        assertEquals(0.176, result.estimate?.centerForwardMeters ?: Double.NaN, 1e-12)
    }

    @Test
    fun `receipt and prediction timestamp validation cannot fabricate freshness`() {
        val estimator = initialized(config = permissiveConfig())
        assertFalse(estimator.updateVelocity(Double.NaN, 0.0, time(20), time(20)))
        assertFalse(estimator.updateVelocity(0.6, 0.1, time(21), time(20)))
        assertFalse(estimator.updateVelocity(0.6, 0.1, time(0) - 1L, time(20)))
        assertFalse(estimator.observe(observation(21), time(20)))
        assertFalse(estimator.observe(observation(-1), time(20)))
        velocity(estimator, 20, 0.6, 0.1)
        assertFalse(estimator.updateVelocity(0.6, 0.1, time(19), time(20)))
        assertFalse(estimator.updateVelocity(0.6, 0.1, time(20), time(20)))
        assertEquals("invalid_query_time", estimator.estimate(time(19), 0.0, time(19)).reason)
        assertEquals("invalid_heading", estimator.estimate(time(20), 0.0, time(21)).reason)
        assertEquals("invalid_heading", estimator.estimate(time(20), Double.NaN, time(20)).reason)
        assertEquals("invalid_prediction_horizon", estimator.estimate(time(20), 0.0, time(20), -0.001).reason)
        assertEquals("invalid_prediction_horizon", estimator.estimate(time(20), 0.0, time(20), Double.NaN).reason)
        assertNotNull(estimator.estimate(time(20), 0.0, time(20), 0.1).estimate)
        assertEquals("invalid_prediction_horizon", estimator.estimate(time(20), 0.0, time(20), 0.100001).reason)
        assertEquals(time(20), checkNotNull(estimator.estimate(time(20), 0.0, time(20)).estimate).lastVelocityAtNanos)

        val nearClockLimit = CircleStateEstimator(permissiveConfig()).also { it.reset(Long.MAX_VALUE - 10L) }
        assertTrue(nearClockLimit.updateVelocity(0.0, 0.0, Long.MAX_VALUE - 10L, Long.MAX_VALUE - 10L))
        assertTrue(nearClockLimit.observe(observation(0).copy(sampleAtNanos = Long.MAX_VALUE - 10L, headingAtNanos = Long.MAX_VALUE - 10L), Long.MAX_VALUE - 10L))
        assertEquals("invalid_prediction_time", nearClockLimit.estimate(Long.MAX_VALUE - 10L, 0.0, Long.MAX_VALUE - 10L).reason)
    }

    @Test
    fun `history overflow latches closed until reset rather than silently discarding chronology`() {
        val estimator = initialized(CircleModelConfig(historyCapacity = 8))
        for (index in 1..6) velocity(estimator, index.toLong(), 0.6, 0.1)
        assertFalse(estimator.updateVelocity(0.6, 0.1, time(7), time(7)))
        assertEquals("history_overflow", estimator.estimate(time(7), 0.0, time(7)).reason)
        assertFalse(estimator.observe(observation(8), time(8)))
        estimator.reset(time(10))
        assertEquals("vision_unavailable", estimator.estimate(time(10), 0.0, time(10)).reason)
        velocity(estimator, 10, 0.6, 0.1)
        assertTrue(estimator.observe(observation(10), time(10)))
        assertNotNull(estimator.estimate(time(10), 0.0, time(10)).estimate)
    }

    @Test
    fun `reset clears provenance and rejects old session delayed frames`() {
        val estimator = CircleStateEstimator()
        assertEquals("not_started", estimator.estimate(time(0), 0.0, time(0)).reason)
        estimator.reset(time(0))
        velocity(estimator, 0, 0.6, 0.1)
        assertTrue(estimator.observe(observation(0), time(0)))
        estimator.reset(time(100))
        assertFalse(estimator.observe(observation(0), time(100)))
        assertFalse(estimator.updateVelocity(0.6, 0.1, time(0), time(100)))
        val empty = estimator.estimate(time(100), 0.0, time(100))
        assertNull(empty.estimate)
        assertEquals(0, empty.acceptedVisionCount)
        assertEquals(0, empty.rejectedVisionCount)
        assertEquals(0, empty.acceptedVelocityCount)
        assertEquals(0, empty.lateReplayCount)
        assertNull(empty.lastInnovationSquared)
        velocity(estimator, 100, 0.2, -0.1)
        assertTrue(estimator.observe(observation(100, north = -0.4, east = -0.6), time(100)))
        val fresh = checkNotNull(estimator.estimate(time(100), 0.0, time(100), 0.0).estimate)
        assertEquals(-0.4, fresh.centerForwardMeters, 1e-12)
        assertEquals(0.2, fresh.forwardVelocityMetersPerSecond, 1e-12)
    }

    @Test(expected = IllegalArgumentException::class)
    fun `unrepresentable process noise is rejected at construction`() {
        CircleStateEstimator(CircleModelConfig(accelerationNoiseMetersPerSecondSquared = Double.MAX_VALUE))
    }

    @Test(expected = IllegalArgumentException::class)
    fun `history cannot be shorter than accepted measurement latency`() {
        CircleStateEstimator(CircleModelConfig(historyDurationNanos = 249_999_999L))
    }

    @Test(expected = IllegalArgumentException::class)
    fun `prediction configuration cannot exceed safety bound`() {
        CircleStateEstimator(CircleModelConfig(maximumPredictionSeconds = 0.3))
    }

    @Test(expected = IllegalArgumentException::class)
    fun `negative session start timestamp is rejected`() {
        CircleStateEstimator().reset(-1L)
    }

    @Test
    fun `late monotonic velocity stream replays intervening images without changing the posterior`() {
        val chronological = initialized()
        val delayed = initialized()
        for (millis in listOf(50L, 100L, 150L)) {
            velocity(chronological, millis, 0.6, 0.1)
            val visionAt = millis + 25L
            val image = observation(visionAt, north = 0.2 - 0.6 * visionAt / 1000.0,
                east = 0.75 - 0.1 * visionAt / 1000.0)
            assertTrue(chronological.observe(image, time(visionAt)))
            assertTrue(delayed.observe(image, time(visionAt)))
        }
        for (millis in listOf(50L, 100L, 150L)) {
            assertTrue(delayed.updateVelocity(0.6, 0.1, time(millis), time(175)))
        }
        val expected = chronological.estimate(time(175), 23.0, time(175))
        val actual = delayed.estimate(time(175), 23.0, time(175))
        assertEquivalent(expected, actual)
        assertEquals(3, actual.lateReplayCount)
        assertEquals(time(150), checkNotNull(actual.estimate).lastVelocityAtNanos)
    }

    private fun initialized(config: CircleModelConfig = CircleModelConfig()): CircleStateEstimator =
        CircleStateEstimator(config).also {
            it.reset(time(0))
            velocity(it, 0, 0.6, 0.1)
            assertTrue(it.observe(observation(0), time(0)))
        }

    private fun permissiveConfig() = CircleModelConfig(
        maximumPositionStdMeters = 1.0,
        maximumVelocityStdMetersPerSecond = 1.0,
    )

    private fun velocity(estimator: CircleStateEstimator, millis: Long, north: Double, east: Double) {
        assertTrue(estimator.updateVelocity(north, east, time(millis), time(millis)))
    }

    private fun observation(millis: Long, north: Double = 0.2, east: Double = 0.75) = CircleArcObservation(
        centerForwardMeters = north,
        centerRightMeters = east,
        centerForwardVariance = 0.0004,
        centerRightVariance = 0.0004,
        centerCovariance = 0.0,
        residualRmsMeters = 0.005,
        inlierCount = 20,
        pointCount = 20,
        arcSpanRadians = 1.0,
        sampleAtNanos = time(millis),
        headingDegrees = 0.0,
        headingAtNanos = time(millis),
    )

    private fun assertEquivalent(expected: CircleEstimateResult, actual: CircleEstimateResult) {
        assertEquals(expected.reason, actual.reason)
        assertStateEquivalent(checkNotNull(expected.estimate), checkNotNull(actual.estimate))
        assertEquals(expected.acceptedVisionCount, actual.acceptedVisionCount)
        assertEquals(expected.rejectedVisionCount, actual.rejectedVisionCount)
        assertEquals(expected.acceptedVelocityCount, actual.acceptedVelocityCount)
        if (expected.lastInnovationSquared == null) {
            assertNull(actual.lastInnovationSquared)
        } else {
            assertEquals(checkNotNull(expected.lastInnovationSquared), checkNotNull(actual.lastInnovationSquared), 1e-10)
        }
    }

    private fun assertStateEquivalent(expected: CircleStateEstimate, actual: CircleStateEstimate) {
        assertEquals(expected.centerForwardMeters, actual.centerForwardMeters, 1e-10)
        assertEquals(expected.centerRightMeters, actual.centerRightMeters, 1e-10)
        assertEquals(expected.forwardVelocityMetersPerSecond, actual.forwardVelocityMetersPerSecond, 1e-10)
        assertEquals(expected.rightVelocityMetersPerSecond, actual.rightVelocityMetersPerSecond, 1e-10)
        assertEquals(expected.positionStdMeters, actual.positionStdMeters, 1e-10)
        assertEquals(expected.velocityStdMetersPerSecond, actual.velocityStdMetersPerSecond, 1e-10)
        assertEquals(expected.estimateAtNanos, actual.estimateAtNanos)
        assertEquals(expected.lastVisionAtNanos, actual.lastVisionAtNanos)
        assertEquals(expected.lastVelocityAtNanos, actual.lastVelocityAtNanos)
        assertEquals(expected.predictionSeconds, actual.predictionSeconds, 0.0)
    }

    private fun time(millis: Long): Long = 1_000_000_000L + millis * 1_000_000L
}
