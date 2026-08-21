package com.durendal.droneagent.lite

import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class ObstacleRangesTest {
    @Test
    fun noDetectionSentinelDoesNotEstablishClearance() {
        assertFalse(isDetectedObstacleDistance(OBSTACLE_NOT_DETECTED_MM))
    }

    @Test
    fun onlyPositiveRangesBelowTheSentinelAreMeasurements() {
        assertFalse(isDetectedObstacleDistance(0))
        assertFalse(isDetectedObstacleDistance(-1))
        assertTrue(isDetectedObstacleDistance(1))
        assertTrue(isDetectedObstacleDistance(OBSTACLE_NOT_DETECTED_MM - 1))
    }

    @Test
    fun autonomousTrackingRequiresFreshObstacleTelemetry() {
        val sampleAtNanos = 1_000_000_000L
        assertTrue(
            isFreshObstacleSample(
                sampleReceived = true,
                sampleAtNanos = sampleAtNanos,
                nowNanos =
                    sampleAtNanos +
                        MAX_AUTONOMOUS_OBSTACLE_SAMPLE_AGE_MS * 1_000_000L,
            ),
        )
        assertFalse(
            isFreshObstacleSample(
                sampleReceived = true,
                sampleAtNanos = sampleAtNanos,
                nowNanos =
                    sampleAtNanos +
                        (MAX_AUTONOMOUS_OBSTACLE_SAMPLE_AGE_MS + 1L) * 1_000_000L,
            ),
        )
    }

    @Test
    fun freshAllSentinelCallbackIsAvailableTelemetry() {
        val sampleAtNanos = 1_000_000_000L

        assertTrue(
            isFreshObstacleSample(
                sampleReceived = true,
                sampleAtNanos = sampleAtNanos,
                nowNanos = sampleAtNanos,
            ),
        )
    }

    @Test
    fun autonomousTrackingStopsAtTheClearanceBoundary() {
        assertFalse(
            breachesAutonomousHorizontalClearance(
                AUTONOMOUS_HORIZONTAL_CLEARANCE_MM + 1,
            ),
        )
        assertTrue(
            breachesAutonomousHorizontalClearance(
                AUTONOMOUS_HORIZONTAL_CLEARANCE_MM,
            ),
        )
    }
}
