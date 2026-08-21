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
