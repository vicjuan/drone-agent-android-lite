package com.durendal.droneagent.lite

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class HorizontalObstacleFilterTest {
    @Test
    fun `dominant ranges matching downward height are treated as floor echoes`() {
        val distances = List(345) { 510 } + List(15) { OBSTACLE_NOT_DETECTED_MM }

        val summary = HorizontalObstacleFilter.summarize(distances, downwardDistanceMm = 500)

        assertTrue(summary.groundEchoDominant)
        assertEquals(345, summary.groundEchoCount)
        assertNull(summary.nearestActionableMm)
    }

    @Test
    fun `obstacle substantially closer than floor remains actionable`() {
        val distances = List(340) { 510 } + List(5) { 300 } + List(15) { OBSTACLE_NOT_DETECTED_MM }

        val summary = HorizontalObstacleFilter.summarize(distances, downwardDistanceMm = 500)

        assertTrue(summary.groundEchoDominant)
        assertEquals(300, summary.nearestActionableMm)
    }

    @Test
    fun `dense low altitude ring matches captured Mini 4 Pro sample shape`() {
        val distances =
            List(100) { 510 } + List(240) { 900 } + List(20) { OBSTACLE_NOT_DETECTED_MM }

        val summary = HorizontalObstacleFilter.summarize(distances, downwardDistanceMm = 500)

        assertTrue(summary.groundEchoDominant)
        assertEquals(100, summary.groundEchoCount)
        assertNull(summary.nearestActionableMm)
    }

    @Test
    fun `sparse close range is never discarded as floor`() {
        val distances = listOf(510) + List(359) { OBSTACLE_NOT_DETECTED_MM }

        val summary = HorizontalObstacleFilter.summarize(distances, downwardDistanceMm = 500)

        assertFalse(summary.groundEchoDominant)
        assertEquals(510, summary.nearestActionableMm)
    }

    @Test
    fun `all sentinel sample remains unknown rather than clear`() {
        val summary = HorizontalObstacleFilter.summarize(
            List(360) { OBSTACLE_NOT_DETECTED_MM },
            downwardDistanceMm = 500,
        )

        assertFalse(summary.groundEchoDominant)
        assertNull(summary.nearestActionableMm)
    }
}
