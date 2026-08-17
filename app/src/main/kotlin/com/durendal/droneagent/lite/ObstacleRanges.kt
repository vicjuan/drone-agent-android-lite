package com.durendal.droneagent.lite

/** DJI sentinel meaning no obstacle was detected in that direction. */
internal const val OBSTACLE_NOT_DETECTED_MM = 60_000

/** True only for a measured range; zero and DJI's 60,000 sentinel are unknown. */
internal fun isDetectedObstacleDistance(distanceMm: Int): Boolean =
    distanceMm in 1 until OBSTACLE_NOT_DETECTED_MM
