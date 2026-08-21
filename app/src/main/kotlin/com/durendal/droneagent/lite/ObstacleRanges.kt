package com.durendal.droneagent.lite

/** DJI sentinel meaning no obstacle was detected in that direction. */
internal const val OBSTACLE_NOT_DETECTED_MM = 60_000

/** Emergency stop distance for autonomous motion up to 0.24 m/s. */
internal const val HORIZONTAL_CLEARANCE_MM = 500

/** A 250 ms ranging gap can cover 6 cm at the current maximum tracking speed. */
internal const val MAX_OBSTACLE_SAMPLE_AGE_MS = 250L

internal fun isFreshObstacleSample(
    sampleValid: Boolean,
    sampleAtNanos: Long,
    nowNanos: Long,
): Boolean =
    sampleValid &&
        sampleAtNanos != 0L &&
        (nowNanos - sampleAtNanos) / 1_000_000L <= MAX_OBSTACLE_SAMPLE_AGE_MS

internal fun breachesAutonomousHorizontalClearance(distanceMm: Int?): Boolean =
    distanceMm != null && distanceMm <= HORIZONTAL_CLEARANCE_MM

/** True only for a measured range; zero and DJI's 60,000 sentinel are unknown. */
internal fun isDetectedObstacleDistance(distanceMm: Int): Boolean =
    distanceMm in 1 until OBSTACLE_NOT_DETECTED_MM
