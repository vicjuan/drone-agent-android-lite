package com.durendal.droneagent.lite

/** DJI sentinel meaning no obstacle was detected in that direction. */
internal const val OBSTACLE_NOT_DETECTED_MM = 60_000

/** Existing operator-control clearance; preserve the semantics used by manual flights. */
internal const val MANUAL_HORIZONTAL_CLEARANCE_MM = 150
internal const val MAX_MANUAL_OBSTACLE_SAMPLE_AGE_MS = 500L

/** Emergency stop distance for autonomous motion up to 0.24 m/s. */
internal const val AUTONOMOUS_HORIZONTAL_CLEARANCE_MM = 500

/** A 250 ms ranging gap can cover 6 cm at the current maximum tracking speed. */
internal const val MAX_AUTONOMOUS_OBSTACLE_SAMPLE_AGE_MS = 250L

internal fun isFreshObstacleSample(
    sampleReceived: Boolean,
    sampleAtNanos: Long,
    nowNanos: Long,
): Boolean =
    sampleReceived &&
        sampleAtNanos != 0L &&
        (nowNanos - sampleAtNanos) / 1_000_000L <=
        MAX_AUTONOMOUS_OBSTACLE_SAMPLE_AGE_MS

internal fun breachesAutonomousHorizontalClearance(distanceMm: Int?): Boolean =
    distanceMm != null && distanceMm <= AUTONOMOUS_HORIZONTAL_CLEARANCE_MM

/** True only for a measured range; zero and DJI's 60,000 sentinel are unknown. */
internal fun isDetectedObstacleDistance(distanceMm: Int): Boolean =
    distanceMm in 1 until OBSTACLE_NOT_DETECTED_MM
