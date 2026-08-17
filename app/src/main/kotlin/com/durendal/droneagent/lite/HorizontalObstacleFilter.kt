package com.durendal.droneagent.lite

import kotlin.math.abs

internal data class HorizontalObstacleSummary(
    val rawMinimumMm: Int?,
    val rawMaximumMm: Int?,
    val detectedCount: Int,
    val groundEchoCount: Int,
    val groundEchoDominant: Boolean,
    val nearestActionableMm: Int?,
)

/** Separates the low-altitude floor ring from actionable horizontal obstacles. */
internal object HorizontalObstacleFilter {
    fun summarize(
        distancesMm: List<Int>,
        downwardDistanceMm: Int,
    ): HorizontalObstacleSummary {
        if (distancesMm.isEmpty()) {
            return HorizontalObstacleSummary(null, null, 0, 0, false, null)
        }

        var detectedCount = 0
        var groundEchoCount = 0
        var nearestDetectedMm = Int.MAX_VALUE
        for (distanceMm in distancesMm) {
            if (!isDetectedObstacleDistance(distanceMm)) continue
            detectedCount += 1
            if (distanceMm < nearestDetectedMm) nearestDetectedMm = distanceMm
            if (isGroundEcho(distanceMm, downwardDistanceMm)) groundEchoCount += 1
        }
        val groundEchoDominant =
            downwardDistanceMm in MIN_LOW_ALTITUDE_MM..MAX_LOW_ALTITUDE_MM &&
                groundEchoCount >= MIN_GROUND_ECHO_EVIDENCE_COUNT &&
                detectedCount * GROUND_RING_DENOMINATOR >=
                distancesMm.size * GROUND_RING_NUMERATOR

        var nearestActionableMm = Int.MAX_VALUE
        if (groundEchoDominant) {
            for (distanceMm in distancesMm) {
                if (
                    isDetectedObstacleDistance(distanceMm) &&
                    distanceMm < downwardDistanceMm - CLOSER_THAN_FLOOR_MARGIN_MM &&
                    distanceMm < nearestActionableMm
                ) {
                    nearestActionableMm = distanceMm
                }
            }
        } else {
            nearestActionableMm = nearestDetectedMm
        }

        return HorizontalObstacleSummary(
            rawMinimumMm = distancesMm.minOrNull(),
            rawMaximumMm = distancesMm.maxOrNull(),
            detectedCount = detectedCount,
            groundEchoCount = groundEchoCount,
            groundEchoDominant = groundEchoDominant,
            nearestActionableMm = nearestActionableMm.takeIf { it != Int.MAX_VALUE },
        )
    }

    private fun isGroundEcho(distanceMm: Int, downwardDistanceMm: Int): Boolean =
        isDetectedObstacleDistance(downwardDistanceMm) &&
            abs(distanceMm - downwardDistanceMm) <= GROUND_ECHO_TOLERANCE_MM

    private const val GROUND_ECHO_TOLERANCE_MM = 150
    private const val CLOSER_THAN_FLOOR_MARGIN_MM = 100
    private const val MIN_LOW_ALTITUDE_MM = 200
    private const val MAX_LOW_ALTITUDE_MM = 1_000
    private const val MIN_GROUND_ECHO_EVIDENCE_COUNT = 60
    private const val GROUND_RING_NUMERATOR = 3
    private const val GROUND_RING_DENOMINATOR = 4
}
