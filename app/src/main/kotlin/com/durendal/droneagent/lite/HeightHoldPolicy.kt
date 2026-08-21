package com.durendal.droneagent.lite

import kotlin.math.abs

/** Pure vertical-control policy shared by every operator-selected height target. */
internal object HeightHoldPolicy {
    const val TARGET_TOLERANCE_METERS = 0.1
    private const val PROPORTIONAL_GAIN = 0.6
    fun isUsableCurrentHeight(meters: Double?): Boolean =
        meters != null && meters.isFinite() && meters >= 0.0

    fun isWithinTarget(targetHeightMeters: Double, currentHeightMeters: Double): Boolean {
        requireValidHeights(targetHeightMeters, currentHeightMeters)
        return abs(targetHeightMeters - currentHeightMeters) <=
            TARGET_TOLERANCE_METERS + FLOATING_POINT_EPSILON
    }

    fun climbRateMetersPerSecond(
        targetHeightMeters: Double,
        currentHeightMeters: Double,
        maximumRateMetersPerSecond: Double,
    ): Double {
        requireValidHeights(targetHeightMeters, currentHeightMeters)
        require(maximumRateMetersPerSecond.isFinite() && maximumRateMetersPerSecond > 0.0)
        return (PROPORTIONAL_GAIN * (targetHeightMeters - currentHeightMeters)).coerceIn(
            -maximumRateMetersPerSecond,
            maximumRateMetersPerSecond,
        )
    }

    private fun requireValidHeights(targetHeightMeters: Double, currentHeightMeters: Double) {
        require(targetHeightMeters.isFinite() && targetHeightMeters > 0.0)
        require(currentHeightMeters.isFinite() && currentHeightMeters >= 0.0)
    }

    private const val FLOATING_POINT_EPSILON = 1e-9
}
