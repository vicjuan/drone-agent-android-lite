package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.sign

internal data class LapEvent(
    val lapIndex: Int,
    val lapSeconds: Double,
)

/**
 * Lap diagnostic requiring both one full angular turn and enough travelled distance.
 * The distance gate prevents controller-angle integration from reporting a stationary lap.
 */
internal class LapTimer(
    private val degreesPerLap: Double = 360.0,
    private val minimumDistanceMeters: Double = 0.0,
) {
    var lapCount: Int = 0
        private set
    var progressDegrees: Double = 0.0
        private set
    var currentLapElapsedSeconds: Double = 0.0
        private set
    var currentLapDistanceMeters: Double = 0.0
        private set
    var armed: Boolean = false
        private set

    private var previousAngleDegrees = 0.0
    private var lapStartedAtNanos = 0L
    private var previousUpdateAtNanos = 0L
    private var directionSign = 0.0

    fun arm(nowNanos: Long, initialAngleDegrees: Double) {
        require(initialAngleDegrees.isFinite())
        lapCount = 0
        progressDegrees = 0.0
        currentLapElapsedSeconds = 0.0
        currentLapDistanceMeters = 0.0
        previousAngleDegrees = initialAngleDegrees
        lapStartedAtNanos = nowNanos
        previousUpdateAtNanos = nowNanos
        directionSign = 0.0
        armed = true
    }

    fun reset() {
        lapCount = 0
        progressDegrees = 0.0
        currentLapElapsedSeconds = 0.0
        currentLapDistanceMeters = 0.0
        lapStartedAtNanos = 0L
        previousUpdateAtNanos = 0L
        directionSign = 0.0
        armed = false
    }

    fun update(
        nowNanos: Long,
        angleDegrees: Double,
        groundSpeedMetersPerSecond: Double = 0.0,
    ): LapEvent? {
        if (!armed || !angleDegrees.isFinite()) return null
        val elapsedSinceUpdateSeconds =
            (nowNanos - previousUpdateAtNanos).coerceAtLeast(0L) / 1_000_000_000.0
        previousUpdateAtNanos = nowNanos
        if (groundSpeedMetersPerSecond.isFinite() && groundSpeedMetersPerSecond > 0.0) {
            currentLapDistanceMeters +=
                groundSpeedMetersPerSecond * elapsedSinceUpdateSeconds
        }
        val delta = shortestAngularDelta(previousAngleDegrees, angleDegrees)
        previousAngleDegrees = angleDegrees
        currentLapElapsedSeconds =
            (nowNanos - lapStartedAtNanos).coerceAtLeast(0L) / 1_000_000_000.0
        if (directionSign == 0.0 && abs(delta) >= MIN_DIRECTION_DELTA_DEGREES) {
            directionSign = sign(delta)
        }
        if (directionSign == 0.0 || delta * directionSign <= 0.0) return null
        progressDegrees += abs(delta)
        if (
            progressDegrees < degreesPerLap ||
            currentLapDistanceMeters < minimumDistanceMeters
        ) {
            return null
        }
        progressDegrees -= degreesPerLap
        lapCount += 1
        val completedSeconds = currentLapElapsedSeconds
        lapStartedAtNanos = nowNanos
        currentLapElapsedSeconds = 0.0
        currentLapDistanceMeters = 0.0
        return LapEvent(lapCount, completedSeconds)
    }

    private companion object {
        const val MIN_DIRECTION_DELTA_DEGREES = 0.2
    }
}
