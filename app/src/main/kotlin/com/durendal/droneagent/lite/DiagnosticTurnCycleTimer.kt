package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.sign

internal data class DiagnosticTurnCycleEvent(
    val index: Int,
    val elapsedSeconds: Double,
    val distanceMeters: Double,
    val turnDegrees: Double,
)

/**
 * Reports distance-gated controller turn cycles for diagnostics.
 *
 * This is deliberately not a physical lap timer: it has no fixed spatial start/finish marker.
 * The distance gate only rejects stationary controller-angle integration.
 */
internal class DiagnosticTurnCycleTimer(
    private val degreesPerCycle: Double = 360.0,
    private val minimumDistanceMeters: Double = 0.0,
) {
    var cycleCount: Int = 0
        private set
    var progressDegrees: Double = 0.0
        private set
    var currentCycleElapsedSeconds: Double = 0.0
        private set
    var currentCycleDistanceMeters: Double = 0.0
        private set
    var armed: Boolean = false
        private set

    private var previousAngleDegrees = 0.0
    private var cycleStartedAtNanos = 0L
    private var previousUpdateAtNanos = 0L
    private var directionSign = 0.0

    fun arm(nowNanos: Long, initialAngleDegrees: Double) {
        require(initialAngleDegrees.isFinite())
        cycleCount = 0
        progressDegrees = 0.0
        currentCycleElapsedSeconds = 0.0
        currentCycleDistanceMeters = 0.0
        previousAngleDegrees = initialAngleDegrees
        cycleStartedAtNanos = nowNanos
        previousUpdateAtNanos = nowNanos
        directionSign = 0.0
        armed = true
    }

    fun reset() {
        cycleCount = 0
        progressDegrees = 0.0
        currentCycleElapsedSeconds = 0.0
        currentCycleDistanceMeters = 0.0
        cycleStartedAtNanos = 0L
        previousUpdateAtNanos = 0L
        directionSign = 0.0
        armed = false
    }

    fun update(
        nowNanos: Long,
        angleDegrees: Double,
        groundSpeedMetersPerSecond: Double = 0.0,
    ): DiagnosticTurnCycleEvent? {
        if (!armed || !angleDegrees.isFinite()) return null
        val elapsedSinceUpdateSeconds =
            (nowNanos - previousUpdateAtNanos).coerceAtLeast(0L) / 1_000_000_000.0
        previousUpdateAtNanos = nowNanos
        if (groundSpeedMetersPerSecond.isFinite() && groundSpeedMetersPerSecond > 0.0) {
            currentCycleDistanceMeters +=
                groundSpeedMetersPerSecond * elapsedSinceUpdateSeconds
        }
        val delta = shortestAngularDelta(previousAngleDegrees, angleDegrees)
        previousAngleDegrees = angleDegrees
        currentCycleElapsedSeconds =
            (nowNanos - cycleStartedAtNanos).coerceAtLeast(0L) / 1_000_000_000.0
        if (directionSign == 0.0 && abs(delta) >= MIN_DIRECTION_DELTA_DEGREES) {
            directionSign = sign(delta)
        }
        if (directionSign == 0.0 || delta * directionSign <= 0.0) return null
        progressDegrees += abs(delta)
        if (
            progressDegrees < degreesPerCycle ||
            currentCycleDistanceMeters < minimumDistanceMeters
        ) {
            return null
        }

        val completedDistanceMeters = currentCycleDistanceMeters
        val completedTurnDegrees = progressDegrees
        progressDegrees -= degreesPerCycle
        cycleCount += 1
        val completedSeconds = currentCycleElapsedSeconds
        cycleStartedAtNanos = nowNanos
        currentCycleElapsedSeconds = 0.0
        currentCycleDistanceMeters = 0.0
        return DiagnosticTurnCycleEvent(
            index = cycleCount,
            elapsedSeconds = completedSeconds,
            distanceMeters = completedDistanceMeters,
            turnDegrees = completedTurnDegrees,
        )
    }

    private companion object {
        const val MIN_DIRECTION_DELTA_DEGREES = 0.2
    }
}
