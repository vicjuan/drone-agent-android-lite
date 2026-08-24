package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.sign

internal data class LapEvent(
    val lapIndex: Int,
    val lapSeconds: Double,
)

/** Angular progress timer for flight diagnostics; it does not prove spatial start-line crossing. */
internal class LapTimer(
    private val degreesPerLap: Double = 360.0,
) {
    var lapCount: Int = 0
        private set
    var progressDegrees: Double = 0.0
        private set
    var currentLapElapsedSeconds: Double = 0.0
        private set
    var armed: Boolean = false
        private set

    private var previousAngleDegrees = 0.0
    private var lapStartedAtNanos = 0L
    private var directionSign = 0.0

    fun arm(nowNanos: Long, initialAngleDegrees: Double) {
        require(initialAngleDegrees.isFinite())
        lapCount = 0
        progressDegrees = 0.0
        currentLapElapsedSeconds = 0.0
        previousAngleDegrees = initialAngleDegrees
        lapStartedAtNanos = nowNanos
        directionSign = 0.0
        armed = true
    }

    fun reset() {
        lapCount = 0
        progressDegrees = 0.0
        currentLapElapsedSeconds = 0.0
        lapStartedAtNanos = 0L
        directionSign = 0.0
        armed = false
    }

    fun update(nowNanos: Long, angleDegrees: Double): LapEvent? {
        if (!armed || !angleDegrees.isFinite()) return null
        val delta = shortestAngularDelta(previousAngleDegrees, angleDegrees)
        previousAngleDegrees = angleDegrees
        currentLapElapsedSeconds = (nowNanos - lapStartedAtNanos).coerceAtLeast(0L) / 1_000_000_000.0
        if (directionSign == 0.0 && abs(delta) >= MIN_DIRECTION_DELTA_DEGREES) {
            directionSign = sign(delta)
        }
        if (directionSign == 0.0 || delta * directionSign <= 0.0) return null
        progressDegrees += abs(delta)
        if (progressDegrees < degreesPerLap) return null
        progressDegrees -= degreesPerLap
        lapCount += 1
        val completedSeconds = currentLapElapsedSeconds
        lapStartedAtNanos = nowNanos
        currentLapElapsedSeconds = 0.0
        return LapEvent(lapCount, completedSeconds)
    }

    private companion object {
        const val MIN_DIRECTION_DELTA_DEGREES = 0.2
    }
}
