package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.sign

internal enum class TapeTrackingPhase {
    DISABLED,
    RECENTERING,
    TRACKING,
    SEARCHING,
}

internal enum class TrackingGimbalTarget {
    DOWN_CENTER,
    SEARCH_LEFT,
    SEARCH_RIGHT,
}

internal data class TapeTrackingDecision(
    val phase: TapeTrackingPhase,
    val yawRateDegreesPerSecond: Double,
    val gimbalTarget: TrackingGimbalTarget? = null,
    val searchTimedOut: Boolean = false,
)

/**
 * Pure timing and steering policy for black-tape tracking. Aircraft and gimbal
 * commands remain in MainActivity, so this class can be tested without DJI SDK.
 */
internal class TapeTrackingController {
    var enabled: Boolean = false
        private set

    var phase: TapeTrackingPhase = TapeTrackingPhase.DISABLED
        private set

    private var lastDetectionAtNanos = 0L
    private var latestAngleDegrees: Double? = null
    private var searchStartedAtNanos = 0L
    private var nextSweepAtNanos = 0L
    private var recenterUntilNanos = 0L
    private var consecutiveSearchDetections = 0
    private var nextSearchTarget = TrackingGimbalTarget.SEARCH_RIGHT
    private var pendingGimbalTarget: TrackingGimbalTarget? = null

    fun start(nowNanos: Long) {
        enabled = true
        phase = TapeTrackingPhase.RECENTERING
        lastDetectionAtNanos = nowNanos
        latestAngleDegrees = null
        searchStartedAtNanos = 0L
        nextSweepAtNanos = 0L
        consecutiveSearchDetections = 0
        pendingGimbalTarget = TrackingGimbalTarget.DOWN_CENTER
        recenterUntilNanos = nowNanos + RECENTER_DURATION_NANOS
    }

    fun stop() {
        enabled = false
        phase = TapeTrackingPhase.DISABLED
        latestAngleDegrees = null
        consecutiveSearchDetections = 0
        pendingGimbalTarget = null
    }

    fun observe(angleFromVerticalDegrees: Double?, nowNanos: Long) {
        if (!enabled) return
        if (angleFromVerticalDegrees == null) {
            latestAngleDegrees = null
            if (phase == TapeTrackingPhase.SEARCHING) consecutiveSearchDetections = 0
            return
        }
        require(angleFromVerticalDegrees in -90.0..90.0)
        when (phase) {
            TapeTrackingPhase.TRACKING -> {
                latestAngleDegrees = angleFromVerticalDegrees
                lastDetectionAtNanos = nowNanos
            }

            TapeTrackingPhase.SEARCHING -> {
                consecutiveSearchDetections += 1
                if (consecutiveSearchDetections >= REACQUIRE_DETECTION_COUNT) {
                    phase = TapeTrackingPhase.RECENTERING
                    latestAngleDegrees = null
                    lastDetectionAtNanos = nowNanos
                    pendingGimbalTarget = TrackingGimbalTarget.DOWN_CENTER
                    recenterUntilNanos = nowNanos + RECENTER_DURATION_NANOS
                }
            }

            TapeTrackingPhase.DISABLED,
            TapeTrackingPhase.RECENTERING,
            -> Unit
        }
    }

    fun tick(nowNanos: Long): TapeTrackingDecision {
        if (!enabled) return TapeTrackingDecision(TapeTrackingPhase.DISABLED, 0.0)

        if (phase == TapeTrackingPhase.RECENTERING && nowNanos >= recenterUntilNanos) {
            phase = TapeTrackingPhase.TRACKING
            lastDetectionAtNanos = nowNanos
            latestAngleDegrees = null
        }
        if (
            phase == TapeTrackingPhase.TRACKING &&
            nowNanos - lastDetectionAtNanos >= DETECTION_LOST_NANOS
        ) {
            phase = TapeTrackingPhase.SEARCHING
            searchStartedAtNanos = nowNanos
            consecutiveSearchDetections = 0
            pendingGimbalTarget = TrackingGimbalTarget.SEARCH_LEFT
            nextSearchTarget = TrackingGimbalTarget.SEARCH_RIGHT
            nextSweepAtNanos = nowNanos + SEARCH_SWEEP_DURATION_NANOS
        }
        if (phase == TapeTrackingPhase.SEARCHING) {
            if (nowNanos - searchStartedAtNanos >= SEARCH_TIMEOUT_NANOS) {
                enabled = false
                phase = TapeTrackingPhase.DISABLED
                pendingGimbalTarget = null
                return TapeTrackingDecision(
                    phase = TapeTrackingPhase.DISABLED,
                    yawRateDegreesPerSecond = 0.0,
                    gimbalTarget = TrackingGimbalTarget.DOWN_CENTER,
                    searchTimedOut = true,
                )
            }
            if (nowNanos >= nextSweepAtNanos) {
                pendingGimbalTarget = nextSearchTarget
                nextSearchTarget = when (nextSearchTarget) {
                    TrackingGimbalTarget.SEARCH_LEFT -> TrackingGimbalTarget.SEARCH_RIGHT
                    TrackingGimbalTarget.SEARCH_RIGHT -> TrackingGimbalTarget.SEARCH_LEFT
                    TrackingGimbalTarget.DOWN_CENTER -> TrackingGimbalTarget.SEARCH_LEFT
                }
                nextSweepAtNanos = nowNanos + SEARCH_SWEEP_DURATION_NANOS
            }
        }

        val target = pendingGimbalTarget
        pendingGimbalTarget = null
        return TapeTrackingDecision(
            phase = phase,
            yawRateDegreesPerSecond = desiredYawRate(nowNanos),
            gimbalTarget = target,
        )
    }

    private fun desiredYawRate(nowNanos: Long): Double {
        if (phase != TapeTrackingPhase.TRACKING) return 0.0
        if (nowNanos - lastDetectionAtNanos > DETECTION_COMMAND_STALE_NANOS) return 0.0
        val angle = latestAngleDegrees ?: return 0.0
        if (abs(angle) <= ALIGNMENT_TOLERANCE_DEGREES) return 0.0
        return sign(angle) * ALIGNMENT_YAW_RATE_DEGREES_PER_SECOND
    }

    internal companion object {
        const val ALIGNMENT_TOLERANCE_DEGREES = 10.0
        const val ALIGNMENT_YAW_RATE_DEGREES_PER_SECOND = 20.0
        const val REACQUIRE_DETECTION_COUNT = 3
        const val RECENTER_DURATION_NANOS = 2_000_000_000L
        const val DETECTION_COMMAND_STALE_NANOS = 1_000_000_000L
        const val DETECTION_LOST_NANOS = 5_000_000_000L
        const val SEARCH_SWEEP_DURATION_NANOS = 3_000_000_000L
        const val SEARCH_TIMEOUT_NANOS = 20_000_000_000L
    }
}
