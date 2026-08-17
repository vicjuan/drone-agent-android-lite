package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.sign

internal enum class TapeTrackingPhase {
    DISABLED,
    RECENTERING,
    TRACKING,
    SEARCHING,
    TURNING,
}

internal enum class TrackingGimbalTarget {
    DOWN_CENTER,
    SEARCH_LEFT,
    SEARCH_RIGHT,
}

internal data class TapeTrackingDecision(
    val phase: TapeTrackingPhase,
    val yawRateDegreesPerSecond: Double,
    val forwardSpeedMetersPerSecond: Double = 0.0,
    val gimbalTarget: TrackingGimbalTarget? = null,
    val endpointReached: Boolean = false,
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
    private var consecutiveEndpointDetections = 0
    private var longestObservedTapeFraction = 0.0
    private var endpointPending = false
    private var nextSearchTarget = TrackingGimbalTarget.SEARCH_RIGHT
    private var pendingGimbalTarget: TrackingGimbalTarget? = null

    fun start(nowNanos: Long) {
        enabled = true
        resetLeg()
        beginRecentering(nowNanos)
    }

    fun stop() {
        enabled = false
        phase = TapeTrackingPhase.DISABLED
        latestAngleDegrees = null
        consecutiveSearchDetections = 0
        consecutiveEndpointDetections = 0
        endpointPending = false
        pendingGimbalTarget = null
    }

    fun resumeAfterTurn(nowNanos: Long) {
        check(enabled && phase == TapeTrackingPhase.TURNING)
        resetLeg()
        beginRecentering(nowNanos)
    }

    fun observe(
        angleFromVerticalDegrees: Double?,
        longSideFraction: Double?,
        nowNanos: Long,
    ) {
        if (!enabled || phase == TapeTrackingPhase.TURNING) return
        if (angleFromVerticalDegrees == null || longSideFraction == null) {
            latestAngleDegrees = null
            consecutiveEndpointDetections = 0
            if (phase == TapeTrackingPhase.SEARCHING) consecutiveSearchDetections = 0
            return
        }
        require(angleFromVerticalDegrees in -90.0..90.0)
        require(longSideFraction > 0.0 && longSideFraction.isFinite())
        when (phase) {
            TapeTrackingPhase.TRACKING -> {
                latestAngleDegrees = angleFromVerticalDegrees
                lastDetectionAtNanos = nowNanos
                longestObservedTapeFraction =
                    maxOf(longestObservedTapeFraction, longSideFraction)
                val endpointCandidate =
                    longestObservedTapeFraction >= ENDPOINT_REFERENCE_MIN_FRACTION &&
                        longSideFraction <= longestObservedTapeFraction * ENDPOINT_LENGTH_RATIO
                consecutiveEndpointDetections =
                    if (endpointCandidate) consecutiveEndpointDetections + 1 else 0
                if (consecutiveEndpointDetections >= ENDPOINT_CONFIRMATION_COUNT) {
                    phase = TapeTrackingPhase.TURNING
                    latestAngleDegrees = null
                    endpointPending = true
                }
            }

            TapeTrackingPhase.SEARCHING -> {
                consecutiveSearchDetections += 1
                if (consecutiveSearchDetections >= REACQUIRE_DETECTION_COUNT) {
                    latestAngleDegrees = null
                    lastDetectionAtNanos = nowNanos
                    consecutiveEndpointDetections = 0
                    beginRecentering(nowNanos)
                }
            }

            TapeTrackingPhase.DISABLED,
            TapeTrackingPhase.RECENTERING,
            TapeTrackingPhase.TURNING,
            -> Unit
        }
    }

    fun tick(nowNanos: Long): TapeTrackingDecision {
        if (!enabled) return TapeTrackingDecision(TapeTrackingPhase.DISABLED, 0.0)
        if (endpointPending) {
            endpointPending = false
            return TapeTrackingDecision(
                phase = TapeTrackingPhase.TURNING,
                yawRateDegreesPerSecond = 0.0,
                endpointReached = true,
            )
        }
        if (phase == TapeTrackingPhase.TURNING) {
            return TapeTrackingDecision(TapeTrackingPhase.TURNING, 0.0)
        }

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
            consecutiveEndpointDetections = 0
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
            forwardSpeedMetersPerSecond = desiredForwardSpeed(nowNanos),
            gimbalTarget = target,
        )
    }

    private fun beginRecentering(nowNanos: Long) {
        phase = TapeTrackingPhase.RECENTERING
        latestAngleDegrees = null
        consecutiveSearchDetections = 0
        pendingGimbalTarget = TrackingGimbalTarget.DOWN_CENTER
        recenterUntilNanos = nowNanos + RECENTER_DURATION_NANOS
    }

    private fun resetLeg() {
        lastDetectionAtNanos = 0L
        searchStartedAtNanos = 0L
        nextSweepAtNanos = 0L
        consecutiveEndpointDetections = 0
        longestObservedTapeFraction = 0.0
        endpointPending = false
    }

    private fun desiredYawRate(nowNanos: Long): Double {
        if (phase != TapeTrackingPhase.TRACKING) return 0.0
        if (nowNanos - lastDetectionAtNanos > DETECTION_COMMAND_STALE_NANOS) return 0.0
        val angle = latestAngleDegrees ?: return 0.0
        if (abs(angle) <= ALIGNMENT_TOLERANCE_DEGREES) return 0.0
        return sign(angle) * ALIGNMENT_YAW_RATE_DEGREES_PER_SECOND
    }

    private fun desiredForwardSpeed(nowNanos: Long): Double {
        if (phase != TapeTrackingPhase.TRACKING || consecutiveEndpointDetections > 0) return 0.0
        if (nowNanos - lastDetectionAtNanos > DETECTION_COMMAND_STALE_NANOS) return 0.0
        val angle = latestAngleDegrees ?: return 0.0
        return if (abs(angle) <= ALIGNMENT_TOLERANCE_DEGREES) {
            TRACKING_FORWARD_SPEED_METERS_PER_SECOND
        } else {
            0.0
        }
    }

    internal companion object {
        const val ALIGNMENT_TOLERANCE_DEGREES = 10.0
        const val ALIGNMENT_YAW_RATE_DEGREES_PER_SECOND = 20.0
        const val TRACKING_FORWARD_SPEED_METERS_PER_SECOND = 0.3
        const val ENDPOINT_REFERENCE_MIN_FRACTION = 0.60
        const val ENDPOINT_LENGTH_RATIO = 0.45
        const val ENDPOINT_CONFIRMATION_COUNT = 3
        const val REACQUIRE_DETECTION_COUNT = 3
        const val RECENTER_DURATION_NANOS = 2_000_000_000L
        const val DETECTION_COMMAND_STALE_NANOS = 1_000_000_000L
        const val DETECTION_LOST_NANOS = 5_000_000_000L
        const val SEARCH_SWEEP_DURATION_NANOS = 3_000_000_000L
        const val SEARCH_TIMEOUT_NANOS = 20_000_000_000L
    }
}
