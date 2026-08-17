package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.sign

internal enum class TapeTrackingPhase {
    DISABLED,
    RECENTERING,
    TRACKING,
    VERIFYING_ENDPOINT,
    TURNING,
}

internal enum class TrackingGimbalTarget {
    DOWN_CENTER,
}

internal data class TapeTrackingDecision(
    val phase: TapeTrackingPhase,
    val yawRateDegreesPerSecond: Double,
    val forwardSpeedMetersPerSecond: Double = 0.0,
    val rightSpeedMetersPerSecond: Double = 0.0,
    val gimbalTarget: TrackingGimbalTarget? = null,
    val endpointReached: Boolean = false,
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
    private var latestHorizontalOffsetFraction: Double? = null
    private var recenterUntilNanos = 0L
    private var consecutiveEndpointDetections = 0
    private var endpointCandidateSinceNanos = 0L
    private var longestObservedTapeFraction = 0.0
    private var endpointPending = false
    private var pendingGimbalTarget: TrackingGimbalTarget? = null
    private var awaitingPostTurnDetection = false
    private var postTurnRecoveryUntilNanos = 0L
    private var endpointVerificationStartedAtNanos = 0L
    private var consecutiveEndpointMisses = 0

    fun start(nowNanos: Long) {
        enabled = true
        resetLeg()
        awaitingPostTurnDetection = false
        beginRecentering(nowNanos)
    }

    fun stop() {
        enabled = false
        phase = TapeTrackingPhase.DISABLED
        latestAngleDegrees = null
        consecutiveEndpointDetections = 0
        endpointCandidateSinceNanos = 0L
        endpointPending = false
        awaitingPostTurnDetection = false
        postTurnRecoveryUntilNanos = 0L
        endpointVerificationStartedAtNanos = 0L
        consecutiveEndpointMisses = 0
        pendingGimbalTarget = null
    }

    fun resumeAfterTurn(nowNanos: Long) {
        check(enabled && phase == TapeTrackingPhase.TURNING)
        resetLeg()
        awaitingPostTurnDetection = true
        postTurnRecoveryUntilNanos =
            nowNanos + RECENTER_DURATION_NANOS + POST_TURN_RECOVERY_NANOS
        beginRecentering(nowNanos)
    }

    fun observe(
        angleFromVerticalDegrees: Double?,
        longSideFraction: Double?,
        nowNanos: Long,
        horizontalOffsetFraction: Double? = 0.0,
    ) {
        if (!enabled || phase == TapeTrackingPhase.TURNING) return
        if (
            angleFromVerticalDegrees == null ||
            horizontalOffsetFraction == null ||
            longSideFraction == null
        ) {
            latestAngleDegrees = null
            latestHorizontalOffsetFraction = null
            if (phase == TapeTrackingPhase.VERIFYING_ENDPOINT) {
                consecutiveEndpointMisses += 1
                if (consecutiveEndpointMisses >= ENDPOINT_DISAPPEARANCE_COUNT) {
                    phase = TapeTrackingPhase.TURNING
                    endpointPending = true
                }
            }
            return
        }
        require(angleFromVerticalDegrees in -90.0..90.0)
        require(horizontalOffsetFraction in -0.5..0.5)
        require(longSideFraction > 0.0 && longSideFraction.isFinite())
        when (phase) {
            TapeTrackingPhase.TRACKING -> {
                latestAngleDegrees = angleFromVerticalDegrees
                latestHorizontalOffsetFraction = horizontalOffsetFraction
                lastDetectionAtNanos = nowNanos
                awaitingPostTurnDetection = false
                longestObservedTapeFraction =
                    maxOf(longestObservedTapeFraction, longSideFraction)
                val endpointCandidate =
                    abs(angleFromVerticalDegrees) <= ALIGNMENT_TOLERANCE_DEGREES &&
                        longestObservedTapeFraction >= ENDPOINT_REFERENCE_MIN_FRACTION &&
                        longSideFraction <= longestObservedTapeFraction * ENDPOINT_LENGTH_RATIO
                if (endpointCandidate) {
                    if (endpointCandidateSinceNanos == 0L) {
                        endpointCandidateSinceNanos = nowNanos
                    }
                    consecutiveEndpointDetections += 1
                } else {
                    endpointCandidateSinceNanos = 0L
                    consecutiveEndpointDetections = 0
                }
                beginEndpointVerificationIfReady(nowNanos)
            }

            TapeTrackingPhase.VERIFYING_ENDPOINT -> {
                consecutiveEndpointMisses = 0
                if (longSideFraction >= longestObservedTapeFraction * ENDPOINT_EXIT_LENGTH_RATIO) {
                    phase = TapeTrackingPhase.TRACKING
                    endpointVerificationStartedAtNanos = 0L
                    endpointCandidateSinceNanos = 0L
                    consecutiveEndpointDetections = 0
                    latestAngleDegrees = angleFromVerticalDegrees
                    latestHorizontalOffsetFraction = horizontalOffsetFraction
                    lastDetectionAtNanos = nowNanos
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
        beginEndpointVerificationIfReady(nowNanos)
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

        val target = pendingGimbalTarget
        pendingGimbalTarget = null
        return TapeTrackingDecision(
            phase = phase,
            yawRateDegreesPerSecond = desiredYawRate(nowNanos),
            forwardSpeedMetersPerSecond = desiredForwardSpeed(nowNanos),
            rightSpeedMetersPerSecond = desiredRightSpeed(nowNanos),
            gimbalTarget = target,
        )
    }
    private fun beginEndpointVerificationIfReady(nowNanos: Long) {
        if (
            phase != TapeTrackingPhase.TRACKING ||
            endpointCandidateSinceNanos == 0L ||
            consecutiveEndpointDetections < ENDPOINT_CONFIRMATION_COUNT ||
            nowNanos - endpointCandidateSinceNanos < ENDPOINT_CONFIRMATION_NANOS
        ) {
            return
        }
        phase = TapeTrackingPhase.VERIFYING_ENDPOINT
        latestAngleDegrees = null
        latestHorizontalOffsetFraction = null
        endpointCandidateSinceNanos = 0L
        consecutiveEndpointDetections = 0
        endpointVerificationStartedAtNanos = nowNanos
        consecutiveEndpointMisses = 0
    }


    private fun beginRecentering(nowNanos: Long) {
        phase = TapeTrackingPhase.RECENTERING
        latestAngleDegrees = null
        latestHorizontalOffsetFraction = null
        pendingGimbalTarget = TrackingGimbalTarget.DOWN_CENTER
        recenterUntilNanos = nowNanos + RECENTER_DURATION_NANOS
    }

    private fun resetLeg() {
        lastDetectionAtNanos = 0L
        consecutiveEndpointDetections = 0
        endpointCandidateSinceNanos = 0L
        longestObservedTapeFraction = 0.0
        endpointPending = false
        endpointVerificationStartedAtNanos = 0L
        consecutiveEndpointMisses = 0
    }

    private fun desiredYawRate(nowNanos: Long): Double {
        if (phase != TapeTrackingPhase.TRACKING) return 0.0
        if (nowNanos - lastDetectionAtNanos > DETECTION_COMMAND_STALE_NANOS) return 0.0
        val angle = latestAngleDegrees ?: return 0.0
        if (abs(angle) <= ALIGNMENT_TOLERANCE_DEGREES) return 0.0
        return sign(angle) * ALIGNMENT_YAW_RATE_DEGREES_PER_SECOND
    }

    private fun desiredRightSpeed(nowNanos: Long): Double {
        if (phase != TapeTrackingPhase.TRACKING || consecutiveEndpointDetections > 0) return 0.0
        if (nowNanos - lastDetectionAtNanos > DETECTION_COMMAND_STALE_NANOS) return 0.0
        val angle = latestAngleDegrees ?: return 0.0
        if (abs(angle) > ALIGNMENT_TOLERANCE_DEGREES) return 0.0
        val offset = latestHorizontalOffsetFraction ?: return 0.0
        if (abs(offset) <= CENTERING_DEAD_ZONE_FRACTION) return 0.0
        return (offset * CENTERING_SPEED_GAIN)
            .coerceIn(-MAX_CENTERING_SPEED_METERS_PER_SECOND, MAX_CENTERING_SPEED_METERS_PER_SECOND)
    }

    private fun desiredForwardSpeed(nowNanos: Long): Double {
        if (phase == TapeTrackingPhase.VERIFYING_ENDPOINT) {
            return if (
                nowNanos - endpointVerificationStartedAtNanos <= ENDPOINT_PROBE_DURATION_NANOS
            ) {
                ENDPOINT_PROBE_SPEED_METERS_PER_SECOND
            } else {
                0.0
            }
        }
        if (awaitingPostTurnDetection && phase == TapeTrackingPhase.TRACKING) {
            return if (nowNanos <= postTurnRecoveryUntilNanos) {
                POST_TURN_RECOVERY_SPEED_METERS_PER_SECOND
            } else {
                0.0
            }
        }
        if (phase != TapeTrackingPhase.TRACKING || consecutiveEndpointDetections > 0) return 0.0
        if (nowNanos - lastDetectionAtNanos > DETECTION_COMMAND_STALE_NANOS) return 0.0
        val angle = latestAngleDegrees ?: return 0.0
        val offset = latestHorizontalOffsetFraction ?: return 0.0
        return if (
            abs(angle) <= ALIGNMENT_TOLERANCE_DEGREES &&
            abs(offset) <= CENTERING_DEAD_ZONE_FRACTION
        ) {
            TRACKING_FORWARD_SPEED_METERS_PER_SECOND
        } else {
            0.0
        }
    }

    internal companion object {
        const val ALIGNMENT_TOLERANCE_DEGREES = 10.0
        const val ALIGNMENT_YAW_RATE_DEGREES_PER_SECOND = 5.0
        const val TRACKING_FORWARD_SPEED_METERS_PER_SECOND = 0.3
        const val CENTERING_DEAD_ZONE_FRACTION = 0.08
        const val CENTERING_SPEED_GAIN = 0.4
        const val MAX_CENTERING_SPEED_METERS_PER_SECOND = 0.15
        const val ENDPOINT_REFERENCE_MIN_FRACTION = 0.60
        const val ENDPOINT_LENGTH_RATIO = 0.55
        const val ENDPOINT_EXIT_LENGTH_RATIO = 0.65
        const val ENDPOINT_CONFIRMATION_COUNT = 3
        const val ENDPOINT_CONFIRMATION_NANOS = 750_000_000L
        const val ENDPOINT_DISAPPEARANCE_COUNT = 3
        const val ENDPOINT_PROBE_SPEED_METERS_PER_SECOND = 0.10
        const val ENDPOINT_PROBE_DURATION_NANOS = 1_500_000_000L
        const val RECENTER_DURATION_NANOS = 2_000_000_000L
        const val POST_TURN_RECOVERY_SPEED_METERS_PER_SECOND = 0.15
        const val POST_TURN_RECOVERY_NANOS = 2_000_000_000L
        const val DETECTION_COMMAND_STALE_NANOS = 1_000_000_000L
    }
}
