package com.durendal.droneagent.lite

internal enum class DirectionalVelocityPulseDirection(
    val displayName: String,
    val forwardSign: Double,
    val rightSign: Double,
) {
    FORWARD("前", 1.0, 0.0),
    RIGHT("右", 0.0, 1.0),
    BACKWARD("後", -1.0, 0.0),
    LEFT("左", 0.0, -1.0),
    ;

    fun next(): DirectionalVelocityPulseDirection =
        entries[(ordinal + 1) % entries.size]
}

internal enum class DirectionalVelocityPulsePhase {
    BASELINE,
    PULSE,
    SETTLE,
    COMPLETE,
}

internal data class DirectionalVelocityPulseStep(
    val phase: DirectionalVelocityPulsePhase,
    val direction: DirectionalVelocityPulseDirection,
    override val forwardMetersPerSecond: Double,
    override val rightMetersPerSecond: Double,
) : HorizontalPulseStep {
    override val phaseName: String
        get() = phase.name
    override val markerWhite: Boolean
        get() = phase == DirectionalVelocityPulsePhase.PULSE
    override val complete: Boolean
        get() = phase == DirectionalVelocityPulsePhase.COMPLETE
}

/** One bounded pulse. The operator recentres manually before starting the next direction. */
internal class DirectionalVelocityPulseSequence(
    private val direction: DirectionalVelocityPulseDirection,
    private val speedMetersPerSecond: Double = DEFAULT_SPEED_METERS_PER_SECOND,
    private val baselineNanos: Long = DEFAULT_BASELINE_NANOS,
    private val pulseNanos: Long = DEFAULT_PULSE_NANOS,
    private val settleNanos: Long = DEFAULT_SETTLE_NANOS,
) : HorizontalPulseSequence {
    private var started = false
    private var phase = DirectionalVelocityPulsePhase.BASELINE
    private var nextTransitionAtNanos = 0L

    init {
        require(speedMetersPerSecond.isFinite() && speedMetersPerSecond > 0.0)
        require(baselineNanos > 0L && pulseNanos > 0L && settleNanos > 0L)
    }

    override fun start(nowNanos: Long): DirectionalVelocityPulseStep {
        check(!started) { "directional velocity pulse sequence already started" }
        require(nowNanos >= 0L)
        started = true
        phase = DirectionalVelocityPulsePhase.BASELINE
        nextTransitionAtNanos = nowNanos + baselineNanos
        return currentStep()
    }

    override fun advance(nowNanos: Long): DirectionalVelocityPulseStep? {
        check(started) { "directional velocity pulse sequence has not started" }
        require(nowNanos >= 0L)
        if (phase == DirectionalVelocityPulsePhase.COMPLETE || nowNanos < nextTransitionAtNanos) {
            return null
        }
        phase = when (phase) {
            DirectionalVelocityPulsePhase.BASELINE -> DirectionalVelocityPulsePhase.PULSE
            DirectionalVelocityPulsePhase.PULSE -> DirectionalVelocityPulsePhase.SETTLE
            DirectionalVelocityPulsePhase.SETTLE -> DirectionalVelocityPulsePhase.COMPLETE
            DirectionalVelocityPulsePhase.COMPLETE -> error("complete sequence cannot advance")
        }
        nextTransitionAtNanos = nowNanos + durationFor(phase)
        return currentStep()
    }

    private fun currentStep(): DirectionalVelocityPulseStep {
        val speed = if (phase == DirectionalVelocityPulsePhase.PULSE) speedMetersPerSecond else 0.0
        return DirectionalVelocityPulseStep(
            phase = phase,
            direction = direction,
            forwardMetersPerSecond = speed * direction.forwardSign,
            rightMetersPerSecond = speed * direction.rightSign,
        )
    }

    private fun durationFor(value: DirectionalVelocityPulsePhase): Long = when (value) {
        DirectionalVelocityPulsePhase.BASELINE -> baselineNanos
        DirectionalVelocityPulsePhase.PULSE -> pulseNanos
        DirectionalVelocityPulsePhase.SETTLE -> settleNanos
        DirectionalVelocityPulsePhase.COMPLETE -> 0L
    }

    companion object {
        const val DEFAULT_SPEED_METERS_PER_SECOND = 0.8
        const val DEFAULT_BASELINE_NANOS = 2_000_000_000L
        const val DEFAULT_PULSE_NANOS = 500_000_000L
        const val DEFAULT_SETTLE_NANOS = 2_000_000_000L
    }
}

internal enum class FixedDirectionSpeedPhase {
    BASELINE,
    RAMP_UP,
    HOLD,
    RAMP_DOWN,
    SETTLE,
    COMPLETE,
}

internal data class FixedDirectionSpeedStep(
    val phase: FixedDirectionSpeedPhase,
    val direction: DirectionalVelocityPulseDirection,
    override val forwardMetersPerSecond: Double,
    override val rightMetersPerSecond: Double,
) : HorizontalPulseStep {
    override val phaseName: String
        get() = phase.name
    override val markerWhite: Boolean
        get() = phase == FixedDirectionSpeedPhase.RAMP_UP ||
            phase == FixedDirectionSpeedPhase.HOLD ||
            phase == FixedDirectionSpeedPhase.RAMP_DOWN
    override val complete: Boolean
        get() = phase == FixedDirectionSpeedPhase.COMPLETE
}

/**
 * Fixed-direction response test with linear ramps around a constant-speed hold.
 *
 * Ramp steps are emitted on every experiment tick so the virtual-stick sender
 * receives a continuously changing setpoint instead of one velocity step.
 */
internal class FixedDirectionSpeedSequence(
    private val direction: DirectionalVelocityPulseDirection,
    private val speedMetersPerSecond: Double,
    private val baselineNanos: Long,
    private val rampUpNanos: Long,
    private val holdNanos: Long,
    private val rampDownNanos: Long,
    private val settleNanos: Long,
) : HorizontalPulseSequence {
    private var started = false
    private var phase = FixedDirectionSpeedPhase.BASELINE
    private var phaseStartedAtNanos = 0L
    private var nextTransitionAtNanos = 0L

    init {
        require(speedMetersPerSecond.isFinite() && speedMetersPerSecond > 0.0)
        require(
            baselineNanos > 0L &&
                rampUpNanos > 0L &&
                holdNanos > 0L &&
                rampDownNanos > 0L &&
                settleNanos > 0L,
        )
    }

    override fun start(nowNanos: Long): FixedDirectionSpeedStep {
        check(!started) { "fixed direction speed sequence already started" }
        require(nowNanos >= 0L)
        started = true
        phase = FixedDirectionSpeedPhase.BASELINE
        phaseStartedAtNanos = nowNanos
        nextTransitionAtNanos = nowNanos + baselineNanos
        return currentStep(nowNanos)
    }

    override fun advance(nowNanos: Long): FixedDirectionSpeedStep? {
        check(started) { "fixed direction speed sequence has not started" }
        require(nowNanos >= 0L)
        if (phase == FixedDirectionSpeedPhase.COMPLETE) return null
        if (nowNanos < nextTransitionAtNanos) {
            return if (
                phase == FixedDirectionSpeedPhase.RAMP_UP ||
                phase == FixedDirectionSpeedPhase.RAMP_DOWN
            ) {
                currentStep(nowNanos)
            } else {
                null
            }
        }

        phase = when (phase) {
            FixedDirectionSpeedPhase.BASELINE -> FixedDirectionSpeedPhase.RAMP_UP
            FixedDirectionSpeedPhase.RAMP_UP -> FixedDirectionSpeedPhase.HOLD
            FixedDirectionSpeedPhase.HOLD -> FixedDirectionSpeedPhase.RAMP_DOWN
            FixedDirectionSpeedPhase.RAMP_DOWN -> FixedDirectionSpeedPhase.SETTLE
            FixedDirectionSpeedPhase.SETTLE -> FixedDirectionSpeedPhase.COMPLETE
            FixedDirectionSpeedPhase.COMPLETE -> error("complete sequence cannot advance")
        }
        phaseStartedAtNanos = nowNanos
        nextTransitionAtNanos = nowNanos + durationFor(phase)
        return currentStep(nowNanos)
    }

    private fun currentStep(nowNanos: Long): FixedDirectionSpeedStep {
        val speed = when (phase) {
            FixedDirectionSpeedPhase.RAMP_UP ->
                speedMetersPerSecond * phaseProgress(nowNanos, rampUpNanos)
            FixedDirectionSpeedPhase.HOLD -> speedMetersPerSecond
            FixedDirectionSpeedPhase.RAMP_DOWN ->
                speedMetersPerSecond * (1.0 - phaseProgress(nowNanos, rampDownNanos))
            FixedDirectionSpeedPhase.BASELINE,
            FixedDirectionSpeedPhase.SETTLE,
            FixedDirectionSpeedPhase.COMPLETE,
            -> 0.0
        }
        return FixedDirectionSpeedStep(
            phase = phase,
            direction = direction,
            forwardMetersPerSecond = speed * direction.forwardSign,
            rightMetersPerSecond = speed * direction.rightSign,
        )
    }

    private fun phaseProgress(nowNanos: Long, durationNanos: Long): Double =
        ((nowNanos - phaseStartedAtNanos).toDouble() / durationNanos)
            .coerceIn(0.0, 1.0)

    private fun durationFor(value: FixedDirectionSpeedPhase): Long = when (value) {
        FixedDirectionSpeedPhase.BASELINE -> baselineNanos
        FixedDirectionSpeedPhase.RAMP_UP -> rampUpNanos
        FixedDirectionSpeedPhase.HOLD -> holdNanos
        FixedDirectionSpeedPhase.RAMP_DOWN -> rampDownNanos
        FixedDirectionSpeedPhase.SETTLE -> settleNanos
        FixedDirectionSpeedPhase.COMPLETE -> 0L
    }
}
