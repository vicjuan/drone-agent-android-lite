package com.durendal.droneagent.lite

internal enum class StraightLineDirection(
    val displayName: String,
    val forwardSign: Double,
) {
    FORWARD("前", 1.0),
    BACKWARD("後", -1.0),
}

internal enum class StraightLineSpeedPhase {
    BASELINE,
    CRUISE,
    BRAKING,
    COMPLETE,
}

internal enum class StraightLineStopReason {
    OPERATOR,
    CANCELLED_BEFORE_CRUISE,
    TIME_LIMIT,
    CONTROL_STALL,
}

internal data class StraightLineSpeedStep(
    val phase: StraightLineSpeedPhase,
    val direction: StraightLineDirection,
    override val forwardMetersPerSecond: Double,
    override val rightMetersPerSecond: Double,
) : HorizontalPulseStep {
    override val phaseName: String
        get() = phase.name
    override val markerWhite: Boolean
        get() = phase == StraightLineSpeedPhase.CRUISE
    override val complete: Boolean
        get() = phase == StraightLineSpeedPhase.COMPLETE
}

/**
 * One manually stopped, bounded straight leg. There is no automatic return leg.
 * BRAKING is a real neutral-command interval, not evidence that the aircraft stopped.
 */
internal class StraightLineSpeedSequence(
    private val direction: StraightLineDirection,
    private val speedMetersPerSecond: Double,
    private val maximumCruiseNanos: Long,
    private val baselineNanos: Long = 2_000_000_000L,
    private val brakingNanos: Long = 3_000_000_000L,
    private val maximumTickGapNanos: Long = 500_000_000L,
) : HorizontalPulseSequence {
    var phase = StraightLineSpeedPhase.BASELINE
        private set
    var stopReason: StraightLineStopReason? = null
        private set
    val commandDeadlineNanos: Long
        get() = if (phase == StraightLineSpeedPhase.CRUISE) nextTransitionAtNanos else 0L

    private var started = false
    private var lastTickAtNanos = 0L
    private var nextTransitionAtNanos = 0L

    init {
        require(speedMetersPerSecond.isFinite() && speedMetersPerSecond > 0.0 && speedMetersPerSecond <= 0.5)
        require(maximumCruiseNanos > 0L && baselineNanos >= 0L && brakingNanos > 0L && maximumTickGapNanos > 0L)
    }

    override fun start(nowNanos: Long): StraightLineSpeedStep {
        check(!started) { "straight line speed sequence already started" }
        phase = if (baselineNanos == 0L) StraightLineSpeedPhase.CRUISE else StraightLineSpeedPhase.BASELINE
        nextTransitionAtNanos = deadline(nowNanos, if (baselineNanos == 0L) maximumCruiseNanos else baselineNanos)
        started = true
        lastTickAtNanos = nowNanos
        return currentStep()
    }

    override fun advance(nowNanos: Long): StraightLineSpeedStep? {
        validateTick(nowNanos)
        val automaticStop = automaticStopReason(nowNanos)
        if (automaticStop != null) return beginBraking(nowNanos, automaticStop)
        lastTickAtNanos = nowNanos
        if (phase == StraightLineSpeedPhase.COMPLETE || nowNanos < nextTransitionAtNanos) return null

        when (phase) {
            StraightLineSpeedPhase.BASELINE -> {
                // The absolute cruise limit starts when the nonzero command is emitted.
                nextTransitionAtNanos = deadline(nowNanos, maximumCruiseNanos)
                phase = StraightLineSpeedPhase.CRUISE
            }
            StraightLineSpeedPhase.BRAKING -> phase = StraightLineSpeedPhase.COMPLETE
            StraightLineSpeedPhase.CRUISE -> error("cruise deadline must enter braking")
            StraightLineSpeedPhase.COMPLETE -> error("complete sequence cannot advance")
        }
        return currentStep()
    }

    fun requestStop(nowNanos: Long): StraightLineSpeedStep? {
        validateTick(nowNanos)
        if (phase == StraightLineSpeedPhase.BRAKING || phase == StraightLineSpeedPhase.COMPLETE) {
            lastTickAtNanos = nowNanos
            return null
        }
        // A late operator action cannot turn a timed-out or stalled run into a normal stop.
        val reason = automaticStopReason(nowNanos) ?: if (phase == StraightLineSpeedPhase.BASELINE) {
            StraightLineStopReason.CANCELLED_BEFORE_CRUISE
        } else {
            StraightLineStopReason.OPERATOR
        }
        return beginBraking(nowNanos, reason)
    }

    private fun automaticStopReason(nowNanos: Long): StraightLineStopReason? {
        if (phase != StraightLineSpeedPhase.BASELINE && phase != StraightLineSpeedPhase.CRUISE) return null
        return when {
            phase == StraightLineSpeedPhase.CRUISE && nowNanos >= nextTransitionAtNanos ->
                StraightLineStopReason.TIME_LIMIT
            nowNanos - lastTickAtNanos >= maximumTickGapNanos -> StraightLineStopReason.CONTROL_STALL
            else -> null
        }
    }

    private fun beginBraking(nowNanos: Long, reason: StraightLineStopReason): StraightLineSpeedStep {
        // Even a very late tick must emit neutral before starting the full braking window.
        nextTransitionAtNanos = deadline(nowNanos, brakingNanos)
        lastTickAtNanos = nowNanos
        stopReason = reason
        phase = StraightLineSpeedPhase.BRAKING
        return currentStep()
    }

    private fun validateTick(nowNanos: Long) {
        check(started) { "straight line speed sequence has not started" }
        require(nowNanos >= lastTickAtNanos) { "sequence time must be monotonic" }
    }

    private fun deadline(nowNanos: Long, durationNanos: Long): Long {
        require(nowNanos >= 0L && nowNanos <= Long.MAX_VALUE - durationNanos) { "invalid sequence time" }
        return nowNanos + durationNanos
    }

    private fun currentStep(): StraightLineSpeedStep = StraightLineSpeedStep(
        phase = phase,
        direction = direction,
        forwardMetersPerSecond =
            if (phase == StraightLineSpeedPhase.CRUISE) speedMetersPerSecond * direction.forwardSign else 0.0,
        rightMetersPerSecond = 0.0,
    )
}
