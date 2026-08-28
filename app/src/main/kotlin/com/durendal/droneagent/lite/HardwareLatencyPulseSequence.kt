package com.durendal.droneagent.lite

internal enum class HardwareLatencyPulsePhase {
    BASELINE,
    FORWARD,
    SETTLE_AFTER_FORWARD,
    BACKWARD,
    SETTLE_AFTER_BACKWARD,
    COMPLETE,
}

internal data class HardwareLatencyPulseStep(
    val phase: HardwareLatencyPulsePhase,
    val cycle: Int,
    val forwardMetersPerSecond: Double,
    val markerWhite: Boolean,
)

/**
 * Deterministic forward/backward steps for measuring command-to-motion latency.
 *
 * Delayed ticks never skip or shorten a phase: the next deadline starts at the
 * instant the command was actually applied. That preserves the neutral recovery
 * interval even if Android stalls, and paired directions keep net travel bounded.
 */
internal class HardwareLatencyPulseSequence(
    private val cycleCount: Int = DEFAULT_CYCLE_COUNT,
    private val speedMetersPerSecond: Double = DEFAULT_SPEED_METERS_PER_SECOND,
    private val baselineNanos: Long = DEFAULT_BASELINE_NANOS,
    private val pulseNanos: Long = DEFAULT_PULSE_NANOS,
    private val settleNanos: Long = DEFAULT_SETTLE_NANOS,
) {
    private var started = false
    private var currentPhase = HardwareLatencyPulsePhase.BASELINE
    private var currentCycle = 0
    private var nextTransitionAtNanos = 0L

    init {
        require(cycleCount > 0)
        require(speedMetersPerSecond.isFinite() && speedMetersPerSecond > 0.0)
        require(baselineNanos > 0L && pulseNanos > 0L && settleNanos > 0L)
    }

    fun start(nowNanos: Long): HardwareLatencyPulseStep {
        check(!started) { "hardware latency pulse sequence already started" }
        require(nowNanos >= 0L)
        started = true
        currentPhase = HardwareLatencyPulsePhase.BASELINE
        currentCycle = 0
        nextTransitionAtNanos = nowNanos + baselineNanos
        return currentStep()
    }

    fun advance(nowNanos: Long): HardwareLatencyPulseStep? {
        check(started) { "hardware latency pulse sequence has not started" }
        require(nowNanos >= 0L)
        if (currentPhase == HardwareLatencyPulsePhase.COMPLETE || nowNanos < nextTransitionAtNanos) {
            return null
        }

        currentPhase = when (currentPhase) {
            HardwareLatencyPulsePhase.BASELINE -> {
                currentCycle = 1
                HardwareLatencyPulsePhase.FORWARD
            }
            HardwareLatencyPulsePhase.FORWARD ->
                HardwareLatencyPulsePhase.SETTLE_AFTER_FORWARD
            HardwareLatencyPulsePhase.SETTLE_AFTER_FORWARD ->
                HardwareLatencyPulsePhase.BACKWARD
            HardwareLatencyPulsePhase.BACKWARD ->
                HardwareLatencyPulsePhase.SETTLE_AFTER_BACKWARD
            HardwareLatencyPulsePhase.SETTLE_AFTER_BACKWARD -> {
                if (currentCycle == cycleCount) {
                    HardwareLatencyPulsePhase.COMPLETE
                } else {
                    currentCycle += 1
                    HardwareLatencyPulsePhase.FORWARD
                }
            }
            HardwareLatencyPulsePhase.COMPLETE -> error("complete sequence cannot advance")
        }
        nextTransitionAtNanos = nowNanos + durationFor(currentPhase)
        return currentStep()
    }

    private fun currentStep(): HardwareLatencyPulseStep {
        val forwardMetersPerSecond = when (currentPhase) {
            HardwareLatencyPulsePhase.FORWARD -> speedMetersPerSecond
            HardwareLatencyPulsePhase.BACKWARD -> -speedMetersPerSecond
            else -> 0.0
        }
        return HardwareLatencyPulseStep(
            phase = currentPhase,
            cycle = currentCycle,
            forwardMetersPerSecond = forwardMetersPerSecond,
            markerWhite = forwardMetersPerSecond != 0.0,
        )
    }

    private fun durationFor(phase: HardwareLatencyPulsePhase): Long = when (phase) {
        HardwareLatencyPulsePhase.BASELINE -> baselineNanos
        HardwareLatencyPulsePhase.FORWARD,
        HardwareLatencyPulsePhase.BACKWARD,
        -> pulseNanos
        HardwareLatencyPulsePhase.SETTLE_AFTER_FORWARD,
        HardwareLatencyPulsePhase.SETTLE_AFTER_BACKWARD,
        -> settleNanos
        HardwareLatencyPulsePhase.COMPLETE -> 0L
    }

    companion object {
        const val DEFAULT_CYCLE_COUNT = 10
        const val DEFAULT_SPEED_METERS_PER_SECOND = 0.5
        const val DEFAULT_BASELINE_NANOS = 3_000_000_000L
        const val DEFAULT_PULSE_NANOS = 1_000_000_000L
        const val DEFAULT_SETTLE_NANOS = 2_000_000_000L
    }
}
