package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.roundToLong
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * State is [centreNorth - aircraftNorth, centreEast - aircraftEast, velocityNorth, velocityEast].
 * North/east and forward/right are horizontal ground axes; heading is clockwise from north.
 * Velocity is metric visual odometry rotated into north/east by the caller. No gain occurs here.
 *
 * The acceleration parameter is the standard deviation of acceleration averaged over a provisional
 * one-second reference interval. White-acceleration PSD is q = sigma^2 * 1 second (m^2/s^3).
 * Integrated Q has negative position/velocity cross terms because relative centre velocity is
 * -aircraft velocity. Propagation composes across arbitrary event boundaries, including rejected
 * measurements and repeated prediction queries.
 *
 * Valid-time vision frames, including gated frames, are retained so an earlier arriving measurement
 * can change a later gate consistently. Counters count unique frames, not replay executions; a
 * later replay may transfer a frame between accepted/rejected counts. Timestamp-invalid receipts
 * and duplicate receipts are not new admitted frames. Overflow latches closed until explicit reset.
 */
internal class CircleStateEstimator(private val config: CircleModelConfig = CircleModelConfig()) {
    private class State {
        val x = DoubleArray(4)
        val p = DoubleArray(16)
        var atNanos = 0L
        var lastVisionAtNanos = -1L
        var lastVelocityAtNanos = -1L
        var lastInnovationSquared: Double? = null

        fun clear(atNanos: Long) {
            x.fill(0.0)
            p.fill(0.0)
            // Explicit unobserved prior, NOT a zero-velocity sensor fallback. No estimate is
            // publishable until a real velocity measurement has been accepted.
            p[10] = UNOBSERVED_VELOCITY_VARIANCE
            p[15] = UNOBSERVED_VELOCITY_VARIANCE
            this.atNanos = atNanos
            lastVisionAtNanos = -1L
            lastVelocityAtNanos = -1L
            lastInnovationSquared = null
        }

        fun copyFrom(other: State) {
            other.x.copyInto(x)
            other.p.copyInto(p)
            atNanos = other.atNanos
            lastVisionAtNanos = other.lastVisionAtNanos
            lastVelocityAtNanos = other.lastVelocityAtNanos
            lastInnovationSquared = other.lastInnovationSquared
        }
    }

    private sealed class Event(val atNanos: Long, val order: Int) {
        var accepted: Boolean? = null
    }

    private class VelocityEvent(atNanos: Long, val north: Double, val east: Double) : Event(atNanos, 0)

    private class VisionEvent(
        atNanos: Long,
        val north: Double,
        val east: Double,
        val varianceNorth: Double,
        val varianceEast: Double,
        val covariance: Double,
        val range: Double,
        val bearing: Double,
        val rangeVariance: Double,
        val bearingVariance: Double,
        val rangeBearingCovariance: Double,
    ) : Event(atNanos, 1)

    private val checkpoint = State()
    private val committed = State()
    private val replay = State()
    private val prediction = State()
    private val events = ArrayList<Event>(config.historyCapacity)
    private val seenVisionTimes = ArrayList<Long>(config.historyCapacity)
    private var startedAtNanos = -1L
    private var checkpointSealed = false
    private var newestVelocityReceiptAtNanos = -1L
    private var faultReason: String? = null
    private var acceptedVisionCount = 0
    private var rejectedVisionCount = 0
    private var acceptedVelocityCount = 0
    private var lateReplayCount = 0

    // Fixed-size workspace shared by propagation and Joseph updates; never allocated per scalar.
    private val h = DoubleArray(8)
    private val pHt = DoubleArray(8)
    private val gain = DoubleArray(8)
    private val transform = DoubleArray(16)
    private val product = DoubleArray(16)
    private val candidateP = DoubleArray(16)
    private val candidateX = DoubleArray(4)
    private val cholesky = DoubleArray(16)
    private var innovationSquared: Double? = null
    private val accelerationVariance = config.accelerationNoiseMetersPerSecondSquared * config.accelerationNoiseMetersPerSecondSquared
    private val velocityVariance = config.velocityMeasurementStdMetersPerSecond * config.velocityMeasurementStdMetersPerSecond

    init {
        require(accelerationVariance.isFinite() && accelerationVariance > 0.0)
        require(velocityVariance.isFinite() && velocityVariance > 0.0)
    }

    fun reset(startedAtNanos: Long) {
        require(startedAtNanos >= 0L)
        this.startedAtNanos = startedAtNanos
        checkpoint.clear(startedAtNanos)
        committed.copyFrom(checkpoint)
        events.clear()
        seenVisionTimes.clear()
        checkpointSealed = false
        newestVelocityReceiptAtNanos = -1L
        faultReason = null
        acceptedVisionCount = 0
        rejectedVisionCount = 0
        acceptedVelocityCount = 0
        lateReplayCount = 0
    }

    fun updateVelocity(
        northMetersPerSecond: Double,
        eastMetersPerSecond: Double,
        sampleAtNanos: Long,
        nowNanos: Long,
    ): Boolean {
        if (!northMetersPerSecond.isFinite() || !eastMetersPerSecond.isFinite()) return false
        if (!validSampleTime(sampleAtNanos, nowNanos, config.maximumVelocityAgeNanos)) return false
        if (sampleAtNanos <= newestVelocityReceiptAtNanos) return false
        if (!trimHistory(nowNanos) || !withinCheckpoint(sampleAtNanos)) return false
        newestVelocityReceiptAtNanos = sampleAtNanos
        return insert(VelocityEvent(sampleAtNanos, northMetersPerSecond, eastMetersPerSecond))
    }

    fun observe(observation: CircleArcObservation, nowNanos: Long): Boolean {
        val sample = observation.sampleAtNanos
        if (!validSampleTime(sample, nowNanos, config.maximumVisionAgeNanos)) return false
        if (!trimHistory(nowNanos) || !withinCheckpoint(sample)) return false
        if (sample in seenVisionTimes) return false
        if (seenVisionTimes.size >= config.historyCapacity) return failClosed("history_overflow")
        seenVisionTimes.add(sample)
        val event = visionEvent(observation)
        if (event == null) {
            rejectedVisionCount += 1
            return false
        }
        return insert(event)
    }

    /** Acquisition must use current replay outcomes, not earlier callback results. */
    fun hasConfirmedVisionSince(barrierAtNanos: Long, minimumFrames: Int, minimumSpanNanos: Long): Boolean {
        var count = 0
        var firstAt = 0L
        var lastAt = 0L
        for (index in events.indices) {
            val event = events[index]
            if (event !is VisionEvent || event.accepted != true || event.atNanos <= barrierAtNanos) continue
            if (count == 0 || event.atNanos - lastAt > config.maximumVisionAgeNanos) {
                count = 0
                firstAt = event.atNanos
            }
            count += 1
            lastAt = event.atNanos
        }
        return count >= minimumFrames && lastAt - firstAt >= minimumSpanNanos
    }

    fun estimate(
        nowNanos: Long,
        headingDegrees: Double,
        headingAtNanos: Long,
        predictionSeconds: Double = config.commandPredictionSeconds,
    ): CircleEstimateResult {
        faultReason?.let { return result(reason = it) }
        if (startedAtNanos < 0L) return result(reason = "not_started")
        if (nowNanos < startedAtNanos || nowNanos < committed.atNanos) return result(reason = "invalid_query_time")
        if (!predictionSeconds.isFinite() || predictionSeconds !in 0.0..config.maximumPredictionSeconds) {
            return result(reason = "invalid_prediction_horizon")
        }
        if (!headingDegrees.isFinite() || headingAtNanos < 0L || headingAtNanos > nowNanos) {
            return result(reason = "invalid_heading")
        }
        if (nowNanos - headingAtNanos > config.maximumHeadingAgeNanos) return result(reason = "heading_expired")
        if (committed.lastVisionAtNanos < 0L) return result(reason = "vision_unavailable")
        if (committed.lastVelocityAtNanos < 0L) return result(reason = "velocity_unavailable")
        if (nowNanos - committed.lastVisionAtNanos > config.maximumVisionAgeNanos) return result(reason = "vision_expired")
        if (nowNanos - committed.lastVelocityAtNanos > config.maximumVelocityAgeNanos) return result(reason = "velocity_expired")
        val horizonNanos = (predictionSeconds * NANOS_PER_SECOND).roundToLong()
        if (nowNanos > Long.MAX_VALUE - horizonNanos) return result(reason = "invalid_prediction_time")
        val estimateAtNanos = nowNanos + horizonNanos
        prediction.copyFrom(committed)
        if (!propagate(prediction, estimateAtNanos)) return result(reason = "invalid_covariance")
        val positionStd = maximumStd(prediction.p[0], prediction.p[5], prediction.p[1])
        val velocityStd = maximumStd(prediction.p[10], prediction.p[15], prediction.p[11])
        if (!positionStd.isFinite() || !velocityStd.isFinite()) return result(reason = "invalid_covariance")
        if (positionStd > config.maximumPositionStdMeters) return result(reason = "position_uncertainty")
        if (velocityStd > config.maximumVelocityStdMetersPerSecond) return result(reason = "velocity_uncertainty")
        val heading = Math.toRadians(headingDegrees % 360.0)
        val c = cos(heading)
        val s = sin(heading)
        return result(
            estimate = CircleStateEstimate(
                centerForwardMeters = c * prediction.x[0] + s * prediction.x[1],
                centerRightMeters = -s * prediction.x[0] + c * prediction.x[1],
                forwardVelocityMetersPerSecond = c * prediction.x[2] + s * prediction.x[3],
                rightVelocityMetersPerSecond = -s * prediction.x[2] + c * prediction.x[3],
                positionStdMeters = positionStd,
                velocityStdMetersPerSecond = velocityStd,
                estimateAtNanos = estimateAtNanos,
                lastVisionAtNanos = committed.lastVisionAtNanos,
                lastVelocityAtNanos = committed.lastVelocityAtNanos,
                predictionSeconds = predictionSeconds,
            ),
        )
    }

    private fun validSampleTime(sample: Long, now: Long, maximumAge: Long): Boolean =
        faultReason == null && startedAtNanos >= 0L && now >= committed.atNanos &&
            sample >= startedAtNanos && sample <= now && now - sample <= maximumAge

    private fun withinCheckpoint(sample: Long): Boolean =
        sample >= checkpoint.atNanos && (!checkpointSealed || sample > checkpoint.atNanos)

    private fun visionEvent(observation: CircleArcObservation): VisionEvent? {
        val f = observation.centerForwardMeters
        val r = observation.centerRightMeters
        val vf = observation.centerForwardVariance
        val vr = observation.centerRightVariance
        val cov = observation.centerCovariance
        if (!f.isFinite() || !r.isFinite() || !positiveDefinite2(vf, vr, cov)) return null
        if (!observation.headingDegrees.isFinite() || observation.headingAtNanos < 0L ||
            observation.headingAtNanos > observation.sampleAtNanos ||
            observation.sampleAtNanos - observation.headingAtNanos > config.maximumHeadingAgeNanos
        ) return null
        if (!observation.residualRmsMeters.isFinite() || observation.residualRmsMeters < 0.0 ||
            observation.inlierCount < 3 || observation.pointCount < observation.inlierCount ||
            !observation.arcSpanRadians.isFinite() || observation.arcSpanRadians <= 0.0 ||
            observation.arcSpanRadians > 2.0 * Math.PI
        ) return null
        val heading = Math.toRadians(observation.headingDegrees % 360.0)
        val c = cos(heading)
        val s = sin(heading)
        val north = c * f - s * r
        val east = s * f + c * r
        val vn = c * c * vf + s * s * vr - 2.0 * c * s * cov
        val ve = s * s * vf + c * c * vr + 2.0 * c * s * cov
        val ne = c * s * (vf - vr) + (c * c - s * s) * cov
        val range = hypot(north, east)
        if (!range.isFinite() || range < MINIMUM_RANGE_METERS || !positiveDefinite2(vn, ve, ne)) return null
        val n = north / range
        val e = east / range
        val rangeVariance = n * n * vn + 2.0 * n * e * ne + e * e * ve
        val bearingVariance = (e * e * vn - 2.0 * n * e * ne + n * n * ve) / range / range
        val rangeBearing = (n * e * (ve - vn) + (n * n - e * e) * ne) / range
        if (!positiveDefinite2(rangeVariance, bearingVariance, rangeBearing)) return null
        return VisionEvent(
            observation.sampleAtNanos, north, east, vn, ve, ne, range, atan2(east, north),
            rangeVariance, bearingVariance, rangeBearing,
        )
    }

    private fun insert(event: Event): Boolean {
        if (events.size >= config.historyCapacity) return failClosed("history_overflow")
        // Velocity precedes vision at equal measurement times, regardless of arrival order.
        var index = events.size
        while (index > 0) {
            val previous = events[index - 1]
            if (previous.atNanos < event.atNanos ||
                (previous.atNanos == event.atNanos && previous.order <= event.order)
            ) break
            index -= 1
        }
        val delayed = index < events.size
        events.add(index, event)
        replay.copyFrom(if (delayed) checkpoint else committed)
        val replayStart = if (delayed) 0 else events.lastIndex
        for (retainedIndex in replayStart until events.size) {
            val retained = events[retainedIndex]
            if (!propagate(replay, retained.atNanos)) return failClosed("invalid_covariance")
            account(retained, apply(replay, retained))
        }
        committed.copyFrom(replay)
        if (delayed) lateReplayCount += 1
        return event.accepted == true
    }

    private fun trimHistory(nowNanos: Long): Boolean {
        val cutoff = nowNanos - config.historyDurationNanos
        var consumed = 0
        while (consumed < events.size && events[consumed].atNanos < cutoff) {
            val event = events[consumed]
            if (!propagate(checkpoint, event.atNanos)) return failClosed("invalid_covariance")
            apply(checkpoint, event)
            checkpointSealed = true
            consumed += 1
        }
        if (consumed > 0) events.subList(0, consumed).clear()
        var index = seenVisionTimes.lastIndex
        while (index >= 0) {
            if (seenVisionTimes[index] < cutoff) seenVisionTimes.removeAt(index)
            index -= 1
        }
        return true
    }

    private fun account(event: Event, accepted: Boolean) {
        if (event.accepted == accepted) return
        if (event is VisionEvent) {
            if (event.accepted == true) acceptedVisionCount -= 1
            if (event.accepted == false) rejectedVisionCount -= 1
            if (accepted) acceptedVisionCount += 1 else rejectedVisionCount += 1
        } else {
            if (event.accepted == true) acceptedVelocityCount -= 1
            if (accepted) acceptedVelocityCount += 1
        }
        event.accepted = accepted
    }

    private fun apply(state: State, event: Event): Boolean = when (event) {
        is VelocityEvent -> {
            if (state.lastVelocityAtNanos < 0L && state.lastVisionAtNanos < 0L) {
                // With no position measurement yet, initialize directly from measured velocity rather
                // than biasing the first velocity toward the unobserved prior mean.
                state.x[2] = event.north
                state.x[3] = event.east
                state.p[10] = velocityVariance
                state.p[15] = velocityVariance
                state.lastVelocityAtNanos = event.atNanos
                true
            } else {
                h.fill(0.0)
                h[2] = 1.0
                h[7] = 1.0
                val accepted = correct(
                    state, event.north - state.x[2], event.east - state.x[3],
                    velocityVariance, velocityVariance, 0.0,
                    gate = state.lastVelocityAtNanos >= 0L,
                )
                if (accepted) state.lastVelocityAtNanos = event.atNanos
                accepted
            }
        }
        is VisionEvent -> {
            if (state.lastVisionAtNanos < 0L) {
                state.x[0] = event.north
                state.x[1] = event.east
                state.p[0] = event.varianceNorth
                state.p[5] = event.varianceEast
                state.p[1] = event.covariance
                state.p[4] = event.covariance
                state.lastVisionAtNanos = event.atNanos
                state.lastInnovationSquared = null
                true
            } else {
                val range = hypot(state.x[0], state.x[1])
                if (!range.isFinite() || range < MINIMUM_RANGE_METERS) {
                    state.lastInnovationSquared = null
                    false
                } else {
                    val n = state.x[0] / range
                    val e = state.x[1] / range
                    h.fill(0.0)
                    h[0] = n
                    h[1] = e
                    h[4] = -e / range
                    h[5] = n / range
                    val bearingError = event.bearing - atan2(state.x[1], state.x[0])
                    val accepted = correct(
                        state, event.range - range, atan2(sin(bearingError), cos(bearingError)),
                        event.rangeVariance, event.bearingVariance, event.rangeBearingCovariance,
                        gate = true,
                    )
                    state.lastInnovationSquared = innovationSquared
                    if (accepted) state.lastVisionAtNanos = event.atNanos
                    accepted
                }
            }
        }
    }

    private fun propagate(state: State, atNanos: Long): Boolean {
        if (atNanos < state.atNanos) return false
        val dt = (atNanos - state.atNanos) / NANOS_PER_SECOND
        if (dt == 0.0) return true
        val p = state.p
        if (state.lastVisionAtNanos >= 0L) {
            state.x[0] -= dt * state.x[2]
            state.x[1] -= dt * state.x[3]
            for (row in 0..3) {
                for (column in 0..3) {
                    var value = p[row * 4 + column]
                    if (row < 2) value -= dt * p[(row + 2) * 4 + column]
                    if (column < 2) value -= dt * p[row * 4 + column + 2]
                    if (row < 2 && column < 2) value += dt * dt * p[(row + 2) * 4 + column + 2]
                    candidateP[row * 4 + column] = value
                }
            }
            val qPosition = accelerationVariance * dt * dt * dt / 3.0
            val qCross = -accelerationVariance * dt * dt / 2.0
            candidateP[0] += qPosition
            candidateP[5] += qPosition
            candidateP[2] += qCross
            candidateP[8] += qCross
            candidateP[7] += qCross
            candidateP[13] += qCross
        } else {
            // No relative position prior exists before the first actual image measurement.
            p.copyInto(candidateP)
        }
        candidateP[10] += accelerationVariance * dt
        candidateP[15] += accelerationVariance * dt
        if (state.x.any { !it.isFinite() } || !validCovariance(candidateP)) return false
        candidateP.copyInto(p)
        state.atNanos = atNanos
        return true
    }

    /** H is the current measurement Jacobian; noise is a correlated two-dimensional R. */
    private fun correct(
        state: State,
        innovation0: Double,
        innovation1: Double,
        noise00: Double,
        noise11: Double,
        noise01: Double,
        gate: Boolean,
    ): Boolean {
        innovationSquared = null
        if (!innovation0.isFinite() || !innovation1.isFinite()) return false
        for (row in 0..3) {
            for (measurement in 0..1) {
                var value = 0.0
                for (column in 0..3) value += state.p[row * 4 + column] * h[measurement * 4 + column]
                pHt[row * 2 + measurement] = value
            }
        }
        var s00 = noise00
        var s11 = noise11
        var s01 = noise01
        for (column in 0..3) {
            s00 += h[column] * pHt[column * 2]
            s11 += h[4 + column] * pHt[column * 2 + 1]
            s01 += h[column] * pHt[column * 2 + 1]
        }
        if (!positiveDefinite2(s00, s11, s01)) return false
        val determinant = s00 * s11 - s01 * s01
        if (!determinant.isFinite() || determinant <= 0.0) return false
        val inverse00 = s11 / determinant
        val inverse11 = s00 / determinant
        val inverse01 = -s01 / determinant
        val nis = innovation0 * (inverse00 * innovation0 + inverse01 * innovation1) +
            innovation1 * (inverse01 * innovation0 + inverse11 * innovation1)
        if (!nis.isFinite() || nis < -NUMERICAL_TOLERANCE) return false
        innovationSquared = max(0.0, nis)
        if (gate && nis > config.innovationGateSquared) return false
        for (row in 0..3) {
            gain[row * 2] = pHt[row * 2] * inverse00 + pHt[row * 2 + 1] * inverse01
            gain[row * 2 + 1] = pHt[row * 2] * inverse01 + pHt[row * 2 + 1] * inverse11
            candidateX[row] = state.x[row] + gain[row * 2] * innovation0 + gain[row * 2 + 1] * innovation1
            for (column in 0..3) {
                transform[row * 4 + column] = (if (row == column) 1.0 else 0.0) -
                    gain[row * 2] * h[column] - gain[row * 2 + 1] * h[4 + column]
            }
        }
        // Joseph covariance: (I-KH) P (I-KH)' + K R K'.
        for (row in 0..3) {
            for (column in 0..3) {
                var value = 0.0
                for (inner in 0..3) value += transform[row * 4 + inner] * state.p[inner * 4 + column]
                product[row * 4 + column] = value
            }
        }
        for (row in 0..3) {
            for (column in 0..3) {
                var value = 0.0
                for (inner in 0..3) value += product[row * 4 + inner] * transform[column * 4 + inner]
                value += gain[row * 2] * (noise00 * gain[column * 2] + noise01 * gain[column * 2 + 1]) +
                    gain[row * 2 + 1] * (noise01 * gain[column * 2] + noise11 * gain[column * 2 + 1])
                candidateP[row * 4 + column] = value
            }
        }
        if (candidateX.any { !it.isFinite() } || !validCovariance(candidateP)) return false
        candidateX.copyInto(state.x)
        candidateP.copyInto(state.p)
        return true
    }

    private fun validCovariance(p: DoubleArray): Boolean {
        if (p.any { !it.isFinite() }) return false
        for (row in 0..3) {
            for (column in 0 until row) {
                val symmetric = p[row * 4 + column] * 0.5 + p[column * 4 + row] * 0.5
                p[row * 4 + column] = symmetric
                p[column * 4 + row] = symmetric
            }
        }
        cholesky.fill(0.0)
        val tolerance = NUMERICAL_TOLERANCE * max(1.0, max(max(p[0], p[5]), max(p[10], p[15])))
        for (row in 0..3) {
            for (column in 0..row) {
                var value = p[row * 4 + column]
                for (inner in 0 until column) value -= cholesky[row * 4 + inner] * cholesky[column * 4 + inner]
                if (!value.isFinite()) return false
                if (row == column) {
                    if (value < -tolerance) return false
                    cholesky[row * 4 + column] = sqrt(max(0.0, value))
                } else if (cholesky[column * 4 + column] > 0.0) {
                    cholesky[row * 4 + column] = value / cholesky[column * 4 + column]
                } else if (abs(value) > tolerance) {
                    return false
                }
            }
        }
        return true
    }

    private fun maximumStd(variance0: Double, variance1: Double, covariance: Double): Double =
        sqrt(max(0.0, variance0 * 0.5 + variance1 * 0.5 + hypot((variance0 - variance1) * 0.5, covariance)))

    private fun positiveDefinite2(variance0: Double, variance1: Double, covariance: Double): Boolean =
        variance0.isFinite() && variance1.isFinite() && covariance.isFinite() &&
            variance0 > 0.0 && variance1 > 0.0 && abs(covariance) < sqrt(variance0) * sqrt(variance1)

    private fun failClosed(reason: String): Boolean {
        faultReason = reason
        events.clear()
        seenVisionTimes.clear()
        return false
    }

    private fun result(estimate: CircleStateEstimate? = null, reason: String? = null) = CircleEstimateResult(
        estimate = estimate,
        reason = reason,
        acceptedVisionCount = acceptedVisionCount,
        rejectedVisionCount = rejectedVisionCount,
        acceptedVelocityCount = acceptedVelocityCount,
        lateReplayCount = lateReplayCount,
        lastInnovationSquared = committed.lastInnovationSquared,
        lastVisionAtNanos = committed.lastVisionAtNanos.coerceAtLeast(0L),
        lastVelocityAtNanos = committed.lastVelocityAtNanos.coerceAtLeast(0L),
    )

    private companion object {
        const val NANOS_PER_SECOND = 1_000_000_000.0
        const val UNOBSERVED_VELOCITY_VARIANCE = 100.0
        const val MINIMUM_RANGE_METERS = 1e-6
        const val NUMERICAL_TOLERANCE = 1e-12
    }
}
