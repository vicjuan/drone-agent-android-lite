package com.durendal.droneagent.lite

import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/** Read-only comparison; results never update the velocity used by flight control. */
internal class VelocityReadDiagnostics(
    private val readVelocity: ((Velocity?, String?) -> Unit) -> Unit,
    private val listenerSnapshot: () -> ListenerSample?,
    private val record: (String, Long, String) -> Unit,
    private val clock: () -> Long = System::nanoTime,
) : AutoCloseable {
    data class Velocity(val x: Double, val y: Double, val z: Double)
    data class ListenerSample(val x: Double, val y: Double, val z: Double, val receivedAtNanos: Long)
    private data class Scope(
        val generation: Long,
        val attemptId: String,
        val phase: String,
        val axisHeading: Double?,
    )
    private data class Request(val id: Long, val atNanos: Long, val scope: Scope)

    private val lock = Any()
    private var scope: Scope? = null
    private var generation = 0L
    private var requestId = 0L
    private var pending: Request? = null
    private var nextRequestAtNanos = Long.MIN_VALUE
    private var closed = false

    fun start(attemptId: String, phase: String, axisHeading: Double?) = synchronized(lock) {
        check(!closed)
        scope = Scope(++generation, attemptId, phase, axisHeading?.takeIf(Double::isFinite))
    }

    fun updatePhase(phase: String) = synchronized(lock) {
        scope = scope?.copy(phase = phase)
    }

    fun stop() = synchronized(lock) {
        scope = null
        // An SDK request cannot be cancelled here. Retain its slot until its callback arrives,
        // even if another experiment starts, rather than creating overlapping hardware reads.
    }

    fun tick() {
        val request = synchronized(lock) {
            val active = scope ?: return
            val now = clock()
            if (closed || pending != null || now < nextRequestAtNanos) return
            Request(++requestId, now, active).also {
                pending = it
                nextRequestAtNanos = now + INTERVAL_NANOS
            }
        }
        val listener = listenerSnapshot()
        record(
            "velocity_read_request", request.atNanos,
            profileDetails(
                "source" to "async_get", "attemptId" to request.scope.attemptId,
                "requestId" to request.id, "requestNanos" to request.atNanos,
                "phase" to request.scope.phase, "axisHeading" to request.scope.axisHeading,
                "listenerReceivedAtNanos" to listener?.receivedAtNanos,
                "listenerAgeMs" to listener?.let { (request.atNanos - it.receivedAtNanos) / 1_000_000.0 },
                "listenerX" to listener?.x, "listenerY" to listener?.y, "listenerZ" to listener?.z,
            ),
        )
        try {
            readVelocity { velocity, error -> complete(request, velocity, error) }
        } catch (error: Exception) {
            complete(request, null, "${error.javaClass.simpleName}:${error.message}")
        }
    }

    private fun complete(request: Request, velocity: Velocity?, error: String?) {
        val responseAtNanos = clock()
        val active = synchronized(lock) {
            if (pending !== request) return
            pending = null
            if (closed) return
            scope?.takeIf { it.generation == request.scope.generation }
        }
        val listener = listenerSnapshot()
        val valid = error == null && velocity != null &&
            velocity.x.isFinite() && velocity.y.isFinite() && velocity.z.isFinite()
        val axis = request.scope.axisHeading?.let(Math::toRadians)
        val along = if (valid && axis != null) velocity!!.x * cos(axis) + velocity.y * sin(axis) else null
        val cross = if (valid && axis != null) -velocity!!.x * sin(axis) + velocity.y * cos(axis) else null
        record(
            "velocity_read_result", responseAtNanos,
            profileDetails(
                "source" to "async_get", "attemptId" to request.scope.attemptId,
                "requestId" to request.id, "requestNanos" to request.atNanos,
                "responseNanos" to responseAtNanos,
                "roundTripMs" to (responseAtNanos - request.atNanos) / 1_000_000.0,
                "phase" to request.scope.phase, "responsePhase" to active?.phase,
                "scopeActive" to (active != null), "success" to (error == null), "valueValid" to valid,
                "error" to (error ?: if (velocity == null) "EMPTY_RESULT" else if (!valid) "NON_FINITE_VALUE" else null)
                    ?.replace(' ', '_')?.replace('\n', '_')?.replace('\r', '_')?.replace('\t', '_'),
                "x" to velocity?.x, "y" to velocity?.y, "z" to velocity?.z,
                "groundSpeed" to if (valid) hypot(velocity!!.x, velocity.y) else null,
                "alongRawMps" to along, "crossRawMps" to cross,
                "axisHeading" to request.scope.axisHeading,
                "listenerReceivedAtNanos" to listener?.receivedAtNanos,
                "listenerAgeMs" to listener?.let { (responseAtNanos - it.receivedAtNanos) / 1_000_000.0 },
                "listenerX" to listener?.x, "listenerY" to listener?.y, "listenerZ" to listener?.z,
                "timestampMeaning" to "app_request_response_not_sensor_sample",
            ),
        )
    }

    override fun close() = synchronized(lock) {
        closed = true
        scope = null
    }

    companion object {
        const val INTERVAL_MS = 200L
        private const val INTERVAL_NANOS = INTERVAL_MS * 1_000_000L
    }
}
