package com.durendal.droneagent.lite

import android.util.Log
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.ThreadPoolExecutor
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.roundToInt
import kotlin.math.sin
import org.opencv.android.OpenCVLoader
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc

/** Owns bounded image snapshots; control feedback is opt-in and scoped to one attempt. */
internal class VisualVelocityDiagnostics(
    private val record: (String, Long, String) -> Unit,
) : AutoCloseable {
    /** Immutable main-thread telemetry cache, not a new sensor sample at image submission. */
    data class FrameContext(
        val heightMeters: Double?,
        val heightReceivedAtNanos: Long,
        val heightSource: String,
        val cameraPitchCommandDegrees: Double?,
        val aircraftHeadingDegrees: Double?,
        val listener: VelocityReadDiagnostics.ListenerSample?,
        val capturedAtNanos: Long,
        val aircraftHeadingReceivedAtNanos: Long = 0L,
        val cameraPitchCommandPending: Boolean = false,
    )

    /**
     * Latest outcome, including invalidation; availability is not evidence of actuation.
     * Velocity is interval displacement in the CURRENT image/body frame (+dy forward,
     * -dx right), with cached heading checked at admission/submission for strict scopes.
     * Frame timestamps are decoded-frame admission times, not camera exposure times.
     */
    data class ControlSample(
        val frameNanos: Long,
        val forwardMps: Double?,
        val rightMps: Double?,
        val aircraftHeadingDegrees: Double?,
        val reason: String?,
        val metersPerPixel: Double? = null,
        val analysisWidth: Int = 0,
        val analysisHeight: Int = 0,
        val aircraftHeadingAtNanos: Long = 0L,
    )

    private data class Scope(
        val generation: Long,
        val attemptId: String,
        val phase: String,
        val axisHeading: Double?,
        val startedAtNanos: Long,
        val controlFeedback: Boolean,
        val maximumHeadingAgeNanos: Long,
    )

    private data class Frame(
        val scope: Scope,
        val frameNanos: Long,
        val context: FrameContext,
        val submittedAtNanos: Long,
        val resizeCompletedNanos: Long,
        val width: Int?,
        val height: Int?,
    )

    private data class State(
        val active: Scope?,
        val observedAtNanos: Long,
        val busyDropped: Long,
        val throttled: Long,
        val staleDropped: Long,
    )

    private val lock = Any()
    @Volatile private var closed = false
    private var scope: Scope? = null
    private var generation = 0L
    private var latestControlSample: ControlSample? = null
    private var busy = false
    private var lastAdmittedFrameNanos: Long? = null
    private var busyDropped = 0L
    private var throttled = 0L
    private var staleDropped = 0L
    private var snapshot: Mat? = null
    private val resizeSize = Size()
    // Only the executor touches estimator state, including its termination hook.
    private var estimator: FixedCameraVelocityEstimator? = null
    private var estimatorGeneration: Long? = null
    private var precedingMetricContextValid = false

    init {
        check(OpenCVLoader.initLocal()) { "OpenCV native runtime failed to initialize" }
    }

    private val worker = object : ThreadPoolExecutor(
        1, 1, 0L, TimeUnit.MILLISECONDS, ArrayBlockingQueue<Runnable>(1),
        { runnable -> Thread(runnable, "VisualVelocityDiagnostics").apply { isDaemon = true } },
    ) {
        override fun terminated() {
            // shutdown() drains admitted work before this hook. With no worker ever
            // started there are no native estimator allocations to release on the caller.
            try {
                releaseResources()
            } finally {
                super.terminated()
            }
        }
    }

    fun isActive(): Boolean = synchronized(lock) { !closed && scope != null }

    fun controlSample(attemptId: String): ControlSample? = synchronized(lock) {
        val active = scope
        if (closed || active == null || active.attemptId != attemptId || !active.controlFeedback) null
        else latestControlSample
    }

    fun start(
        attemptId: String,
        phase: String,
        axisHeading: Double?,
        controlFeedback: Boolean = false,
        maximumHeadingAgeNanos: Long = MAX_HEADING_AGE_NANOS,
    ) {
        require(maximumHeadingAgeNanos > 0L) { "Heading age budget must be positive" }
        val previous: Scope?
        val current: Scope
        synchronized(lock) {
            if (closed) return
            previous = scope
            current = Scope(
                ++generation, attemptId, phase, axisHeading.finite(), System.nanoTime(),
                controlFeedback, maximumHeadingAgeNanos,
            )
            scope = current
            latestControlSample = null
            // Do not clear busy or reuse the snapshot while an old generation still owns it.
        }
        previous?.let { recordSession(it, "stop", "replaced") }
        recordSession(current, "start", null)
    }

    fun updatePhase(phase: String, attemptId: String? = null) {
        synchronized(lock) {
            if (closed || (attemptId != null && scope?.attemptId != attemptId)) return
            scope = scope?.copy(phase = phase)
            // Same-generation phase changes intentionally retain the preceding image baseline.
        }
    }

    fun stop(attemptId: String? = null) {
        val previous = synchronized(lock) {
            if (closed || (attemptId != null && scope?.attemptId != attemptId)) return
            scope.also {
                scope = null
                latestControlSample = null
            }
        }
        previous?.let { recordSession(it, "stop", null) }
    }

    /** Borrows the detector's stable RGBA Mat only for this synchronous small-image copy. */
    fun submitRgba(rgba: Mat, frameNanos: Long, context: FrameContext) {
        var failedFrame: Frame? = null
        var failure: Throwable? = null
        var failureStage = "snapshot"
        synchronized(lock) {
            val active = scope ?: return
            if (closed) return
            val previousFrameNanos = lastAdmittedFrameNanos
            if (frameNanos < active.startedAtNanos ||
                (previousFrameNanos != null && frameNanos <= previousFrameNanos)
            ) {
                staleDropped++
                return
            }
            if (previousFrameNanos != null && frameNanos - previousFrameNanos < FRAME_INTERVAL_NANOS) {
                throttled++
                return
            }
            if (busy) {
                busyDropped++
                return
            }
            busy = true
            lastAdmittedFrameNanos = frameNanos
            val submittedAtNanos = System.nanoTime()
            var width: Int? = null
            var height: Int? = null
            var resizedAtNanos: Long? = null
            try {
                require(!rgba.empty() && rgba.type() == CvType.CV_8UC4) { "Expected nonempty CV_8UC4 frame" }
                val sourceWidth = rgba.cols()
                val sourceHeight = rgba.rows()
                // Also bound the long edge for malformed/portrait input; the estimator
                // reports unsupported metric geometry rather than assuming landscape.
                val scale = minOf(1.0, MAX_ANALYSIS_EDGE / max(sourceWidth, sourceHeight).toDouble())
                val analysisWidth = (sourceWidth * scale).roundToInt().coerceAtLeast(1)
                val analysisHeight = (sourceHeight * scale).roundToInt().coerceAtLeast(1)
                width = analysisWidth
                height = analysisHeight
                val owned = snapshot ?: Mat().also { snapshot = it }
                if (analysisWidth == sourceWidth && analysisHeight == sourceHeight) {
                    rgba.copyTo(owned)
                } else {
                    resizeSize.width = analysisWidth.toDouble()
                    resizeSize.height = analysisHeight.toDouble()
                    Imgproc.resize(rgba, owned, resizeSize, 0.0, 0.0, Imgproc.INTER_AREA)
                }
                val resizeCompletedNanos = System.nanoTime()
                resizedAtNanos = resizeCompletedNanos
                val frame = Frame(
                    active, frameNanos, context, submittedAtNanos, resizeCompletedNanos,
                    analysisWidth, analysisHeight,
                )
                failureStage = "schedule"
                worker.execute { process(frame, owned) }
            } catch (error: Throwable) {
                // Admission, copying and scheduling share the close lock: release can
                // never run while the borrowed source or owned snapshot is being copied.
                busy = false
                failure = error
                failedFrame = Frame(
                    active, frameNanos, context, submittedAtNanos,
                    resizedAtNanos ?: System.nanoTime(), width, height,
                )
            }
        }
        val frame = failedFrame ?: return
        recordOutcome(frame, null, null, System.nanoTime(), failureStage, failure)
    }

    private fun process(frame: Frame, rgba: Mat) {
        try {
            if (closed) return
            val processingStartedNanos = System.nanoTime()
            var estimate: FixedCameraVelocityEstimator.Estimate? = null
            var failure: Throwable? = null
            try {
                val current = estimator ?: FixedCameraVelocityEstimator().also { estimator = it }
                val metricContextValid =
                    if (frame.scope.maximumHeadingAgeNanos == MAX_HEADING_AGE_NANOS) {
                        metricContextReason(frame) == null
                    } else {
                        // Heading gates consumption, not image registration or marker scale.
                        // A delayed 5Hz attitude update must not erase valid ground geometry.
                        imageContextReason(frame) == null
                    }
                if (estimatorGeneration != frame.scope.generation ||
                    (metricContextValid && !precedingMetricContextValid)
                ) {
                    current.reset()
                    estimatorGeneration = frame.scope.generation
                }
                // Pixel tracking may continue with unsupported context, but its scale
                // must never propagate into the next supported camera/context interval.
                precedingMetricContextValid = metricContextValid
                estimate = current.process(rgba, frame.frameNanos)
            } catch (error: Throwable) {
                estimatorGeneration = null
                failure = error
            }
            recordOutcome(frame, estimate, processingStartedNanos, System.nanoTime(), "process", failure)
        } finally {
            // Includes logging: no new admission can overwrite the image during processing.
            synchronized(lock) { busy = false }
        }
    }

    private fun state(): State? = synchronized(lock) {
        if (closed) return null
        State(scope, System.nanoTime(), busyDropped, throttled, staleDropped)
    }

    private fun recordOutcome(
        frame: Frame,
        estimate: FixedCameraVelocityEstimator.Estimate?,
        processingStartedNanos: Long?,
        processingCompletedNanos: Long,
        stage: String,
        error: Throwable?,
    ) {
        val context = frame.context
        val listener = context.listener
        val listenerValid = listener != null && listener.x.isFinite() &&
            listener.y.isFinite() && listener.z.isFinite()
        val axisRadians = frame.scope.axisHeading?.let(Math::toRadians)
        val listenerAlong = if (listenerValid && axisRadians != null) {
            (listener!!.x * cos(axisRadians) + listener.y * sin(axisRadians)).finite()
        } else null
        val motionValid = estimate?.motionValid == true &&
            estimate.deltaXPixels.finite() != null && estimate.deltaYPixels.finite() != null
        val contextReason = metricContextReason(frame)
        val metricEstimateValid = motionValid && estimate?.valueValid == true &&
            estimate.forwardMps.finite() != null && estimate.rightMps.finite() != null &&
            estimate.metersPerPixel.positive() != null &&
            !estimate.scaleSource.isNullOrBlank() &&
            estimate.scaleAgeMs.finite()?.let { it >= 0.0 } == true
        val state: State
        val active: Scope?
        val valueValid: Boolean
        val reason: String?
        val controlSampleOffered: Boolean
        synchronized(lock) {
            if (closed) return
            state = State(scope, System.nanoTime(), busyDropped, throttled, staleDropped)
            active = state.active?.takeIf { it.generation == frame.scope.generation }
            valueValid = error == null && metricEstimateValid && contextReason == null && active != null
            reason = when {
                error != null -> "${stage.uppercase()}_ERROR"
                estimate == null -> "NO_ESTIMATE"
                active == null -> "SESSION_ENDED"
                estimate.motionValid && !motionValid -> "NON_FINITE_MOTION"
                motionValid && contextReason != null -> contextReason
                estimate.valueValid && !metricEstimateValid -> "INVALID_METRIC_VALUE"
                !valueValid -> estimate.reason ?: "INVALID_ESTIMATE"
                else -> estimate.reason
            }
            controlSampleOffered = active?.controlFeedback == true && valueValid
            if (active?.controlFeedback == true) {
                // Check generation and publish atomically, before any external callback.
                // Invalid outcomes revoke the previous velocity rather than retaining it.
                latestControlSample = ControlSample(
                    frameNanos = frame.frameNanos,
                    forwardMps = estimate?.forwardMps?.takeIf { valueValid },
                    rightMps = estimate?.rightMps?.takeIf { valueValid },
                    aircraftHeadingDegrees = context.aircraftHeadingDegrees.finite(),
                    reason = reason,
                    metersPerPixel = estimate?.metersPerPixel?.takeIf { valueValid },
                    analysisWidth = estimate?.width ?: frame.width ?: 0,
                    analysisHeight = estimate?.height ?: frame.height ?: 0,
                    aircraftHeadingAtNanos = context.aircraftHeadingReceivedAtNanos,
                )
            }
        }
        if (error != null) {
            emit(
                "visual_velocity_error", processingCompletedNanos,
                profileDetails(
                    "source" to "vision_translation", "attemptId" to frame.scope.attemptId,
                    "generation" to frame.scope.generation, "phase" to frame.scope.phase,
                    "scopeActive" to (active != null), "frameNanos" to frame.frameNanos,
                    "stage" to stage, "reason" to reason, "error" to errorDescription(error),
                    "motionValid" to false, "valueValid" to false,
                    "controlFeedback" to frame.scope.controlFeedback, "controlSampleOffered" to false,
                    "controlFeedbackMeaning" to "enabled_not_consumption",
                ),
            )
        }
        emit(
            "visual_velocity_result", processingCompletedNanos,
            profileDetails(
                "source" to "vision_translation", "attemptId" to frame.scope.attemptId,
                "generation" to frame.scope.generation, "phase" to frame.scope.phase,
                "responsePhase" to active?.phase, "scopeActive" to (active != null),
                "scopeObservedAtNanos" to state.observedAtNanos,
                "sessionStartedAtNanos" to frame.scope.startedAtNanos,
                "frameNanos" to frame.frameNanos, "previousFrameNanos" to estimate?.previousFrameNanos,
                "frameIntervalMs" to estimate?.previousFrameNanos?.let { milliseconds(frame.frameNanos - it) },
                "submittedAtNanos" to frame.submittedAtNanos,
                "processingStartedNanos" to processingStartedNanos,
                "processingCompletedNanos" to processingCompletedNanos,
                "frameAgeMs" to milliseconds(processingCompletedNanos - frame.frameNanos),
                "processingMs" to processingStartedNanos?.let { milliseconds(processingCompletedNanos - it) },
                "resizeMs" to milliseconds(frame.resizeCompletedNanos - frame.submittedAtNanos),
                "workerWaitMs" to processingStartedNanos?.let { milliseconds(it - frame.resizeCompletedNanos) },
                "motionValid" to motionValid, "valueValid" to valueValid, "reason" to reason,
                "dx" to estimate?.deltaXPixels.finite(), "dy" to estimate?.deltaYPixels.finite(),
                "forwardMps" to estimate?.forwardMps?.takeIf { valueValid },
                "rightMps" to estimate?.rightMps?.takeIf { valueValid },
                "heightMeaning" to "telemetry_only_not_metric_scale",
                "inputHeightMeters" to context.heightMeters.positive(),
                "inputHeightReason" to heightReason(context.heightMeters),
                "heightReceivedAtNanos" to context.heightReceivedAtNanos.takeUnless { it == 0L },
                "heightAgeMs" to context.heightReceivedAtNanos.takeUnless { it == 0L }
                    ?.let { milliseconds(frame.submittedAtNanos - it) },
                "heightSource" to context.heightSource,
                "contextCapturedAtNanos" to context.capturedAtNanos,
                "contextAgeMs" to milliseconds(frame.submittedAtNanos - context.capturedAtNanos),
                "metricContextReason" to contextReason,
                "cameraPitchCommandDegrees" to context.cameraPitchCommandDegrees.finite(),
                "cameraPitchReason" to context.cameraPitchCommandDegrees.finiteReason(),
                "cameraPitchCommandPending" to context.cameraPitchCommandPending,
                "cameraPitchMeaning" to "accepted_command_not_measured_camera_pose",
                "axisHeading" to frame.scope.axisHeading,
                "axisHeadingReason" to if (frame.scope.axisHeading == null) "MISSING_OR_NON_FINITE" else null,
                "aircraftHeading" to context.aircraftHeadingDegrees.finite(),
                "aircraftHeadingReason" to context.aircraftHeadingDegrees.finiteReason(),
                "aircraftHeadingReceivedAtNanos" to context.aircraftHeadingReceivedAtNanos.takeUnless { it == 0L },
                "aircraftHeadingAgeMs" to context.aircraftHeadingReceivedAtNanos.takeUnless { it == 0L }
                    ?.let { milliseconds(frame.submittedAtNanos - it) },
                "aircraftHeadingAgeAtFrameMs" to context.aircraftHeadingReceivedAtNanos.takeUnless { it == 0L }
                    ?.let { milliseconds(frame.frameNanos - it) },
                "maximumHeadingAgeNanos" to frame.scope.maximumHeadingAgeNanos,
                "listenerX" to listener?.x.finite(), "listenerY" to listener?.y.finite(),
                "listenerZ" to listener?.z.finite(), "listenerValueValid" to listenerValid,
                "listenerReason" to when {
                    listener == null -> "MISSING_LISTENER"
                    !listenerValid -> "NON_FINITE_LISTENER"
                    else -> null
                },
                "listenerReceivedAtNanos" to listener?.receivedAtNanos?.takeUnless { it == 0L },
                "listenerAgeMs" to listener?.receivedAtNanos?.takeUnless { it == 0L }
                    ?.let { milliseconds(frame.submittedAtNanos - it) },
                "listenerAgeAtFrameMs" to listener?.receivedAtNanos?.takeUnless { it == 0L }
                    ?.let { milliseconds(frame.frameNanos - it) },
                "listenerAlongRawMps" to listenerAlong,
                "listenerSnapshotMeaning" to "cached_main_thread_context",
                "analysisWidth" to (estimate?.width ?: frame.width),
                "analysisHeight" to (estimate?.height ?: frame.height),
                "tracks" to estimate?.tracks, "inliers" to estimate?.inliers,
                "supportAreaPixels" to estimate?.supportAreaPixels.finite(),
                "residualRmsPixels" to estimate?.residualRmsPixels.finite(),
                "rotationDegrees" to estimate?.rotationDegrees.finite(),
                "imageScale" to estimate?.imageScale.positive(),
                "metersPerPixel" to estimate?.metersPerPixel.positive(),
                "scaleSource" to estimate?.scaleSource,
                "scaleAgeMs" to estimate?.scaleAgeMs.finite(),
                "markerDistancePixels" to estimate?.markerDistancePixels.positive(),
                "markerScaleErrorPercent" to estimate?.markerScaleErrorPercent.finite(),
                "busyDropped" to state.busyDropped, "throttled" to state.throttled,
                "staleDropped" to state.staleDropped, "counterScope" to "diagnostics_lifetime",
                "fixedCamera" to true, "rotationEstimated" to true,
                "calibrationId" to FixedCameraVelocityEstimator.CALIBRATION_ID,
                "markerDistanceMeters" to FixedCameraVelocityEstimator.MARKER_DISTANCE_METERS,
                "calibrationMeaning" to "isolated_yellow_pairs_center_to_center_0p20m",
                "motionModel" to "ransac_planar_similarity_about_image_center",
                "modelLimitation" to "in_plane_rotation_isotropic_zoom_not_calibrated_camera_pose_or_perspective",
                "metricScaleMeaning" to "marker_observed_or_short_term_similarity_propagated_no_height_fallback",
                "floorMaskMeaning" to "scene_specific_cardboard_and_room_floor_not_general_classifier",
                "timestampMeaning" to "app_decoded_frame_admission_not_camera_exposure",
                "phaseMeaning" to "image_submission", "controlFeedback" to frame.scope.controlFeedback,
                "controlSampleOffered" to controlSampleOffered,
                "controlFeedbackMeaning" to "enabled_not_consumption",
            ),
        )
    }

    private fun recordSession(session: Scope, action: String, reason: String?) {
        val state = state() ?: return
        emit(
            "visual_velocity_session", state.observedAtNanos,
            profileDetails(
                "source" to "vision_translation", "action" to action, "reason" to reason,
                "attemptId" to session.attemptId, "generation" to session.generation,
                "phase" to session.phase, "axisHeading" to session.axisHeading,
                "scopeActive" to (state.active?.generation == session.generation),
                "startedAtNanos" to session.startedAtNanos,
                "busyDropped" to state.busyDropped, "throttled" to state.throttled,
                "staleDropped" to state.staleDropped, "counterScope" to "diagnostics_lifetime",
                "fixedCamera" to true, "rotationEstimated" to true,
                "calibrationId" to FixedCameraVelocityEstimator.CALIBRATION_ID,
                "markerDistanceMeters" to FixedCameraVelocityEstimator.MARKER_DISTANCE_METERS,
                "motionModel" to "ransac_planar_similarity_about_image_center",
                "modelLimitation" to "in_plane_rotation_isotropic_zoom_not_calibrated_camera_pose_or_perspective",
                "metricScaleMeaning" to "marker_observed_or_short_term_similarity_propagated_no_height_fallback",
                "controlFeedback" to session.controlFeedback,
                "maximumHeadingAgeNanos" to session.maximumHeadingAgeNanos,
                "controlFeedbackMeaning" to "enabled_not_consumption",
            ),
        )
    }

    private fun emit(event: String, atNanos: Long, details: String) {
        // Never invoke external code under the image/lifecycle lock. Close suppresses
        // late results; an already-entered callback may finish concurrently with close.
        if (closed) return
        try {
            record(event, atNanos, details)
        } catch (error: Throwable) {
            // A failed profiler callback must not fail detector work or a flight lifecycle call.
            Log.w(TAG, "Visual velocity profiling callback failed", error)
        }
    }

    override fun close() {
        stop()
        synchronized(lock) {
            if (closed) return
            closed = true
            scope = null
            latestControlSample = null
            // No waiting on main, cancellation, or concurrent native release. The
            // termination hook owns cleanup after the sole admitted task returns.
            worker.shutdown()
        }
    }

    private fun releaseResources() {
        try {
            estimator?.close()
        } catch (error: Throwable) {
            Log.w(TAG, "Visual velocity estimator release failed", error)
        } finally {
            estimator = null
            estimatorGeneration = null
            try {
                snapshot?.release()
            } catch (error: Throwable) {
                Log.w(TAG, "Visual velocity snapshot release failed", error)
            } finally {
                snapshot = null
            }
        }
    }

    private fun imageContextReason(frame: Frame): String? {
        val context = frame.context
        val contextAge = frame.submittedAtNanos - context.capturedAtNanos
        return when {
            context.capturedAtNanos == 0L -> "MISSING_CONTEXT_TIMESTAMP"
            contextAge < 0L -> "FUTURE_CONTEXT"
            contextAge > MAX_CONTEXT_AGE_NANOS -> "STALE_CONTEXT"
            context.cameraPitchCommandPending -> "CAMERA_PITCH_PENDING"
            context.cameraPitchCommandDegrees == null -> "MISSING_CAMERA_PITCH"
            !context.cameraPitchCommandDegrees.isFinite() -> "NON_FINITE_CAMERA_PITCH"
            abs(context.cameraPitchCommandDegrees + 90.0) > 0.5 -> "CAMERA_NOT_DOWNWARD"
            else -> null
        }
    }

    private fun metricContextReason(frame: Frame): String? {
        imageContextReason(frame)?.let { return it }
        val context = frame.context
        val headingAge = frame.submittedAtNanos - context.aircraftHeadingReceivedAtNanos
        val headingAgeAtFrame = frame.frameNanos - context.aircraftHeadingReceivedAtNanos
        // Legacy fixed-heading callers retain their submission-only check. A configured
        // budget also pairs heading with frame admission: later telemetry is not the
        // heading of this rotating image, even if it is fresh at submission.
        val pairHeadingWithFrame = frame.scope.maximumHeadingAgeNanos != MAX_HEADING_AGE_NANOS
        return when {
            context.aircraftHeadingDegrees == null -> "MISSING_AIRCRAFT_HEADING"
            !context.aircraftHeadingDegrees.isFinite() -> "NON_FINITE_AIRCRAFT_HEADING"
            context.aircraftHeadingReceivedAtNanos == 0L -> "MISSING_HEADING_TIMESTAMP"
            headingAge < 0L || (pairHeadingWithFrame && headingAgeAtFrame < 0L) -> "FUTURE_AIRCRAFT_HEADING"
            headingAge > frame.scope.maximumHeadingAgeNanos ||
                (pairHeadingWithFrame && headingAgeAtFrame > frame.scope.maximumHeadingAgeNanos) ->
                "STALE_AIRCRAFT_HEADING"
            else -> null
        }
    }

    private fun Double?.finite(): Double? = this?.takeIf { it.isFinite() }
    private fun Double?.positive(): Double? = finite()?.takeIf { it > 0.0 }
    private fun Double?.finiteReason(): String? = when {
        this == null -> "MISSING_VALUE"
        !isFinite() -> "NON_FINITE_VALUE"
        else -> null
    }
    private fun milliseconds(nanos: Long): Double = nanos / 1_000_000.0
    private fun heightReason(height: Double?): String? = when {
        height == null -> "MISSING_HEIGHT"
        !height.isFinite() -> "NON_FINITE_HEIGHT"
        height <= 0.0 -> "NON_POSITIVE_HEIGHT"
        else -> null
    }

    private fun errorDescription(error: Throwable): String =
        "${error.javaClass.simpleName}:${error.message}"
            .replace(' ', '_').replace('\n', '_').replace('\r', '_').replace('\t', '_')

    private companion object {
        const val TAG = "VisualVelocityDiag"
        const val MAX_ANALYSIS_EDGE = 848
        // Ceiling division prevents accepting even slightly more than 15 Hz.
        const val FRAME_INTERVAL_NANOS = 66_666_667L
        const val MAX_CONTEXT_AGE_NANOS = 250_000_000L
        const val MAX_HEADING_AGE_NANOS = 1_000_000_000L
    }
}
