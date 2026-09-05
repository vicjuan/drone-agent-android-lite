package com.durendal.droneagent.lite

import org.opencv.android.OpenCVLoader
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.hypot

internal object BilateralBoardPolicy {
    const val MIN_PAIR_COUNT = 10
    const val MIN_SAMPLE_COVERAGE_FRACTION = 0.70
    const val MIN_TRACKED_SAMPLE_COVERAGE_FRACTION = 0.65
    const val MIN_COMPATIBLE_PAIR_FRACTION = 0.70
    const val MIN_SIDE_MATCH_FRACTION = 0.85
    const val MIN_BOTH_SIDES_MATCH_FRACTION = 0.80

    fun hasCompatibleSides(
        pairCount: Int,
        sampleCoverageFraction: Double,
        compatiblePairFraction: Double,
    ): Boolean =
        hasCompatibleSides(
            pairCount = pairCount,
            sampleCoverageFraction = sampleCoverageFraction,
            compatiblePairFraction = compatiblePairFraction,
            minimumSampleCoverageFraction = MIN_SAMPLE_COVERAGE_FRACTION,
        )

    fun accepts(
        pairCount: Int,
        sampleCoverageFraction: Double,
        compatiblePairFraction: Double,
        leftMatchFraction: Double,
        rightMatchFraction: Double,
        bothSidesMatchFraction: Double,
    ): Boolean =
        accepts(
            pairCount = pairCount,
            sampleCoverageFraction = sampleCoverageFraction,
            compatiblePairFraction = compatiblePairFraction,
            leftMatchFraction = leftMatchFraction,
            rightMatchFraction = rightMatchFraction,
            bothSidesMatchFraction = bothSidesMatchFraction,
            minimumSampleCoverageFraction = MIN_SAMPLE_COVERAGE_FRACTION,
        )

    fun acceptsTrackedReference(
        pairCount: Int,
        sampleCoverageFraction: Double,
        compatiblePairFraction: Double,
        leftMatchFraction: Double,
        rightMatchFraction: Double,
        bothSidesMatchFraction: Double,
    ): Boolean =
        accepts(
            pairCount = pairCount,
            sampleCoverageFraction = sampleCoverageFraction,
            compatiblePairFraction = compatiblePairFraction,
            leftMatchFraction = leftMatchFraction,
            rightMatchFraction = rightMatchFraction,
            bothSidesMatchFraction = bothSidesMatchFraction,
            minimumSampleCoverageFraction = MIN_TRACKED_SAMPLE_COVERAGE_FRACTION,
        )

    private fun accepts(
        pairCount: Int,
        sampleCoverageFraction: Double,
        compatiblePairFraction: Double,
        leftMatchFraction: Double,
        rightMatchFraction: Double,
        bothSidesMatchFraction: Double,
        minimumSampleCoverageFraction: Double,
    ): Boolean =
        hasCompatibleSides(
            pairCount = pairCount,
            sampleCoverageFraction = sampleCoverageFraction,
            compatiblePairFraction = compatiblePairFraction,
            minimumSampleCoverageFraction = minimumSampleCoverageFraction,
        ) &&
            leftMatchFraction >= MIN_SIDE_MATCH_FRACTION &&
            rightMatchFraction >= MIN_SIDE_MATCH_FRACTION &&
            bothSidesMatchFraction >= MIN_BOTH_SIDES_MATCH_FRACTION

    private fun hasCompatibleSides(
        pairCount: Int,
        sampleCoverageFraction: Double,
        compatiblePairFraction: Double,
        minimumSampleCoverageFraction: Double,
    ): Boolean =
        pairCount >= MIN_PAIR_COUNT &&
            sampleCoverageFraction >= minimumSampleCoverageFraction &&
            compatiblePairFraction >= MIN_COMPATIBLE_PAIR_FRACTION
}

internal const val TAPE_IMAGE_RESIZE_INTERPOLATION = "INTER_AREA"



internal data class TapeFrameProfile(
    val sequence: Long,
    val receivedAtNanos: Long,
    val processingStartedAtNanos: Long,
    val processingCompletedAtNanos: Long,
    val preprocessingNanos: Long,
    val thresholdingNanos: Long,
    val floorContextNanos: Long,
    val morphologyAndContoursNanos: Long,
    val candidateScoringNanos: Long,
    val candidateMaskNanos: Long,
    val candidateCenterlineNanos: Long,
    val candidateAppearanceNanos: Long,
    val candidateTemporalNanos: Long,
    val centerlineDownsampleNanos: Long,
    val centerlineGapFillNanos: Long,
    val centerlineDistanceNanos: Long,
    val centerlineThinningNanos: Long,
    val centerlineRouteNanos: Long,
    val centerlineQualityNanos: Long,
    val centerlineTopologyNanos: Long,
    val candidateMeasurementNanos: Long,
    val contourCount: Int,
    val fullCenterlineExtractions: Int,
    val temporalCenterlineAttempts: Int,
    val temporalCenterlineAccepts: Int,
    val cleanupNanos: Long,
    val width: Int,
    val height: Int,
    val detected: Boolean,
)
/**
 * Finds a dark, tape-shaped region on brown corrugated board. Otsu adapts to
 * exposure, but its threshold is capped so a bright wall cannot make the whole
 * brown board part of the dark class. Geometry, colour context and short-term
 * overlap reject scenery and keep the box on one physical strip.
 */
class BlackTapeDetector internal constructor(
    private val onResult: (TapeDetection?) -> Unit,
    private val onError: (Throwable) -> Unit = {},
    private val onFrameProfile: (TapeFrameProfile) -> Unit = {},
    // Replay evidence is opt-in and owned by the caller: the detector knows the
    // frame and its masks, but only the Activity knows gimbal pose and height.
    private val captureRecorder: TapeCaptureRecorder? = null,
    private val captureFlightContext: () -> Map<String, String> = ::emptyMap,
) : AutoCloseable {

    private val worker = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "BlackTapeDetector").apply { isDaemon = true }
    }
    private val processing = AtomicBoolean(false)
    private val closed = AtomicBoolean(false)
    private val lastAcceptedAtNanos = AtomicLong(0L)
    private val receivedFrameCount = AtomicLong(0L)
    private val invalidFrameCount = AtomicLong(0L)
    private val throttledFrameCount = AtomicLong(0L)
    private val busyDroppedFrameCount = AtomicLong(0L)
    private val acceptedFrameCount = AtomicLong(0L)
    private val completedFrameCount = AtomicLong(0L)
    private val failedFrameCount = AtomicLong(0L)
    private val firstAcceptedAtNanos = AtomicLong(0L)
    private var previousBounds: Rect? = null
    private var previousPathMedianWidthFraction: Double? = null
    private var previousRouteStartXFraction: Double? = null
    private var previousRouteStartYFraction: Double? = null
    private var previousCenterlineEstimate: CenterlineEstimate? = null
    private var temporalFramesSinceFullExtraction = 0
    private var boardReferenceA: Double? = null
    private var boardReferenceB: Double? = null
    private var trackingSessionActive = false
    private var requiresConsistentCurveAcquisition = false
    private val boardReferenceAcquisitionSamples = ArrayList<BoardReferenceAcquisitionSample>()
    private val previewBoardReferenceSamples = ArrayList<BoardReferenceAcquisitionSample>()
    private var previewBoardReferenceLastAtNanos = 0L
    @Volatile private var suppressResultsUntilSessionReady = false
    @Volatile private var boardReferenceAcquisitionCount = 0
    @Volatile private var detectionMode = TapeDetectionMode.PATH
    private var consecutiveDetectionMisses = 0
    @Volatile private var lastOtsuThreshold = 0.0
    @Volatile private var lastEffectiveThreshold = 0.0
    @Volatile private var lastClassSeparation = 0.0
    @Volatile private var lastShadowPixelCount = 0
    @Volatile private var lastContourCount = 0
    private val rejectionCounts = IntArray(TapeCandidateRejection.entries.size)
    private var luminanceBytes = ByteArray(0)
    private var labBytes = ByteArray(0)
    private var blackMaskBytes = ByteArray(0)
    @Volatile private var lastDetectionMode = TapeDetectionMode.PATH
    @Volatile private var lastPathSampleCount = 0
    @Volatile private var lastPathAxis = "NONE"
    @Volatile private var lastPathCurvatureDegrees = 0.0
    @Volatile private var lastPathCurvatureSmoothness = 0.0
    @Volatile private var lastFloorSeedCount = 0
    @Volatile private var lastFloorFraction = 0.0
    @Volatile private var lastFrameMillis = 0.0
    private var lastPreprocessingNanos = 0L
    private var lastThresholdingNanos = 0L
    private var lastFloorContextNanos = 0L
    private var lastMorphologyAndContoursNanos = 0L
    private var lastCandidateScoringNanos = 0L
    private var lastCandidateMaskNanos = 0L
    private var lastCandidateCenterlineNanos = 0L
    private var lastCandidateAppearanceNanos = 0L
    private var lastCandidateTemporalNanos = 0L
    private var lastCenterlineDownsampleNanos = 0L
    private var lastCenterlineGapFillNanos = 0L
    private var lastCenterlineDistanceNanos = 0L
    private var lastCenterlineThinningNanos = 0L
    private var lastCenterlineRouteNanos = 0L
    private var lastCenterlineQualityNanos = 0L
    private var lastCenterlineTopologyNanos = 0L
    private var lastCandidateMeasurementNanos = 0L
    private var lastFullCenterlineExtractions = 0
    private var lastTemporalCenterlineAttempts = 0
    private var lastTemporalCenterlineAccepts = 0
    private var lastCleanupNanos = 0L
    @Volatile private var lastPathCoverage = 0.0
    @Volatile private var lastBranchCount = 0
    @Volatile private var lastRegionDescription = "none"
    @Volatile private var lastArcRejection = "none"
    @Volatile private var lastCandidateMetrics = "none"

    private var lastBoardColorAttemptedPairCount = 0
    private var lastBoardColorPairCount = 0
    private var candidateMaskBytes = ByteArray(0)

    // Non-null only while the current frame is being captured. detect() runs on
    // the single detector thread, so a plain field is the whole synchronisation.
    private var capturePlanes: MutableList<TapeCapturePlane>? = null
    private var capturePlaneBytes = ByteArray(0)
    // processing owns this buffer until its worker task finishes, so accepted frames
    // reuse one full-resolution allocation without racing the decoder callback.
    private var rgbaFrameBytes = ByteArray(0)
    private val centerlineExtractor = CenterlineExtractor(onProfile = ::recordCenterlineProfile)
    private val temporalCenterlineTracker = TemporalCenterlineTracker()
    private var frameSequence = 0L
    private val pipelineBuffersDelegate =
        lazy(LazyThreadSafetyMode.NONE, ::createPipelineBuffers)
    private val pipelineBuffers by pipelineBuffersDelegate

    init {
        check(OpenCVLoader.initLocal()) { "OpenCV native runtime failed to initialize" }
    }

    /** Copies at most one frame per interval; DJI owns the callback byte array. */
    fun submitRgba(
        frameData: ByteArray,
        offset: Int,
        length: Int,
        width: Int,
        height: Int,
    ) {
        if (closed.get()) return
        receivedFrameCount.incrementAndGet()
        if (width <= 0 || height <= 0 || offset < 0 || length <= 0) {
            invalidFrameCount.incrementAndGet()
            return
        }
        val requiredBytes = width.toLong() * height * RGBA_CHANNELS
        if (requiredBytes > length || offset.toLong() + requiredBytes > frameData.size) {
            invalidFrameCount.incrementAndGet()
            return
        }
        val now = System.nanoTime()
        val previous = lastAcceptedAtNanos.get()
        if (now - previous < FRAME_INTERVAL_NANOS) {
            throttledFrameCount.incrementAndGet()
            return
        }
        if (!processing.compareAndSet(false, true)) {
            busyDroppedFrameCount.incrementAndGet()
            return
        }
        lastAcceptedAtNanos.set(now)
        firstAcceptedAtNanos.compareAndSet(0L, now)
        acceptedFrameCount.incrementAndGet()
        val requiredByteCount = requiredBytes.toInt()
        if (rgbaFrameBytes.size != requiredByteCount) {
            rgbaFrameBytes = ByteArray(requiredByteCount)
        }
        val frameBytes = rgbaFrameBytes
        frameData.copyInto(
            destination = frameBytes,
            destinationOffset = 0,
            startIndex = offset,
            endIndex = offset + requiredByteCount,
        )
        try {
            worker.execute {
                try {
                    val result = detect(frameBytes, width, height, now)
                    completedFrameCount.incrementAndGet()
                    if (!closed.get()) {
                        onResult(if (suppressResultsUntilSessionReady) null else result)
                    }
                } catch (error: Throwable) {
                    failedFrameCount.incrementAndGet()
                    if (!closed.get()) onError(error)
                } finally {
                    processing.set(false)
                }
            }
        } catch (error: RuntimeException) {
            processing.set(false)
            failedFrameCount.incrementAndGet()
            if (!closed.get()) onError(error)
        }
    }

    /**
     * Re-runs one stored capture through the live pipeline on the calling thread.
     *
     * This is the offline replay entry point: it exists so a recorded failure can
     * be re-examined against the current build instead of against a screen
     * recording. It must only be used on a detector that is not receiving frames,
     * because [detect] carries this frame's diagnostics and the previous frame's
     * tracking state in fields owned by the detector thread. Replaying a recorded
     * sequence therefore means a detector of its own, fed the captures in order.
     */
    internal fun replay(capture: TapeCapture): TapeDetection? {
        val frame = capture.frame
        require(frame.channels == RGBA_CHANNELS.toInt()) {
            "replay needs an RGBA frame, capture holds ${frame.channels} channels"
        }
        val frameNanos = capture.metadata["frame.acceptedAtNanos"]?.toLongOrNull() ?: 0L
        return detect(frame.pixels, frame.width, frame.height, frameNanos)
    }

    internal fun setDetectionMode(mode: TapeDetectionMode) {
        if (detectionMode == mode) return
        detectionMode = mode
        resetTracking()
    }

    /**
     * Starts one flight-scoped detector session. A recent three-frame preview lock
     * is promoted so the controller receives the same route the operator saw.
     * Without that lock, flight acquisition starts from clean state.
     */
    fun beginTrackingSession(requireConsistentCurve: Boolean = false) {
        if (closed.get()) return
        suppressResultsUntilSessionReady = true
        executeStateChange {
            val previewPromoted =
                promotePreviewBoardReference(
                    requireConsistentCurve = requireConsistentCurve,
                    nowNanos = System.nanoTime(),
                )
            if (!previewPromoted) clearTrackingState()
            trackingSessionActive = true
            requiresConsistentCurveAcquisition = requireConsistentCurve
            clearPreviewBoardReference()
            suppressResultsUntilSessionReady = false
        }
    }

    /** Ends the flight-scoped session and returns the detector to stateless preview. */
    fun endTrackingSession() {
        if (closed.get()) return
        suppressResultsUntilSessionReady = true
        executeStateChange {
            trackingSessionActive = false
            requiresConsistentCurveAcquisition = false
            clearTrackingState()
            suppressResultsUntilSessionReady = false
        }
    }

    fun resetTracking() {
        if (closed.get()) return
        executeStateChange(::clearTrackingState)
    }

    private fun executeStateChange(change: () -> Unit) {
        try {
            worker.execute(change)
        } catch (error: RuntimeException) {
            if (!closed.get()) onError(error)
        }
    }

    private fun clearTrackingState() {
        previousBounds = null
        previousPathMedianWidthFraction = null
        previousRouteStartXFraction = null
        previousRouteStartYFraction = null
        previousCenterlineEstimate = null
        temporalFramesSinceFullExtraction = 0
        clearBoardReferenceState()
        consecutiveDetectionMisses = 0
    }

    private fun clearBoardReferenceState() {
        boardReferenceA = null
        boardReferenceB = null
        clearBoardReferenceAcquisition()
    }

    private fun clearPreviewBoardReference() {
        previewBoardReferenceSamples.clear()
        previewBoardReferenceLastAtNanos = 0L
    }

    fun diagnosticsSummary(): String {
        val accepted = acceptedFrameCount.get()
        val firstAccepted = firstAcceptedAtNanos.get()
        val lastAccepted = lastAcceptedAtNanos.get()
        val acceptedElapsedNanos = lastAccepted - firstAccepted
        val acceptedHz =
            if (accepted >= 2L && acceptedElapsedNanos > 0L) {
                (accepted - 1L) * NANOS_PER_SECOND / acceptedElapsedNanos
            } else {
                0.0
            }
        return (
            "frames=received:%d accepted:%d acceptedHz=%.2f throttle:%d busy:%d invalid:%d " +
                "completed:%d failed:%d mode=%s acquire=%d/%d frameMs=%.1f pathAxis=%s pathSamples=%d " +
                "pathCurve=%.1f pathSmooth=%.2f otsu=%.1f effective=%.1f separation=%.1f " +
                "shadowPixels=%d contours=%d floorSeeds=%d floor=%.2f coverage=%.2f branches=%d " +
                "region=%s candidate=[%s] arcReject=[%s] " +
                "rejects=invalid:%d area:%d width:%d length:%d curve:%d direction:%d " +
                "edge:%d chroma:%d floor:%d boardColor:%d noCenterline:%d noNearField:%d " +
                "branch:%d shortLookahead:%d"
            ).format(
            receivedFrameCount.get(),
            accepted,
            acceptedHz,
            throttledFrameCount.get(),
            busyDroppedFrameCount.get(),
            invalidFrameCount.get(),
            completedFrameCount.get(),
            failedFrameCount.get(),
            lastDetectionMode,
            boardReferenceAcquisitionCount,
            BOARD_REFERENCE_ACQUISITION_SAMPLES,
            lastFrameMillis,
            lastPathAxis,
            lastPathSampleCount,
            lastPathCurvatureDegrees,
            lastPathCurvatureSmoothness,
            lastOtsuThreshold,
            lastEffectiveThreshold,
            lastClassSeparation,
            lastShadowPixelCount,
            lastContourCount,
            lastFloorSeedCount,
            lastFloorFraction,
            lastPathCoverage,
            lastBranchCount,
            lastRegionDescription,
            lastCandidateMetrics,
            lastArcRejection,
            rejectionCounts[TapeCandidateRejection.INVALID_GEOMETRY.ordinal],
            rejectionCounts[TapeCandidateRejection.AREA.ordinal],
            rejectionCounts[TapeCandidateRejection.WIDTH.ordinal],
            rejectionCounts[TapeCandidateRejection.LENGTH.ordinal],
            rejectionCounts[TapeCandidateRejection.CURVATURE.ordinal],
            rejectionCounts[TapeCandidateRejection.DIRECTION_CONTINUITY.ordinal],
            rejectionCounts[TapeCandidateRejection.HORIZONTAL_FRAME_EDGE.ordinal],
            rejectionCounts[TapeCandidateRejection.CHROMA.ordinal],
            rejectionCounts[TapeCandidateRejection.FLOOR_CONTEXT.ordinal],
            rejectionCounts[TapeCandidateRejection.BOARD_COLOR_CONTEXT.ordinal],
            rejectionCounts[TapeCandidateRejection.NO_CENTERLINE.ordinal],
            rejectionCounts[TapeCandidateRejection.NO_NEAR_FIELD_COMPONENT.ordinal],
            rejectionCounts[TapeCandidateRejection.AMBIGUOUS_BRANCH.ordinal],
            rejectionCounts[TapeCandidateRejection.INSUFFICIENT_LOOKAHEAD.ordinal],
        )
    }

    override fun close() {
        if (!closed.compareAndSet(false, true)) return
        try {
            // At most one detection can be queued. Releasing on the same executor
            // keeps native buffers alive until that detection has finished.
            worker.execute(::releasePipelineResources)
            worker.shutdown()
        } catch (_: RuntimeException) {
            if (!processing.get()) releasePipelineResources()
            worker.shutdownNow()
        }
    }
    private fun createPipelineBuffers(): PipelineBuffers =
        PipelineBuffers(
            pathCloseKernel =
                Imgproc.getStructuringElement(
                    Imgproc.MORPH_ELLIPSE,
                    Size(PATH_CLOSE_KERNEL_SIZE, PATH_CLOSE_KERNEL_SIZE),
                ),
            pathGlareBridgeKernels =
                arrayOf(
                    Imgproc.getStructuringElement(
                        Imgproc.MORPH_ELLIPSE,
                        Size(PATH_GLARE_BRIDGE_SHORT_SIZE, PATH_GLARE_BRIDGE_LONG_SIZE),
                    ),
                    Imgproc.getStructuringElement(
                        Imgproc.MORPH_ELLIPSE,
                        Size(PATH_GLARE_BRIDGE_LONG_SIZE, PATH_GLARE_BRIDGE_SHORT_SIZE),
                    ),
                    diagonalGlareBridgeKernel(descending = false),
                    diagonalGlareBridgeKernel(descending = true),
                ),
            pathOpenKernel =
                Imgproc.getStructuringElement(
                    Imgproc.MORPH_ELLIPSE,
                    Size(PATH_OPEN_KERNEL_SIZE, PATH_OPEN_KERNEL_SIZE),
                ),
            straightCloseKernel =
                Imgproc.getStructuringElement(
                    Imgproc.MORPH_ELLIPSE,
                    Size(TAPE_MASK_KERNEL_WIDTH, VERTICAL_CLOSE_KERNEL_HEIGHT),
                ),
            straightOpenKernel =
                Imgproc.getStructuringElement(
                    Imgproc.MORPH_ELLIPSE,
                    Size(TAPE_MASK_KERNEL_WIDTH, VERTICAL_OPEN_KERNEL_HEIGHT),
                ),
        )

    private fun releasePipelineResources() {
        if (pipelineBuffersDelegate.isInitialized()) pipelineBuffers.release()
    }

    /**
     * Runs one frame and, when the recorder is armed, keeps that frame and its
     * masks as replayable evidence. Capture is assembled after the pipeline has
     * finished so a rejected frame is recorded exactly like an accepted one.
     */
    private fun detect(
        rgbaBytes: ByteArray,
        width: Int,
        height: Int,
        frameNanos: Long,
    ): TapeDetection? {
        frameSequence++
        val currentFrameSequence = frameSequence
        val capturing = captureRecorder?.isArmed == true
        capturePlanes = if (capturing) ArrayList() else null
        val startedAtNanos = System.nanoTime()
        lastPreprocessingNanos = 0L
        lastThresholdingNanos = 0L
        lastFloorContextNanos = 0L
        lastMorphologyAndContoursNanos = 0L
        lastCandidateScoringNanos = 0L
        lastCleanupNanos = 0L
        lastCandidateMaskNanos = 0L
        lastCandidateCenterlineNanos = 0L
        lastCandidateAppearanceNanos = 0L
        lastCandidateTemporalNanos = 0L
        lastCenterlineDownsampleNanos = 0L
        lastCenterlineGapFillNanos = 0L
        lastCenterlineDistanceNanos = 0L
        lastCenterlineThinningNanos = 0L
        lastCenterlineRouteNanos = 0L
        lastCenterlineQualityNanos = 0L
        lastCenterlineTopologyNanos = 0L
        lastCandidateMeasurementNanos = 0L
        lastFullCenterlineExtractions = 0
        lastTemporalCenterlineAttempts = 0
        lastTemporalCenterlineAccepts = 0
        val verdict = detectFrame(rgbaBytes, width, height, frameNanos)
        val completedAtNanos = System.nanoTime()
        lastFrameMillis = (completedAtNanos - startedAtNanos) / 1_000_000.0
        try {
            onFrameProfile(
                TapeFrameProfile(
                    sequence = currentFrameSequence,
                    receivedAtNanos = frameNanos,
                    processingStartedAtNanos = startedAtNanos,
                    processingCompletedAtNanos = completedAtNanos,
                    preprocessingNanos = lastPreprocessingNanos,
                    thresholdingNanos = lastThresholdingNanos,
                    floorContextNanos = lastFloorContextNanos,
                    morphologyAndContoursNanos = lastMorphologyAndContoursNanos,
                    candidateScoringNanos = lastCandidateScoringNanos,
                    cleanupNanos = lastCleanupNanos,
                    candidateMaskNanos = lastCandidateMaskNanos,
                    candidateCenterlineNanos = lastCandidateCenterlineNanos,
                    candidateAppearanceNanos = lastCandidateAppearanceNanos,
                    candidateTemporalNanos = lastCandidateTemporalNanos,
                    centerlineDownsampleNanos = lastCenterlineDownsampleNanos,
                    centerlineGapFillNanos = lastCenterlineGapFillNanos,
                    centerlineDistanceNanos = lastCenterlineDistanceNanos,
                    centerlineThinningNanos = lastCenterlineThinningNanos,
                    centerlineRouteNanos = lastCenterlineRouteNanos,
                    centerlineQualityNanos = lastCenterlineQualityNanos,
                    centerlineTopologyNanos = lastCenterlineTopologyNanos,
                    candidateMeasurementNanos = lastCandidateMeasurementNanos,
                    contourCount = lastContourCount,
                    fullCenterlineExtractions = lastFullCenterlineExtractions,
                    temporalCenterlineAttempts = lastTemporalCenterlineAttempts,
                    temporalCenterlineAccepts = lastTemporalCenterlineAccepts,
                    width = width,
                    height = height,
                    detected = verdict != null,
                ),
            )
        } catch (error: Throwable) {
            onError(IllegalStateException("profiling callback failed", error))
        }
        if (capturing) offerCapture(rgbaBytes, width, height, frameNanos, verdict)
        capturePlanes = null
        return verdict
    }

    private fun detectFrame(
        rgbaBytes: ByteArray,
        width: Int,
        height: Int,
        frameNanos: Long,
    ): TapeDetection? {
        val preprocessingStartedAtNanos = System.nanoTime()
        val buffers = pipelineBuffers
        val source = buffers.source
        val resized = buffers.resized
        val rgb = buffers.rgb
        val gray = buffers.gray
        val blurred = buffers.blurred
        val lab = buffers.lab
        val blackMask = buffers.blackMask
        val bridgedBlackMask = buffers.bridgedBlackMask
        val directionalBridgeMask = buffers.directionalBridgeMask
        val floorMask = buffers.floorMask
        val floorMaskWithBorder = buffers.floorMaskWithBorder
        val cleanedBlackMask = buffers.cleanedBlackMask
        val candidateMask = buffers.candidateMask
        val hierarchy = buffers.hierarchy
        val contours = buffers.contours
        val mode = detectionMode
        lastDetectionMode = mode
        // Path diagnostics describe this frame's winner only, so an early return
        // or an all-rejected frame must not keep publishing the previous one.
        lastPathSampleCount = 0
        lastPathAxis = "NONE"
        lastPathCurvatureDegrees = 0.0
        lastPathCurvatureSmoothness = 0.0
        lastCandidateMetrics = "none"
        // Fluorescent glare can cut completely across glossy tape. A tracked PATH
        // combines its normal isotropic close with narrow directional closes. The
        // wider repair is never used for acquisition: without a previous winner it
        // could join unrelated wall or floor edges into a plausible path.
        val closeKernel =
            if (mode.usesPathGeometry) buffers.pathCloseKernel else buffers.straightCloseKernel
        val openKernel =
            if (mode.usesPathGeometry) buffers.pathOpenKernel else buffers.straightOpenKernel
        try {
            rejectionCounts.fill(0)
            lastContourCount = 0
            lastFloorSeedCount = 0
            lastShadowPixelCount = 0
            lastFloorFraction = 0.0
            source.create(height, width, org.opencv.core.CvType.CV_8UC4)
            source.put(0, 0, rgbaBytes)
            val scale = min(1.0, MAX_ANALYSIS_DIMENSION / max(width, height).toDouble())
            val analysis = if (scale < 1.0) {
                Imgproc.resize(
                    source,
                    resized,
                    Size(width * scale, height * scale),
                    0.0,
                    0.0,
                    Imgproc.INTER_AREA,
                )
                resized
            } else {
                source
            }
            Imgproc.cvtColor(analysis, rgb, Imgproc.COLOR_RGBA2RGB)
            Imgproc.cvtColor(rgb, gray, Imgproc.COLOR_RGB2GRAY)
            Imgproc.GaussianBlur(gray, blurred, Size(5.0, 5.0), 0.0)
            lastPreprocessingNanos = System.nanoTime() - preprocessingStartedAtNanos
            val thresholdingStartedAtNanos = System.nanoTime()
            val otsuThreshold = Imgproc.threshold(
                blurred,
                blackMask,
                0.0,
                255.0,
                Imgproc.THRESH_BINARY_INV + Imgproc.THRESH_OTSU,
            )
            val effectiveThreshold = TapeLuminancePolicy.effectiveThreshold(otsuThreshold)
            lastOtsuThreshold = otsuThreshold
            lastEffectiveThreshold = effectiveThreshold
            if (effectiveThreshold < otsuThreshold) {
                Imgproc.threshold(
                    blurred,
                    blackMask,
                    effectiveThreshold,
                    255.0,
                    Imgproc.THRESH_BINARY_INV,
                )
            }
            val separation = classSeparation(blurred, effectiveThreshold)
            lastThresholdingNanos = System.nanoTime() - thresholdingStartedAtNanos
            lastClassSeparation = separation
            if (separation < MIN_CLASS_SEPARATION_LUMINANCE) {
                registerDetectionMiss()
                return null
            }

            val floorContextStartedAtNanos = System.nanoTime()
            Imgproc.cvtColor(rgb, lab, Imgproc.COLOR_RGB2Lab)
            val analysisPixelCount = analysis.cols() * analysis.rows()
            if (labBytes.size != analysisPixelCount * LAB_CHANNELS) {
                labBytes = ByteArray(analysisPixelCount * LAB_CHANNELS)
            }
            if (blackMaskBytes.size != analysisPixelCount) {
                blackMaskBytes = ByteArray(analysisPixelCount)
            }
            lab.get(0, 0, labBytes)
            blackMask.get(0, 0, blackMaskBytes)
            lastShadowPixelCount = removeBoardColoredShadowPixels(blackMask)
            recordCapturePlane("blackMask", blackMask)
            lastFloorSeedCount = buildFloorMask(lab, blackMask, floorMask, floorMaskWithBorder)
            lastFloorFraction =
                Core.countNonZero(floorMask).toDouble() / floorMask.total().coerceAtLeast(1L)
            recordCapturePlane("floorMask", floorMask)
            lastFloorContextNanos = System.nanoTime() - floorContextStartedAtNanos
            val morphologyStartedAtNanos = System.nanoTime()
            Imgproc.morphologyEx(
                blackMask,
                bridgedBlackMask,
                Imgproc.MORPH_CLOSE,
                closeKernel,
            )
            if (mode.usesPathGeometry && previousBounds != null) {
                for (kernel in buffers.pathGlareBridgeKernels) {
                    Imgproc.morphologyEx(
                        blackMask,
                        directionalBridgeMask,
                        Imgproc.MORPH_CLOSE,
                        kernel,
                    )
                    Core.bitwise_or(bridgedBlackMask, directionalBridgeMask, bridgedBlackMask)
                }
            }
            Imgproc.morphologyEx(bridgedBlackMask, cleanedBlackMask, Imgproc.MORPH_OPEN, openKernel)
            recordCapturePlane("bridgedBlackMask", bridgedBlackMask)
            // findContours may consume its input, so the accepted-component mask
            // is recorded while it still describes this frame.
            recordCapturePlane("cleanedBlackMask", cleanedBlackMask)
            Imgproc.findContours(
                cleanedBlackMask,
                contours,
                hierarchy,
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE,
            )
            lastMorphologyAndContoursNanos = System.nanoTime() - morphologyStartedAtNanos
            val candidateScoringStartedAtNanos = System.nanoTime()
            lastContourCount = contours.size

            val frameArea = analysis.cols().toDouble() * analysis.rows()
            val frameShortSide = min(analysis.cols(), analysis.rows()).toDouble()
            var best: Candidate? = null
            var bestSelectionScore = Double.NEGATIVE_INFINITY
            var highestSelectionScore = Double.NEGATIVE_INFINITY
            for ((contourIndex, contour) in contours.withIndex()) {
                val candidate =
                    scoreCandidate(
                        contour,
                        contourIndex,
                        contours,
                        blackMask,
                        candidateMask,
                        hierarchy,
                        rgb,
                        floorMask,
                        frameArea,
                        frameShortSide,
                        mode,
                    ) ?: continue
                if (
                    mode.usesPathGeometry &&
                    !matchesPreviousRouteStart(candidate) &&
                    !overlapsPrevious(candidate.bounds)
                ) {
                    rejectionCounts[TapeCandidateRejection.DIRECTION_CONTINUITY.ordinal] += 1
                    continue
                }
                val continuityBonus =
                    if (overlapsPrevious(candidate.bounds)) PREVIOUS_OVERLAP_BONUS else 1.0
                val selectionScore = candidate.score * continuityBonus
                highestSelectionScore = max(highestSelectionScore, selectionScore)
                val counterclockwiseAcquisition =
                    mode.usesPathGeometry && previousRouteStartXFraction == null
                val shouldSelect = when {
                    best == null -> true
                    bestSelectionScore <
                        highestSelectionScore - DIRECTION_SCORE_TOLERANCE -> true
                    counterclockwiseAcquisition &&
                        selectionScore >=
                        highestSelectionScore - DIRECTION_SCORE_TOLERANCE &&
                        candidate.routeStartXFraction > best.routeStartXFraction -> true
                    !counterclockwiseAcquisition &&
                        selectionScore > bestSelectionScore -> true
                    else -> false
                }
                if (shouldSelect) {
                    best = candidate
                    bestSelectionScore = selectionScore
                }
            }
            val winner = best
            lastCandidateScoringNanos = System.nanoTime() - candidateScoringStartedAtNanos
            if (winner == null) {
                registerDetectionMiss()
                return null
            }
            consecutiveDetectionMisses = 0
            if (trackingSessionActive && boardReferenceA == null) {
                if (!acquireBoardReference(winner)) return null
            }
            if (!trackingSessionActive) {
                recordPreviewBoardReference(winner, frameNanos)
            }
            val rect = winner.bounds
            lastPathCurvatureDegrees = winner.totalPathTurnDegrees
            lastPathCurvatureSmoothness = winner.turnConsistency
            previousBounds = Rect(rect.x, rect.y, rect.width, rect.height)
            previousCenterlineEstimate = winner.estimate
            temporalFramesSinceFullExtraction =
                if (winner.temporallyTracked) temporalFramesSinceFullExtraction + 1 else 0
            previousPathMedianWidthFraction = winner.pathMedianWidthFraction
            previousRouteStartXFraction = winner.routeStartXFraction
            previousRouteStartYFraction = winner.routeStartYFraction
            lastPathAxis = if (winner.temporallyTracked) "TEMPORAL_NORMALS" else "CENTERLINE"
            lastPathSampleCount = winner.pathSampleCount
            val detection = TapeDetection(
                sourceWidth = width,
                sourceHeight = height,
                capturedAtNanos = frameNanos,
                bounds = NormalizedRect(
                    left = rect.x.toDouble() / analysis.cols(),
                    top = rect.y.toDouble() / analysis.rows(),
                    right = (rect.x + rect.width).toDouble() / analysis.cols(),
                    bottom = (rect.y + rect.height).toDouble() / analysis.rows(),
                ),
                confidence = winner.score.coerceIn(0.0, 1.0),
                angleFromVerticalDegrees = winner.angleFromVerticalDegrees,
                longSideFraction = winner.longSideFraction,
                nearFieldOffsetFraction = winner.nearFieldOffsetFraction,
                anchorXFraction = winner.anchorXFraction,
                anchorYFraction = winner.anchorYFraction,
                lookahead = winner.lookahead,
                endpointCandidate = winner.endpointCandidate,
                closedLoop = winner.closedLoop,
                centerline = winner.centerline,
            )
            return detection
        } finally {
            val cleanupStartedAtNanos = System.nanoTime()
            contours.forEach(MatOfPoint::release)
            contours.clear()
            lastCleanupNanos = System.nanoTime() - cleanupStartedAtNanos
        }
    }
    private fun diagonalGlareBridgeKernel(descending: Boolean): Mat {
        val size = PATH_GLARE_BRIDGE_LONG_SIZE.toInt()
        val last = (size - 1).toDouble()
        return Mat.zeros(
            size,
            size,
            org.opencv.core.CvType.CV_8UC1,
        ).also { kernel ->
            Imgproc.line(
                kernel,
                if (descending) Point(0.0, 0.0) else Point(0.0, last),
                if (descending) Point(last, last) else Point(last, 0.0),
                Scalar(255.0),
                PATH_GLARE_BRIDGE_SHORT_SIZE.toInt(),
            )
        }
    }

    private fun buildFloorMask(
        lab: Mat,
        blackMask: Mat,
        floorMask: Mat,
        floorMaskWithBorder: Mat,
    ): Int {
        floorMaskWithBorder.create(
            lab.rows() + 2,
            lab.cols() + 2,
            org.opencv.core.CvType.CV_8UC1,
        )
        floorMaskWithBorder.setTo(Scalar(0.0))
        var acceptedSeeds = 0
        val seedRows = intArrayOf(
            (lab.rows() * 0.92).toInt().coerceIn(0, lab.rows() - 1),
            lab.rows() - 2,
        )
        for (y in seedRows) {
            for (xFraction in FLOOR_SEED_X_FRACTIONS) {
                val x = (lab.cols() * xFraction).toInt().coerceIn(0, lab.cols() - 1)
                if (blackMask.get(y, x)[0] != 0.0) continue
                val filled = Imgproc.floodFill(
                    lab,
                    floorMaskWithBorder,
                    Point(x.toDouble(), y.toDouble()),
                    Scalar(0.0),
                    Rect(),
                    FLOOR_LAB_LOWER_DIFFERENCE,
                    FLOOR_LAB_UPPER_DIFFERENCE,
                    FLOOR_FLOOD_FILL_FLAGS,
                )
                if (filled > 0) acceptedSeeds++
            }
        }
        val interior = floorMaskWithBorder.submat(1, lab.rows() + 1, 1, lab.cols() + 1)
        try {
            interior.copyTo(floorMask)
        } finally {
            interior.release()
        }
        return acceptedSeeds
    }

    /**
     * Removes dark pixels whose chroma still matches the previously observed
     * board. A cast shadow changes L* far more than a* and b*, while black tape does
     * not retain the brown board's chroma. Removing the shadow before morphology
     * prevents its edge from joining the tape into one false branch.
     */
    private fun removeBoardColoredShadowPixels(blackMask: Mat): Int {
        val referenceA = boardReferenceA ?: return 0
        val referenceB = boardReferenceB ?: return 0
        val referenceChromaA = referenceA - LAB_NEUTRAL_CHROMA
        val referenceChromaB = referenceB - LAB_NEUTRAL_CHROMA
        val referenceChromaSquared =
            referenceChromaA * referenceChromaA + referenceChromaB * referenceChromaB
        if (referenceChromaSquared < MIN_BOARD_REFERENCE_CHROMA_SQUARED) return 0
        val maximumShadowChromaSquared =
            (hypot(referenceChromaA, referenceChromaB) + MAX_SHADOW_CHROMA_GAIN).let { it * it }
        var removed = 0
        for (pixel in blackMaskBytes.indices) {
            if (blackMaskBytes[pixel] == 0.toByte()) continue
            val labPixel = pixel * LAB_CHANNELS
            val chromaA =
                (labBytes[labPixel + LAB_A_CHANNEL].toInt() and 0xFF) - LAB_NEUTRAL_CHROMA
            val chromaB =
                (labBytes[labPixel + LAB_B_CHANNEL].toInt() and 0xFF) - LAB_NEUTRAL_CHROMA
            val pixelChromaSquared = chromaA * chromaA + chromaB * chromaB
            val dot = chromaA * referenceChromaA + chromaB * referenceChromaB
            val retainsBoardHue =
                pixelChromaSquared >= MIN_SHADOW_PIXEL_CHROMA_SQUARED &&
                    pixelChromaSquared <= maximumShadowChromaSquared &&
                    dot > 0.0 &&
                    dot * dot >=
                    MIN_BOARD_SHADOW_HUE_COSINE_SQUARED *
                    pixelChromaSquared * referenceChromaSquared
            if (retainsBoardHue) {
                blackMaskBytes[pixel] = 0
                removed++
            }
        }
        if (removed > 0) blackMask.put(0, 0, blackMaskBytes)
        return removed
    }

    private fun isCorrugatedBoardChroma(chromaA: Double, chromaB: Double): Boolean =
        chromaA >= MIN_CORRUGATED_BOARD_CHROMA_A &&
            chromaB >= MIN_CORRUGATED_BOARD_CHROMA_B

    private fun recordCenterlineProfile(profile: CenterlineStageProfile) {
        lastCenterlineDownsampleNanos += profile.downsampleNanos
        lastCenterlineGapFillNanos += profile.gapFillNanos
        lastCenterlineDistanceNanos += profile.distanceNanos
        lastCenterlineThinningNanos += profile.thinningNanos
        lastCenterlineRouteNanos += profile.routeNanos
        lastCenterlineQualityNanos += profile.qualityNanos
        lastCenterlineTopologyNanos += profile.topologyNanos
    }

    /**
     * Scores one contour using the direction-independent centerline as its only
     * geometry source.
     *
     * The cheap image gates stay ahead of it — area, chroma, board context and
     * previous-frame continuity are far cheaper than a skeleton walk, and there
     * is no reason to extract a centerline from a candidate that colour already
     * disqualified. What changed is that the path itself, its topology and its
     * quality now decide acceptance, instead of a row scan that treated vertical
     * tape as more real than horizontal tape.
     */
    private fun scoreCandidate(
        contour: MatOfPoint,
        contourIndex: Int,
        contours: List<MatOfPoint>,
        rawBlackMask: Mat,
        candidateMask: Mat,
        hierarchy: Mat,
        rgb: Mat,
        floorMask: Mat,
        frameArea: Double,
        frameShortSide: Double,
        mode: TapeDetectionMode,
    ): Candidate? {
        val bounds = Imgproc.boundingRect(contour)
        if (bounds.width <= 0 || bounds.height <= 0) {
            rejectionCounts[TapeCandidateRejection.INVALID_GEOMETRY.ordinal] += 1
            return null
        }
        if (Imgproc.contourArea(contour) / frameArea < MIN_COARSE_CONTOUR_AREA_FRACTION) {
            rejectionCounts[TapeCandidateRejection.AREA.ordinal] += 1
            return null
        }
        val candidateMaskStartedAtNanos = System.nanoTime()
        val frameWidth = rawBlackMask.cols()
        val frameHeight = rawBlackMask.rows()
        val region = paddedRegion(bounds, frameWidth, frameHeight)
        candidateMask.create(region.height, region.width, org.opencv.core.CvType.CV_8UC1)
        candidateMask.setTo(Scalar(0.0))
        Imgproc.drawContours(
            candidateMask,
            contours,
            contourIndex,
            Scalar(255.0),
            Imgproc.FILLED,
            Imgproc.LINE_8,
            hierarchy,
            0,
            Point(-region.x.toDouble(), -region.y.toDouble()),
        )
        val regionPixels = region.width * region.height
        if (candidateMaskBytes.size != regionPixels) candidateMaskBytes = ByteArray(regionPixels)
        candidateMask.get(0, 0, candidateMaskBytes)
        val regionMask = ByteSegmentationMask(region.width, region.height, candidateMaskBytes)
        lastRegionDescription =
            region.width.toString() + "x" + region.height + ":" + regionMask.tapePixelCount
        lastCandidateMaskNanos += System.nanoTime() - candidateMaskStartedAtNanos

        val candidateChromaStartedAtNanos = System.nanoTime()
        val rawBlackRegion = rawBlackMask.submat(region)
        val rgbRegion = rgb.submat(region)
        val candidateMeanRgb =
            try {
                Core.bitwise_and(candidateMask, rawBlackRegion, candidateMask)
                Core.mean(rgbRegion, candidateMask)
            } finally {
                rawBlackRegion.release()
                rgbRegion.release()
            }
        val maximumChannel =
            max(candidateMeanRgb.`val`[0], max(candidateMeanRgb.`val`[1], candidateMeanRgb.`val`[2]))
        val minimumChannel =
            min(candidateMeanRgb.`val`[0], min(candidateMeanRgb.`val`[1], candidateMeanRgb.`val`[2]))
        val channelBalance = if (maximumChannel > 0.0) minimumChannel / maximumChannel else 0.0
        lastCandidateAppearanceNanos += System.nanoTime() - candidateChromaStartedAtNanos
        if (channelBalance < MIN_TRACKED_TAPE_CHANNEL_BALANCE) {
            rejectionCounts[TapeCandidateRejection.CHROMA.ordinal] += 1
            return null
        }

        val candidateCenterlineStartedAtNanos = System.nanoTime()
        val previousEstimate = previousCenterlineEstimate
        var usedTemporalTracking =
            mode.usesPathGeometry &&
                temporalFramesSinceFullExtraction < TEMPORAL_FRAMES_BETWEEN_FULL_EXTRACTIONS &&
                previousEstimate != null &&
                overlapsPrevious(bounds) &&
                (!trackingSessionActive || boardReferenceA != null)
        var regionEstimate: CenterlineEstimate? = null
        if (usedTemporalTracking) {
            lastTemporalCenterlineAttempts++
            val temporalStartedAtNanos = System.nanoTime()
            regionEstimate = temporalCenterlineTracker.track(
                mask = regionMask,
                previous = checkNotNull(previousEstimate),
                maskOriginX = region.x,
                maskOriginY = region.y,
            )
            lastCandidateTemporalNanos += System.nanoTime() - temporalStartedAtNanos
        }
        if (regionEstimate == null) {
            usedTemporalTracking = false
            lastFullCenterlineExtractions++
            // drawContours(FILLED) produced a solid external component, so this
            // mask cannot contain a bounded background hole for the extractor to fill.
            regionEstimate = centerlineExtractor.extract(regionMask, fillEnclosedGaps = false)
        }
        var estimate = translateToFrame(regionEstimate, region, frameWidth, frameHeight)
        var measurementStartedAtNanos = System.nanoTime()
        var measurement = CenterlineMeasurement.measure(
            estimate = estimate,
            frameWidth = frameWidth,
            frameHeight = frameHeight,
            trackingTargetYFraction = mode.trackingTargetYFraction,
        )
        var coverage = pathCoverage(measurement, regionMask.tapePixelCount, frameShortSide)
        var verdict = TapePathQualityPolicy.evaluate(
            estimate = estimate,
            measurement = measurement,
            mode = mode,
            frameHeight = frameHeight,
            pathCoverage = coverage,
        )
        lastCandidateMeasurementNanos += System.nanoTime() - measurementStartedAtNanos
        if (
            usedTemporalTracking &&
            (verdict.quality != PathQuality.FULL_PATH ||
                coverage < TapePathQualityPolicy.MIN_PATH_COVERAGE)
        ) {
            usedTemporalTracking = false
            lastFullCenterlineExtractions++
            regionEstimate = centerlineExtractor.extract(regionMask, fillEnclosedGaps = false)
            estimate = translateToFrame(regionEstimate, region, frameWidth, frameHeight)
            measurementStartedAtNanos = System.nanoTime()
            measurement = CenterlineMeasurement.measure(
                estimate = estimate,
                frameWidth = frameWidth,
                frameHeight = frameHeight,
                trackingTargetYFraction = mode.trackingTargetYFraction,
            )
            coverage = pathCoverage(measurement, regionMask.tapePixelCount, frameShortSide)
            verdict = TapePathQualityPolicy.evaluate(
                estimate = estimate,
                measurement = measurement,
                mode = mode,
                frameHeight = frameHeight,
                pathCoverage = coverage,
            )
            lastCandidateMeasurementNanos += System.nanoTime() - measurementStartedAtNanos
        }
        if (usedTemporalTracking) lastTemporalCenterlineAccepts++
        lastCandidateCenterlineNanos += System.nanoTime() - candidateCenterlineStartedAtNanos
        lastPathCoverage = coverage
        lastBranchCount = estimate.topology.branchCount
        val refinedBounds = centerlineBounds(estimate, frameWidth, frameHeight)
        lastCandidateMetrics =
            (
                "#%d bounds=%dx%d@%d,%d anchor=%.3f,%.3f arc=%.3f width=%.3f " +
                    "widthConsistency=%.3f coverage=%.3f"
                ).format(
                    contourIndex,
                    refinedBounds.width,
                    refinedBounds.height,
                    refinedBounds.x,
                    refinedBounds.y,
                    measurement?.anchorXFraction ?: 0.0,
                    measurement?.anchorYFraction ?: 0.0,
                    measurement?.arcLengthFraction ?: 0.0,
                    measurement?.medianWidthFraction ?: 0.0,
                    estimate.components.widthConsistency,
                    coverage,
                )
        // `verdict` was computed together with the selected extraction path
        // above so a temporal failure can fall back to the full skeleton once.
        if (verdict.quality == PathQuality.LOST || measurement == null) {
            rejectionCounts[
                (verdict.rejection ?: TapeCandidateRejection.NO_CENTERLINE).ordinal,
            ] += 1
            return null
        }

        val overlapsPrevious = overlapsPrevious(refinedBounds)
        // Today's multi-panel course exposed dark cardboard joins only a few
        // analysis pixels wide. They can overlap real tape at an intersection
        // and inherit continuity, but their median width is below the smallest
        // genuine tape path retained by the recorded-flight corpus.
        if (measurement.medianWidthFraction < MIN_PATH_MEDIAN_WIDTH_FRACTION) {
            rejectionCounts[TapeCandidateRejection.WIDTH.ordinal] += 1
            return null
        }
        // The arc test exists to stop a wall or floor edge being acquired as
        // curved tape. It guards acquisition only: a path already being tracked
        // has proved itself, and demanding it re-prove curvature every frame
        // throws away the straighter stretches of the very same tape.
        if (
            mode.usesPathGeometry &&
            !overlapsPrevious &&
            !TapePathQualityPolicy.isClosedLoopPath(estimate.topology) &&
            !TapePathQualityPolicy.isCredibleArc(measurement)
        ) {
            rejectionCounts[TapeCandidateRejection.CURVATURE.ordinal] += 1
            // Rejections need their own record: the winner fields describe the
            // accepted path, so without this a frame that accepted nothing says
            // nothing about why.
            lastArcRejection = "turn=%.1f consistency=%.2f arc=%.2f".format(
                measurement.totalPathTurnDegrees,
                measurement.turnConsistency,
                measurement.arcLengthFraction,
            )
            return null
        }

        val pathAreaFraction =
            measurement.arcLengthFraction * measurement.medianWidthFraction *
                frameShortSide * frameShortSide / frameArea
        if (pathAreaFraction !in MIN_PATH_AREA_FRACTION..MAX_PATH_AREA_FRACTION) {
            rejectionCounts[TapeCandidateRejection.AREA.ordinal] += 1
            return null
        }
        val spansFrameWidth =
            refinedBounds.x <= HORIZONTAL_EDGE_MARGIN &&
                refinedBounds.x + refinedBounds.width >=
                floorMask.cols() - HORIZONTAL_EDGE_MARGIN
        if (!overlapsPrevious && spansFrameWidth) {
            rejectionCounts[TapeCandidateRejection.HORIZONTAL_FRAME_EDGE.ordinal] += 1
            return null
        }
        val minimumChannelBalance =
            if (overlapsPrevious) MIN_TRACKED_TAPE_CHANNEL_BALANCE else MIN_TAPE_CHANNEL_BALANCE
        lastCandidateMetrics += " chroma=%.3f".format(channelBalance)
        if (channelBalance < minimumChannelBalance) {
            rejectionCounts[TapeCandidateRejection.CHROMA.ordinal] += 1
            return null
        }
        val candidateAppearanceStartedAtNanos = System.nanoTime()
        val boardColor = boardColorContext(estimate, frameWidth, frameHeight)
        if (boardColor == null) {
            lastCandidateMetrics +=
                " board=none samples=%d/%d".format(
                    lastBoardColorPairCount,
                    lastBoardColorAttemptedPairCount,
                )
        }
        val boardColorHasEnoughSamples =
            boardColor != null &&
                boardColor.pairCount >= BilateralBoardPolicy.MIN_PAIR_COUNT &&
                boardColor.sampleCoverageFraction >=
                BilateralBoardPolicy.MIN_SAMPLE_COVERAGE_FRACTION
        val absoluteBoardSidesMatch =
            boardColor?.let {
                BilateralBoardPolicy.accepts(
                    pairCount = it.pairCount,
                    sampleCoverageFraction = it.sampleCoverageFraction,
                    compatiblePairFraction = it.compatiblePairFraction,
                    leftMatchFraction = it.leftAbsoluteMatchFraction,
                    rightMatchFraction = it.rightAbsoluteMatchFraction,
                    bothSidesMatchFraction = it.bothAbsoluteMatchFraction,
                )
            } == true
        val bilateralBoardSidesMatch =
            boardColor?.let {
                BilateralBoardPolicy.hasCompatibleSides(
                    pairCount = it.pairCount,
                    sampleCoverageFraction = it.sampleCoverageFraction,
                    compatiblePairFraction = it.compatiblePairFraction,
                )
            } == true
        val boardColorStrictlyMatchesReference =
            boardReferenceA != null &&
                boardColor?.let {
                    BilateralBoardPolicy.accepts(
                        pairCount = it.pairCount,
                        sampleCoverageFraction = it.sampleCoverageFraction,
                        compatiblePairFraction = it.compatiblePairFraction,
                        leftMatchFraction = it.leftReferenceMatchFraction,
                        rightMatchFraction = it.rightReferenceMatchFraction,
                        bothSidesMatchFraction = it.bothReferenceMatchFraction,
                    ) &&
                        it.referenceDistance?.let { distance ->
                            distance <= MAX_BOARD_REFERENCE_CHROMA_DISTANCE
                        } == true
                } == true
        // A panel join can mask a few otherwise valid side samples. Once the
        // same route overlaps its previous frame, 65% coverage still means at
        // least 40 agreeing pairs in the recorded 640x360 flight frames.
        val trackedBoardColorMatchesReference =
            overlapsPrevious &&
                boardReferenceA != null &&
                boardColor?.let {
                    BilateralBoardPolicy.acceptsTrackedReference(
                        pairCount = it.pairCount,
                        sampleCoverageFraction = it.sampleCoverageFraction,
                        compatiblePairFraction = it.compatiblePairFraction,
                        leftMatchFraction = it.leftReferenceMatchFraction,
                        rightMatchFraction = it.rightReferenceMatchFraction,
                        bothSidesMatchFraction = it.bothReferenceMatchFraction,
                    ) &&
                        it.referenceDistance?.let { distance ->
                            distance <= MAX_BOARD_REFERENCE_CHROMA_DISTANCE
                        } == true
                } == true
        val boardColorMatchesReference =
            boardColorStrictlyMatchesReference || trackedBoardColorMatchesReference
        val isCorrugatedBoard =
            if (boardReferenceA == null) {
                absoluteBoardSidesMatch
            } else {
                boardColorMatchesReference
            }
        val canLearnBoardReference =
            trackingSessionActive &&
                boardReferenceA == null &&
                bilateralBoardSidesMatch &&
                boardColor?.let(::isPlausibleBoardReference) == true
        if (boardColor != null) {
            lastCandidateMetrics +=
                (
                    " board=cov%.2f pair%.2f L%.2f R%.2f both%.2f/%d " +
                        "ref=%s substrate=%s"
                    ).format(
                    boardColor.sampleCoverageFraction,
                    boardColor.compatiblePairFraction,
                    if (boardReferenceA == null) {
                        boardColor.leftAbsoluteMatchFraction
                    } else {
                        boardColor.leftReferenceMatchFraction
                    },
                    if (boardReferenceA == null) {
                        boardColor.rightAbsoluteMatchFraction
                    } else {
                        boardColor.rightReferenceMatchFraction
                    },
                    if (boardReferenceA == null) {
                        boardColor.bothAbsoluteMatchFraction
                    } else {
                        boardColor.bothReferenceMatchFraction
                    },
                    boardColor.pairCount,
                    boardColor.referenceDistance?.let { "%.1f".format(it) } ?: "none",
                    when {
                        isCorrugatedBoard -> "cardboard"
                        canLearnBoardReference -> "learning"
                        else -> "other"
                    },
                )
        }
        if (
            trackingSessionActive &&
            !isCorrugatedBoard &&
            !canLearnBoardReference
        ) {
            lastCandidateAppearanceNanos += System.nanoTime() - candidateAppearanceStartedAtNanos
            rejectionCounts[TapeCandidateRejection.BOARD_COLOR_CONTEXT.ordinal] += 1
            return null
        }
        if (boardColorHasEnoughSamples && !isCorrugatedBoard && !canLearnBoardReference) {
            rejectionCounts[TapeCandidateRejection.BOARD_COLOR_CONTEXT.ordinal] += 1
            lastCandidateAppearanceNanos += System.nanoTime() - candidateAppearanceStartedAtNanos
            return null
        }
        val context = floorContext(floorMask, refinedBounds)
        val referenceMatchedTrackedPath = overlapsPrevious && boardColorMatchesReference
        val minimumSurroundingFloor = when {
            referenceMatchedTrackedPath -> MIN_REFERENCE_MATCHED_PATH_FLOOR
            overlapsPrevious -> MIN_TRACKED_PATH_SURROUNDING_FLOOR
            else -> MIN_PATH_SURROUNDING_FLOOR
        }
        val minimumSideFloor =
            if (referenceMatchedTrackedPath) {
                MIN_REFERENCE_MATCHED_PATH_FLOOR
            } else {
                MIN_PATH_SIDE_FLOOR
            }
        lastCandidateMetrics +=
            " floor=%.3f side=%.3f".format(
                context.surroundingFraction,
                context.minimumSideFraction,
            )
        if (
            context.surroundingFraction < minimumSurroundingFloor ||
            context.minimumSideFraction < minimumSideFloor
        ) {
            rejectionCounts[TapeCandidateRejection.FLOOR_CONTEXT.ordinal] += 1
            lastCandidateAppearanceNanos += System.nanoTime() - candidateAppearanceStartedAtNanos
            return null
        }
        val minimumPathFraction = when {
            verdict.quality == PathQuality.NEAR_FIELD_ONLY ->
                TapePathQualityPolicy.MIN_NEAR_FIELD_ARC_FRACTION
            overlapsPrevious -> MIN_TRACKED_PATH_FRACTION
            else -> MIN_PATH_FRACTION
        }
        if (measurement.arcLengthFraction < minimumPathFraction) {
            rejectionCounts[TapeCandidateRejection.LENGTH.ordinal] += 1
            lastCandidateAppearanceNanos += System.nanoTime() - candidateAppearanceStartedAtNanos
            return null
        }
        val pathConfidence =
            (measurement.arcLengthFraction / IDEAL_PATH_FRACTION).coerceIn(0.0, 1.0)
        val continuityConfidence = if (overlapsPrevious) 1.0 else 0.5
        val floorConfidence =
            (context.surroundingFraction + context.minimumSideFraction) / 2.0
        val score = (
            floorConfidence * 0.35 +
                estimate.components.widthConsistency * 0.30 +
                pathConfidence * 0.25 +
                continuityConfidence * 0.10
            ).coerceIn(0.0, 1.0)
        val anchorXFraction = measurement.anchorXFraction
        lastCandidateAppearanceNanos += System.nanoTime() - candidateAppearanceStartedAtNanos
        return Candidate(
            contourIndex = contourIndex,
            bounds = refinedBounds,
            score = score,
            angleFromVerticalDegrees = measurement.nearFieldAngleFromVerticalDegrees,
            longSideFraction = measurement.arcLengthFraction,
            nearFieldOffsetFraction = (anchorXFraction - 0.5).coerceIn(-0.5, 0.5),
            anchorXFraction = anchorXFraction,
            anchorYFraction = measurement.anchorYFraction,
            routeStartXFraction = (estimate.points.first().x / frameWidth).coerceIn(0.0, 1.0),
            routeStartYFraction = (estimate.points.first().y / frameHeight).coerceIn(0.0, 1.0),
            lookahead = verdict.lookahead,
            quality = verdict.quality,
            rejection = verdict.rejection,
            pathSampleCount = estimate.points.size,
            pathMedianWidthFraction = measurement.medianWidthFraction,
            lookaheadHeadingChangeDegrees = measurement.lookaheadHeadingChangeDegrees,
            totalPathTurnDegrees = measurement.totalPathTurnDegrees,
            turnConsistency = measurement.turnConsistency,
            endpointCandidate = TapePathQualityPolicy.isEndpointCandidate(estimate.topology, mode),
            closedLoop = estimate.topology.closedLoop,
            branchCount = estimate.topology.branchCount,
            centerline = centerlinePath(
                estimate,
                measurement,
                verdict,
                frameWidth,
                frameHeight,
                mode,
            ),
            boardColor = boardColor,
            estimate = estimate,
            temporallyTracked = usedTemporalTracking,
        )
    }

    /**
     * The share of the candidate's tape pixels the reported chain accounts for,
     * as chain arc length times its own width against the component's area.
     * A ribbon the chain follows end to end scores about one; a junction, where
     * a whole arm lies off the chain, scores visibly less.
     */
    private fun pathCoverage(
        measurement: CenterlinePathMeasurement?,
        componentPixelCount: Int,
        frameShortSide: Double,
    ): Double {
        if (measurement == null || componentPixelCount <= 0) return 0.0
        val chainArea = measurement.arcLengthFraction * frameShortSide *
            measurement.medianWidthFraction * frameShortSide
        return (chainArea / componentPixelCount).coerceIn(0.0, 1.0)
    }

    /** One pixel of background around the contour, so a crop border is not a tape end. */
    private fun paddedRegion(bounds: Rect, frameWidth: Int, frameHeight: Int): Rect {
        val left = (bounds.x - 1).coerceAtLeast(0)
        val top = (bounds.y - 1).coerceAtLeast(0)
        val right = (bounds.x + bounds.width + 1).coerceAtMost(frameWidth)
        val bottom = (bounds.y + bounds.height + 1).coerceAtMost(frameHeight)
        return Rect(left, top, right - left, bottom - top)
    }

    /**
     * Moves a chain extracted from a crop back into frame coordinates and
     * reclassifies its far end against the real frame border.
     */
    private fun translateToFrame(
        estimate: CenterlineEstimate,
        region: Rect,
        frameWidth: Int,
        frameHeight: Int,
    ): CenterlineEstimate {
        if (estimate.points.isEmpty()) return estimate
        val translatedPoints = estimate.points.map { point ->
            CenterlinePoint(
                x = point.x + region.x,
                y = point.y + region.y,
                widthPixels = point.widthPixels,
            )
        }
        val previousX = previousRouteStartXFraction
        val previousY = previousRouteStartYFraction
        val first = translatedPoints.first()
        val last = translatedPoints.last()
        val points =
            if (previousX != null && previousY != null) {
                val firstDeltaX = first.x / frameWidth - previousX
                val firstDeltaY = first.y / frameHeight - previousY
                val lastDeltaX = last.x / frameWidth - previousX
                val lastDeltaY = last.y / frameHeight - previousY
                val firstDistance = firstDeltaX * firstDeltaX + firstDeltaY * firstDeltaY
                val lastDistance = lastDeltaX * lastDeltaX + lastDeltaY * lastDeltaY
                if (lastDistance < firstDistance) translatedPoints.asReversed() else translatedPoints
            } else {
                translatedPoints
            }
        val distal = points.last()
        val borderDistance = min(
            min(distal.x, frameWidth - 1.0 - distal.x),
            min(distal.y, frameHeight - 1.0 - distal.y),
        )
        val margin = max(
            FRAME_BORDER_MARGIN_PIXELS,
            distal.widthPixels * FRAME_BORDER_MARGIN_WIDTH_FACTOR,
        )
        val terminus = when {
            estimate.topology.closedLoop -> CenterlineTerminus.NONE
            borderDistance <= margin -> CenterlineTerminus.AT_FRAME_BORDER
            else -> CenterlineTerminus.INSIDE_FRAME
        }
        return CenterlineEstimate(
            points = points,
            confidence = estimate.confidence,
            components = estimate.components,
            topology = CenterlineTopology(
                distalTerminus = terminus,
                distalBorderDistancePixels = borderDistance,
                branchCount = estimate.topology.branchCount,
                closedLoop = estimate.topology.closedLoop,
            ),
        )
    }

    /** The chain's own extent, which is tighter and truer than the contour box. */
    private fun centerlineBounds(
        estimate: CenterlineEstimate,
        frameWidth: Int,
        frameHeight: Int,
    ): Rect {
        var left = Double.MAX_VALUE
        var top = Double.MAX_VALUE
        var right = -Double.MAX_VALUE
        var bottom = -Double.MAX_VALUE
        // Padded by the local tape width so the box covers the ribbon, not just
        // its medial axis: the board-context ring is measured just outside these
        // bounds, and an unpadded box would put that ring on the tape itself.
        estimate.points.forEach { point ->
            val halfWidth = point.widthPixels / 2.0
            left = min(left, point.x - halfWidth)
            right = max(right, point.x + halfWidth)
            top = min(top, point.y - halfWidth)
            bottom = max(bottom, point.y + halfWidth)
        }
        val boundedLeft = left.coerceIn(0.0, frameWidth - 1.0).toInt()
        val boundedTop = top.coerceIn(0.0, frameHeight - 1.0).toInt()
        val boundedRight = right.coerceIn(boundedLeft + 1.0, frameWidth.toDouble()).toInt()
        val boundedBottom = bottom.coerceIn(boundedTop + 1.0, frameHeight.toDouble()).toInt()
        return Rect(boundedLeft, boundedTop, boundedRight - boundedLeft, boundedBottom - boundedTop)
    }

    private fun centerlinePath(
        estimate: CenterlineEstimate,
        measurement: CenterlinePathMeasurement,
        verdict: TapePathVerdict,
        frameWidth: Int,
        frameHeight: Int,
        mode: TapeDetectionMode,
    ): TapeCenterlinePath {
        val pointCount = estimate.points.size
        val xFractions = FloatArray(pointCount)
        val yFractions = FloatArray(pointCount)
        estimate.points.forEachIndexed { index, point ->
            xFractions[index] = (point.x / frameWidth).toFloat().coerceIn(0f, 1f)
            yFractions[index] = (point.y / frameHeight).toFloat().coerceIn(0f, 1f)
        }
        return TapeCenterlinePath(
            sourceWidth = frameWidth,
            sourceHeight = frameHeight,
            xFractions = xFractions,
            yFractions = yFractions,
            anchorXFraction = measurement.anchorXFraction.toFloat(),
            anchorYFraction = measurement.anchorYFraction.toFloat(),
            lookaheadXFraction = verdict.lookahead?.xFraction?.toFloat(),
            lookaheadYFraction = verdict.lookahead?.yFraction?.toFloat(),
            quality = verdict.quality,
            rejection = verdict.rejection?.name,
            branchCount = estimate.topology.branchCount,
            closedLoop = estimate.topology.closedLoop,
            endpointCandidate = TapePathQualityPolicy.isEndpointCandidate(
                estimate.topology,
                mode = mode,
            ),
        )
    }

    private fun floorContext(floorMask: Mat, bounds: Rect): FloorContext {
        val surround = expand(bounds, floorMask.cols(), floorMask.rows())
        val surroundArea = (surround.area() - bounds.area()).coerceAtLeast(1.0)
        val surroundingFloorPixels =
            (countNonZero(floorMask, surround) - countNonZero(floorMask, bounds)).coerceAtLeast(0)
        val surroundingFraction =
            (surroundingFloorPixels / surroundArea).coerceIn(0.0, 1.0)

        val maximumPadding = max(1, floorMask.cols() / MAX_SIDE_PADDING_DIVISOR)
        val sidePadding =
            min(max(MIN_SURROUND_PADDING, (bounds.width * SIDE_CONTEXT_SCALE).toInt()), maximumPadding)
        val leftWidth = min(sidePadding, bounds.x)
        val rightStart = bounds.x + bounds.width
        val rightWidth = min(sidePadding, floorMask.cols() - rightStart)
        val leftVisible = leftWidth >= MIN_SURROUND_PADDING
        val rightVisible = rightWidth >= MIN_SURROUND_PADDING
        val leftFraction =
            if (!leftVisible) 0.0
            else maskFraction(floorMask, Rect(bounds.x - leftWidth, bounds.y, leftWidth, bounds.height))
        val rightFraction =
            if (!rightVisible) 0.0
            else maskFraction(floorMask, Rect(rightStart, bounds.y, rightWidth, bounds.height))
        val minimumVisibleSideFraction = when {
            !leftVisible && !rightVisible -> 0.0
            !leftVisible -> rightFraction
            !rightVisible -> leftFraction
            else -> min(leftFraction, rightFraction)
        }
        return FloorContext(
            surroundingFraction = surroundingFraction,
            minimumSideFraction = minimumVisibleSideFraction,
        )
    }
    /**
     * Samples CIELAB chroma symmetrically outside the two tape edges. A real strip
     * on one board sees the same material on both sides; a dark board edge sees
     * different materials even when both passed the coarse floor flood-fill.
     *
     * L* is deliberately ignored so a cast shadow or exposure gradient does not
     * turn one physical board into two colours.
     */
    private fun boardColorContext(
        estimate: CenterlineEstimate,
        frameWidth: Int,
        frameHeight: Int,
    ): BoardColorContext? {
        val points = estimate.points
        if (points.size < 2) return null
        val sampleStep = max(1, points.size / MAX_BOARD_COLOR_SAMPLES)
        val referenceA = boardReferenceA
        val referenceB = boardReferenceB
        var attemptedPairCount = 0
        var pairCount = 0
        var compatiblePairCount = 0
        var leftAbsoluteMatchCount = 0
        var rightAbsoluteMatchCount = 0
        var bothAbsoluteMatchCount = 0
        var leftReferenceMatchCount = 0
        var rightReferenceMatchCount = 0
        var bothReferenceMatchCount = 0
        var leftASum = 0.0
        var leftBSum = 0.0
        var rightASum = 0.0
        var rightBSum = 0.0

        for (index in points.indices step sampleStep) {
            val previous = points[max(0, index - BOARD_COLOR_TANGENT_SPAN)]
            val next = points[min(points.lastIndex, index + BOARD_COLOR_TANGENT_SPAN)]
            val tangentX = next.x - previous.x
            val tangentY = next.y - previous.y
            val tangentLength = hypot(tangentX, tangentY)
            if (tangentLength <= 0.0) continue
            attemptedPairCount++

            val point = points[index]
            val normalX = -tangentY / tangentLength
            val normalY = tangentX / tangentLength
            val sampleDistance = max(
                MIN_BOARD_COLOR_SAMPLE_DISTANCE_PIXELS,
                point.widthPixels * BOARD_COLOR_SAMPLE_DISTANCE_WIDTH_FACTOR,
            )
            val leftX = (point.x + normalX * sampleDistance).toInt()
            val leftY = (point.y + normalY * sampleDistance).toInt()
            val rightX = (point.x - normalX * sampleDistance).toInt()
            val rightY = (point.y - normalY * sampleDistance).toInt()
            if (
                leftX !in 0 until frameWidth || leftY !in 0 until frameHeight ||
                rightX !in 0 until frameWidth || rightY !in 0 until frameHeight
            ) {
                continue
            }

            val leftPixel = leftY * frameWidth + leftX
            val rightPixel = rightY * frameWidth + rightX
            if (
                blackMaskBytes[leftPixel] != 0.toByte() ||
                blackMaskBytes[rightPixel] != 0.toByte()
            ) {
                continue
            }
            val leftLab = leftPixel * LAB_CHANNELS
            val rightLab = rightPixel * LAB_CHANNELS
            val leftA = (labBytes[leftLab + LAB_A_CHANNEL].toInt() and 0xFF).toDouble()
            val leftB = (labBytes[leftLab + LAB_B_CHANNEL].toInt() and 0xFF).toDouble()
            val rightA = (labBytes[rightLab + LAB_A_CHANNEL].toInt() and 0xFF).toDouble()
            val rightB = (labBytes[rightLab + LAB_B_CHANNEL].toInt() and 0xFF).toDouble()
            val leftAbsoluteMatch =
                isCorrugatedBoardChroma(
                    leftA - LAB_NEUTRAL_CHROMA,
                    leftB - LAB_NEUTRAL_CHROMA,
                )
            val rightAbsoluteMatch =
                isCorrugatedBoardChroma(
                    rightA - LAB_NEUTRAL_CHROMA,
                    rightB - LAB_NEUTRAL_CHROMA,
                )
            val leftReferenceMatch =
                referenceA != null &&
                    referenceB != null &&
                    hypot(leftA - referenceA, leftB - referenceB) <=
                    MAX_BOARD_REFERENCE_CHROMA_DISTANCE
            val rightReferenceMatch =
                referenceA != null &&
                    referenceB != null &&
                    hypot(rightA - referenceA, rightB - referenceB) <=
                    MAX_BOARD_REFERENCE_CHROMA_DISTANCE

            if (hypot(leftA - rightA, leftB - rightB) <= MAX_BOARD_SIDE_CHROMA_DISTANCE) {
                compatiblePairCount++
            }
            if (leftAbsoluteMatch) leftAbsoluteMatchCount++
            if (rightAbsoluteMatch) rightAbsoluteMatchCount++
            if (leftAbsoluteMatch && rightAbsoluteMatch) bothAbsoluteMatchCount++
            if (leftReferenceMatch) leftReferenceMatchCount++
            if (rightReferenceMatch) rightReferenceMatchCount++
            if (leftReferenceMatch && rightReferenceMatch) bothReferenceMatchCount++
            pairCount++
            leftASum += leftA
            leftBSum += leftB
            rightASum += rightA
            rightBSum += rightB
        }
        lastBoardColorAttemptedPairCount = attemptedPairCount
        lastBoardColorPairCount = pairCount
        if (pairCount == 0 || attemptedPairCount == 0) return null

        val leftA = leftASum / pairCount
        val leftB = leftBSum / pairCount
        val rightA = rightASum / pairCount
        val rightB = rightBSum / pairCount
        val referenceDistance =
            if (referenceA == null || referenceB == null) {
                null
            } else {
                max(
                    hypot(leftA - referenceA, leftB - referenceB),
                    hypot(rightA - referenceA, rightB - referenceB),
                )
            }
        fun matchFraction(count: Int): Double = count.toDouble() / pairCount
        return BoardColorContext(
            pairCount = pairCount,
            sampleCoverageFraction = pairCount.toDouble() / attemptedPairCount,
            compatiblePairFraction = matchFraction(compatiblePairCount),
            leftAbsoluteMatchFraction = matchFraction(leftAbsoluteMatchCount),
            rightAbsoluteMatchFraction = matchFraction(rightAbsoluteMatchCount),
            bothAbsoluteMatchFraction = matchFraction(bothAbsoluteMatchCount),
            leftReferenceMatchFraction = matchFraction(leftReferenceMatchCount),
            rightReferenceMatchFraction = matchFraction(rightReferenceMatchCount),
            bothReferenceMatchFraction = matchFraction(bothReferenceMatchCount),
            meanA = (leftA + rightA) / 2.0,
            meanB = (leftB + rightB) / 2.0,
            referenceDistance = referenceDistance,
        )
    }

    /**
     * Establishes a board reference only from a stable, steerable path. A single
     * candidate can be scenery, so it may seed confirmation but can never move
     * the aircraft or define the session reference by itself.
     */
    private fun acquireBoardReference(candidate: Candidate): Boolean {
        val sample =
            boardReferenceSample(
                candidate = candidate,
                requireConsistentCurve = requiresConsistentCurveAcquisition,
                maximumBranchCount = 0,
            )
                ?: run {
                    clearBoardReferenceAcquisition()
                    return false
                }
        val previous = boardReferenceAcquisitionSamples.lastOrNull()
        if (
            previous != null &&
            !previous.isConsistentWith(sample, requiresConsistentCurveAcquisition)
        ) {
            clearBoardReferenceAcquisition()
        }
        boardReferenceAcquisitionSamples += sample
        boardReferenceAcquisitionCount = boardReferenceAcquisitionSamples.size
        if (boardReferenceAcquisitionSamples.size < BOARD_REFERENCE_ACQUISITION_SAMPLES) {
            return false
        }

        establishBoardReference(boardReferenceAcquisitionSamples)
        return true
    }

    private fun recordPreviewBoardReference(candidate: Candidate, frameNanos: Long) {
        val sample =
            boardReferenceSample(
                candidate = candidate,
                requireConsistentCurve = false,
                maximumBranchCount = 1,
            )
                ?: run {
                    clearPreviewBoardReference()
                    return
                }
        val previous = previewBoardReferenceSamples.lastOrNull()
        if (previous != null && !previous.isConsistentWith(sample, false)) {
            clearPreviewBoardReference()
        }
        previewBoardReferenceSamples += sample
        while (previewBoardReferenceSamples.size > BOARD_REFERENCE_ACQUISITION_SAMPLES) {
            previewBoardReferenceSamples.removeAt(0)
        }
        previewBoardReferenceLastAtNanos = frameNanos
    }

    private fun promotePreviewBoardReference(
        requireConsistentCurve: Boolean,
        nowNanos: Long,
    ): Boolean {
        if (
            previewBoardReferenceSamples.size < BOARD_REFERENCE_ACQUISITION_SAMPLES ||
            previewBoardReferenceLastAtNanos == 0L ||
            nowNanos - previewBoardReferenceLastAtNanos > PREVIEW_REFERENCE_MAX_AGE_NANOS
        ) {
            return false
        }
        if (requireConsistentCurve) {
            val firstCurveDirection =
                previewBoardReferenceSamples.first().curveHeadingChangeDegrees
            if (
                previewBoardReferenceSamples.any {
                    abs(it.curveHeadingChangeDegrees) <
                        MIN_ACQUISITION_CURVE_HEADING_CHANGE_DEGREES ||
                        it.curveHeadingChangeDegrees * firstCurveDirection <= 0.0
                }
            ) {
                return false
            }
        }
        establishBoardReference(previewBoardReferenceSamples)
        return true
    }

    private fun establishBoardReference(samples: List<BoardReferenceAcquisitionSample>) {
        boardReferenceA = samples.map { it.boardA }.sorted()[1]
        boardReferenceB = samples.map { it.boardB }.sorted()[1]
        clearBoardReferenceAcquisition()
    }

    private fun boardReferenceSample(
        candidate: Candidate,
        requireConsistentCurve: Boolean,
        maximumBranchCount: Int,
    ): BoardReferenceAcquisitionSample? {
        val boardColor = candidate.boardColor ?: return null
        val lookahead = candidate.lookahead ?: return null
        val boardColorAccepted =
            BilateralBoardPolicy.hasCompatibleSides(
                pairCount = boardColor.pairCount,
                sampleCoverageFraction = boardColor.sampleCoverageFraction,
                compatiblePairFraction = boardColor.compatiblePairFraction,
            ) && isPlausibleBoardReference(boardColor)
        if (
            candidate.quality != PathQuality.FULL_PATH ||
            candidate.branchCount > maximumBranchCount ||
            candidate.longSideFraction < MIN_PATH_FRACTION ||
            !boardColorAccepted ||
            (
                requireConsistentCurve &&
                    abs(candidate.lookaheadHeadingChangeDegrees) <
                    MIN_ACQUISITION_CURVE_HEADING_CHANGE_DEGREES
                )
        ) {
            return null
        }
        return BoardReferenceAcquisitionSample(
            anchorXFraction = candidate.anchorXFraction,
            anchorYFraction = candidate.anchorYFraction,
            angleFromVerticalDegrees = candidate.angleFromVerticalDegrees,
            lookaheadXFraction = lookahead.xFraction,
            lookaheadYFraction = lookahead.yFraction,
            boardA = boardColor.meanA,
            boardB = boardColor.meanB,
            curveHeadingChangeDegrees = candidate.lookaheadHeadingChangeDegrees,
            totalPathTurnDegrees = candidate.totalPathTurnDegrees,
        )
    }

    private fun isPlausibleBoardReference(boardColor: BoardColorContext): Boolean =
        boardColor.meanA - LAB_NEUTRAL_CHROMA >= MIN_BOARD_REFERENCE_CHROMA_A &&
            boardColor.meanB - LAB_NEUTRAL_CHROMA >= MIN_BOARD_REFERENCE_CHROMA_B

    private fun clearBoardReferenceAcquisition() {
        boardReferenceAcquisitionSamples.clear()
        boardReferenceAcquisitionCount = 0
    }


    private fun maskFraction(mask: Mat, bounds: Rect): Double =
        countNonZero(mask, bounds).toDouble() / bounds.area().coerceAtLeast(1.0)


    private fun classSeparation(blurred: Mat, threshold: Double): Double {
        val pixelCount = blurred.total().toInt()
        if (luminanceBytes.size != pixelCount) luminanceBytes = ByteArray(pixelCount)
        blurred.get(0, 0, luminanceBytes)
        var darkSum = 0L
        var darkCount = 0L
        var lightSum = 0L
        var lightCount = 0L
        for (index in 0 until pixelCount) {
            val luminance = luminanceBytes[index].toInt() and 0xFF
            if (luminance <= threshold) {
                darkSum += luminance
                darkCount++
            } else {
                lightSum += luminance
                lightCount++
            }
        }
        if (darkCount == 0L || lightCount == 0L) return 0.0
        return lightSum.toDouble() / lightCount - darkSum.toDouble() / darkCount
    }

    private fun overlapsPrevious(bounds: Rect): Boolean {
        val previous = previousBounds ?: return false
        val left = max(bounds.x, previous.x)
        val top = max(bounds.y, previous.y)
        val right = min(bounds.x + bounds.width, previous.x + previous.width)
        val bottom = min(bounds.y + bounds.height, previous.y + previous.height)
        if (left >= right || top >= bottom) return false
        val intersection = (right - left).toDouble() * (bottom - top)
        val smallerArea = min(bounds.area(), previous.area()).coerceAtLeast(1.0)
        return intersection / smallerArea >= MIN_PREVIOUS_OVERLAP
    }

    private fun matchesPreviousRouteStart(candidate: Candidate): Boolean {
        val previousX = previousRouteStartXFraction ?: return true
        val previousY = previousRouteStartYFraction ?: return true
        val deltaX = candidate.routeStartXFraction - previousX
        val deltaY = candidate.routeStartYFraction - previousY
        return deltaX * deltaX + deltaY * deltaY <= MAX_ANCHOR_STEP_FRACTION_SQUARED
    }

    /**
     * Copies [mat] into the frame's evidence. Called unconditionally from the
     * pipeline and cheap when capture is off, so the pipeline reads the same
     * whether or not an operator is collecting evidence.
     */
    private fun recordCapturePlane(name: String, mat: Mat) {
        val planes = capturePlanes ?: return
        if (mat.empty() || mat.channels() != 1) return
        val required = (mat.total() * mat.channels()).toInt()
        if (capturePlaneBytes.size < required) capturePlaneBytes = ByteArray(required)
        mat.get(0, 0, capturePlaneBytes)
        planes.add(
            TapeCapturePlane(
                name = name,
                width = mat.cols(),
                height = mat.rows(),
                channels = 1,
                pixels = capturePlaneBytes.copyOf(required),
            ),
        )
    }

    private fun offerCapture(
        rgbaBytes: ByteArray,
        width: Int,
        height: Int,
        frameNanos: Long,
        verdict: TapeDetection?,
    ) {
        val recorder = captureRecorder ?: return
        val planes = capturePlanes ?: return
        val metadata = LinkedHashMap<String, String>()
        metadata["frame.sequence"] = frameSequence.toString()
        metadata["frame.acceptedAtNanos"] = frameNanos.toString()
        metadata["detector.mode"] = lastDetectionMode.name
        metadata["detector.diagnostics"] = diagnosticsSummary()
        metadata["detection"] = if (verdict == null) "none" else "accepted"
        if (verdict != null) {
            metadata["detection.confidence"] = verdict.confidence.toString()
            metadata["detection.angleFromVerticalDegrees"] =
                verdict.angleFromVerticalDegrees.toString()
            metadata["detection.longSideFraction"] = verdict.longSideFraction.toString()
            metadata["detection.nearFieldOffsetFraction"] =
                verdict.nearFieldOffsetFraction.toString()
            metadata["detection.anchorXFraction"] = verdict.anchorXFraction.toString()
            metadata["detection.anchorYFraction"] = verdict.anchorYFraction.toString()
            metadata["detection.lookaheadXFraction"] =
                verdict.lookahead?.xFraction?.toString() ?: "none"
            metadata["detection.lookaheadYFraction"] =
                verdict.lookahead?.yFraction?.toString() ?: "none"
            metadata["detection.quality"] = verdict.quality.name
        }
        // Flight state last so a detector key can never be shadowed by it.
        runCatching { captureFlightContext() }
            .onSuccess { context -> context.forEach { (key, value) -> metadata[key] = value } }
            .onFailure { metadata["flightContext.error"] = it.message ?: it::class.java.simpleName }
        recorder.offer(
            TapeCapture(
                metadata = metadata,
                frame = TapeCapturePlane(
                    name = TapeCapture.FRAME_PLANE_NAME,
                    width = width,
                    height = height,
                    channels = RGBA_CHANNELS.toInt(),
                    pixels = rgbaBytes.copyOf(),
                ),
                masks = planes,
            ),
        )
    }

    private fun registerDetectionMiss() {
        // The frame that first loses a tracked path is the one worth replaying;
        // the recorder reaches back for the run-up that explains it.
        if (consecutiveDetectionMisses == 0 && previousBounds != null) {
            captureRecorder?.trigger("path-lost")
        }
        consecutiveDetectionMisses++
        if (consecutiveDetectionMisses >= PREVIOUS_SELECTION_MISS_LIMIT) {
            previousBounds = null
            previousPathMedianWidthFraction = null
            previousRouteStartXFraction = null
            previousRouteStartYFraction = null
            previousCenterlineEstimate = null
            temporalFramesSinceFullExtraction = 0
        }
    }

    private fun expand(bounds: Rect, frameWidth: Int, frameHeight: Int): Rect {
        val paddingX = max(MIN_SURROUND_PADDING, (bounds.width * SURROUND_SCALE).toInt())
        val paddingY = max(MIN_SURROUND_PADDING, (bounds.height * SURROUND_SCALE).toInt())
        val left = (bounds.x - paddingX).coerceAtLeast(0)
        val top = (bounds.y - paddingY).coerceAtLeast(0)
        val right = (bounds.x + bounds.width + paddingX).coerceAtMost(frameWidth)
        val bottom = (bounds.y + bounds.height + paddingY).coerceAtMost(frameHeight)
        return Rect(left, top, right - left, bottom - top)
    }


    private fun countNonZero(mask: Mat, bounds: Rect): Int {
        val region = mask.submat(bounds)
        return try {
            Core.countNonZero(region)
        } finally {
            region.release()
        }
    }

    private data class Candidate(
        val contourIndex: Int,
        val bounds: Rect,
        val score: Double,
        val angleFromVerticalDegrees: Double,
        val longSideFraction: Double,
        val nearFieldOffsetFraction: Double,
        val anchorXFraction: Double,
        val anchorYFraction: Double,
        val routeStartXFraction: Double,
        val routeStartYFraction: Double,
        val lookahead: TapeLookahead?,
        val quality: PathQuality,
        val rejection: TapeCandidateRejection?,
        val pathSampleCount: Int,
        val pathMedianWidthFraction: Double,
        val lookaheadHeadingChangeDegrees: Double,
        val totalPathTurnDegrees: Double,
        val turnConsistency: Double,
        val endpointCandidate: Boolean,
        val closedLoop: Boolean,
        val branchCount: Int,
        val centerline: TapeCenterlinePath,
        val boardColor: BoardColorContext?,
        val estimate: CenterlineEstimate,
        val temporallyTracked: Boolean,
    )
    private data class FloorContext(
        val surroundingFraction: Double,
        val minimumSideFraction: Double,
    )
    private data class BoardColorContext(
        val pairCount: Int,
        val sampleCoverageFraction: Double,
        val compatiblePairFraction: Double,
        val leftAbsoluteMatchFraction: Double,
        val rightAbsoluteMatchFraction: Double,
        val bothAbsoluteMatchFraction: Double,
        val leftReferenceMatchFraction: Double,
        val rightReferenceMatchFraction: Double,
        val bothReferenceMatchFraction: Double,
        val meanA: Double,
        val meanB: Double,
        val referenceDistance: Double?,
    )
    private data class BoardReferenceAcquisitionSample(
        val anchorXFraction: Double,
        val anchorYFraction: Double,
        val angleFromVerticalDegrees: Double,
        val lookaheadXFraction: Double,
        val lookaheadYFraction: Double,
        val boardA: Double,
        val boardB: Double,
        val curveHeadingChangeDegrees: Double,
        val totalPathTurnDegrees: Double,
    ) {
        fun isConsistentWith(
            other: BoardReferenceAcquisitionSample,
            requiresConsistentCurve: Boolean,
        ): Boolean =
            abs(anchorXFraction - other.anchorXFraction) <= MAX_ACQUISITION_ANCHOR_STEP &&
                abs(anchorYFraction - other.anchorYFraction) <= MAX_ACQUISITION_ANCHOR_STEP &&
                abs(angleFromVerticalDegrees - other.angleFromVerticalDegrees) <=
                MAX_ACQUISITION_ANGLE_STEP_DEGREES &&
                abs(lookaheadXFraction - other.lookaheadXFraction) <=
                MAX_ACQUISITION_LOOKAHEAD_STEP &&
                abs(lookaheadYFraction - other.lookaheadYFraction) <=
                MAX_ACQUISITION_LOOKAHEAD_STEP &&
                hypot(boardA - other.boardA, boardB - other.boardB) <=
                MAX_ACQUISITION_BOARD_CHROMA_STEP &&
                (
                    !requiresConsistentCurve ||
                        (
                            curveHeadingChangeDegrees * other.curveHeadingChangeDegrees > 0.0 &&
                                abs(totalPathTurnDegrees - other.totalPathTurnDegrees) <=
                                MAX_ACQUISITION_PATH_TURN_STEP_DEGREES
                            )
                    )
    }
    private class PipelineBuffers(
        val pathCloseKernel: Mat,
        val pathGlareBridgeKernels: Array<Mat>,
        val pathOpenKernel: Mat,
        val straightCloseKernel: Mat,
        val straightOpenKernel: Mat,
    ) {
        val source = Mat()
        val resized = Mat()
        val rgb = Mat()
        val gray = Mat()
        val blurred = Mat()
        val lab = Mat()
        val blackMask = Mat()
        val bridgedBlackMask = Mat()
        val directionalBridgeMask = Mat()
        val floorMask = Mat()
        val floorMaskWithBorder = Mat()
        val cleanedBlackMask = Mat()
        val candidateMask = Mat()
        val hierarchy = Mat()
        val contours = mutableListOf<MatOfPoint>()

        fun release() {
            contours.forEach(MatOfPoint::release)
            contours.clear()
            source.release()
            resized.release()
            rgb.release()
            gray.release()
            blurred.release()
            lab.release()
            blackMask.release()
            bridgedBlackMask.release()
            directionalBridgeMask.release()
            floorMask.release()
            floorMaskWithBorder.release()
            cleanedBlackMask.release()
            candidateMask.release()
            hierarchy.release()
            pathCloseKernel.release()
            pathGlareBridgeKernels.forEach(Mat::release)
            pathOpenKernel.release()
            straightCloseKernel.release()
            straightOpenKernel.release()
        }
    }


    private companion object {
        const val RGBA_CHANNELS = 4L
        const val LAB_CHANNELS = 3
        const val LAB_A_CHANNEL = 1
        const val LAB_NEUTRAL_CHROMA = 128
        const val MIN_BOARD_REFERENCE_CHROMA_SQUARED = 144.0
        const val MIN_SHADOW_PIXEL_CHROMA_SQUARED = 36.0
        const val MIN_BOARD_SHADOW_HUE_COSINE_SQUARED = 0.8836
        const val MAX_SHADOW_CHROMA_GAIN = 8.0
        const val MIN_CORRUGATED_BOARD_CHROMA_A = 4.0
        const val MIN_CORRUGATED_BOARD_CHROMA_B = 12.0
        // Reference learning uses bilateral agreement plus a deliberately broad
        // warm-paper prior. This admits white-balance-shifted cardboard without
        // teaching the detector that a green or neutral floor is cardboard.
        const val MIN_BOARD_REFERENCE_CHROMA_A = -4.0
        const val MIN_BOARD_REFERENCE_CHROMA_B = 6.0
        const val LAB_B_CHANNEL = 2
        const val MAX_BOARD_COLOR_SAMPLES = 64
        const val BOARD_COLOR_TANGENT_SPAN = 2
        const val MIN_BOARD_COLOR_SAMPLE_DISTANCE_PIXELS = 4.0
        const val BOARD_COLOR_SAMPLE_DISTANCE_WIDTH_FACTOR = 0.75
        const val MAX_BOARD_SIDE_CHROMA_DISTANCE = 24.0
        const val MAX_BOARD_REFERENCE_CHROMA_DISTANCE = 20.0
        const val BOARD_REFERENCE_ACQUISITION_SAMPLES = 3
        const val PREVIEW_REFERENCE_MAX_AGE_NANOS = 3_000_000_000L
        const val MAX_ACQUISITION_ANCHOR_STEP = 0.10
        const val MAX_ACQUISITION_LOOKAHEAD_STEP = 0.12
        const val MAX_ACQUISITION_ANGLE_STEP_DEGREES = 20.0
        const val MAX_ACQUISITION_BOARD_CHROMA_STEP = 12.0
        const val MIN_ACQUISITION_CURVE_HEADING_CHANGE_DEGREES = 4.0
        const val MAX_ACQUISITION_PATH_TURN_STEP_DEGREES = 25.0
        const val MAX_ANALYSIS_DIMENSION = 640.0
        // Pixel 8 Pro flight evidence measured 40-100 ms analysis time. A 50 ms admission
        // window lets the single worker accept the next callback as soon as it is free,
        // instead of adding a second 100 ms throttle after every completed frame.
        const val FRAME_INTERVAL_NANOS = 50_000_000L
        const val TEMPORAL_FRAMES_BETWEEN_FULL_EXTRACTIONS = 3
        private const val NANOS_PER_SECOND = 1_000_000_000.0
        const val SURROUND_SCALE = 0.35
        const val SIDE_CONTEXT_SCALE = 0.35
        const val MAX_SIDE_PADDING_DIVISOR = 8
        const val HORIZONTAL_EDGE_MARGIN = 1
        const val MIN_SURROUND_PADDING = 8
        const val MIN_CLASS_SEPARATION_LUMINANCE = 30.0
        const val TAPE_MASK_KERNEL_WIDTH = 3.0
        const val VERTICAL_OPEN_KERNEL_HEIGHT = 31.0
        const val VERTICAL_CLOSE_KERNEL_HEIGHT = 25.0
        const val PATH_CLOSE_KERNEL_SIZE = 5.0
        const val PATH_GLARE_BRIDGE_SHORT_SIZE = 7.0
        const val PATH_GLARE_BRIDGE_LONG_SIZE = 25.0
        const val PATH_OPEN_KERNEL_SIZE = 3.0
        const val MIN_PATH_AREA_FRACTION = 0.0008
        // Reject sub-pixel noise before skeletonization. The 50% margin stays
        // below the path-area gate, so every geometrically acceptable ribbon
        // still reaches exact centerline measurement.
        const val MIN_COARSE_CONTOUR_AREA_FRACTION = MIN_PATH_AREA_FRACTION * 0.5
        const val MAX_PATH_AREA_FRACTION = 0.18
        const val MIN_PATH_MEDIAN_WIDTH_FRACTION = 0.012
        const val MIN_PATH_SURROUNDING_FLOOR = 0.22
        const val MIN_PATH_SIDE_FLOOR = 0.30
        const val MIN_TRACKED_PATH_SURROUNDING_FLOOR = 0.12
        // Temporal overlap plus bilateral learned-board samples already prove
        // board on both sides. Panel joins can reduce the coarser flood-fill
        // context to zero, so it must not override that stronger evidence.
        const val MIN_REFERENCE_MATCHED_PATH_FLOOR = 0.0

        /**
         * Thinning retracts a border-cut ribbon's medial axis by half its width,
         * so a terminus nearer than that is indistinguishable from truncation.
         */
        const val FRAME_BORDER_MARGIN_PIXELS = 12.0
        const val FRAME_BORDER_MARGIN_WIDTH_FACTOR = 0.75
        const val MIN_PATH_FRACTION = 0.20
        const val MIN_TRACKED_PATH_FRACTION = 0.12
        const val IDEAL_PATH_FRACTION = 0.80
        const val PREVIOUS_OVERLAP_BONUS = 1.35
        const val MIN_TAPE_CHANNEL_BALANCE = 0.32
        // The 2026-08-20 flight measured the same physical tape at 0.327 then
        // 0.313 on consecutive frames under a warm floor reflection. Continuity
        // may absorb that small cast; acquisition retains the stricter gate so a
        // coloured floor feature cannot create a new route.
        const val MIN_TRACKED_TAPE_CHANNEL_BALANCE = 0.30
        const val MIN_PREVIOUS_OVERLAP = 0.20
        const val PREVIOUS_SELECTION_MISS_LIMIT = 8
        const val DIRECTION_SCORE_TOLERANCE = 0.08
        const val MAX_ANCHOR_STEP_FRACTION_SQUARED = 0.09
        val FLOOR_SEED_X_FRACTIONS = doubleArrayOf(0.08, 0.20, 0.35, 0.50, 0.65, 0.80, 0.92)
        val FLOOR_LAB_LOWER_DIFFERENCE = Scalar(24.0, 18.0, 18.0)
        val FLOOR_LAB_UPPER_DIFFERENCE = Scalar(24.0, 18.0, 18.0)
        val FLOOR_FLOOD_FILL_FLAGS =
            4 or (255 shl 8) or Imgproc.FLOODFILL_FIXED_RANGE or Imgproc.FLOODFILL_MASK_ONLY
    }
}
