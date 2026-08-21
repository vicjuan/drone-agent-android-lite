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
import kotlin.math.max
import kotlin.math.min
import kotlin.math.hypot

internal fun acceptsCorrugatedBoard(
    absoluteChromaMatch: Boolean,
    overlapsPreviousRoute: Boolean,
    matchesAcceptedBoardReference: Boolean,
): Boolean =
    absoluteChromaMatch ||
        (overlapsPreviousRoute && matchesAcceptedBoardReference)

/**
 * Finds a dark, tape-shaped region on brown corrugated board. Otsu adapts to
 * exposure, but its threshold is capped so a bright wall cannot make the whole
 * brown board part of the dark class. Geometry, colour context and short-term
 * overlap reject scenery and keep the box on one physical strip.
 */
class BlackTapeDetector internal constructor(
    private val onResult: (TapeDetection?) -> Unit,
    private val onError: (Throwable) -> Unit = {},
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
    private var boardReferenceA: Double? = null
    private var boardReferenceB: Double? = null
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
    @Volatile private var lastPathCoverage = 0.0
    @Volatile private var lastBranchCount = 0
    @Volatile private var lastRegionDescription = "none"
    @Volatile private var lastArcRejection = "none"
    @Volatile private var lastCandidateMetrics = "none"

    private var candidateMaskBytes = ByteArray(0)
    private var centerlineTape = BooleanArray(0)

    // Non-null only while the current frame is being captured. detect() runs on
    // the single detector thread, so a plain field is the whole synchronisation.
    private var capturePlanes: MutableList<TapeCapturePlane>? = null
    private var capturePlaneBytes = ByteArray(0)
    // processing owns this buffer until its worker task finishes, so accepted frames
    // reuse one full-resolution allocation without racing the decoder callback.
    private var rgbaFrameBytes = ByteArray(0)
    private val centerlineExtractor = CenterlineExtractor()
    private var frameSequence = 0L

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
                    if (!closed.get()) onResult(result)
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

    fun resetTracking() {
        if (closed.get()) return
        try {
            worker.execute {
                previousBounds = null
                previousPathMedianWidthFraction = null
                previousRouteStartXFraction = null
                previousRouteStartYFraction = null
                boardReferenceA = null
                boardReferenceB = null
                consecutiveDetectionMisses = 0
            }
        } catch (error: RuntimeException) {
            if (!closed.get()) onError(error)
        }
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
                "completed:%d failed:%d mode=%s frameMs=%.1f pathAxis=%s pathSamples=%d " +
                "pathCurve=%.1f pathSmooth=%.2f otsu=%.1f effective=%.1f separation=%.1f " +
                "shadowPixels=%d contours=%d floorSeeds=%d floor=%.2f coverage=%.2f branches=%d " +
                "region=%s candidate=[%s] arcReject=[%s] " +
                "rejects=invalid:%d area:%d length:%d curve:%d direction:%d " +
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
        if (closed.compareAndSet(false, true)) worker.shutdownNow()
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
        val capturing = captureRecorder?.isArmed == true
        capturePlanes = if (capturing) ArrayList() else null
        val startedAtNanos = System.nanoTime()
        val verdict = detectFrame(rgbaBytes, width, height, frameNanos)
        lastFrameMillis = (System.nanoTime() - startedAtNanos) / 1_000_000.0
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
        val source = Mat(height, width, org.opencv.core.CvType.CV_8UC4)
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
        val closeKernels =
            if (mode == TapeDetectionMode.PATH) {
                val kernels = mutableListOf(
                    Imgproc.getStructuringElement(
                        Imgproc.MORPH_ELLIPSE,
                        Size(PATH_CLOSE_KERNEL_SIZE, PATH_CLOSE_KERNEL_SIZE),
                    ),
                )
                if (previousBounds != null) {
                    kernels += Imgproc.getStructuringElement(
                        Imgproc.MORPH_ELLIPSE,
                        Size(PATH_GLARE_BRIDGE_SHORT_SIZE, PATH_GLARE_BRIDGE_LONG_SIZE),
                    )
                    kernels += Imgproc.getStructuringElement(
                        Imgproc.MORPH_ELLIPSE,
                        Size(PATH_GLARE_BRIDGE_LONG_SIZE, PATH_GLARE_BRIDGE_SHORT_SIZE),
                    )
                    kernels += diagonalGlareBridgeKernel(descending = true)
                    kernels += diagonalGlareBridgeKernel(descending = false)
                }
                kernels
            } else {
                listOf(
                    Imgproc.getStructuringElement(
                        Imgproc.MORPH_RECT,
                        Size(TAPE_MASK_KERNEL_WIDTH, VERTICAL_CLOSE_KERNEL_HEIGHT),
                    ),
                )
            }
        val openKernel = Imgproc.getStructuringElement(
            if (mode == TapeDetectionMode.PATH) Imgproc.MORPH_ELLIPSE else Imgproc.MORPH_RECT,
            if (mode == TapeDetectionMode.PATH) {
                Size(PATH_OPEN_KERNEL_SIZE, PATH_OPEN_KERNEL_SIZE)
            } else {
                Size(TAPE_MASK_KERNEL_WIDTH, VERTICAL_OPEN_KERNEL_HEIGHT)
            },
        )
        val contours = mutableListOf<MatOfPoint>()
        try {
            rejectionCounts.fill(0)
            lastContourCount = 0
            lastFloorSeedCount = 0
            lastShadowPixelCount = 0
            lastFloorFraction = 0.0
            source.put(0, 0, rgbaBytes)
            val scale = min(1.0, MAX_ANALYSIS_DIMENSION / max(width, height).toDouble())
            val analysis = if (scale < 1.0) {
                Imgproc.resize(source, resized, Size(width * scale, height * scale), 0.0, 0.0, Imgproc.INTER_AREA)
                resized
            } else {
                source
            }
            Imgproc.cvtColor(analysis, rgb, Imgproc.COLOR_RGBA2RGB)
            Imgproc.cvtColor(rgb, gray, Imgproc.COLOR_RGB2GRAY)
            Imgproc.GaussianBlur(gray, blurred, Size(5.0, 5.0), 0.0)
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
            lastClassSeparation = separation
            if (separation < MIN_CLASS_SEPARATION_LUMINANCE) {
                registerDetectionMiss()
                return null
            }

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
            Imgproc.morphologyEx(
                blackMask,
                bridgedBlackMask,
                Imgproc.MORPH_CLOSE,
                closeKernels.first(),
            )
            for (kernel in closeKernels.drop(1)) {
                Imgproc.morphologyEx(
                    blackMask,
                    directionalBridgeMask,
                    Imgproc.MORPH_CLOSE,
                    kernel,
                )
                Core.bitwise_or(bridgedBlackMask, directionalBridgeMask, bridgedBlackMask)
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
                        rgb,
                        floorMask,
                        frameArea,
                        frameShortSide,
                        mode,
                    ) ?: continue
                if (
                    mode == TapeDetectionMode.PATH &&
                    !matchesPreviousRouteStart(candidate)
                ) {
                    rejectionCounts[TapeCandidateRejection.DIRECTION_CONTINUITY.ordinal] += 1
                    continue
                }
                val continuityBonus =
                    if (overlapsPrevious(candidate.bounds)) PREVIOUS_OVERLAP_BONUS else 1.0
                val selectionScore = candidate.score * continuityBonus
                highestSelectionScore = max(highestSelectionScore, selectionScore)
                val counterclockwiseAcquisition =
                    mode == TapeDetectionMode.PATH && previousRouteStartXFraction == null
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
            if (winner == null) {
                registerDetectionMiss()
                return null
            }
            consecutiveDetectionMisses = 0
            updateBoardReference(winner.boardColor)
            val rect = winner.bounds
            lastPathCurvatureDegrees = winner.totalPathTurnDegrees
            lastPathCurvatureSmoothness = winner.turnConsistency
            previousBounds = Rect(rect.x, rect.y, rect.width, rect.height)
            previousPathMedianWidthFraction = winner.pathMedianWidthFraction
            previousRouteStartXFraction = winner.routeStartXFraction
            previousRouteStartYFraction = winner.routeStartYFraction
            lastPathAxis = "CENTERLINE"
            lastPathSampleCount = winner.pathSampleCount
            return TapeDetection(
                sourceWidth = width,
                sourceHeight = height,
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
        } finally {
            candidateMask.release()
            contours.forEach(MatOfPoint::release)
            hierarchy.release()
            closeKernels.forEach(Mat::release)
            openKernel.release()
            directionalBridgeMask.release()
            cleanedBlackMask.release()
            floorMaskWithBorder.release()
            floorMask.release()
            bridgedBlackMask.release()
            blackMask.release()
            lab.release()
            blurred.release()
            gray.release()
            rgb.release()
            resized.release()
            source.release()
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
        if (candidateMask.empty() || candidateMask.size() != rawBlackMask.size()) {
            candidateMask.create(rawBlackMask.rows(), rawBlackMask.cols(), org.opencv.core.CvType.CV_8UC1)
        }
        candidateMask.setTo(Scalar(0.0))
        Imgproc.drawContours(candidateMask, contours, contourIndex, Scalar(255.0), Imgproc.FILLED)
        val frameWidth = candidateMask.cols()
        val frameHeight = candidateMask.rows()
        val pixelCount = frameWidth * frameHeight
        if (candidateMaskBytes.size != pixelCount) candidateMaskBytes = ByteArray(pixelCount)
        candidateMask.get(0, 0, candidateMaskBytes)

        // The skeleton walk costs time proportional to the area it is given, and
        // a candidate occupies a small part of the frame. Extracting over its own
        // bounding box instead of the whole frame is the difference between a
        // pipeline that fits the intake interval and one that does not.
        val region = paddedRegion(bounds, frameWidth, frameHeight)
        val regionPixels = region.width * region.height
        if (centerlineTape.size != regionPixels) centerlineTape = BooleanArray(regionPixels)
        for (row in 0 until region.height) {
            val sourceOffset = (region.y + row) * frameWidth + region.x
            val targetOffset = row * region.width
            for (column in 0 until region.width) {
                centerlineTape[targetOffset + column] =
                    candidateMaskBytes[sourceOffset + column] != 0.toByte()
            }
        }
        val regionMask = SegmentationMask(region.width, region.height, centerlineTape)
        lastRegionDescription = region.width.toString() + "x" + region.height + ":" + regionMask.tapePixelCount
        val regionEstimate = centerlineExtractor.extract(regionMask)
        // Branches and loops are local facts and survive the crop. Where the far
        // end sits relative to the *frame* does not, so it is recomputed here
        // rather than read off a topology that only ever saw the crop.
        val estimate = translateToFrame(regionEstimate, region, frameWidth, frameHeight)
        val measurement = CenterlineMeasurement.measure(estimate, frameWidth, frameHeight)
        val coverage = pathCoverage(measurement, regionMask.tapePixelCount, frameShortSide)
        lastPathCoverage = coverage
        lastBranchCount = estimate.topology.branchCount
        val refinedBounds = centerlineBounds(estimate, frameWidth, frameHeight)
        lastCandidateMetrics =
            "#%d bounds=%dx%d@%d,%d anchor=%.3f,%.3f arc=%.3f width=%.3f coverage=%.3f".format(
                contourIndex,
                refinedBounds.width,
                refinedBounds.height,
                refinedBounds.x,
                refinedBounds.y,
                measurement?.anchorXFraction ?: 0.0,
                measurement?.anchorYFraction ?: 0.0,
                measurement?.arcLengthFraction ?: 0.0,
                estimate.components.widthConsistency,
                coverage,
            )
        val verdict = TapePathQualityPolicy.evaluate(
            estimate = estimate,
            measurement = measurement,
            mode = mode,
            frameHeight = frameHeight,
            pathCoverage = coverage,
        )
        if (verdict.quality == PathQuality.LOST || measurement == null) {
            rejectionCounts[
                (verdict.rejection ?: TapeCandidateRejection.NO_CENTERLINE).ordinal,
            ] += 1
            return null
        }

        val overlapsPrevious = overlapsPrevious(refinedBounds)
        // The arc test exists to stop a wall or floor edge being acquired as
        // curved tape. It guards acquisition only: a path already being tracked
        // has proved itself, and demanding it re-prove curvature every frame
        // throws away the straighter stretches of the very same tape.
        if (
            mode == TapeDetectionMode.PATH &&
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
        // Morphological closing bridges glare gaps with nearby floor pixels. Colour must
        // therefore be measured only from pixels that were dark in the original mask.
        Core.bitwise_and(candidateMask, rawBlackMask, candidateMask)
        val candidateMeanRgb = Core.mean(rgb, candidateMask)
        val maximumChannel =
            max(candidateMeanRgb.`val`[0], max(candidateMeanRgb.`val`[1], candidateMeanRgb.`val`[2]))
        val minimumChannel =
            min(candidateMeanRgb.`val`[0], min(candidateMeanRgb.`val`[1], candidateMeanRgb.`val`[2]))
        val channelBalance = if (maximumChannel > 0.0) minimumChannel / maximumChannel else 0.0
        val minimumChannelBalance =
            if (overlapsPrevious) MIN_TRACKED_TAPE_CHANNEL_BALANCE else MIN_TAPE_CHANNEL_BALANCE
        lastCandidateMetrics += " chroma=%.3f".format(channelBalance)
        if (channelBalance < minimumChannelBalance) {
            rejectionCounts[TapeCandidateRejection.CHROMA.ordinal] += 1
            return null
        }
        val boardColor = boardColorContext(estimate, frameWidth, frameHeight)
        val boardColorHasEnoughSamples =
            boardColor != null && boardColor.pairCount >= MIN_BOARD_COLOR_PAIR_COUNT
        val boardColorMatchesReference =
            boardColorHasEnoughSamples &&
                boardColor?.referenceDistance?.let {
                    it <= MAX_BOARD_REFERENCE_CHROMA_DISTANCE
                } == true
        if (boardColor != null) {
            val boardChromaA = boardColor.meanA - LAB_NEUTRAL_CHROMA
            val boardChromaB = boardColor.meanB - LAB_NEUTRAL_CHROMA
            // Acquisition requires the absolute cardboard chroma. Once an overlapping
            // route matches that accepted reference, continuity supplies hysteresis
            // across exposure and white-balance drift.
            val isCorrugatedBoard =
                acceptsCorrugatedBoard(
                    absoluteChromaMatch = isCorrugatedBoardChroma(boardChromaA, boardChromaB),
                    overlapsPreviousRoute = overlapsPrevious,
                    matchesAcceptedBoardReference = boardColorMatchesReference,
                )
            lastCandidateMetrics +=
                " board=%.2f/%d ref=%s substrate=%s".format(
                    boardColor.compatiblePairFraction,
                    boardColor.pairCount,
                    boardColor.referenceDistance?.let { "%.1f".format(it) } ?: "none",
                    if (isCorrugatedBoard) "cardboard" else "other",
                )
            if (
                boardColorHasEnoughSamples &&
                (
                    !isCorrugatedBoard ||
                        boardColor.compatiblePairFraction < MIN_BOARD_COLOR_PAIR_FRACTION ||
                        boardColor.referenceDistance?.let {
                            it > MAX_BOARD_REFERENCE_CHROMA_DISTANCE
                        } == true
                    )
            ) {
                rejectionCounts[TapeCandidateRejection.BOARD_COLOR_CONTEXT.ordinal] += 1
                return null
            }
        }
        val context = floorContext(floorMask, refinedBounds)
        val minimumSurroundingFloor =
            if (overlapsPrevious) MIN_TRACKED_PATH_SURROUNDING_FLOOR else MIN_PATH_SURROUNDING_FLOOR
        val minimumSideFloor =
            if (overlapsPrevious && boardColorMatchesReference) {
                MIN_TRACKED_PATH_SIDE_FLOOR
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
            centerline = centerlinePath(estimate, measurement, verdict, frameWidth, frameHeight),
            boardColor = boardColor,
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
                mode = TapeDetectionMode.PATH,
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
        var pairCount = 0
        var compatiblePairCount = 0
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
            if (hypot(leftA - rightA, leftB - rightB) <= MAX_BOARD_SIDE_CHROMA_DISTANCE) {
                compatiblePairCount++
            }
            pairCount++
            leftASum += leftA
            leftBSum += leftB
            rightASum += rightA
            rightBSum += rightB
        }
        if (pairCount == 0) return null

        val leftA = leftASum / pairCount
        val leftB = leftBSum / pairCount
        val rightA = rightASum / pairCount
        val rightB = rightBSum / pairCount
        val referenceA = boardReferenceA
        val referenceB = boardReferenceB
        val referenceDistance =
            if (referenceA == null || referenceB == null) {
                null
            } else {
                max(
                    hypot(leftA - referenceA, leftB - referenceB),
                    hypot(rightA - referenceA, rightB - referenceB),
                )
            }
        return BoardColorContext(
            pairCount = pairCount,
            compatiblePairFraction = compatiblePairCount.toDouble() / pairCount,
            meanA = (leftA + rightA) / 2.0,
            meanB = (leftB + rightB) / 2.0,
            referenceDistance = referenceDistance,
        )
    }

    private fun updateBoardReference(context: BoardColorContext?) {
        if (
            context == null ||
            context.pairCount < MIN_BOARD_COLOR_PAIR_COUNT ||
            context.compatiblePairFraction < MIN_BOARD_COLOR_PAIR_FRACTION
        ) {
            return
        }
        val previousA = boardReferenceA
        val previousB = boardReferenceB
        if (previousA == null || previousB == null) {
            boardReferenceA = context.meanA
            boardReferenceB = context.meanB
            return
        }
        boardReferenceA = previousA + BOARD_REFERENCE_FILTER_ALPHA * (context.meanA - previousA)
        boardReferenceB = previousB + BOARD_REFERENCE_FILTER_ALPHA * (context.meanB - previousB)
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
                    pixels = rgbaBytes,
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
    )
    private data class FloorContext(
        val surroundingFraction: Double,
        val minimumSideFraction: Double,
    )
    private data class BoardColorContext(
        val pairCount: Int,
        val compatiblePairFraction: Double,
        val meanA: Double,
        val meanB: Double,
        val referenceDistance: Double?,
    )


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
        const val LAB_B_CHANNEL = 2
        const val MAX_BOARD_COLOR_SAMPLES = 64
        const val BOARD_COLOR_TANGENT_SPAN = 2
        const val MIN_BOARD_COLOR_PAIR_COUNT = 10
        const val MIN_BOARD_COLOR_SAMPLE_DISTANCE_PIXELS = 4.0
        const val BOARD_COLOR_SAMPLE_DISTANCE_WIDTH_FACTOR = 0.75
        const val MAX_BOARD_SIDE_CHROMA_DISTANCE = 24.0
        const val MIN_BOARD_COLOR_PAIR_FRACTION = 0.70
        const val MAX_BOARD_REFERENCE_CHROMA_DISTANCE = 20.0
        const val BOARD_REFERENCE_FILTER_ALPHA = 0.15
        const val MAX_ANALYSIS_DIMENSION = 640.0
        // At 0.24 m/s, 10 Hz limits open-loop travel between observations to about 2.4 cm.
        const val FRAME_INTERVAL_NANOS = 100_000_000L
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
        const val MAX_PATH_AREA_FRACTION = 0.18
        const val MIN_PATH_SURROUNDING_FLOOR = 0.22
        const val MIN_PATH_SIDE_FLOOR = 0.30
        // A tracked path may cross a floor-material boundary, and glare can make
        // flood-fill miss every pixel in one side window. Once overlap and CIELAB
        // confirm the established route, keep only the surrounding-floor gate;
        // acquisition remains strict, and continuity still expires after bounded misses.
        const val MIN_TRACKED_PATH_SURROUNDING_FLOOR = 0.12
        const val MIN_TRACKED_PATH_SIDE_FLOOR = 0.0

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
