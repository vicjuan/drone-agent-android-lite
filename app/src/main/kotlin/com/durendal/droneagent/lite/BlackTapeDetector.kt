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
    // Shadow comparison of the replacement centerline against the estimator that
    // flies today. Temporary by design: it exists to justify the cutover and is
    // deleted with the old estimator, never kept as a permanent second track.
    private val onShadowComparison: ((String) -> Unit)? = null,
) : AutoCloseable {

    private val worker = Executors.newSingleThreadExecutor { runnable ->
        Thread(runnable, "BlackTapeDetector").apply { isDaemon = true }
    }
    private val processing = AtomicBoolean(false)
    private val closed = AtomicBoolean(false)
    private val lastAcceptedAtNanos = AtomicLong(0L)
    private var previousBounds: Rect? = null
    private var previousPathMedianWidthFraction: Double? = null
    private var previousAnchorXFraction: Double? = null
    private var previousAnchorYFraction: Double? = null
    @Volatile private var detectionMode = TapeDetectionMode.PATH
    private var consecutiveDetectionMisses = 0
    @Volatile private var lastOtsuThreshold = 0.0
    @Volatile private var lastEffectiveThreshold = 0.0
    @Volatile private var lastClassSeparation = 0.0
    @Volatile private var lastContourCount = 0
    private val rejectionCounts = IntArray(TapeCandidateRejection.entries.size)
    private var luminanceBytes = ByteArray(0)
    @Volatile private var lastDetectionMode = TapeDetectionMode.PATH
    @Volatile private var lastPathSampleCount = 0
    @Volatile private var lastPathAxis = "NONE"
    @Volatile private var lastPathCurvatureDegrees = 0.0
    @Volatile private var lastPathCurvatureSmoothness = 0.0
    @Volatile private var lastFloorSeedCount = 0
    @Volatile private var lastFloorFraction = 0.0
    private val pathDirectionEstimator = TapePathDirectionEstimator()
    private var candidateMaskBytes = ByteArray(0)

    // Non-null only while the current frame is being captured. detect() runs on
    // the single detector thread, so a plain field is the whole synchronisation.
    private var capturePlanes: MutableList<TapeCapturePlane>? = null
    private var capturePlaneBytes = ByteArray(0)
    private val centerlineExtractor = CenterlineExtractor()
    private var shadowMaskBytes = ByteArray(0)
    private var shadowTape = BooleanArray(0)
    private var pendingShadowLine: String? = null
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
        if (width <= 0 || height <= 0 || offset < 0 || length <= 0) return
        val requiredBytes = width.toLong() * height * RGBA_CHANNELS
        if (requiredBytes > length || offset.toLong() + requiredBytes > frameData.size) return
        val now = System.nanoTime()
        val previous = lastAcceptedAtNanos.get()
        if (now - previous < FRAME_INTERVAL_NANOS || !processing.compareAndSet(false, true)) return
        lastAcceptedAtNanos.set(now)
        val copy = frameData.copyOfRange(offset, offset + requiredBytes.toInt())
        try {
            worker.execute {
                try {
                    val result = detect(copy, width, height, now)
                    if (!closed.get()) onResult(result)
                } catch (error: Throwable) {
                    if (!closed.get()) onError(error)
                } finally {
                    processing.set(false)
                }
            }
        } catch (error: RuntimeException) {
            processing.set(false)
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
                previousAnchorXFraction = null
                previousAnchorYFraction = null
                consecutiveDetectionMisses = 0
            }
        } catch (error: RuntimeException) {
            if (!closed.get()) onError(error)
        }
    }

    fun diagnosticsSummary(): String =
        (
            "mode=%s pathAxis=%s pathSamples=%d pathCurve=%.1f pathSmooth=%.2f " +
                "otsu=%.1f effective=%.1f separation=%.1f contours=%d floorSeeds=%d floor=%.2f " +
                "rejects=invalid:%d area:%d length:%d curve:%d direction:%d " +
                "edge:%d chroma:%d floor:%d"
            ).format(
            lastDetectionMode,
            lastPathAxis,
            lastPathSampleCount,
            lastPathCurvatureDegrees,
            lastPathCurvatureSmoothness,
            lastOtsuThreshold,
            lastEffectiveThreshold,
            lastClassSeparation,
            lastContourCount,
            lastFloorSeedCount,
            lastFloorFraction,
            rejectionCounts[TapeCandidateRejection.INVALID_GEOMETRY.ordinal],
            rejectionCounts[TapeCandidateRejection.AREA.ordinal],
            rejectionCounts[TapeCandidateRejection.LENGTH.ordinal],
            rejectionCounts[TapeCandidateRejection.CURVATURE.ordinal],
            rejectionCounts[TapeCandidateRejection.DIRECTION_CONTINUITY.ordinal],
            rejectionCounts[TapeCandidateRejection.HORIZONTAL_FRAME_EDGE.ordinal],
            rejectionCounts[TapeCandidateRejection.CHROMA.ordinal],
            rejectionCounts[TapeCandidateRejection.FLOOR_CONTEXT.ordinal],
        )

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
        pendingShadowLine = null
        val startedAtNanos = System.nanoTime()
        val verdict = detectFrame(rgbaBytes, width, height, frameNanos)
        val frameNanosElapsed = System.nanoTime() - startedAtNanos
        if (capturing) offerCapture(rgbaBytes, width, height, frameNanos, verdict)
        capturePlanes = null
        // Emitted here, not inside the pipeline, because only the caller knows
        // what the whole frame cost — which is the number the 250 ms intake
        // interval is actually spent against.
        pendingShadowLine?.let { line ->
            onShadowComparison?.invoke("$line frameMs=%.1f".format(frameNanosElapsed / 1_000_000.0))
        }
        pendingShadowLine = null
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
            recordCapturePlane("blackMask", blackMask)
            val separation = classSeparation(blurred, effectiveThreshold)
            lastClassSeparation = separation
            if (separation < MIN_CLASS_SEPARATION_LUMINANCE) {
                registerDetectionMiss()
                return null
            }

            Imgproc.cvtColor(rgb, lab, Imgproc.COLOR_RGB2Lab)
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
                    !matchesPreviousAnchor(candidate)
                ) {
                    rejectionCounts[TapeCandidateRejection.DIRECTION_CONTINUITY.ordinal] += 1
                    continue
                }
                val continuityBonus =
                    if (overlapsPrevious(candidate.bounds)) PREVIOUS_OVERLAP_BONUS else 1.0
                val selectionScore = candidate.score * continuityBonus
                highestSelectionScore = max(highestSelectionScore, selectionScore)
                val counterclockwiseAcquisition =
                    mode == TapeDetectionMode.PATH && previousAnchorXFraction == null
                val shouldSelect = when {
                    best == null -> true
                    bestSelectionScore <
                        highestSelectionScore - DIRECTION_SCORE_TOLERANCE -> true
                    counterclockwiseAcquisition &&
                        selectionScore >=
                        highestSelectionScore - DIRECTION_SCORE_TOLERANCE &&
                        candidate.anchorXFraction > best.anchorXFraction -> true
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
            val rect = winner.bounds
            lastPathCurvatureDegrees = winner.pathCurvatureDegrees
            lastPathCurvatureSmoothness = winner.pathCurvatureSmoothness
            previousBounds = Rect(rect.x, rect.y, rect.width, rect.height)
            previousPathMedianWidthFraction = winner.pathMedianWidthFraction
            previousAnchorXFraction = winner.anchorXFraction
            previousAnchorYFraction = winner.anchorYFraction
            lastPathAxis = if (winner.horizontalFallback) "HORIZONTAL_FALLBACK" else "CENTERLINE"
            lastPathSampleCount = winner.pathSampleCount
            runShadowComparison(winner, contours, candidateMask, frameNanos)
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
                lookahead = TapeLookahead(
                    xFraction = winner.lookaheadXFraction,
                    yFraction = winner.lookaheadYFraction,
                ),
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

    /** Prefers the trace that follows more tape at a more consistent width; vertical wins a tie. */
    private fun betterPath(
        verticalPath: TapePathEstimate?,
        horizontalPath: TapePathEstimate?,
    ): TapePathEstimate? {
        if (verticalPath == null) return horizontalPath
        if (horizontalPath == null) return verticalPath
        val verticalQuality = verticalPath.arcLengthFraction * verticalPath.widthConsistency
        val horizontalQuality = horizontalPath.arcLengthFraction * horizontalPath.widthConsistency
        return if (horizontalQuality > verticalQuality) horizontalPath else verticalPath
    }

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
        val requireCurvature = mode == TapeDetectionMode.PATH
        if (candidateMask.empty() || candidateMask.size() != rawBlackMask.size()) {
            candidateMask.create(rawBlackMask.rows(), rawBlackMask.cols(), org.opencv.core.CvType.CV_8UC1)
        }
        candidateMask.setTo(Scalar(0.0))
        Imgproc.drawContours(candidateMask, contours, contourIndex, Scalar(255.0), Imgproc.FILLED)
        val pixelCount = candidateMask.total().toInt()
        if (candidateMaskBytes.size != pixelCount) candidateMaskBytes = ByteArray(pixelCount)
        candidateMask.get(0, 0, candidateMaskBytes)
        // In the camera-down image, entering a rainbow at its right endpoint and
        // tracing toward the left follows the circle counterclockwise.
        val verticalPath = pathDirectionEstimator.estimateVerticalPath(
            mask = candidateMaskBytes,
            frameWidth = candidateMask.cols(),
            frameHeight = candidateMask.rows(),
            left = bounds.x,
            top = bounds.y,
            right = bounds.x + bounds.width,
            bottom = bounds.y + bounds.height,
            initialCenterHint =
                previousAnchorXFraction?.times(candidateMask.cols()),
            preferRightmostInitialRun =
                requireCurvature && previousAnchorXFraction == null,
            expectedMedianWidthFraction = previousPathMedianWidthFraction,
        )
        val horizontalPath = pathDirectionEstimator.estimateHorizontalFallback(
            mask = candidateMaskBytes,
            frameWidth = candidateMask.cols(),
            frameHeight = candidateMask.rows(),
            left = bounds.x,
            top = bounds.y,
            right = bounds.x + bounds.width,
            bottom = bounds.y + bounds.height,
            expectedMedianWidthFraction = previousPathMedianWidthFraction,
            preferredNearFieldX =
                previousAnchorXFraction?.times(candidateMask.cols()),
            preferredNearFieldY =
                previousAnchorYFraction?.times(candidateMask.rows()),
            preferRightToLeft =
                requireCurvature && previousAnchorXFraction == null,
        )
        val path = betterPath(verticalPath, horizontalPath)
        if (path == null) {
            rejectionCounts[TapeCandidateRejection.LENGTH.ordinal] += 1
            return null
        }
        // A wall or floor edge can acquire an apparent bend at one noisy endpoint.
        // Circular tape must change direction across several path segments.
        if (
            requireCurvature &&
            (
                path.curvatureDegrees < MIN_PATH_CURVATURE_DEGREES ||
                    path.curvatureSmoothness < MIN_PATH_CURVATURE_SMOOTHNESS
                )
        ) {
            rejectionCounts[TapeCandidateRejection.CURVATURE.ordinal] += 1
            return null
        }
        val pathBounds = path.bounds
        val refinedBounds = Rect(
            pathBounds.left,
            pathBounds.top,
            pathBounds.right - pathBounds.left,
            pathBounds.bottom - pathBounds.top,
        )
        val pathAreaFraction =
            path.arcLengthFraction * path.medianWidthFraction *
                frameShortSide * frameShortSide / frameArea
        if (pathAreaFraction !in MIN_PATH_AREA_FRACTION..MAX_PATH_AREA_FRACTION) {
            rejectionCounts[TapeCandidateRejection.AREA.ordinal] += 1
            return null
        }
        val overlapsPrevious = overlapsPrevious(refinedBounds)
        val spansFrameWidth =
            refinedBounds.x <= HORIZONTAL_EDGE_MARGIN &&
                refinedBounds.x + refinedBounds.width >=
                floorMask.cols() - HORIZONTAL_EDGE_MARGIN
        if (!overlapsPrevious && spansFrameWidth && !path.horizontalFallback) {
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
        if (channelBalance < MIN_TAPE_CHANNEL_BALANCE) {
            rejectionCounts[TapeCandidateRejection.CHROMA.ordinal] += 1
            return null
        }
        val context = floorContext(floorMask, refinedBounds)
        val minimumSurroundingFloor = when {
            path.horizontalFallback -> MIN_HORIZONTAL_PATH_SURROUNDING_FLOOR
            overlapsPrevious -> MIN_TRACKED_PATH_SURROUNDING_FLOOR
            else -> MIN_PATH_SURROUNDING_FLOOR
        }
        val minimumSideFloor = when {
            path.horizontalFallback -> MIN_HORIZONTAL_PATH_SIDE_FLOOR
            overlapsPrevious -> MIN_TRACKED_PATH_SIDE_FLOOR
            else -> MIN_PATH_SIDE_FLOOR
        }
        if (
            context.surroundingFraction < minimumSurroundingFloor ||
            context.minimumSideFraction < minimumSideFloor
        ) {
            rejectionCounts[TapeCandidateRejection.FLOOR_CONTEXT.ordinal] += 1
            return null
        }
        val minimumPathFraction =
            if (overlapsPrevious) MIN_TRACKED_PATH_FRACTION else MIN_PATH_FRACTION
        if (path.arcLengthFraction < minimumPathFraction) {
            rejectionCounts[TapeCandidateRejection.LENGTH.ordinal] += 1
            return null
        }
        val pathConfidence =
            (path.arcLengthFraction / IDEAL_PATH_FRACTION).coerceIn(0.0, 1.0)
        val continuityConfidence = if (overlapsPrevious) 1.0 else 0.5
        val floorConfidence =
            (context.surroundingFraction + context.minimumSideFraction) / 2.0
        val score = (
            floorConfidence * 0.35 +
                path.widthConsistency * 0.30 +
                pathConfidence * 0.25 +
                continuityConfidence * 0.10
            ).coerceIn(0.0, 1.0)
        return Candidate(
            contourIndex = contourIndex,
            bounds = refinedBounds,
            score = score,
            angleFromVerticalDegrees = path.nearFieldAngleFromVerticalDegrees,
            longSideFraction = path.arcLengthFraction,
            nearFieldOffsetFraction =
                (path.nearFieldCenterX / floorMask.cols() - 0.5).coerceIn(-0.5, 0.5),
            anchorXFraction = (path.nearFieldCenterX / floorMask.cols()).coerceIn(0.0, 1.0),
            anchorYFraction = (path.nearFieldCenterY / floorMask.rows()).coerceIn(0.0, 1.0),
            lookaheadXFraction = (path.lookaheadCenterX / floorMask.cols()).coerceIn(0.0, 1.0),
            lookaheadYFraction = (path.lookaheadCenterY / floorMask.rows()).coerceIn(0.0, 1.0),
            pathSampleCount = path.sampleCount,
            pathMedianWidthFraction = path.medianWidthFraction,
            horizontalFallback = path.horizontalFallback,
            pathCurvatureDegrees = path.curvatureDegrees,
            pathCurvatureSmoothness = path.curvatureSmoothness,
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
        val leftFraction =
            if (leftWidth == 0) 0.0
            else maskFraction(floorMask, Rect(bounds.x - leftWidth, bounds.y, leftWidth, bounds.height))
        val rightFraction =
            if (rightWidth == 0) 0.0
            else maskFraction(floorMask, Rect(rightStart, bounds.y, rightWidth, bounds.height))
        val minimumVisibleSideFraction = when {
            leftWidth == 0 -> rightFraction
            rightWidth == 0 -> leftFraction
            else -> min(leftFraction, rightFraction)
        }
        return FloorContext(
            surroundingFraction = surroundingFraction,
            minimumSideFraction = minimumVisibleSideFraction,
        )
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

    private fun matchesPreviousAnchor(candidate: Candidate): Boolean {
        val previousX = previousAnchorXFraction ?: return true
        val previousY = previousAnchorYFraction ?: return true
        val deltaX = candidate.anchorXFraction - previousX
        val deltaY = candidate.anchorYFraction - previousY
        return deltaX * deltaX + deltaY * deltaY <= MAX_ANCHOR_STEP_FRACTION_SQUARED
    }

    /**
     * Copies [mat] into the frame's evidence. Called unconditionally from the
     * pipeline and cheap when capture is off, so the pipeline reads the same
     * whether or not an operator is collecting evidence.
     */
    /**
     * Runs the replacement centerline over the winner's own mask and logs both
     * results against one frame identifier.
     *
     * The mask is rebuilt from the winning contour rather than reusing the last
     * candidate scored, because the last candidate scored is whichever contour
     * happened to come last, not the one that won. Nothing here can influence
     * the returned detection: this is the evidence that justifies the cutover,
     * not a second path into the controller.
     */
    private fun runShadowComparison(
        winner: Candidate,
        contours: List<MatOfPoint>,
        candidateMask: Mat,
        frameNanos: Long,
    ) {
        if (onShadowComparison == null) return
        val startedAtNanos = System.nanoTime()
        candidateMask.setTo(Scalar(0.0))
        Imgproc.drawContours(candidateMask, contours, winner.contourIndex, Scalar(255.0), Imgproc.FILLED)
        val width = candidateMask.cols()
        val height = candidateMask.rows()
        val pixelCount = width * height
        if (shadowMaskBytes.size < pixelCount) shadowMaskBytes = ByteArray(pixelCount)
        // SegmentationMask requires an exactly sized array and never copies it,
        // so the scratch array is reallocated only when the analysis resolution
        // changes and is wrapped fresh each frame.
        if (shadowTape.size != pixelCount) shadowTape = BooleanArray(pixelCount)
        candidateMask.get(0, 0, shadowMaskBytes)
        for (index in 0 until pixelCount) shadowTape[index] = shadowMaskBytes[index] != 0.toByte()
        val maskNanos = System.nanoTime() - startedAtNanos

        val extractStartedAtNanos = System.nanoTime()
        // A fresh wrapper each frame: SegmentationMask counts its tape pixels
        // once at construction, so a reused instance would keep reporting the
        // count it was built with and the extractor would see an empty mask.
        val estimate = centerlineExtractor.extract(SegmentationMask(width, height, shadowTape))
        val extractNanos = System.nanoTime() - extractStartedAtNanos
        val measureStartedAtNanos = System.nanoTime()
        val measurement = CenterlineMeasurement.measure(estimate, width, height)
        val measureNanos = System.nanoTime() - measureStartedAtNanos

        pendingShadowLine =
            formatShadowComparison(
                frameSequence = frameSequence,
                frameNanos = frameNanos,
                oldAnchorXFraction = winner.anchorXFraction,
                oldAnchorYFraction = winner.anchorYFraction,
                oldLookaheadXFraction = winner.lookaheadXFraction,
                oldLookaheadYFraction = winner.lookaheadYFraction,
                oldNearFieldAngleDegrees = winner.angleFromVerticalDegrees,
                oldArcLengthFraction = winner.longSideFraction,
                oldMedianWidthFraction = winner.pathMedianWidthFraction,
                oldCurvatureDegrees = winner.pathCurvatureDegrees,
                oldHorizontalFallback = winner.horizontalFallback,
                oldSampleCount = winner.pathSampleCount,
                newPointCount = estimate.points.size,
                newConfidence = estimate.confidence,
                newSupport = estimate.components.support,
                newWidthConsistency = estimate.components.widthConsistency,
                newContinuity = estimate.components.continuity,
                newFitResidual = estimate.components.fitResidual,
                newTerminus = estimate.topology.distalTerminus,
                newDistalBorderDistancePixels = estimate.topology.distalBorderDistancePixels,
                newBranchCount = estimate.topology.branchCount,
                newClosedLoop = estimate.topology.closedLoop,
                newMeasurement = measurement,
                maskNanos = maskNanos,
                extractNanos = extractNanos,
                measureNanos = measureNanos,
            )
    }

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
            previousAnchorXFraction = null
            previousAnchorYFraction = null
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
        val lookaheadXFraction: Double,
        val lookaheadYFraction: Double,
        val pathSampleCount: Int,
        val pathMedianWidthFraction: Double,
        val horizontalFallback: Boolean,
        val pathCurvatureDegrees: Double,
        val pathCurvatureSmoothness: Double,
    )
    private data class FloorContext(
        val surroundingFraction: Double,
        val minimumSideFraction: Double,
    )


    private companion object {
        const val RGBA_CHANNELS = 4L
        const val MAX_ANALYSIS_DIMENSION = 640.0
        const val FRAME_INTERVAL_NANOS = 250_000_000L
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
        // A tracked path may cross a floor-material boundary, so surrounding coverage
        // stays relaxed. Both visible sides must still be corrugated board; otherwise
        // a board edge or a seam on the adjacent floor becomes a plausible continuation.
        const val MIN_TRACKED_PATH_SURROUNDING_FLOOR = 0.12
        const val MIN_TRACKED_PATH_SIDE_FLOOR = MIN_PATH_SIDE_FLOOR
        const val MIN_PATH_CURVATURE_SMOOTHNESS = 0.08
        const val MIN_HORIZONTAL_PATH_SURROUNDING_FLOOR = 0.0
        const val MIN_HORIZONTAL_PATH_SIDE_FLOOR = 0.0
        const val MIN_PATH_FRACTION = 0.20
        const val MIN_TRACKED_PATH_FRACTION = 0.12
        const val MIN_PATH_CURVATURE_DEGREES = 8.0
        const val IDEAL_PATH_FRACTION = 0.80
        const val PREVIOUS_OVERLAP_BONUS = 1.35
        const val MIN_TAPE_CHANNEL_BALANCE = 0.32
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
