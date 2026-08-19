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
class BlackTapeDetector(
    private val onResult: (TapeDetection?) -> Unit,
    private val onError: (Throwable) -> Unit = {},
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
                    val result = detect(copy, width, height)
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

    internal fun setDetectionMode(mode: TapeDetectionMode) {
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

    private fun detect(rgbaBytes: ByteArray, width: Int, height: Int): TapeDetection? {
        val source = Mat(height, width, org.opencv.core.CvType.CV_8UC4)
        val resized = Mat()
        val rgb = Mat()
        val gray = Mat()
        val blurred = Mat()
        val lab = Mat()
        val blackMask = Mat()
        val bridgedBlackMask = Mat()
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
        // Fluorescent glare creates short bright gaps inside the black tape. Straight
        // tape is closed along its axis so those fragments reconnect without merging
        // nearby objects; a curved path needs an isotropic kernel instead.
        val closeKernel = Imgproc.getStructuringElement(
            if (mode == TapeDetectionMode.PATH) Imgproc.MORPH_ELLIPSE else Imgproc.MORPH_RECT,
            if (mode == TapeDetectionMode.PATH) {
                Size(PATH_CLOSE_KERNEL_SIZE, PATH_CLOSE_KERNEL_SIZE)
            } else {
                Size(TAPE_MASK_KERNEL_WIDTH, VERTICAL_CLOSE_KERNEL_HEIGHT)
            },
        )
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
            Imgproc.morphologyEx(blackMask, bridgedBlackMask, Imgproc.MORPH_CLOSE, closeKernel)
            Imgproc.morphologyEx(bridgedBlackMask, cleanedBlackMask, Imgproc.MORPH_OPEN, openKernel)
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
                lookaheadXFraction = winner.lookaheadXFraction,
                lookaheadYFraction = winner.lookaheadYFraction,
            )
        } finally {
            candidateMask.release()
            contours.forEach(MatOfPoint::release)
            hierarchy.release()
            closeKernel.release()
            openKernel.release()
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
        val minimumSurroundingFloor =
            if (path.horizontalFallback) MIN_HORIZONTAL_PATH_SURROUNDING_FLOOR
            else MIN_PATH_SURROUNDING_FLOOR
        val minimumSideFloor =
            if (path.horizontalFallback) MIN_HORIZONTAL_PATH_SIDE_FLOOR
            else MIN_PATH_SIDE_FLOOR
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

    private fun registerDetectionMiss() {
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
        const val PATH_OPEN_KERNEL_SIZE = 3.0
        const val MIN_PATH_AREA_FRACTION = 0.0008
        const val MAX_PATH_AREA_FRACTION = 0.18
        const val MIN_PATH_SURROUNDING_FLOOR = 0.22
        const val MIN_PATH_SIDE_FLOOR = 0.30
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
