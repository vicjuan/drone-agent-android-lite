package com.durendal.droneagent.lite

import org.opencv.android.OpenCVLoader
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
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
    @Volatile private var detectionMode = TapeDetectionMode.STRAIGHT
    private var consecutiveDetectionMisses = 0
    @Volatile private var lastOtsuThreshold = 0.0
    @Volatile private var lastEffectiveThreshold = 0.0
    @Volatile private var lastClassSeparation = 0.0
    @Volatile private var lastContourCount = 0
    private val rejectionCounts = IntArray(TapeCandidateRejection.entries.size)
    private var luminanceBytes = ByteArray(0)
    @Volatile private var lastDetectionMode = TapeDetectionMode.STRAIGHT
    @Volatile private var lastPathSampleCount = 0
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
                consecutiveDetectionMisses = 0
            }
        } catch (error: RuntimeException) {
            if (!closed.get()) onError(error)
        }
    }

    fun diagnosticsSummary(): String =
        (
            "mode=%s pathSamples=%d otsu=%.1f effective=%.1f separation=%.1f contours=%d " +
                "rejects=invalid:%d area:%d aspect:%d length:%d width:%d edge:%d fill:%d brown:%d"
            ).format(
            lastDetectionMode,
            lastPathSampleCount,
            lastOtsuThreshold,
            lastEffectiveThreshold,
            lastClassSeparation,
            lastContourCount,
            rejectionCounts[TapeCandidateRejection.INVALID_GEOMETRY.ordinal],
            rejectionCounts[TapeCandidateRejection.AREA.ordinal],
            rejectionCounts[TapeCandidateRejection.ASPECT.ordinal],
            rejectionCounts[TapeCandidateRejection.LENGTH.ordinal],
            rejectionCounts[TapeCandidateRejection.WIDTH.ordinal],
            rejectionCounts[TapeCandidateRejection.HORIZONTAL_FRAME_EDGE.ordinal],
            rejectionCounts[TapeCandidateRejection.ORIENTED_FILL.ordinal],
            rejectionCounts[TapeCandidateRejection.BROWN_CONTEXT.ordinal],
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
        val hsv = Mat()
        val blackMask = Mat()
        val bridgedBlackMask = Mat()
        val brownMask = Mat()
        val cleanedBlackMask = Mat()
        val candidateMask = Mat()
        val hierarchy = Mat()
        // Fluorescent glare creates short bright gaps inside the black tape. Close only
        // along its axis so those fragments reconnect without merging nearby objects.
        val mode = detectionMode
        lastDetectionMode = mode
        lastPathSampleCount = 0
        val closeKernel = Imgproc.getStructuringElement(
            if (mode == TapeDetectionMode.CURVED) Imgproc.MORPH_ELLIPSE else Imgproc.MORPH_RECT,
            if (mode == TapeDetectionMode.CURVED) {
                Size(CURVED_CLOSE_KERNEL_SIZE, CURVED_CLOSE_KERNEL_SIZE)
            } else {
                Size(TAPE_MASK_KERNEL_WIDTH, VERTICAL_CLOSE_KERNEL_HEIGHT)
            },
        )
        val openKernel = Imgproc.getStructuringElement(
            if (mode == TapeDetectionMode.CURVED) Imgproc.MORPH_ELLIPSE else Imgproc.MORPH_RECT,
            if (mode == TapeDetectionMode.CURVED) {
                Size(CURVED_OPEN_KERNEL_SIZE, CURVED_OPEN_KERNEL_SIZE)
            } else {
                Size(TAPE_MASK_KERNEL_WIDTH, VERTICAL_OPEN_KERNEL_HEIGHT)
            },
        )
        val contours = mutableListOf<MatOfPoint>()
        try {
            rejectionCounts.fill(0)
            lastContourCount = 0
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

            Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV)
            Core.inRange(hsv, BROWN_MIN_HSV, BROWN_MAX_HSV, brownMask)
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
            for ((contourIndex, contour) in contours.withIndex()) {
                val candidate =
                    scoreCandidate(
                        contour,
                        contourIndex,
                        contours,
                        cleanedBlackMask,
                        candidateMask,
                        brownMask,
                        frameArea,
                        frameShortSide,
                        mode,
                    ) ?: continue
                val continuityBonus =
                    if (overlapsPrevious(candidate.bounds)) PREVIOUS_OVERLAP_BONUS else 1.0
                val selectionScore = candidate.score * continuityBonus
                if (selectionScore > bestSelectionScore) {
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
            previousBounds = Rect(rect.x, rect.y, rect.width, rect.height)
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
            )
        } finally {
            candidateMask.release()
            contours.forEach(MatOfPoint::release)
            hierarchy.release()
            closeKernel.release()
            openKernel.release()
            cleanedBlackMask.release()
            brownMask.release()
            bridgedBlackMask.release()
            blackMask.release()
            hsv.release()
            blurred.release()
            gray.release()
            rgb.release()
            resized.release()
            source.release()
        }
    }

    private fun scoreCandidate(
        contour: MatOfPoint,
        contourIndex: Int,
        contours: List<MatOfPoint>,
        blackMask: Mat,
        candidateMask: Mat,
        brownMask: Mat,
        frameArea: Double,
        frameShortSide: Double,
        mode: TapeDetectionMode,
    ): Candidate? {
        val contourArea = Imgproc.contourArea(contour)
        val bounds = Imgproc.boundingRect(contour)
        if (bounds.width <= 0 || bounds.height <= 0) {
            rejectionCounts[TapeCandidateRejection.INVALID_GEOMETRY.ordinal] += 1
            return null
        }
        if (mode == TapeDetectionMode.CURVED) {
            return scoreCurvedCandidate(
                contourIndex,
                contours,
                blackMask,
                candidateMask,
                brownMask,
                contourArea,
                bounds,
                frameArea,
                frameShortSide,
            )
        }

        val contourPoints = MatOfPoint2f(*contour.toArray())
        val orientedBounds = try {
            Imgproc.minAreaRect(contourPoints)
        } finally {
            contourPoints.release()
        }
        val orientedWidth = orientedBounds.size.width
        val orientedHeight = orientedBounds.size.height
        if (orientedWidth <= 0.0 || orientedHeight <= 0.0) {
            rejectionCounts[TapeCandidateRejection.INVALID_GEOMETRY.ordinal] += 1
            return null
        }

        val shortSide = min(orientedWidth, orientedHeight)
        val longSide = max(orientedWidth, orientedHeight)
        val orientedArea = orientedWidth * orientedHeight
        val metrics = TapeCandidateMetrics(
            areaFraction = contourArea / frameArea,
            aspectRatio = longSide / shortSide,
            shortSideFraction = shortSide / frameShortSide,
            longSideFraction = longSide / frameShortSide,
            orientedFill = (contourArea / orientedArea).coerceIn(0.0, 1.0),
            surroundingBrown = surroundingBrownFraction(brownMask, bounds),
            touchesHorizontalFrameEdge =
                bounds.x <= HORIZONTAL_EDGE_MARGIN ||
                    bounds.x + bounds.width >= brownMask.cols() - HORIZONTAL_EDGE_MARGIN,
            overlapsPreviousDetection = overlapsPrevious(bounds),
        )
        val rejection = TapeCandidatePolicy.rejectionReason(metrics)
        if (rejection != null) {
            rejectionCounts[rejection.ordinal] += 1
            return null
        }
        val axis = tapeAxis(orientedBounds)
        return Candidate(
            bounds = bounds,
            score = checkNotNull(TapeCandidatePolicy.score(metrics)),
            angleFromVerticalDegrees = axis.angleFromVerticalDegrees,
            longSideFraction = metrics.longSideFraction,
            nearFieldOffsetFraction =
                (axis.nearFieldCenterX / brownMask.cols() - 0.5).coerceIn(-0.5, 0.5),
        )
    }

    private fun scoreCurvedCandidate(
        contourIndex: Int,
        contours: List<MatOfPoint>,
        blackMask: Mat,
        candidateMask: Mat,
        brownMask: Mat,
        contourArea: Double,
        bounds: Rect,
        frameArea: Double,
        frameShortSide: Double,
    ): Candidate? {
        val areaFraction = contourArea / frameArea
        if (areaFraction !in MIN_CURVED_AREA_FRACTION..MAX_CURVED_AREA_FRACTION) {
            rejectionCounts[TapeCandidateRejection.AREA.ordinal] += 1
            return null
        }
        val overlapsPrevious = overlapsPrevious(bounds)
        if (
            !overlapsPrevious &&
            (bounds.x <= HORIZONTAL_EDGE_MARGIN ||
                bounds.x + bounds.width >= brownMask.cols() - HORIZONTAL_EDGE_MARGIN)
        ) {
            rejectionCounts[TapeCandidateRejection.HORIZONTAL_FRAME_EDGE.ordinal] += 1
            return null
        }
        val surroundingBrown = surroundingBrownFraction(brownMask, bounds)
        if (surroundingBrown < MIN_CURVED_SURROUNDING_BROWN) {
            rejectionCounts[TapeCandidateRejection.BROWN_CONTEXT.ordinal] += 1
            return null
        }

        if (candidateMask.empty() || candidateMask.size() != blackMask.size()) {
            candidateMask.create(blackMask.rows(), blackMask.cols(), org.opencv.core.CvType.CV_8UC1)
        }
        candidateMask.setTo(Scalar(0.0))
        Imgproc.drawContours(candidateMask, contours, contourIndex, Scalar(255.0), Imgproc.FILLED)
        val pixelCount = candidateMask.total().toInt()
        if (candidateMaskBytes.size != pixelCount) candidateMaskBytes = ByteArray(pixelCount)
        candidateMask.get(0, 0, candidateMaskBytes)
        val path = pathDirectionEstimator.estimate(
            mask = candidateMaskBytes,
            frameWidth = candidateMask.cols(),
            frameHeight = candidateMask.rows(),
            left = bounds.x,
            top = bounds.y,
            right = bounds.x + bounds.width,
            bottom = bounds.y + bounds.height,
        )
        if (path == null) {
            rejectionCounts[TapeCandidateRejection.LENGTH.ordinal] += 1
            return null
        }
        val minimumPathFraction =
            if (overlapsPrevious) MIN_TRACKED_CURVED_PATH_FRACTION else MIN_CURVED_PATH_FRACTION
        if (path.arcLengthFraction < minimumPathFraction) {
            rejectionCounts[TapeCandidateRejection.LENGTH.ordinal] += 1
            return null
        }
        val pathConfidence =
            (path.arcLengthFraction / IDEAL_CURVED_PATH_FRACTION).coerceIn(0.0, 1.0)
        val continuityConfidence = if (overlapsPrevious) 1.0 else 0.5
        val score = (
            surroundingBrown * 0.35 +
                path.widthConsistency * 0.30 +
                pathConfidence * 0.25 +
                continuityConfidence * 0.10
            ).coerceIn(0.0, 1.0)
        return Candidate(
            bounds = bounds,
            score = score,
            angleFromVerticalDegrees = path.lookaheadAngleFromVerticalDegrees,
            longSideFraction = path.arcLengthFraction,
            nearFieldOffsetFraction =
                (path.nearFieldCenterX / brownMask.cols() - 0.5).coerceIn(-0.5, 0.5),
            pathSampleCount = path.sampleCount,
        )
    }

    private fun tapeAxis(bounds: org.opencv.core.RotatedRect): TapeAxis {
        val corners = Array(4) { Point() }
        bounds.points(corners)
        var longestSquared = Double.NEGATIVE_INFINITY
        var longestDeltaX = 0.0
        var longestDeltaY = 0.0
        var nearest = corners[0]
        var secondNearest = corners[1]
        if (secondNearest.y > nearest.y) {
            nearest = corners[1]
            secondNearest = corners[0]
        }
        for (index in corners.indices) {
            val start = corners[index]
            val end = corners[(index + 1) % corners.size]
            val deltaX = end.x - start.x
            val deltaY = end.y - start.y
            val lengthSquared = deltaX * deltaX + deltaY * deltaY
            if (lengthSquared > longestSquared) {
                longestSquared = lengthSquared
                longestDeltaX = deltaX
                longestDeltaY = deltaY
            }
            if (index < 2) continue
            when {
                start.y > nearest.y -> {
                    secondNearest = nearest
                    nearest = start
                }
                start.y > secondNearest.y -> secondNearest = start
            }
        }
        return TapeAxis(
            angleFromVerticalDegrees =
                TapeOrientation.deviationFromVerticalDegrees(longestDeltaX, longestDeltaY),
            nearFieldCenterX = (nearest.x + secondNearest.x) / 2.0,
        )
    }

    private fun surroundingBrownFraction(brownMask: Mat, bounds: Rect): Double {
        val surround = expand(bounds, brownMask.cols(), brownMask.rows())
        val surroundArea = (surround.area() - bounds.area()).coerceAtLeast(1.0)
        val surroundingBrownPixels =
            (countNonZero(brownMask, surround) - countNonZero(brownMask, bounds)).coerceAtLeast(0)
        return (surroundingBrownPixels / surroundArea).coerceIn(0.0, 1.0)
    }

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

    private fun registerDetectionMiss() {
        consecutiveDetectionMisses++
        if (consecutiveDetectionMisses >= PREVIOUS_SELECTION_MISS_LIMIT) previousBounds = null
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
        val pathSampleCount: Int = 0,
    )

    private data class TapeAxis(
        val angleFromVerticalDegrees: Double,
        val nearFieldCenterX: Double,
    )

    private companion object {
        const val RGBA_CHANNELS = 4L
        const val MAX_ANALYSIS_DIMENSION = 640.0
        const val FRAME_INTERVAL_NANOS = 250_000_000L
        const val SURROUND_SCALE = 0.35
        const val HORIZONTAL_EDGE_MARGIN = 1
        const val MIN_SURROUND_PADDING = 8
        const val MIN_CLASS_SEPARATION_LUMINANCE = 30.0
        const val TAPE_MASK_KERNEL_WIDTH = 3.0
        const val VERTICAL_OPEN_KERNEL_HEIGHT = 31.0
        const val VERTICAL_CLOSE_KERNEL_HEIGHT = 25.0
        const val CURVED_CLOSE_KERNEL_SIZE = 5.0
        const val CURVED_OPEN_KERNEL_SIZE = 3.0
        const val MIN_CURVED_AREA_FRACTION = 0.0008
        const val MAX_CURVED_AREA_FRACTION = 0.18
        const val MIN_CURVED_SURROUNDING_BROWN = 0.22
        const val MIN_CURVED_PATH_FRACTION = 0.20
        const val MIN_TRACKED_CURVED_PATH_FRACTION = 0.12
        const val IDEAL_CURVED_PATH_FRACTION = 0.80
        const val PREVIOUS_OVERLAP_BONUS = 1.35
        const val MIN_PREVIOUS_OVERLAP = 0.20
        const val PREVIOUS_SELECTION_MISS_LIMIT = 8
        val BROWN_MIN_HSV = Scalar(4.0, 45.0, 35.0)
        val BROWN_MAX_HSV = Scalar(40.0, 255.0, 255.0)
    }
}
