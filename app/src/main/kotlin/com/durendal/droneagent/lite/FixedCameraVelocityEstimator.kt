package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.roundToInt
import kotlin.math.sqrt
import org.opencv.calib3d.Calib3d
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfByte
import org.opencv.core.MatOfFloat
import org.opencv.core.MatOfInt
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.core.TermCriteria
import org.opencv.imgproc.Imgproc
import org.opencv.video.Video

/** Read-only ground-plane similarity motion with measured marker scale, not camera-pose feedback. */
internal class FixedCameraVelocityEstimator : AutoCloseable {
    data class Estimate(
        val previousFrameNanos: Long?,
        val frameNanos: Long,
        val motionValid: Boolean,
        val valueValid: Boolean,
        val reason: String?,
        val deltaXPixels: Double?,
        val deltaYPixels: Double?,
        val forwardMps: Double?,
        val rightMps: Double?,
        val tracks: Int,
        val inliers: Int,
        val supportAreaPixels: Double?,
        val residualRmsPixels: Double?,
        val width: Int,
        val height: Int,
        val metersPerPixel: Double? = null,
        val rotationDegrees: Double? = null,
        val imageScale: Double? = null,
        val scaleSource: String? = null,
        val scaleAgeMs: Double? = null,
        val markerDistancePixels: Double? = null,
        val markerScaleErrorPercent: Double? = null,
    )

    private val rgb = Mat()
    private val gray = Mat()
    private val blurredRgb = Mat()
    private val hsv = Mat()
    private var currentGray = Mat()
    private var previousGray = Mat()
    private var currentMask = Mat()
    private var previousMask = Mat()
    private val erosionKernel = Mat()
    private val contrast = Imgproc.createCLAHE(2.0, Size(8.0, 8.0))
    private val corners = MatOfPoint()
    private val sourcePoints = MatOfPoint2f()
    private val destinationPoints = MatOfPoint2f()
    private val backPoints = MatOfPoint2f()
    private val forwardStatus = MatOfByte()
    private val backwardStatus = MatOfByte()
    private val forwardError = MatOfFloat()
    private val backwardError = MatOfFloat()
    private val supportPoints = MatOfPoint()
    private val hull = MatOfInt()
    private val markerScale = YellowMarkerScale()
    private val affineInliers = Mat()
    private val fullAffineInliers = Mat()
    private val affineValues = DoubleArray(6)
    private val fullAffineValues = DoubleArray(6)
    private val affineMask = ByteArray(MAX_FEATURES)
    private val source = FloatArray(MAX_FEATURES * 2)
    private val destination = FloatArray(MAX_FEATURES * 2)
    private val back = FloatArray(MAX_FEATURES * 2)
    private val forwardOk = ByteArray(MAX_FEATURES)
    private val backwardOk = ByteArray(MAX_FEATURES)
    private val trackSourceX = IntArray(MAX_FEATURES)
    private val trackSourceY = IntArray(MAX_FEATURES)
    private val supportCoordinates = IntArray(MAX_FEATURES * 2)
    private val hullIndices = IntArray(MAX_FEATURES)
    private var maskPixels = ByteArray(0)
    private val floorMinimum = Scalar(12.0, 28.0, 35.0)
    private val floorMaximum = Scalar(100.0, 255.0, 255.0)
    private val kernelValue = Scalar(1.0)
    private var blurSize = Size(9.0, 9.0)
    private var flowWindow = Size(25.0, 25.0)
    private var flowCriteria = TermCriteria(TermCriteria.COUNT or TermCriteria.EPS, 30, 0.01)
    private var blockSize = 5
    private var scale = 1.0
    private var width = 0
    private var height = 0
    private var previousNanos: Long? = null
    private var closed = false

    fun reset() {
        check(!closed) { "Estimator is closed" }
        previousNanos = null
        markerScale.reset()
    }

    /** [rgba] is borrowed only for this call; baseline images are owned, double-buffered Mats. */
    fun process(rgba: Mat, frameNanos: Long): Estimate {
        check(!closed) { "Estimator is closed" }
        require(!rgba.empty() && rgba.type() == CvType.CV_8UC4) { "Expected nonempty CV_8UC4 frame" }
        val previous = previousNanos
        if (previous != null && frameNanos <= previous) {
            return result(frameNanos, previous, "NON_MONOTONIC_FRAME", resultWidth = rgba.cols(), resultHeight = rgba.rows())
        }
        val resized = width != rgba.cols() || height != rgba.rows()
        if (resized) configure(rgba.cols(), rgba.rows())
        val baselineReason = when {
            previous == null -> "BASELINE"
            resized -> "FRAME_SIZE_CHANGED"
            frameNanos - previous > MAX_FRAME_GAP_NANOS -> "FRAME_GAP"
            else -> null
        }
        try {
            Imgproc.cvtColor(rgba, rgb, Imgproc.COLOR_RGBA2RGB)
            Imgproc.cvtColor(rgb, gray, Imgproc.COLOR_RGB2GRAY)
            contrast.apply(gray, currentGray)
            // Room floor/cardboard mask; excludes non-ground objects, not a general floor classifier.
            Imgproc.GaussianBlur(rgb, blurredRgb, blurSize, 0.0)
            Imgproc.cvtColor(blurredRgb, hsv, Imgproc.COLOR_RGB2HSV)
            Core.inRange(hsv, floorMinimum, floorMaximum, currentMask)
            Imgproc.erode(currentMask, currentMask, erosionKernel)
            val estimate = if (baselineReason != null) {
                markerScale.reset()
                val calibration = markerScale.update(rgb, frameNanos, null)
                result(frameNanos, previous, baselineReason, calibration = calibration)
            } else {
                measure(frameNanos, checkNotNull(previous))
            }
            if (!estimate.motionValid && baselineReason == null) markerScale.reset()
            // Even failed tracking advances the baseline. A bad frame cannot wedge recovery.
            val oldGray = previousGray
            previousGray = currentGray
            currentGray = oldGray
            val oldMask = previousMask
            previousMask = currentMask
            currentMask = oldMask
            previousNanos = frameNanos
            return estimate
        } catch (error: Exception) {
            reset()
            throw error
        }
    }

    private fun configure(newWidth: Int, newHeight: Int) {
        width = newWidth
        height = newHeight
        scale = width / REFERENCE_WIDTH
        val blur = oddPixels(9)
        blurSize = Size(blur.toDouble(), blur.toDouble())
        val window = oddPixels(25).coerceAtLeast(3)
        flowWindow = Size(window.toDouble(), window.toDouble())
        flowCriteria = TermCriteria(TermCriteria.COUNT or TermCriteria.EPS, 30, 0.01 * scale)
        blockSize = oddPixels(5).coerceAtLeast(3)
        erosionKernel.create(blur, blur, CvType.CV_8UC1)
        erosionKernel.setTo(kernelValue)
        maskPixels = ByteArray(width * height)
    }

    private fun oddPixels(reference: Int): Int = (reference * scale).roundToInt().coerceAtLeast(1) or 1

    private fun measure(frameNanos: Long, previous: Long): Estimate {
        Imgproc.goodFeaturesToTrack(previousGray, corners, MAX_FEATURES, 0.002, 7.0 * scale, previousMask, blockSize)
        val featureCount = corners.total().toInt()
        if (featureCount < MIN_SUPPORT) return result(frameNanos, previous, "NO_FEATURES")
        // The Java goodFeaturesToTrack binding returns MatOfPoint (integer pixel corners).
        corners.convertTo(sourcePoints, CvType.CV_32F)
        Video.calcOpticalFlowPyrLK(previousGray, currentGray, sourcePoints, destinationPoints,
            forwardStatus, forwardError, flowWindow, 3, flowCriteria)
        Video.calcOpticalFlowPyrLK(currentGray, previousGray, destinationPoints, backPoints,
            backwardStatus, backwardError, flowWindow, 3, flowCriteria)
        sourcePoints.get(0, 0, source)
        destinationPoints.get(0, 0, destination)
        backPoints.get(0, 0, back)
        forwardStatus.get(0, 0, forwardOk)
        backwardStatus.get(0, 0, backwardOk)
        currentMask.get(0, 0, maskPixels)
        val fbThresholdSquared = (0.8 * scale) * (0.8 * scale)
        var tracks = 0
        for (index in 0 until featureCount) {
            if (forwardOk[index].toInt() == 0 || backwardOk[index].toInt() == 0) continue
            val offset = index * 2
            val x = destination[offset].toDouble()
            val y = destination[offset + 1].toDouble()
            val bx = back[offset].toDouble() - source[offset]
            val by = back[offset + 1].toDouble() - source[offset + 1]
            if (!x.isFinite() || !y.isFinite() || !bx.isFinite() || !by.isFinite()) continue
            if (bx * bx + by * by >= fbThresholdSquared) continue
            val pixelX = Math.rint(x).toInt()
            val pixelY = Math.rint(y).toInt()
            if (pixelX !in 0 until width || pixelY !in 0 until height || maskPixels[pixelY * width + pixelX].toInt() == 0) continue
            trackSourceX[tracks] = source[offset].toInt()
            trackSourceY[tracks] = source[offset + 1].toInt()
            source[tracks * 2] = source[offset]
            source[tracks * 2 + 1] = source[offset + 1]
            destination[tracks * 2] = x.toFloat()
            destination[tracks * 2 + 1] = y.toFloat()
            tracks++
        }
        if (tracks < MIN_SUPPORT) return result(frameNanos, previous, "TOO_FEW_TRACKS", tracks = tracks)
        sourcePoints.create(tracks, 1, CvType.CV_32FC2)
        destinationPoints.create(tracks, 1, CvType.CV_32FC2)
        sourcePoints.put(0, 0, source)
        destinationPoints.put(0, 0, destination)
        val threshold = 1.4 * scale
        val transform = Calib3d.estimateAffinePartial2D(
            sourcePoints, destinationPoints, affineInliers, Calib3d.RANSAC, threshold, 1000L, 0.99, 10L,
        )
        try {
            if (transform.empty()) return result(frameNanos, previous, "NO_SIMILARITY", tracks = tracks)
            transform.get(0, 0, affineValues)
            affineInliers.get(0, 0, affineMask)
        } finally {
            transform.release()
        }
        val a = affineValues[0]
        val b = affineValues[3]
        val tx = affineValues[2]
        val ty = affineValues[5]
        val imageScale = hypot(a, b)
        val rotation = Math.toDegrees(atan2(b, a))
        if (affineValues.any { !it.isFinite() } || imageScale !in 0.85..1.18) {
            return result(frameNanos, previous, "EXCESSIVE_SCALE_CHANGE", tracks = tracks)
        }
        val cx = (width - 1) * 0.5
        val cy = (height - 1) * 0.5
        // Translation at the optical image centre, not at an off-centre feature centroid.
        val dx = a * cx - b * cy + tx - cx
        val dy = b * cx + a * cy + ty - cy
        val thresholdSquared = threshold * threshold
        val fullTransform = Calib3d.estimateAffine2D(
            sourcePoints, destinationPoints, fullAffineInliers, Calib3d.RANSAC, threshold, 1000L, 0.99, 10L,
        )
        try {
            if (fullTransform.empty()) return result(frameNanos, previous, "NO_AFFINE_SUPPORT", tracks = tracks)
            fullTransform.get(0, 0, fullAffineValues)
            val u = hypot(fullAffineValues[0], fullAffineValues[3])
            val v = hypot(fullAffineValues[1], fullAffineValues[4])
            val shear = (fullAffineValues[0] * fullAffineValues[1] +
                fullAffineValues[3] * fullAffineValues[4]) / (u * v)
            // Two known lengths are not a full camera calibration: reject appreciable
            // anisotropy/shear instead of reporting perspective distortion as metric speed.
            if (fullAffineValues.any { !it.isFinite() } || u <= 0.0 || v <= 0.0 ||
                abs(u / v - 1.0) > 0.025 || abs(shear) > 0.025
            ) return result(frameNanos, previous, "NON_SIMILARITY_MOTION", tracks = tracks)
        } finally {
            fullTransform.release()
        }
        var inliers = 0
        var residualSum = 0.0
        for (index in 0 until tracks) {
            val residual = residualSquared(index, a, b, tx, ty)
            if (affineMask[index].toInt() == 0 || residual > thresholdSquared) continue
            supportCoordinates[inliers * 2] = trackSourceX[index]
            supportCoordinates[inliers * 2 + 1] = trackSourceY[index]
            residualSum += residual
            inliers++
        }
        val rms = if (inliers > 0) sqrt(residualSum / inliers) else null
        if (inliers < MIN_SUPPORT) return result(frameNanos, previous, "TOO_FEW_INLIERS", dx = dx, dy = dy,
            tracks = tracks, inliers = inliers, rms = rms)
        supportPoints.create(inliers, 1, CvType.CV_32SC2)
        supportPoints.put(0, 0, supportCoordinates)
        Imgproc.convexHull(supportPoints, hull)
        hull.get(0, 0, hullIndices)
        val hullCount = hull.total().toInt()
        var doubleArea = 0.0
        for (index in 0 until hullCount) {
            val first = hullIndices[index] * 2
            val second = hullIndices[(index + 1) % hullCount] * 2
            doubleArea += supportCoordinates[first].toDouble() * supportCoordinates[second + 1] -
                supportCoordinates[second].toDouble() * supportCoordinates[first + 1]
        }
        val area = abs(doubleArea) * 0.5
        if (area < 8000.0 * scale * scale) return result(frameNanos, previous, "NARROW_SUPPORT", dx = dx, dy = dy,
            tracks = tracks, inliers = inliers, area = area, rms = rms)
        val calibration = markerScale.update(rgb, frameNanos, imageScale)
        val metricMultiplier = calibration.metersPerPixel?.let { it / ((frameNanos - previous) * 1e-9) }
        return result(frameNanos, previous, calibration.reason, motionValid = true, dx = dx, dy = dy,
            forward = metricMultiplier?.let { dy * it }, right = metricMultiplier?.let { -dx * it },
            tracks = tracks, inliers = inliers, area = area, rms = rms,
            rotation = rotation, imageScale = imageScale, calibration = calibration)
    }

    private fun residualSquared(index: Int, a: Double, b: Double, tx: Double, ty: Double): Double {
        val x = destination[index * 2] - (a * trackSourceX[index] - b * trackSourceY[index] + tx)
        val y = destination[index * 2 + 1] - (b * trackSourceX[index] + a * trackSourceY[index] + ty)
        return x * x + y * y
    }

    private fun result(
        frameNanos: Long,
        previous: Long?,
        reason: String?,
        motionValid: Boolean = false,
        dx: Double? = null,
        dy: Double? = null,
        forward: Double? = null,
        right: Double? = null,
        tracks: Int = 0,
        inliers: Int = 0,
        area: Double? = null,
        rms: Double? = null,
        resultWidth: Int = width,
        resultHeight: Int = height,
        rotation: Double? = null,
        imageScale: Double? = null,
        calibration: YellowMarkerScale.Observation? = null,
    ) = Estimate(previous, frameNanos, motionValid, motionValid && reason == null, reason, dx, dy,
        forward, right, tracks, inliers, area, rms, resultWidth, resultHeight,
        calibration?.metersPerPixel, rotation, imageScale, calibration?.source, calibration?.ageMs,
        calibration?.markerDistancePixels, calibration?.errorPercent)

    override fun close() {
        if (closed) return
        reset()
        closed = true
        rgb.release()
        gray.release()
        blurredRgb.release()
        hsv.release()
        currentGray.release()
        previousGray.release()
        currentMask.release()
        previousMask.release()
        erosionKernel.release()
        corners.release()
        sourcePoints.release()
        destinationPoints.release()
        backPoints.release()
        forwardStatus.release()
        backwardStatus.release()
        forwardError.release()
        backwardError.release()
        supportPoints.release()
        hull.release()
        affineInliers.release()
        fullAffineInliers.release()
        markerScale.close()
        contrast.collectGarbage()
        contrast.clear()
        maskPixels = ByteArray(0)
    }

    companion object {
        const val CALIBRATION_ID = "yellow_isolated_pairs_measured_20cm_similarity"
        const val MARKER_DISTANCE_METERS = YellowMarkerScale.DISTANCE_METERS
        const val MAX_FRAME_GAP_NANOS = 500_000_000L
        private const val REFERENCE_WIDTH = 848.0
        private const val MAX_FEATURES = 650
        private const val MIN_SUPPORT = 20
    }
}
