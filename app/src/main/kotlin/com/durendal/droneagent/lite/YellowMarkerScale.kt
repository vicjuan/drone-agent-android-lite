package com.durendal.droneagent.lite

import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.sqrt
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

/** Scene-specific scale references, not sticker OCR or a calibrated camera-pose solver.
 * The four yellow round stickers form two isolated close pairs, each measured at 0.20 m.
 * Cross-pair distances are unknown and must never be used as references.
 */
internal class YellowMarkerScale : AutoCloseable {
    data class Observation(
        val metersPerPixel: Double?,
        val source: String?,
        val ageMs: Double?,
        val markerDistancePixels: Double?,
        val errorPercent: Double?,
        val reason: String?,
    )

    private val hsv = Mat()
    private val mask = Mat()
    private val labels = Mat()
    private val stats = Mat()
    private val centroids = Mat()
    private val low = Scalar(25.0, 80.0, 190.0)
    private val high = Scalar(40.0, 255.0, 255.0)
    private val component = IntArray(5)
    private val centre = DoubleArray(2)
    private val x = DoubleArray(MAX_MARKERS)
    private val y = DoubleArray(MAX_MARKERS)
    private val diameter = DoubleArray(MAX_MARKERS)
    private val nearest = IntArray(MAX_MARKERS)
    private var metersPerPixel: Double? = null
    private var lastObservedNanos: Long? = null
    private var pendingScale: Double? = null
    private var pendingAtNanos: Long? = null

    fun reset() {
        metersPerPixel = null
        lastObservedNanos = null
        pendingScale = null
        pendingAtNanos = null
    }

    fun update(rgb: Mat, frameNanos: Long, imageScale: Double?): Observation {
        // Never carry scale through a failed image registration or a new baseline.
        if (imageScale == null || !imageScale.isFinite() || imageScale <= 0.0) reset()
        val propagated = metersPerPixel?.let { it / checkNotNull(imageScale) }
        val precedingCandidate = pendingScale?.let { it / checkNotNull(imageScale) }
        val age = lastObservedNanos?.let { (frameNanos - it) * 1e-6 }
        val pairPixels = measurePair(rgb)
        val observed = pairPixels?.let { DISTANCE_METERS / it }
        val error = if (observed != null && propagated != null) {
            (observed / propagated - 1.0) * 100.0
        } else null
        val coherent = observed != null && precedingCandidate != null &&
            pendingAtNanos?.let { frameNanos - it in 1..500_000_000L } == true &&
            abs(observed / precedingCandidate - 1.0) <= MAX_SCALE_DISAGREEMENT
        pendingScale = observed
        pendingAtNanos = frameNanos.takeIf { observed != null }

        if (observed != null && propagated != null &&
            abs(observed / propagated - 1.0) > MAX_SCALE_DISAGREEMENT
        ) {
            metersPerPixel = null
            lastObservedNanos = null
            return Observation(null, null, null, pairPixels, error, "SCALE_DISAGREEMENT")
        }
        if (coherent) {
            metersPerPixel = observed
            lastObservedNanos = frameNanos
            return Observation(observed, "MARKER_PAIR_20CM", 0.0, pairPixels, error, null)
        }
        if (propagated != null && age != null && age <= MAX_AGE_MS) {
            metersPerPixel = propagated
            return Observation(propagated, "PROPAGATED_SIMILARITY", age, pairPixels, error, null)
        }
        metersPerPixel = null
        return Observation(null, null, age, pairPixels, error, if (age != null) "SCALE_EXPIRED" else "NO_SCALE")
    }

    private fun measurePair(rgb: Mat): Double? {
        Imgproc.cvtColor(rgb, hsv, Imgproc.COLOR_RGB2HSV)
        Core.inRange(hsv, low, high, mask)
        val components = Imgproc.connectedComponentsWithStats(mask, labels, stats, centroids)
        val resolutionScale = rgb.cols() / 848.0
        var count = 0
        for (index in 1 until components) {
            stats.get(index, 0, component)
            val left = component[Imgproc.CC_STAT_LEFT]
            val top = component[Imgproc.CC_STAT_TOP]
            val width = component[Imgproc.CC_STAT_WIDTH]
            val height = component[Imgproc.CC_STAT_HEIGHT]
            val area = component[Imgproc.CC_STAT_AREA]
            if (left <= 1 || top <= 1 || left + width >= rgb.cols() - 1 || top + height >= rgb.rows() - 1) continue
            if (width < 4.0 * resolutionScale || height < 4.0 * resolutionScale ||
                width > 20.0 * resolutionScale || height > 20.0 * resolutionScale ||
                area < 15.0 * resolutionScale * resolutionScale ||
                area.toDouble() / (width * height) < 0.45 ||
                width.toDouble() / height !in 0.65..1.54
            ) continue
            // More than the four expected stickers means this scene is ambiguous.
            if (count == MAX_MARKERS) return null
            centroids.get(index, 0, centre)
            x[count] = centre[0]
            y[count] = centre[1]
            diameter[count] = sqrt(width.toDouble() * height)
            count++
        }
        if (count !in 2..MAX_MARKERS) return null
        for (a in 0 until count) {
            var best = Double.POSITIVE_INFINITY
            nearest[a] = -1
            for (b in 0 until count) {
                if (a == b) continue
                val distance = hypot(x[a] - x[b], y[a] - y[b])
                if (distance < best) {
                    best = distance
                    nearest[a] = b
                }
            }
        }
        var pairCount = 0
        var sum = 0.0
        var first = 0.0
        for (a in 0 until count) {
            val b = nearest[a]
            if (b <= a || nearest[b] != a) continue
            val distance = hypot(x[a] - x[b], y[a] - y[b])
            val meanDiameter = (diameter[a] + diameter[b]) * 0.5
            // This size/separation gate belongs to the photographed round-sticker setup.
            // It excludes the unknown, much longer distances between the two pairs.
            if (diameter[a] / diameter[b] !in 0.70..1.43 || distance / meanDiameter !in 6.0..16.0) continue
            if (pairCount > 0 && abs(distance / first - 1.0) > MAX_SCALE_DISAGREEMENT) return null
            if (pairCount == 0) first = distance
            sum += distance
            pairCount++
        }
        return if (pairCount == 0) null else sum / pairCount
    }

    override fun close() {
        hsv.release()
        mask.release()
        labels.release()
        stats.release()
        centroids.release()
        reset()
    }

    companion object {
        const val DISTANCE_METERS = 0.20
        private const val MAX_MARKERS = 4
        private const val MAX_AGE_MS = 10_000.0
        private const val MAX_SCALE_DISAGREEMENT = 0.08
    }
}
