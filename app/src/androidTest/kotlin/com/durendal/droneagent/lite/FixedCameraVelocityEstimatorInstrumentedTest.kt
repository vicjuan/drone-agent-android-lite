package com.durendal.droneagent.lite

import androidx.test.ext.junit.runners.AndroidJUnit4
import java.util.Random
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test
import org.junit.runner.RunWith
import org.opencv.android.OpenCVLoader
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc

@RunWith(AndroidJUnit4::class)
class FixedCameraVelocityEstimatorInstrumentedTest {
    @Before
    fun initializeOpenCv() {
        assertTrue(OpenCVLoader.initLocal())
    }

    @Test
    fun measuredMarkerDistanceDeterminesAircraftSignsAndActualInterval() {
        val frame = texturedFloor(markers = true)
        val moved = translated(frame, 3.0, 2.0)
        try {
            FixedCameraVelocityEstimator().use { estimator ->
                val baselineTime = establishScale(estimator, frame)
                val fast = estimator.process(moved, baselineTime + STEP)
                assertMetricTranslation(fast, 3.0, 2.0)
                // A 100px pair is physically 0.20m, independent of telemetry height.
                assertEquals(0.002, checkNotNull(fast.metersPerPixel), 0.0001)
                assertEquals(0.04, checkNotNull(fast.forwardMps), 0.004)
                assertEquals(-0.06, checkNotNull(fast.rightMps), 0.004)

                estimator.reset()
                val slowBaselineTime = establishScale(estimator, frame)
                val slow = estimator.process(moved, slowBaselineTime + 2 * STEP)
                assertMetricTranslation(slow, 3.0, 2.0)
                assertEquals(checkNotNull(fast.forwardMps) / 2.0, checkNotNull(slow.forwardMps), 0.002)
                assertEquals(checkNotNull(fast.rightMps) / 2.0, checkNotNull(slow.rightMps), 0.002)

                estimator.reset()
                val reverseBaselineTime = establishScale(estimator, moved)
                val reverse = estimator.process(frame, reverseBaselineTime + STEP)
                assertMetricTranslation(reverse, -3.0, -2.0)
                assertEquals(-0.04, checkNotNull(reverse.forwardMps), 0.004)
                assertEquals(0.06, checkNotNull(reverse.rightMps), 0.004)
            }
        } finally {
            frame.release()
            moved.release()
        }
    }

    @Test
    fun additionalPairsAndAnIsolatedMarkerPreserveMetricSpeedAcrossSceneReset() {
        val many = texturedFloor(markers = true)
        for (y in doubleArrayOf(80.0, 400.0)) {
            for (x in doubleArrayOf(100.0, 350.0, 600.0)) {
                drawMarkerPair(many, x, y)
            }
        }
        // Seven isolated 20cm pairs plus one unpaired marker: count is not ambiguity.
        Imgproc.circle(many, Point(800.0, 240.0), 5, MARKER_COLOUR, -1)
        val movedMany = translated(many, 3.0, 2.0)
        val few = texturedFloor()
        drawMarkerPair(few, WIDTH / 2.0, HEIGHT / 2.0, distancePixels = 80.0)
        val movedFew = translated(few, 3.0, 2.0)
        try {
            FixedCameraVelocityEstimator().use { estimator ->
                val manyBaseline = establishScale(estimator, many)
                val manyResult = estimator.process(movedMany, manyBaseline + STEP)
                assertMetricTranslation(manyResult, 3.0, 2.0)
                assertEquals(0.04, checkNotNull(manyResult.forwardMps), 0.004)
                assertEquals(-0.06, checkNotNull(manyResult.rightMps), 0.004)

                // Reused storage must not mix the old pairs into a smaller, new scene.
                estimator.reset()
                val fewBaseline = establishScale(estimator, few, START + 10 * STEP)
                val fewResult = estimator.process(movedFew, fewBaseline + STEP)
                assertMetricTranslation(fewResult, 3.0, 2.0)
                assertEquals(0.05, checkNotNull(fewResult.forwardMps), 0.004)
                assertEquals(-0.075, checkNotNull(fewResult.rightMps), 0.004)
            }
        } finally {
            many.release()
            movedMany.release()
            few.release()
            movedFew.release()
        }
    }

    @Test
    fun conflictingPairScalesCannotPublishMetricVelocity() {
        val frame = texturedFloor()
        drawMarkerPair(frame, 150.0, 80.0)
        drawMarkerPair(frame, 650.0, 80.0)
        drawMarkerPair(frame, 424.0, 360.0, distancePixels = 140.0)
        val moved = translated(frame, 3.0, 2.0)
        try {
            FixedCameraVelocityEstimator().use { estimator ->
                for (index in 0 until 4) {
                    estimator.process(frame, START + index * STEP)
                }
                val result = estimator.process(moved, START + 4 * STEP)
                assertPixelTranslation(result, 3.0, 2.0)
                assertNoScale(result)
            }
        } finally {
            frame.release()
            moved.release()
        }
    }

    @Test
    fun texturelessIsInvalidButTexturedStationaryHasZeroPixelMotionAndTrackingRecovers() {
        val flat = Mat(HEIGHT, WIDTH, CvType.CV_8UC4, FLOOR_COLOUR)
        val texture = texturedFloor()
        val moved = translated(texture, 3.0, 2.0)
        try {
            FixedCameraVelocityEstimator().use { estimator ->
                estimator.process(flat, START)
                assertNoMotion(estimator.process(flat, START + STEP), "NO_FEATURES")
                assertNoMotion(estimator.process(texture, START + 2 * STEP), "NO_FEATURES")
                // Failure advances to the textured baseline rather than retaining the flat image.
                val recovered = estimator.process(moved, START + 3 * STEP)
                assertPixelTranslation(recovered, 3.0, 2.0)
                assertNoScale(recovered)
                val stationary = estimator.process(moved, START + 4 * STEP)
                assertPixelTranslation(stationary, 0.0, 0.0)
                assertNoScale(stationary)
            }
        } finally {
            flat.release()
            texture.release()
            moved.release()
        }
    }

    @Test
    fun resetGapAndResizeRequireFreshBaselinesThenRecoverWithAdjacentFrameIntervals() {
        val frame = texturedFloor()
        val moved = translated(frame, 3.0, 2.0)
        val otherScene = texturedFloor(seed = 947L)
        val otherMoved = translated(otherScene, -2.0, 3.0)
        val smaller = texturedFloor(640, 360)
        val smallerMoved = translated(smaller, 4.0, 3.0)
        try {
            FixedCameraVelocityEstimator().use { estimator ->
                assertNoMotion(estimator.process(frame, START), "BASELINE")
                assertPixelTranslation(estimator.process(moved, START + STEP), 3.0, 2.0)
                estimator.reset()
                val resetBaseline = estimator.process(otherScene, START + 2 * STEP)
                assertNoMotion(resetBaseline, "BASELINE")
                assertNull(resetBaseline.previousFrameNanos)
                assertPixelTranslation(estimator.process(otherMoved, START + 3 * STEP), -2.0, 3.0)

                val afterGap = START + 3 * STEP + FixedCameraVelocityEstimator.MAX_FRAME_GAP_NANOS + 1L
                assertNoMotion(estimator.process(frame, afterGap), "FRAME_GAP")
                val recovered = estimator.process(moved, afterGap + STEP)
                assertPixelTranslation(recovered, 3.0, 2.0)
                assertEquals(afterGap, checkNotNull(recovered.previousFrameNanos))

                assertNoMotion(estimator.process(smaller, afterGap + 2 * STEP), "FRAME_SIZE_CHANGED")
                val resized = estimator.process(smallerMoved, afterGap + 3 * STEP)
                assertPixelTranslation(resized, 4.0, 3.0)
                assertNoScale(resized)
                assertEquals(afterGap + 2 * STEP, checkNotNull(resized.previousFrameNanos))
            }
        } finally {
            frame.release()
            moved.release()
            otherScene.release()
            otherMoved.release()
            smaller.release()
            smallerMoved.release()
        }
    }

    @Test
    fun olderAndEqualTimestampFramesCannotReplaceNewerBaseline() {
        val frame = texturedFloor()
        val moved = translated(frame, 3.0, 2.0)
        val twiceMoved = translated(frame, 6.0, 4.0)
        val unrelated = Mat(HEIGHT, WIDTH, CvType.CV_8UC4, FLOOR_COLOUR)
        try {
            FixedCameraVelocityEstimator().use { estimator ->
                estimator.process(frame, START)
                assertPixelTranslation(estimator.process(moved, START + STEP), 3.0, 2.0)
                assertNoMotion(estimator.process(unrelated, START), "NON_MONOTONIC_FRAME")
                assertNoMotion(estimator.process(unrelated, START + STEP), "NON_MONOTONIC_FRAME")
                val result = estimator.process(twiceMoved, START + 2 * STEP)
                assertPixelTranslation(result, 3.0, 2.0)
                assertEquals(START + STEP, checkNotNull(result.previousFrameNanos))
            }
        } finally {
            frame.release()
            moved.release()
            twiceMoved.release()
            unrelated.release()
        }
    }

    @Test
    fun anisotropicImageDistortionCannotPublishMetricVelocity() {
        val frame = texturedFloor(markers = true)
        val warped = Mat()
        val transform = Mat(2, 3, CvType.CV_64F)
        try {
            transform.put(0, 0, 1.06, 0.0, -0.06 * WIDTH / 2.0, 0.0, 1.0, 0.0)
            Imgproc.warpAffine(frame, warped, transform, frame.size(), Imgproc.INTER_LINEAR,
                Core.BORDER_CONSTANT, FLOOR_COLOUR)
            FixedCameraVelocityEstimator().use { estimator ->
                val baselineTime = establishScale(estimator, frame)
                val distorted = estimator.process(warped, baselineTime + STEP)
                assertNoMotion(distorted, "NON_SIMILARITY_MOTION")
                assertNull(distorted.forwardMps)
                assertNull(distorted.rightMps)
            }
        } finally {
            transform.release()
            frame.release()
            warped.release()
        }
    }

    @Test
    fun markerlessTranslationCannotAcquireMetricScale() {
        val frame = texturedFloor()
        val moved = translated(frame, 3.0, 2.0)
        try {
            FixedCameraVelocityEstimator().use { estimator ->
                estimator.process(frame, START)
                val result = estimator.process(moved, START + STEP)
                assertPixelTranslation(result, 3.0, 2.0)
                assertNoScale(result)
            }
        } finally {
            frame.release()
            moved.release()
        }
    }

    @Test
    fun centredRotationAndZoomDoNotBecomeTranslationWithOffCentreFeatures() {
        // Most support is left of centre: median optical flow would report false translation.
        val frame = texturedFloor(markers = true, offCentreSupport = true)
        val rotated = transformed(frame, angleDegrees = 1.0)
        val zoomed = transformed(frame, imageScale = 1.025)
        val movedAndRotated = transformed(frame, angleDegrees = 1.0, dx = 4.0, dy = 3.0)
        try {
            FixedCameraVelocityEstimator().use { estimator ->
                val rotationBaselineTime = establishScale(estimator, frame)
                val rotation = estimator.process(rotated, rotationBaselineTime + STEP)
                assertMetricTranslation(rotation, 0.0, 0.0, tolerance = 0.35)
                assertEquals(0.0, checkNotNull(rotation.forwardMps), 0.008)
                assertEquals(0.0, checkNotNull(rotation.rightMps), 0.008)

                estimator.reset()
                val zoomBaselineTime = establishScale(estimator, frame)
                val zoom = estimator.process(zoomed, zoomBaselineTime + STEP)
                assertMetricTranslation(zoom, 0.0, 0.0, tolerance = 0.35)
                assertEquals(0.0, checkNotNull(zoom.forwardMps), 0.008)
                assertEquals(0.0, checkNotNull(zoom.rightMps), 0.008)
                assertEquals(1.025, checkNotNull(zoom.imageScale), 0.005)

                estimator.reset()
                val translatedBaselineTime = establishScale(estimator, frame)
                val translated = estimator.process(movedAndRotated, translatedBaselineTime + STEP)
                assertMetricTranslation(translated, 4.0, 3.0, tolerance = 0.35)
                assertEquals(0.06, checkNotNull(translated.forwardMps), 0.008)
                assertEquals(-0.08, checkNotNull(translated.rightMps), 0.008)
            }
        } finally {
            frame.release()
            rotated.release()
            zoomed.release()
            movedAndRotated.release()
        }
    }

    @Test
    fun racingYawAndTravelRemainSignedCurrentBodyVelocityInBothDirections() {
        // Metric precision requires distributed support. Sparse/off-centre support
        // may legitimately fail the unchanged inlier gate; pure yaw is tested below.
        val frame = texturedFloor(markers = true)
        try {
            for (yawDegrees in doubleArrayOf(-6.0, 6.0)) {
                for (direction in doubleArrayOf(-1.0, 1.0)) {
                    val forwardMps = direction * 0.56
                    val rightMps = direction * 0.42
                    // The measured pair is 0.20m / 100px. A 0.07m displacement
                    // over 100ms is 35px, explicitly in the CURRENT body frame.
                    // Static-ground image coordinates obey q = R(p-c) + c + t,
                    // t = (-right, forward) * dt / metersPerPixel. Rotation acts
                    // on the previous features, NOT on t a second time.
                    val moved = transformed(
                        frame, angleDegrees = yawDegrees,
                        dx = -rightMps * 0.1 / 0.002,
                        dy = forwardMps * 0.1 / 0.002,
                    )
                    try {
                        FixedCameraVelocityEstimator().use { estimator ->
                            val baselineTime = establishScale(estimator, frame)
                            val result = estimator.process(moved, baselineTime + STEP)
                            assertMetricTranslation(
                                result, -rightMps * 0.1 / 0.002, forwardMps * 0.1 / 0.002,
                                tolerance = 0.5,
                            )
                            assertEquals(-yawDegrees, checkNotNull(result.rotationDegrees), 0.15)
                            assertEquals(0.002, checkNotNull(result.metersPerPixel), 0.0001)
                            assertEquals(forwardMps, checkNotNull(result.forwardMps), 0.015)
                            // A second 6deg rotation changes these components by
                            // over 0.04m/s, well outside either tolerance.
                            assertEquals(rightMps, checkNotNull(result.rightMps), 0.015)
                        }
                    } finally {
                        moved.release()
                    }
                }
            }
        } finally {
            frame.release()
        }
    }

    @Test
    fun racingPureYawWithOffCentreFeaturesDoesNotBecomeGroundTravel() {
        val frame = texturedFloor(markers = true, offCentreSupport = true)
        try {
            for (yawDegrees in doubleArrayOf(-6.0, 6.0)) {
                val rotated = transformed(frame, angleDegrees = yawDegrees)
                try {
                    FixedCameraVelocityEstimator().use { estimator ->
                        val baselineTime = establishScale(estimator, frame)
                        val result = estimator.process(rotated, baselineTime + STEP)
                        assertMetricTranslation(result, 0.0, 0.0, tolerance = 0.5)
                        assertEquals(-yawDegrees, checkNotNull(result.rotationDegrees), 0.15)
                        assertEquals(0.0, checkNotNull(result.forwardMps), 0.01)
                        assertEquals(0.0, checkNotNull(result.rightMps), 0.01)
                    }
                } finally {
                    rotated.release()
                }
            }
        } finally {
            frame.release()
        }
    }

    @Test
    fun disappearedMarkersExpireAndResetCannotReuseTheirMetricScale() {
        val marked = texturedFloor(markers = true)
        val unmarked = texturedFloor()
        val moved = translated(unmarked, 3.0, 2.0)
        try {
            FixedCameraVelocityEstimator().use { estimator ->
                val baselineTime = establishScale(estimator, marked)
                val fresh = estimator.process(marked, baselineTime + STEP)
                assertMetricTranslation(fresh, 0.0, 0.0)
                val disappeared = estimator.process(unmarked, baselineTime + 2 * STEP)
                assertMetricTranslation(disappeared, 0.0, 0.0)
                // A carried scale must not masquerade as a new marker observation.
                assertNotEquals(fresh.scaleSource, disappeared.scaleSource)
                assertTrue(checkNotNull(disappeared.scaleAgeMs) > 0.0)
                // Adjacent frames keep motion tracking alive while the metric evidence ages out.
                val disappearanceTime = baselineTime + 2 * STEP
                for (index in 1..25) {
                    val tracked = estimator.process(unmarked, disappearanceTime + index * 400_000_000L)
                    if (index == 24) {
                        assertMetricTranslation(tracked, 0.0, 0.0)
                    }
                }
                val expired = estimator.process(moved, disappearanceTime + 26 * 400_000_000L)
                assertPixelTranslation(expired, 3.0, 2.0)
                assertNoScale(expired, "SCALE_EXPIRED")

                estimator.reset()
                val reacquiredTime = establishScale(estimator, marked, baselineTime + 120 * STEP)
                estimator.reset()
                assertNoMotion(estimator.process(unmarked, reacquiredTime + STEP), "BASELINE")
                val afterReset = estimator.process(moved, reacquiredTime + 2 * STEP)
                assertPixelTranslation(afterReset, 3.0, 2.0)
                assertNoScale(afterReset)
            }
        } finally {
            marked.release()
            unmarked.release()
            moved.release()
        }
    }

    private fun establishScale(
        estimator: FixedCameraVelocityEstimator,
        frame: Mat,
        start: Long = START,
    ): Long {
        for (index in 0 until 4) {
            estimator.process(frame, start + index * STEP)
        }
        val lastTime = start + 4 * STEP
        assertMetricTranslation(estimator.process(frame, lastTime), 0.0, 0.0)
        return lastTime
    }

    private fun assertPixelTranslation(
        result: FixedCameraVelocityEstimator.Estimate,
        dx: Double,
        dy: Double,
        tolerance: Double = 0.25
    ) {
        assertTrue(result.toString(), result.motionValid)
        assertEquals(dx, checkNotNull(result.deltaXPixels), tolerance)
        assertEquals(dy, checkNotNull(result.deltaYPixels), tolerance)
    }

    private fun assertMetricTranslation(
        result: FixedCameraVelocityEstimator.Estimate,
        dx: Double,
        dy: Double,
        tolerance: Double = 0.25
    ) {
        assertPixelTranslation(result, dx, dy, tolerance)
        assertTrue(result.toString(), result.valueValid)
    }

    private fun assertNoScale(result: FixedCameraVelocityEstimator.Estimate, reason: String = "NO_SCALE") {
        assertFalse(result.toString(), result.valueValid)
        assertEquals(reason, result.reason)
        assertNull(result.forwardMps)
        assertNull(result.rightMps)
    }

    private fun assertNoMotion(result: FixedCameraVelocityEstimator.Estimate, reason: String) {
        assertFalse(result.toString(), result.motionValid)
        assertFalse(result.toString(), result.valueValid)
        assertEquals(reason, result.reason)
        assertNull(result.forwardMps)
        assertNull(result.rightMps)
    }

    private fun texturedFloor(
        width: Int = WIDTH,
        height: Int = HEIGHT,
        seed: Long = 71431L,
        markers: Boolean = false,
        offCentreSupport: Boolean = false
    ): Mat {
        val random = Random(seed)
        val cellSize = 8
        val cellsWide = (width + cellSize - 1) / cellSize
        val cellsHigh = (height + cellSize - 1) / cellSize
        val shades = IntArray(cellsWide * cellsHigh) { 90 + random.nextInt(136) }
        val bytes = ByteArray(width * height * 4)
        for (y in 0 until height) {
            for (x in 0 until width) {
                val brightness = if (offCentreSupport && x > width * 0.62) {
                    150
                } else {
                    shades[(y / cellSize) * cellsWide + x / cellSize]
                }
                val offset = (y * width + x) * 4
                bytes[offset] = brightness.toByte()
                bytes[offset + 1] = (brightness * 0.68).toInt().toByte()
                bytes[offset + 2] = (brightness * 0.28).toInt().toByte()
                bytes[offset + 3] = 255.toByte()
            }
        }
        return Mat(height, width, CvType.CV_8UC4).also {
            it.put(0, 0, bytes)
            Imgproc.GaussianBlur(it, it, Size(3.0, 3.0), 0.0)
            if (markers) {
                drawMarkerPair(it, width / 2.0, height / 2.0)
            }
        }
    }

    private fun drawMarkerPair(
        frame: Mat,
        x: Double,
        y: Double,
        distancePixels: Double = 100.0 * frame.cols() / WIDTH,
    ) {
        val radius = maxOf(2, (5.0 * frame.cols() / WIDTH).toInt())
        Imgproc.circle(frame, Point(x - distancePixels / 2.0, y), radius, MARKER_COLOUR, -1)
        Imgproc.circle(frame, Point(x + distancePixels / 2.0, y), radius, MARKER_COLOUR, -1)
    }

    private fun translated(source: Mat, dx: Double, dy: Double): Mat =
        transformed(source, dx = dx, dy = dy)

    private fun transformed(
        source: Mat,
        angleDegrees: Double = 0.0,
        imageScale: Double = 1.0,
        dx: Double = 0.0,
        dy: Double = 0.0
    ): Mat {
        val transform = Imgproc.getRotationMatrix2D(
            Point((source.cols() - 1) * 0.5, (source.rows() - 1) * 0.5), angleDegrees, imageScale
        )
        val destination = Mat()
        try {
            transform.put(0, 2, transform.get(0, 2)[0] + dx)
            transform.put(1, 2, transform.get(1, 2)[0] + dy)
            Imgproc.warpAffine(source, destination, transform, source.size(), Imgproc.INTER_LINEAR,
                Core.BORDER_CONSTANT, FLOOR_COLOUR)
            return destination
        } catch (error: Exception) {
            destination.release()
            throw error
        } finally {
            transform.release()
        }
    }

    companion object {
        private const val WIDTH = 848
        private const val HEIGHT = 480
        private const val START = 1_000_000_000L
        private const val STEP = 100_000_000L
        private val FLOOR_COLOUR = Scalar(150.0, 102.0, 42.0, 255.0)
        private val MARKER_COLOUR = Scalar(245.0, 225.0, 30.0, 255.0)
    }
}
