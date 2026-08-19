package com.durendal.droneagent.lite

import android.graphics.BitmapFactory
import android.util.Log
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import kotlin.math.roundToLong
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith

/**
 * The production pipeline, centerline and all, over every scenario it has to
 * survive — including the two real camera frames this repository already holds.
 *
 * Those two are photographs of the actual working surface, not drawings, so they
 * are the only inputs here that can disagree with an assumption. The generated
 * ones cover shapes no recorded frame exists for yet; where a claim rests on
 * them it rests on geometry we chose, which is weaker evidence and is called out
 * as such rather than counted as flight validation.
 */
@RunWith(AndroidJUnit4::class)
class TapeCenterlinePipelineInstrumentedTest {

    // The recorded frames ship in the test APK, so they are read from the test
    // context; targetContext is the app under test and has no such assets.
    private val testContext = InstrumentationRegistry.getInstrumentation().context

    @Test
    fun everyScenarioProducesTheQualityItShould() {
        val failures = mutableListOf<String>()
        val frameMillis = mutableListOf<Double>()
        val detector = detector()

        detector.use { active ->
            active.setDetectionMode(TapeDetectionMode.PATH)
            repeat(WARMUP_FRAMES) { submit(active, curvedFrame()) }

            SCENARIOS.forEach { scenario ->
                val outcome = run(scenario, active)
                Log.i(TAG, "scenario=${scenario.name} $outcome")
                frameMillis += outcome.millis
                val actual = outcome.finalQuality
                if (actual != scenario.expected) {
                    failures += "${scenario.name}: expected ${scenario.expected}, got $actual " +
                        "(${outcome.detail})"
                }
            }
        }

        val sorted = frameMillis.sorted()
        Log.i(
            TAG,
            "production frame n=${sorted.size} p50=${quantile(sorted, 0.50)} " +
                "p95=${quantile(sorted, 0.95)} p99=${quantile(sorted, 0.99)} " +
                "max=${quantile(sorted, 1.0)} (ms) budget=$FRAME_BUDGET_MILLIS",
        )
        assertTrue(failures.joinToString("\n"), failures.isEmpty())
        assertTrue(
            "whole-frame p95=${quantile(sorted, 0.95)} ms leaves too little of the " +
                "$FRAME_BUDGET_MILLIS ms intake interval",
            quantile(sorted, 0.95) < FRAME_BUDGET_MILLIS * MAXIMUM_BUDGET_SHARE,
        )
    }

    /** The recorded working surface: a real photograph, not a drawing. */
    @Test
    fun theRecordedCardboardSceneIsFollowedAsAFullPath() {
        val detector = detector()
        val detection = detector.use { active ->
            active.setDetectionMode(TapeDetectionMode.PATH)
            submit(active, rgbaAsset("final-cardboard-tape.jpg"))
        }

        assertNotNull(
            "the recorded scene must still be detected: " + detector.diagnosticsSummary(),
            detection,
        )
        requireNotNull(detection)
        assertEquals(PathQuality.FULL_PATH, detection.quality)
        val centerline = requireNotNull(detection.centerline)
        assertTrue("a real scene should trace many points", centerline.pointCount >= 20)
        assertEquals(0, centerline.branchCount)
        Log.i(
            TAG,
            "cardboard points=${centerline.pointCount} quality=${detection.quality} " +
                "anchor=${detection.anchorXFraction} endpoint=${detection.endpointCandidate}",
        )
    }

    /** The recorded false positive: a wall/floor edge that must stay rejected. */
    @Test
    fun theRecordedWallFloorEdgeIsStillRejected() {
        val detector = detector()
        val detection = detector.use { active ->
            active.setDetectionMode(TapeDetectionMode.PATH)
            submit(active, rgbaAsset("straight-wall-floor-edge.jpg"))
        }

        assertNull("a wall edge is not a curved tape path", detection)
    }

    @Test
    fun aBranchIsNeverGivenAuthorityToAdvance() {
        val detector = detector()
        // PATH mode: its opening kernel is isotropic, so a junction survives to
        // reach the geometry stage at all. STRAIGHT mode opens with a 3x31
        // vertical element and erases a horizontal arm before contours are found.
        val detection = detector.use { active ->
            active.setDetectionMode(TapeDetectionMode.PATH)
            submit(active, branchFrame())
        }
        Log.i(TAG, "branch " + detector.diagnosticsSummary())

        // Either the junction is rejected outright or it is admitted without a
        // look-ahead. What it may never be is a full path.
        if (detection != null) {
            assertEquals(
                "a junction must not become a full path",
                PathQuality.NEAR_FIELD_ONLY,
                detection.quality,
            )
            assertNull(detection.lookahead)
        }
    }

    private fun run(scenario: Scenario, detector: BlackTapeDetector): Outcome {
        detector.setDetectionMode(otherMode(scenario.mode))
        detector.setDetectionMode(scenario.mode)
        val millis = mutableListOf<Double>()
        var last: TapeDetection? = null
        scenario.frames().forEach { frame ->
            // The intake throttle sleep must sit outside the timed section, or
            // the number measured is the test's own pacing, not the pipeline.
            Thread.sleep(FRAME_GAP_MILLIS)
            val started = System.nanoTime()
            last = submitNow(detector, frame)
            millis += (System.nanoTime() - started) / 1_000_000.0
        }
        return Outcome(
            finalQuality = last?.quality,
            millis = millis,
            detail = last?.centerline?.label ?: detector.diagnosticsSummary(),
        )
    }

    private fun submit(detector: BlackTapeDetector, frame: ByteArray): TapeDetection? {
        Thread.sleep(FRAME_GAP_MILLIS)
        return submitNow(detector, frame)
    }

    private fun submitNow(detector: BlackTapeDetector, frame: ByteArray): TapeDetection? {
        detector.submitRgba(frame, 0, frame.size, WIDTH, HEIGHT)
        val outcome = results.poll(20, TimeUnit.SECONDS)
            ?: throw AssertionError("detector did not finish a frame")
        return outcome.value
    }

    private val results = LinkedBlockingQueue<Optional>()

    private fun detector() = BlackTapeDetector(
        onResult = { results.put(Optional(it)) },
        onError = { results.put(Optional(null)) },
    )

    private fun quantile(sorted: List<Double>, quantile: Double): Double =
        if (sorted.isEmpty()) 0.0 else sorted[((sorted.size - 1) * quantile).roundToLong().toInt()]

    private fun otherMode(mode: TapeDetectionMode): TapeDetectionMode =
        if (mode == TapeDetectionMode.PATH) TapeDetectionMode.STRAIGHT else TapeDetectionMode.PATH

    private fun rgbaAsset(name: String): ByteArray {
        val bitmap = testContext.assets.open(name).use(BitmapFactory::decodeStream)
        val scaled = android.graphics.Bitmap.createScaledBitmap(bitmap, WIDTH, HEIGHT, true)
        val frame = ByteArray(WIDTH * HEIGHT * 4)
        val pixels = IntArray(WIDTH * HEIGHT)
        scaled.getPixels(pixels, 0, WIDTH, 0, 0, WIDTH, HEIGHT)
        pixels.forEachIndexed { index, pixel ->
            frame[index * 4] = ((pixel shr 16) and 0xFF).toByte()
            frame[index * 4 + 1] = ((pixel shr 8) and 0xFF).toByte()
            frame[index * 4 + 2] = (pixel and 0xFF).toByte()
            frame[index * 4 + 3] = 255.toByte()
        }
        return frame
    }

    private class Optional(val value: TapeDetection?)

    private class Outcome(
        val finalQuality: PathQuality?,
        val millis: List<Double>,
        val detail: String,
    ) {
        override fun toString(): String = "quality=$finalQuality frames=${millis.size} | $detail"
    }

    private class Scenario(
        val name: String,
        val mode: TapeDetectionMode,
        val expected: PathQuality?,
        val frames: () -> List<ByteArray>,
    )

    private companion object {
        const val TAG = "LiteVisionPipeline"
        const val WIDTH = 640
        const val HEIGHT = 360
        const val FRAME_GAP_MILLIS = 300L
        const val WARMUP_FRAMES = 3
        const val FRAME_BUDGET_MILLIS = 250.0
        const val MAXIMUM_BUDGET_SHARE = 0.75
        const val TAPE_HALF_WIDTH = 15.0
        const val CURVE_DISPLACEMENT = 140.0

        val SCENARIOS: List<Scenario> = listOf(
            Scenario("straight", TapeDetectionMode.STRAIGHT, PathQuality.FULL_PATH) {
                listOf(straightFrame())
            },
            Scenario("curved", TapeDetectionMode.PATH, PathQuality.FULL_PATH) {
                listOf(curvedFrame())
            },
            Scenario("glare-break", TapeDetectionMode.PATH, PathQuality.FULL_PATH) {
                listOf(curvedFrame(), glareBrokenCurvedFrame())
            },
            Scenario("dark-floor", TapeDetectionMode.STRAIGHT, PathQuality.FULL_PATH) {
                listOf(straightFrameOnDarkFloor())
            },
            Scenario("board-edge", TapeDetectionMode.STRAIGHT, PathQuality.FULL_PATH) {
                listOf(straightFrameWithBoardEdge())
            },
            Scenario("endpoint", TapeDetectionMode.STRAIGHT, PathQuality.FULL_PATH) {
                listOf(straightFrame(), shortenedStraightFrame())
            },
            Scenario("short-horizontal-residue", TapeDetectionMode.STRAIGHT, null) {
                listOf(shortHorizontalResidueFrame())
            },
            Scenario("lose-then-reacquire", TapeDetectionMode.PATH, PathQuality.FULL_PATH) {
                listOf(curvedFrame(), emptyBoardFrame(), curvedFrame())
            },
            Scenario("no-path", TapeDetectionMode.PATH, null) { listOf(emptyBoardFrame()) },
        )

        fun board(red: Int = 180, green: Int = 120, blue: Int = 50): ByteArray =
            ByteArray(WIDTH * HEIGHT * 4).also { frame ->
                for (pixel in 0 until WIDTH * HEIGHT) {
                    val offset = pixel * 4
                    frame[offset] = red.toByte()
                    frame[offset + 1] = green.toByte()
                    frame[offset + 2] = blue.toByte()
                    frame[offset + 3] = 255.toByte()
                }
            }

        fun fillRect(frame: ByteArray, left: Int, top: Int, right: Int, bottom: Int, value: Int) {
            for (y in top.coerceAtLeast(0) until bottom.coerceAtMost(HEIGHT)) {
                for (x in left.coerceAtLeast(0) until right.coerceAtMost(WIDTH)) {
                    val offset = (y * WIDTH + x) * 4
                    frame[offset] = value.toByte()
                    frame[offset + 1] = value.toByte()
                    frame[offset + 2] = value.toByte()
                }
            }
        }

        fun emptyBoardFrame(): ByteArray = board()

        fun straightFrame(): ByteArray = board().also { fillRect(it, 305, 0, 335, HEIGHT, 20) }

        fun shortenedStraightFrame(): ByteArray =
            board().also { fillRect(it, 305, 150, 335, HEIGHT, 20) }

        fun straightFrameOnDarkFloor(): ByteArray =
            board(red = 96, green = 88, blue = 80).also { fillRect(it, 305, 0, 335, HEIGHT, 20) }

        /** Tape running beside the edge of the board it is stuck to. */
        fun straightFrameWithBoardEdge(): ByteArray = board().also { frame ->
            fillRect(frame, 0, 0, 120, HEIGHT, 70)
            fillRect(frame, 305, 0, 335, HEIGHT, 20)
        }

        /** A stub of tape lying across the frame: never enough to fly along. */
        fun shortHorizontalResidueFrame(): ByteArray =
            board().also { fillRect(it, 250, 300, 390, 326, 20) }

        fun curvedFrame(): ByteArray = board().also { frame ->
            for (y in 0 until HEIGHT) {
                val forward = (HEIGHT - 1 - y) / (HEIGHT - 1.0)
                val center = WIDTH / 2.0 + CURVE_DISPLACEMENT * forward * forward
                fillRect(
                    frame,
                    (center - TAPE_HALF_WIDTH).toInt(),
                    y,
                    (center + TAPE_HALF_WIDTH).toInt(),
                    y + 1,
                    20,
                )
            }
        }

        fun glareBrokenCurvedFrame(): ByteArray = curvedFrame().also { frame ->
            for (y in 150 until 168) {
                val forward = (HEIGHT - 1 - y) / (HEIGHT - 1.0)
                val center = WIDTH / 2.0 + CURVE_DISPLACEMENT * forward * forward
                fillRect(
                    frame,
                    (center - TAPE_HALF_WIDTH).toInt(),
                    y,
                    (center + TAPE_HALF_WIDTH).toInt(),
                    y + 1,
                    220,
                )
            }
        }

        /**
         * A curved trunk with a real arm leaving it. Curved so the arc gate lets
         * it through to the geometry stage, and thick enough to survive opening,
         * because the question here is what the geometry does with a junction —
         * not whether morphology happens to delete one.
         */
        fun branchFrame(): ByteArray = curvedFrame().also { frame ->
            val forward = (HEIGHT - 1 - 200) / (HEIGHT - 1.0)
            val center = WIDTH / 2.0 + CURVE_DISPLACEMENT * forward * forward
            fillRect(frame, center.toInt(), 180, WIDTH - 20, 220, 20)
        }
    }
}
