package com.durendal.droneagent.lite

import android.util.Log
import androidx.test.ext.junit.runners.AndroidJUnit4
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import kotlin.math.roundToLong
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith

/**
 * Runs the replacement centerline beside the estimator that flies today, over
 * the scenarios the redesign has to survive, and reports where they disagree.
 *
 * The shadow never touches the returned detection, so a disagreement here costs
 * nothing in the air. What it buys is the evidence needed to justify — or to
 * refuse — the cutover, before any of it drives a motor.
 *
 * The frames are generated. Real captures replace them as soon as a flight has
 * recorded some; until then a pass here means the geometry is sound on shapes we
 * control, not that it is sound on a real floor.
 */
@RunWith(AndroidJUnit4::class)
class TapeShadowPipelineInstrumentedTest {

    private val shadowLines = mutableListOf<String>()
    private val results = LinkedBlockingQueue<Optional>()

    @Test
    fun theReplacementCenterlineSurvivesEveryScenarioTheOldEstimatorDoes() {
        val failures = mutableListOf<String>()
        val steadyStateMillis = mutableListOf<Double>()
        val frameMillis = mutableListOf<Double>()

        // One long-lived detector, as in the app. A detector per scenario would
        // pay the extractor's workspace allocation and JIT warm-up every time and
        // then report that as the steady-state cost of the shadow.
        val detector = BlackTapeDetector(
            onResult = { results.put(Optional(it)) },
            onError = { results.put(Optional(null)) },
            onShadowComparison = { line -> synchronized(shadowLines) { shadowLines += line } },
        )

        detector.use { active ->
            active.setDetectionMode(TapeDetectionMode.PATH)
            repeat(WARMUP_FRAMES) { index -> submit(active, curvedFrame(), index) }
            val coldStart = fieldValues(drainShadowLines(), "totalMs")
            Log.i(TAG, "cold-start shadow cost=${coldStart.firstOrNull()} ms over ${coldStart.size} frames")

            SCENARIOS.forEach { scenario ->
                val outcome = run(scenario, active)
                Log.i(TAG, "scenario=${scenario.name} ${outcome.summary}")
                steadyStateMillis += outcome.shadowMillis
                frameMillis += outcome.frameMillis
                failures += verdict(scenario, outcome)
            }
        }

        val shadowSorted = steadyStateMillis.sorted()
        val frameSorted = frameMillis.sorted()
        assertTrue("the shadow produced no timings at all", shadowSorted.isNotEmpty())
        Log.i(
            TAG,
            "shadow-only n=${shadowSorted.size} p50=${quantile(shadowSorted, 0.50)} " +
                "p95=${quantile(shadowSorted, 0.95)} max=${quantile(shadowSorted, 1.0)} (ms)",
        )
        Log.i(
            TAG,
            "whole frame n=${frameSorted.size} p50=${quantile(frameSorted, 0.50)} " +
                "p95=${quantile(frameSorted, 0.95)} p99=${quantile(frameSorted, 0.99)} " +
                "max=${quantile(frameSorted, 1.0)} (ms) budget=$FRAME_BUDGET_MILLIS",
        )
        assertTrue(failures.joinToString("\n"), failures.isEmpty())
        // The requirement is the intake interval, not a sub-budget: detection plus
        // shadow must fit one frame with room left for a heavier real scene.
        assertTrue(
            "whole-frame p95=${quantile(frameSorted, 0.95)} ms leaves less than the " +
                "required margin inside the $FRAME_BUDGET_MILLIS ms intake interval",
            quantile(frameSorted, 0.95) < FRAME_BUDGET_MILLIS * MAXIMUM_BUDGET_SHARE,
        )
    }

    private fun verdict(scenario: Scenario, outcome: Outcome): List<String> = buildList {
        if (scenario.expectsPath && outcome.oldAccepted == 0) {
            add("${scenario.name}: the old estimator found nothing, the fixture is wrong")
        }
        if (scenario.expectsPath && outcome.oldAccepted > 0 && outcome.newFullPath == 0) {
            add(
                "${scenario.name}: old accepted ${outcome.oldAccepted}, new produced no " +
                    "FULL_PATH (reasons=${outcome.rejectionReasons})",
            )
        }
        if (!scenario.expectsPath && outcome.oldAccepted > 0) {
            add("${scenario.name}: expected no path, old accepted ${outcome.oldAccepted}")
        }
    }

    private fun run(scenario: Scenario, detector: BlackTapeDetector): Outcome {
        drainShadowLines()
        results.clear()
        // Every scenario starts from no tracked path, which a fresh detector used
        // to provide. Toggling the mode is what resets the previous winner.
        detector.setDetectionMode(otherMode(scenario.mode))
        detector.setDetectionMode(scenario.mode)

        var oldAccepted = 0
        scenario.frames().forEachIndexed { index, frame ->
            submit(detector, frame, index)?.let { if (it.value != null) oldAccepted++ }
        }
        val lines = drainShadowLines()
        return Outcome(
            oldAccepted = oldAccepted,
            newFullPath = lines.count { it.contains(" newQuality=FULL_PATH") },
            rejectionReasons = lines
                .mapNotNull { Regex(" newReject=(\\S+)").find(it)?.groupValues?.get(1) }
                .filter { it != "none" }
                .distinct(),
            shadowMillis = fieldValues(lines, "totalMs"),
            frameMillis = fieldValues(lines, "frameMs"),
            sample = lines.firstOrNull().orEmpty(),
        )
    }

    private fun submit(detector: BlackTapeDetector, frame: ByteArray, index: Int): Optional? {
        // submitRgba throttles to one frame per 250 ms and silently ignores the
        // rest, including the first frame of a scenario that follows the previous
        // one too closely.
        Thread.sleep(FRAME_GAP_MILLIS)
        detector.submitRgba(frame, 0, frame.size, WIDTH, HEIGHT)
        return results.poll(20, TimeUnit.SECONDS)
            ?: throw AssertionError("detector did not finish frame $index")
    }

    private fun drainShadowLines(): List<String> = synchronized(shadowLines) {
        val copy = shadowLines.toList()
        shadowLines.clear()
        copy
    }

    private fun fieldValues(lines: List<String>, field: String): List<Double> =
        lines.mapNotNull { line ->
            Regex(" $field=(\\S+)").find(line)?.groupValues?.get(1)?.toDoubleOrNull()
        }

    private fun quantile(sorted: List<Double>, quantile: Double): Double =
        sorted[((sorted.size - 1) * quantile).roundToLong().toInt()]

    private fun otherMode(mode: TapeDetectionMode): TapeDetectionMode =
        if (mode == TapeDetectionMode.PATH) TapeDetectionMode.STRAIGHT else TapeDetectionMode.PATH

    private class Optional(val value: TapeDetection?)

    private class Outcome(
        val oldAccepted: Int,
        val newFullPath: Int,
        val rejectionReasons: List<String>,
        val shadowMillis: List<Double>,
        val frameMillis: List<Double>,
        val sample: String,
    ) {
        val summary: String
            get() = "oldAccepted=$oldAccepted newFullPath=$newFullPath " +
                "rejects=$rejectionReasons | $sample"
    }

    private class Scenario(
        val name: String,
        val mode: TapeDetectionMode,
        val expectsPath: Boolean,
        val frames: () -> List<ByteArray>,
    )

    private companion object {
        const val TAG = "LiteVisionShadow"
        const val WIDTH = 640
        const val HEIGHT = 360
        const val FRAME_GAP_MILLIS = 300L
        const val WARMUP_FRAMES = 3
        const val FRAME_BUDGET_MILLIS = 250.0

        /**
         * Headroom the shadow must leave inside the intake interval. Real floors
         * produce many more contours than these fixtures, so a pass that only
         * just fits would say nothing about flight.
         */
        const val MAXIMUM_BUDGET_SHARE = 0.75

        const val TAPE_HALF_WIDTH = 15.0
        const val CURVE_DISPLACEMENT = 140.0

        val SCENARIOS: List<Scenario> = listOf(
            Scenario("straight", TapeDetectionMode.STRAIGHT, expectsPath = true) {
                listOf(straightFrame())
            },
            Scenario("curved", TapeDetectionMode.PATH, expectsPath = true) {
                listOf(curvedFrame())
            },
            Scenario("glare-break", TapeDetectionMode.PATH, expectsPath = true) {
                listOf(curvedFrame(), glareBrokenCurvedFrame())
            },
            Scenario("dark-floor", TapeDetectionMode.STRAIGHT, expectsPath = true) {
                listOf(straightFrameOnDarkFloor())
            },
            Scenario("endpoint", TapeDetectionMode.STRAIGHT, expectsPath = true) {
                listOf(straightFrame(), shortenedStraightFrame())
            },
            Scenario("lose-then-reacquire", TapeDetectionMode.PATH, expectsPath = true) {
                listOf(curvedFrame(), emptyBoardFrame(), curvedFrame())
            },
            Scenario("no-path", TapeDetectionMode.PATH, expectsPath = false) {
                listOf(emptyBoardFrame())
            },
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
    }
}
