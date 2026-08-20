package com.durendal.droneagent.lite

import android.os.Debug
import android.util.Log
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import java.io.File
import java.util.concurrent.Executors
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import kotlin.math.roundToLong
import kotlin.random.Random
import org.junit.After
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith

/**
 * Per-stage timing on the aircraft's own class of hardware.
 *
 * This exists because a JVM unit test proves nothing about whether a
 * multi-stage image and graph pipeline fits inside the detector's 250 ms frame
 * budget on a phone. It measures the pipeline that flies today, so there is a
 * baseline to compare the redesign against rather than an opinion.
 *
 * What these numbers are NOT: a measurement of real flight frames. The frames
 * here are generated, and generated content is kinder than a real floor —
 * fewer contours survive thresholding, so the segmentation and contour stages
 * are optimistic. The cluttered case narrows that gap but does not close it.
 * Treat every figure below as a lower bound until the same harness is pointed
 * at captures recorded in the air.
 */
@RunWith(AndroidJUnit4::class)
class TapeVisionPerformanceInstrumentedTest {

    private val captureWriter = Executors.newSingleThreadExecutor()

    private val root: File =
        File(
            InstrumentationRegistry.getInstrumentation().targetContext.cacheDir,
            "tape-performance-test",
        ).apply {
            deleteRecursively()
            mkdirs()
        }

    @After
    fun tearDown() {
        captureWriter.shutdownNow()
        root.deleteRecursively()
    }

    @Test
    fun currentPipelineFitsTheFrameBudget() {
        val clean = report("detect/clean", detectSamples(cluttered = false, captureArmed = false))
        val cluttered = report("detect/cluttered", detectSamples(cluttered = true, captureArmed = false))

        assertTrue(
            "clean p95=${clean.p95Millis} ms exceeds the ${FRAME_BUDGET_MILLIS} ms frame budget",
            clean.p95Millis < FRAME_BUDGET_MILLIS,
        )
        assertTrue(
            "cluttered p95=${cluttered.p95Millis} ms exceeds the ${FRAME_BUDGET_MILLIS} ms frame budget",
            cluttered.p95Millis < FRAME_BUDGET_MILLIS,
        )
    }

    /**
     * What the detector thread pays for having capture armed, with the writer on
     * its own thread as in the app. This is the number that decides whether an
     * operator may arm capture in flight at all.
     */
    @Test
    fun armingCaptureStaysInsideTheFrameBudget() {
        val off = report("detect/capture-off", detectSamples(cluttered = true, captureArmed = false))
        val on = report("detect/capture-on", detectSamples(cluttered = true, captureArmed = true))

        Log.i(TAG, "capture cost delta p95=${on.p95Millis - off.p95Millis} ms")
        assertTrue(
            "armed p95=${on.p95Millis} ms exceeds the ${FRAME_BUDGET_MILLIS} ms frame budget",
            on.p95Millis < FRAME_BUDGET_MILLIS,
        )
    }

    /**
     * How fast captures can actually reach storage, measured on its own because
     * the detector never waits for it. If one capture costs more than a frame
     * interval, arming capture means a permanently lagging, dropping writer, and
     * the evidence an operator believes they are collecting is partial.
     */
    @Test
    fun captureWriteThroughputKeepsUpWithTheDetector() {
        val store = TapeCaptureStore(root, maximumCaptures = 200, maximumTotalBytes = Long.MAX_VALUE)
        val capture = fullCapture()
        repeat(2) { store.save(capture, sequence = it.toLong(), reason = "warmup") }

        val samples = LongArray(WRITE_SAMPLES) { index ->
            val started = System.nanoTime()
            store.save(capture, sequence = (index + 100).toLong(), reason = "measure")
            System.nanoTime() - started
        }
        val measurement = report("store.save/${FRAME_WIDTH}x$FRAME_HEIGHT", samples)
        val bytesPerCapture = store.totalBytes() / store.captures().size
        val rawBytes = FRAME_WIDTH.toLong() * FRAME_HEIGHT * 4 +
            4L * ANALYSIS_WIDTH * ANALYSIS_HEIGHT
        Log.i(
            TAG,
            "capture size=%.2f MiB raw=%.2f MiB ratio=%.2f -> %d captures fill the %d MiB ceiling"
                .format(
                    bytesPerCapture / 1048576.0,
                    rawBytes / 1048576.0,
                    rawBytes.toDouble() / bytesPerCapture,
                    TapeCaptureStore.DEFAULT_MAXIMUM_TOTAL_BYTES / bytesPerCapture,
                    TapeCaptureStore.DEFAULT_MAXIMUM_TOTAL_BYTES / 1048576,
                ),
        )
        Log.i(
            TAG,
            "writer capacity=%.2f captures/s vs detector %.1f frames/s".format(
                1000.0 / measurement.p50Millis,
                1000.0 / FRAME_BUDGET_MILLIS,
            ),
        )

        assertTrue(
            "one capture costs p50=${measurement.p50Millis} ms, so the writer cannot keep up " +
                "with a ${FRAME_BUDGET_MILLIS} ms frame interval",
            measurement.p50Millis < FRAME_BUDGET_MILLIS,
        )
    }

    /** A capture the size the detector really produces: full frame plus four masks. */
    private fun fullCapture(): TapeCapture = TapeCapture(
        metadata = mapOf("detection" to "accepted"),
        frame = TapeCapturePlane(
            name = TapeCapture.FRAME_PLANE_NAME,
            width = FRAME_WIDTH,
            height = FRAME_HEIGHT,
            channels = 4,
            pixels = clutteredFrame(),
        ),
        masks = listOf("blackMask", "floorMask", "bridgedBlackMask", "cleanedBlackMask")
            .map { name ->
                TapeCapturePlane(
                    name = name,
                    width = ANALYSIS_WIDTH,
                    height = ANALYSIS_HEIGHT,
                    channels = 1,
                    pixels = ByteArray(ANALYSIS_WIDTH * ANALYSIS_HEIGHT) {
                        if (it % 7 == 0) 255.toByte() else 0
                    },
                )
            },
    )

    /**
     * The centerline extractor on its own, fed a mask the size the detector
     * actually produces. This is the number that decides whether stage three can
     * add the new geometry layer at all, so it is measured before anything is
     * wired to it.
     */
    @Test
    fun centerlineExtractionFitsTheFrameBudget() {
        val extractor = CenterlineExtractor()
        val mask = curvedRibbonMask(ANALYSIS_WIDTH, ANALYSIS_HEIGHT)
        repeat(WARMUP_FRAMES) { extractor.extract(mask) }

        val gcBefore = gcCount()
        val samples = LongArray(MEASURED_FRAMES) {
            val started = System.nanoTime()
            val estimate = extractor.extract(mask)
            val elapsed = System.nanoTime() - started
            assertTrue("extractor produced no centerline", estimate.points.isNotEmpty())
            elapsed
        }
        val measurement = report("centerline/${ANALYSIS_WIDTH}x$ANALYSIS_HEIGHT", samples)
        Log.i(TAG, "centerline gc-count delta=${gcCount() - gcBefore}")

        assertTrue(
            "centerline p95=${measurement.p95Millis} ms exceeds the ${FRAME_BUDGET_MILLIS} ms frame budget",
            measurement.p95Millis < FRAME_BUDGET_MILLIS,
        )
    }

    private fun detectSamples(cluttered: Boolean, captureArmed: Boolean): LongArray {
        val frame = if (cluttered) clutteredFrame() else cleanFrame()
        val recorder = if (captureArmed) {
            TapeCaptureRecorder(
                root = root,
                log = {},
                leadingFrameCount = 4,
                trailingFrameCount = 4,
                store = TapeCaptureStore(root),
                // A background writer, as in the app: the point is what the
                // detector thread pays, not what storage costs.
                writer = captureWriter,
            ).also { it.arm() }
        } else {
            null
        }
        val results = LinkedBlockingQueue<Optional>()
        val detector = BlackTapeDetector(
            onResult = { results.put(Optional(it)) },
            onError = { results.put(Optional(null)) },
            captureRecorder = recorder,
        )
        val samples = LongArray(MEASURED_FRAMES)
        detector.use {
            it.setDetectionMode(TapeDetectionMode.PATH)
            repeat(WARMUP_FRAMES + MEASURED_FRAMES) { index ->
                // submitRgba throttles to one frame per 250 ms, so the timing is
                // taken around a direct replay call instead: same detect() path,
                // no waiting for a clock this test is not measuring.
                val started = System.nanoTime()
                it.replay(captureOf(frame))
                val elapsed = System.nanoTime() - started
                if (index >= WARMUP_FRAMES) samples[index - WARMUP_FRAMES] = elapsed
            }
        }
        recorder?.let { armed ->
            Log.i(
                TAG,
                "capture saved=${armed.savedCaptureCount} dropped=${armed.droppedCaptureCount} " +
                    "failed=${armed.failedCaptureCount} pending=${armed.pendingCaptureCount}",
            )
            armed.disarm()
        }
        assertNotNull(samples)
        return samples
    }

    private fun captureOf(frame: ByteArray) = TapeCapture(
        metadata = emptyMap(),
        frame = TapeCapturePlane(
            name = TapeCapture.FRAME_PLANE_NAME,
            width = FRAME_WIDTH,
            height = FRAME_HEIGHT,
            channels = 4,
            pixels = frame,
        ),
    )

    private fun report(label: String, samples: LongArray): Measurement {
        val sorted = samples.sortedArray()
        val measurement = Measurement(
            p50Millis = millis(sorted, 0.50),
            p95Millis = millis(sorted, 0.95),
            p99Millis = millis(sorted, 0.99),
            maxMillis = millis(sorted, 1.0),
        )
        Log.i(
            TAG,
            "$label n=${samples.size} p50=${measurement.p50Millis} p95=${measurement.p95Millis} " +
                "p99=${measurement.p99Millis} max=${measurement.maxMillis} (ms)",
        )
        return measurement
    }

    private fun millis(sortedNanos: LongArray, quantile: Double): Double {
        val index = ((sortedNanos.size - 1) * quantile).roundToLong().toInt()
        return sortedNanos[index] / 1_000_000.0
    }

    private fun gcCount(): Long =
        Debug.getRuntimeStat("art.gc.gc-count")?.toLongOrNull() ?: -1L

    /** Board plus one curved tape: the friendliest input the detector ever sees. */
    private fun cleanFrame(): ByteArray = boardFrame().also { addCurvedTape(it) }

    /**
     * Adds noise and unrelated dark patches so thresholding yields many contours
     * instead of one. Still generated, but closer to the contour counts a real
     * floor produces than a single clean strip is.
     */
    private fun clutteredFrame(): ByteArray {
        val frame = boardFrame()
        val random = Random(CLUTTER_SEED)
        repeat(CLUTTER_PATCH_COUNT) {
            val left = random.nextInt(FRAME_WIDTH - 120)
            val top = random.nextInt(FRAME_HEIGHT - 120)
            val width = 30 + random.nextInt(90)
            val height = 30 + random.nextInt(90)
            for (y in top until minOf(top + height, FRAME_HEIGHT)) {
                for (x in left until minOf(left + width, FRAME_WIDTH)) {
                    val base = (y * FRAME_WIDTH + x) * 4
                    frame[base] = 45
                    frame[base + 1] = 40
                    frame[base + 2] = 35
                }
            }
        }
        for (index in 0 until FRAME_WIDTH * FRAME_HEIGHT) {
            val jitter = random.nextInt(-18, 19)
            val base = index * 4
            frame[base] = ((frame[base].toInt() and 0xFF) + jitter).coerceIn(0, 255).toByte()
            frame[base + 1] = ((frame[base + 1].toInt() and 0xFF) + jitter).coerceIn(0, 255).toByte()
            frame[base + 2] = ((frame[base + 2].toInt() and 0xFF) + jitter).coerceIn(0, 255).toByte()
        }
        addCurvedTape(frame)
        return frame
    }

    private fun boardFrame(): ByteArray = ByteArray(FRAME_WIDTH * FRAME_HEIGHT * 4).also { frame ->
        for (index in 0 until FRAME_WIDTH * FRAME_HEIGHT) {
            frame[index * 4] = 180.toByte()
            frame[index * 4 + 1] = 120.toByte()
            frame[index * 4 + 2] = 50.toByte()
            frame[index * 4 + 3] = 255.toByte()
        }
    }

    private fun addCurvedTape(frame: ByteArray) {
        val halfWidth = FRAME_WIDTH * TAPE_WIDTH_FRACTION / 2.0
        for (y in 0 until FRAME_HEIGHT) {
            val forward = (FRAME_HEIGHT - 1 - y) / (FRAME_HEIGHT - 1.0)
            val center = FRAME_WIDTH / 2.0 + FRAME_WIDTH * CURVE_FRACTION * forward * forward
            val left = (center - halfWidth).toInt().coerceAtLeast(0)
            val right = (center + halfWidth).toInt().coerceAtMost(FRAME_WIDTH)
            for (x in left until right) {
                val base = (y * FRAME_WIDTH + x) * 4
                frame[base] = 20
                frame[base + 1] = 20
                frame[base + 2] = 20
            }
        }
    }

    private fun curvedRibbonMask(width: Int, height: Int): SegmentationMask {
        val tape = BooleanArray(width * height)
        val halfWidth = width * TAPE_WIDTH_FRACTION / 2.0
        for (y in 0 until height) {
            val forward = (height - 1 - y) / (height - 1.0)
            val center = width / 2.0 + width * CURVE_FRACTION * forward * forward
            val left = (center - halfWidth).toInt().coerceAtLeast(0)
            val right = (center + halfWidth).toInt().coerceAtMost(width)
            for (x in left until right) tape[y * width + x] = true
        }
        return SegmentationMask(width, height, tape)
    }

    private class Optional(val value: TapeDetection?)

    private data class Measurement(
        val p50Millis: Double,
        val p95Millis: Double,
        val p99Millis: Double,
        val maxMillis: Double,
    )

    private companion object {
        const val TAG = "LiteVisionPerf"

        /** The detector accepts at most one frame per 250 ms. */
        const val FRAME_BUDGET_MILLIS = 250.0

        /** The DJI RGBA stream this app consumes. */
        const val FRAME_WIDTH = 1920
        const val FRAME_HEIGHT = 1080

        /** What detect() resizes to, and therefore the size a mask really is. */
        const val ANALYSIS_WIDTH = 640
        const val ANALYSIS_HEIGHT = 360

        const val WARMUP_FRAMES = 5
        const val MEASURED_FRAMES = 40
        const val TAPE_WIDTH_FRACTION = 0.05
        const val CURVE_FRACTION = 0.22
        const val CLUTTER_PATCH_COUNT = 40
        const val CLUTTER_SEED = 20260820
        const val WRITE_SAMPLES = 10
    }
}
