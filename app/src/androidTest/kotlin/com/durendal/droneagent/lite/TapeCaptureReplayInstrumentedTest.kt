package com.durendal.droneagent.lite

import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import java.io.File
import java.util.concurrent.Executor
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import org.junit.After
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith

/**
 * The point of recording frames is that a failure can be re-run against the
 * current build. That only holds if a stored capture reproduces the detection
 * the live pipeline reached, so these tests exercise the whole loop: detect a
 * frame with recording armed, read the capture back off storage, and replay it.
 */
@RunWith(AndroidJUnit4::class)
class TapeCaptureReplayInstrumentedTest {

    private val root: File =
        File(
            InstrumentationRegistry.getInstrumentation().targetContext.cacheDir,
            "tape-capture-replay-test",
        ).apply {
            deleteRecursively()
            mkdirs()
        }

    @After
    fun tearDown() {
        root.deleteRecursively()
    }

    @Test
    fun storedCaptureReplaysToTheLiveDetection() {
        val width = 640
        val height = 360
        val frame = frameWithVerticalTape(width, height)

        val live = detectWithCapture(frame, width, height)
        val capture = singleStoredCapture()

        assertEquals(width, capture.frame.width)
        assertEquals(height, capture.frame.height)
        assertArrayEquals(frame, capture.frame.pixels)

        val replayed = replay(capture)
        assertNotNull("replay produced no detection", replayed)
        assertSameDetection(live, replayed!!)
    }

    @Test
    fun replayingTheSameCaptureTwiceAgrees() {
        val width = 640
        val height = 360
        detectWithCapture(frameWithVerticalTape(width, height), width, height)
        val capture = singleStoredCapture()

        val first = replay(capture)
        val second = replay(capture)

        assertNotNull(first)
        assertNotNull(second)
        assertSameDetection(first!!, second!!)
    }

    @Test
    fun aCaptureCarriesTheMasksAndTheVerdictThatExplainTheFrame() {
        val width = 640
        val height = 360
        detectWithCapture(frameWithVerticalTape(width, height), width, height)

        val capture = singleStoredCapture()

        assertEquals(
            listOf("blackMask", "floorMask", "bridgedBlackMask", "cleanedBlackMask"),
            capture.masks.map { it.name },
        )
        capture.masks.forEach { mask ->
            assertEquals(mask.name, 1, mask.channels)
            assertTrue("${mask.name} is empty", mask.pixels.any { it != 0.toByte() })
        }
        assertEquals("accepted", capture.metadata["detection"])
        assertEquals("PATH", capture.metadata["detector.mode"])
        assertTrue(
            capture.metadata.toString(),
            capture.metadata.containsKey("detector.diagnostics"),
        )
        assertTrue(
            capture.metadata.toString(),
            capture.metadata.containsKey("frame.acceptedAtNanos"),
        )
    }

    private fun assertSameDetection(expected: TapeDetection, actual: TapeDetection) {
        assertEquals(expected.confidence, actual.confidence, 0.0)
        assertEquals(expected.angleFromVerticalDegrees, actual.angleFromVerticalDegrees, 0.0)
        assertEquals(expected.longSideFraction, actual.longSideFraction, 0.0)
        assertEquals(expected.nearFieldOffsetFraction, actual.nearFieldOffsetFraction, 0.0)
        assertEquals(expected.anchorXFraction, actual.anchorXFraction, 0.0)
        assertEquals(expected.anchorYFraction, actual.anchorYFraction, 0.0)
        assertEquals(expected.lookaheadXFraction, actual.lookaheadXFraction)
        assertEquals(expected.lookaheadYFraction, actual.lookaheadYFraction)
        assertEquals(expected.quality, actual.quality)
        assertEquals(expected.bounds, actual.bounds)
    }

    /**
     * Runs one frame through a live detector with recording armed, then flushes
     * the recorder's window so the frame reaches storage.
     */
    private fun detectWithCapture(frame: ByteArray, width: Int, height: Int): TapeDetection {
        val recorder = TapeCaptureRecorder(
            root = root,
            log = {},
            leadingFrameCount = 1,
            trailingFrameCount = 0,
            store = TapeCaptureStore(root),
            writer = Executor { it.run() },
        )
        val results = LinkedBlockingQueue<Optional>()
        val detector = BlackTapeDetector(
            onResult = { results.put(Optional(it)) },
            onError = { results.put(Optional(null)) },
            captureRecorder = recorder,
            captureFlightContext = { mapOf("gimbal.pitchDegrees" to "-90.0") },
        )
        return detector.use {
            recorder.arm()
            it.submitRgba(frame, 0, frame.size, width, height)
            val result = results.poll(20, TimeUnit.SECONDS)
            recorder.trigger("test")
            recorder.disarm()
            assertNotNull("detector produced no result", result)
            assertNotNull("detector rejected the synthetic tape", result!!.value)
            result.value!!
        }
    }

    private fun replay(capture: TapeCapture): TapeDetection? =
        BlackTapeDetector(onResult = {}).use { it.replay(capture) }

    private fun singleStoredCapture(): TapeCapture {
        val directories = TapeCaptureStore(root).captures()
        assertEquals("expected exactly one stored capture", 1, directories.size)
        return TapeCaptureCodec.read(directories.single())
    }

    /** Brown board with one dark vertical strip: the detector's simplest accept. */
    private fun frameWithVerticalTape(width: Int, height: Int): ByteArray {
        val frame = ByteArray(width * height * 4)
        for (index in 0 until width * height) {
            frame[index * 4] = 180.toByte()
            frame[index * 4 + 1] = 120.toByte()
            frame[index * 4 + 2] = 50.toByte()
            frame[index * 4 + 3] = 255.toByte()
        }
        for (y in 0 until height) {
            for (x in 305 until 335) {
                val base = (y * width + x) * 4
                frame[base] = 20
                frame[base + 1] = 20
                frame[base + 2] = 20
            }
        }
        return frame
    }

    /** LinkedBlockingQueue rejects nulls; a rejected frame is a real outcome. */
    private class Optional(val value: TapeDetection?)
}
