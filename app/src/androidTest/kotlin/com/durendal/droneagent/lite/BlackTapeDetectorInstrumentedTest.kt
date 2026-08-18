package com.durendal.droneagent.lite

import androidx.test.ext.junit.runners.AndroidJUnit4
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith

@RunWith(AndroidJUnit4::class)
class BlackTapeDetectorInstrumentedTest {
    @Test
    fun verticalTapeRemainsDetectableAcrossDarkHorizontalSeam() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillRect(frame, width, left = 305, top = 0, right = 335, bottom = height, value = 20)
        fillRect(frame, width, left = 0, top = 165, right = width, bottom = 185, value = 40)

        val detection = detect(frame, width, height)
        assertEquals(0.0, detection.angleFromVerticalDegrees, 1.0)
        assertTrue(detection.longSideFraction >= 0.95)
        assertEquals(0.0, detection.nearFieldOffsetFraction, 0.01)
    }

    @Test
    fun verticalTapeRemainsDetectableAcrossBrightGlareGaps() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillRect(frame, width, left = 305, top = 0, right = 335, bottom = height, value = 20)
        fillRect(frame, width, left = 305, top = 155, right = 335, bottom = 167, value = 220)
        fillRect(frame, width, left = 305, top = 178, right = 335, bottom = 190, value = 220)

        val detection = detect(frame, width, height)
        assertEquals(0.0, detection.angleFromVerticalDegrees, 1.0)
        assertTrue(detection.longSideFraction >= 0.95)
        assertEquals(0.0, detection.nearFieldOffsetFraction, 0.01)
    }

    @Test
    fun trackedTapeRemainsDetectableAsOnlyItsShortTerminalSegmentRemains() {
        val width = 640
        val height = 360
        val fullTape = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillRect(fullTape, width, left = 305, top = 0, right = 335, bottom = height, value = 20)
        val terminalTape = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillRect(terminalTape, width, left = 300, top = 290, right = 340, bottom = height, value = 20)

        val detections =
            detectSequence(listOf(fullTape, terminalTape), width, height, TapeDetectionMode.STRAIGHT)
        assertTrue(detections[0] != null)
        val terminal = checkNotNull(detections[1])
        assertEquals(0.0, terminal.angleFromVerticalDegrees, 1.0)
        assertTrue(terminal.longSideFraction < 0.25)
        assertEquals(0.0, terminal.nearFieldOffsetFraction, 0.01)
    }

    @Test
    fun previewModeDetectsCurvedTapeWithoutStartingTracking() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillCurvedRibbon(frame, width, height, direction = 1.0, value = 20)

        val detection = checkNotNull(
            detectSequence(
                frames = listOf(frame),
                width = width,
                height = height,
            ).single(),
        )

        assertTrue(detection.angleFromVerticalDegrees > 10.0)
        assertEquals(0.0, detection.nearFieldOffsetFraction, 0.02)
        assertTrue(detection.longSideFraction > 0.9)
    }

    @Test
    fun previewModeDetectsCurvedTapeOnGreenFloor() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 70, green = 145, blue = 65)
        fillCurvedRibbon(frame, width, height, direction = -1.0, value = 20)

        val detection = checkNotNull(detectSequence(listOf(frame), width, height).single())

        assertTrue(detection.angleFromVerticalDegrees < -10.0)
        assertEquals(0.0, detection.nearFieldOffsetFraction, 0.02)
    }

    @Test
    fun darkDoorEdgeAboveGrayBaseboardIsNotTape() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 70, green = 145, blue = 65)
        fillRectRgb(frame, width, 0, 0, width, 245, red = 235, green = 235, blue = 225)
        fillRectRgb(frame, width, 0, 245, width, 275, red = 145, green = 145, blue = 140)
        fillRectRgb(frame, width, 120, 0, 300, 245, red = 130, green = 78, blue = 42)
        fillRect(frame, width, left = 270, top = 0, right = 300, bottom = 245, value = 20)

        val detection = detectSequence(listOf(frame), width, height).single()

        assertEquals(null, detection)
    }

    private fun detect(frame: ByteArray, width: Int, height: Int): TapeDetection =
        checkNotNull(detectSequence(listOf(frame), width, height, TapeDetectionMode.STRAIGHT).single()) {
            "detector rejected the synthetic tape"
        }

    private fun detectSequence(
        frames: List<ByteArray>,
        width: Int,
        height: Int,
        mode: TapeDetectionMode? = null,
    ): List<TapeDetection?> {
        val outcomes = LinkedBlockingQueue<DetectionOutcome>()
        val detector = BlackTapeDetector(
            onResult = { outcomes.offer(DetectionOutcome(it, null)) },
            onError = { outcomes.offer(DetectionOutcome(null, it)) },
        )
        mode?.let(detector::setDetectionMode)
        try {
            return frames.mapIndexed { index, frame ->
                if (index > 0) Thread.sleep(300)
                detector.submitRgba(frame, 0, frame.size, width, height)
                val outcome =
                    outcomes.poll(3, TimeUnit.SECONDS)
                        ?: throw AssertionError("detector did not finish")
                outcome.failure?.let { throw AssertionError("detector failed", it) }
                outcome.detection
            }
        } finally {
            detector.close()
        }
    }

    private data class DetectionOutcome(
        val detection: TapeDetection?,
        val failure: Throwable?,
    )

    private fun rgbaFrame(
        width: Int,
        height: Int,
        red: Int,
        green: Int,
        blue: Int,
    ): ByteArray = ByteArray(width * height * 4).also { frame ->
        for (pixel in 0 until width * height) {
            val offset = pixel * 4
            frame[offset] = red.toByte()
            frame[offset + 1] = green.toByte()
            frame[offset + 2] = blue.toByte()
            frame[offset + 3] = 255.toByte()
        }
    }

    private fun fillRect(
        frame: ByteArray,
        frameWidth: Int,
        left: Int,
        top: Int,
        right: Int,
        bottom: Int,
        value: Int,
    ) {
        for (y in top until bottom) {
            for (x in left until right) {
                val offset = (y * frameWidth + x) * 4
                frame[offset] = value.toByte()
                frame[offset + 1] = value.toByte()
                frame[offset + 2] = value.toByte()
            }
        }
    }

    private fun fillRectRgb(
        frame: ByteArray,
        frameWidth: Int,
        left: Int,
        top: Int,
        right: Int,
        bottom: Int,
        red: Int,
        green: Int,
        blue: Int,
    ) {
        for (y in top until bottom) {
            for (x in left until right) {
                val offset = (y * frameWidth + x) * 4
                frame[offset] = red.toByte()
                frame[offset + 1] = green.toByte()
                frame[offset + 2] = blue.toByte()
            }
        }
    }

    private fun fillCurvedRibbon(
        frame: ByteArray,
        frameWidth: Int,
        frameHeight: Int,
        direction: Double,
        value: Int,
    ) {
        for (y in 0 until frameHeight) {
            val forwardFraction = (frameHeight - 1 - y) / (frameHeight - 1.0)
            val center =
                frameWidth / 2.0 + direction * 140.0 * forwardFraction * forwardFraction
            val left = (center - 15.0).toInt().coerceAtLeast(0)
            val right = (center + 15.0).toInt().coerceAtMost(frameWidth)
            fillRect(frame, frameWidth, left, y, right, y + 1, value)
        }
    }
}
