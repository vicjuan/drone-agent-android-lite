package com.durendal.droneagent.lite

import android.graphics.BitmapFactory
import androidx.test.platform.app.InstrumentationRegistry
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
        assertEquals(0.5, detection.anchorXFraction, 0.01)
        assertTrue(detection.anchorYFraction > 0.95)
        assertTrue(detection.lookaheadY < detection.anchorYFraction)
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
    fun curvedPathRemainsContinuousAcrossBrightGlareBands() {
        val width = 640
        val height = 360
        val cleanFrame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillCurvedRibbon(cleanFrame, width, height, direction = 1.0, value = 20)
        val frame = cleanFrame.copyOf()
        for (y in 145 until 160) {
            val forwardFraction = (height - 1 - y) / (height - 1.0)
            val center = width / 2.0 + CURVE_DISPLACEMENT * forwardFraction * forwardFraction
            fillRect(
                frame,
                width,
                left = (center - TAPE_HALF_WIDTH).toInt(),
                top = y,
                right = (center + TAPE_HALF_WIDTH).toInt(),
                bottom = y + 1,
                value = 220,
            )
        }
        for (y in 185 until 200) {
            val forwardFraction = (height - 1 - y) / (height - 1.0)
            val center = width / 2.0 + CURVE_DISPLACEMENT * forwardFraction * forwardFraction
            fillRect(
                frame,
                width,
                left = (center - TAPE_HALF_WIDTH).toInt(),
                top = y,
                right = (center + TAPE_HALF_WIDTH).toInt(),
                bottom = y + 1,
                value = 220,
            )
        }

        val diagnostics = mutableListOf<String>()
        val detections =
            detectSequence(
                listOf(cleanFrame, frame),
                width,
                height,
                onDiagnostics = diagnostics::add,
            )
        assertTrue(detections[0] != null)
        val detection = checkNotNull(detections[1]) { diagnostics.last() }

        assertTrue(
            "expected a continuous path, actual=$detection diagnostics=${diagnostics.last()}",
            detection.longSideFraction > 0.9,
        )
        assertEquals(0.0, detection.nearFieldOffsetFraction, 0.03)
        assertTrue(detection.lookaheadX > detection.anchorXFraction)
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
    fun defaultPathModeDetectsCurvedTape() {
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

        assertEquals(0.0, detection.angleFromVerticalDegrees, 2.0)
        assertTrue(detection.lookaheadX > detection.anchorXFraction)
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

        assertEquals(0.0, detection.angleFromVerticalDegrees, 2.0)
        assertTrue(detection.lookaheadX < detection.anchorXFraction)
        assertEquals(0.0, detection.nearFieldOffsetFraction, 0.02)
    }

    @Test
    fun trackedTapeIsRejectedWhenEitherSideLeavesCorrugatedBoard() {
        val width = 640
        val height = 360
        val uniformBoard = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillCurvedRibbon(uniformBoard, width, height, direction = 1.0, value = 20)
        val boardBesideGrayFloor = stripedSplitFloorWithCurvedTape(width, height)

        val tracked = detectSequence(listOf(uniformBoard, boardBesideGrayFloor), width, height)

        assertTrue(tracked[0] != null)
        assertEquals(null, tracked[1])
    }
    @Test
    fun selectingTheCurrentPathModePreservesTheTrackedCandidate() {
        val width = 640
        val height = 360
        val fullTape = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillRect(fullTape, width, left = 305, top = 0, right = 335, bottom = height, value = 20)
        val terminalTape = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillRect(terminalTape, width, left = 305, top = 300, right = 335, bottom = height, value = 20)

        val tracked = detectSequence(
            frames = listOf(fullTape, terminalTape),
            width = width,
            height = height,
            mode = TapeDetectionMode.STRAIGHT,
            beforeFrame = { index, detector ->
                if (index == 1) detector.setDetectionMode(TapeDetectionMode.STRAIGHT)
            },
        )

        assertTrue(tracked[0] != null)
        assertTrue(tracked[1] != null)
    }


    @Test
    fun curvedTapeCanBeAcquiredWhileEnteringFromFrameEdge() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        for (y in 0 until height) {
            val forwardFraction = (height - 1 - y) / (height - 1.0)
            val center = 625.0 - 180.0 * forwardFraction * forwardFraction
            val left = (center - 15.0).toInt().coerceAtLeast(0)
            val right = (center + 15.0).toInt().coerceAtMost(width)
            fillRect(frame, width, left, y, right, y + 1, value = 20)
        }

        val detection = checkNotNull(detectSequence(listOf(frame), width, height).single())

        assertEquals(1.0, detection.bounds.right, 0.001)
        assertEquals(0.0, detection.angleFromVerticalDegrees, 2.0)
        assertTrue(detection.lookaheadX < detection.anchorXFraction)
    }

    @Test
    fun thickTapeWinsWhenItIntersectsAThinFloorSeam() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        for (y in 0 until height) {
            val nearFraction = y / (height - 1.0)
            val junctionX = 360.0
            val tapeCenter = junctionX + 80.0 * nearFraction * nearFraction
            val seamCenter = junctionX + 40.0 * nearFraction
            fillRect(
                frame,
                width,
                (tapeCenter - 15.0).toInt(),
                y,
                (tapeCenter + 15.0).toInt(),
                y + 1,
                value = 20,
            )
            fillRect(
                frame,
                width,
                (seamCenter - 3.0).toInt(),
                y,
                (seamCenter + 3.0).toInt(),
                y + 1,
                value = 70,
            )
        }

        val detection = checkNotNull(detectSequence(listOf(frame), width, height).single())

        assertTrue(detection.nearFieldOffsetFraction > 0.15)
        assertTrue(detection.bounds.right > 0.68)
    }
    @Test
    fun finalCardboardSceneKeepsVisibleTapeDetectable() {
        val frame = rgbaAsset("final-cardboard-tape.jpg")
        val diagnostics = mutableListOf<String>()

        val detection =
            checkNotNull(
                detectSequence(
                    listOf(frame),
                    width = 640,
                    height = 360,
                    onDiagnostics = diagnostics::add,
                ).single(),
            ) {
                diagnostics.single()
            }

        assertTrue(kotlin.math.abs(detection.angleFromVerticalDegrees) > 70.0)
        assertTrue(detection.longSideFraction > 0.50)
    }


    @Test
    fun horizontalCurvedTapeCanBeAcquiredWithoutPreviousDetection() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        for (x in 60 until 590) {
            val horizontalFraction = (x - 60) / 529.0
            val centerY = (230.0 + 55.0 * horizontalFraction * horizontalFraction).toInt()
            fillRect(
                frame,
                width,
                left = x,
                top = centerY - 15,
                right = x + 1,
                bottom = centerY + 15,
                value = 20,
            )
        }

        val diagnostics = mutableListOf<String>()
        val detection =
            checkNotNull(
                detectSequence(
                    listOf(frame),
                    width,
                    height,
                    onDiagnostics = diagnostics::add,
                ).single(),
            ) {
                diagnostics.single()
            }

        assertTrue(kotlin.math.abs(detection.angleFromVerticalDegrees) > 70.0)
        assertTrue(detection.longSideFraction > 1.0)
    }

    @Test
    fun rainbowTapeAcquiresTheRightEndpointForCounterclockwiseTracking() {
        val width = 640
        val height = 360
        val detection =
            checkNotNull(
                detectSequence(listOf(rainbowFrame(width, height)), width, height).single(),
            )

        assertTrue(detection.anchorXFraction > 0.75)
        assertTrue(detection.lookaheadX < detection.anchorXFraction)
        assertTrue(detection.lookaheadX > 0.5)
    }

    @Test
    fun rainbowTrackingDoesNotSwitchToTheLowerLeftEndpoint() {
        val width = 640
        val height = 360
        val detections =
            detectSequence(
                listOf(
                    rainbowFrame(width, height),
                    rainbowFrame(width, height, leftEndpointDrop = 40.0),
                ),
                width,
                height,
            ).map(::checkNotNull)

        assertTrue(detections.all { it.anchorXFraction > 0.75 })
        assertTrue(detections.all { it.lookaheadX < it.anchorXFraction })
    }

    @Test
    fun trackedTapeReacquiresHorizontallyWithoutSelectingThinFloorSeam() {
        val width = 640
        val height = 360
        val floorRed = 180
        val floorGreen = 120
        val floorBlue = 50
        val verticalTape = rgbaFrame(width, height, floorRed, floorGreen, floorBlue)
        for (y in 240 until height) {
            val forwardFraction = (height - 1 - y) / 119.0
            val centerX = 500.0 + 40.0 * forwardFraction * forwardFraction
            fillRect(
                verticalTape,
                width,
                left = (centerX - 15.0).toInt(),
                top = y,
                right = (centerX + 15.0).toInt(),
                bottom = y + 1,
                value = 20,
            )
        }
        val horizontalTape = rgbaFrame(width, height, floorRed, floorGreen, floorBlue)
        for (x in 100 until 550) {
            val horizontalFraction = (x - 100) / 449.0
            val centerY = (240.0 + 50.0 * horizontalFraction * horizontalFraction).toInt()
            fillRect(
                horizontalTape,
                width,
                left = x,
                top = centerY - 15,
                right = x + 1,
                bottom = centerY + 15,
                value = 20,
            )
        }
        fillRect(horizontalTape, width, left = 0, top = 322, right = 450, bottom = 328, value = 40)

        val diagnostics = mutableListOf<String>()
        val detections =
            detectSequence(
                listOf(verticalTape, horizontalTape),
                width,
                height,
                onDiagnostics = diagnostics::add,
            )
        assertTrue(detections[0] != null)
        val horizontal = checkNotNull(detections[1]) { diagnostics.last() }

        assertTrue(kotlin.math.abs(horizontal.angleFromVerticalDegrees) > 70.0)
        assertTrue(horizontal.longSideFraction > 1.0)
        assertTrue(horizontal.bounds.bottom < 0.85)
        assertTrue(horizontal.nearFieldOffsetFraction > 0.30)
        assertTrue(horizontal.anchorXFraction > 0.75)
        assertTrue(horizontal.anchorYFraction > 0.75)
        assertTrue(horizontal.lookaheadX < horizontal.anchorXFraction)
    }
    @Test
    fun veryDarkTapeRemainsDetectableUnderWarmColourCast() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillCurvedRibbonRgb(
            frame,
            width,
            height,
            direction = 1.0,
            red = 55,
            green = 38,
            blue = 25,
        )

        val detection = detectSequence(listOf(frame), width, height).single()

        assertTrue(detection != null)
    }

    @Test
    fun curvedRedPlasticIsNotTape() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillCurvedRibbonRgb(
            frame,
            width,
            height,
            direction = 1.0,
            red = 120,
            green = 20,
            blue = 20,
        )

        val diagnostics = mutableListOf<String>()
        val detection = detectSequence(listOf(frame), width, height, onDiagnostics = diagnostics::add).single()

        assertEquals(null, detection)
        assertTrue(diagnostics.single().contains("chroma:1"))
    }

    @Test
    fun curvedFloorShadowIsNotTape() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillCurvedRibbonRgb(
            frame,
            width,
            height,
            direction = -1.0,
            red = 72,
            green = 48,
            blue = 20,
        )

        val diagnostics = mutableListOf<String>()
        val detection = detectSequence(listOf(frame), width, height, onDiagnostics = diagnostics::add).single()

        assertEquals(null, detection)
        assertTrue(diagnostics.single().contains("chroma:1"))
    }

    @Test
    fun straightPathIsRejectedInCurvedTapeMode() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillRect(frame, width, left = 305, top = 0, right = 335, bottom = height, value = 20)

        val detection = detectSequence(listOf(frame), width, height).single()

        assertEquals(null, detection)
    }

    @Test
    fun straightWallFloorEdgeFromFlightIsNotCurvedTape() {
        val frame = rgbaAsset("straight-wall-floor-edge.jpg")
        val diagnostics = mutableListOf<String>()

        val detection =
            detectSequence(
                listOf(frame),
                width = 640,
                height = 360,
                onDiagnostics = diagnostics::add,
            ).single()

        assertEquals(diagnostics.single(), null, detection)
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
    private fun stripedSplitFloorWithCurvedTape(
        width: Int,
        height: Int,
    ): ByteArray {
        val frame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        for (y in 0 until height step 8) {
            fillRectRgb(
                frame,
                width,
                left = 0,
                top = y,
                right = width / 2,
                bottom = (y + 4).coerceAtMost(height),
                red = 130,
                green = 130,
                blue = 130,
            )
            fillRectRgb(
                frame,
                width,
                left = 0,
                top = (y + 4).coerceAtMost(height),
                right = width / 2,
                bottom = (y + 8).coerceAtMost(height),
                red = 220,
                green = 220,
                blue = 220,
            )
        }
        fillCurvedRibbon(frame, width, height, direction = 1.0, value = 20)
        return frame
    }

    private fun rainbowFrame(
        width: Int,
        height: Int,
        leftEndpointDrop: Double = 0.0,
    ): ByteArray {
        val frame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        for (x in 100 until 541) {
            val horizontalFraction = (x - 320.0) / 220.0
            val drop = leftEndpointDrop * (540.0 - x) / 440.0
            val centerY =
                (
                    280.0 -
                        110.0 * (1.0 - horizontalFraction * horizontalFraction) +
                        drop
                    ).toInt()
            fillRect(
                frame,
                width,
                left = x,
                top = centerY - 15,
                right = x + 1,
                bottom = centerY + 15,
                value = 20,
            )
        }
        return frame
    }

    private fun rgbaAsset(name: String): ByteArray {
        val context = InstrumentationRegistry.getInstrumentation().context
        val bitmap = context.assets.open(name).use(BitmapFactory::decodeStream)
        check(bitmap.width == 640 && bitmap.height == 360)
        val pixels = IntArray(bitmap.width * bitmap.height)
        bitmap.getPixels(pixels, 0, bitmap.width, 0, 0, bitmap.width, bitmap.height)
        return ByteArray(pixels.size * 4).also { frame ->
            pixels.forEachIndexed { index, pixel ->
                val offset = index * 4
                frame[offset] = (pixel shr 16).toByte()
                frame[offset + 1] = (pixel shr 8).toByte()
                frame[offset + 2] = pixel.toByte()
                frame[offset + 3] = 255.toByte()
            }
        }
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
        onDiagnostics: (String) -> Unit = {},
        beforeFrame: (Int, BlackTapeDetector) -> Unit = { _, _ -> },
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
                beforeFrame(index, detector)
                detector.submitRgba(frame, 0, frame.size, width, height)
                val outcome =
                    outcomes.poll(3, TimeUnit.SECONDS)
                        ?: throw AssertionError("detector did not finish")
                outcome.failure?.let { throw AssertionError("detector failed", it) }
                onDiagnostics(detector.diagnosticsSummary())
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

    /**
     * Every detector case here expects a path good enough to steer by, so a
     * missing look-ahead is a failure of the case rather than a value to skip.
     */
    private val TapeDetection.lookaheadX: Double
        get() = requireNotNull(lookaheadXFraction) { "expected a FULL_PATH look-ahead point" }

    private val TapeDetection.lookaheadY: Double
        get() = requireNotNull(lookaheadYFraction) { "expected a FULL_PATH look-ahead point" }

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
    ) = fillRectRgb(frame, frameWidth, left, top, right, bottom, value, value, value)

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
    ) = fillCurvedRibbonRgb(frame, frameWidth, frameHeight, direction, value, value, value)

    private fun fillCurvedRibbonRgb(
        frame: ByteArray,
        frameWidth: Int,
        frameHeight: Int,
        direction: Double,
        red: Int,
        green: Int,
        blue: Int,
    ) {
        for (y in 0 until frameHeight) {
            val forwardFraction = (frameHeight - 1 - y) / (frameHeight - 1.0)
            val center =
                frameWidth / 2.0 + direction * CURVE_DISPLACEMENT * forwardFraction * forwardFraction
            val left = (center - TAPE_HALF_WIDTH).toInt().coerceAtLeast(0)
            val right = (center + TAPE_HALF_WIDTH).toInt().coerceAtMost(frameWidth)
            fillRectRgb(frame, frameWidth, left, y, right, y + 1, red, green, blue)
        }
    }

    private companion object {
        const val CURVE_DISPLACEMENT = 140.0
        const val TAPE_HALF_WIDTH = 15.0
    }
}
