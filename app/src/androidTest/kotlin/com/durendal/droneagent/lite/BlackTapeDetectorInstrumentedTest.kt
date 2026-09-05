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
        assertEquals(PathQuality.FULL_PATH, detection.quality)
        assertEquals(0.0, detection.nearFieldOffsetFraction, 0.01)
        assertEquals(0.5, detection.anchorXFraction, 0.01)
        assertEquals(TRACKING_TARGET_Y_FRACTION, detection.anchorYFraction, 0.01)
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
        assertEquals(PathQuality.FULL_PATH, detection.quality)
        assertEquals(0.0, detection.nearFieldOffsetFraction, 0.01)
        assertTrue(detection.lookaheadY < detection.anchorYFraction)
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
        assertEquals(0.0, detection.nearFieldOffsetFraction, 0.04)
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

        val diagnostics = mutableListOf<String>()
        val detections =
            detectSequence(
                listOf(fullTape, terminalTape),
                width,
                height,
                TapeDetectionMode.STRAIGHT,
                onDiagnostics = diagnostics::add,
            )
        assertTrue(detections[0] != null)
        val terminal = checkNotNull(detections[1]) { diagnostics.last() }
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
    fun sameDesaturatedCardboardOnBothSidesEstablishesTrackingReference() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 173, green = 160, blue = 129)
        fillCurvedRibbon(frame, width, height, direction = 1.0, value = 20)

        val diagnostics = mutableListOf<String>()
        val detections =
            detectSequence(
                listOf(frame, frame, frame),
                width,
                height,
                onDiagnostics = diagnostics::add,
                beforeFrame = { index, detector ->
                    if (index == 0) detector.beginTrackingSession(requireConsistentCurve = true)
                },
            )

        assertEquals(diagnostics.joinToString("\n"), listOf(null, null), detections.take(2))
        assertTrue(diagnostics.last(), detections.last() != null)
    }

    @Test
    fun stableOffAxisPreviewLockIsPromotedIntoTrackingSession() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillCurvedRibbon(
            frame,
            width,
            height,
            direction = -1.0,
            value = 20,
            baseXFraction = 0.75,
        )

        val diagnostics = mutableListOf<String>()
        val detections =
            detectSequence(
                List(13) { frame },
                width,
                height,
                onDiagnostics = diagnostics::add,
                beforeFrame = { index, detector ->
                    if (index == 12) detector.beginTrackingSession(requireConsistentCurve = true)
                },
            )

        assertTrue(diagnostics.joinToString("\n"), detections.take(12).count { it != null } >= 3)
        assertTrue(diagnostics.joinToString("\n"), detections.last() != null)
    }



    @Test
    fun trackingReferenceCannotBeLearnedFromGreenFloor() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 70, green = 145, blue = 65)
        fillCurvedRibbon(frame, width, height, direction = -1.0, value = 20)

        val diagnostics = mutableListOf<String>()
        val detections =
            detectSequence(
                listOf(frame, frame, frame),
                width,
                height,
                onDiagnostics = diagnostics::add,
                beforeFrame = { index, detector ->
                    if (index == 0) detector.beginTrackingSession(requireConsistentCurve = true)
                },
            )

        assertTrue(diagnostics.joinToString("\n"), detections.all { it == null })
        assertTrue(
            diagnostics.joinToString("\n"),
            diagnostics.all(BOARD_COLOR_REJECTION::containsMatchIn),
        )
    }

    @Test
    fun tapeShapedMarkOnGreenFloorIsNotCardboardTape() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 70, green = 145, blue = 65)
        fillCurvedRibbon(frame, width, height, direction = -1.0, value = 20)

        val diagnostics = mutableListOf<String>()
        val detection =
            detectSequence(
                listOf(frame),
                width,
                height,
                onDiagnostics = diagnostics::add,
            ).single()

        assertEquals(null, detection)
        assertTrue(diagnostics.single(), BOARD_COLOR_REJECTION.containsMatchIn(diagnostics.single()))
    }

    @Test
    fun tapeShapedMarkOnDarkFloorIsNotCardboardTape() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 62, green = 62, blue = 62)
        fillCurvedRibbon(frame, width, height, direction = 1.0, value = 12)

        val diagnostics = mutableListOf<String>()
        val detection =
            detectSequence(
                listOf(frame),
                width,
                height,
                onDiagnostics = diagnostics::add,
            ).single()

        assertEquals(null, detection)
        assertTrue(diagnostics.single(), BOARD_COLOR_REJECTION.containsMatchIn(diagnostics.single()))
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

        val diagnostics = mutableListOf<String>()
        val tracked = detectSequence(
            frames = listOf(fullTape, terminalTape),
            width = width,
            height = height,
            mode = TapeDetectionMode.STRAIGHT,
            onDiagnostics = diagnostics::add,
            beforeFrame = { index, detector ->
                if (index == 1) detector.setDetectionMode(TapeDetectionMode.STRAIGHT)
            },
        )

        assertTrue(tracked[0] != null)
        assertTrue(diagnostics.last(), tracked[1] != null)
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

        var diagnostics = ""
        val detection = checkNotNull(
            detectSequence(
                listOf(frame),
                width,
                height,
                onDiagnostics = { diagnostics = it },
            ).single(),
        ) { diagnostics }

        assertTrue(detection.bounds.right > 0.99)
        assertTrue(!detection.endpointCandidate)
        assertTrue(detection.angleFromVerticalDegrees in -30.0..0.0)
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

        assertTrue(detection.nearFieldOffsetFraction > 0.10)
        assertTrue(detection.bounds.right > 0.68)
    }

    @Test
    fun trackedTapeDoesNotCollapseOntoThinCardboardJoin() {
        val width = 640
        val height = 360
        val tape = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillCurvedRibbon(tape, width, height, direction = 1.0, value = 20)
        val cardboardJoin = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        for (y in 0 until height) {
            val forwardFraction = (height - 1 - y) / (height - 1.0)
            val center =
                width / 2.0 + CURVE_DISPLACEMENT * forwardFraction * forwardFraction
            fillRect(
                cardboardJoin,
                width,
                left = (center - 1.0).toInt(),
                top = y,
                right = (center + 1.0).toInt(),
                bottom = y + 1,
                value = 45,
            )
        }
        val diagnostics = mutableListOf<String>()

        val detections =
            detectSequence(
                listOf(tape, cardboardJoin, cardboardJoin),
                width,
                height,
                onDiagnostics = diagnostics::add,
            )

        assertTrue(diagnostics.first(), detections.first() != null)
        assertTrue(diagnostics.joinToString("\n"), detections.drop(1).all { it == null })
        assertTrue(diagnostics.last(), diagnostics.last().contains("width:"))
    }
    @Test
    fun finalCardboardScreenshotUiIsNotAcceptedAsCameraTape() {
        // This asset is a screenshot with the app's dark status panel, controls,
        // and bottom bar burned into the pixels. The detector receives raw camera
        // frames in production, so accepting any of those overlay components as
        // physical tape would be a false positive rather than real-camera proof.
        val frame = rgbaAsset("final-cardboard-tape.jpg")

        val detection =
            detectSequence(
                listOf(frame),
                width = 640,
                height = 360,
            ).single()

        assertEquals(null, detection)
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
    fun rainbowTapeLookaheadFollowsCounterclockwiseDirectionFromAircraftProjection() {
        val width = 640
        val height = 360
        val detection =
            checkNotNull(
                detectSequence(listOf(rainbowFrame(width, height)), width, height).single(),
            )

        assertEquals(TRACKING_TARGET_X_FRACTION, detection.anchorXFraction, 0.03)
        assertTrue(detection.lookaheadX < detection.anchorXFraction)
    }

    @Test
    fun rainbowTrackingDoesNotSwitchToTheLowerLeftEndpoint() {
        val width = 640
        val height = 360
        val diagnostics = mutableListOf<String>()
        val outcomes =
            detectSequence(
                listOf(
                    rainbowFrame(width, height),
                    rainbowFrame(width, height, leftEndpointDrop = 40.0),
                ),
                width,
                height,
                onDiagnostics = diagnostics::add,
            )
        val detections = outcomes.mapIndexed { index, detection ->
            checkNotNull(detection) { diagnostics[index] }
        }

        assertTrue(
            detections.all {
                kotlin.math.abs(it.anchorXFraction - TRACKING_TARGET_X_FRACTION) < 0.08
            },
        )
        assertTrue(detections.all { it.quality == PathQuality.FULL_PATH })
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
        assertTrue(kotlin.math.abs(horizontal.nearFieldOffsetFraction) < 0.10)
        assertTrue(horizontal.anchorXFraction in 0.45..0.60)
        assertTrue(horizontal.anchorYFraction in 0.65..0.80)
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
    fun trackedFlightPathSurvivesWarmChromaFlicker() {
        val width = 1920
        val height = 1080
        val outcomes =
            detectSequence(
                listOf(
                    rgbaAsset("flight-path-before-chroma-loss.jpg", width, height),
                    rgbaAsset("flight-path-chroma-loss.jpg", width, height),
                ),
                width,
                height,
            )

        assertTrue("recorded run-up frame must establish continuity", outcomes.first() != null)
        assertEquals(PathQuality.FULL_PATH, outcomes.last()?.quality)
    }

    @Test
    fun trackedTapeSurvivesRecordedGlare() {
        val width = 1920
        val height = 1080
        val diagnostics = mutableListOf<String>()
        val established = rgbaAsset("flight-glare-before-loss.png", width, height)
        val detections =
            detectSequence(
                listOf(
                    established,
                    established,
                    established,
                    rgbaAsset("flight-glare-loss.png", width, height),
                ),
                width,
                height,
                onDiagnostics = diagnostics::add,
                beforeFrame = { index, detector ->
                    if (index == 0) detector.beginTrackingSession()
                },
            )

        assertTrue(diagnostics[2], detections[2] != null)
        assertEquals(diagnostics.last(), PathQuality.FULL_PATH, detections.last()?.quality)
    }

    @Test
    fun trackedTapeSurvivesLatestFlightReflection() {
        val width = 1920
        val height = 1080
        val diagnostics = mutableListOf<String>()
        val established =
            rgbaAsset("flight-reflection-pause-establish.png", width, height)
        val detections =
            detectSequence(
                listOf(
                    established,
                    established,
                    established,
                    rgbaAsset("flight-reflection-pause-before.png", width, height),
                    rgbaAsset("flight-reflection-pause-at.png", width, height),
                ),
                width,
                height,
                onDiagnostics = diagnostics::add,
                beforeFrame = { index, detector ->
                    if (index == 0) detector.beginTrackingSession()
                },
            )
        assertTrue(diagnostics.joinToString("\n"), detections.take(2).all { it == null })
        detections.drop(2).forEachIndexed { index, detection ->
            assertEquals(diagnostics[index + 2], PathQuality.FULL_PATH, detection?.quality)
        }
    }

    @Test
    fun trackedReferenceMatchedTapeSurvivesMissingCoarseFloorContext() {
        val width = 1920
        val height = 1080
        val diagnostics = mutableListOf<String>()
        val established =
            rgbaAsset("flight-reference-zero-floor-before.jpg", width, height)
        val detections =
            detectSequence(
                listOf(
                    established,
                    established,
                    established,
                    rgbaAsset("flight-reference-zero-floor-at.jpg", width, height),
                ),
                width,
                height,
                onDiagnostics = diagnostics::add,
                beforeFrame = { index, detector ->
                    if (index == 0) detector.beginTrackingSession()
                },
            )

        assertTrue(diagnostics[2], detections[2] != null)
        assertEquals(diagnostics.last(), PathQuality.FULL_PATH, detections.last()?.quality)
    }

    @Test
    fun oneSidedDeskEdgeCannotProvideEndpointEvidenceBeforeTapeAppears() {
        val width = 1920
        val height = 1080
        val diagnostics = mutableListOf<String>()
        val detections =
            detectSequence(
                listOf(
                    rgbaAsset("flight-endpoint-missed-1.png", width, height),
                    rgbaAsset("flight-endpoint-missed-2.png", width, height),
                    rgbaAsset("flight-endpoint-missed-3.png", width, height),
                    rgbaAsset("flight-endpoint-missed-4.png", width, height),
                ),
                width,
                height,
                onDiagnostics = diagnostics::add,
                beforeFrame = { index, detector ->
                    detector.setDetectionMode(
                        if (index == 0) TapeDetectionMode.STRAIGHT else TapeDetectionMode.PATH,
                    )
                },
            )

        val evidence = detections.mapIndexed { index, detection ->
            "$index:${detection?.quality}:${detection?.endpointCandidate}:${diagnostics[index]}"
        }
        assertTrue(evidence.joinToString("\n"), detections[0]?.endpointCandidate == true)
        assertTrue(evidence.joinToString("\n"), detections[1] == null)
        assertTrue(evidence.joinToString("\n"), detections[2] == null)
        assertEquals(evidence.joinToString("\n"), PathQuality.FULL_PATH, detections[3]?.quality)
        assertTrue(evidence.joinToString("\n"), detections[3]?.endpointCandidate != true)
    }

    @Test
    fun flightFrameChoosesTapeOverDeskDistractors() {
        val width = 1920
        val height = 1080
        val diagnostics = mutableListOf<String>()

        val detection =
            detectSequence(
                listOf(rgbaAsset("flight-tape-left-desk-distractors.jpg", width, height)),
                width,
                height,
                onDiagnostics = diagnostics::add,
            ).single()

        assertTrue(diagnostics.single(), detection != null)
        assertTrue(diagnostics.single(), checkNotNull(detection).anchorXFraction < 0.6)
        assertEquals(diagnostics.single(), PathQuality.FULL_PATH, detection.quality)
    }

    @Test
    fun trackedCircleSurvivesVisibleRouteStartTopologyChange() {
        val width = 1920
        val height = 1080
        val diagnostics = mutableListOf<String>()
        val acquisition = rgbaAsset("flight-path-before-chroma-loss.jpg", width, height)
        val before = rgbaAsset("flight-circle-before-direction-reject.png", width, height)
        val topologyChange = rgbaAsset("flight-circle-direction-reject.png", width, height)
        val frames = buildList {
            repeat(3) { add(acquisition) }
            add(before)
            add(topologyChange)
        }

        val detections =
            detectSequence(
                frames,
                width,
                height,
                onDiagnostics = diagnostics::add,
                beforeFrame = { index, detector ->
                    if (index == 0) detector.beginTrackingSession()
                },
            )
        assertTrue(diagnostics.joinToString("\n"), detections.drop(2).all { it != null })
    }

    @Test
    fun deskDistractorsCannotQualifyForFlightControl() {
        val width = 640
        val height = 360
        val diagnostics = mutableListOf<String>()
        val frame = rgbaAsset("flight-desk-distractors-only.jpg", width, height)

        val detections =
            detectSequence(
                listOf(frame, frame, frame),
                width,
                height,
                onDiagnostics = diagnostics::add,
                beforeFrame = { index, detector ->
                    if (index == 0) detector.beginTrackingSession()
                },
            )

        assertTrue(diagnostics.joinToString("\n"), detections.all { it == null })
    }

    @Test
    fun trackedTapeDoesNotSwitchToDeskDistractorsAfterPathLoss() {
        val width = 1920
        val height = 1080
        val diagnostics = mutableListOf<String>()
        val tape = rgbaAsset("flight-path-before-chroma-loss.jpg", width, height)
        val distractors = rgbaAsset("flight-desk-distractors-full.jpg", width, height)
        val frames = buildList {
            repeat(3) { add(tape) }
            repeat(10) { add(distractors) }
        }

        val detections =
            detectSequence(
                frames,
                width,
                height,
                onDiagnostics = diagnostics::add,
                beforeFrame = { index, detector ->
                    if (index == 0) detector.beginTrackingSession()
                },
            )

        assertTrue(diagnostics.joinToString("\n"), detections[2] != null)
        assertTrue(diagnostics.joinToString("\n"), detections.drop(3).all { it == null })
    }

    @Test
    fun flightBoardBoundaryIsNotReacquiredAsTape() {
        val width = 1920
        val height = 1080
        val diagnostics = mutableListOf<String>()
        val blankBoard = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        val initialPath = rgbaAsset("flight-path-before-chroma-loss.jpg", width, height)
        val frames = buildList {
            repeat(3) { add(initialPath) }
            repeat(8) { add(blankBoard) }
            add(rgbaAsset("flight-non-tape-boundary.jpg", width, height))
        }

        val detections =
            detectSequence(
                frames,
                width,
                height,
                onDiagnostics = diagnostics::add,
                beforeFrame = { index, detector ->
                    if (index == 0) detector.beginTrackingSession()
                },
            )

        assertTrue("recorded tape must establish the board colour", detections[2] != null)
        assertEquals(diagnostics.last(), null, detections.last())
        assertTrue(diagnostics.last(), BOARD_COLOR_REJECTION.containsMatchIn(diagnostics.last()))
    }

    @Test
    fun newTrackingSessionAcquiresCardboardAfterRejectingIdleFloor() {
        val width = 640
        val height = 360
        val idleScenery = rgbaFrame(width, height, red = 70, green = 145, blue = 65)
        fillCurvedRibbon(idleScenery, width, height, direction = 1.0, value = 20)
        val flightBoard = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillCurvedRibbon(flightBoard, width, height, direction = 1.0, value = 20)
        val diagnostics = mutableListOf<String>()

        val detections =
            detectSequence(
                listOf(idleScenery, flightBoard, flightBoard, flightBoard),
                width,
                height,
                onDiagnostics = diagnostics::add,
                beforeFrame = { index, detector ->
                    if (index == 1) {
                        detector.beginTrackingSession(requireConsistentCurve = true)
                    }
                },
            )

        assertEquals(diagnostics.first(), null, detections.first())
        assertTrue(diagnostics.joinToString("\n"), detections.slice(1..2).all { it == null })
        assertEquals(diagnostics.last(), PathQuality.FULL_PATH, detections.last()?.quality)
    }

    @Test
    fun circularTrackingSessionRejectsStraightAcquisitionFrames() {
        val width = 640
        val height = 360
        val curve = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillCurvedRibbon(curve, width, height, direction = 1.0, value = 20)
        val straight = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillRect(straight, width, left = 305, top = 0, right = 335, bottom = height, value = 20)
        val diagnostics = mutableListOf<String>()

        val detections =
            detectSequence(
                listOf(curve, straight, curve, straight),
                width,
                height,
                onDiagnostics = diagnostics::add,
                beforeFrame = { index, detector ->
                    if (index == 0) {
                        detector.beginTrackingSession(requireConsistentCurve = true)
                    }
                },
            )

        assertTrue(diagnostics.joinToString("\n"), detections.all { it == null })
    }

    @Test
    fun oneWrongBoardCannotPoisonTrackingSessionAcquisition() {
        val width = 640
        val height = 360
        val wrongBoard = rgbaFrame(width, height, red = 240, green = 150, blue = 20)
        fillCurvedRibbon(wrongBoard, width, height, direction = 1.0, value = 20)
        val flightBoard = rgbaFrame(width, height, red = 190, green = 132, blue = 58)
        fillCurvedRibbon(flightBoard, width, height, direction = 1.0, value = 20)
        val diagnostics = mutableListOf<String>()

        val detections =
            detectSequence(
                listOf(wrongBoard, flightBoard, flightBoard, flightBoard),
                width,
                height,
                onDiagnostics = diagnostics::add,
                beforeFrame = { index, detector ->
                    if (index == 0) detector.beginTrackingSession()
                },
            )

        assertTrue(diagnostics.joinToString("\n"), detections.take(3).all { it == null })
        assertEquals(diagnostics.last(), PathQuality.FULL_PATH, detections.last()?.quality)
    }

    @Test
    fun trackedWarmFloorShadowStillFailsChromaGate() {
        val width = 640
        val height = 360
        val tape = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillCurvedRibbon(tape, width, height, direction = -1.0, value = 20)
        val shadow = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillCurvedRibbonRgb(
            shadow,
            width,
            height,
            direction = -1.0,
            red = 72,
            green = 48,
            blue = 20,
        )

        val outcomes = detectSequence(listOf(tape, shadow), width, height)

        assertTrue(outcomes.first() != null)
        assertEquals(null, outcomes.last())
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
    fun curvedBoundaryBetweenDifferentBoardColoursIsNotTape() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        for (y in 0 until height) {
            val forwardFraction = (height - 1 - y) / (height - 1.0)
            val center = width / 2.0 + CURVE_DISPLACEMENT * forwardFraction * forwardFraction
            fillRectRgb(
                frame,
                width,
                left = 0,
                top = y,
                right = (center - TAPE_HALF_WIDTH).toInt(),
                bottom = y + 1,
                red = 70,
                green = 145,
                blue = 65,
            )
        }
        fillCurvedRibbon(frame, width, height, direction = 1.0, value = 20)

        val diagnostics = mutableListOf<String>()
        val detection =
            detectSequence(
                listOf(frame),
                width,
                height,
                onDiagnostics = diagnostics::add,
            ).single()

        assertEquals(diagnostics.single(), null, detection)
        assertTrue(diagnostics.single(), BOARD_COLOR_REJECTION.containsMatchIn(diagnostics.single()))
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
    @Test
    fun curvedCastShadowEdgeIsNotTape() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 186, green = 128, blue = 58)
        for (y in 0 until height) {
            val forwardFraction = (height - 1 - y) / (height - 1.0)
            val shadowEdge = (340.0 + 120.0 * forwardFraction * forwardFraction).toInt()
            fillRectRgb(
                frame,
                width,
                left = shadowEdge,
                top = y,
                right = width,
                bottom = y + 1,
                red = 100,
                green = 70,
                blue = 32,
            )
        }

        val diagnostics = mutableListOf<String>()
        val detection =
            detectSequence(
                listOf(frame),
                width,
                height,
                onDiagnostics = diagnostics::add,
            ).single()

        assertEquals(diagnostics.single(), null, detection)
    }
    @Test
    fun trackedTapeDoesNotFollowCastShadowEdge() {
        val width = 640
        val height = 360
        val cleanFrame = rgbaFrame(width, height, red = 186, green = 128, blue = 58)
        fillCurvedRibbon(cleanFrame, width, height, direction = 1.0, value = 20)
        val shadowFrame = rgbaFrame(width, height, red = 186, green = 128, blue = 58)
        for (y in 0 until height) {
            val forwardFraction = (height - 1 - y) / (height - 1.0)
            val shadowEdge = (220.0 + 240.0 * forwardFraction).toInt()
            fillRectRgb(
                shadowFrame,
                width,
                left = shadowEdge,
                top = y,
                right = width,
                bottom = y + 1,
                red = 100,
                green = 70,
                blue = 32,
            )
        }
        fillCurvedRibbon(shadowFrame, width, height, direction = 1.0, value = 20)

        val diagnostics = mutableListOf<String>()
        val detections =
            detectSequence(
                listOf(cleanFrame, cleanFrame, cleanFrame, shadowFrame),
                width,
                height,
                onDiagnostics = diagnostics::add,
                beforeFrame = { index, detector ->
                    if (index == 0) detector.beginTrackingSession()
                },
            )
        val clean = checkNotNull(detections[2]) { diagnostics[2] }
        val shadowed = checkNotNull(detections[3]) { diagnostics[3] }
        assertEquals(clean.anchorXFraction, shadowed.anchorXFraction, 0.04)
        assertEquals(clean.lookaheadX, shadowed.lookaheadX, 0.04)
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

    private fun rgbaAsset(
        name: String,
        expectedWidth: Int = 640,
        expectedHeight: Int = 360,
    ): ByteArray {
        val context = InstrumentationRegistry.getInstrumentation().context
        val bitmap = context.assets.open(name).use(BitmapFactory::decodeStream)
        check(bitmap.width == expectedWidth && bitmap.height == expectedHeight)
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

    @Test
    fun frameDiagnosticsSeparateInvalidAcceptedAndThrottledSubmissions() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        val outcomes = LinkedBlockingQueue<DetectionOutcome>()
        val detector = BlackTapeDetector(
            onResult = { outcomes.offer(DetectionOutcome(it, null)) },
            onError = { outcomes.offer(DetectionOutcome(null, it)) },
        )
        try {
            detector.submitRgba(ByteArray(1), 0, 1, width, height)
            detector.submitRgba(frame, 0, frame.size, width, height)
            detector.submitRgba(frame, 0, frame.size, width, height)
            val outcome =
                outcomes.poll(3, TimeUnit.SECONDS)
                    ?: throw AssertionError("detector did not finish")
            outcome.failure?.let { throw AssertionError("detector failed", it) }

            val diagnostics = detector.diagnosticsSummary()
            assertTrue(diagnostics, diagnostics.contains("frames=received:3 accepted:1"))
            assertTrue(diagnostics, diagnostics.contains("throttle:1 busy:0 invalid:1"))
            assertTrue(diagnostics, diagnostics.contains("completed:1 failed:0"))
        } finally {
            detector.close()
        }
    }

    @Test
    fun frameProfileAccountsForEveryOpenCvStage() {
        val width = 640
        val height = 360
        val frame = rgbaFrame(width, height, red = 180, green = 120, blue = 50)
        fillRect(frame, width, left = 305, top = 0, right = 335, bottom = height, value = 20)
        val profiles = LinkedBlockingQueue<TapeFrameProfile>()
        val detector = BlackTapeDetector(
            onResult = {},
            onError = { throw AssertionError("detector failed", it) },
            onFrameProfile = profiles::offer,
        )
        detector.setDetectionMode(TapeDetectionMode.STRAIGHT)
        try {
            detector.submitRgba(frame, 0, frame.size, width, height)
            val profile =
                profiles.poll(3, TimeUnit.SECONDS)
                    ?: throw AssertionError("detector did not publish a frame profile")
            val accountedNanos =
                profile.preprocessingNanos +
                    profile.thresholdingNanos +
                    profile.floorContextNanos +
                    profile.morphologyAndContoursNanos +
                    profile.candidateScoringNanos +
                    profile.cleanupNanos
            val processingNanos =
                profile.processingCompletedAtNanos - profile.processingStartedAtNanos

            assertTrue(profile.preprocessingNanos > 0L)
            assertTrue(profile.thresholdingNanos > 0L)
            assertTrue(profile.floorContextNanos > 0L)
            assertTrue(profile.morphologyAndContoursNanos > 0L)
            assertTrue(profile.candidateScoringNanos > 0L)
            assertTrue(profile.cleanupNanos > 0L)
            assertTrue("accounted=$accountedNanos processing=$processingNanos", accountedNanos <= processingNanos)
        } finally {
            detector.close()
        }
    }

    private fun detect(frame: ByteArray, width: Int, height: Int): TapeDetection {
        var diagnostics = ""
        return checkNotNull(
            detectSequence(
                listOf(frame),
                width,
                height,
                TapeDetectionMode.STRAIGHT,
                onDiagnostics = { diagnostics = it },
            ).single(),
        ) {
            "detector rejected the synthetic tape: $diagnostics"
        }
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
        get() = requireNotNull(lookahead) { "expected a FULL_PATH look-ahead point" }.xFraction

    private val TapeDetection.lookaheadY: Double
        get() = requireNotNull(lookahead) { "expected a FULL_PATH look-ahead point" }.yFraction

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
        baseXFraction: Double = 0.5,
    ) = fillCurvedRibbonRgb(
        frame,
        frameWidth,
        frameHeight,
        direction,
        value,
        value,
        value,
        baseXFraction,
    )

    private fun fillCurvedRibbonRgb(
        frame: ByteArray,
        frameWidth: Int,
        frameHeight: Int,
        direction: Double,
        red: Int,
        green: Int,
        blue: Int,
        baseXFraction: Double = 0.5,
    ) {
        for (y in 0 until frameHeight) {
            val forwardFraction = (frameHeight - 1 - y) / (frameHeight - 1.0)
            val center =
                frameWidth * baseXFraction +
                    direction * CURVE_DISPLACEMENT * forwardFraction * forwardFraction
            val left = (center - TAPE_HALF_WIDTH).toInt().coerceAtLeast(0)
            val right = (center + TAPE_HALF_WIDTH).toInt().coerceAtMost(frameWidth)
            fillRectRgb(frame, frameWidth, left, y, right, y + 1, red, green, blue)
        }
    }

    private companion object {
        const val CURVE_DISPLACEMENT = 140.0
        const val TAPE_HALF_WIDTH = 15.0
        val BOARD_COLOR_REJECTION = Regex("""boardColor:[1-9]\d*""")
    }
}
