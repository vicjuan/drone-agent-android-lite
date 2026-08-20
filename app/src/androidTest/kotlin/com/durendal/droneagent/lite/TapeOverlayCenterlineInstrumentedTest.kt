package com.durendal.droneagent.lite

import android.graphics.Bitmap
import android.graphics.Canvas
import java.io.File
import android.view.View
import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith

/**
 * Renders the overlay and inspects the pixels.
 *
 * The overlay draws the same centerline the controller follows, so what an
 * operator sees is what the aircraft is steering by. That claim is only worth
 * making if something actually reaches the screen, so this draws the view for
 * real and looks for the line rather than trusting that the draw call was made.
 */
@RunWith(AndroidJUnit4::class)
class TapeOverlayCenterlineInstrumentedTest {

    private val context = InstrumentationRegistry.getInstrumentation().targetContext

    @Test
    fun aFullPathDrawsItsCenterlineAnchorAndLookahead() {
        val bitmap = render(centerlinePath(withLookahead = true))

        val shadowPixels = centerlinePixelPositions(bitmap)
        assertTrue("no centerline was drawn at all", shadowPixels.size > 50)
        // The synthetic chain runs up the middle, so the line must appear across
        // most of the view's height rather than as one isolated blob.
        val topmost = shadowPixels.minOf { it.second }
        val bottommost = shadowPixels.maxOf { it.second }
        assertTrue(
            "the centerline spans only ${bottommost - topmost}px of $VIEW_HEIGHT",
            bottommost - topmost > VIEW_HEIGHT / 2,
        )
        assertTrue(
            "the centerline is not near the horizontal centre",
            shadowPixels.map { it.first }.average() in (VIEW_WIDTH * 0.35)..(VIEW_WIDTH * 0.65),
        )
    }

    @Test
    fun aNearFieldOnlyPathDrawsNoLookaheadMarker() {
        val withTarget = centerlinePixelPositions(render(centerlinePath(withLookahead = true))).size
        val withoutTarget = centerlinePixelPositions(render(centerlinePath(withLookahead = false))).size

        // The look-ahead marker is a ring, so dropping it must visibly remove
        // pixels. An operator has to be able to tell the two states apart.
        assertTrue(
            "near-field-only drew $withoutTarget pixels, full path drew $withTarget",
            withoutTarget < withTarget,
        )
    }

    @Test
    fun theOverlayShowsTheCenterlineEvenWithNoAcceptedDetection() {
        val view = overlay()
        view.showDetection(null)
        view.showCenterline(centerlinePath(withLookahead = true))

        val shadowPixels = centerlinePixelPositions(draw(view))

        assertTrue("the centerline vanished when the old estimator found nothing", shadowPixels.isNotEmpty())
    }

    @Test
    fun theLabelReportsTheQualityAndTheReasonItIsNotFull() {
        assertEquals(
            "CENTERLINE FULL  40pt",
            centerlinePath(withLookahead = true).label,
        )
        assertEquals(
            "CENTERLINE NEAR ONLY  40pt  INSUFFICIENT_LOOKAHEAD",
            centerlinePath(withLookahead = false).label,
        )
    }

    /**
     * Renders the overlay over a frame shaped like what the camera sends, and
     * writes it where it can be pulled off the device. This is the artefact the
     * claim "the centerline is visible on the preview" actually rests on.
     */
    @Test
    fun aPreviewCompositeIsWrittenForInspection() {
        val frame = curvedTapePreview()
        val detector = CenterlineExtractor()
        val mask = maskFromPreview(frame)
        val estimate = detector.extract(mask)
        val measurement = CenterlineMeasurement.measure(estimate, VIEW_WIDTH, VIEW_HEIGHT)
        assertTrue("the extractor found nothing to draw", estimate.points.size >= 2)
        requireNotNull(measurement)

        val view = overlay()
        view.showCenterline(
            TapeCenterlinePath(
                sourceWidth = VIEW_WIDTH,
                sourceHeight = VIEW_HEIGHT,
                xFractions = FloatArray(estimate.points.size) {
                    (estimate.points[it].x / VIEW_WIDTH).toFloat()
                },
                yFractions = FloatArray(estimate.points.size) {
                    (estimate.points[it].y / VIEW_HEIGHT).toFloat()
                },
                anchorXFraction = measurement.anchorXFraction.toFloat(),
                anchorYFraction = measurement.anchorYFraction.toFloat(),
                lookaheadXFraction = measurement.lookahead?.xFraction?.toFloat(),
                lookaheadYFraction = measurement.lookahead?.yFraction?.toFloat(),
                quality = if (measurement.lookahead == null) {
                    PathQuality.NEAR_FIELD_ONLY
                } else {
                    PathQuality.FULL_PATH
                },
                rejection = null,
            ),
        )

        val composite = frame.copy(Bitmap.Config.ARGB_8888, true)
        view.draw(Canvas(composite))
        // Internal storage: it is the only app-private location a shell can read
        // back with run-as, which is how this artefact leaves the device.
        val output = File(context.filesDir, "centerline-preview.png")
        output.outputStream().use { composite.compress(Bitmap.CompressFormat.PNG, 100, it) }
        assertTrue("composite was not written", output.length() > 0)
    }

    /** A brown board with one curved dark strip, as the detector's fixtures use. */
    private fun curvedTapePreview(): Bitmap =
        Bitmap.createBitmap(VIEW_WIDTH, VIEW_HEIGHT, Bitmap.Config.ARGB_8888).also { bitmap ->
            for (y in 0 until VIEW_HEIGHT) {
                val forward = (VIEW_HEIGHT - 1 - y) / (VIEW_HEIGHT - 1.0)
                val center = VIEW_WIDTH / 2.0 + 140.0 * forward * forward
                for (x in 0 until VIEW_WIDTH) {
                    val onTape = kotlin.math.abs(x - center) <= 15.0
                    bitmap.setPixel(
                        x,
                        y,
                        if (onTape) 0xFF141414.toInt() else 0xFFB47832.toInt(),
                    )
                }
            }
        }

    private fun maskFromPreview(frame: Bitmap): SegmentationMask {
        val tape = BooleanArray(VIEW_WIDTH * VIEW_HEIGHT)
        for (y in 0 until VIEW_HEIGHT) {
            for (x in 0 until VIEW_WIDTH) {
                // Split on red: the board is 0xB4 there and the tape 0x14, while
                // their blue channels are close enough to classify the whole
                // board as tape.
                val red = (frame.getPixel(x, y) shr 16) and 0xFF
                tape[y * VIEW_WIDTH + x] = red < TAPE_RED_THRESHOLD
            }
        }
        return SegmentationMask(VIEW_WIDTH, VIEW_HEIGHT, tape)
    }

    private fun render(path: TapeCenterlinePath): Bitmap {
        val view = overlay()
        view.showCenterline(path)
        return draw(view)
    }

    private fun overlay(): TapeOverlayView = TapeOverlayView(context).apply {
        measure(
            View.MeasureSpec.makeMeasureSpec(VIEW_WIDTH, View.MeasureSpec.EXACTLY),
            View.MeasureSpec.makeMeasureSpec(VIEW_HEIGHT, View.MeasureSpec.EXACTLY),
        )
        layout(0, 0, VIEW_WIDTH, VIEW_HEIGHT)
    }

    private fun draw(view: TapeOverlayView): Bitmap =
        Bitmap.createBitmap(VIEW_WIDTH, VIEW_HEIGHT, Bitmap.Config.ARGB_8888).also { bitmap ->
            view.draw(Canvas(bitmap))
        }

    /** Tolerant of antialiasing: the overlay colour is the only strong magenta. */
    private fun centerlinePixelPositions(bitmap: Bitmap): List<Pair<Int, Int>> = buildList {
        for (y in 0 until bitmap.height) {
            for (x in 0 until bitmap.width) {
                val pixel = bitmap.getPixel(x, y)
                val red = (pixel shr 16) and 0xFF
                val green = (pixel shr 8) and 0xFF
                val blue = pixel and 0xFF
                if (red > 200 && green < 140 && blue > 150) add(x to y)
            }
        }
    }

    /** A chain up the middle of the frame, as the extractor emits them. */
    private fun centerlinePath(withLookahead: Boolean): TapeCenterlinePath {
        val pointCount = 40
        val xFractions = FloatArray(pointCount) { 0.5f }
        val yFractions = FloatArray(pointCount) { index -> 0.95f - index * 0.02f }
        return TapeCenterlinePath(
            sourceWidth = 640,
            sourceHeight = 360,
            xFractions = xFractions,
            yFractions = yFractions,
            anchorXFraction = 0.5f,
            anchorYFraction = 0.95f,
            lookaheadXFraction = if (withLookahead) 0.5f else null,
            lookaheadYFraction = if (withLookahead) 0.4f else null,
            quality = if (withLookahead) PathQuality.FULL_PATH else PathQuality.NEAR_FIELD_ONLY,
            rejection = if (withLookahead) null else "INSUFFICIENT_LOOKAHEAD",
        )
    }

    private companion object {
        const val VIEW_WIDTH = 640
        const val VIEW_HEIGHT = 360
        const val TAPE_RED_THRESHOLD = 0x60
    }
}
