package com.durendal.droneagent.lite

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.view.View
import kotlin.math.min
import kotlin.math.roundToInt

/** Draws the latest detector result over the CENTER_INSIDE DJI preview. */
class TapeOverlayView(context: Context) : View(context) {

    private val boxPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = DETECTED_COLOR
        style = Paint.Style.STROKE
        strokeWidth = density(3f)
    }
    private val railPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.CYAN
        style = Paint.Style.STROKE
        strokeWidth = density(3f)
    }
    private val anchorPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.YELLOW
        style = Paint.Style.FILL
    }
    private val targetPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        style = Paint.Style.STROKE
        strokeWidth = density(2f)
    }
    private val labelBackgroundPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(210, 0, 0, 0)
        style = Paint.Style.FILL
    }
    // The centerline the controller follows, drawn in its own colour so it reads
    // apart from the detection box and the aircraft reference cross.
    private val centerlinePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = CENTERLINE_COLOR
        style = Paint.Style.STROKE
        strokeWidth = density(2f)
    }
    private val centerlineAnchorPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = CENTERLINE_COLOR
        style = Paint.Style.FILL
    }
    private val centerlineLookaheadPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = CENTERLINE_COLOR
        style = Paint.Style.STROKE
        strokeWidth = density(2f)
    }
    private val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = density(14f)
        isFakeBoldText = true
    }

    // The overlay redraws at camera frame rate, so every reusable value is resolved once
    // here instead of per draw: dp conversions, font metrics and the two rectangles.
    private val boxCornerRadius = density(6f)
    private val labelCornerRadius = density(4f)
    private val labelPadding = density(6f)
    private val anchorRadius = density(6f)
    private val targetRadius = density(10f)
    private val searchLabelLeft = density(16f)
    private val searchLabelTop = density(72f)
    private val centerlineLabelTop = density(108f)
    private val labelBaselineOffset = labelPadding - labelPaint.fontMetrics.top
    private val labelTextHeight = labelPaint.fontMetrics.run { bottom - top }
    private val searchLabelWidth = labelPaint.measureText(SEARCH_LABEL)

    private val detectionBounds = RectF()
    private val labelBounds = RectF()

    // Reused across draws: the polyline is rebuilt every frame at camera rate and
    // Canvas.drawLines wants consecutive x0,y0,x1,y1 pairs.
    private var centerlineSegments = FloatArray(0)

    private var detection: TapeDetection? = null
    private var detectionLabel = ""
    private var centerlinePath: TapeCenterlinePath? = null

    fun showDetection(value: TapeDetection?) {
        detection = value
        detectionLabel = value?.let {
            formatTapeDetectionLabel(
                confidence = it.confidence,
                angleFromVerticalDegrees = it.angleFromVerticalDegrees,
            )
        } ?: ""
        postInvalidateOnAnimation()
    }

    /** The path the controller is following, as the controller sees it. */
    fun showCenterline(value: TapeCenterlinePath?) {
        centerlinePath = value
        postInvalidateOnAnimation()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val result = detection
        if (result == null) {
            drawCenterlinePath(canvas)
            drawSearchStatus(canvas)
            return
        }
        val source = result.bounds
        val previewScale = min(
            width / result.sourceWidth.toFloat(),
            height / result.sourceHeight.toFloat(),
        )
        val previewWidth = result.sourceWidth * previewScale
        val previewHeight = result.sourceHeight * previewScale
        val previewLeft = (width - previewWidth) / 2f
        val previewTop = (height - previewHeight) / 2f
        detectionBounds.set(
            previewLeft + (source.left * previewWidth).toFloat(),
            previewTop + (source.top * previewHeight).toFloat(),
            previewLeft + (source.right * previewWidth).toFloat(),
            previewTop + (source.bottom * previewHeight).toFloat(),
        )
        val hasCenterline = centerlinePath != null
        if (!hasCenterline) {
            canvas.drawRoundRect(detectionBounds, boxCornerRadius, boxCornerRadius, boxPaint)
        }

        val anchorX = previewLeft + (result.anchorXFraction * previewWidth).toFloat()
        val anchorY = previewTop + (result.anchorYFraction * previewHeight).toFloat()
        val targetX = previewLeft + previewWidth / 2f
        val targetY = previewTop + previewHeight * TRACKING_TARGET_Y_FRACTION.toFloat()
        if (!hasCenterline) {
            // A near-field-only path has no trustworthy look-ahead. Drawing a rail to
            // a made-up point would show the operator guidance the controller is not
            // allowed to use, so the rail simply disappears with the evidence.
            result.lookahead?.let { lookahead ->
                canvas.drawLine(
                    anchorX,
                    anchorY,
                    previewLeft + (lookahead.xFraction * previewWidth).toFloat(),
                    previewTop + (lookahead.yFraction * previewHeight).toFloat(),
                    railPaint,
                )
            }
            canvas.drawCircle(anchorX, anchorY, anchorRadius, anchorPaint)
        }
        canvas.drawLine(
            targetX - targetRadius,
            targetY,
            targetX + targetRadius,
            targetY,
            targetPaint,
        )
        canvas.drawLine(
            targetX,
            targetY - targetRadius,
            targetX,
            targetY + targetRadius,
            targetPaint,
        )

        drawCenterlinePath(canvas)

        if (hasCenterline) {
            drawLabel(
                canvas,
                detectionLabel,
                searchLabelLeft,
                searchLabelTop,
                labelPaint.measureText(detectionLabel),
            )
        } else {
            val labelTop =
                (detectionBounds.top - labelTextHeight - labelPadding * 2).coerceAtLeast(0f)
            drawLabel(
                canvas,
                detectionLabel,
                detectionBounds.left,
                labelTop,
                labelPaint.measureText(detectionLabel),
            )
        }
    }

    /**
     * Draws the extracted chain, its anchor and its look-ahead target. A path
     * with no look-ahead draws no target: the absence is the point, and inventing
     * a marker would show guidance the geometry did not produce.
     */
    private fun drawCenterlinePath(canvas: Canvas) {
        val path = centerlinePath ?: return
        val previewScale = min(
            width / path.sourceWidth.toFloat(),
            height / path.sourceHeight.toFloat(),
        )
        val previewWidth = path.sourceWidth * previewScale
        val previewHeight = path.sourceHeight * previewScale
        val previewLeft = (width - previewWidth) / 2f
        val previewTop = (height - previewHeight) / 2f

        if (path.pointCount >= 2) {
            val segmentValues = (path.pointCount - 1) * 4
            if (centerlineSegments.size != segmentValues) centerlineSegments = FloatArray(segmentValues)
            for (index in 0 until path.pointCount - 1) {
                val offset = index * 4
                centerlineSegments[offset] = previewLeft + path.xFractions[index] * previewWidth
                centerlineSegments[offset + 1] = previewTop + path.yFractions[index] * previewHeight
                centerlineSegments[offset + 2] = previewLeft + path.xFractions[index + 1] * previewWidth
                centerlineSegments[offset + 3] = previewTop + path.yFractions[index + 1] * previewHeight
            }
            canvas.drawLines(centerlineSegments, centerlinePaint)
        }

        canvas.drawCircle(
            previewLeft + path.anchorXFraction * previewWidth,
            previewTop + path.anchorYFraction * previewHeight,
            anchorRadius,
            centerlineAnchorPaint,
        )
        val lookaheadX = path.lookaheadXFraction
        val lookaheadY = path.lookaheadYFraction
        if (lookaheadX != null && lookaheadY != null) {
            canvas.drawCircle(
                previewLeft + lookaheadX * previewWidth,
                previewTop + lookaheadY * previewHeight,
                targetRadius,
                centerlineLookaheadPaint,
            )
        }
        drawLabel(
            canvas,
            path.label,
            searchLabelLeft,
            centerlineLabelTop,
            labelPaint.measureText(path.label),
        )
    }

    private fun drawSearchStatus(canvas: Canvas) {
        drawLabel(canvas, SEARCH_LABEL, searchLabelLeft, searchLabelTop, searchLabelWidth)
    }

    private fun drawLabel(canvas: Canvas, label: String, left: Float, top: Float, textWidth: Float) {
        labelBounds.set(
            left,
            top,
            left + textWidth + labelPadding * 2,
            top + labelTextHeight + labelPadding * 2,
        )
        canvas.drawRoundRect(labelBounds, labelCornerRadius, labelCornerRadius, labelBackgroundPaint)
        canvas.drawText(label, labelBounds.left + labelPadding, labelBounds.top + labelBaselineOffset, labelPaint)
    }

    private fun density(value: Float): Float = value * resources.displayMetrics.density

    private companion object {
        val DETECTED_COLOR = Color.rgb(0, 230, 118)
        val CENTERLINE_COLOR = Color.rgb(255, 82, 200)
        const val SEARCH_LABEL = "OpenCV • 搜尋黑膠帶"
    }
}

internal fun formatTapeDetectionLabel(
    confidence: Double,
    angleFromVerticalDegrees: Double,
): String {
    val roundedAngle = angleFromVerticalDegrees.roundToInt()
    val signedAngle = if (roundedAngle > 0) "+$roundedAngle" else roundedAngle.toString()
    return "BLACK TAPE ${(confidence * 100).toInt()}%  $signedAngle°"
}
