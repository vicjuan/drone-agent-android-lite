package com.durendal.droneagent.lite

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.view.View
import kotlin.math.min

/** Draws the latest detector result over the CENTER_INSIDE DJI preview. */
class TapeOverlayView(context: Context) : View(context) {

    private val boxPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = DETECTED_COLOR
        style = Paint.Style.STROKE
        strokeWidth = density(3f)
    }
    private val labelBackgroundPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.argb(210, 0, 0, 0)
        style = Paint.Style.FILL
    }
    private val labelPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = density(14f)
        isFakeBoldText = true
    }

    private var detection: TapeDetection? = null

    fun showDetection(value: TapeDetection?) {
        detection = value
        postInvalidateOnAnimation()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val result = detection
        if (result == null) {
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
        val bounds = RectF(
            previewLeft + (source.left * previewWidth).toFloat(),
            previewTop + (source.top * previewHeight).toFloat(),
            previewLeft + (source.right * previewWidth).toFloat(),
            previewTop + (source.bottom * previewHeight).toFloat(),
        )
        canvas.drawRoundRect(bounds, density(6f), density(6f), boxPaint)

        val label = "BLACK TAPE ${(result.confidence * 100).toInt()}%"
        val padding = density(6f)
        val textWidth = labelPaint.measureText(label)
        val textHeight = labelPaint.fontMetrics.run { bottom - top }
        val labelTop = (bounds.top - textHeight - padding * 2).coerceAtLeast(0f)
        val labelBounds = RectF(
            bounds.left,
            labelTop,
            bounds.left + textWidth + padding * 2,
            labelTop + textHeight + padding * 2,
        )
        canvas.drawRoundRect(labelBounds, density(4f), density(4f), labelBackgroundPaint)
        canvas.drawText(
            label,
            labelBounds.left + padding,
            labelBounds.top + padding - labelPaint.fontMetrics.top,
            labelPaint,
        )
    }

    private fun drawSearchStatus(canvas: Canvas) {
        val label = "OpenCV • 搜尋黑膠帶"
        val padding = density(6f)
        val left = density(16f)
        val top = density(72f)
        val textWidth = labelPaint.measureText(label)
        val textHeight = labelPaint.fontMetrics.run { bottom - top }
        val bounds = RectF(
            left,
            top,
            left + textWidth + padding * 2,
            top + textHeight + padding * 2,
        )
        canvas.drawRoundRect(bounds, density(4f), density(4f), labelBackgroundPaint)
        canvas.drawText(
            label,
            bounds.left + padding,
            bounds.top + padding - labelPaint.fontMetrics.top,
            labelPaint,
        )
    }

    private fun density(value: Float): Float = value * resources.displayMetrics.density

    private companion object {
        val DETECTED_COLOR = Color.rgb(0, 230, 118)
    }
}
