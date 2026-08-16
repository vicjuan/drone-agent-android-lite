package com.durendal.droneagent.lite

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.util.TypedValue
import android.view.MotionEvent
import android.view.View
import kotlin.math.hypot
import kotlin.math.min

/** What each direction of a pad commands, in the operator's words. */
data class StickAxisLabels(val up: String, val down: String, val left: String, val right: String)

/**
 * One round analogue stick, drawn and behaving like the pad in the main
 * drone-agent-android debug dashboard (`VirtualStickControls.ManualVirtualStickOverlay`).
 *
 * Visuals and feel are deliberately copied from that project — the same panel
 * fill, cyan ring, crosshair and knob travel — because it is the interface the
 * operator already reads. What is NOT copied is its Compose/state machinery:
 * this app has one control owner, so a plain View with a callback is the whole
 * requirement.
 *
 * Output is normalised to [-1, 1] with **y positive upwards**, dead zone
 * applied and magnitude rescaled so the first pixel outside the dead zone is
 * still a near-zero command instead of a step. Releasing the finger reports
 * neutral, so a lifted finger is an explicit stop, never a stale command.
 */
class StickPadView(context: Context) : View(context) {

    /** Called on every touch sample and once more with (0,0) on release. */
    var onPosition: (x: Double, y: Double) -> Unit = { _, _ -> }

    /** Direction captions drawn inside the pad; null draws a bare crosshair. */
    var axisLabels: StickAxisLabels? = null
        set(value) {
            field = value
            invalidate()
        }

    private var normalizedX = 0.0
    private var normalizedY = 0.0

    /** Pointer that owns this pad; a second finger elsewhere must not steal it. */
    private var activePointerId = MotionEvent.INVALID_POINTER_ID

    private val fillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val strokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE }
    private val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        textAlign = Paint.Align.CENTER
    }

    init {
        isClickable = true
    }

    override fun setEnabled(enabled: Boolean) {
        if (enabled == isEnabled) return
        super.setEnabled(enabled)
        if (!enabled) releaseStick()
        invalidate()
    }

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val preferred = dp(PAD_DIAMETER_DP).toInt()
        val width = resolveSize(preferred, widthMeasureSpec)
        val height = resolveSize(preferred, heightMeasureSpec)
        val side = min(width, height)
        setMeasuredDimension(side, side)
    }

    override fun onTouchEvent(event: MotionEvent): Boolean {
        if (!isEnabled) return false
        when (event.actionMasked) {
            MotionEvent.ACTION_DOWN, MotionEvent.ACTION_POINTER_DOWN -> {
                if (activePointerId != MotionEvent.INVALID_POINTER_ID) return false
                val index = event.actionIndex
                activePointerId = event.getPointerId(index)
                parent?.requestDisallowInterceptTouchEvent(true)
                publish(event.getX(index), event.getY(index))
            }
            MotionEvent.ACTION_MOVE -> {
                val index = event.findPointerIndex(activePointerId)
                if (index < 0) return false
                publish(event.getX(index), event.getY(index))
            }
            MotionEvent.ACTION_POINTER_UP -> {
                if (event.getPointerId(event.actionIndex) == activePointerId) releaseStick()
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> releaseStick()
        }
        return true
    }

    private fun publish(xPixels: Float, yPixels: Float) {
        val radius = min(width, height) / 2f
        if (radius <= 0f) return
        val normalized = normalize(xPixels - width / 2f, yPixels - height / 2f, radius)
        normalizedX = normalized.first
        normalizedY = normalized.second
        onPosition(normalizedX, normalizedY)
        invalidate()
    }

    private fun releaseStick() {
        activePointerId = MotionEvent.INVALID_POINTER_ID
        normalizedX = 0.0
        normalizedY = 0.0
        onPosition(0.0, 0.0)
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        val centerX = width / 2f
        val centerY = height / 2f
        val radius = min(width, height) / 2f

        fillPaint.color = PANEL_COLOR
        canvas.drawCircle(centerX, centerY, radius - dp(1f), fillPaint)

        strokePaint.color = if (isEnabled) RING_COLOR else STROKE_COLOR
        strokePaint.strokeWidth = dp(1f)
        canvas.drawCircle(centerX, centerY, radius - dp(2f), strokePaint)

        strokePaint.color = STROKE_COLOR
        canvas.drawLine(centerX, dp(8f), centerX, height - dp(8f), strokePaint)
        canvas.drawLine(dp(8f), centerY, width - dp(8f), centerY, strokePaint)

        drawAxisLabels(canvas, centerX, centerY, radius)

        val travel = radius - dp(23f)
        val knobX = centerX + normalizedX.toFloat() * travel
        val knobY = centerY - normalizedY.toFloat() * travel
        fillPaint.color = if (isEnabled) KNOB_FILL_COLOR else KNOB_DISABLED_COLOR
        canvas.drawCircle(knobX, knobY, dp(20f), fillPaint)
        strokePaint.color = if (isEnabled) CYAN else MUTED
        strokePaint.strokeWidth = dp(2f)
        canvas.drawCircle(knobX, knobY, dp(19f), strokePaint)
    }

    /**
     * What each direction actually does, printed where the finger goes. Yaw in
     * particular is not guessable from a bare circle: on Mode 2 it is the left
     * stick's horizontal axis, and the pad says so.
     */
    private fun drawAxisLabels(canvas: Canvas, centerX: Float, centerY: Float, radius: Float) {
        val inset = dp(22f)
        val labels = axisLabels ?: return
        textPaint.color = if (isEnabled) TEXT else MUTED
        textPaint.textSize = dp(9f)
        val baselineShift = (textPaint.descent() + textPaint.ascent()) / 2f
        canvas.drawText(labels.up, centerX, centerY - radius + inset - baselineShift, textPaint)
        canvas.drawText(labels.down, centerX, centerY + radius - inset - baselineShift, textPaint)
        canvas.drawText(labels.left, centerX - radius + inset, centerY - baselineShift, textPaint)
        canvas.drawText(labels.right, centerX + radius - inset, centerY - baselineShift, textPaint)
    }

    private fun dp(value: Float): Float =
        TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, value, resources.displayMetrics)

    companion object {
        const val PAD_DIAMETER_DP = 146f

        /** Palette of the main project's debug dashboard, so both apps read alike. */
        const val CYAN = 0xFF2FD4CF.toInt()
        const val GREEN = 0xFF35E08A.toInt()
        const val AMBER = 0xFFF5B23E.toInt()
        const val RED = 0xFFFF7676.toInt()
        const val TEXT = 0xFFEAF1F2.toInt()
        const val MUTED = 0xFF93A0A2.toInt()
        const val PANEL_COLOR = 0xC70A1114.toInt()
        const val STROKE_COLOR = 0x5539BDB8.toInt()
        private const val RING_COLOR = 0x7A2FD4CF
        private const val KNOB_FILL_COLOR = 0x4D2FD4CF
        private const val KNOB_DISABLED_COLOR = 0x334D5A5D

        /**
         * Same dead zone as the main project's Mode2VirtualStickTransform: below
         * it the stick is exactly neutral, above it the remaining travel is
         * rescaled to the full range so control stays continuous.
         */
        private const val DEAD_ZONE = 0.08

        /** Returns (x, y) in [-1, 1] with y positive upwards. */
        fun normalize(xPixels: Float, yPixels: Float, radiusPixels: Float): Pair<Double, Double> {
            if (!xPixels.isFinite() || !yPixels.isFinite()) return 0.0 to 0.0
            val rawX = xPixels.toDouble() / radiusPixels
            val rawUp = -yPixels.toDouble() / radiusPixels
            val magnitude = hypot(rawX, rawUp)
            if (magnitude <= DEAD_ZONE) return 0.0 to 0.0
            val clamped = magnitude.coerceAtMost(1.0)
            val output = (clamped - DEAD_ZONE) / (1.0 - DEAD_ZONE)
            return rawX / magnitude * output to rawUp / magnitude * output
        }
    }
}
