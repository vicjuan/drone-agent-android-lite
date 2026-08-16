package com.durendal.droneagent.lite

import android.app.Activity
import android.graphics.Color
import android.graphics.drawable.GradientDrawable
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.util.TypedValue
import android.view.Gravity
import android.view.View
import android.view.ViewGroup
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.TextView
import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.common.register.DJISDKInitEvent
import dji.v5.manager.KeyManager
import dji.v5.manager.SDKManager
import dji.v5.manager.interfaces.SDKManagerCallback

/**
 * Minimal DJI Mini 4 Pro takeoff / landing / virtual-stick demo.
 *
 * Lifecycle, in order — every step is a precondition of the next one:
 *   1. MSDK init            SDKManager.init
 *   2. app registration     SDKManager.registerApp (needs internet + valid app key)
 *   3. aircraft connected   FlightControllerKey.KeyConnection == true
 *   4. takeoff              performAction(FlightControllerKey.KeyStartTakeoff)
 *
 * KeyStartTakeoff is the aircraft's own auto-takeoff: it spins up, climbs to
 * roughly 1.2 m and then hovers on its own.
 *
 * Landing is not a single action. KeyStartAutoLanding only begins the descent;
 * near the ground the Mini 4 Pro's landing protection stops and waits for
 * KeyConfirmLanding, reporting the wait through KeyIsLandingConfirmationNeeded.
 * An app that never confirms leaves the aircraft hovering low with an "accepted"
 * landing command — the exact failure observed on 2026-08-14. This activity
 * therefore confirms automatically, but only for a landing it started itself.
 *
 * The interface — live camera behind a translucent status panel, Mode 2 dual
 * analogue sticks, pill actions — follows the main drone-agent-android debug
 * dashboard so an operator reads one layout on both apps. Only the look is
 * borrowed; the control path stays this app's single flown one.
 */
class MainActivity : Activity() {

    private lateinit var preview: CameraPreview
    private lateinit var headlineView: TextView
    private lateinit var detailView: TextView
    private lateinit var telemetryView: TextView
    private lateinit var takeoffButton: TextView
    private lateinit var landButton: TextView
    private lateinit var leftPad: StickPadView
    private lateinit var rightPad: StickPadView

    private var registered = false
    private var aircraftConnected = false
    private var flying = false
    private var landingRequested = false
    private var confirmationNeeded = false
    private var confirmAttempts = 0
    private var stickStatus = VirtualStickStatus()

    /** True once MSDK accepted enableVirtualStick; the app then owns the control link. */
    private var stickOwned = false
    private var stickTransitionPending = false

    private val mainHandler = Handler(Looper.getMainLooper())
    private val virtualStick = VirtualStickSession { status ->
        stickStatus = status
        render("控制權=${status.authority}")
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(buildUi())
        render("啟動 MSDK…")
        startSdk()
    }

    override fun onDestroy() {
        // Listeners are held by `this`; leaking them across an Activity restart
        // would deliver key updates to a dead view hierarchy. The stick link is
        // released too, so a killed UI can never leave a live control link.
        mainHandler.removeCallbacksAndMessages(null)
        if (stickOwned) virtualStick.disable {}
        virtualStick.close()
        preview.release()
        runCatching { KeyManager.getInstance().cancelListen(this) }
        super.onDestroy()
    }

    // ---------------------------------------------------------------- UI ----

    private fun buildUi(): ViewGroup {
        val root = FrameLayout(this).apply { setBackgroundColor(Color.BLACK) }

        preview = CameraPreview(this)
        root.addView(
            preview.view,
            FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.MATCH_PARENT,
            ),
        )
        root.addView(
            buildTopBar(),
            FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT,
                Gravity.TOP,
            ).apply { setMargins(dp(10), dp(10), dp(10), dp(10)) },
        )
        root.addView(
            buildStickBar(),
            FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.WRAP_CONTENT,
                Gravity.BOTTOM,
            ).apply { setMargins(dp(10), dp(10), dp(10), dp(14)) },
        )
        return root
    }

    /**
     * One row: takeoff on the left, status in the middle, landing on the right.
     * The panel takes the leftover width, so a long status line can never slide
     * under either action, and the two actions stay physically apart — reaching
     * for one can never graze the other.
     */
    private fun buildTopBar(): ViewGroup = LinearLayout(this).apply {
        orientation = LinearLayout.HORIZONTAL
        takeoffButton = PillButton("起飛並停留", StickPadView.GREEN) { takeoff() }
        landButton = PillButton("降落", StickPadView.AMBER) { land() }
        addView(takeoffButton, actionParams(marginEnd = dp(10)))
        addView(
            buildStatusPanel(),
            LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f),
        )
        addView(landButton, actionParams(marginStart = dp(10)))
    }

    private fun actionParams(marginStart: Int = 0, marginEnd: Int = 0) =
        LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.WRAP_CONTENT,
            ViewGroup.LayoutParams.WRAP_CONTENT,
        ).apply {
            gravity = Gravity.TOP
            this.marginStart = marginStart
            this.marginEnd = marginEnd
        }

    private fun buildStatusPanel(): ViewGroup {
        headlineView = label(11f, StickPadView.CYAN, bold = true)
        detailView = label(10f, StickPadView.TEXT)
        telemetryView = label(9f, StickPadView.MUTED)
        return LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            background = panelBackground()
            setPadding(dp(12), dp(8), dp(12), dp(8))
            addView(headlineView)
            addView(detailView)
            addView(telemetryView)
        }
    }


    /**
     * Mode 2, the layout every DJI pilot already has in their hands:
     * left stick is yaw and climb, right stick is lateral and forward.
     */
    private fun buildStickBar(): ViewGroup {
        leftPad = stickPad(
            StickSide.LEFT,
            StickAxisLabels(up = "上升", down = "下降", left = "左旋轉", right = "右旋轉"),
        )
        rightPad = stickPad(
            StickSide.RIGHT,
            StickAxisLabels(up = "前進", down = "後退", left = "左移", right = "右移"),
        )
        return LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.BOTTOM
            addView(padColumn(leftPad, "左桿 • 旋轉 / 升降"))
            addView(
                View(this@MainActivity),
                LinearLayout.LayoutParams(0, 1, 1f),
            )
            addView(padColumn(rightPad, "右桿 • 平移"))
        }
    }

    private fun stickPad(side: StickSide, labels: StickAxisLabels) = StickPadView(this).apply {
        isEnabled = false
        axisLabels = labels
        onPosition = { x, y -> virtualStick.setStick(side, x, y) }
    }

    private fun padColumn(pad: StickPadView, caption: String): ViewGroup =
        LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.CENTER_HORIZONTAL
            addView(pad)
            addView(
                label(9f, StickPadView.TEXT, bold = true).apply {
                    text = caption
                    gravity = Gravity.CENTER
                },
            )
        }

    /** Single place that turns state into what the operator sees. */
    private fun render(message: String) = runOnUiThread {
        val ready = registered && aircraftConnected
        val (headline, headlineColor) = when {
            !registered -> "LOCAL FLIGHT • REGISTERING" to StickPadView.AMBER
            !aircraftConnected -> "LOCAL FLIGHT • NO AIRCRAFT" to StickPadView.RED
            confirmationNeeded -> "LOCAL FLIGHT • LANDING CONFIRM" to StickPadView.AMBER
            stickStatus.enabled -> "LOCAL FLIGHT • MANUAL" to StickPadView.GREEN
            flying -> "LOCAL FLIGHT • AIRBORNE" to StickPadView.GREEN
            else -> "LOCAL FLIGHT • READY" to StickPadView.CYAN
        }
        headlineView.text = headline
        headlineView.setTextColor(headlineColor)
        detailView.text = message
        telemetryView.text = buildString {
            append("registered=").append(registered)
            append(" · aircraft=").append(if (aircraftConnected) "connected" else "disconnected")
            append(" · flying=").append(flying)
            append("\nstick=").append(stickStatus.enabled)
            append(" advanced=").append(stickStatus.advancedMode)
            append(" authority=").append(stickStatus.authority)
            append(" · landingConfirmNeeded=").append(confirmationNeeded)
        }

        takeoffButton.isEnabled = ready && !flying
        landButton.isEnabled = ready && flying
        leftPad.isEnabled = stickOwned
        rightPad.isEnabled = stickOwned
    }

    // ------------------------------------------------------- UI plumbing ----

    private fun label(sizeSp: Float, color: Int, bold: Boolean = false) = TextView(this).apply {
        textSize = sizeSp
        setTextColor(color)
        letterSpacing = 0.06f
        if (bold) setTypeface(typeface, android.graphics.Typeface.BOLD)
    }

    private fun panelBackground() = GradientDrawable().apply {
        cornerRadius = dp(14).toFloat()
        setColor(StickPadView.PANEL_COLOR)
        setStroke(dp(1), StickPadView.STROKE_COLOR)
    }

    /**
     * Flat pill action in the dashboard's accent palette. It owns its own
     * enabled styling so callers only ever set [isEnabled], and a disabled pill
     * cannot be clicked into an action.
     */
    private inner class PillButton(
        caption: String,
        private val accent: Int,
        private val onPressed: () -> Unit,
    ) : TextView(this@MainActivity) {
        init {
            text = caption
            textSize = 11f
            gravity = Gravity.CENTER
            setTypeface(typeface, android.graphics.Typeface.BOLD)
            setPadding(dp(16), dp(9), dp(16), dp(9))
            layoutParams = LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT,
                ViewGroup.LayoutParams.WRAP_CONTENT,
            ).apply { topMargin = dp(6) }
            setOnClickListener { if (isEnabled) onPressed() }
            isEnabled = false
        }

        override fun setEnabled(enabled: Boolean) {
            if (enabled == isEnabled) return
            super.setEnabled(enabled)
            applyAccent()
        }

        private fun applyAccent() {
            val color = if (isEnabled) accent else StickPadView.MUTED
            setTextColor(color)
            background = GradientDrawable().apply {
                cornerRadius = dp(18).toFloat()
                setColor(color and 0x00FFFFFF or PILL_FILL_ALPHA)
                setStroke(dp(1), color)
            }
        }
    }

    private fun dp(value: Int): Int =
        TypedValue.applyDimension(
            TypedValue.COMPLEX_UNIT_DIP,
            value.toFloat(),
            resources.displayMetrics,
        ).toInt()

    // -------------------------------------------------- SDK registration ----

    private fun startSdk() {
        SDKManager.getInstance().init(
            applicationContext,
            object : SDKManagerCallback {
                override fun onInitProcess(event: DJISDKInitEvent, totalProcess: Int) {
                    Log.i(TAG, "init event=$event total=$totalProcess")
                    if (event == DJISDKInitEvent.INITIALIZE_COMPLETE) {
                        // registerApp is only legal after INITIALIZE_COMPLETE.
                        render("MSDK 初始化完成，開始註冊 app key…")
                        SDKManager.getInstance().registerApp()
                    }
                }

                override fun onRegisterSuccess() {
                    registered = true
                    render("app key 註冊成功，等待飛機連線…")
                    listenAircraftState()
                    virtualStick.start()
                    runOnUiThread { preview.refresh() }
                }

                override fun onRegisterFailure(error: IDJIError) {
                    registered = false
                    Log.e(TAG, "registration failed: $error")
                    render("註冊失敗：${error.description() ?: error}")
                }

                override fun onProductConnect(productId: Int) {
                    Log.i(TAG, "product connected: $productId")
                }

                override fun onProductDisconnect(productId: Int) {
                    Log.i(TAG, "product disconnected: $productId")
                }

                override fun onProductChanged(productId: Int) {
                    Log.i(TAG, "product changed: $productId")
                }

                override fun onDatabaseDownloadProgress(current: Long, total: Long) {
                    Log.i(TAG, "fly-safe database: $current/$total")
                }
            },
        )
    }

    /**
     * KeyConnection on the *flight controller* means the aircraft itself is
     * reachable. ProductKey.KeyConnection would already be true with only the RC
     * attached, and a takeoff issued in that state is refused.
     */
    private fun listenAircraftState() {
        val keyManager = KeyManager.getInstance()
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyConnection),
            this,
            true,
        ) { _, connected ->
            aircraftConnected = connected == true
            // The camera stream only exists while the aircraft is linked, and the
            // Surface usually exists first, so the link is what triggers attach.
            runOnUiThread { preview.refresh() }
            syncStickOwnership()
            render(if (aircraftConnected) "飛機已連線，可以起飛" else "飛機未連線")
        }
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyIsFlying),
            this,
            true,
        ) { _, isFlying ->
            flying = isFlying == true
            if (!flying) landingRequested = false
            syncStickOwnership()
            render(if (flying) "飛行中（自動懸停）" else "在地面")
        }
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyIsLandingConfirmationNeeded),
            this,
            true,
        ) { _, needed ->
            confirmationNeeded = needed == true
            if (confirmationNeeded) {
                confirmLanding()
            } else {
                render("降落無需確認")
            }
        }
    }

    // -------------------------------------------------------- actuation ----

    private fun takeoff() {
        render("送出起飛指令…")
        performAction(KeyTools.createKey(FlightControllerKey.KeyStartTakeoff), "起飛")
    }

    private fun land() {
        landingRequested = true
        confirmAttempts = 0
        render("送出降落指令…")
        performAction(KeyTools.createKey(FlightControllerKey.KeyStartAutoLanding), "降落")
        // Landing protection may already be waiting when the descent starts.
        if (confirmationNeeded) confirmLanding()
    }

    /**
     * Answers the aircraft's landing-protection prompt for a landing this app
     * started. It is retried because the prompt stays up until it is answered:
     * one refused confirmation must not strand the aircraft in a low hover.
     */
    private fun confirmLanding() {
        if (!landingRequested || !confirmationNeeded) return
        if (confirmAttempts >= MAX_CONFIRM_ATTEMPTS) {
            render("降落確認 $MAX_CONFIRM_ATTEMPTS 次都失敗，請用遙控器降落")
            return
        }
        confirmAttempts += 1
        render("低空降落保護要求確認，自動確認中（第 $confirmAttempts 次）…")
        KeyManager.getInstance().performAction(
            KeyTools.createKey(FlightControllerKey.KeyConfirmLanding),
            object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(result: EmptyMsg) {
                    Log.i(TAG, "landing confirmation accepted")
                    render("降落確認已被飛機接受，正在觸地…")
                }

                override fun onFailure(error: IDJIError) {
                    Log.e(TAG, "landing confirmation refused: $error")
                    render("降落確認被拒絕：${error.description() ?: error}，重試中…")
                    mainHandler.postDelayed(::confirmLanding, CONFIRM_RETRY_MS)
                }
            },
        )
    }

    private fun performAction(key: DJIKey.ActionKey<*, EmptyMsg>, label: String) {
        KeyManager.getInstance().performAction(
            key,
            object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(result: EmptyMsg) {
                    Log.i(TAG, "$label accepted")
                    render("$label 指令已被飛機接受")
                }

                override fun onFailure(error: IDJIError) {
                    // A refusal is the normal outcome of an unmet precondition
                    // (props off, no GPS, RC in the wrong mode). Show it verbatim.
                    Log.e(TAG, "$label refused: $error")
                    render("$label 被拒絕：${error.description() ?: error}")
                }
            },
        )
    }

    // ----------------------------------------------------- virtual stick ----

    /**
     * The sticks follow the flight state instead of a button: airborne means the
     * app holds the control link, on the ground it hands the link back. There is
     * no "turn the sticks off" action because a hovering aircraft with no live
     * link is exactly the state an operator must never be dropped into — and the
     * physical RC still takes over at any moment simply by moving its own sticks.
     */
    private fun syncStickOwnership() {
        val shouldOwn = registered && aircraftConnected && flying
        if (shouldOwn == stickOwned || stickTransitionPending) return
        stickTransitionPending = true
        if (shouldOwn) {
            // Stick input and an auto-landing descent are two owners of the same
            // aircraft, so taking the link cancels this app's landing intent.
            landingRequested = false
            virtualStick.enable { error ->
                stickTransitionPending = false
                stickOwned = error == null
                render(error?.let { "取得控制權失敗：$it" } ?: "已取得控制權，推動搖桿即可移動")
            }
        } else {
            virtualStick.disable { error ->
                stickTransitionPending = false
                stickOwned = false
                render(error?.let { "釋放控制權失敗：$it" } ?: "已釋放控制權，交回遙控器")
            }
        }
    }

    private companion object {
        const val TAG = "LiteMainActivity"

        /** Pill fill: the accent colour at ~12% opacity, matching the dashboard panels. */
        const val PILL_FILL_ALPHA = 0x1F000000

        const val MAX_CONFIRM_ATTEMPTS = 8
        const val CONFIRM_RETRY_MS = 700L
    }
}
