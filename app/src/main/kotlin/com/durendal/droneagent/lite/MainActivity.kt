package com.durendal.droneagent.lite

import android.app.Activity
import android.graphics.Color
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.view.Gravity
import android.view.MotionEvent
import android.view.View
import android.view.ViewGroup
import android.widget.Button
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
 */
class MainActivity : Activity() {

    private lateinit var statusView: TextView
    private lateinit var takeoffButton: Button
    private lateinit var landButton: Button
    private lateinit var virtualStickButton: Button
    private lateinit var stickPad: ViewGroup

    private var registered = false
    private var aircraftConnected = false
    private var flying = false
    private var landingRequested = false
    private var confirmationNeeded = false
    private var confirmAttempts = 0
    private var stickStatus = VirtualStickStatus()

    private val mainHandler = Handler(Looper.getMainLooper())
    private val virtualStick = VirtualStickSession { status ->
        stickStatus = status
        render(if (status.enabled) "虛擬搖桿已開啟，控制權=${status.authority}" else "虛擬搖桿已關閉")
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
        if (stickStatus.enabled) virtualStick.disable {}
        virtualStick.close()
        runCatching { KeyManager.getInstance().cancelListen(this) }
        super.onDestroy()
    }

    // ---------------------------------------------------------------- UI ----

    private fun buildUi(): ViewGroup {
        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.CENTER_HORIZONTAL
            setBackgroundColor(Color.BLACK)
            setPadding(32, 32, 32, 32)
        }
        statusView = TextView(this).apply {
            textSize = 15f
            setTextColor(Color.WHITE)
            gravity = Gravity.CENTER
        }
        takeoffButton = Button(this).apply {
            text = "起飛並停留"
            isEnabled = false
            setOnClickListener { takeoff() }
        }
        landButton = Button(this).apply {
            text = "降落"
            isEnabled = false
            setOnClickListener { land() }
        }
        virtualStickButton = Button(this).apply {
            text = "開啟虛擬搖桿"
            isEnabled = false
            setOnClickListener { toggleVirtualStick() }
        }
        stickPad = buildStickPad()
        root.addView(statusView)
        root.addView(takeoffButton)
        root.addView(landButton)
        root.addView(virtualStickButton)
        root.addView(stickPad)
        return root
    }

    /**
     * Hold-to-move pad. Press sets one axis, release zeroes it, so lifting a
     * finger is an explicit neutral rather than a stale command left behind.
     */
    private fun buildStickPad(): ViewGroup = LinearLayout(this).apply {
        orientation = LinearLayout.VERTICAL
        visibility = View.GONE
        addView(
            stickRow(
                "前" to (StickAxis.FORWARD to HORIZONTAL_MPS),
                "後" to (StickAxis.FORWARD to -HORIZONTAL_MPS),
            ),
        )
        addView(
            stickRow(
                "左" to (StickAxis.RIGHT to -HORIZONTAL_MPS),
                "右" to (StickAxis.RIGHT to HORIZONTAL_MPS),
            ),
        )
        addView(
            stickRow(
                "上升" to (StickAxis.UP to VERTICAL_MPS),
                "下降" to (StickAxis.UP to -VERTICAL_MPS),
            ),
        )
        addView(
            stickRow(
                "左轉" to (StickAxis.YAW to -YAW_DEGREES_PER_SECOND),
                "右轉" to (StickAxis.YAW to YAW_DEGREES_PER_SECOND),
            ),
        )
    }

    private fun stickRow(vararg buttons: Pair<String, Pair<StickAxis, Double>>): ViewGroup =
        LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            buttons.forEach { (label, movement) ->
                addView(
                    holdButton(label, movement.first, movement.second),
                    LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f),
                )
            }
        }

    private fun holdButton(label: String, axis: StickAxis, value: Double): Button =
        Button(this).apply {
            text = label
            setOnTouchListener { view, event ->
                when (event.actionMasked) {
                    MotionEvent.ACTION_DOWN -> {
                        virtualStick.setAxis(axis, value)
                        view.isPressed = true
                    }
                    MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                        virtualStick.setAxis(axis, 0.0)
                        view.isPressed = false
                    }
                }
                true
            }
        }

    /** Single place that turns state into what the operator sees. */
    private fun render(message: String) = runOnUiThread {
        statusView.text = buildString {
            append("registered=").append(registered)
            append("\naircraft=").append(if (aircraftConnected) "connected" else "disconnected")
            append("\nflying=").append(flying)
            append("\nlandingConfirmNeeded=").append(confirmationNeeded)
            append("\nvirtualStick=").append(stickStatus.enabled)
            append(" advanced=").append(stickStatus.advancedMode)
            append(" authority=").append(stickStatus.authority)
            append("\n\n").append(message)
        }
        val ready = registered && aircraftConnected
        takeoffButton.isEnabled = ready && !flying && !stickStatus.enabled
        landButton.isEnabled = ready && flying
        virtualStickButton.isEnabled = ready
        virtualStickButton.text = if (stickStatus.enabled) "關閉虛擬搖桿（交回遙控器）" else "開啟虛擬搖桿"
        stickPad.visibility = if (stickStatus.enabled) View.VISIBLE else View.GONE
    }

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
            render(if (aircraftConnected) "飛機已連線，可以起飛" else "飛機未連線")
        }
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyIsFlying),
            this,
            true,
        ) { _, isFlying ->
            flying = isFlying == true
            if (!flying) landingRequested = false
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

    private fun toggleVirtualStick() {
        virtualStickButton.isEnabled = false
        if (stickStatus.enabled) {
            virtualStick.disable { error ->
                virtualStickButton.isEnabled = true
                render(error?.let { "關閉虛擬搖桿失敗：$it" } ?: "虛擬搖桿已關閉，控制權交回遙控器")
            }
            return
        }
        // Taking the control link cancels the app's landing intent: stick input
        // and an auto-landing descent are two owners of the same aircraft.
        landingRequested = false
        virtualStick.enable { error ->
            virtualStickButton.isEnabled = true
            render(error?.let { "開啟虛擬搖桿失敗：$it" } ?: "虛擬搖桿已開啟，按住方向鍵移動")
        }
    }

    private companion object {
        const val TAG = "LiteMainActivity"

        /** Deliberately gentle for a first stick trial. */
        const val HORIZONTAL_MPS = 0.5
        const val VERTICAL_MPS = 0.3
        const val YAW_DEGREES_PER_SECOND = 20.0

        const val MAX_CONFIRM_ATTEMPTS = 8
        const val CONFIRM_RETRY_MS = 700L
    }
}
