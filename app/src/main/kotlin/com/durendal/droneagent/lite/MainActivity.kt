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
import kotlin.math.abs

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
    private lateinit var flightLog: FlightLog
    private lateinit var headlineView: TextView
    private lateinit var detailView: TextView
    private lateinit var holdView: TextView
    private lateinit var telemetryView: TextView
    private lateinit var takeoffButton: PillButton
    private lateinit var landButton: PillButton
    private lateinit var holdButton: PillButton
    private lateinit var registerButton: PillButton
    private lateinit var leftPad: StickPadView
    private lateinit var rightPad: StickPadView

    private var registered = false
    private var registerAttempts = 0
    private var aircraftConnected = false
    private var flying = false
    private var landingRequested = false
    private var confirmationNeeded = false
    private var confirmAttempts = 0

    /** Height when the landing command was issued, to prove the aircraft descended. */
    private var landingHeightAtCommand: Double? = null
    private var stickStatus = VirtualStickStatus()

    /** True once MSDK accepted enableVirtualStick; the app then owns the control link. */
    private var stickOwned = false
    private var stickTransitionPending = false
    private var acquireAttempts = 0

    /** Ground-relative height and when it was observed; null until the first sample. */
    private var altitudeMeters: Double? = null
    private var altitudeAtNanos = 0L
    private var heightSource = HeightSource.NONE

    /** Raw KeyUltrasonicHeight, logged for unit identification; never drives the loop yet. */
    private var ultrasonicRaw: Int? = null

    /** True while the one-shot descent to [TARGET_HEIGHT_METERS] is running. */
    private var holdingHeight = false
    private var holdStartedAtNanos = 0L
    private var holdStableSamples = 0

    /** True once the aircraft named MSDK as authority owner during this manoeuvre. */
    private var holdAuthoritySeen = false

    /** Whether a finger is currently deflecting each stick. */
    private var leftStickActive = false
    private var rightStickActive = false

    /** Latest height-loop line, kept out of [detailView] so nothing overwrites it. */
    private var holdStatus = ""

    private val mainHandler = Handler(Looper.getMainLooper())
    private val virtualStick = VirtualStickSession(
        onStatus = { status ->
            val previous = stickStatus
            stickStatus = status
            if (previous.enabled != status.enabled || previous.authority != status.authority) {
                flightLog.write("stick state enabled=${status.enabled} authority=${status.authority}")
            }
            render("控制權=${status.authority}")
        },
        onFrameSummary = { summary -> flightLog.write(summary) },
    )

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        flightLog = FlightLog(this)
        setContentView(buildUi())
        flightLog.write("app start; log=${flightLog.path}")
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
        flightLog.write("app destroy")
        flightLog.close()
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
        holdButton = PillButton("定高 50 公分", StickPadView.CYAN) { startHeightHold() }
        registerButton = PillButton("重新註冊", StickPadView.AMBER) {
            registerAttempts = 0
            requestRegistration("操作者手動")
        }
        addView(registerButton, actionParams(marginEnd = dp(10)))
        addView(takeoffButton, actionParams(marginEnd = dp(10)))
        addView(holdButton, actionParams(marginEnd = dp(10)))
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
        holdView = label(13f, StickPadView.CYAN, bold = true)
        telemetryView = label(9f, StickPadView.MUTED)
        return LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            background = panelBackground()
            setPadding(dp(12), dp(8), dp(12), dp(8))
            addView(headlineView)
            addView(detailView)
            addView(holdView)
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

    /**
     * The sticks borrow the control link only while a finger is actually on them.
     * A push takes the link, and a few seconds of both sticks centred gives it
     * back to the RC. Holding the link with nothing to say is what let this app
     * fight the operator for the aircraft on 2026-08-17.
     */
    private fun stickPad(side: StickSide, labels: StickAxisLabels) = StickPadView(this).apply {
        isEnabled = false
        axisLabels = labels
        onPosition = { x, y -> onStickMoved(side, x, y) }
    }

    private fun onStickMoved(side: StickSide, x: Double, y: Double) {
        val wasActive = leftStickActive || rightStickActive
        when (side) {
            StickSide.LEFT -> leftStickActive = x != 0.0 || y != 0.0
            StickSide.RIGHT -> rightStickActive = x != 0.0 || y != 0.0
        }
        val isActive = leftStickActive || rightStickActive
        // Only the edges are logged: a drag produces samples far too fast to record.
        if (wasActive != isActive) {
            flightLog.write(
                "stick ${if (isActive) "engaged" else "released"} $side x=%.2f y=%.2f owned=$stickOwned".format(x, y),
            )
        }
        // A hand on the left stick is a vertical intent, and there can only be
        // one: the operator's touch always wins over the height loop.
        if (side == StickSide.LEFT && y != 0.0 && holdingHeight) {
            flightLog.write("descent cancelled by operator stick input")
            finishHeightHold("操作者接管升降，定高已取消", release = false)
        }
        virtualStick.setStick(side, x, y)
        if (leftStickActive || rightStickActive) {
            mainHandler.removeCallbacks(idleReleaseRunnable)
            if (!stickOwned) {
                render("搖桿輸入：取得控制權…")
                acquireControlLink { render("搖桿控制中") }
            }
        } else {
            // Both sticks centred: hand the aircraft back after a short grace
            // period, so a finger lifted between two inputs does not thrash the
            // control link.
            mainHandler.removeCallbacks(idleReleaseRunnable)
            mainHandler.postDelayed(idleReleaseRunnable, STICK_IDLE_RELEASE_MS)
        }
    }

    private val idleReleaseRunnable = Runnable {
        if (leftStickActive || rightStickActive || holdingHeight || !stickOwned) return@Runnable
        releaseControlLink { error ->
            render(error?.let { "釋放控制權失敗：$it" } ?: "搖桿放手，控制權已交回遙控器")
        }
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
        holdView.text = holdStatus
        holdView.visibility = if (holdStatus.isEmpty()) View.GONE else View.VISIBLE
        telemetryView.text = buildString {
            append("registered=").append(registered)
            append(" · aircraft=").append(if (aircraftConnected) "connected" else "disconnected")
            append(" · flying=").append(flying)
            append(" · height=").append(altitudeMeters?.let { "%.2f m".format(it) } ?: "—")
            append("(").append(heightSource.name).append(")")
            append(" · usonic=").append(ultrasonicRaw ?: "—")
            append("\nstick=").append(stickStatus.enabled)
            append(" advanced=").append(stickStatus.advancedMode)
            append(" authority=").append(stickStatus.authority)
            append(" · hold=").append(if (holdingHeight) "50cm" else "off")
            append(" · landingConfirmNeeded=").append(confirmationNeeded)
        }

        // The escape hatch only exists while it can do something: once registered
        // it would be a button that cannot help.
        registerButton.visibility = if (registered) View.GONE else View.VISIBLE
        registerButton.available = !registered
        takeoffButton.available = ready && !flying
        landButton.available = ready && flying
        holdButton.available = ready && flying && !holdingHeight
        // The pads are live whenever the aircraft is airborne: touching one is
        // what asks for the control link, so they must not be gated on owning it.
        leftPad.isEnabled = ready && flying
        rightPad.isEnabled = ready && flying
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
     * Flat pill action in the dashboard's accent palette.
     *
     * It stays clickable even when unavailable, and only its colours dim. A
     * disabled Android View swallows touches without a trace, which is how a
     * refused precondition became indistinguishable from a dead button: the
     * handler must always run so it can say why nothing happened.
     */
    private inner class PillButton(
        caption: String,
        private val accent: Int,
        private val onPressed: () -> Unit,
    ) : TextView(this@MainActivity) {

        var available: Boolean = false
            set(value) {
                if (value == field) return
                field = value
                applyAccent()
            }

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
            setOnClickListener { onPressed() }
            applyAccent()
        }

        private fun applyAccent() {
            val color = if (available) accent else StickPadView.MUTED
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

    /**
     * Asks the MSDK to register this app key. Registration needs the internet and
     * is happier once the aircraft is present, so every attempt is counted and
     * logged with its trigger — a silent single attempt is what made the app look
     * permanently stuck in "registering".
     */
    private fun requestRegistration(trigger: String) {
        if (registered) return
        mainHandler.removeCallbacks(registrationRetryRunnable)
        registerAttempts += 1
        flightLog.write("registerApp attempt=$registerAttempts trigger=$trigger")
        render("註冊 app key…（第 $registerAttempts 次，$trigger）")
        SDKManager.getInstance().registerApp()
    }

    private val registrationRetryRunnable = Runnable { requestRegistration("自動重試") }

    private fun startSdk() {
        SDKManager.getInstance().init(
            applicationContext,
            object : SDKManagerCallback {
                override fun onInitProcess(event: DJISDKInitEvent, totalProcess: Int) {
                    flightLog.write("init event=$event total=$totalProcess")
                    if (event == DJISDKInitEvent.INITIALIZE_COMPLETE) {
                        // registerApp is only legal after INITIALIZE_COMPLETE.
                        requestRegistration("初始化完成")
                    }
                }

                override fun onRegisterSuccess() {
                    registered = true
                    registerAttempts = 0
                    mainHandler.removeCallbacks(registrationRetryRunnable)
                    flightLog.write("registration succeeded")
                    render("app key 註冊成功，等待飛機連線…")
                    listenAircraftState()
                    virtualStick.start()
                    runOnUiThread { preview.refresh() }
                }

                /**
                 * Registration is retried instead of abandoned. Starting the app
                 * before the aircraft is powered on left it stuck reporting
                 * "registering" forever, because a single failed attempt was the
                 * end of it and only restarting the app tried again.
                 */
                override fun onRegisterFailure(error: IDJIError) {
                    registered = false
                    flightLog.write("registration failed attempt=$registerAttempts: $error")
                    val detail = error.description() ?: error.toString()
                    if (registerAttempts < MAX_REGISTER_ATTEMPTS) {
                        render("註冊失敗（第 $registerAttempts 次，將重試）：$detail")
                        mainHandler.postDelayed(registrationRetryRunnable, REGISTER_RETRY_MS)
                    } else {
                        render("註冊失敗 $registerAttempts 次：$detail（請按「重新註冊」）")
                    }
                }

                override fun onProductConnect(productId: Int) {
                    flightLog.write("product connected: $productId registered=$registered")
                    // The aircraft appearing is the event most likely to turn a
                    // failed registration into a successful one.
                    if (!registered) requestRegistration("飛機已連線")
                }

                override fun onProductDisconnect(productId: Int) {
                    flightLog.write("product disconnected: $productId")
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
            releaseIfNotFlying()
            render(if (aircraftConnected) "飛機已連線，可以起飛" else "飛機未連線")
        }
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyIsFlying),
            this,
            true,
        ) { _, isFlying ->
            flying = isFlying == true
            if (!flying) landingRequested = false
            releaseIfNotFlying()
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
        // Height sources, in the order this app trusts them. On the Mini 4 Pro
        // (2026-08-17) KeyAircraftLocation3D never published a value — it is GPS
        // derived, so indoors or before a fix it stays silent — and the height
        // loop then had no input at all. Every candidate is therefore listened to,
        // and the raw arrivals are logged so the working source is a measurement
        // and not an assumption.
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyAltitude),
            this,
            true,
        ) { _, meters ->
            // Height arrives at ~10 Hz per key; logging every sample floods the
            // log ring and evicted the incident evidence on 2026-08-17. Only the
            // hold loop logs heights now, and only while it is actually running.
            publishHeight(meters?.takeIf(Double::isFinite))
        }
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D),
            this,
            true,
        ) { _, location ->
            val height = location?.altitude?.takeIf(Double::isFinite)
            if (heightSource != HeightSource.ALTITUDE) publishHeight(height, HeightSource.LOCATION_3D)
        }
        // Unit of KeyUltrasonicHeight is not documented for this aircraft, so it
        // is observed only; it must never drive the loop before the numbers on a
        // known height confirm the scale.
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyUltrasonicHeight),
            this,
            true,
        ) { _, raw ->
            ultrasonicRaw = raw
        }
    }

    /** Which key the current height came from; a better source may replace a worse one. */
    private enum class HeightSource { NONE, LOCATION_3D, ALTITUDE }

    private fun publishHeight(meters: Double?, source: HeightSource = HeightSource.ALTITUDE) {
        if (meters == null) {
            if (source == heightSource) {
                altitudeMeters = null
                altitudeAtNanos = 0L
                heightSource = HeightSource.NONE
            }
            return
        }
        altitudeMeters = meters
        altitudeAtNanos = System.nanoTime()
        heightSource = source
        driveHeightHold()
    }

    // -------------------------------------------------------- actuation ----

    private fun takeoff() {
        flightLog.write("press: takeoff (registered=$registered connected=$aircraftConnected flying=$flying)")
        if (!registered || !aircraftConnected) {
            render("尚未註冊或飛機未連線，無法起飛")
            return
        }
        if (flying) {
            render("已在空中，忽略起飛")
            return
        }
        render("送出起飛指令…")
        performAction(KeyTools.createKey(FlightControllerKey.KeyStartTakeoff), "起飛")
    }

    /**
     * Landing has two preconditions this app must satisfy itself, both learned on
     * hardware (2026-08-17):
     *
     * 1. The control link must be released **and the aircraft must have named the
     *    RC as authority owner again**. A landing command sent inside the ~140 ms
     *    handover window is answered "accepted" and then silently cancelled by the
     *    authority change, leaving the flight controller latched: every later
     *    command is also "accepted" while the aircraft never moves.
     * 2. Acceptance is not execution. The descent is verified against the height,
     *    and a landing that is not descending is cleared with KeyStopAutoLanding
     *    and re-issued.
     */
    private fun land() {
        flightLog.write("press: land (connected=$aircraftConnected flying=$flying owned=$stickOwned hold=$holdingHeight authority=${stickStatus.authority})")
        if (!aircraftConnected) {
            render("飛機未連線，無法降落")
            return
        }
        holdingHeight = false
        holdStatus = ""
        if (!stickOwned) {
            awaitRemoteAuthority { sendLandingCommand() }
            return
        }
        render("先交回控制權，再送降落…")
        releaseControlLink { error ->
            if (error != null) {
                render("釋放控制權失敗，請用遙控器降落：$error")
            } else {
                awaitRemoteAuthority { sendLandingCommand() }
            }
        }
    }

    /** Polls until the aircraft reports the RC as authority owner, then proceeds. */
    private fun awaitRemoteAuthority(onRemote: () -> Unit) {
        val deadline = System.nanoTime() + AUTHORITY_HANDOVER_TIMEOUT_MS * 1_000_000L
        fun poll() {
            if (stickStatus.authority != VirtualStickSession.MSDK_AUTHORITY_OWNER) {
                onRemote()
                return
            }
            if (System.nanoTime() >= deadline) {
                flightLog.write("landing proceeding without RC authority (owner=${stickStatus.authority})")
                onRemote()
                return
            }
            render("等待控制權交回遙控器…")
            mainHandler.postDelayed(::poll, AUTHORITY_POLL_MS)
        }
        poll()
    }

    private fun sendLandingCommand(attempt: Int = 1) {
        landingRequested = true
        confirmAttempts = 0
        landingHeightAtCommand = altitudeMeters
        flightLog.write("landing command attempt=$attempt h=$landingHeightAtCommand authority=${stickStatus.authority}")
        render("送出降落指令…")
        performAction(KeyTools.createKey(FlightControllerKey.KeyStartAutoLanding), "降落")
        // Landing protection may already be waiting when the descent starts.
        if (confirmationNeeded) confirmLanding()
        mainHandler.postDelayed({ verifyLandingProgress(attempt) }, LANDING_VERIFY_MS)
    }

    private fun verifyLandingProgress(attempt: Int) {
        if (!landingRequested || !flying) return
        val before = landingHeightAtCommand
        val now = altitudeMeters
        val descended = before != null && now != null && before - now >= LANDING_MIN_DESCENT_METERS
        if (descended || confirmationNeeded) {
            flightLog.write("landing verified: before=$before now=$now confirmNeeded=$confirmationNeeded")
            return
        }
        flightLog.write("landing not descending: before=$before now=$now attempt=$attempt")
        if (attempt >= MAX_LANDING_ATTEMPTS) {
            render("降落指令被接受但飛機未下降，請用遙控器降落")
            return
        }
        // Clear the latched request before re-issuing; a stale "landing requested"
        // state is what makes the next command another silent no-op.
        render("降落沒有生效，重新送出…")
        performAction(KeyTools.createKey(FlightControllerKey.KeyStopAutoLanding), "取消降落")
        mainHandler.postDelayed({ sendLandingCommand(attempt + 1) }, LANDING_RETRY_MS)
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
                    flightLog.write("landing confirmation accepted")
                    render("降落確認已被飛機接受，正在觸地…")
                }

                override fun onFailure(error: IDJIError) {
                    flightLog.write("landing confirmation refused: $error")
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
                    flightLog.write("$label accepted")
                    render("$label 指令已被飛機接受")
                }

                override fun onFailure(error: IDJIError) {
                    // A refusal is the normal outcome of an unmet precondition
                    // (props off, no GPS, RC in the wrong mode). Show it verbatim.
                    flightLog.write("$label refused: $error")
                    render("$label 被拒絕：${error.description() ?: error}")
                }
            },
        )
    }

    // ----------------------------------------------------- virtual stick ----

    /**
     * Control-link policy after the 2026-08-17 incident: **the RC owns the
     * aircraft unless the operator explicitly hands control to this app.**
     *
     * The app used to grab the link the moment `KeyIsFlying` turned true. That
     * left an aircraft in the air whose landing command was refused (landing is
     * illegal while virtual stick holds the link) and whose RC could not win it
     * back, because the 20 Hz frame stream immediately re-took authority. Both
     * failures came from the same decision — taking control without being asked.
     *
     * So acquisition happens only from an operator action ([acquireControlLink]),
     * release happens automatically whenever the reason to hold it disappears,
     * and the aircraft's own authority report can stop the frames at any time.
     */
    private fun releaseIfNotFlying() {
        if (!stickOwned || stickTransitionPending) return
        if (registered && aircraftConnected && flying) return
        releaseControlLink { error ->
            render(error?.let { "釋放控制權失敗：$it" } ?: "已釋放控制權，交回遙控器")
        }
    }

    private fun releaseControlLink(onDone: (String?) -> Unit) {
        if (stickTransitionPending) {
            onDone("控制權切換進行中")
            return
        }
        beginTransition("release")
        holdingHeight = false
        holdStatus = ""
        virtualStick.disable { error ->
            endTransition()
            stickOwned = false
            flightLog.write("release done error=$error")
            onDone(error)
        }
    }

    /**
     * Acquisition is retried because `KeyIsFlying` turns true while the aircraft
     * is still climbing on its own auto-takeoff, and in that window it refuses
     * virtual stick with CONTROL_AUTH_TAKING_OFF (observed on the Mini 4 Pro,
     * 2026-08-17). A single attempt loses the link for the whole flight.
     */
    private fun acquireControlLink(onOwned: () -> Unit) {
        if (stickOwned) {
            onOwned()
            return
        }
        if (!registered || !aircraftConnected || !flying) {
            flightLog.write("acquire refused: registered=$registered connected=$aircraftConnected flying=$flying")
            render("飛機未在空中，不取得控制權")
            return
        }
        if (stickTransitionPending) {
            // Never silent: a swallowed press is indistinguishable from a dead
            // button, which is exactly how this looked to the operator.
            flightLog.write("acquire ignored: transition already pending")
            render("控制權切換進行中，請稍候再試")
            return
        }
        beginTransition("acquire")
        landingRequested = false
        acquireAttempts += 1
        val attempt = acquireAttempts
        virtualStick.enable { error ->
            endTransition()
            stickOwned = error == null
            flightLog.write("acquire attempt=$attempt owned=$stickOwned error=$error")
            when {
                stickOwned -> onOwned()
                attempt < MAX_ACQUIRE_ATTEMPTS && flying -> {
                    render("等待起飛完成才能取得控制權（第 $attempt 次）：$error")
                    mainHandler.postDelayed({ acquireControlLink(onOwned) }, ACQUIRE_RETRY_MS)
                }
                else -> render("取得控制權失敗 $attempt 次，請用遙控器操作：$error")
            }
        }
    }

    /**
     * A control-link call that never calls back would latch [stickTransitionPending]
     * forever, and every later button press would be dropped — the app would look
     * dead while takeoff and landing still worked, because those do not need the
     * link. The latch therefore has a deadline.
     */
    private fun beginTransition(what: String) {
        stickTransitionPending = true
        flightLog.write("transition begin: $what")
        mainHandler.postDelayed(transitionTimeoutRunnable, TRANSITION_TIMEOUT_MS)
    }

    private fun endTransition() {
        stickTransitionPending = false
        mainHandler.removeCallbacks(transitionTimeoutRunnable)
    }

    private val transitionTimeoutRunnable = Runnable {
        if (!stickTransitionPending) return@Runnable
        stickTransitionPending = false
        flightLog.write("transition timed out; latch cleared")
        render("控制權切換無回應，已解除鎖定，可再試一次")
    }

    /**
     * The one operator action that hands control to this app: pressing it takes
     * the control link (retrying through the auto-takeoff window) and only then
     * closes the height loop. Nothing else in the app takes control on its own.
     */
    private fun startHeightHold() {
        flightLog.write(
            "press: hold (flying=$flying owned=$stickOwned pending=$stickTransitionPending " +
                "height=$altitudeMeters src=$heightSource ageMs=${heightAgeMillis()})",
        )
        if (!flying) {
            render("飛機不在空中，無法定高")
            return
        }
        if (usableHeightMeters() == null) {
            flightLog.write("descent refused: no usable height last=$altitudeMeters src=$heightSource connected=$aircraftConnected ageMs=${heightAgeMillis()}")
            render("沒有高度資料，無法定高")
            return
        }
        render("取得控制權後開始下降…")
        acquireControlLink {
            if (usableHeightMeters() == null) {
                finishHeightHold("沒有高度資料，已放棄定高並交回遙控器")
                return@acquireControlLink
            }
            holdingHeight = true
            holdStartedAtNanos = System.nanoTime()
            holdStableSamples = 0
            holdAuthoritySeen = false
            flightLog.write("descent armed target=$TARGET_HEIGHT_METERS")
            mainHandler.post(holdTickRunnable)
        }
    }

    /**
     * Ends the manoeuvre and, by default, hands the aircraft back. Holding the
     * control link after the descent finishes has no purpose: with the sticks
     * neutral the aircraft keeps the height on its own, and an idle link is
     * exactly what stopped the RC from taking over on 2026-08-17.
     */
    private fun finishHeightHold(message: String, release: Boolean = true) {
        holdingHeight = false
        holdStableSamples = 0
        holdAuthoritySeen = false
        holdStatus = message
        mainHandler.removeCallbacks(holdTickRunnable)
        virtualStick.setClimbRate(0.0)
        if (!release || !stickOwned) {
            render(message)
            return
        }
        releaseControlLink { error ->
            holdStatus = if (error == null) message else "$message（釋放控制權失敗：$error）"
            render(holdStatus)
        }
    }

    /**
     * DJI publishes heights on change and only in 0.1 m steps, so a still hover
     * legitimately produces no sample for tens of seconds — the log shows a 23 s
     * gap at a steady 1.0 m. Sample age therefore cannot decide validity: a live
     * aircraft link plus one sample from this flight can.
     *
     * Age still matters once the aircraft is *supposed* to be moving; that check
     * lives in [driveHeightHold], where a command is actually being issued.
     */
    private fun usableHeightMeters(): Double? {
        if (!aircraftConnected) return null
        if (altitudeAtNanos == 0L) return null
        return altitudeMeters
    }

    /** Age of the last height sample in milliseconds; -1 when none was ever seen. */
    private fun heightAgeMillis(): Long =
        if (altitudeAtNanos == 0L) -1L else (System.nanoTime() - altitudeAtNanos) / 1_000_000L

    /**
     * The loop runs on its own 10 Hz tick instead of only on arriving samples: a
     * hovering aircraft sends no samples, and a manoeuvre that only reacts to
     * samples would never start moving.
     */
    private val holdTickRunnable = object : Runnable {
        override fun run() {
            if (!holdingHeight) return
            driveHeightHold()
            if (holdingHeight) mainHandler.postDelayed(this, HOLD_TICK_MS)
        }
    }

    /**
     * Proportional height loop, run on every height sample rather than on a timer
     * so a command is only ever derived from a height that actually arrived.
     *
     * Stale or missing height means no vertical intent can be justified, so the
     * loop commands zero climb — the aircraft then holds its own height, which is
     * the safe outcome, instead of coasting on the last error.
     */
    private fun driveHeightHold() {
        if (!holdingHeight || !stickOwned) return
        // Two different things look identical in one sample: the aircraft has not
        // finished handing authority over yet, and the RC has taken it away. They
        // are told apart by history — a takeover can only happen after this app
        // has actually held authority. enableVirtualStick returns ~60 ms before the
        // aircraft names MSDK as the owner, and treating that gap as a takeover is
        // what made the descent stop 7 ms after it armed (2026-08-17).
        val ownsAuthority = stickStatus.authority == VirtualStickSession.MSDK_AUTHORITY_OWNER
        if (ownsAuthority) {
            holdAuthoritySeen = true
        } else if (holdAuthoritySeen) {
            flightLog.write("descent stopped: authority taken by ${stickStatus.authority}")
            finishHeightHold("遙控器已接管，定高停止", release = false)
            return
        } else {
            virtualStick.setClimbRate(0.0)
            val waitingMs = (System.nanoTime() - holdStartedAtNanos) / 1_000_000L
            if (waitingMs > AUTHORITY_HANDOVER_TIMEOUT_MS) {
                flightLog.write("descent aborted: authority never arrived (owner=${stickStatus.authority})")
                finishHeightHold("未取得控制權（owner=${stickStatus.authority}），已放棄定高")
            } else {
                holdStatus = "等待控制權移交…（owner=${stickStatus.authority}）"
                render(holdStatus)
            }
            return
        }
        val height = usableHeightMeters()
        if (height == null) {
            flightLog.write("descent aborted: height unusable ageMs=${heightAgeMillis()}")
            finishHeightHold("高度資料中斷，已停止下降並交回遙控器")
            return
        }
        if (System.nanoTime() - holdStartedAtNanos > HOLD_TIMEOUT_NANOS) {
            flightLog.write("descent aborted: timeout h=%.2f".format(height))
            finishHeightHold("下降逾時（目前 %.2f m），已交回遙控器".format(height))
            return
        }
        val error = TARGET_HEIGHT_METERS - height
        if (abs(error) <= HEIGHT_TOLERANCE_METERS) {
            virtualStick.setClimbRate(0.0)
            holdStableSamples += 1
            // A few consecutive in-band samples, not one, decide arrival: a single
            // sample can be noise on the way past the target.
            if (holdStableSamples >= HOLD_STABLE_SAMPLES) {
                flightLog.write("descent complete h=%.2f; releasing".format(height))
                finishHeightHold("已到 %.2f m，控制權交回遙控器".format(height))
            } else {
                holdStatus = "定高 0.5 m：確認中（目前 %.2f m）".format(height)
                render(holdStatus)
            }
            return
        }
        holdStableSamples = 0
        // Commanding a descent while the height has stopped updating is flying
        // blind: the aircraft should be moving, so samples should be arriving. The
        // check waits out a grace window, because a hover legitimately starts with
        // an old sample and the first frames need time to produce motion.
        val descendingForMs = (System.nanoTime() - holdStartedAtNanos) / 1_000_000L
        if (descendingForMs > MAX_MOVING_HEIGHT_AGE_MS && heightAgeMillis() > MAX_MOVING_HEIGHT_AGE_MS) {
            flightLog.write(
                "descent aborted: height not updating ageMs=${heightAgeMillis()} h=%.2f".format(height),
            )
            finishHeightHold("高度停止更新，已停止下降並交回遙控器")
            return
        }
        val climbRate = (HEIGHT_GAIN * error)
            .coerceIn(-VirtualStickSession.MAX_VERTICAL_MPS, VirtualStickSession.MAX_VERTICAL_MPS)
        virtualStick.setClimbRate(climbRate)
        flightLog.write("descent h=%.2f e=%+.2f cmd=%+.2f age=${heightAgeMillis()}".format(height, error, climbRate))
        holdStatus = "下降至 0.5 m：目前 %.2f m，垂直 %+.2f m/s".format(height, climbRate)
        render(holdStatus)
    }

    private companion object {
        const val TAG = "LiteMainActivity"

        /** Pill fill: the accent colour at ~12% opacity, matching the dashboard panels. */
        const val PILL_FILL_ALPHA = 0x1F000000

        const val MAX_CONFIRM_ATTEMPTS = 8
        const val CONFIRM_RETRY_MS = 700L

        /** Hover height for the bench trial, in metres above the takeoff point. */
        const val TARGET_HEIGHT_METERS = 0.5

        /**
         * Arrival band. KeyAltitude is quantised to 0.1 m, so a tighter band than
         * one quantisation step could only be satisfied by an exact 0.50 reading.
         */
        const val HEIGHT_TOLERANCE_METERS = 0.1

        /**
         * Proportional gain in (m/s) per metre of error. 0.6 with the session's
         * 0.3 m/s clamp means full descent rate above 0.5 m of error and a gentle
         * approach inside it — deliberately slow, because overshoot at 0.5 m is
         * ground contact.
         */
        const val HEIGHT_GAIN = 0.6

        /**
         * While a descent is commanded, the height must keep arriving: the aircraft
         * is moving, so samples must move too. Silence longer than this means the
         * loop is blind and must stop instead of guessing.
         */
        const val MAX_MOVING_HEIGHT_AGE_MS = 2_000L

        /** Height-loop period; 10 Hz is half the frame rate the aircraft is fed. */
        const val HOLD_TICK_MS = 100L

        /** Consecutive in-band samples that count as "arrived" (~0.3 s at 10 Hz). */
        const val HOLD_STABLE_SAMPLES = 3

        /** A descent that has not converged by now is abandoned, not prolonged. */
        const val HOLD_TIMEOUT_NANOS = 20_000_000_000L

        /** Grace period before centred sticks hand the aircraft back to the RC. */
        const val STICK_IDLE_RELEASE_MS = 3_000L

        /**
         * Auto-takeoff climbs for a couple of seconds and refuses virtual stick
         * until it finishes, so acquisition retries for ~12 s before giving up.
         */
        const val MAX_ACQUIRE_ATTEMPTS = 24
        const val ACQUIRE_RETRY_MS = 500L

        /**
         * How long the aircraft may take to name MSDK as authority owner after
         * enableVirtualStick succeeds. Measured at ~60 ms on the Mini 4 Pro; the
         * bound is generous because waiting costs nothing but a zero command.
         */
        const val AUTHORITY_HANDOVER_TIMEOUT_MS = 1_500L

        /** Registration retries before the operator has to press the button. */
        const val MAX_REGISTER_ATTEMPTS = 10
        const val REGISTER_RETRY_MS = 3_000L

        /** How often the authority owner is polled while waiting for a handover. */
        const val AUTHORITY_POLL_MS = 50L

        /** Time given to the aircraft to visibly start descending after a landing command. */
        const val LANDING_VERIFY_MS = 3_000L

        /** Descent that counts as "the landing is happening" (one KeyAltitude step). */
        const val LANDING_MIN_DESCENT_METERS = 0.1

        /** Landing command re-issues before the operator is told to use the RC. */
        const val MAX_LANDING_ATTEMPTS = 3
        const val LANDING_RETRY_MS = 700L

        /** Deadline for a control-link call to answer before the latch is cleared. */
        const val TRANSITION_TIMEOUT_MS = 4_000L
    }
}
