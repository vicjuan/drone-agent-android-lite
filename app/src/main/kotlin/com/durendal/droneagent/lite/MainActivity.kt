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
import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.FlightAssistantKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.Attitude
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.common.register.DJISDKInitEvent
import dji.v5.manager.KeyManager
import dji.v5.manager.SDKManager
import dji.v5.manager.aircraft.perception.PerceptionManager
import dji.v5.manager.aircraft.perception.listener.ObstacleDataListener
import dji.v5.manager.interfaces.SDKManagerCallback
import java.io.File
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
    private lateinit var tapeOverlay: TapeOverlayView
    private var tapeDetector: BlackTapeDetector? = null
    private var tapeDetected = false
    private var tapeLoggedAtNanos = 0L
    private var consecutiveTapeMisses = 0
    private lateinit var flightLog: FlightLog
    private lateinit var headlineView: TextView
    private lateinit var detailView: TextView
    private lateinit var holdView: TextView
    private lateinit var telemetryView: TextView
    private lateinit var takeoffButton: PillButton
    private lateinit var landButton: PillButton
    private lateinit var holdButton: PillButton
    private lateinit var oneMeterHoldButton: PillButton
    private lateinit var registerButton: PillButton
    private lateinit var cameraDownButton: PillButton
    private lateinit var curvedOutAndBackTrackingButton: PillButton
    private lateinit var captureButton: PillButton
    private lateinit var tapeTrackingButton: PillButton
    private lateinit var circularTapeTrackingButton: PillButton
    private lateinit var quarterArcButton: PillButton
    private lateinit var leftPad: StickPadView
    private lateinit var rightPad: StickPadView
    private lateinit var gimbalPad: StickPadView

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
    private var transitionTimeoutAction: (() -> Unit)? = null
    private var transitionGeneration = 0L

    /** Ground-relative height and when it was observed; null until the first sample. */
    private var altitudeMeters: Double? = null
    private var altitudeAtNanos = 0L
    private var heightSource = HeightSource.NONE

    private var obstacleLoggedAtNanos = 0L
    private var obstacleTelemetryRenderedAtNanos = 0L

    /** Aircraft avoidance mode; BRAKE normally, temporarily CLOSE during tape tracking. */
    private val avoidanceCheck = AvoidanceCheck()
    private var avoidance = AvoidanceCheck.Status()

    /** True while the aircraft reports it is braking for an obstacle. */
    private var activelyAvoiding = false

    /** Latest 360-degree horizontal obstacle range sample, in documented mm. */
    private var nearestHorizontalObstacleMm: Int? = null
    private var horizontalRawMinMm: Int? = null
    private var horizontalRawMaxMm: Int? = null
    private var horizontalDetectedCount = 0
    private var horizontalGroundEchoCount = 0
    private var horizontalGroundEchoSuppressed = false
    private var horizontalSampleCount = 0
    private var horizontalAngleIntervalDegrees = 0
    private var upwardObstacleMm: Int? = null
    private var downwardObstacleMm: Int? = null
    private var obstacleSampleReceived = false
    private var obstacleSampleAtNanos = 0L
    private var obstacleSampleValid = false
    private var horizontalSafetyReady = false
    private var obstacleDataListener: ObstacleDataListener? = null



    /** Battery percentage and one-shot forced-landing policy. */
    private var batteryPercent: Int? = null
    private var lowCellVoltage = false
    private val batteryLandingGate = BatteryLandingGate()

    /** Latest aircraft yaw and one operator-requested directed half-turn. */
    private var aircraftHeadingDegrees: Double? = null
    private var aircraftHeadingAtNanos = 0L
    private var headingTurn: HeadingTurn? = null
    private var turnStartedAtNanos = 0L
    private var turnCommandStartedAtNanos = 0L
    private var turnAuthoritySeen = false

    /** Camera-independent 0.75 m radius clockwise quarter-arc experiment. */
    private var quarterArcController: QuarterArcController? = null
    private var quarterArcStartedAtNanos = 0L
    private var quarterArcAuthoritySeen = false
    private var quarterArcLastRenderedAtNanos = 0L

    /** Camera gimbal is independent of the aircraft's virtual-stick authority. */
    private var gimbalActive = false
    private var selectedCameraPitchDegrees: Double? = null
    private var captureRecorder: TapeCaptureRecorder? = null
    private var cameraPitchCommandPending = false
    private var cameraPitchCommandGeneration = 0L
    private val cameraPitchCommandTimeoutRunnable = Runnable {
        if (!cameraPitchCommandPending) return@Runnable
        cameraPitchCommandGeneration += 1
        cameraPitchCommandPending = false
        selectedCameraPitchDegrees = null
        flightLog.write("camera pitch command timed out")
        render("鏡頭角度控制逾時，已解除鎖定，可再試一次")
    }
    private val tapeTracking = TapeTrackingController()
    private var tapeTrackingAuthoritySeen = false
    private var tapeTrackingStartedAtNanos = 0L
    private var tapeTrackingStartPending = false
    @Volatile private var activityDestroying = false
    private var renderedTapeTrackingPhase = TapeTrackingPhase.DISABLED
    private var commandedTapeYawRate = 0.0
    private var commandedTapeForwardSpeed = 0.0
    private var commandedTapeRightSpeed = 0.0
    private var tapeEndpointTurn = false
    private var tapeCommandLoggedAtNanos = 0L
    private var activeTapeTrackingMode: TapeTrackingMode? = null



    /** Raw KeyUltrasonicHeight, logged for unit identification; never drives the loop yet. */
    private var ultrasonicRaw: Int? = null

    /** Target for the active one-shot height manoeuvre; null while inactive. */
    private var heightTargetMeters: Double? = null
    private val holdingHeight: Boolean get() = heightTargetMeters != null
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
        onStatus = ::handleVirtualStickStatus,
        onFrameSummary = { summary -> flightLog.write(summary) },
    )

    private fun handleVirtualStickStatus(status: VirtualStickStatus) = runOnUiThread {
        val previous = stickStatus
        stickStatus = status
        if (previous.enabled != status.enabled || previous.authority != status.authority) {
            flightLog.write("stick state enabled=${status.enabled} authority=${status.authority}")
        }
        if (tapeTracking.enabled) {
            if (status.authority == VirtualStickSession.MSDK_AUTHORITY_OWNER) {
                tapeTrackingAuthoritySeen = true
            } else if (tapeTrackingAuthoritySeen) {
                stopTapeTracking("實體遙控器已接管，黑膠帶追蹤已停止", release = false)
            }
        }
        if (quarterArcController != null) {
            if (status.authority == VirtualStickSession.MSDK_AUTHORITY_OWNER) {
                quarterArcAuthoritySeen = true
                driveQuarterArc()
            } else if (quarterArcAuthoritySeen) {
                finishQuarterArc("實體遙控器已接管，無視覺 1/4 圈已停止", release = false)
            }
        }
        render("控制權=${status.authority}")
    }
    private val gimbal = GimbalSession(
        onFailure = { error ->
            runOnUiThread {
                flightLog.write("gimbal command refused: $error")
                if (tapeTracking.enabled) {
                    stopTapeTracking("雲台控制失敗，黑膠帶追蹤已停止：$error", release = true)
                } else {
                    render("攝影機角度控制被拒絕：$error")
                }
            }
        },
    )

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        flightLog = FlightLog(this)
        captureRecorder = runCatching {
            TapeCaptureRecorder(
                root = File(getExternalFilesDir(null), TAPE_CAPTURE_DIRECTORY),
                log = flightLog::write,
            )
        }.onFailure { error ->
            flightLog.write("tape capture unavailable: $error")
        }.getOrNull()
        tapeDetector = runCatching {
            BlackTapeDetector(
                onResult = ::handleTapeDetection,
                onError = { error ->
                    runOnUiThread { flightLog.write("OpenCV detection failed: $error") }
                },
                captureRecorder = captureRecorder,
                captureFlightContext = ::tapeCaptureFlightContext,
            )
        }.onFailure { error ->
            flightLog.write("OpenCV initialization failed: $error")
        }.getOrNull()
        setContentView(buildUi())
        flightLog.write("app start; log=${flightLog.path}")
        render("啟動 MSDK…")
        startSdk()
        mainHandler.post(horizontalSafetyWatchdog)
    }

    override fun onDestroy() {
        // Listeners are held by `this`; leaking them across an Activity restart
        // would deliver key updates to a dead view hierarchy. The stick link is
        // released too, so a killed UI can never leave a live control link.
        activityDestroying = true
        mainHandler.removeCallbacksAndMessages(null)
        tapeTracking.stop()
        tapeTrackingStartPending = false
        captureRecorder?.disarm()
        if (avoidance.closedConfirmed) avoidanceCheck.ensureBrake {}
        if (stickOwned) virtualStick.disable {}
        virtualStick.close()
        gimbal.close()
        preview.release()
        tapeDetector?.close()
        tapeDetector = null
        obstacleDataListener?.let { listener ->
            runCatching { PerceptionManager.getInstance().removeObstacleDataListener(listener) }
        }
        obstacleDataListener = null
        runCatching { KeyManager.getInstance().cancelListen(this) }
        flightLog.write("app destroy")
        flightLog.close()
        super.onDestroy()
    }

    // ---------------------------------------------------------------- UI ----

    private fun buildUi(): ViewGroup {
        val root = FrameLayout(this).apply { setBackgroundColor(Color.BLACK) }

        preview = CameraPreview(
            context = this,
            onRgbaFrame = { frameData, offset, length, width, height ->
                tapeDetector?.submitRgba(frameData, offset, length, width, height)
            },
            onFrameStreamStale = ::handleTapeFrameStreamStale,
        )
        root.addView(
            preview.view,
            FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.MATCH_PARENT,
            ),
        )
        tapeOverlay = TapeOverlayView(this)
        root.addView(
            tapeOverlay,
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

    private fun handleTapeDetection(detection: TapeDetection?) = runOnUiThread {
        if (tapeEndpointTurn) return@runOnUiThread
        val now = System.nanoTime()
        if (tapeTracking.enabled) {
            tapeTracking.observe(
                detection?.let {
                    TapeTrackingObservation(
                        angleFromVerticalDegrees = it.angleFromVerticalDegrees,
                        longSideFraction = it.longSideFraction,
                        nearFieldOffsetFraction = it.nearFieldOffsetFraction,
                        bounds = it.bounds,
                        lookahead = it.lookahead,
                        quality = it.quality,
                        endpointCandidate = it.endpointCandidate,
                        closedLoop = it.closedLoop,
                        frameWidthPixels = it.sourceWidth,
                        frameHeightPixels = it.sourceHeight,
                        heightAboveGroundMeters = usableHeightMeters()?.takeIf { height ->
                            height.isFinite() && height > 0.0
                        },
                    )
                },
                now,
            )
        }
        if (detection == null) {
            consecutiveTapeMisses += 1
            if (consecutiveTapeMisses < TAPE_MISSES_TO_CLEAR) return@runOnUiThread
        } else {
            consecutiveTapeMisses = 0
        }
        tapeOverlay.showDetection(detection)
        // The overlay draws the same centerline the controller steers by, so the
        // two can never show different paths for one frame.
        tapeOverlay.showCenterline(detection?.centerline)
        val detected = detection != null
        if (detected != tapeDetected || now - tapeLoggedAtNanos >= TAPE_LOG_PERIOD_NANOS) {
            tapeDetected = detected
            tapeLoggedAtNanos = now
            val frameDiagnostics =
                "${preview.diagnosticsSummary()} ${tapeDetector?.diagnosticsSummary().orEmpty()}"
            flightLog.write(
                detection?.let {
                    "black tape detected confidence=%.2f angle=%+.1f anchor=(%.3f,%.3f) lookahead=(%.3f,%.3f) boxOffset=%+.3f length=%.3f bounds=%s %s".format(
                        it.confidence,
                        it.angleFromVerticalDegrees,
                        it.anchorXFraction,
                        it.anchorYFraction,
                        it.lookahead?.xFraction,
                        it.lookahead?.yFraction,
                        it.bounds.centerX - 0.5,
                        it.longSideFraction,
                        it.bounds,
                        frameDiagnostics,
                    )
                } ?: "black tape not detected $frameDiagnostics",
            )
        }
    }

    private fun handleTapeFrameStreamStale() {
        tapeDetector?.resetTracking()
        tapeTracking.observe(null, System.nanoTime())
        if (tapeTracking.enabled) {
            commandedTapeYawRate = 0.0
            commandedTapeForwardSpeed = 0.0
            commandedTapeRightSpeed = 0.0
            virtualStick.setYawRate(0.0)
            virtualStick.setForwardOnly(0.0)
        }
        flightLog.write("RGBA frame stream stale; detector reset")
        runOnUiThread {
            if (!::tapeOverlay.isInitialized) return@runOnUiThread
            consecutiveTapeMisses = 0
            tapeDetected = false
            tapeOverlay.showDetection(null)
            tapeOverlay.showCenterline(null)
        }
    }

    /**
     * One row: takeoff on the left, status in the middle, landing on the right.
     * The panel takes the leftover width, so a long status line can never slide
     * under either action, and the two actions stay physically apart — reaching
     * for one can never graze the other.
     */
    private fun buildTopBar(): ViewGroup = LinearLayout(this).apply {
        orientation = LinearLayout.VERTICAL
        addView(buildPrimaryActionRow())
        addView(buildExperimentActionRow())
    }

    private fun buildPrimaryActionRow(): ViewGroup = LinearLayout(this).apply {
        orientation = LinearLayout.HORIZONTAL
        takeoffButton = PillButton("起飛並停留", StickPadView.GREEN) { takeoff() }
        landButton = PillButton("降落", StickPadView.AMBER) { land() }
        holdButton = PillButton(
            "定高 %.0f 公分".format(LOW_HEIGHT_TARGET_METERS * 100.0),
            StickPadView.CYAN,
        ) { startHeightHold(LOW_HEIGHT_TARGET_METERS) }
        oneMeterHoldButton = PillButton("離地 1 公尺", StickPadView.CYAN) {
            startHeightHold(ONE_METER_TARGET_HEIGHT_METERS)
        }
        cameraDownButton = PillButton("鏡頭 -90°", StickPadView.CYAN) {
            moveCameraToPitch(CAMERA_DOWN_PITCH_DEGREES)
        }
        tapeTrackingButton =
            PillButton("直線黑膠帶追蹤", StickPadView.GREEN) { toggleTapeTracking() }
        registerButton = PillButton("重新註冊", StickPadView.AMBER) {
            registerAttempts = 0
            requestRegistration("操作者手動")
        }
        addView(registerButton, actionParams(marginEnd = dp(10)))
        addView(takeoffButton, actionParams(marginEnd = dp(10)))
        addView(holdButton, actionParams(marginEnd = dp(10)))
        addView(oneMeterHoldButton, actionParams(marginEnd = dp(10)))
        addView(cameraDownButton, actionParams(marginEnd = dp(10)))
        addView(tapeTrackingButton, actionParams(marginEnd = dp(10)))
        addView(
            buildStatusPanel(),
            LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f),
        )
        addView(landButton, actionParams(marginStart = dp(10)))
    }

    private fun buildExperimentActionRow(): ViewGroup = LinearLayout(this).apply {
        orientation = LinearLayout.VERTICAL
        gravity = Gravity.START
        circularTapeTrackingButton =
            PillButton("圓形黑膠帶追蹤", StickPadView.CYAN) { toggleCircularTapeTracking() }
        curvedOutAndBackTrackingButton =
            PillButton("弧形往返追蹤", StickPadView.GREEN) { toggleCurvedOutAndBackTracking() }
        quarterArcButton =
            PillButton("無視覺右轉 1/4 圈", StickPadView.AMBER) { toggleQuarterArc() }
        captureButton = PillButton("錄製影格證據", StickPadView.AMBER) { toggleFrameCapture() }
        addView(
            LinearLayout(this@MainActivity).apply {
                orientation = LinearLayout.HORIZONTAL
                addView(circularTapeTrackingButton, actionParams(marginEnd = dp(10)))
                addView(curvedOutAndBackTrackingButton, actionParams(marginEnd = dp(10)))
                addView(captureButton, actionParams())
            },
        )
        addView(
            quarterArcButton,
            actionParams(marginStart = dp(190), marginTop = dp(16)),
        )
    }

    /**
     * Arms or disarms replayable frame capture. Only the operator turns this on:
     * it retains several full-resolution frames and writes them to storage, and
     * the evidence is only worth keeping around a run the operator is watching.
     */
    private fun toggleFrameCapture() {
        val recorder = captureRecorder
        if (recorder == null) {
            render("影格證據儲存無法使用")
            return
        }
        if (recorder.isArmed) {
            recorder.disarm()
            captureButton.text = "錄製影格證據"
            // Queued is not saved: report what reached storage, and say so plainly
            // when the writer could not keep up, rather than implying evidence
            // exists that was dropped.
            val dropped = recorder.droppedCaptureCount + recorder.failedCaptureCount
            render(
                if (dropped == 0) {
                    "影格證據已停止，已保存 ${recorder.savedCaptureCount} 張"
                } else {
                    "影格證據已停止，已保存 ${recorder.savedCaptureCount} 張，" +
                        "另有 $dropped 張未能保存（寫入跟不上）"
                },
            )
        } else {
            recorder.arm()
            captureButton.text = "停止錄製影格"
            render("影格證據錄製中：路徑消失時自動保存前後影格")
        }
    }

    /**
     * Flight state a capture cannot reconstruct from the frame itself. Camera
     * pitch is the commanded-and-accepted value, so a capture taken while a
     * pitch command is in flight reports it as unknown rather than guessing.
     */
    private fun tapeCaptureFlightContext(): Map<String, String> = linkedMapOf(
        "gimbal.pitchDegrees" to (selectedCameraPitchDegrees?.toString() ?: "unknown"),
        "aircraft.heightMeters" to (usableHeightMeters()?.toString() ?: "unknown"),
        "aircraft.heightSource" to heightSource.name,
        "aircraft.headingDegrees" to (aircraftHeadingDegrees?.toString() ?: "unknown"),
        "aircraft.flying" to flying.toString(),
        "tracking.mode" to (activeTapeTrackingMode?.name ?: "off"),
        "tracking.phase" to renderedTapeTrackingPhase.name,
    )

    private fun actionParams(
        marginStart: Int = 0,
        marginEnd: Int = 0,
        marginTop: Int = 0,
    ) = LinearLayout.LayoutParams(
        ViewGroup.LayoutParams.WRAP_CONTENT,
        ViewGroup.LayoutParams.WRAP_CONTENT,
    ).apply {
        gravity = Gravity.TOP
        this.marginStart = marginStart
        this.marginEnd = marginEnd
        topMargin = marginTop
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
        gimbalPad = StickPadView(this).apply {
            isEnabled = false
            axisLabels = StickAxisLabels(up = "抬高", down = "低頭", left = "左看", right = "右看")
            onPosition = ::onGimbalMoved
        }
        return LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.BOTTOM
            addView(padColumn(leftPad, "左桿 • 旋轉 / 升降"))
            addView(View(this@MainActivity), LinearLayout.LayoutParams(0, 1, 1f))
            addView(gimbalColumn())
            addView(View(this@MainActivity), LinearLayout.LayoutParams(0, 1, 1f))
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

    private fun gimbalColumn(): ViewGroup =
        LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.CENTER_HORIZONTAL
            addView(gimbalPad)
            addView(
                label(9f, StickPadView.TEXT, bold = true).apply {
                    text = "攝影機 • 方向"
                    gravity = Gravity.CENTER
                },
            )
        }

    private fun onGimbalMoved(x: Double, y: Double) {
        val active = x != 0.0 || y != 0.0
        if (active && tapeTracking.enabled) {
            stopTapeTracking("攝影機搖桿已接管，黑膠帶追蹤已停止", release = true)
        }
        if (active) {
            cameraPitchCommandGeneration += 1
            mainHandler.removeCallbacks(cameraPitchCommandTimeoutRunnable)
            cameraPitchCommandPending = false
            selectedCameraPitchDegrees = null
        }
        if (active != gimbalActive) {
            gimbalActive = active
            flightLog.write(
                "gimbal ${if (active) "engaged" else "released"} x=%.2f y=%.2f".format(x, y),
            )
            render(if (active) "攝影機角度控制中" else "攝影機角度已停止")
        }
        gimbal.setInput(x, y)
    }

    private fun moveCameraToPitch(pitchDegrees: Double) {
        flightLog.write(
            "press: camera pitch=%.0f registered=$registered connected=$aircraftConnected".format(
                pitchDegrees,
            ),
        )
        if (cameraPitchCommandPending) {
            render("鏡頭角度切換中，請稍候")
            return
        }
        if (!registered || !aircraftConnected) {
            render("飛機未連線，無法控制攝影機")
            return
        }
        if (tapeTracking.enabled) {
            stopTapeTracking("攝影機角度變更，黑膠帶追蹤已停止", release = true)
        }
        val commandGeneration = ++cameraPitchCommandGeneration
        cameraPitchCommandPending = true
        selectedCameraPitchDegrees = null
        render("鏡頭正在移至 %.0f°…".format(pitchDegrees))
        mainHandler.removeCallbacks(cameraPitchCommandTimeoutRunnable)
        mainHandler.postDelayed(
            cameraPitchCommandTimeoutRunnable,
            CAMERA_PITCH_COMMAND_TIMEOUT_MS,
        )
        gimbal.rotateTo(
            pitchDegrees,
            0.0,
            CAMERA_RECENTER_DURATION_SECONDS,
        ) { error ->
            runOnUiThread {
                if (commandGeneration != cameraPitchCommandGeneration) return@runOnUiThread
                mainHandler.removeCallbacks(cameraPitchCommandTimeoutRunnable)
                cameraPitchCommandPending = false
                if (error == null) {
                    selectedCameraPitchDegrees = pitchDegrees
                    flightLog.write("camera pitch=%.0f accepted".format(pitchDegrees))
                    render("鏡頭已移至 %.0f°".format(pitchDegrees))
                } else {
                    render("鏡頭角度控制失敗：$error")
                }
            }
        }
    }

    private fun toggleTapeTracking() {
        toggleTapeTracking(TapeTrackingMode.STRAIGHT)
    }

    private fun toggleCircularTapeTracking() {
        toggleTapeTracking(TapeTrackingMode.CIRCULAR)
    }

    private fun toggleCurvedOutAndBackTracking() {
        toggleTapeTracking(TapeTrackingMode.CURVED_OUT_AND_BACK)
    }

    private fun toggleTapeTracking(mode: TapeTrackingMode) {
        when {
            tapeTracking.enabled && activeTapeTrackingMode == mode ->
                stopTapeTracking("${tapeTrackingName(mode)}已由操作者停止", release = true)
            tapeTracking.enabled -> render("另一個黑膠帶追蹤模式正在使用中")
            tapeTrackingStartPending -> render("正在切換飛機避障模式，請稍候")
            else -> startTapeTracking(mode)
        }
    }


    private fun startTapeTracking(mode: TapeTrackingMode) {
        val trackingName = tapeTrackingName(mode)
        flightLog.write(
            "press: tape tracking mode=$mode registered=$registered connected=$aircraftConnected " +
                "flying=$flying owned=$stickOwned cameraPitch=" +
                (selectedCameraPitchDegrees?.let { "%.0f".format(it) } ?: "unknown"),
        )
        if (!registered || !aircraftConnected || !flying) {
            render("飛機未在空中，無法啟動$trackingName")
            return
        }
        if (tapeDetector == null) {
            render("OpenCV 未啟動，無法啟動$trackingName")
            return
        }
        if (anotherFlightControlActive()) {
            render("另一個飛行控制正在使用中")
            return
        }
        // Tracking closes obstacle avoidance and reads the route through the
        // camera alone, so the -90° quick command must be confirmed before any
        // control is taken. The tolerance absorbs floating-point representation
        // noise, never a deliberate tilt.
        val confirmedPitch = selectedCameraPitchDegrees
        val cameraNotDownReason = when {
            cameraPitchCommandPending -> "鏡頭角度切換中，請稍候再試"
            confirmedPitch == null -> "鏡頭角度未確認，請先按「鏡頭 -90°」把鏡頭朝下"
            abs(confirmedPitch - CAMERA_DOWN_PITCH_DEGREES) >
                CAMERA_DOWN_PITCH_TOLERANCE_DEGREES ->
                "鏡頭在 %.0f° 而非 -90°，請先按「鏡頭 -90°」".format(confirmedPitch)
            else -> null
        }
        if (cameraNotDownReason != null) {
            render("無法啟動$trackingName：$cameraNotDownReason")
            return
        }
        mainHandler.removeCallbacks(idleReleaseRunnable)
        activeTapeTrackingMode = mode
        tapeTrackingStartPending = true
        render("$trackingName：正在關閉飛機避障…")
        avoidanceCheck.ensureClosed { status ->
            if (activityDestroying) {
                if (status.closedConfirmed) avoidanceCheck.ensureBrake {}
            } else {
                runOnUiThread {
                    if (activityDestroying) {
                        if (status.closedConfirmed) avoidanceCheck.ensureBrake {}
                        return@runOnUiThread
                    }
                    tapeTrackingStartPending = false
                    avoidance = status
                    flightLog.write("tape tracking avoidance ${status.summary} detail=${status.detail}")
                    if (!status.closedConfirmed) {
                        activeTapeTrackingMode = null
                        render(status.detail?.let { "無法關閉飛機避障：$it" } ?: "無法確認飛機避障已關閉")
                        readAvoidanceConfiguration()
                        return@runOnUiThread
                    }
                    if (!aircraftConnected || !flying) {
                        stopTapeTracking("飛行狀態已改變，$trackingName 取消", release = false)
                        return@runOnUiThread
                    }
                    render("飛機避障已關閉；$trackingName：取得控制權…")
                    acquireControlLink(
                        onFailure = { reason ->
                            stopTapeTracking("$trackingName 取消：$reason", release = false)
                        },
                    ) {
                        tapeEndpointTurn = false
                        tapeDetector?.setDetectionMode(
                            if (mode.followsCurvedPath) {
                                TapeDetectionMode.PATH
                            } else {
                                TapeDetectionMode.STRAIGHT
                            },
                        )
                        // The idle preview may have accepted scenery before takeoff.
                        // A flight session must learn its board from the current route.
                        tapeDetector?.resetTracking()
                        val now = System.nanoTime()
                        // The circular experiment uses a physically closed loop. Curved
                        // out-and-back tracking uses the same path detector with its endpoint armed.
                        val endpointTurnEnabled = mode != TapeTrackingMode.CIRCULAR
                        tapeTracking.start(now, mode, endpointTurnEnabled)
                        tapeTrackingStartedAtNanos = now
                        tapeTrackingAuthoritySeen =
                            stickStatus.authority == VirtualStickSession.MSDK_AUTHORITY_OWNER
                        renderedTapeTrackingPhase = TapeTrackingPhase.DISABLED
                        commandedTapeYawRate = 0.0
                        commandedTapeForwardSpeed = 0.0
                        commandedTapeRightSpeed = 0.0
                        tapeCommandLoggedAtNanos = 0L
                        when (mode) {
                            TapeTrackingMode.STRAIGHT ->
                                tapeTrackingButton.text = "停止直線黑膠帶追蹤"
                            TapeTrackingMode.CIRCULAR ->
                                circularTapeTrackingButton.text = "停止圓形黑膠帶追蹤"
                            TapeTrackingMode.CURVED_OUT_AND_BACK ->
                                curvedOutAndBackTrackingButton.text = "停止弧形往返追蹤"
                        }
                        flightLog.write(
                            "tape tracking started mode=$mode avoidance=CLOSE " +
                                "endpointTurnEnabled=$endpointTurnEnabled",
                        )
                        mainHandler.removeCallbacks(tapeTrackingTickRunnable)
                        mainHandler.post(tapeTrackingTickRunnable)
                    }
                }
            }
        }
    }

    private val tapeTrackingTickRunnable = object : Runnable {
        override fun run() {
            if (!tapeTracking.enabled) return
            if (!registered || !aircraftConnected || !flying || !stickOwned) {
                stopTapeTracking("飛行或控制權狀態失效，黑膠帶追蹤已停止", release = false)
                return
            }
            val now = System.nanoTime()
            val ownsAuthority =
                stickStatus.authority == VirtualStickSession.MSDK_AUTHORITY_OWNER
            if (ownsAuthority) {
                tapeTrackingAuthoritySeen = true
            } else {
                virtualStick.setYawRate(0.0)
                virtualStick.setForwardOnly(0.0)
                if (
                    tapeTrackingAuthoritySeen ||
                    now - tapeTrackingStartedAtNanos >= AUTHORITY_HANDOVER_TIMEOUT_NANOS
                ) {
                    stopTapeTracking("未取得或已失去控制權，黑膠帶追蹤已停止", release = false)
                    return
                }
            }


            val decision = tapeTracking.tick(now)
            if (decision.stopRequested) {
                stopTapeTracking("末端確認逾時，黑膠帶追蹤已停止", release = true)
                return
            }
            if (decision.endpointReached) {
                beginTapeTurnaround()
                return
            }
            val yawRate = if (ownsAuthority) decision.yawRateDegreesPerSecond else 0.0
            val requestedForwardSpeed =
                if (ownsAuthority) decision.forwardSpeedMetersPerSecond else 0.0
            val requestedRightSpeed =
                if (ownsAuthority) decision.rightSpeedMetersPerSecond else 0.0
            if (requestedForwardSpeed != 0.0 || requestedRightSpeed != 0.0) {
                tapeTrackingStopReason()?.let { reason ->
                    stopTapeTracking(reason, release = true)
                    return
                }
            }
            virtualStick.setYawRate(yawRate)
            virtualStick.setHorizontalVelocity(requestedForwardSpeed, requestedRightSpeed)
            val commandChanged =
                yawRate != commandedTapeYawRate ||
                    requestedForwardSpeed != commandedTapeForwardSpeed ||
                    requestedRightSpeed != commandedTapeRightSpeed
            if (commandChanged) {
                commandedTapeYawRate = yawRate
                commandedTapeForwardSpeed = requestedForwardSpeed
                commandedTapeRightSpeed = requestedRightSpeed
                val stopped =
                    yawRate == 0.0 && requestedForwardSpeed == 0.0 && requestedRightSpeed == 0.0
                if (stopped || now - tapeCommandLoggedAtNanos >= TAPE_COMMAND_LOG_PERIOD_NANOS) {
                    tapeCommandLoggedAtNanos = now
                    flightLog.write(
                        "tape tracking rawAngle=${decision.rawAngleDegrees.logNumber(1)} " +
                            "angle=${decision.controlledAngleDegrees.logNumber(1)} " +
                            "rawOffset=${decision.rawOffsetFraction.logNumber(3)} " +
                            "offset=${decision.controlledOffsetFraction.logNumber(3)} " +
                            "offsetRate=%+.3f ppYaw=%+.1f yaw=%+.1f forward=%.2f right=%+.2f phase=${decision.phase}".format(
                                decision.offsetRatePerSecond,
                                decision.purePursuitYawRateDegreesPerSecond,
                                yawRate,
                                requestedForwardSpeed,
                                requestedRightSpeed,
                            ),
                    )
                }
            }
            if (decision.phase != renderedTapeTrackingPhase) {
                renderedTapeTrackingPhase = decision.phase
                val trackingName = tapeTrackingName(activeTapeTrackingMode)
                val status = when (decision.phase) {
                    TapeTrackingPhase.RECENTERING -> "$trackingName：循跡資料穩定中"
                    TapeTrackingPhase.RECOVERING_AFTER_TURN ->
                        "$trackingName：回轉完成，低速前移重新取得膠帶"
                    TapeTrackingPhase.TRACKING ->
                        "$trackingName（Pure Pursuit 前視點導引）"
                    TapeTrackingPhase.ALIGNING_CURVE ->
                        "$trackingName：急彎原地對準中"
                    TapeTrackingPhase.REACQUIRING_PATH ->
                        "$trackingName：等待可信路徑重新出現"
                    TapeTrackingPhase.VERIFYING_ENDPOINT ->
                        "$trackingName：疑似末端，低速前進確認中"
                    TapeTrackingPhase.TURNING -> "$trackingName：確認末端，向右旋轉 180°"
                    TapeTrackingPhase.DISABLED -> "$trackingName 已停止"
                }
                holdStatus = status
                flightLog.write("tape tracking phase=${decision.phase}")
                if (
                    decision.phase == TapeTrackingPhase.VERIFYING_ENDPOINT &&
                    activeTapeTrackingMode?.followsCurvedPath == true
                ) {
                    // Curved modes reach endpoint verification only after a confirmed
                    // reliable route disappears; straight mode enters by shrinking-endpoint
                    // evidence and must not carry this reason.
                    flightLog.write(
                        "tape tracking endpoint verification reason=reliable-path-lost " +
                            tapeDetector?.diagnosticsSummary().orEmpty(),
                    )
                }
                render(status)
            }
            mainHandler.postDelayed(this, TAPE_TRACKING_TICK_MS)
        }
    }



    private fun tapeTrackingName(mode: TapeTrackingMode? = activeTapeTrackingMode): String =
        when (mode) {
            TapeTrackingMode.CIRCULAR -> "圓形黑膠帶追蹤"
            TapeTrackingMode.CURVED_OUT_AND_BACK -> "弧形往返追蹤"
            else -> "直線黑膠帶追蹤"
        }

    private fun beginTapeTurnaround() {
        check(activeTapeTrackingMode != null) {
            "Tape endpoint turnaround requires an active tracking mode"
        }
        val heading = aircraftHeadingDegrees
        if (heading == null || aircraftHeadingAtNanos == 0L) {
            stopTapeTracking(
                "已確認膠帶末端，但沒有機頭方向資料，追蹤已停止",
                release = true,
            )
            return
        }
        mainHandler.removeCallbacks(tapeTrackingTickRunnable)
        virtualStick.setForwardOnly(0.0)
        virtualStick.setYawRate(0.0)
        commandedTapeForwardSpeed = 0.0
        commandedTapeYawRate = 0.0
        commandedTapeRightSpeed = 0.0
        tapeEndpointTurn = true
        tapeDetector?.resetTracking()
        tapeOverlay.showDetection(null)
        flightLog.write("tape endpoint confirmed; detector remains active; right 180 armed")
        holdStatus = "確認膠帶末端：向右旋轉 180°"
        render(holdStatus)
        armHeadingTurn(heading)
    }


    private fun stopTapeTracking(
        message: String,
        release: Boolean,
    ) {
        val wasActive =
            tapeTrackingStartPending || tapeTracking.enabled ||
                renderedTapeTrackingPhase != TapeTrackingPhase.DISABLED ||
                avoidance.closedConfirmed
        if (tapeEndpointTurn) {
            tapeEndpointTurn = false
            headingTurn = null
            turnCommandStartedAtNanos = 0L
            turnAuthoritySeen = false
            mainHandler.removeCallbacks(headingTurnTickRunnable)
        }
        tapeTracking.stop()
        tapeDetector?.setDetectionMode(TapeDetectionMode.PATH)
        tapeDetector?.resetTracking()
        tapeTrackingStartPending = false
        mainHandler.removeCallbacks(tapeTrackingTickRunnable)
        virtualStick.setYawRate(0.0)
        virtualStick.setForwardOnly(0.0)
        commandedTapeYawRate = 0.0
        commandedTapeForwardSpeed = 0.0
        commandedTapeRightSpeed = 0.0
        tapeTrackingAuthoritySeen = false
        tapeTrackingStartedAtNanos = 0L
        renderedTapeTrackingPhase = TapeTrackingPhase.DISABLED
        tapeCommandLoggedAtNanos = 0L
        activeTapeTrackingMode = null
        if (::tapeTrackingButton.isInitialized) tapeTrackingButton.text = "直線黑膠帶追蹤"
        if (::circularTapeTrackingButton.isInitialized) {
            circularTapeTrackingButton.text = "圓形黑膠帶追蹤"
        }
        if (::curvedOutAndBackTrackingButton.isInitialized) {
            curvedOutAndBackTrackingButton.text = "弧形往返追蹤"
        }
        if (!wasActive) {
            render(message)
            return
        }
        holdStatus = ""
        flightLog.write(message)
        restoreBrakeAfterTapeTracking(message, release)
    }

    private fun restoreBrakeAfterTapeTracking(message: String, release: Boolean) {
        if (!aircraftConnected) {
            render(message)
            return
        }
        avoidanceCheck.ensureBrake { status ->
            runOnUiThread {
                avoidance = status
                flightLog.write("tape tracking restore ${status.summary} detail=${status.detail}")
                val restoredMessage = if (status.brakeConfirmed) {
                    "$message；BRAKE 避障已恢復"
                } else {
                    "$message；警告：${status.warning ?: "BRAKE 避障恢復失敗"}"
                }
                refreshHorizontalSafety("BRAKE read-back")
                if (
                    release && stickOwned && !stickTransitionPending &&
                    !leftStickActive && !rightStickActive && !holdingHeight &&
                    headingTurn == null
                ) {
                    releaseControlLink { error ->
                        render(error?.let { "$restoredMessage（釋放控制權失敗：$it）" } ?: restoredMessage)
                    }
                } else {
                    render(restoredMessage)
                }
            }
        }
    }

    private fun onStickMoved(side: StickSide, x: Double, y: Double) {
        val isDeflected = x != 0.0 || y != 0.0
        if (isDeflected && tapeTracking.enabled) {
            stopTapeTracking("畫面搖桿已接管，黑膠帶追蹤已停止", release = false)
        }
        if (isDeflected && quarterArcController != null) {
            finishQuarterArc("畫面搖桿已接管，無視覺 1/4 圈已停止", release = false)
        }
        val horizontalStopReason =
            if (side == StickSide.RIGHT && isDeflected) {
                horizontalActuationStopReason()
            } else {
                null
            }
        if (horizontalStopReason != null) {
            rightStickActive = false
            virtualStick.setStick(StickSide.RIGHT, 0.0, 0.0)
            flightLog.write("horizontal stick refused: $horizontalStopReason")
            render(horizontalStopReason)
            return
        }
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
            flightLog.write("height target cancelled by operator stick input")
            finishHeightHold("操作者接管升降，定高已取消", release = false)
        }
        if (side == StickSide.LEFT && (x != 0.0 || y != 0.0) && headingTurn != null) {
            finishHeadingTurn("操作者接管旋轉，180° 旋轉已取消", release = false)
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
        if (
            leftStickActive || rightStickActive || holdingHeight || headingTurn != null ||
            quarterArcController != null || tapeTracking.enabled || !stickOwned
        ) return@Runnable
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
            lowCellVoltage || (batteryPercent?.let { it <= BATTERY_CRITICAL_PERCENT } == true) ->
                "LOCAL FLIGHT • BATTERY CRITICAL" to StickPadView.RED
            !registered -> "LOCAL FLIGHT • REGISTERING" to StickPadView.AMBER
            !aircraftConnected -> "LOCAL FLIGHT • NO AIRCRAFT" to StickPadView.RED
            confirmationNeeded -> "LOCAL FLIGHT • LANDING CONFIRM" to StickPadView.AMBER
            stickStatus.enabled && !horizontalSafetyReady ->
                "LOCAL FLIGHT • MANUAL • NO OA" to StickPadView.AMBER
            stickStatus.enabled -> "LOCAL FLIGHT • MANUAL" to StickPadView.GREEN
            flying -> "LOCAL FLIGHT • AIRBORNE" to StickPadView.GREEN
            else -> "LOCAL FLIGHT • READY" to StickPadView.CYAN
        }
        headlineView.text = headline
        headlineView.setTextColor(headlineColor)
        detailView.text = message
        holdView.text = holdStatus
        holdView.visibility = if (holdStatus.isEmpty()) View.GONE else View.VISIBLE
        updateTelemetryText()

        // The escape hatch only exists while it can do something: once registered
        // it would be a button that cannot help.
        registerButton.visibility = if (registered) View.GONE else View.VISIBLE
        registerButton.available = !registered
        takeoffButton.available = ready && !flying
        landButton.available = ready && flying
        val turning = headingTurn != null
        val quarterArcActive = quarterArcController != null
        val heightButtonAvailable =
            ready && flying && !holdingHeight && !turning && !quarterArcActive &&
                !tapeTracking.enabled
        holdButton.available = heightButtonAvailable
        oneMeterHoldButton.available = heightButtonAvailable
        captureButton.available = captureRecorder != null
        cameraDownButton.available = ready && !cameraPitchCommandPending
        val tapeTrackingCanStart =
            ready && flying && !turning && !quarterArcActive && !holdingHeight
        tapeTrackingButton.available =
            (tapeTracking.enabled && activeTapeTrackingMode == TapeTrackingMode.STRAIGHT) ||
                (!tapeTracking.enabled && tapeTrackingCanStart)
        circularTapeTrackingButton.available =
            (tapeTracking.enabled && activeTapeTrackingMode == TapeTrackingMode.CIRCULAR) ||
                (!tapeTracking.enabled && tapeTrackingCanStart)
        curvedOutAndBackTrackingButton.available =
            (tapeTracking.enabled &&
                activeTapeTrackingMode == TapeTrackingMode.CURVED_OUT_AND_BACK) ||
                (!tapeTracking.enabled && tapeTrackingCanStart)
        quarterArcButton.available =
            quarterArcActive ||
                (ready && flying && !turning && !holdingHeight && !tapeTracking.enabled)
        leftPad.isEnabled = ready && flying
        rightPad.isEnabled = ready && flying
        gimbalPad.isEnabled = ready
    }

    private fun updateTelemetryText() {
        val obstacleAgeMs = obstacleSampleAtNanos.takeIf { it != 0L }?.let {
            ((System.nanoTime() - it) / 1_000_000L).coerceAtLeast(0L)
        }
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
            append(" · hold=").append(heightTargetMeters?.let { "%.1fm".format(it) } ?: "off")
            append(" · heading=").append(aircraftHeadingDegrees?.let { "%.1f°".format(it) } ?: "—")
            append(" · turn=").append(headingTurn?.let { "%.0f/180°".format(it.progressDegrees) } ?: "off")
            append(" · landingConfirmNeeded=").append(confirmationNeeded)
            append("\nbattery=").append(batteryPercent?.let { "$it%" } ?: "—")
            append(if (lowCellVoltage) " lowCell" else "")
            append(" · ").append(avoidance.summary)
            append(if (activelyAvoiding) " · AVOIDING" else "")
            append(" · vision=").append(if (horizontalSafetyReady) "RANGE OK" else "NO RANGE")
            append(" · horizontal=MANUAL")
            append("\n視覺 raw(mm)：水平=")
            append(horizontalRawMinMm ?: "—").append("..").append(horizontalRawMaxMm ?: "—")
            append(" · 真值=").append(horizontalDetectedCount).append("/").append(horizontalSampleCount)
            append(" · 地面回波=").append(horizontalGroundEchoCount)
            append(if (horizontalGroundEchoSuppressed) "（已排除）" else "")
            append(" · 間隔=").append(horizontalAngleIntervalDegrees).append("°")
            append(" · age=").append(obstacleAgeMs?.let { "${it}ms" } ?: "—")
            append("\n視覺 raw(mm)：上=").append(upwardObstacleMm ?: "—")
            append(" · 下=").append(downwardObstacleMm ?: "—")
            append(" · 最近真值=").append(nearestHorizontalObstacleMm ?: "—")
            append("\n撞擊後停槳：只用實體 RC 執行 CSC 並持續 2 秒")
        }
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

    private fun Double?.logNumber(decimalPlaces: Int): String =
        this?.let { "%+.${decimalPlaces}f".format(it) } ?: "null"

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
        val sdkManager = SDKManager.getInstance()
        if (sdkManager.isRegistered) {
            activateRegisteredSdk("restored after Activity restart")
            return
        }
        sdkManager.init(
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
                    activateRegisteredSdk("registration succeeded")
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

    private fun activateRegisteredSdk(logMessage: String) {
        if (registered) return
        registered = true
        registerAttempts = 0
        mainHandler.removeCallbacks(registrationRetryRunnable)
        flightLog.write(logMessage)
        render("app key 註冊成功，等待飛機連線…")
        listenAircraftState()
        virtualStick.start()
        runOnUiThread { preview.refresh() }
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
            aircraftHeadingDegrees = null
            aircraftHeadingAtNanos = 0L
            obstacleSampleReceived = false
            obstacleSampleValid = false
            obstacleSampleAtNanos = 0L
            nearestHorizontalObstacleMm = null
            horizontalRawMinMm = null
            horizontalRawMaxMm = null
            horizontalDetectedCount = 0
            horizontalGroundEchoCount = 0
            horizontalGroundEchoSuppressed = false
            horizontalSampleCount = 0
            horizontalAngleIntervalDegrees = 0
            upwardObstacleMm = null
            downwardObstacleMm = null
            if (aircraftConnected) {
                readAvoidanceConfiguration()
            } else {
                avoidance = AvoidanceCheck.Status()
                refreshHorizontalSafety("aircraft disconnected")
                runOnUiThread {
                    consecutiveTapeMisses = 0
                    tapeDetected = false
                    tapeOverlay.showDetection(null)
                    tapeOverlay.showCenterline(null)
                }
            }
            // Re-assert both preview and RGBA listener across link transitions;
            // MSDK can restore the Surface before the flight-controller key.
            runOnUiThread { preview.refresh() }
            releaseIfNotFlying()
            render(if (aircraftConnected) "飛機已連線，正在確認 BRAKE 與障礙距離…" else "飛機未連線")
            if (aircraftConnected) evaluateBattery()
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
            evaluateBattery()
        }
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyAircraftAttitude),
            this,
            true,
        ) { _, attitude ->
            runOnUiThread { publishAircraftAttitude(attitude) }
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
        // Boolean, so there is no unit to get wrong: the aircraft telling us it is
        // braking for an obstacle is the earliest trustworthy stop signal we have.
        keyManager.listen(
            KeyTools.createKey(FlightAssistantKey.KeyIsActivelyAvoidingObstacle),
            this,
            true,
        ) { _, avoiding ->
            val active = avoiding == true
            if (active != activelyAvoiding) flightLog.write("actively avoiding obstacle=$active")
            activelyAvoiding = active
            refreshHorizontalSafety("active avoidance changed")
        }
        keyManager.listen(
            KeyTools.createKey(BatteryKey.KeyChargeRemainingInPercent),
            this,
            true,
        ) { _, percent ->
            val previous = batteryPercent
            batteryPercent = percent
            if (previous != percent) flightLog.write("battery percent=$percent")
            evaluateBattery()
        }
        keyManager.listen(
            KeyTools.createKey(BatteryKey.KeyIsLowCellVoltageDetected),
            this,
            true,
        ) { _, detected ->
            lowCellVoltage = detected == true
            if (lowCellVoltage) flightLog.write("low cell voltage detected")
            evaluateBattery()
        }
        observeObstacles()
    }

    private fun readAvoidanceConfiguration() {
        avoidanceCheck.ensureBrake { status ->
            runOnUiThread {
                avoidance = status
                flightLog.write("avoidance config ${status.summary} detail=${status.detail}")
                refreshHorizontalSafety("BRAKE read-back")
            }
        }
    }

    /**
     * DJI uses 60,000 mm to mean "not detected", so an all-sentinel sample still
     * fails closed. At sub-metre altitude the Mini 4 Pro reports the floor in
     * almost every horizontal azimuth. A dense low-altitude ring with enough
     * floor-height evidence is therefore removed. Sparse ranges and objects
     * substantially closer than the floor remain stop conditions.
     */
    private fun observeObstacles() {
        if (obstacleDataListener != null) return
        val listener = ObstacleDataListener { data ->
            val horizontal = data.horizontalObstacleDistance
            val downwardMm = data.downwardObstacleDistance
            val summary = HorizontalObstacleFilter.summarize(horizontal, downwardMm)
            val hasHorizontalSamples = horizontal.isNotEmpty()
            val interval = data.horizontalAngleInterval
            val upwardMm = data.upwardObstacleDistance
            val now = System.nanoTime()
            runOnUiThread {
                obstacleSampleReceived = interval > 0 && hasHorizontalSamples
                obstacleSampleValid =
                    obstacleSampleReceived &&
                        (summary.nearestActionableMm != null || summary.groundEchoDominant)
                nearestHorizontalObstacleMm = summary.nearestActionableMm
                horizontalRawMinMm = summary.rawMinimumMm
                horizontalRawMaxMm = summary.rawMaximumMm
                horizontalDetectedCount = summary.detectedCount
                horizontalGroundEchoCount = summary.groundEchoCount
                horizontalGroundEchoSuppressed = summary.groundEchoDominant
                horizontalSampleCount = horizontal.size
                horizontalAngleIntervalDegrees = interval
                upwardObstacleMm = upwardMm
                downwardObstacleMm = downwardMm
                obstacleSampleAtNanos = if (obstacleSampleReceived) now else 0L
                refreshHorizontalSafety("obstacle sample")
                if (now - obstacleTelemetryRenderedAtNanos >= OBSTACLE_TELEMETRY_PERIOD_NANOS) {
                    obstacleTelemetryRenderedAtNanos = now
                    updateTelemetryText()
                }
            }
            if (now - obstacleLoggedAtNanos >= OBSTACLE_LOG_PERIOD_NANOS) {
                obstacleLoggedAtNanos = now
                flightLog.write(
                    "obstacle actionableMinMm=${summary.nearestActionableMm ?: "none"} " +
                        "detected=${summary.detectedCount}/${horizontal.size} " +
                        "groundEchoes=${summary.groundEchoCount} suppressed=${summary.groundEchoDominant} " +
                        "intervalDeg=$interval rawMinMm=${summary.rawMinimumMm ?: "none"} " +
                        "rawMaxMm=${summary.rawMaximumMm ?: "none"} upMm=$upwardMm downMm=$downwardMm",
                )
            }
        }
        obstacleDataListener = listener
        runCatching {
            PerceptionManager.getInstance().apply {
                init()
                addObstacleDataListener(listener)
            }
            flightLog.write("obstacle listener registered")
        }.onFailure {
            obstacleDataListener = null
            flightLog.write("obstacle listener unavailable: $it")
        }
    }

    private fun horizontalSafetyFailure(nowNanos: Long = System.nanoTime()): String? {
        if (!aircraftConnected) return "水平避障不可用：飛機未連線"
        if (!avoidance.brakeConfirmed) return avoidance.warning ?: "水平避障不可用：BRAKE 未確認"
        if (activelyAvoiding) return "水平控制停止：飛機正在避障剎車"
        if (!obstacleSampleReceived || obstacleSampleAtNanos == 0L) {
            return "水平避障不可用：沒有障礙距離資料"
        }
        if (!obstacleSampleValid) {
            return "水平避障不可用：360° 感測器未看見任何門牆"
        }
        val ageMs = (nowNanos - obstacleSampleAtNanos) / 1_000_000L
        if (ageMs > MAX_MANUAL_OBSTACLE_SAMPLE_AGE_MS) {
            return "水平避障不可用：障礙距離資料已中斷 ${ageMs}ms"
        }
        val nearest = nearestHorizontalObstacleMm
        if (nearest == null && horizontalGroundEchoSuppressed) return null
        if (nearest == null) return "水平避障不可用：沒有水平障礙距離"
        if (nearest <= MANUAL_HORIZONTAL_CLEARANCE_MM) {
            return "水平控制停止：障礙距離 ${nearest}mm（門檻 ${MANUAL_HORIZONTAL_CLEARANCE_MM}mm）"
        }
        return null
    }

    /**
     * Tape tracking disables DJI avoidance, so a fresh callback is required and
     * the app's own clearance is the final horizontal stop. DJI's 60 m sentinel
     * is a fresh callback reporting no detected obstacle, not missing telemetry.
     */
    private fun tapeTrackingStopReason(nowNanos: Long = System.nanoTime()): String? {
        if (!avoidance.closedConfirmed) {
            return avoidance.warning ?: "黑膠帶追蹤停止：無法確認飛機避障已關閉"
        }
        if (!isFreshObstacleSample(obstacleSampleReceived, obstacleSampleAtNanos, nowNanos)) {
            if (!obstacleSampleReceived || obstacleSampleAtNanos == 0L) {
                return "黑膠帶追蹤停止：障礙距離資料不可用"
            }
            val ageMs = (nowNanos - obstacleSampleAtNanos) / 1_000_000L
            return "黑膠帶追蹤停止：障礙距離資料已中斷 ${ageMs}ms"
        }
        val nearest = nearestHorizontalObstacleMm ?: return null
        return if (breachesAutonomousHorizontalClearance(nearest)) {
            "黑膠帶追蹤停止：障礙距離 ${nearest}mm（門檻 ${AUTONOMOUS_HORIZONTAL_CLEARANCE_MM}mm）"
        } else {
            null
        }
    }

    /**
     * Missing horizontal vision does not block operator-requested control on the
     * Mini 4 Pro. A real close range, DJI braking, disconnect, or landing still
     * zeros horizontal output immediately.
     */
    private fun horizontalActuationStopReason(nowNanos: Long = System.nanoTime()): String? {
        if (!aircraftConnected) return "水平控制停止：飛機未連線"
        if (!flying) return "水平控制停止：飛機不在空中"
        if (activelyAvoiding) return "水平控制停止：飛機正在避障剎車"
        val nearest = nearestHorizontalObstacleMm ?: return null
        if (!obstacleSampleValid || obstacleSampleAtNanos == 0L) return null
        val ageMs = (nowNanos - obstacleSampleAtNanos) / 1_000_000L
        if (ageMs > MAX_MANUAL_OBSTACLE_SAMPLE_AGE_MS) return null
        return if (nearest <= MANUAL_HORIZONTAL_CLEARANCE_MM) {
            "水平控制停止：障礙距離 ${nearest}mm（門檻 ${MANUAL_HORIZONTAL_CLEARANCE_MM}mm）"
        } else {
            null
        }
    }

    private fun refreshHorizontalSafety(trigger: String) {
        val failure = horizontalSafetyFailure()
        val ready = failure == null
        val changed = ready != horizontalSafetyReady
        horizontalSafetyReady = ready
        if (changed) {
            flightLog.write(
                "horizontal safety ready=$ready trigger=$trigger nearestMm=$nearestHorizontalObstacleMm " +
                    "reason=${failure ?: "clear"}",
            )
        }
        horizontalActuationStopReason()?.let { reason ->
            if (rightStickActive) stopHorizontalActuation(reason)
        }
        if (quarterArcController != null) {
            quarterArcStopReason(System.nanoTime())?.let { reason ->
                finishQuarterArc("無視覺 1/4 圈安全停止：$reason")
            }
        }
        if (changed || trigger == "BRAKE read-back") {
            render(failure ?: "BRAKE 已確認，水平避障資料可用")
        }
    }

    private fun stopHorizontalActuation(reason: String) {
        rightStickActive = false
        virtualStick.setForwardOnly(0.0)
        holdStatus = reason
        flightLog.write("horizontal actuation stopped: $reason")
        if (
            stickOwned && !leftStickActive && !holdingHeight && headingTurn == null &&
            quarterArcController == null && !tapeTracking.enabled
        ) {
            mainHandler.removeCallbacks(idleReleaseRunnable)
            mainHandler.post(idleReleaseRunnable)
        }
    }

    private val horizontalSafetyWatchdog = object : Runnable {
        override fun run() {
            refreshHorizontalSafety("watchdog")
            mainHandler.postDelayed(this, HORIZONTAL_WATCHDOG_MS)
        }
    }

    /** Which key the current height came from; a better source may replace a worse one. */
    private enum class HeightSource { NONE, LOCATION_3D, ALTITUDE }

    private fun publishAircraftAttitude(attitude: Attitude?) {
        val heading = attitude?.yaw?.takeIf(Double::isFinite) ?: return
        aircraftHeadingDegrees = heading
        aircraftHeadingAtNanos = System.nanoTime()
        headingTurn?.update(heading)
        driveHeadingTurn()
        quarterArcController?.updateHeading(heading)
        driveQuarterArc()
    }



    private fun publishHeight(meters: Double?, source: HeightSource = HeightSource.ALTITUDE) {
        if (!HeightHoldPolicy.isUsableCurrentHeight(meters)) {
            if (source == heightSource) {
                altitudeMeters = null
                altitudeAtNanos = 0L
                heightSource = HeightSource.NONE
                if (holdingHeight) driveHeightHold()
            }
            return
        }
        altitudeMeters = meters
        altitudeAtNanos = System.nanoTime()
        heightSource = source
        driveHeightHold()
    }

    // -------------------------------------------------------- actuation ----

    /**
     * Every automated manoeuvre and both on-screen sticks are mutually exclusive: they all
     * drive the same virtual-stick axes. Each entry point asks this one question, so a new
     * manoeuvre can never be forgotten in one caller's copy of the list. Callers reached
     * through a toggle have already proven their own activity is idle.
     */
    private fun anotherFlightControlActive(): Boolean =
        headingTurn != null || quarterArcController != null || holdingHeight || tapeTracking.enabled ||
            tapeTrackingStartPending || leftStickActive || rightStickActive

    private fun toggleQuarterArc() {
        if (quarterArcController != null) {
            finishQuarterArc("操作者停止無視覺 1/4 圈")
            return
        }
        quarterArcStartFailure()?.let { reason ->
            flightLog.write("quarter arc refused: $reason")
            render(reason)
            return
        }
        render("正在取得控制權，準備無視覺右轉 1/4 圈…")
        acquireControlLink(
            onFailure = { reason -> render("無視覺 1/4 圈未啟動：$reason") },
        ) {
            quarterArcStartFailure()?.let { reason ->
                flightLog.write("quarter arc aborted after acquire: $reason")
                releaseControlLink { error ->
                    render(
                        if (error == null) {
                            "無視覺 1/4 圈未啟動：$reason"
                        } else {
                            "無視覺 1/4 圈未啟動：$reason（釋放控制權失敗：$error）"
                        },
                    )
                }
                return@acquireControlLink
            }
            val initialHeading = checkNotNull(aircraftHeadingDegrees)
            quarterArcController = QuarterArcController(initialHeading)
            quarterArcStartedAtNanos = System.nanoTime()
            quarterArcAuthoritySeen =
                stickStatus.authority == VirtualStickSession.MSDK_AUTHORITY_OWNER
            quarterArcLastRenderedAtNanos = 0L
            runOnUiThread { quarterArcButton.text = "停止無視覺 1/4 圈" }
            virtualStick.setHorizontalVelocity(0.0, 0.0)
            virtualStick.setYawRate(0.0)
            flightLog.write(
                "quarter arc armed heading=%.1f radius=%.2f target=%.0f maxForward=%.2f acceleration=%.2f"
                    .format(
                        initialHeading,
                        QuarterArcController.RADIUS_METERS,
                        QuarterArcController.TARGET_DEGREES,
                        QuarterArcController.MAXIMUM_FORWARD_SPEED_METERS_PER_SECOND,
                        QuarterArcController.FORWARD_ACCELERATION_METERS_PER_SECOND_SQUARED,
                    ),
            )
            mainHandler.post(quarterArcTickRunnable)
        }
    }

    private fun quarterArcStartFailure(nowNanos: Long = System.nanoTime()): String? {
        if (anotherFlightControlActive()) return "請先停止其他飛行控制"
        if (!registered || !aircraftConnected || !flying) return "飛機未在空中，無法執行 1/4 圈"
        if (!avoidance.brakeConfirmed) {
            return avoidance.warning ?: "BRAKE 尚未確認，無視覺 1/4 圈不啟動"
        }
        horizontalActuationStopReason(nowNanos)?.let { return it }
        val height = usableHeightMeters() ?: return "沒有高度資料，無視覺 1/4 圈不啟動"
        if (height !in QUARTER_ARC_START_MIN_HEIGHT_METERS..QUARTER_ARC_START_MAX_HEIGHT_METERS) {
            return "請先定高 70 公分（目前 %.2f m）".format(height)
        }
        val heading = aircraftHeadingDegrees
        if (
            heading == null ||
            aircraftHeadingAtNanos == 0L ||
            nowNanos - aircraftHeadingAtNanos > MAX_MOVING_HEADING_AGE_NANOS
        ) {
            return "沒有即時機頭方向，無視覺 1/4 圈不啟動"
        }
        return null
    }

    private fun quarterArcStopReason(nowNanos: Long): String? {
        if (!avoidance.brakeConfirmed) {
            return avoidance.warning ?: "BRAKE 狀態失效"
        }
        horizontalActuationStopReason(nowNanos)?.let { return it }
        val height = usableHeightMeters() ?: return "高度資料中斷"
        if (height !in QUARTER_ARC_FLIGHT_MIN_HEIGHT_METERS..QUARTER_ARC_FLIGHT_MAX_HEIGHT_METERS) {
            return "高度超出安全範圍（目前 %.2f m）".format(height)
        }
        return null
    }

    private val quarterArcTickRunnable = object : Runnable {
        override fun run() {
            if (quarterArcController == null) return
            driveQuarterArc()
            if (quarterArcController != null) {
                mainHandler.postDelayed(this, QUARTER_ARC_TICK_MS)
            }
        }
    }

    private fun driveQuarterArc() {
        val controller = quarterArcController ?: return
        if (!stickOwned) {
            finishQuarterArc("無視覺 1/4 圈失去控制權，已停止", release = false)
            return
        }
        val now = System.nanoTime()
        val ownsAuthority = stickStatus.authority == VirtualStickSession.MSDK_AUTHORITY_OWNER
        if (ownsAuthority) {
            quarterArcAuthoritySeen = true
        } else if (quarterArcAuthoritySeen) {
            finishQuarterArc("實體遙控器已接管，無視覺 1/4 圈已停止", release = false)
            return
        } else {
            virtualStick.setHorizontalVelocity(0.0, 0.0)
            virtualStick.setYawRate(0.0)
            if (now - quarterArcStartedAtNanos > AUTHORITY_HANDOVER_TIMEOUT_NANOS) {
                finishQuarterArc("未取得控制權，無視覺 1/4 圈已取消")
            } else {
                holdStatus = "等待控制權移交後開始無視覺右轉 1/4 圈…"
                render(holdStatus)
            }
            return
        }
        quarterArcStopReason(now)?.let { reason ->
            finishQuarterArc("無視覺 1/4 圈安全停止：$reason")
            return
        }
        if (
            aircraftHeadingAtNanos == 0L ||
            now - aircraftHeadingAtNanos > MAX_MOVING_HEADING_AGE_NANOS
        ) {
            finishQuarterArc("機頭方向資料停止更新，無視覺 1/4 圈已停止")
            return
        }
        if (now - quarterArcStartedAtNanos > QUARTER_ARC_TIMEOUT_NANOS) {
            finishQuarterArc(
                "無視覺 1/4 圈逾時，已轉 %.0f°".format(controller.progressDegrees),
            )
            return
        }
        if (
            controller.remainingDegrees <= QUARTER_ARC_TOLERANCE_DEGREES
        ) {
            finishQuarterArc(
                "無視覺 1/4 圈完成（%.0f°）".format(controller.progressDegrees),
                succeeded = true,
            )
            return
        }
        val command = controller.command(now)
        virtualStick.setHorizontalVelocity(command.forwardSpeedMetersPerSecond, 0.0)
        virtualStick.setYawRate(command.yawRateDegreesPerSecond)
        if (now - quarterArcLastRenderedAtNanos >= QUARTER_ARC_RENDER_PERIOD_NANOS) {
            quarterArcLastRenderedAtNanos = now
            holdStatus =
                "無視覺右轉 1/4 圈：%.0f° / 90°，前進 %.2f m/s，yaw %.1f°/s"
                    .format(
                        controller.progressDegrees,
                        command.forwardSpeedMetersPerSecond,
                        command.yawRateDegreesPerSecond,
                    )
            flightLog.write(
                "quarter arc progress=%.1f remaining=%.1f forward=%.3f yaw=%.2f height=%.2f"
                    .format(
                        controller.progressDegrees,
                        controller.remainingDegrees,
                        command.forwardSpeedMetersPerSecond,
                        command.yawRateDegreesPerSecond,
                        usableHeightMeters(),
                    ),
            )
            render(holdStatus)
        }
    }

    private fun finishQuarterArc(
        message: String,
        succeeded: Boolean = false,
        release: Boolean = true,
    ) {
        val controller = quarterArcController ?: return
        val elapsedSeconds =
            (System.nanoTime() - quarterArcStartedAtNanos).coerceAtLeast(0L) / 1_000_000_000.0
        quarterArcController = null
        quarterArcAuthoritySeen = false
        mainHandler.removeCallbacks(quarterArcTickRunnable)
        virtualStick.setHorizontalVelocity(0.0, 0.0)
        virtualStick.setYawRate(0.0)
        runOnUiThread { quarterArcButton.text = "無視覺右轉 1/4 圈" }
        holdStatus = message
        flightLog.write(
            "quarter arc stopped success=$succeeded elapsed=%.2f progress=%.1f reason=$message"
                .format(elapsedSeconds, controller.progressDegrees),
        )
        if (!release || !stickOwned) {
            render(message)
            return
        }
        releaseControlLink { error ->
            holdStatus = if (error == null) message else "$message（釋放控制權失敗：$error）"
            render(holdStatus)
        }
    }

    private fun armHeadingTurn(currentHeading: Double) {
        headingTurn = HeadingTurn(currentHeading)
        turnStartedAtNanos = System.nanoTime()
        turnCommandStartedAtNanos = 0L
        turnAuthoritySeen = false
        virtualStick.setYawRate(0.0)
        flightLog.write("$TURN_LABEL 180 armed heading=$currentHeading")
        mainHandler.post(headingTurnTickRunnable)
    }

    private val headingTurnTickRunnable = object : Runnable {
        override fun run() {
            if (headingTurn == null) return
            driveHeadingTurn()
            if (headingTurn != null) mainHandler.postDelayed(this, TURN_TICK_MS)
        }
    }

    private fun driveHeadingTurn() {
        val turn = headingTurn ?: return
        if (!stickOwned) {
            finishHeadingTurn("$TURN_LABEL 180° 失去控制權，已停止", release = false)
            return
        }
        val ownsAuthority = stickStatus.authority == VirtualStickSession.MSDK_AUTHORITY_OWNER
        if (ownsAuthority) {
            turnAuthoritySeen = true
        } else if (turnAuthoritySeen) {
            finishHeadingTurn("遙控器已接管，$TURN_LABEL 180° 已停止", release = false)
            return
        } else {
            virtualStick.setYawRate(0.0)
            val waitedMs = (System.nanoTime() - turnStartedAtNanos) / 1_000_000L
            if (waitedMs > AUTHORITY_HANDOVER_TIMEOUT_MS) {
                finishHeadingTurn("未取得控制權，$TURN_LABEL 180° 已取消")
            } else {
                holdStatus = "等待控制權移交後$TURN_LABEL 180°…"
                render(holdStatus)
            }
            return
        }
        val now = System.nanoTime()
        if (turnCommandStartedAtNanos == 0L) turnCommandStartedAtNanos = now
        if (
            now - turnCommandStartedAtNanos > MAX_MOVING_HEADING_AGE_NANOS &&
            now - aircraftHeadingAtNanos > MAX_MOVING_HEADING_AGE_NANOS
        ) {
            finishHeadingTurn("機頭方向資料停止更新，旋轉已停止")
            return
        }
        if (now - turnStartedAtNanos > TURN_TIMEOUT_NANOS) {
            finishHeadingTurn(
                "$TURN_LABEL 180° 逾時，已轉 %.0f°".format(turn.progressDegrees),
            )
            return
        }
        val remaining = (turn.targetDegrees - turn.progressDegrees).coerceAtLeast(0.0)
        if (remaining <= TURN_TOLERANCE_DEGREES) {
            finishHeadingTurn(
                "$TURN_LABEL 180° 完成（%.0f°）".format(turn.progressDegrees),
                succeeded = true,
            )
            return
        }
        val speed = if (remaining <= TURN_SLOWDOWN_DEGREES) TURN_SLOW_SPEED_DPS else TURN_SPEED_DPS
        virtualStick.setYawRate(speed)
        holdStatus = "$TURN_LABEL 180°：已轉 %.0f°，剩 %.0f°".format(
            turn.progressDegrees,
            remaining,
        )
        render(holdStatus)
    }

    private fun finishHeadingTurn(
        message: String,
        succeeded: Boolean = false,
        release: Boolean = true,
    ) {
        if (headingTurn == null) return
        headingTurn = null
        turnCommandStartedAtNanos = 0L
        turnAuthoritySeen = false
        mainHandler.removeCallbacks(headingTurnTickRunnable)
        virtualStick.setYawRate(0.0)
        holdStatus = message
        flightLog.write(message)
        if (tapeEndpointTurn) {
            tapeEndpointTurn = false
            tapeDetector?.resetTracking()
            if (succeeded && tapeTracking.enabled && flying && stickOwned) {
                val now = System.nanoTime()
                tapeTracking.resumeAfterTurn(now)
                tapeTrackingStartedAtNanos = now
                tapeTrackingAuthoritySeen =
                    stickStatus.authority == VirtualStickSession.MSDK_AUTHORITY_OWNER
                renderedTapeTrackingPhase = TapeTrackingPhase.TURNING
                flightLog.write("tape endpoint turn complete; detection resumed")
                mainHandler.post(tapeTrackingTickRunnable)
                return
            }
            stopTapeTracking(
                "$message，黑膠帶追蹤已停止",
                release = release,
            )
            return
        }
        if (release && stickOwned && !leftStickActive && !rightStickActive && !holdingHeight) {
            releaseControlLink { error ->
                render(error?.let { "$message（釋放控制權失敗：$it）" } ?: message)
            }
        } else {
            render(message)
        }
    }

    private fun takeoff() {
        flightLog.write("press: takeoff (registered=$registered connected=$aircraftConnected flying=$flying)")
        if (lowCellVoltage || (batteryPercent?.let { it <= BATTERY_CRITICAL_PERCENT } == true)) {
            render("電量狀態危急，禁止起飛；請檢查或更換電池")
            return
        }
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
    private fun land(trigger: String = "operator") {
        flightLog.write(
            "landing requested trigger=$trigger connected=$aircraftConnected flying=$flying " +
                "owned=$stickOwned hold=$holdingHeight authority=${stickStatus.authority}",
        )
        if (!aircraftConnected) {
            render("飛機未連線，無法降落")
            return
        }
        if (holdingHeight) finishHeightHold("降落操作取消定高", release = false)
        if (headingTurn != null) finishHeadingTurn("降落操作取消 180° 旋轉", release = false)
        if (quarterArcController != null) {
            finishQuarterArc("降落操作取消無視覺 1/4 圈", release = false)
        }
        if (tapeTracking.enabled) {
            stopTapeTracking("降落操作取消黑膠帶追蹤", release = false)
        }
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
        if (registered && aircraftConnected && flying) return
        if (headingTurn != null) {
            finishHeadingTurn("飛行狀態結束，旋轉已停止", release = false)
        }
        if (quarterArcController != null) {
            finishQuarterArc("飛行狀態結束，無視覺 1/4 圈已停止", release = false)
        }
        if (tapeTracking.enabled) {
            stopTapeTracking("飛行狀態結束，黑膠帶追蹤已停止", release = false)
        }
        if (!stickOwned || stickTransitionPending) return
        releaseControlLink { error ->
            render(error?.let { "釋放控制權失敗：$it" } ?: "已釋放控制權，交回遙控器")
        }
    }

    private fun releaseControlLink(onDone: (String?) -> Unit) {
        if (stickTransitionPending) {
            onDone("控制權切換進行中")
            return
        }
        if (tapeTracking.enabled) {
            stopTapeTracking("控制權釋放，黑膠帶追蹤已停止", release = false)
        }
        if (quarterArcController != null) {
            finishQuarterArc("控制權釋放，無視覺 1/4 圈已停止", release = false)
        }
        val generation = beginTransition("release") {
            onDone("釋放控制權無回應")
        }
        heightTargetMeters = null
        mainHandler.removeCallbacks(holdTickRunnable)
        virtualStick.setClimbRate(0.0)
        holdStatus = ""
        if (headingTurn != null) {
            headingTurn = null
            turnAuthoritySeen = false
            turnCommandStartedAtNanos = 0L
            mainHandler.removeCallbacks(headingTurnTickRunnable)
            virtualStick.setYawRate(0.0)
        }
        virtualStick.disable { error ->
            if (!endTransition(generation)) {
                if (error == null) stickOwned = false
                flightLog.write("late release callback ignored generation=$generation error=$error")
                return@disable
            }
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
    private fun acquireControlLink(
        onFailure: ((String) -> Unit)? = null,
        onOwned: () -> Unit,
    ) {
        if (stickOwned) {
            onOwned()
            return
        }
        if (!registered || !aircraftConnected || !flying) {
            val reason = "飛機未在空中，不取得控制權"
            flightLog.write("acquire refused: registered=$registered connected=$aircraftConnected flying=$flying")
            render(reason)
            onFailure?.invoke(reason)
            return
        }
        if (stickTransitionPending) {
            // Never silent: a swallowed press is indistinguishable from a dead
            // button, which is exactly how this looked to the operator.
            val reason = "控制權切換進行中，請稍候再試"
            flightLog.write("acquire ignored: transition already pending")
            render(reason)
            onFailure?.invoke(reason)
            return
        }
        val generation = beginTransition("acquire") {
            acquireAttempts = 0
            onFailure?.invoke("控制權切換無回應")
        }
        landingRequested = false
        acquireAttempts += 1
        val attempt = acquireAttempts
        virtualStick.enable { error ->
            if (!endTransition(generation)) {
                flightLog.write("late acquire callback ignored generation=$generation error=$error")
                if (error == null) virtualStick.disable {}
                return@enable
            }
            stickOwned = error == null
            flightLog.write("acquire attempt=$attempt owned=$stickOwned error=$error")
            when {
                stickOwned -> {
                    acquireAttempts = 0
                    onOwned()
                }
                attempt < MAX_ACQUIRE_ATTEMPTS && flying -> {
                    render("等待起飛完成才能取得控制權（第 $attempt 次）：$error")
                    mainHandler.postDelayed(
                        { acquireControlLink(onFailure, onOwned) },
                        ACQUIRE_RETRY_MS,
                    )
                }
                else -> {
                    val reason = "取得控制權失敗 $attempt 次，請用遙控器操作：$error"
                    acquireAttempts = 0
                    render(reason)
                    onFailure?.invoke(reason)
                }
            }
        }
    }

    /**
     * A control-link call that never calls back would latch [stickTransitionPending]
     * forever, and every later button press would be dropped — the app would look
     * dead while takeoff and landing still worked, because those do not need the
     * link. The latch therefore has a deadline.
     */
    private fun beginTransition(what: String, onTimeout: (() -> Unit)? = null): Long {
        stickTransitionPending = true
        transitionTimeoutAction = onTimeout
        transitionGeneration += 1
        flightLog.write("transition begin: $what generation=$transitionGeneration")
        mainHandler.postDelayed(transitionTimeoutRunnable, TRANSITION_TIMEOUT_MS)
        return transitionGeneration
    }

    private fun endTransition(generation: Long): Boolean {
        if (!stickTransitionPending || generation != transitionGeneration) return false
        stickTransitionPending = false
        transitionTimeoutAction = null
        mainHandler.removeCallbacks(transitionTimeoutRunnable)
        return true
    }

    private val transitionTimeoutRunnable = Runnable {
        if (!stickTransitionPending) return@Runnable
        stickTransitionPending = false
        val onTimeout = transitionTimeoutAction
        transitionTimeoutAction = null
        flightLog.write("transition timed out; latch cleared")
        render("控制權切換無回應，已解除鎖定，可再試一次")
        onTimeout?.invoke()
    }

    // ------------------------------------------------------------- battery ----

    /**
     * At 13% the app requests the same verified landing sequence as the landing
     * button. The gate emits once per flight; retries and low-altitude landing
     * confirmation remain owned by the existing landing state machine.
     */
    private fun evaluateBattery() {
        val percent = batteryPercent
        if (
            batteryLandingGate.evaluate(
                percent = percent,
                flying = flying,
                connected = aircraftConnected,
                landingRequested = landingRequested,
            )
        ) {
            flightLog.write("battery forced landing threshold reached percent=$percent")
            render("電量 $percent%：強制啟動自動降落")
            land(trigger = "battery=$percent%")
            return
        }
        when {
            lowCellVoltage ->
                render("電芯電壓過低：請立即使用實體遙控器降落")
            percent != null && percent <= BATTERY_CRITICAL_PERCENT ->
                render("電量 $percent%：自動降落已啟動")
            percent != null && percent <= BATTERY_WARN_PERCENT ->
                render("電量 $percent%：接近低電量，請準備降落")
        }
    }




    /**
     * Pressing either height button takes the control link and closes a bounded
     * vertical loop around that button's target. The same path safely supports
     * ascent and descent; no fixed-duration movement is inferred from altitude.
     */
    private fun startHeightHold(targetHeightMeters: Double) {
        flightLog.write(
            "press: height target=$targetHeightMeters (flying=$flying owned=$stickOwned " +
                "pending=$stickTransitionPending height=$altitudeMeters src=$heightSource " +
                "ageMs=${heightAgeMillis()})",
        )
        if (!flying) {
            render("飛機不在空中，無法調整高度")
            return
        }
        if (headingTurn != null) {
            render("180° 旋轉進行中，無法同時調整高度")
            return
        }
        if (quarterArcController != null) {
            render("無視覺 1/4 圈進行中，無法同時調整高度")
            return
        }
        if (tapeTracking.enabled) {
            render("黑膠帶追蹤進行中，無法同時調整高度")
            return
        }
        if (usableHeightMeters() == null) {
            flightLog.write(
                "height target refused: no usable height last=$altitudeMeters " +
                    "src=$heightSource connected=$aircraftConnected ageMs=${heightAgeMillis()}",
            )
            render("沒有高度資料，無法調整高度")
            return
        }
        render("取得控制權後調整至 %.1f m…".format(targetHeightMeters))
        acquireControlLink {
            if (usableHeightMeters() == null) {
                finishHeightHold("沒有高度資料，已放棄調整高度並交回遙控器")
                return@acquireControlLink
            }
            heightTargetMeters = targetHeightMeters
            holdStartedAtNanos = System.nanoTime()
            holdStableSamples = 0
            holdAuthoritySeen = false
            flightLog.write("height target armed target=$targetHeightMeters")
            mainHandler.post(holdTickRunnable)
        }
    }

    /**
     * Ends the manoeuvre and, by default, hands the aircraft back. Holding the
     * control link after reaching the target has no purpose: with the sticks
     * neutral the aircraft keeps the height on its own, and an idle link is
     * exactly what stopped the RC from taking over on 2026-08-17.
     */
    private fun finishHeightHold(message: String, release: Boolean = true) {
        heightTargetMeters = null
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
        val targetHeightMeters = heightTargetMeters ?: return
        // Two different things look identical in one sample: the aircraft has not
        // finished handing authority over yet, and the RC has taken it away. They
        // are told apart by history — a takeover can only happen after this app
        // has actually held authority. enableVirtualStick returns ~60 ms before the
        // aircraft names MSDK as the owner, and treating that gap as a takeover is
        // what made the original height manoeuvre stop 7 ms after it armed (2026-08-17).
        val ownsAuthority = stickStatus.authority == VirtualStickSession.MSDK_AUTHORITY_OWNER
        if (ownsAuthority) {
            holdAuthoritySeen = true
        } else if (holdAuthoritySeen) {
            flightLog.write("height target stopped: authority taken by ${stickStatus.authority}")
            finishHeightHold("遙控器已接管，高度調整停止", release = false)
            return
        } else {
            virtualStick.setClimbRate(0.0)
            val waitingMs = (System.nanoTime() - holdStartedAtNanos) / 1_000_000L
            if (waitingMs > AUTHORITY_HANDOVER_TIMEOUT_MS) {
                flightLog.write("height target aborted: authority never arrived (owner=${stickStatus.authority})")
                finishHeightHold("未取得控制權（owner=${stickStatus.authority}），已放棄調整高度")
            } else {
                holdStatus = "等待控制權移交…（owner=${stickStatus.authority}）"
                render(holdStatus)
            }
            return
        }
        val height = usableHeightMeters()
        if (height == null) {
            flightLog.write("height target aborted: height unusable ageMs=${heightAgeMillis()}")
            finishHeightHold("高度資料中斷，已停止升降並交回遙控器")
            return
        }
        if (System.nanoTime() - holdStartedAtNanos > HOLD_TIMEOUT_NANOS) {
            flightLog.write("height target aborted: timeout h=%.2f".format(height))
            finishHeightHold("高度調整逾時（目前 %.2f m），已交回遙控器".format(height))
            return
        }
        val error = targetHeightMeters - height
        if (HeightHoldPolicy.isWithinTarget(targetHeightMeters, height)) {
            virtualStick.setClimbRate(0.0)
            holdStableSamples += 1
            // A few consecutive in-band samples, not one, decide arrival: a single
            // sample can be noise on the way past the target.
            if (holdStableSamples >= HOLD_STABLE_SAMPLES) {
                flightLog.write(
                    "height target complete target=%.1f h=%.2f; releasing"
                        .format(targetHeightMeters, height),
                )
                finishHeightHold("已到 %.2f m，控制權交回遙控器".format(height))
            } else {
                holdStatus = "定高 %.1f m：確認中（目前 %.2f m）"
                    .format(targetHeightMeters, height)
                render(holdStatus)
            }
            return
        }
        holdStableSamples = 0
        // Commanding vertical motion while the height has stopped updating is
        // flying blind: the aircraft should be moving, so samples should arrive.
        // The check waits out a grace window because a hover legitimately starts
        // with an old sample and the first frames need time to produce motion.
        val movingForMs = (System.nanoTime() - holdStartedAtNanos) / 1_000_000L
        if (movingForMs > MAX_MOVING_HEIGHT_AGE_MS && heightAgeMillis() > MAX_MOVING_HEIGHT_AGE_MS) {
            flightLog.write(
                "height target aborted: height not updating ageMs=${heightAgeMillis()} " +
                    "h=%.2f".format(height),
            )
            finishHeightHold("高度停止更新，已停止升降並交回遙控器")
            return
        }
        val climbRate = HeightHoldPolicy.climbRateMetersPerSecond(
            targetHeightMeters = targetHeightMeters,
            currentHeightMeters = height,
            maximumRateMetersPerSecond = VirtualStickSession.MAX_VERTICAL_MPS,
        )
        virtualStick.setClimbRate(climbRate)
        flightLog.write(
            "height target=%.1f h=%.2f e=%+.2f cmd=%+.2f age=${heightAgeMillis()}"
                .format(targetHeightMeters, height, error, climbRate),
        )
        val direction = if (climbRate > 0.0) "上升" else "下降"
        holdStatus = "$direction 至 %.1f m：目前 %.2f m，垂直 %+.2f m/s"
            .format(targetHeightMeters, height, climbRate)
        render(holdStatus)
    }

    private companion object {
        const val TAG = "LiteMainActivity"

        /** Pill fill: the accent colour at ~12% opacity, matching the dashboard panels. */
        const val PILL_FILL_ALPHA = 0x1F000000

        const val MAX_CONFIRM_ATTEMPTS = 8
        const val CONFIRM_RETRY_MS = 700L

        /** Existing low-altitude target retained for comparison experiments. */
        const val LOW_HEIGHT_TARGET_METERS = 0.7

        /** Height of the successful three-lap flight, rounded to its operating target. */
        const val ONE_METER_TARGET_HEIGHT_METERS = 1.0

        /**
         * While vertical motion is commanded, height must keep arriving: the
         * aircraft is moving, so samples must move too. Silence longer than this
         * means the loop is blind and must stop instead of guessing.
         */
        const val MAX_MOVING_HEIGHT_AGE_MS = 2_000L

        /** Height-loop period; 10 Hz is half the frame rate the aircraft is fed. */
        const val HOLD_TICK_MS = 100L

        /** Consecutive in-band samples that count as "arrived" (~0.3 s at 10 Hz). */
        const val HOLD_STABLE_SAMPLES = 3

        /** A height manoeuvre that has not converged by now is abandoned. */
        const val HOLD_TIMEOUT_NANOS = 20_000_000_000L

        /** Gentle closed-loop half-turn, slowed near the target to limit overshoot. */
        const val TURN_LABEL = "右旋"
        const val TURN_SPEED_DPS = 20.0
        const val TURN_SLOW_SPEED_DPS = 8.0
        const val TURN_SLOWDOWN_DEGREES = 25.0
        const val TURN_TOLERANCE_DEGREES = 2.0
        const val TURN_TICK_MS = 50L
        const val TURN_TIMEOUT_NANOS = 15_000_000_000L
        const val MAX_MOVING_HEADING_AGE_NANOS = 1_000_000_000L

        /** Fixed-radius quarter-arc experiment gates and closed-heading endpoint. */
        const val QUARTER_ARC_START_MIN_HEIGHT_METERS = 0.55
        const val QUARTER_ARC_START_MAX_HEIGHT_METERS = 0.85
        const val QUARTER_ARC_FLIGHT_MIN_HEIGHT_METERS = 0.45
        const val QUARTER_ARC_FLIGHT_MAX_HEIGHT_METERS = 0.95
        const val QUARTER_ARC_TOLERANCE_DEGREES = 2.0
        const val QUARTER_ARC_TICK_MS = 50L
        const val QUARTER_ARC_TIMEOUT_NANOS = 20_000_000_000L
        const val QUARTER_ARC_RENDER_PERIOD_NANOS = 250_000_000L


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
        const val AUTHORITY_HANDOVER_TIMEOUT_NANOS =
            AUTHORITY_HANDOVER_TIMEOUT_MS * 1_000_000L

        /**
         * Display and takeoff-lockout thresholds only. They never initiate a
         * manoeuvre; DJI's own battery protection remains authoritative.
         */
        const val BATTERY_WARN_PERCENT = 25
        const val BATTERY_CRITICAL_PERCENT = BatteryLandingGate.FORCE_LANDING_PERCENT

        const val HORIZONTAL_WATCHDOG_MS = 100L



        /** Obstacle distances are recorded at most this often. */
        const val OBSTACLE_LOG_PERIOD_NANOS = 1_000_000_000L

        /** Live sensor numbers are readable at 4 Hz without redrawing every callback. */
        const val OBSTACLE_TELEMETRY_PERIOD_NANOS = 250_000_000L

        /** Detector state is logged at one hertz; the overlay still updates at frame cadence. */
        const val TAPE_LOG_PERIOD_NANOS = 1_000_000_000L

        /** Preserve the original ~750 ms overlay debounce at the 10 Hz detector rate. */
        const val TAPE_MISSES_TO_CLEAR = 8
        const val TAPE_TRACKING_TICK_MS = 100L
        const val TAPE_CAPTURE_DIRECTORY = "tape-captures"
        const val CAMERA_DOWN_PITCH_DEGREES = -90.0
        /** Absorbs floating-point noise in a confirmed pitch, never a deliberate tilt. */
        const val CAMERA_DOWN_PITCH_TOLERANCE_DEGREES = 0.5
        const val CAMERA_RECENTER_DURATION_SECONDS = 2.0
        // MSDK is given two seconds to rotate; one extra second allows callback delivery.
        const val CAMERA_PITCH_COMMAND_TIMEOUT_MS = 3_000L
        const val TAPE_COMMAND_LOG_PERIOD_NANOS = 250_000_000L

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
