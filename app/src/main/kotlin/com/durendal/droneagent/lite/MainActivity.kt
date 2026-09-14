package com.durendal.droneagent.lite

import android.app.Activity
import android.app.AlertDialog
import android.graphics.Color
import android.graphics.drawable.GradientDrawable
import android.os.Bundle
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.util.TypedValue
import android.view.Gravity
import android.view.View
import android.view.ViewGroup
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.HorizontalScrollView
import android.widget.TextView
import java.util.Locale
import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.FlightAssistantKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.key.RemoteControllerKey
import dji.sdk.keyvalue.value.common.Attitude
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.common.Velocity3D
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
import kotlin.math.atan2
import kotlin.math.hypot

internal fun mathematicalCircleArmedLogMessage(
    mode: String,
    diameterMeters: Double,
    secondsPerLap: Double,
    laps: Int,
    speedMetersPerSecond: Double,
    angularRateDegreesPerSecond: Double,
    headingDegrees: Double,
): String =
    (
        "mathematical circle armed mode=$mode diameter=%.2f secondsPerLap=%.1f " +
            "laps=%d speed=%.3f angularRate=%.1f heading=%.1f"
    ).format(
        diameterMeters,
        secondsPerLap,
        laps,
        speedMetersPerSecond,
        angularRateDegreesPerSecond,
        headingDegrees,
    )


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
    private enum class HorizontalPulseExperiment {
        HARDWARE_LATENCY,
        STRAIGHT_REHEARSAL,
        STRAIGHT_MEASUREMENT,
    }


    private lateinit var preview: CameraPreview
    private lateinit var tapeOverlay: TapeOverlayView
    private var tapeDetector: BlackTapeDetector? = null
    private var tapeDetected = false
    private var tapeLoggedAtNanos = 0L
    private var consecutiveTapeMisses = 0
    private lateinit var flightLog: FlightLog
    private lateinit var headlineView: TextView
    private lateinit var detailView: TextView
    private lateinit var firmwareView: TextView
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
    private lateinit var angleCircularTapeTrackingButton: PillButton
    private lateinit var fixedHeadingLowSpeedButton: PillButton
    private lateinit var fixedHeadingFourteenPhaseLeadButton: PillButton
    private lateinit var fixedHeadingFasterPhaseLeadButton: PillButton
    private lateinit var fixedHeadingCurvatureFeedforwardButton: PillButton
    private lateinit var fixedHeadingFastCruiseButton: PillButton
    private lateinit var fixedHeadingScheduledActuationButton: PillButton
    private lateinit var fixedHeadingSpeedTargetButton: PillButton
    private lateinit var yawLapTestButton: PillButton
    private lateinit var hardwareLatencyButton: PillButton
    private lateinit var straightRehearsalButton: PillButton
    private lateinit var straightMeasurementButton: PillButton
    private lateinit var mathematicalSkatingCircleButton: PillButton
    private lateinit var mathematicalRacingCircleButton: PillButton
    private lateinit var mathematicalFastRacingCircleButton: PillButton
    private lateinit var tiltStraightButton: PillButton
    private lateinit var tiltCircleButton: PillButton
    private lateinit var tiltSkatingCircleButton: PillButton
    private lateinit var virtualStickFrameRateButton: PillButton
    private lateinit var leftPad: StickPadView
    private lateinit var rightPad: StickPadView
    private lateinit var gimbalPad: StickPadView
    private lateinit var hardwareLatencySyncMarker: TextView

    private var registered = false
    private var registerAttempts = 0
    private var aircraftConnected = false
    private var flying = false
    private var landingRequested = false
    private var landingAuthorityWaitPending = false
    private var confirmationNeeded = false
    private var confirmAttempts = 0

    private class FirmwareReadout(
        val key: DJIKey<String>,
        val logName: String,
    ) {
        var connected = false
        var generation = 0L
        var status = "未連線"
    }

    private val aircraftFirmware = FirmwareReadout(
        KeyTools.createKey(ProductKey.KeyFirmwareVersion),
        "aircraftFirmwareVersion",
    )
    private val remoteControllerFirmware = FirmwareReadout(
        KeyTools.createKey(RemoteControllerKey.KeyFirmwareVersion),
        "remoteControllerFirmwareVersion",
    )

    /** Height when the landing command was issued, to prove the aircraft descended. */
    private var landingHeightAtCommand: Double? = null
    private var stickStatus = VirtualStickStatus()
    private var lastStickStatusAtNanos = 0L
    private var lastReleaseRequestedAtNanos = 0L

    /** True once MSDK accepted enableVirtualStick; the app then owns the control link. */
    private var stickOwned = false
    private var stickTransitionPending = false
    private var acquireAttempts = 0
    private var transitionTimeoutAction: (() -> Unit)? = null
    private var transitionGeneration = 0L
    private var pendingAcquireCallbacks = 0

    /** Ground-relative height and when it was observed; null until the first sample. */
    private var altitudeMeters: Double? = null
    private var altitudeAtNanos = 0L
    private var heightSource = HeightSource.NONE

    private var obstacleLoggedAtNanos = 0L
    private var obstacleTelemetryRenderedAtNanos = 0L
    private var obstaclePreviousCallbackAtNanos = 0L
    private var obstacleCallbacksSinceLog = 0
    private var obstacleMaximumCallbackGapMsSinceLog = 0L

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
    @Volatile private var aircraftHeadingDegrees: Double? = null
    @Volatile private var aircraftHeadingAtNanos = 0L
    private var aircraftPitchDegrees: Double? = null
    private var aircraftRollDegrees: Double? = null
    private var flightModeName = "UNKNOWN"
    private var fcFlightModeName = "UNKNOWN"
    private var remoteControllerFlightModeName = "UNKNOWN"
    private var remoteControllerSwitchModeName = "UNKNOWN"
    private var flightLimitSpeedMetersPerSecond: Double? = null
    private var visionSensorUsed: Boolean? = null
    private var visionPositionUnavailable: Boolean? = null
    private var visionAvoidanceEnabled: Boolean? = null
    private var fixedHeadingReferenceDegrees: Double? = null
    private val aircraftVelocityKey =
        KeyTools.createKey(FlightControllerKey.KeyAircraftVelocity)
    /** Earth-frame horizontal velocity used to verify and disambiguate fixed-heading travel. */
    private val aircraftVelocityLock = Any()
    @Volatile private var aircraftVelocityX = 0.0
    @Volatile private var aircraftVelocityY = 0.0
    private var aircraftVelocityZ = 0.0
    @Volatile private var aircraftVelocityAtNanos = 0L
    @Volatile private var groundSpeedMetersPerSecond = 0.0
    private var headingTurn: HeadingTurn? = null
    private var turnStartedAtNanos = 0L
    private var turnCommandStartedAtNanos = 0L
    private var turnAuthoritySeen = false
    private var yawLapTestActive = false
    private var turnLastRenderedAtNanos = 0L
    private var turnLastMeasuredAtNanos = 0L
    private var turnLastMeasuredProgressDegrees = 0.0

    /** Camera-independent 0.75 m radius clockwise quarter-arc experiment. */
    private var quarterArcController: QuarterArcController? = null
    private var quarterArcStartedAtNanos = 0L
    private var quarterArcAuthoritySeen = false
    private var quarterArcLastRenderedAtNanos = 0L

    /** Camera-visible velocity experiments; only one may own these shared controls. */
    private var hardwareLatencyPulseSequence: HorizontalPulseSequence? = null
    private var hardwareLatencyStartPending = false
    private var hardwareLatencyAuthoritySeen = false
    private var hardwareLatencyArmedAtNanos = 0L
    private var hardwareLatencyStartedAtNanos = 0L
    private var horizontalPulseExperiment: HorizontalPulseExperiment? = null
    private var horizontalPulseGeneration = 0L
    private data class StraightLineTestConfig(
        val direction: StraightLineDirection,
        val speedMetersPerSecond: Double,
        val maximumCruiseSeconds: Int,
    )

    private var straightLineTestConfig: StraightLineTestConfig? = null
    private var straightLineInitialHeading: Double? = null
    private var straightLineAttemptId: String? = null
    private var straightLineLastSampleAtNanos = 0L
    private var straightLineReleasePending = false
    private var straightLineTracePending = false
    @Volatile private var visualVelocityDiagnostics: VisualVelocityDiagnostics? = null
    @Volatile private var visualVelocityFrameContext: VisualVelocityDiagnostics.FrameContext? = null
    private var fixedHeadingVisualVelocityAttemptId: String? = null
    private var activityForeground = false

    private val velocityReadDiagnostics = VelocityReadDiagnostics(
        readVelocity = { complete ->
            KeyManager.getInstance().getValue(
                aircraftVelocityKey,
                object : CommonCallbacks.CompletionCallbackWithParam<Velocity3D> {
                    override fun onSuccess(result: Velocity3D) {
                        complete(VelocityReadDiagnostics.Velocity(result.x, result.y, result.z), null)
                    }

                    override fun onFailure(error: IDJIError) {
                        complete(null, error.toString())
                    }
                },
            )
        },
        listenerSnapshot = {
            synchronized(aircraftVelocityLock) {
                if (aircraftVelocityAtNanos == 0L) null else VelocityReadDiagnostics.ListenerSample(
                    aircraftVelocityX, aircraftVelocityY, aircraftVelocityZ, aircraftVelocityAtNanos,
                )
            }
        },
        record = { event, atNanos, details ->
            flightProfiler.record(event = event, atNanos = atNanos, details = details)
        },
    )
    private val velocityReadTickRunnable = object : Runnable {
        override fun run() {
            if (activityDestroying || !registered || !aircraftConnected || !isStraightLineTest() ||
                (!hardwareLatencyStartPending && hardwareLatencyPulseSequence == null)
            ) {
                velocityReadDiagnostics.stop()
                if (isStraightLineTest()) {
                    straightLineAttemptId?.let { visualVelocityDiagnostics?.stop(it) }
                    refreshVisualVelocityContext()
                }
                return
            }
            velocityReadDiagnostics.tick()
            mainHandler.postDelayed(this, VelocityReadDiagnostics.INTERVAL_MS)
        }
    }

    /** Main-thread telemetry snapshot; image workers never read mutable flight state. */
    private fun refreshVisualVelocityContext() {
        if (visualVelocityDiagnostics?.isActive() != true) {
            visualVelocityFrameContext = null
            return
        }
        val listener = synchronized(aircraftVelocityLock) {
            if (aircraftVelocityAtNanos == 0L) null else VelocityReadDiagnostics.ListenerSample(
                aircraftVelocityX, aircraftVelocityY, aircraftVelocityZ, aircraftVelocityAtNanos,
            )
        }
        visualVelocityFrameContext = VisualVelocityDiagnostics.FrameContext(
            heightMeters = usableHeightMeters(),
            heightReceivedAtNanos = altitudeAtNanos,
            heightSource = heightSource.name,
            cameraPitchCommandDegrees = selectedCameraPitchDegrees,
            aircraftHeadingDegrees = aircraftHeadingDegrees,
            listener = listener,
            capturedAtNanos = System.nanoTime(),
            aircraftHeadingReceivedAtNanos = aircraftHeadingAtNanos,
            cameraPitchCommandPending = cameraPitchCommandPending,
        )
    }

    private fun submitVisualVelocityFrame(rgba: org.opencv.core.Mat, frameNanos: Long) {
        val diagnostic = visualVelocityDiagnostics ?: return
        val context = visualVelocityFrameContext ?: return
        try {
            diagnostic.submitRgba(rgba, frameNanos, context)
        } catch (error: Exception) {
            // A diagnostic failure must not abort the existing tape-detection path.
            flightProfiler.record(
                event = "visual_velocity_error",
                frameNanos = frameNanos,
                details = profileDetails("stage" to "submission", "error" to error, "controlFeedback" to false),
            )
        }
    }

    /** Camera- and position-independent circle generated solely from elapsed time. */
    @Volatile private var mathematicalCircleController: MathematicalCircleController? = null
    @Volatile private var mathematicalCircleStartPending = false
    @Volatile private var mathematicalCircleAuthoritySeen = false
    @Volatile private var mathematicalCircleMode = MathematicalCircleMode.SKATING
    private var mathematicalCircleArmedAtNanos = 0L
    private var mathematicalCircleStartedAtNanos = 0L
    private var mathematicalCircleInitialHeadingDegrees = 0.0
    private var mathematicalCircleLastLap = 0
    private var mathematicalCircleLastRenderedAtNanos = 0L
    private var mathematicalCircleLastTickAtNanos = 0L

    private class TiltFlightRun(val mode: TiltFlightMode) {
        val controller = TiltFlightController(mode)
        var armedAtNanos = 0L
        var startedAtNanos = 0L
        var lastTickAtNanos = 0L
        var lastRenderedAtNanos = 0L
        var lastPhase: TiltFlightPhase? = null
        @Volatile var authoritySeen = false
        @Volatile var stopping = false
    }

    @Volatile private var tiltFlightRun: TiltFlightRun? = null


    /** Camera gimbal is independent of the aircraft's virtual-stick authority. */
    private var gimbalActive = false
    private var selectedCameraPitchDegrees: Double? = null
    private var captureRecorder: TapeCaptureRecorder? = null
    private var captureAutoArmedByTracking = false
    private lateinit var flightProfiler: FlightProfiler
    private lateinit var flightProfileSessionId: String
    @Volatile private var latestProfiledFrameNanos = 0L
    @Volatile private var latestVisionCompletedAtNanos = 0L
    private var latestDetectionUiFrameNanos = 0L
    private var latestDetectionUiAtNanos = 0L
    private var lastControlledProfileFrameNanos = 0L
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
    private enum class CircularYawControlMode { RATE, HEADING }
    private enum class MathematicalCircleMode(
        val displayName: String,
        val secondsPerLap: Double = MathematicalCircleController.SECONDS_PER_LAP,
    ) {
        SKATING("滑冰"),
        RACING("賽車"),
        RACING_FAST("賽車快版", 7.0),
    }
    private enum class TapeControlTrigger { PERIODIC, FRESH_VISION }
    private var activeTapeControlTrigger = TapeControlTrigger.PERIODIC

    private val tapeTracking = TapeTrackingController()
    private val angleHeadingController = AngleHeadingController()
    private var tapeTrackingAuthoritySeen = false
    private var tapeTrackingStartedAtNanos = 0L
    @Volatile private var cameraFrameStreamStaleAtNanos = 0L
    private var tapeTrackingStartPending = false
    @Volatile private var activityDestroying = false
    private var renderedTapeTrackingPhase = TapeTrackingPhase.DISABLED
    private var commandedTapeYawRate = 0.0
    private var commandedTapeForwardSpeed = 0.0
    private var commandedTapeRightSpeed = 0.0
    private var tapeEndpointTurn = false
    private var tapeCommandLoggedAtNanos = 0L
    private var activeTapeTrackingMode: TapeTrackingMode? = null
    private var activeCircularTrackingSpeed = CircularTrackingSpeed.FAST
    private var activeCircularYawControlMode = CircularYawControlMode.RATE
    private var activeFixedHeadingActuationPhaseLead =
        FixedHeadingActuationPhaseLead.DEGREES_0
    private var selectedFixedHeadingSpeedTarget = FixedHeadingSpeedTarget.STEP_075
    private var activeFixedHeadingSpeedTarget = FixedHeadingSpeedTarget.BASELINE


    private val diagnosticTurnCycleTimer =
        DiagnosticTurnCycleTimer(
            minimumDistanceMeters = MIN_DIAGNOSTIC_TURN_CYCLE_DISTANCE_METERS,
        )

    /** Raw KeyUltrasonicHeight, logged for unit identification; never drives the loop yet. */
    private var ultrasonicRaw: Int? = null

    /** Target for the active one-shot height manoeuvre; null while inactive. */
    private var heightTargetMeters: Double? = null
    private val holdingHeight: Boolean get() = heightTargetMeters != null
    private var holdStartedAtNanos = 0L
    private var holdStableSamples = 0

    /** True once enabled MSDK/UNKNOWN authority was observed during this manoeuvre. */
    private var holdAuthoritySeen = false


    /** Whether a finger is currently deflecting each stick. */
    private var leftStickActive = false
    private var rightStickActive = false

    /** Latest height-loop line, kept out of [detailView] so nothing overwrites it. */
    private var holdStatus = ""

    private val mainHandler = Handler(Looper.getMainLooper())
    private val autoCaptureDisarmRunnable = Runnable {
        val recorder = captureRecorder
        if (!captureAutoArmedByTracking || tapeTracking.enabled || recorder == null) {
            return@Runnable
        }
        recorder.disarm()
        captureAutoArmedByTracking = false
        if (::captureButton.isInitialized) captureButton.text = "錄製影格證據"
        flightLog.write(
            "automatic tape capture stopped saved=${recorder.savedCaptureCount} " +
                "dropped=${recorder.droppedCaptureCount} failed=${recorder.failedCaptureCount}",
        )
    }
    private val virtualStick = VirtualStickSession(
        onStatus = { status -> handleVirtualStickStatus(status, System.nanoTime()) },
        onFrameSummary = { summary -> flightLog.write(summary) },
        onFrameSent = ::recordVirtualStickFrame,
        onBeforeFrame = ::driveTiltFlight,
    )

    private fun handleVirtualStickStatus(status: VirtualStickStatus, receivedAtNanos: Long) = runOnUiThread {
        if (receivedAtNanos < lastStickStatusAtNanos) return@runOnUiThread
        lastStickStatusAtNanos = receivedAtNanos
        val previous = stickStatus
        stickStatus = status
        if (previous.enabled != status.enabled || previous.authority != status.authority) {
            flightLog.write("stick state enabled=${status.enabled} authority=${status.authority}")
        }
        if (tapeTracking.enabled) {
            if (status.hasMsdkAuthority) {
                tapeTrackingAuthoritySeen = true
            } else if (tapeTrackingAuthoritySeen) {
                stopTapeTracking("實體遙控器已接管，黑膠帶追蹤已停止", release = false)
            }
        }
        if (headingTurn != null) {
            driveHeadingTurn()
        }
        if (quarterArcController != null) {
            if (status.hasMsdkAuthority) {
                quarterArcAuthoritySeen = true
                driveQuarterArc()
            } else if (quarterArcAuthoritySeen) {
                finishQuarterArc("實體遙控器已接管，無視覺 1/4 圈已停止", release = false)
            }
        }
        if (
            hardwareLatencyPulseSequence != null &&
            !status.hasMsdkAuthority &&
            hardwareLatencyAuthoritySeen
        ) {
            stopHardwareLatencyTest(
                "實體遙控器已接管，${activeHorizontalPulseName()}已停止",
                release = false,
            )
        }
        if (
            mathematicalCircleController != null &&
            !status.hasMsdkAuthority &&
            mathematicalCircleAuthoritySeen
        ) {
            stopMathematicalCircle("實體遙控器已接管，數學圓周已停止", release = false)
        }
        tiltFlightRun?.let { run ->
            if (run.authoritySeen &&
                (!status.hasMsdkAuthority || !status.advancedMode)
            ) {
                stopTiltFlight("控制權已離開 MSDK，傾角實驗已停止", release = false)
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
        flightProfileSessionId = "${System.currentTimeMillis()}-${System.nanoTime()}"
        flightProfiler =
            FlightProfiler(
                File(
                    getExternalFilesDir(null),
                    "$FLIGHT_PROFILE_FILE_PREFIX$flightProfileSessionId$FLIGHT_PROFILE_FILE_SUFFIX",
                ),
            )
        flightProfiler.record(
            event = "session_start",
            atNanos = System.nanoTime(),
            details = profileDetails(
                "sessionId" to flightProfileSessionId,
                "wallTimeMillis" to System.currentTimeMillis(),
                "timestampMeaning" to "app_monotonic_receive_not_camera_exposure",
                "device" to "${Build.MANUFACTURER}/${Build.MODEL}",
                "android" to Build.VERSION.RELEASE,
                "sdk" to Build.VERSION.SDK_INT,
                "abis" to Build.SUPPORTED_ABIS.joinToString(","),
                "msdk" to "5.18.0",
                "opencv" to "4.14.0",
                "visionHz" to 20,
                "controlHz" to (1_000L / TAPE_TRACKING_TICK_MS),
                "stickHz" to virtualStick.frameRate().hertz,
                "imageResizeInterpolation" to TAPE_IMAGE_RESIZE_INTERPOLATION,
            ),
        )
        captureRecorder = runCatching {
            TapeCaptureRecorder(
                root = File(getExternalFilesDir(null), TAPE_CAPTURE_DIRECTORY),
                log = flightLog::write,
            )
        }.onFailure { error ->
            flightLog.write("tape capture unavailable: $error")
        }.getOrNull()
        visualVelocityDiagnostics = runCatching {
            VisualVelocityDiagnostics { event, atNanos, details ->
                flightProfiler.record(event = event, atNanos = atNanos, details = details)
            }
        }.onFailure { error ->
            flightLog.write("visual velocity diagnostics unavailable: $error")
        }.getOrNull()
        tapeDetector = runCatching {
            BlackTapeDetector(
                onResult = ::handleTapeDetection,
                onFrameProfile = ::recordTapeFrameProfile,
                onError = { error ->
                    runOnUiThread { flightLog.write("OpenCV detection failed: $error") }
                },
                captureRecorder = captureRecorder,
                captureFlightContext = ::tapeCaptureFlightContext,
                onRgbaFrameReady = ::submitVisualVelocityFrame,
            )
        }.onFailure { error ->
            flightLog.write("OpenCV initialization failed: $error")
        }.getOrNull()
        setContentView(buildUi())
        flightLog.write("app start; log=${flightLog.path}")
        flightLog.write(
            "profiling start session=$flightProfileSessionId trace=${flightProfiler.path}",
        )
        render("啟動 MSDK…")
        startSdk()
        mainHandler.post(horizontalSafetyWatchdog)
    }

    override fun onResume() {
        super.onResume()
        activityForeground = true
    }

    override fun onPause() {
        activityForeground = false
        mainHandler.removeCallbacks(velocityReadTickRunnable)
        velocityReadDiagnostics.stop()
        visualVelocityDiagnostics?.stop()
        visualVelocityFrameContext = null
        fixedHeadingVisualVelocityAttemptId = null
        if (isStraightLineTest()) {
            stopHardwareLatencyTest("測試畫面離開前景，已中止直線測試")
        }
        stopTiltFlight("畫面離開前景，傾角實驗已停止")
        super.onPause()
    }

    override fun onDestroy() {
        // Listeners are held by `this`; leaking them across an Activity restart
        // would deliver key updates to a dead view hierarchy. The stick link is
        // released too, so a killed UI can never leave a live control link.
        activityDestroying = true
        velocityReadDiagnostics.close()
        visualVelocityDiagnostics?.close()
        visualVelocityDiagnostics = null
        visualVelocityFrameContext = null
        fixedHeadingVisualVelocityAttemptId = null
        stopHardwareLatencyTest("App 關閉，${activeHorizontalPulseName()}已停止", release = false)
        stopMathematicalCircle("App 關閉，數學圓周已停止", release = false)
        stopTiltFlight("App 關閉，傾角實驗已停止", release = false)
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
        flightProfiler.record(event = "session_end")
        runCatching { flightProfiler.close() }
            .onFailure { error -> flightLog.write("profiling close failed: $error") }
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
            onFrameStreamRecovered = ::handleTapeFrameStreamRecovered,
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
            LinearLayout(this).apply {
                orientation = LinearLayout.VERTICAL
                setPadding(dp(10), dp(10), dp(10), dp(14))
                addView(
                    buildTopBar(),
                    LinearLayout.LayoutParams(
                        ViewGroup.LayoutParams.MATCH_PARENT,
                        ViewGroup.LayoutParams.WRAP_CONTENT,
                    ),
                )
                addView(
                    View(this@MainActivity),
                    LinearLayout.LayoutParams(
                        ViewGroup.LayoutParams.MATCH_PARENT,
                        0,
                        1f,
                    ),
                )
                addView(
                    buildStickBar(),
                    LinearLayout.LayoutParams(
                        ViewGroup.LayoutParams.MATCH_PARENT,
                        ViewGroup.LayoutParams.WRAP_CONTENT,
                    ),
                )
            },
            FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.MATCH_PARENT,
            ),
        )
        hardwareLatencyButton =
            PillButton("硬體延遲脈衝・0.50 m/s", StickPadView.RED) {
                toggleHardwareLatencyTest()
            }
        straightRehearsalButton =
            PillButton("低速試跑・0.30 m/s", StickPadView.GREEN) {
                toggleStraightLineTest(HorizontalPulseExperiment.STRAIGHT_REHEARSAL)
            }
        straightMeasurementButton =
            PillButton("直線測速・0.50 m/s", StickPadView.CYAN) {
                toggleStraightLineTest(HorizontalPulseExperiment.STRAIGHT_MEASUREMENT)
            }
        root.addView(
            LinearLayout(this).apply {
                orientation = LinearLayout.HORIZONTAL
                gravity = Gravity.CENTER_HORIZONTAL
                addView(hardwareLatencyButton)
                addView(straightRehearsalButton)
                addView(straightMeasurementButton)
            },
            FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT,
                ViewGroup.LayoutParams.WRAP_CONTENT,
                Gravity.TOP or Gravity.CENTER_HORIZONTAL,
            ).apply { topMargin = dp(46) },
        )
        hardwareLatencySyncMarker = buildHardwareLatencySyncMarker()
        root.addView(
            hardwareLatencySyncMarker,
            FrameLayout.LayoutParams(dp(150), dp(150), Gravity.CENTER),
        )
        return root
    }

    private fun handleTapeDetection(detection: TapeDetection?) {
        val callbackAtNanos = System.nanoTime()
        runOnUiThread {
            if (tapeEndpointTurn) return@runOnUiThread
            val now = System.nanoTime()
            flightProfiler.record(
                event = "detection_ui",
                atNanos = now,
                frameNanos = detection?.capturedAtNanos ?: latestProfiledFrameNanos,
                durationNanos = now - callbackAtNanos,
                details =
                    profileDetails(
                        "detected" to (detection != null),
                        "visionToCallbackMs" to
                            (callbackAtNanos - latestVisionCompletedAtNanos).coerceAtLeast(0L) /
                                1_000_000.0,
                    ),
            )
            latestDetectionUiFrameNanos = detection?.capturedAtNanos ?: latestProfiledFrameNanos
            latestDetectionUiAtNanos = now
        tapeTracking.updateAircraftHeading(
            aircraftHeadingDegrees?.takeIf {
                aircraftHeadingAtNanos != 0L &&
                    now - aircraftHeadingAtNanos <= MAX_MOVING_HEADING_AGE_NANOS
            },
        )
        if (tapeTracking.enabled) {
            val visualFeedback = usesVisualVelocityFeedback()
            tapeTracking.observe(
                detection?.let {
                    synchronized(aircraftVelocityLock) {
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
                        capturedAtNanos = it.capturedAtNanos,
                        heightAboveGroundMeters = usableHeightMeters()?.takeIf { height ->
                            height.isFinite() && height > 0.0
                        },
                        confidence = it.confidence,
                        centerline = it.centerline,
                        actualTravelDirectionDegrees =
                            if (visualFeedback) null else usableBodyTravelDirectionDegrees(now),
                        actualGroundSpeedMetersPerSecond =
                            if (visualFeedback) null else groundSpeedMetersPerSecond,
                        speedFeedbackSampleAtNanos = if (visualFeedback) 0L else aircraftVelocityAtNanos,
                        speedFeedbackUnavailableReason =
                            if (visualFeedback) null else speedFeedbackInputFailure(now),
                    )
                    }
                },
                now,
            )
            activeTapeControlTrigger = TapeControlTrigger.FRESH_VISION
            tapeTrackingControlRunnable.run()
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
        }

    private fun recordTapeFrameProfile(profile: TapeFrameProfile) {
        latestProfiledFrameNanos = profile.receivedAtNanos
        latestVisionCompletedAtNanos = profile.processingCompletedAtNanos
        flightProfiler.recordLazy(
            event = "vision",
            atNanos = profile.processingCompletedAtNanos,
            frameNanos = profile.receivedAtNanos,
            durationNanos = profile.processingCompletedAtNanos - profile.processingStartedAtNanos,
        ) {
            profileDetails(
                "sequence" to profile.sequence,
                "queueMs" to
                    (profile.processingStartedAtNanos - profile.receivedAtNanos) / 1_000_000.0,
                "totalMs" to
                    (profile.processingCompletedAtNanos - profile.receivedAtNanos) / 1_000_000.0,
                "preprocessMs" to profile.preprocessingNanos / 1_000_000.0,
                "thresholdMs" to profile.thresholdingNanos / 1_000_000.0,
                "floorContextMs" to profile.floorContextNanos / 1_000_000.0,
                "morphologyContoursMs" to profile.morphologyAndContoursNanos / 1_000_000.0,
                "candidateMs" to profile.candidateScoringNanos / 1_000_000.0,
                "candidateMaskMs" to profile.candidateMaskNanos / 1_000_000.0,
                "candidateCenterlineMs" to profile.candidateCenterlineNanos / 1_000_000.0,
                "candidateAppearanceMs" to profile.candidateAppearanceNanos / 1_000_000.0,
                "candidateTemporalMs" to profile.candidateTemporalNanos / 1_000_000.0,
                "centerlineDownsampleMs" to profile.centerlineDownsampleNanos / 1_000_000.0,
                "centerlineGapFillMs" to profile.centerlineGapFillNanos / 1_000_000.0,
                "centerlineDistanceMs" to profile.centerlineDistanceNanos / 1_000_000.0,
                "centerlineThinningMs" to profile.centerlineThinningNanos / 1_000_000.0,
                "centerlineRouteMs" to profile.centerlineRouteNanos / 1_000_000.0,
                "centerlineQualityMs" to profile.centerlineQualityNanos / 1_000_000.0,
                "centerlineTopologyMs" to profile.centerlineTopologyNanos / 1_000_000.0,
                "candidateMeasurementMs" to profile.candidateMeasurementNanos / 1_000_000.0,
                "contours" to profile.contourCount,
                "fullCenterlines" to profile.fullCenterlineExtractions,
                "temporalAttempts" to profile.temporalCenterlineAttempts,
                "temporalAccepts" to profile.temporalCenterlineAccepts,
                "cleanupMs" to profile.cleanupNanos / 1_000_000.0,
                "otherMs" to
                    (
                        profile.processingCompletedAtNanos -
                            profile.processingStartedAtNanos -
                            profile.preprocessingNanos -
                            profile.thresholdingNanos -
                            profile.floorContextNanos -
                            profile.morphologyAndContoursNanos -
                            profile.candidateScoringNanos -
                            profile.cleanupNanos
                    ).coerceAtLeast(0L) / 1_000_000.0,
                "width" to profile.width,
                "height" to profile.height,
                "detected" to profile.detected,
            )
        }
    }

    private fun recordVirtualStickFrame(profile: VirtualStickFrameProfile) {
        if (tiltFlightRun != null && !profile.succeeded) {
            stopTiltFlight("傾角命令發送失敗，已切回零速度")
        }
        driveMathematicalCircle(profile.sentAtNanos)
        flightProfiler.record(
            event = "virtual_stick",
            atNanos = profile.sentAtNanos,
            durationNanos = profile.sendDurationNanos,
            details = profileDetails(
                "success" to profile.succeeded,
                "sendCallMs" to profile.sendDurationNanos / 1_000_000.0,
                "stickHz" to profile.configuredRateHz,
                "commandToFrameMs" to
                    profile.horizontalCommandUpdatedAtNanos
                        .takeIf { it > 0L }
                        ?.let { (profile.sentAtNanos - it).coerceAtLeast(0L) / 1_000_000.0 },
                "forward" to profile.forwardMetersPerSecond,
                "right" to profile.rightMetersPerSecond,
                "rollPitchMode" to profile.rollPitchMode,
                "rollDegrees" to profile.rollDegrees,
                "pitchDegrees" to profile.pitchDegrees,
                "up" to profile.climbMetersPerSecond,
                "yawMode" to profile.yawMode,
                "yaw" to profile.yawValue,
            ),
        )
    }

    private fun handleTapeFrameStreamStale() {
        val nowNanos = System.nanoTime()
        cameraFrameStreamStaleAtNanos = nowNanos
        tapeDetector?.resetTracking()
        val waitingForFrameRecovery = tapeTracking.beginFrameStreamRecovery(nowNanos)
        if (!waitingForFrameRecovery) {
            tapeTracking.observe(null, nowNanos)
        }
        if (tapeTracking.enabled) {
            commandedTapeYawRate = 0.0
            commandedTapeForwardSpeed = 0.0
            commandedTapeRightSpeed = 0.0
            virtualStick.setYawRate(0.0)
            virtualStick.setHorizontalVelocity(0.0, 0.0)
        }
        flightProfiler.record(
            event = "camera_stream_stale",
            atNanos = nowNanos,
            details = profileDetails("boundedRecovery" to waitingForFrameRecovery),
        )
        flightLog.write(
            "RGBA frame stream stale; detector reset boundedRecovery=$waitingForFrameRecovery",
        )
        runOnUiThread {
            if (!::tapeOverlay.isInitialized) return@runOnUiThread
            consecutiveTapeMisses = 0
            tapeDetected = false
            tapeOverlay.showDetection(null)
            tapeOverlay.showCenterline(null)
        }
    }

    private fun handleTapeFrameStreamRecovered() {
        val recoveredAtNanos = System.nanoTime()
        val staleAtNanos = cameraFrameStreamStaleAtNanos
        val recoveryDurationNanos =
            if (staleAtNanos > 0L) {
                (recoveredAtNanos - staleAtNanos).coerceAtLeast(0L)
            } else {
                0L
            }
        val recoveryMillis =
            recoveryDurationNanos.takeIf { staleAtNanos > 0L }?.div(1_000_000.0)
        flightProfiler.record(
            event = "camera_stream_recovered",
            atNanos = recoveredAtNanos,
            durationNanos = recoveryDurationNanos,
            details = profileDetails("recoveryMs" to recoveryMillis),
        )
        flightLog.write(
            "RGBA frame stream recovered recoveryMs=" +
                (recoveryMillis?.let { "%.1f".format(it) } ?: "unknown"),
        )
        cameraFrameStreamStaleAtNanos = 0L
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

    private fun buildExperimentActionRow(): ViewGroup = HorizontalScrollView(this).apply {
        isHorizontalScrollBarEnabled = false
        overScrollMode = View.OVER_SCROLL_NEVER
        addView(
            LinearLayout(this@MainActivity).apply {
                orientation = LinearLayout.HORIZONTAL
                gravity = Gravity.START
                virtualStickFrameRateButton =
                    PillButton(virtualStickFrameRateLabel(), StickPadView.CYAN) {
                        toggleVirtualStickFrameRate()
                    }
                circularTapeTrackingButton =
                    PillButton("方案 C：Ø1.5m 定曲率＋視覺修正・0.70 m/s", StickPadView.CYAN) {
                        toggleCircularTapeTracking()
                    }
                angleCircularTapeTrackingButton =
                    PillButton("ANGLE 前視點・0.10 m/s", StickPadView.RED) {
                        toggleAngleCircularTapeTracking()
                    }
                fixedHeadingLowSpeedButton =
                    PillButton("B0：定向滑行・0.60 m/s", StickPadView.GREEN) {
                        toggleFixedHeadingLap(FixedHeadingActuationPhaseLead.LOW_SPEED_14)
                    }
                fixedHeadingFourteenPhaseLeadButton =
                    PillButton("方案 B：定向滑行・相位超前 14°", StickPadView.GREEN) {
                        toggleFixedHeadingLap(FixedHeadingActuationPhaseLead.DEGREES_14)
                    }
                fixedHeadingFasterPhaseLeadButton =
                    PillButton("方案 B：更快・1.60 m/s・超前 16°", StickPadView.AMBER) {
                        toggleFixedHeadingLap(FixedHeadingActuationPhaseLead.DEGREES_16)
                    }
                fixedHeadingCurvatureFeedforwardButton =
                    PillButton("方案 B2：曲率前饋・巡航 1.60 m/s・最大 1.90 m/s", StickPadView.RED) {
                        toggleFixedHeadingLap(
                            FixedHeadingActuationPhaseLead.CURVATURE_FEEDFORWARD_16,
                        )
                    }
                fixedHeadingFastCruiseButton =
                    PillButton("方案 B3：視覺速度回饋・巡航 1.60 m/s・最大 1.90 m/s", StickPadView.RED) {
                        toggleFixedHeadingLap(
                            FixedHeadingActuationPhaseLead.CURVATURE_FEEDFORWARD_16_VISUAL,
                        )
                    }
                fixedHeadingScheduledActuationButton =
                    PillButton(fixedHeadingScheduledActuationLabel(), StickPadView.RED) {
                        toggleFixedHeadingLap(
                            FixedHeadingActuationPhaseLead.SPEED_SCHEDULED_VISUAL,
                        )
                    }
                fixedHeadingSpeedTargetButton =
                    PillButton(fixedHeadingSpeedTargetLabel(), StickPadView.AMBER) {
                        cycleFixedHeadingSpeedTarget()
                    }
                mathematicalSkatingCircleButton =
                    PillButton(mathematicalCircleLabel(MathematicalCircleMode.SKATING), StickPadView.GREEN) {
                        toggleMathematicalCircle(MathematicalCircleMode.SKATING)
                    }
                mathematicalRacingCircleButton =
                    PillButton(mathematicalCircleLabel(MathematicalCircleMode.RACING), StickPadView.AMBER) {
                        toggleMathematicalCircle(MathematicalCircleMode.RACING)
                    }
                mathematicalFastRacingCircleButton =
                    PillButton(mathematicalCircleLabel(MathematicalCircleMode.RACING_FAST), StickPadView.RED) {
                        toggleMathematicalCircle(MathematicalCircleMode.RACING_FAST)
                    }
                tiltStraightButton =
                    PillButton(tiltFlightLabel(TiltFlightMode.STRAIGHT), StickPadView.AMBER) {
                        toggleTiltFlight(TiltFlightMode.STRAIGHT)
                    }
                tiltCircleButton =
                    PillButton(tiltFlightLabel(TiltFlightMode.CIRCLE), StickPadView.RED) {
                        toggleTiltFlight(TiltFlightMode.CIRCLE)
                    }
                tiltSkatingCircleButton =
                    PillButton(tiltFlightLabel(TiltFlightMode.SKATING_CIRCLE), StickPadView.GREEN) {
                        toggleTiltFlight(TiltFlightMode.SKATING_CIRCLE)
                    }
                yawLapTestButton =
                    PillButton("航向模式原地旋轉 360°", StickPadView.AMBER) {
                        toggleYawLapTest()
                    }
                curvedOutAndBackTrackingButton =
                    PillButton("弧形往返追蹤", StickPadView.GREEN) {
                        toggleCurvedOutAndBackTracking()
                    }
                captureButton =
                    PillButton("錄製影格證據", StickPadView.AMBER) {
                        toggleFrameCapture()
                    }
                addView(virtualStickFrameRateButton, actionParams(marginEnd = dp(4)))
                addView(tiltStraightButton, actionParams(marginEnd = dp(4)))
                addView(tiltCircleButton, actionParams(marginEnd = dp(4)))
                addView(tiltSkatingCircleButton, actionParams(marginEnd = dp(4)))
                addView(circularTapeTrackingButton, actionParams(marginEnd = dp(4)))
                addView(fixedHeadingLowSpeedButton, actionParams(marginEnd = dp(4)))
                addView(fixedHeadingFourteenPhaseLeadButton, actionParams(marginEnd = dp(4)))
                addView(fixedHeadingFasterPhaseLeadButton, actionParams(marginEnd = dp(4)))
                addView(fixedHeadingCurvatureFeedforwardButton, actionParams(marginEnd = dp(4)))
                addView(fixedHeadingFastCruiseButton, actionParams(marginEnd = dp(4)))
                addView(fixedHeadingScheduledActuationButton, actionParams(marginEnd = dp(4)))
                addView(fixedHeadingSpeedTargetButton, actionParams(marginEnd = dp(4)))
                addView(mathematicalSkatingCircleButton, actionParams(marginEnd = dp(4)))
                addView(mathematicalRacingCircleButton, actionParams(marginEnd = dp(4)))
                addView(mathematicalFastRacingCircleButton, actionParams(marginEnd = dp(4)))
                addView(yawLapTestButton, actionParams(marginEnd = dp(4)))
                addView(curvedOutAndBackTrackingButton, actionParams(marginEnd = dp(4)))
                addView(captureButton, actionParams())
            },
        )
    }
    private fun virtualStickFrameRateLabel(): String {
        val frameRate = virtualStick.frameRate()
        val profile =
            if (frameRate == VirtualStickFrameRate.BASELINE_20) "基準" else "提速"
        return "VS 發送：${frameRate.hertz} Hz（$profile）"
    }



    private fun isStraightLineTest(experiment: HorizontalPulseExperiment? = horizontalPulseExperiment): Boolean =
        experiment == HorizontalPulseExperiment.STRAIGHT_REHEARSAL ||
            experiment == HorizontalPulseExperiment.STRAIGHT_MEASUREMENT

    private fun straightLineButton(experiment: HorizontalPulseExperiment): PillButton =
        if (experiment == HorizontalPulseExperiment.STRAIGHT_REHEARSAL) {
            straightRehearsalButton
        } else {
            straightMeasurementButton
        }

    private fun toggleStraightLineTest(experiment: HorizontalPulseExperiment) {
        if (straightLineReleasePending) {
            if (stickTransitionPending || pendingAcquireCallbacks > 0 || straightLineTracePending) return
            releaseControlLink { error ->
                render(error?.let { "交還控制權失敗：$it，請用實體遙控器接管" } ?: "等待飛機確認 RC 控制權")
            }
            return
        }
        if (hardwareLatencyStartPending || hardwareLatencyPulseSequence != null) {
            if (horizontalPulseExperiment != experiment) return
            val sequence = hardwareLatencyPulseSequence as? StraightLineSpeedSequence
            if (sequence == null || hardwareLatencyStartedAtNanos == 0L) {
                stopHardwareLatencyTest("操作者取消；本趟未開始移動")
            } else {
                sequence.requestStop(System.nanoTime())?.let(::applyHorizontalPulseStep)
            }
            return
        }
        if (stickTransitionPending || anotherFlightControlActive()) return
        startStraightLineTest(experiment)
    }


    private fun startStraightLineTest(experiment: HorizontalPulseExperiment) {
        val rehearsal = experiment == HorizontalPulseExperiment.STRAIGHT_REHEARSAL
        val config = StraightLineTestConfig(
            direction = StraightLineDirection.FORWARD,
            speedMetersPerSecond = if (rehearsal) 0.3 else 0.5,
            maximumCruiseSeconds = 20,
        )
        val attemptId = "${if (rehearsal) "rehearsal" else "measurement"}-${System.currentTimeMillis()}"
        straightLineTestConfig = config
        straightLineAttemptId = attemptId
        straightLineInitialHeading = aircraftHeadingDegrees?.takeIf { it.isFinite() }
        straightLineLastSampleAtNanos = 0L
        flightProfiler.record(
            event = "straight_test_requested",
            details = profileDetails(
                "attemptId" to attemptId, "kind" to experiment,
                "direction" to config.direction, "commandMps" to config.speedMetersPerSecond,
                "maximumCruiseSeconds" to config.maximumCruiseSeconds,
                "markerDistanceMeters" to null,
                "heightMeters" to usableHeightMeters(), "heightSource" to heightSource,
                "heading" to straightLineInitialHeading, "gimbalPitch" to selectedCameraPitchDegrees,
                "recordingConfirmedByOperator" to false,
                "safetyConfirmedByOperator" to false,
                "wallTimeMillis" to System.currentTimeMillis(),
            ),
        )
        startHorizontalPulseExperiment(experiment)
    }


    private fun recordStraightLineSample(nowNanos: Long, sequence: StraightLineSpeedSequence) {
        if (nowNanos - straightLineLastSampleAtNanos < 100_000_000L) return
        straightLineLastSampleAtNanos = nowNanos
        val config = straightLineTestConfig ?: return
        val details = synchronized(aircraftVelocityLock) {
            val sampleAt = aircraftVelocityAtNanos
            val age = sampleAt.takeIf { it > 0L }?.let { (nowNanos - it).coerceAtLeast(0L) / 1_000_000.0 }
            val axis = straightLineInitialHeading?.let(Math::toRadians)
            val sign = config.direction.forwardSign
            val rawAlong = axis?.let {
                sign * (aircraftVelocityX * kotlin.math.cos(it) + aircraftVelocityY * kotlin.math.sin(it))
            }
            val rawCross = axis?.let {
                sign * (-aircraftVelocityX * kotlin.math.sin(it) + aircraftVelocityY * kotlin.math.cos(it))
            }
            profileDetails(
                "attemptId" to straightLineAttemptId, "phase" to sequence.phase,
                "stopReason" to sequence.stopReason, "direction" to config.direction,
                "velocitySampleNanos" to sampleAt, "velocityAgeMs" to age,
                "velocityValid" to (age != null && age <= 1_000.0),
                "velocityX" to aircraftVelocityX, "velocityY" to aircraftVelocityY, "velocityZ" to aircraftVelocityZ,
                "alongRawMps" to rawAlong, "crossRawMps" to rawCross,
                "axisHeading" to straightLineInitialHeading, "heading" to aircraftHeadingDegrees,
                "headingSampleNanos" to aircraftHeadingAtNanos, "pitch" to aircraftPitchDegrees,
                "roll" to aircraftRollDegrees, "heightMeters" to usableHeightMeters(),
                "heightSampleNanos" to altitudeAtNanos, "heightSource" to heightSource,
                "authority" to stickStatus.authority, "flightMode" to flightModeName,
                "obstacleMm" to nearestHorizontalObstacleMm, "upMm" to upwardObstacleMm,
            )
        }
        flightProfiler.record(event = "straight_test_sample", atNanos = nowNanos, details = details)
    }

    private fun toggleVirtualStickFrameRate() {
        if (
            stickOwned || stickTransitionPending || stickStatus.enabled ||
            anotherFlightControlActive()
        ) {
            render("請先停止飛行控制並交回遙控器，再切換 Virtual Stick 發送頻率")
            return
        }
        val selected = virtualStick.frameRate().toggled()
        if (!virtualStick.selectFrameRate(selected)) {
            render("Virtual Stick sender 仍在執行，發送頻率未切換")
            return
        }
        virtualStickFrameRateButton.text = virtualStickFrameRateLabel()
        flightLog.write("virtual stick frame rate selected=${selected.hertz}Hz")
        flightProfiler.record(
            event = "stick_rate_selected",
            details = profileDetails("stickHz" to selected.hertz),
        )
        render("Virtual Stick 發送頻率已切換為 ${selected.hertz} Hz")
    }



    private fun buildHardwareLatencySyncMarker(): TextView = TextView(this).apply {
        text = "SYNC"
        textSize = 22f
        gravity = Gravity.CENTER
        setTypeface(typeface, android.graphics.Typeface.BOLD)
        setPadding(dp(8), dp(8), dp(8), dp(8))
        setHardwareLatencyMarker(white = false)
        visibility = View.GONE
    }

    private fun setHardwareLatencyMarker(white: Boolean) {
        if (!::hardwareLatencySyncMarker.isInitialized) return
        hardwareLatencySyncMarker.setTextColor(if (white) Color.BLACK else Color.WHITE)
        hardwareLatencySyncMarker.background = GradientDrawable().apply {
            setColor(if (white) Color.WHITE else Color.BLACK)
            setStroke(dp(3), if (white) Color.BLACK else Color.WHITE)
        }
    }

    /**
     * Manual capture remains available for arbitrary experiments. Schemes B and
     * C arm the same recorder automatically when their flight control actually starts.
     */
    private fun toggleFrameCapture() {
        val recorder = captureRecorder
        if (recorder == null) {
            render("影格證據儲存無法使用")
            return
        }
        mainHandler.removeCallbacks(autoCaptureDisarmRunnable)
        captureAutoArmedByTracking = false
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

    private fun armFrameCaptureFor(mode: TapeTrackingMode) {
        if (!mode.automaticallyCapturesEvidence) return
        val recorder = captureRecorder
        if (recorder == null) {
            flightLog.write("automatic tape capture unavailable mode=$mode")
            return
        }
        mainHandler.removeCallbacks(autoCaptureDisarmRunnable)
        if (captureAutoArmedByTracking && recorder.isArmed) {
            // A rapid restart is a new run: never mix its leading window with
            // frames retained for the previous run.
            recorder.disarm()
            captureAutoArmedByTracking = false
        }
        if (!recorder.isArmed) {
            recorder.arm()
            captureAutoArmedByTracking = true
            captureButton.text = "停止錄製影格"
            flightLog.write("automatic tape capture armed mode=$mode")
        } else {
            flightLog.write("automatic tape capture using operator-armed recorder mode=$mode")
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
        "tracking.circularSpeed" to activeCircularTrackingSpeed.name,
        "tracking.circularYawControl" to activeCircularYawControlMode.name,
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
        firmwareView = label(10f, StickPadView.CYAN).apply {
            minHeight = dp(36)
            gravity = Gravity.CENTER_VERTICAL
            setOnClickListener {
                if (!registered) {
                    render("尚未完成 MSDK 註冊，無法讀取韌體版本")
                } else {
                    readFirmwareVersion(aircraftFirmware)
                    readFirmwareVersion(remoteControllerFirmware)
                }
            }
        }
        updateFirmwareText()
        return LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            background = panelBackground()
            setPadding(dp(12), dp(8), dp(12), dp(8))
            addView(headlineView)
            addView(detailView)
            addView(holdView)
            addView(telemetryView)
            addView(firmwareView)
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
        toggleTapeTracking(TapeTrackingMode.CIRCULAR, CircularTrackingSpeed.FAST)
    }

    private fun toggleAngleCircularTapeTracking() {
        toggleTapeTracking(
            TapeTrackingMode.CIRCULAR,
            CircularTrackingSpeed.ANGLE,
            CircularYawControlMode.HEADING,
        )
    }


    private fun fixedHeadingSpeedTargetLabel(
        target: FixedHeadingSpeedTarget = selectedFixedHeadingSpeedTarget,
    ): String =
        "B4 沿線目標 %.2f m/s・命令上限 %.2f m/s".format(
            target.desiredAlongTrackSpeedMetersPerSecond,
            target.maximumCommandSpeedMetersPerSecond,
        )

    private fun fixedHeadingScheduledActuationLabel(
        target: FixedHeadingSpeedTarget = selectedFixedHeadingSpeedTarget,
    ): String =
        "方案 B4：動態提前＋增益・沿線目標 %.2f m/s・命令上限 %.2f m/s".format(
            target.desiredAlongTrackSpeedMetersPerSecond,
            target.maximumCommandSpeedMetersPerSecond,
        )

    private fun canSelectFixedHeadingSpeedTarget(): Boolean =
        activeTapeTrackingMode == null && !anotherFlightControlActive() &&
            !yawLapTestActive && !stickTransitionPending

    private fun cycleFixedHeadingSpeedTarget() {
        if (!canSelectFixedHeadingSpeedTarget()) return
        selectedFixedHeadingSpeedTarget = when (selectedFixedHeadingSpeedTarget) {
            FixedHeadingSpeedTarget.BASELINE -> FixedHeadingSpeedTarget.STEP_075
            FixedHeadingSpeedTarget.STEP_075 -> FixedHeadingSpeedTarget.STEP_080
            FixedHeadingSpeedTarget.STEP_080 -> FixedHeadingSpeedTarget.STEP_085
            FixedHeadingSpeedTarget.STEP_085 -> FixedHeadingSpeedTarget.BASELINE
        }
        fixedHeadingSpeedTargetButton.text = fixedHeadingSpeedTargetLabel()
        fixedHeadingScheduledActuationButton.text = fixedHeadingScheduledActuationLabel()
        render("${fixedHeadingSpeedTargetLabel()}（僅選擇，下次手動啟動 B4 時套用）")
    }

    private fun toggleFixedHeadingLap(
        actuationPhaseLead: FixedHeadingActuationPhaseLead,
    ) {
        toggleTapeTracking(
            mode = TapeTrackingMode.FIXED_HEADING,
            fixedHeadingActuationPhaseLead = actuationPhaseLead,
            fixedHeadingSpeedTarget =
                if (actuationPhaseLead.usesSpeedScheduledActuation) {
                    selectedFixedHeadingSpeedTarget
                } else {
                    FixedHeadingSpeedTarget.BASELINE
                },
        )
    }


    private fun toggleCurvedOutAndBackTracking() {
        toggleTapeTracking(TapeTrackingMode.CURVED_OUT_AND_BACK)
    }

    private fun toggleTapeTracking(
        mode: TapeTrackingMode,
        circularTrackingSpeed: CircularTrackingSpeed = CircularTrackingSpeed.FAST,
        circularYawControlMode: CircularYawControlMode = CircularYawControlMode.RATE,
        fixedHeadingActuationPhaseLead: FixedHeadingActuationPhaseLead =
            FixedHeadingActuationPhaseLead.DEGREES_0,
        fixedHeadingSpeedTarget: FixedHeadingSpeedTarget = FixedHeadingSpeedTarget.BASELINE,
    ) {
        val sameSpeed =
            mode != TapeTrackingMode.CIRCULAR ||
                activeCircularTrackingSpeed == circularTrackingSpeed
        val sameYawControl =
            mode != TapeTrackingMode.CIRCULAR ||
                activeCircularYawControlMode == circularYawControlMode
        val sameFixedHeadingPhaseLead =
            mode != TapeTrackingMode.FIXED_HEADING ||
                activeFixedHeadingActuationPhaseLead == fixedHeadingActuationPhaseLead
        val sameRun =
            activeTapeTrackingMode == mode &&
                sameSpeed &&
                sameYawControl &&
                sameFixedHeadingPhaseLead
        when {
            tapeTracking.enabled && sameRun ->
                stopTapeTracking(
                    "${
                        tapeTrackingName(
                            mode,
                            circularTrackingSpeed,
                            circularYawControlMode,
                            fixedHeadingActuationPhaseLead,
                            activeFixedHeadingSpeedTarget,
                        )
                    }已由操作者停止",
                    release = true,
                )
            tapeTracking.enabled -> render("另一個黑膠帶追蹤模式正在使用中")
            tapeTrackingStartPending -> render("正在切換飛機避障模式，請稍候")
            else ->
                startTapeTracking(
                    mode,
                    circularTrackingSpeed,
                    circularYawControlMode,
                    fixedHeadingActuationPhaseLead,
                    fixedHeadingSpeedTarget,
                )
        }
    }

    private fun startTapeTracking(
        mode: TapeTrackingMode,
        circularTrackingSpeed: CircularTrackingSpeed = CircularTrackingSpeed.FAST,
        circularYawControlMode: CircularYawControlMode = CircularYawControlMode.RATE,
        fixedHeadingActuationPhaseLead: FixedHeadingActuationPhaseLead =
            FixedHeadingActuationPhaseLead.DEGREES_0,
        fixedHeadingSpeedTarget: FixedHeadingSpeedTarget = FixedHeadingSpeedTarget.BASELINE,
    ) {
        val trackingName =
            tapeTrackingName(
                mode,
                circularTrackingSpeed,
                circularYawControlMode,
                fixedHeadingActuationPhaseLead,
                fixedHeadingSpeedTarget,
            )
        val speedScheduledActuation =
            mode == TapeTrackingMode.FIXED_HEADING &&
                fixedHeadingActuationPhaseLead.usesSpeedScheduledActuation
        val configuredPhaseLeadDegrees =
            fixedHeadingActuationPhaseLead.degrees.takeUnless { speedScheduledActuation }
        val desiredAlongTrackSpeed =
            if (speedScheduledActuation) {
                fixedHeadingSpeedTarget.desiredAlongTrackSpeedMetersPerSecond
            } else {
                fixedHeadingActuationPhaseLead.desiredAlongTrackSpeedMetersPerSecond
            }
        val nominalCommandSpeed =
            if (speedScheduledActuation) {
                fixedHeadingActuationPhaseLead.targetSpeedMetersPerSecond *
                    fixedHeadingSpeedTarget.desiredAlongTrackSpeedMetersPerSecond /
                    FixedHeadingSpeedTarget.BASELINE.desiredAlongTrackSpeedMetersPerSecond
            } else {
                fixedHeadingActuationPhaseLead.targetSpeedMetersPerSecond
            }
        val maximumCommandSpeed =
            if (speedScheduledActuation) {
                fixedHeadingSpeedTarget.maximumCommandSpeedMetersPerSecond
            } else {
                fixedHeadingActuationPhaseLead.maximumCommandSpeedMetersPerSecond
            }
        flightLog.write(
            "press: tape tracking mode=$mode circularSpeed=$circularTrackingSpeed " +
                "circularYawControl=$circularYawControlMode " +
                "fixedHeadingProfile=$fixedHeadingActuationPhaseLead " +
                "fixedHeadingPhaseLeadDegrees=${configuredPhaseLeadDegrees ?: "speed_scheduled"} " +
                "fixedHeadingTargetSpeed=$nominalCommandSpeed " +
                "fixedHeadingDesiredAlongTrackSpeed=$desiredAlongTrackSpeed " +
                "fixedHeadingMaximumCommandSpeed=$maximumCommandSpeed " +
                "stickHz=${virtualStick.frameRate().hertz} " +
                "registered=$registered connected=$aircraftConnected flying=$flying " +
                "owned=$stickOwned cameraPitch=" +
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
        activeCircularTrackingSpeed = circularTrackingSpeed
        activeCircularYawControlMode = circularYawControlMode
        activeFixedHeadingActuationPhaseLead = fixedHeadingActuationPhaseLead
        activeFixedHeadingSpeedTarget = fixedHeadingSpeedTarget
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
                        activeCircularTrackingSpeed = CircularTrackingSpeed.FAST
                        activeCircularYawControlMode = CircularYawControlMode.RATE
                        activeFixedHeadingActuationPhaseLead =
                            FixedHeadingActuationPhaseLead.DEGREES_0
                        activeFixedHeadingSpeedTarget = FixedHeadingSpeedTarget.BASELINE
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
                        tapeDetector?.setDetectionMode(mode.detectionMode)
                        // Preview is stateless. Flight control receives no path until
                        // the detector confirms one full route across multiple frames.
                        tapeDetector?.beginTrackingSession(
                            requireConsistentCurve =
                                mode == TapeTrackingMode.CIRCULAR ||
                                    mode == TapeTrackingMode.FIXED_HEADING,
                        )
                        val now = System.nanoTime()
                        if (
                            mode == TapeTrackingMode.FIXED_HEADING ||
                            mode == TapeTrackingMode.CIRCULAR
                        ) {
                            val heading = aircraftHeadingDegrees
                            if (
                                heading == null ||
                                aircraftHeadingAtNanos == 0L ||
                                now - aircraftHeadingAtNanos > MAX_MOVING_HEADING_AGE_NANOS
                            ) {
                                val schemeName =
                                    if (mode == TapeTrackingMode.FIXED_HEADING) "方案 B" else "方案 C"
                                stopTapeTracking(
                                    "沒有即時機頭方向，$schemeName 未啟動",
                                    release = true,
                                )
                                return@acquireControlLink
                            }
                            tapeTracking.updateAircraftHeading(heading)
                            if (mode == TapeTrackingMode.FIXED_HEADING) {
                                fixedHeadingReferenceDegrees = heading
                            }
                        }
                        // The circular experiment uses a physically closed loop. Curved
                        // out-and-back tracking uses the same path detector with its endpoint armed.
                        val endpointTurnEnabled =
                            mode != TapeTrackingMode.CIRCULAR &&
                                mode != TapeTrackingMode.FIXED_HEADING
                        tapeTracking.start(
                            nowNanos = now,
                            mode = mode,
                            endpointTurnEnabled = endpointTurnEnabled,
                            circularTrackingSpeed = circularTrackingSpeed,
                            fixedHeadingActuationPhaseLead = fixedHeadingActuationPhaseLead,
                            fixedHeadingSpeedTarget = fixedHeadingSpeedTarget,
                        )
                        if (
                            mode == TapeTrackingMode.FIXED_HEADING &&
                            fixedHeadingActuationPhaseLead.usesCurvatureFeedforward &&
                            activityForeground && !activityDestroying
                        ) {
                            val profileTag = when {
                                fixedHeadingActuationPhaseLead.usesSpeedScheduledActuation -> "b4"
                                fixedHeadingActuationPhaseLead.usesVisualVelocity -> "b3"
                                else -> "b2"
                            }
                            val attemptId = "$flightProfileSessionId-$profileTag-$now"
                            fixedHeadingVisualVelocityAttemptId = attemptId
                            visualVelocityDiagnostics?.start(
                                attemptId, TapeTrackingPhase.RECENTERING.name, fixedHeadingReferenceDegrees,
                                controlFeedback = fixedHeadingActuationPhaseLead.usesVisualVelocity,
                            )
                            refreshVisualVelocityContext()
                        }
                        angleHeadingController.reset()
                        armFrameCaptureFor(mode)
                        tapeTrackingStartedAtNanos = now
                        tapeTrackingAuthoritySeen = stickStatus.hasMsdkAuthority
                        renderedTapeTrackingPhase = TapeTrackingPhase.DISABLED
                        commandedTapeYawRate = 0.0
                        commandedTapeForwardSpeed = 0.0
                        commandedTapeRightSpeed = 0.0
                        diagnosticTurnCycleTimer.reset()
                        tapeCommandLoggedAtNanos = 0L
                        flightProfiler.record(
                            event = "tracking_start",
                            atNanos = now,
                            details = profileDetails(
                                "mode" to mode,
                                "circularSpeed" to circularTrackingSpeed,
                                "yawControl" to circularYawControlMode,
                                "fixedHeadingPhaseLeadDegrees" to
                                    configuredPhaseLeadDegrees,
                                "fixedHeadingProfile" to fixedHeadingActuationPhaseLead,
                                "fixedHeadingTargetSpeed" to nominalCommandSpeed,
                                "fixedHeadingDesiredAlongTrackSpeed" to desiredAlongTrackSpeed,
                                "fixedHeadingMaximumCommandSpeed" to maximumCommandSpeed,
                                "fixedHeadingSpeedTarget" to
                                    fixedHeadingSpeedTarget.takeIf { speedScheduledActuation },
                                "actuationModel" to
                                    "bounded_empirical_one_pole_relative_gain".takeIf { speedScheduledActuation },
                                "actuationModelSeed" to
                                    "flown_16deg_at_0p71radps_not_identified_transfer_function"
                                        .takeIf { speedScheduledActuation },
                                "actuationGainNormalization" to
                                    "relative_to_reference_no_inverse_K".takeIf { speedScheduledActuation },
                                "actuationAdditionalDelaySeconds" to 0.0.takeIf { speedScheduledActuation },
                                "actuationReferencePhaseLeadDegrees" to
                                    FixedHeadingLapController.SCHEDULED_REFERENCE_PHASE_LEAD_DEGREES
                                        .takeIf { speedScheduledActuation },
                                "actuationReferenceTurnRateRadiansPerSecond" to
                                    FixedHeadingLapController.SCHEDULED_REFERENCE_TURN_RATE_RADIANS_PER_SECOND
                                        .takeIf { speedScheduledActuation },
                                "actuationResponseTimeSeconds" to
                                    FixedHeadingLapController.SCHEDULED_RESPONSE_TIME_SECONDS
                                        .takeIf { speedScheduledActuation },
                                "actuationTurnFilterSeconds" to
                                    FixedHeadingLapController.SCHEDULED_TURN_FILTER_SECONDS
                                        .takeIf { speedScheduledActuation },
                                "actuationMaximumPhaseLeadDegrees" to
                                    FixedHeadingLapController.SCHEDULED_MAX_PHASE_LEAD_DEGREES
                                        .takeIf { speedScheduledActuation },
                                "actuationPhaseSlewDegreesPerSecond" to
                                    FixedHeadingLapController.SCHEDULED_PHASE_SLEW_DEGREES_PER_SECOND
                                        .takeIf { speedScheduledActuation },
                                "actuationMaximumGain" to
                                    FixedHeadingLapController.SCHEDULED_MAX_GAIN
                                        .takeIf { speedScheduledActuation },
                                "actuationMaximumVirtualTurnRateDegreesPerSecond" to
                                    FixedHeadingLapController.SCHEDULED_MAX_VIRTUAL_TURN_RATE_DEGREES_PER_SECOND
                                        .takeIf { speedScheduledActuation },
                                "schemeCControl" to
                                    if (mode == TapeTrackingMode.CIRCULAR) {
                                        "planned_1p5m_curvature_visual_tangent_correction"
                                    } else {
                                        null
                                    },
                                "schemeCLookaheadMeters" to
                                    if (mode == TapeTrackingMode.CIRCULAR) {
                                        TapeTrackingController.VISUAL_CURVATURE_LOOKAHEAD_METERS
                                    } else {
                                        null
                                    },
                                "stickHz" to virtualStick.frameRate().hertz,
                                "periodicControlHz" to (1_000L / TAPE_TRACKING_TICK_MS),
                            ),
                        )
                        when (mode) {
                            TapeTrackingMode.STRAIGHT ->
                                tapeTrackingButton.text = "停止直線黑膠帶追蹤"
                            TapeTrackingMode.CIRCULAR -> {
                                when {
                                    circularYawControlMode == CircularYawControlMode.HEADING ->
                                        angleCircularTapeTrackingButton.text =
                                            "停止 ANGLE 前視點修正版・0.10 m/s"
                                    else ->
                                        circularTapeTrackingButton.text =
                                            "停止方案 C：Ø1.5m 定曲率＋視覺修正・0.70 m/s"
                                }
                            }
                            TapeTrackingMode.FIXED_HEADING -> {
                                val button =
                                    when (fixedHeadingActuationPhaseLead) {
                                        FixedHeadingActuationPhaseLead.DEGREES_0 -> null
                                        FixedHeadingActuationPhaseLead.LOW_SPEED_14 ->
                                            fixedHeadingLowSpeedButton
                                        FixedHeadingActuationPhaseLead.DEGREES_14 ->
                                            fixedHeadingFourteenPhaseLeadButton
                                        FixedHeadingActuationPhaseLead.DEGREES_16 ->
                                            fixedHeadingFasterPhaseLeadButton
                                        FixedHeadingActuationPhaseLead.CURVATURE_FEEDFORWARD_16 ->
                                            fixedHeadingCurvatureFeedforwardButton
                                        FixedHeadingActuationPhaseLead.CURVATURE_FEEDFORWARD_16_VISUAL ->
                                            fixedHeadingFastCruiseButton
                                        FixedHeadingActuationPhaseLead.SPEED_SCHEDULED_VISUAL ->
                                            fixedHeadingScheduledActuationButton
                                    }
                                button?.text = "停止$trackingName"
                            }
                            TapeTrackingMode.CURVED_OUT_AND_BACK ->
                                curvedOutAndBackTrackingButton.text = "停止弧形往返追蹤"
                        }
                        flightLog.write(
                            "tape tracking started mode=$mode " +
                                "circularSpeed=$circularTrackingSpeed " +
                                "circularYawControl=$circularYawControlMode " +
                                "fixedHeadingProfile=$fixedHeadingActuationPhaseLead " +
                                "fixedHeadingPhaseLeadDegrees=${configuredPhaseLeadDegrees ?: "speed_scheduled"} " +
                                "fixedHeadingTargetSpeed=$nominalCommandSpeed " +
                                "fixedHeadingDesiredAlongTrackSpeed=$desiredAlongTrackSpeed " +
                                "fixedHeadingMaximumCommandSpeed=$maximumCommandSpeed " +
                                "schemeCControl=" +
                                if (mode == TapeTrackingMode.CIRCULAR) {
                                    "planned_1p5m_curvature_visual_tangent_correction "
                                } else {
                                    "none "
                                } +
                                "stickHz=${virtualStick.frameRate().hertz} " +
                                "periodicControlHz=${1_000L / TAPE_TRACKING_TICK_MS} " +
                                "avoidance=CLOSE endpointTurnEnabled=$endpointTurnEnabled",
                        )
                        mainHandler.removeCallbacks(tapeTrackingPeriodicRunnable)
                        mainHandler.post(tapeTrackingPeriodicRunnable)
                    }
                }
            }
        }
    }

    private fun usesVisualVelocityFeedback(): Boolean =
        activeTapeTrackingMode == TapeTrackingMode.FIXED_HEADING &&
            activeFixedHeadingActuationPhaseLead.usesVisualVelocity

    private val tapeTrackingControlRunnable = object : Runnable {
        override fun run() {
            if (!tapeTracking.enabled) return
            if (!registered || !aircraftConnected || !flying || !stickOwned) {
                stopTapeTracking("飛行或控制權狀態失效，黑膠帶追蹤已停止", release = false)
                return
            }
            val now = System.nanoTime()
            val ownsAuthority = stickStatus.hasMsdkAuthority
            if (ownsAuthority) {
                tapeTrackingAuthoritySeen = true
            } else {
                virtualStick.setYawRate(0.0)
                virtualStick.setHorizontalVelocity(0.0, 0.0)
                if (
                    tapeTrackingAuthoritySeen ||
                    now - tapeTrackingStartedAtNanos >= AUTHORITY_HANDOVER_TIMEOUT_NANOS
                ) {
                    stopTapeTracking("未取得或已失去控制權，黑膠帶追蹤已停止", release = false)
                    return
                }
            }

            val fixedHeadingYawRate =
                if (activeTapeTrackingMode == TapeTrackingMode.FIXED_HEADING && ownsAuthority) {
                    val currentHeading = aircraftHeadingDegrees
                    val referenceHeading = fixedHeadingReferenceDegrees
                    if (
                        currentHeading == null ||
                        referenceHeading == null ||
                        aircraftHeadingAtNanos == 0L ||
                        now - aircraftHeadingAtNanos > MAX_MOVING_HEADING_AGE_NANOS
                    ) {
                        stopTapeTracking("機頭方向資料停止更新，方案 B 已停止", release = true)
                        return
                    }
                    fixedHeadingHoldYawRate(currentHeading, referenceHeading)
                } else {
                    null
                }

            if (activeTapeTrackingMode == TapeTrackingMode.CIRCULAR && ownsAuthority) {
                val currentHeading = aircraftHeadingDegrees
                if (
                    currentHeading == null ||
                    aircraftHeadingAtNanos == 0L ||
                    now - aircraftHeadingAtNanos > MAX_MOVING_HEADING_AGE_NANOS
                ) {
                    stopTapeTracking("機頭方向資料停止更新，方案 C 已停止", release = true)
                    return
                }
                tapeTracking.updateAircraftHeading(currentHeading)
            }

            val decisionStartedAtNanos = System.nanoTime()
            val visualFeedback = usesVisualVelocityFeedback()
            if (visualFeedback) {
                val sample = fixedHeadingVisualVelocityAttemptId?.let {
                    visualVelocityDiagnostics?.controlSample(it)
                }
                val cameraReady = !cameraPitchCommandPending &&
                    selectedCameraPitchDegrees?.let { it.isFinite() && abs(it + 90.0) <= 0.5 } == true
                tapeTracking.updateVisualVelocity(
                    forwardMetersPerSecond = sample?.forwardMps?.takeIf { cameraReady },
                    rightMetersPerSecond = sample?.rightMps?.takeIf { cameraReady },
                    sampleHeadingDegrees = sample?.aircraftHeadingDegrees,
                    currentHeadingDegrees = aircraftHeadingDegrees?.takeIf {
                        aircraftHeadingAtNanos != 0L &&
                            now - aircraftHeadingAtNanos in 0L..MAX_MOVING_HEADING_AGE_NANOS
                    },
                    sampleAtNanos = sample?.frameNanos ?: 0L,
                    nowNanos = now,
                    unavailableReason = when {
                        !cameraReady -> "CAMERA_NOT_READY"
                        sample == null -> "MISSING_VISUAL_VELOCITY"
                        else -> sample.reason
                    },
                )
            }
            val decision = tapeTracking.tick(now)
            val decisionCompletedAtNanos = System.nanoTime()
            if (decision.endpointReached) {
                if (activeTapeTrackingMode == TapeTrackingMode.FIXED_HEADING) {
                    stopTapeTracking("方案 B 已抵達黑膠帶終點", release = true)
                } else {
                    beginTapeTurnaround()
                }
                return
            }
            if (decision.stopRequested) {
                val reason =
                    when (activeTapeTrackingMode) {
                        TapeTrackingMode.FIXED_HEADING ->
                            "方案 B 失去有效路徑或取得逾時，已停止"
                        TapeTrackingMode.CIRCULAR ->
                            "${tapeTrackingName()}路徑偏離或影格失效，已停止"
                        else ->
                            "末端確認逾時，黑膠帶追蹤已停止"
                    }
                stopTapeTracking(reason, release = true)
                return
            }
            val yawRate =
                if (ownsAuthority) {
                    fixedHeadingYawRate ?: decision.yawRateDegreesPerSecond
                } else {
                    0.0
                }
            val usesAngleYawControl =
                ownsAuthority &&
                    activeTapeTrackingMode == TapeTrackingMode.CIRCULAR &&
                    activeCircularYawControlMode == CircularYawControlMode.HEADING
            val requestedForwardSpeed =
                if (ownsAuthority) decision.forwardSpeedMetersPerSecond else 0.0
            val requestedRightSpeed =
                if (ownsAuthority) decision.rightSpeedMetersPerSecond else 0.0
            val turnAngle = when (activeTapeTrackingMode) {
                TapeTrackingMode.CIRCULAR -> aircraftHeadingDegrees
                TapeTrackingMode.FIXED_HEADING -> decision.controlledAngleDegrees
                else -> null
            }
            if (
                decision.phase == TapeTrackingPhase.TRACKING &&
                turnAngle != null &&
                !diagnosticTurnCycleTimer.armed
            ) {
                diagnosticTurnCycleTimer.arm(now, turnAngle)
                flightLog.write(
                    "diagnostic turn cycle armed mode=$activeTapeTrackingMode " +
                        "angle=%.1f notPhysicalLap=true".format(turnAngle),
                )
            }
            turnAngle?.let { angle ->
                diagnosticTurnCycleTimer.update(
                    now,
                    angle,
                    groundSpeedMetersPerSecond,
                )?.let { event ->
                    flightProfiler.record(
                        event = "diagnostic_turn_cycle",
                        atNanos = now,
                        details = profileDetails(
                            "mode" to activeTapeTrackingMode,
                            "index" to event.index,
                            "seconds" to event.elapsedSeconds,
                            "distanceMeters" to event.distanceMeters,
                            "turnDegrees" to event.turnDegrees,
                            "instantGroundSpeed" to groundSpeedMetersPerSecond,
                            "notPhysicalLap" to true,
                        ),
                    )
                    flightLog.write(
                        (
                            "diagnostic turn cycle mode=$activeTapeTrackingMode " +
                                "n=${event.index} seconds=%.2f distance=%.2f " +
                                "turnDegrees=%.1f instantGs=%.2f notPhysicalLap=true"
                        ).format(
                            event.elapsedSeconds,
                            event.distanceMeters,
                            event.turnDegrees,
                            groundSpeedMetersPerSecond,
                        ),
                    )
                }
            }
            if (requestedForwardSpeed != 0.0 || requestedRightSpeed != 0.0) {
                tapeTrackingStopReason()?.let { reason ->
                    stopTapeTracking(reason, release = true)
                    return
                }
            }
            val yawHeadingTarget =
                if (usesAngleYawControl) {
                    val currentHeading = aircraftHeadingDegrees
                    if (
                        currentHeading == null ||
                        aircraftHeadingAtNanos == 0L ||
                        now - aircraftHeadingAtNanos > MAX_MOVING_HEADING_AGE_NANOS
                    ) {
                        stopTapeTracking(
                            "機頭方向資料停止更新，ANGLE 前視點修正版已停止",
                            release = true,
                        )
                        return
                    }
                    angleHeadingController.update(
                        currentHeadingDegrees = currentHeading,
                        relativePathBearingDegrees = decision.pathBearingDegrees,
                        pathTracking =
                            decision.phase == TapeTrackingPhase.TRACKING &&
                                decision.pathQuality == PathQuality.FULL_PATH,
                        nowNanos = now,
                    )
                } else {
                    null
                }
            if (usesAngleYawControl) {
                if (yawHeadingTarget == null || !virtualStick.setYawHeading(yawHeadingTarget)) {
                    stopTapeTracking(
                        "航向設定點無效，ANGLE 前視點修正版已停止",
                        release = true,
                    )
                    return
                }
            } else {
                virtualStick.setYawRate(yawRate)
            }
            virtualStick.setHorizontalVelocity(
                requestedForwardSpeed,
                requestedRightSpeed,
                maximumMagnitudeMetersPerSecond =
                    if (
                        activeTapeTrackingMode == TapeTrackingMode.FIXED_HEADING &&
                        activeFixedHeadingActuationPhaseLead.usesSpeedScheduledActuation
                    ) {
                        activeFixedHeadingSpeedTarget.maximumCommandSpeedMetersPerSecond
                    } else {
                        0.0
                    },
            )
            val controlCompletedAtNanos = System.nanoTime()
            val profiledFrameNanos = latestDetectionUiFrameNanos
            val usesNewProfiledFrame =
                profiledFrameNanos != 0L && profiledFrameNanos != lastControlledProfileFrameNanos
            flightProfiler.record(
                event = "control",
                atNanos = controlCompletedAtNanos,
                frameNanos = profiledFrameNanos,
                durationNanos = controlCompletedAtNanos - now,
                details = profileDetails(
                    "mode" to activeTapeTrackingMode,
                    "trigger" to activeTapeControlTrigger,
                    "newFrame" to usesNewProfiledFrame,
                    "uiToTickMs" to
                        if (usesNewProfiledFrame) {
                            (now - latestDetectionUiAtNanos).coerceAtLeast(0L) / 1_000_000.0
                        } else {
                            null
                        },
                    "preDecisionMs" to (decisionStartedAtNanos - now) / 1_000_000.0,
                    "decisionMs" to
                        (decisionCompletedAtNanos - decisionStartedAtNanos) / 1_000_000.0,
                    "decisionToCommandMs" to
                        (controlCompletedAtNanos - decisionCompletedAtNanos) / 1_000_000.0,
                    "phase" to decision.phase,
                    "forward" to requestedForwardSpeed,
                    "right" to requestedRightSpeed,
                    "yawRate" to yawRate,
                    "yawHeading" to yawHeadingTarget,
                    "rawAngle" to decision.rawAngleDegrees,
                    "controlledAngle" to decision.controlledAngleDegrees,
                    "pathBearing" to decision.pathBearingDegrees,
                    "circularFeedforwardYawRate" to
                        decision.circularFeedforwardYawRateDegreesPerSecond,
                    "circularTurnDirection" to decision.circularTurnDirection,
                    "offset" to decision.controlledOffsetFraction,
                    "groundSpeed" to groundSpeedMetersPerSecond,
                    "alongTrackSpeed" to decision.measuredAlongTrackSpeedMetersPerSecond,
                    "speedFeedbackSource" to if (visualFeedback) "visual_velocity" else "dji_listener",
                    "controlFeedback" to
                        (visualFeedback && decision.measuredAlongTrackSpeedMetersPerSecond != null),
                    "visualVelocityAttemptId" to fixedHeadingVisualVelocityAttemptId.takeIf { visualFeedback },
                    "speedFeedbackSampleNanos" to decision.speedFeedbackSampleAtNanos,
                    "speedFeedbackObservedNanos" to decision.speedFeedbackObservedAtNanos,
                    "speedFeedbackSampleAgeMs" to decision.speedFeedbackSampleAtNanos
                        .takeIf { it > 0L }?.let { (now - it).coerceAtLeast(0L) / 1_000_000.0 },
                    "speedFeedbackUnavailableReason" to decision.speedFeedbackUnavailableReason,
                    "speedFeedbackDirectionError" to decision.speedFeedbackDirectionErrorDegrees,
                    "speedFeedbackBoost" to decision.speedFeedbackBoostMetersPerSecond,
                    "commandTargetSpeed" to decision.commandTargetSpeedMetersPerSecond,
                    "appliedPhaseLeadDegrees" to decision.appliedPhaseLeadDegrees,
                    "actuationGain" to decision.actuationGain,
                    "scheduledTurnRateRadiansPerSecond" to decision.scheduledTurnRateRadiansPerSecond,
                    "desiredAlongTrackSpeedMetersPerSecond" to decision.desiredAlongTrackSpeedMetersPerSecond,
                    "maximumCommandSpeedMetersPerSecond" to decision.maximumCommandSpeedMetersPerSecond,
                    "actuationCompensationActive" to decision.actuationCompensationActive,
                    "heading" to aircraftHeadingDegrees,
                ),
            )
            if (usesNewProfiledFrame) lastControlledProfileFrameNanos = profiledFrameNanos
            val commandChanged =
                yawRate != commandedTapeYawRate ||
                    requestedForwardSpeed != commandedTapeForwardSpeed ||
                    requestedRightSpeed != commandedTapeRightSpeed
            if (commandChanged) {
                commandedTapeYawRate = yawRate
                commandedTapeForwardSpeed = requestedForwardSpeed
                commandedTapeRightSpeed = requestedRightSpeed
            }
            val stopped =
                yawRate == 0.0 && requestedForwardSpeed == 0.0 && requestedRightSpeed == 0.0
            if (
                (commandChanged && stopped) ||
                now - tapeCommandLoggedAtNanos >= TAPE_COMMAND_LOG_PERIOD_NANOS
            ) {
                tapeCommandLoggedAtNanos = now
                flightLog.write(
                    (
                        "tape tracking mode=$activeTapeTrackingMode " +
                            "yawControl=$activeCircularYawControlMode " +
                            "headingTarget=${yawHeadingTarget.logNumber(1)} " +
                            "rawAngle=${decision.rawAngleDegrees.logNumber(1)} " +
                            "angle=${decision.controlledAngleDegrees.logNumber(1)} " +
                            "rawOffset=${decision.rawOffsetFraction.logNumber(3)} " +
                            "offset=${decision.controlledOffsetFraction.logNumber(3)} " +
                            "offsetRate=%+.3f ppYaw=%+.1f yaw=%+.1f " +
                            "rawCurv=${decision.rawCurvaturePerMeter.logNumber(3)} " +
                            "predCurv=${decision.predictedCurvaturePerMeter.logNumber(3)} " +
                            "trustedCurv=${decision.trustedCurvaturePerMeter.logNumber(3)} " +
                            "instCap=${decision.instantaneousCurvatureSpeedCapMetersPerSecond.logNumber(2)} " +
                            "trustedCap=${decision.trustedCurvatureSpeedCapMetersPerSecond.logNumber(2)} " +
                            "profile=%.2f accel=%.2f forward=%.2f right=%+.2f gs=%.2f " +
                            "travel=${usableBodyTravelDirectionDegrees(now).logNumber(1)} " +
                            "phase=${decision.phase}"
                        ).format(
                        decision.offsetRatePerSecond,
                        decision.purePursuitYawRateDegreesPerSecond,
                        yawRate,
                        decision.profileForwardSpeedMetersPerSecond,
                        decision.accelerationLimitedForwardSpeedMetersPerSecond,
                        requestedForwardSpeed,
                        requestedRightSpeed,
                        groundSpeedMetersPerSecond,
                    ),
                )
            }
            if (decision.phase != renderedTapeTrackingPhase) {
                fixedHeadingVisualVelocityAttemptId?.let {
                    visualVelocityDiagnostics?.updatePhase(decision.phase.name, it)
                }
                renderedTapeTrackingPhase = decision.phase
                val trackingName = tapeTrackingName(activeTapeTrackingMode)
                val status = when (decision.phase) {
                    TapeTrackingPhase.RECENTERING -> "$trackingName：循跡資料穩定中"
                    TapeTrackingPhase.RECOVERING_AFTER_TURN ->
                        "$trackingName：回轉完成，低速前移重新取得膠帶"
                    TapeTrackingPhase.RECOVERING_FRAME_STREAM ->
                        "$trackingName：相機影格恢復中，保持懸停"
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
        }
    }
    private val tapeTrackingPeriodicRunnable = object : Runnable {
        override fun run() {
            refreshVisualVelocityContext()
            activeTapeControlTrigger = TapeControlTrigger.PERIODIC
            tapeTrackingControlRunnable.run()
            if (tapeTracking.enabled && !tapeEndpointTurn) {
                mainHandler.postDelayed(this, TAPE_TRACKING_TICK_MS)
            }
        }
    }



    private fun tapeTrackingName(
        mode: TapeTrackingMode? = activeTapeTrackingMode,
        circularTrackingSpeed: CircularTrackingSpeed = activeCircularTrackingSpeed,
        circularYawControlMode: CircularYawControlMode = activeCircularYawControlMode,
        fixedHeadingActuationPhaseLead: FixedHeadingActuationPhaseLead =
            activeFixedHeadingActuationPhaseLead,
        fixedHeadingSpeedTarget: FixedHeadingSpeedTarget = activeFixedHeadingSpeedTarget,
    ): String =
        when (mode) {
            TapeTrackingMode.CIRCULAR ->
                when {
                    circularYawControlMode == CircularYawControlMode.HEADING ->
                        "ANGLE 前視點修正版・0.10 m/s"
                    else ->
                        "方案 C：Ø1.5m 定曲率＋視覺修正・0.70 m/s"
                }
            TapeTrackingMode.FIXED_HEADING ->
                when (fixedHeadingActuationPhaseLead) {
                    FixedHeadingActuationPhaseLead.LOW_SPEED_14 ->
                        "B0：定向滑行・0.60 m/s"
                    FixedHeadingActuationPhaseLead.CURVATURE_FEEDFORWARD_16 ->
                        "方案 B2：曲率前饋・巡航 1.60 m/s・最大 1.90 m/s"
                    FixedHeadingActuationPhaseLead.CURVATURE_FEEDFORWARD_16_VISUAL ->
                        "方案 B3：視覺速度回饋・巡航 1.60 m/s・最大 1.90 m/s"
                    FixedHeadingActuationPhaseLead.SPEED_SCHEDULED_VISUAL ->
                        fixedHeadingScheduledActuationLabel(fixedHeadingSpeedTarget)
                    else ->
                        "方案 B：定向滑行・%.2f m/s・相位超前 %.0f°".format(
                            fixedHeadingActuationPhaseLead.targetSpeedMetersPerSecond,
                            fixedHeadingActuationPhaseLead.degrees,
                        )
                }
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
        mainHandler.removeCallbacks(tapeTrackingPeriodicRunnable)
        virtualStick.setHorizontalVelocity(0.0, 0.0)
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
        fixedHeadingVisualVelocityAttemptId?.let {
            visualVelocityDiagnostics?.stop(it)
            fixedHeadingVisualVelocityAttemptId = null
            refreshVisualVelocityContext()
        }
        val wasActive =
            tapeTrackingStartPending || tapeTracking.enabled ||
                renderedTapeTrackingPhase != TapeTrackingPhase.DISABLED ||
                avoidance.closedConfirmed
        if (wasActive) {
            flightProfiler.record(
                event = "tracking_stop",
                details = profileDetails(
                    "mode" to activeTapeTrackingMode,
                    "elapsedMs" to
                        if (tapeTrackingStartedAtNanos == 0L) {
                            null
                        } else {
                            (System.nanoTime() - tapeTrackingStartedAtNanos) / 1_000_000.0
                        },
                    "reason" to message,
                ),
            )
        }
        if (tapeEndpointTurn) {
            tapeEndpointTurn = false
            headingTurn = null
            turnCommandStartedAtNanos = 0L
            turnAuthoritySeen = false
            mainHandler.removeCallbacks(headingTurnTickRunnable)
        }
        tapeTracking.stop()
        angleHeadingController.reset()
        tapeDetector?.setDetectionMode(TapeDetectionMode.PATH)
        tapeDetector?.endTrackingSession()
        tapeTrackingStartPending = false
        mainHandler.removeCallbacks(tapeTrackingPeriodicRunnable)
        virtualStick.setYawRate(0.0)
        virtualStick.setHorizontalVelocity(0.0, 0.0)
        commandedTapeYawRate = 0.0
        commandedTapeForwardSpeed = 0.0
        commandedTapeRightSpeed = 0.0
        tapeTrackingAuthoritySeen = false
        tapeTrackingStartedAtNanos = 0L
        fixedHeadingReferenceDegrees = null
        if (captureAutoArmedByTracking) {
            mainHandler.removeCallbacks(autoCaptureDisarmRunnable)
            mainHandler.postDelayed(
                autoCaptureDisarmRunnable,
                AUTO_CAPTURE_DISARM_DELAY_MS,
            )
            flightLog.write(
                "automatic tape capture trailing window scheduled delayMs=" +
                    AUTO_CAPTURE_DISARM_DELAY_MS,
            )
        }
        renderedTapeTrackingPhase = TapeTrackingPhase.DISABLED
        tapeCommandLoggedAtNanos = 0L
        activeTapeTrackingMode = null
        activeCircularTrackingSpeed = CircularTrackingSpeed.FAST
        activeCircularYawControlMode = CircularYawControlMode.RATE
        activeFixedHeadingActuationPhaseLead = FixedHeadingActuationPhaseLead.DEGREES_0
        activeFixedHeadingSpeedTarget = FixedHeadingSpeedTarget.BASELINE
        if (::tapeTrackingButton.isInitialized) tapeTrackingButton.text = "直線黑膠帶追蹤"
        if (::circularTapeTrackingButton.isInitialized) {
            circularTapeTrackingButton.text =
                "方案 C：Ø1.5m 定曲率＋視覺修正・0.70 m/s"
        }
        if (::angleCircularTapeTrackingButton.isInitialized) {
            angleCircularTapeTrackingButton.text = "ANGLE 前視點修正版・0.10 m/s"
        }
        diagnosticTurnCycleTimer.reset()
        if (::fixedHeadingLowSpeedButton.isInitialized) {
            fixedHeadingLowSpeedButton.text = "B0：定向滑行・0.60 m/s"
        }
        if (::fixedHeadingFourteenPhaseLeadButton.isInitialized) {
            fixedHeadingFourteenPhaseLeadButton.text = "方案 B：定向滑行・相位超前 14°"
        }
        if (::fixedHeadingFasterPhaseLeadButton.isInitialized) {
            fixedHeadingFasterPhaseLeadButton.text = "方案 B：更快・1.60 m/s・超前 16°"
        }
        if (::fixedHeadingCurvatureFeedforwardButton.isInitialized) {
            fixedHeadingCurvatureFeedforwardButton.text =
                "方案 B2：曲率前饋・巡航 1.60 m/s・最大 1.90 m/s"
        }
        if (::fixedHeadingFastCruiseButton.isInitialized) {
            fixedHeadingFastCruiseButton.text =
                "方案 B3：視覺速度回饋・巡航 1.60 m/s・最大 1.90 m/s"
        }
        if (::fixedHeadingScheduledActuationButton.isInitialized) {
            fixedHeadingScheduledActuationButton.text = fixedHeadingScheduledActuationLabel()
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
        restoreBrakeAfterAutonomousControl(message, release, "tape tracking")
    }

    private fun restoreBrakeAfterAutonomousControl(
        message: String,
        release: Boolean,
        logContext: String,
    ) {
        if (!aircraftConnected) {
            render(message)
            return
        }
        avoidanceCheck.ensureBrake { status ->
            runOnUiThread {
                avoidance = status
                flightLog.write("$logContext restore ${status.summary} detail=${status.detail}")
                val restoredMessage = if (status.brakeConfirmed) {
                    "$message；BRAKE 避障已恢復"
                } else {
                    "$message；警告：${status.warning ?: "BRAKE 避障恢復失敗"}"
                }
                refreshHorizontalSafety("BRAKE read-back")
                if (
                    release && stickOwned && !stickTransitionPending &&
                    !leftStickActive && !rightStickActive && !holdingHeight &&
                    headingTurn == null && !anotherFlightControlActive()
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
        if (
            isDeflected &&
            (hardwareLatencyStartPending || hardwareLatencyPulseSequence != null)
        ) {
            stopHardwareLatencyTest(
                "畫面搖桿已接管，${activeHorizontalPulseName()}已停止",
                release = false,
            )
        }
        if (
            isDeflected &&
            (mathematicalCircleStartPending || mathematicalCircleController != null)
        ) {
            stopMathematicalCircle("畫面搖桿已接管，數學圓周已停止", release = false)
        }
        if (isDeflected) {
            stopTiltFlight("畫面搖桿已接管，傾角實驗已停止", release = false)
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
        if (isDeflected && yawLapTestActive) {
            finishHeadingTurn("操作者接管，航向模式原地旋轉已停止", release = false)
        }
        if (
            side == StickSide.LEFT && (x != 0.0 || y != 0.0) &&
            headingTurn != null && !yawLapTestActive
        ) {
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
            quarterArcController != null || tapeTracking.enabled || hardwareLatencyStartPending ||
            hardwareLatencyPulseSequence != null || mathematicalCircleStartPending ||
            mathematicalCircleController != null || tiltFlightRun != null || !stickOwned
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
        if (straightLineReleasePending && !straightLineTracePending &&
            pendingAcquireCallbacks == 0 && !stickTransitionPending &&
            stickStatus.isReleased && lastStickStatusAtNanos >= lastReleaseRequestedAtNanos
        ) {
            straightLineReleasePending = false
            stickOwned = false
            resetHorizontalPulseButtonLabels()
        }
        val ready = registered && aircraftConnected
        val hardwareLatencyActive =
            hardwareLatencyStartPending || hardwareLatencyPulseSequence != null
        val mathematicalCircleActive =
            mathematicalCircleStartPending || mathematicalCircleController != null
        val tiltRun = tiltFlightRun
        val tiltActive = tiltRun != null
        val straightIdle = !hardwareLatencyActive &&
            !straightLineReleasePending && !stickTransitionPending && !anotherFlightControlActive()
        straightRehearsalButton.available = straightIdle ||
            hardwareLatencyActive && horizontalPulseExperiment == HorizontalPulseExperiment.STRAIGHT_REHEARSAL
        straightMeasurementButton.available = straightIdle ||
            hardwareLatencyActive && horizontalPulseExperiment == HorizontalPulseExperiment.STRAIGHT_MEASUREMENT
        if (straightLineReleasePending) {
            straightRehearsalButton.text = "重試交還控制權"
            straightRehearsalButton.available = !straightLineTracePending &&
                !stickTransitionPending && pendingAcquireCallbacks == 0 && !landingAuthorityWaitPending
            straightMeasurementButton.text = "等待回交・禁止重啟"
            straightMeasurementButton.available = false
        }
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
        yawLapTestButton.available =
            yawLapTestActive ||
                (!turning && !quarterArcActive && !holdingHeight && !tapeTracking.enabled &&
                    !hardwareLatencyActive && !mathematicalCircleActive && !tiltActive && ready && flying)
        val heightButtonAvailable =
            ready && flying && !holdingHeight && !turning && !quarterArcActive &&
                !tapeTracking.enabled && !hardwareLatencyActive && !mathematicalCircleActive && !tiltActive
        holdButton.available = heightButtonAvailable
        oneMeterHoldButton.available = heightButtonAvailable
        captureButton.available = captureRecorder != null
        virtualStickFrameRateButton.available =
            !stickOwned && !stickTransitionPending && !stickStatus.enabled &&
                !anotherFlightControlActive()
        hardwareLatencyButton.available =
            hardwareLatencyActive && horizontalPulseExperiment == HorizontalPulseExperiment.HARDWARE_LATENCY ||
                ready && flying && !anotherFlightControlActive()
        mathematicalSkatingCircleButton.available =
            (mathematicalCircleActive && mathematicalCircleMode == MathematicalCircleMode.SKATING) ||
                (!mathematicalCircleActive && ready && flying && !anotherFlightControlActive())
        mathematicalRacingCircleButton.available =
            (mathematicalCircleActive && mathematicalCircleMode == MathematicalCircleMode.RACING) ||
                (!mathematicalCircleActive && ready && flying && !anotherFlightControlActive())
        mathematicalFastRacingCircleButton.available =
            (mathematicalCircleActive && mathematicalCircleMode == MathematicalCircleMode.RACING_FAST) ||
                (!mathematicalCircleActive && ready && flying && !anotherFlightControlActive())
        tiltStraightButton.available =
            if (tiltRun != null) tiltRun.mode == TiltFlightMode.STRAIGHT && !tiltRun.stopping
            else ready && flying && !stickTransitionPending && !anotherFlightControlActive()
        tiltCircleButton.available =
            if (tiltRun != null) tiltRun.mode == TiltFlightMode.CIRCLE && !tiltRun.stopping
            else ready && flying && !stickTransitionPending && !anotherFlightControlActive()
        tiltSkatingCircleButton.available =
            if (tiltRun != null) tiltRun.mode == TiltFlightMode.SKATING_CIRCLE && !tiltRun.stopping
            else ready && flying && !stickTransitionPending && !anotherFlightControlActive()
        cameraDownButton.available = ready && !cameraPitchCommandPending
        val tapeTrackingCanStart =
            ready && flying && !turning && !quarterArcActive && !holdingHeight &&
                !hardwareLatencyActive && !mathematicalCircleActive && !tiltActive
        tapeTrackingButton.available =
            (tapeTracking.enabled && activeTapeTrackingMode == TapeTrackingMode.STRAIGHT) ||
                (!tapeTracking.enabled && tapeTrackingCanStart)
        circularTapeTrackingButton.available =
            (
                tapeTracking.enabled &&
                    activeTapeTrackingMode == TapeTrackingMode.CIRCULAR &&
                    activeCircularTrackingSpeed == CircularTrackingSpeed.FAST &&
                    activeCircularYawControlMode == CircularYawControlMode.RATE
                ) ||
                (!tapeTracking.enabled && tapeTrackingCanStart)
        angleCircularTapeTrackingButton.available =
            (
                tapeTracking.enabled &&
                    activeTapeTrackingMode == TapeTrackingMode.CIRCULAR &&
                    activeCircularTrackingSpeed == CircularTrackingSpeed.FAST &&
                    activeCircularYawControlMode == CircularYawControlMode.HEADING
                ) ||
                (!tapeTracking.enabled && tapeTrackingCanStart)
        fixedHeadingLowSpeedButton.available =
            (
                tapeTracking.enabled &&
                    activeTapeTrackingMode == TapeTrackingMode.FIXED_HEADING &&
                    activeFixedHeadingActuationPhaseLead ==
                    FixedHeadingActuationPhaseLead.LOW_SPEED_14
                ) ||
                (!tapeTracking.enabled && tapeTrackingCanStart)
        fixedHeadingFourteenPhaseLeadButton.available =
            (
                tapeTracking.enabled &&
                    activeTapeTrackingMode == TapeTrackingMode.FIXED_HEADING &&
                    activeFixedHeadingActuationPhaseLead ==
                    FixedHeadingActuationPhaseLead.DEGREES_14
                ) ||
                (!tapeTracking.enabled && tapeTrackingCanStart)
        fixedHeadingFasterPhaseLeadButton.available =
            (
                tapeTracking.enabled &&
                    activeTapeTrackingMode == TapeTrackingMode.FIXED_HEADING &&
                    activeFixedHeadingActuationPhaseLead ==
                    FixedHeadingActuationPhaseLead.DEGREES_16
                ) ||
                (!tapeTracking.enabled && tapeTrackingCanStart)
        fixedHeadingCurvatureFeedforwardButton.available =
            (
                tapeTracking.enabled &&
                    activeTapeTrackingMode == TapeTrackingMode.FIXED_HEADING &&
                    activeFixedHeadingActuationPhaseLead ==
                    FixedHeadingActuationPhaseLead.CURVATURE_FEEDFORWARD_16
                ) ||
                (!tapeTracking.enabled && tapeTrackingCanStart)
        fixedHeadingFastCruiseButton.available =
            (
                tapeTracking.enabled &&
                    activeTapeTrackingMode == TapeTrackingMode.FIXED_HEADING &&
                    activeFixedHeadingActuationPhaseLead ==
                    FixedHeadingActuationPhaseLead.CURVATURE_FEEDFORWARD_16_VISUAL
                ) ||
                (!tapeTracking.enabled && tapeTrackingCanStart)
        fixedHeadingScheduledActuationButton.available =
            (
                tapeTracking.enabled &&
                    activeTapeTrackingMode == TapeTrackingMode.FIXED_HEADING &&
                    activeFixedHeadingActuationPhaseLead.usesSpeedScheduledActuation
                ) ||
                (!tapeTracking.enabled && tapeTrackingCanStart &&
                    !anotherFlightControlActive() && !yawLapTestActive && !stickTransitionPending)
        fixedHeadingSpeedTargetButton.available = canSelectFixedHeadingSpeedTarget()
        curvedOutAndBackTrackingButton.available =
            (tapeTracking.enabled &&
                activeTapeTrackingMode == TapeTrackingMode.CURVED_OUT_AND_BACK) ||
                (!tapeTracking.enabled && tapeTrackingCanStart)
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
            append(" · turn=").append(
                headingTurn?.let { "%.0f/%.0f°".format(it.progressDegrees, it.targetDegrees) } ?: "off",
            )
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

    private fun updateFirmwareConnection(readout: FirmwareReadout, connected: Boolean) = runOnUiThread {
        if (activityDestroying || readout.connected == connected) return@runOnUiThread
        readout.connected = connected
        if (connected) {
            readFirmwareVersion(readout)
        } else {
            readout.generation += 1
            readout.status = "未連線"
            recordFlightState(readout.logName, null)
            updateFirmwareText()
        }
    }

    private fun updateFirmwareText() {
        firmwareView.text = "韌體（點此重讀，僅讀取）：\n" +
            "飛機 ${aircraftFirmware.status} · RC ${remoteControllerFirmware.status}"
    }

    private fun readFirmwareVersion(readout: FirmwareReadout) {
        if (activityDestroying) return
        val generation = ++readout.generation
        if (!readout.connected) {
            readout.status = "未連線"
            flightLog.write("firmware read skipped key=${readout.logName}: disconnected")
            updateFirmwareText()
            return
        }
        readout.status = "讀取中…"
        updateFirmwareText()
        flightLog.write("firmware read requested key=${readout.logName}")

        fun complete(version: String?, error: String?) = runOnUiThread {
            // A previous connection or manual read must not replace the current result.
            if (activityDestroying || !readout.connected || generation != readout.generation) {
                return@runOnUiThread
            }
            val value = version?.trim()?.takeIf { it.isNotEmpty() }
            readout.status = value ?: "讀取失敗"
            recordFlightState(readout.logName, value)
            recordFlightState("${readout.logName}Error", if (value == null) error ?: "empty version" else null)
            updateFirmwareText()
        }

        // Firmware keys are get-only: listening does not retrieve their values.
        runCatching {
            KeyManager.getInstance().getValue(
                readout.key,
                object : CommonCallbacks.CompletionCallbackWithParam<String> {
                    override fun onSuccess(result: String) {
                        complete(result, null)
                    }

                    override fun onFailure(error: IDJIError) {
                        complete(null, error.toString())
                    }
                },
            )
        }.onFailure { error -> complete(null, error.toString()) }
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
            updateFirmwareConnection(aircraftFirmware, connected == true)
            aircraftHeadingDegrees = null
            aircraftHeadingAtNanos = 0L
            aircraftVelocityX = 0.0
            aircraftVelocityY = 0.0
            aircraftVelocityZ = 0.0
            groundSpeedMetersPerSecond = 0.0
            aircraftVelocityAtNanos = 0L
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
                    visualVelocityDiagnostics?.stop()
                    visualVelocityFrameContext = null
                    fixedHeadingVisualVelocityAttemptId = null
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
            KeyTools.createKey(RemoteControllerKey.KeyConnection),
            this,
            true,
        ) { _, connected ->
            updateFirmwareConnection(remoteControllerFirmware, connected == true)
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
            aircraftVelocityKey,
            this,
            true,
        ) { _, velocity ->
            publishAircraftVelocity(velocity)
        }
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyFlightMode),
            this,
            true,
        ) { _, mode ->
            runOnUiThread {
                flightModeName = mode?.name ?: "UNKNOWN"
                recordFlightState("flightMode", flightModeName)
            }
        }
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyFCFlightMode),
            this,
            true,
        ) { _, mode ->
            runOnUiThread {
                fcFlightModeName = mode?.name ?: "UNKNOWN"
                recordFlightState("fcFlightMode", fcFlightModeName)
            }
        }
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyRemoteControllerFlightMode),
            this,
            true,
        ) { _, mode ->
            runOnUiThread {
                remoteControllerFlightModeName = mode?.name ?: "UNKNOWN"
                recordFlightState("remoteControllerFlightMode", remoteControllerFlightModeName)
            }
        }
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyFCRemoteControllerSwitchMode),
            this,
            true,
        ) { _, mode ->
            runOnUiThread {
                remoteControllerSwitchModeName = mode?.name ?: "UNKNOWN"
                recordFlightState("remoteControllerSwitchMode", remoteControllerSwitchModeName)
            }
        }
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyLimitFlightSpeed),
            this,
            true,
        ) { _, speed ->
            runOnUiThread {
                flightLimitSpeedMetersPerSecond = speed?.takeIf(Double::isFinite)
                recordFlightState("limitFlightSpeedMps", flightLimitSpeedMetersPerSecond)
            }
        }
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyIsVisionSensorUsed),
            this,
            true,
        ) { _, used ->
            runOnUiThread {
                visionSensorUsed = used
                recordFlightState("visionSensorUsed", used)
            }
        }
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyIsWithoutVisionPosition),
            this,
            true,
        ) { _, unavailable ->
            runOnUiThread {
                visionPositionUnavailable = unavailable
                recordFlightState("visionPositionUnavailable", unavailable)
            }
        }
        keyManager.listen(
            KeyTools.createKey(FlightControllerKey.KeyVisionAvoidEnable),
            this,
            true,
        ) { _, enabled ->
            runOnUiThread {
                visionAvoidanceEnabled = enabled
                recordFlightState("visionAvoidanceEnabled", enabled)
            }
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
     * DJI uses 60,000 mm to mean "not detected", so an all-sentinel sample is a
     * valid callback with no actionable obstacle. At sub-metre altitude the
     * Mini 4 Pro reports the floor in almost every horizontal azimuth. A dense
     * low-altitude ring with enough floor-height evidence is therefore removed.
     * Sparse ranges and objects substantially closer than the floor remain stop
     * conditions.
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
            if (obstaclePreviousCallbackAtNanos != 0L) {
                val callbackGapMs = (now - obstaclePreviousCallbackAtNanos) / 1_000_000L
                obstacleMaximumCallbackGapMsSinceLog =
                    maxOf(obstacleMaximumCallbackGapMsSinceLog, callbackGapMs)
            }
            obstaclePreviousCallbackAtNanos = now
            obstacleCallbacksSinceLog += 1
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
                val callbackCount = obstacleCallbacksSinceLog
                val maximumCallbackGapMs = obstacleMaximumCallbackGapMsSinceLog
                obstacleCallbacksSinceLog = 0
                obstacleMaximumCallbackGapMsSinceLog = 0L
                obstacleLoggedAtNanos = now
                flightLog.write(
                    "obstacle callbacks=$callbackCount maxGapMs=$maximumCallbackGapMs " +
                        "actionableMinMm=${summary.nearestActionableMm ?: "none"} " +
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
     * Tape tracking disables DJI avoidance and applies the app's larger
     * autonomous clearance whenever DJI reports an actionable obstacle.
     * Callback freshness remains diagnostic until its real cadence is measured.
     */
    private fun tapeTrackingStopReason(): String? {
        if (!avoidance.closedConfirmed) {
            return avoidance.warning ?: "黑膠帶追蹤停止：無法確認飛機避障已關閉"
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
        if (hardwareLatencyPulseSequence != null) {
            hardwareLatencyRuntimeFailure(System.nanoTime())?.let { reason ->
                stopHardwareLatencyTest("${activeHorizontalPulseName()}安全停止：$reason")
            }
        }
        if (mathematicalCircleController != null) {
            noVisionFlightRuntimeFailure(System.nanoTime())?.let { reason ->
                stopMathematicalCircle("數學圓周安全停止：$reason")
            }
        }
        checkTiltFlightSafety(System.nanoTime())
        if (changed || trigger == "BRAKE read-back") {
            render(failure ?: "BRAKE 已確認，水平避障資料可用")
        }
    }

    private fun stopHorizontalActuation(reason: String) {
        rightStickActive = false
        virtualStick.setHorizontalVelocity(0.0, 0.0)
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
            if (isStraightLineTest()) refreshVisualVelocityContext()
            mainHandler.postDelayed(this, HORIZONTAL_WATCHDOG_MS)
        }
    }

    /** Which key the current height came from; a better source may replace a worse one. */
    private enum class HeightSource { NONE, LOCATION_3D, ALTITUDE }
    private fun recordFlightState(name: String, value: Any?) {
        val receivedAtNanos = System.nanoTime()
        flightProfiler.record(
            event = "flight_state",
            atNanos = receivedAtNanos,
            details = profileDetails("name" to name, "value" to value),
        )
        flightLog.write("flight state $name=${value ?: "null"}")
    }


    private fun publishAircraftAttitude(attitude: Attitude?) {
        val heading = attitude?.yaw?.takeIf(Double::isFinite) ?: return
        val pitch = attitude.pitch?.takeIf(Double::isFinite)
        val roll = attitude.roll?.takeIf(Double::isFinite)
        val receivedAtNanos = System.nanoTime()
        aircraftHeadingDegrees = heading
        aircraftHeadingAtNanos = receivedAtNanos
        aircraftPitchDegrees = pitch
        aircraftRollDegrees = roll
        flightProfiler.record(
            event = "attitude",
            atNanos = receivedAtNanos,
            details = profileDetails(
                "heading" to heading,
                "pitch" to pitch,
                "roll" to roll,
            ),
        )
        headingTurn?.update(heading)
        driveHeadingTurn()
        // Heading samples close both the endpoint half-turn and the standalone
        // yaw-capability test; neither estimates completion from commanded time.
        quarterArcController?.updateHeading(heading)
        driveQuarterArc()
    }

    private fun publishAircraftVelocity(
        velocity: Velocity3D?,
        receivedAtNanos: Long = System.nanoTime(),
    ): Boolean {
        val x = velocity?.x?.takeIf(Double::isFinite) ?: return false
        val y = velocity.y?.takeIf(Double::isFinite) ?: return false
        val z = velocity.z?.takeIf(Double::isFinite) ?: return false
        val measuredSpeed = hypot(x, y)
        val previousReceivedAtNanos = synchronized(aircraftVelocityLock) {
            val previous = aircraftVelocityAtNanos
            aircraftVelocityX = x
            aircraftVelocityY = y
            aircraftVelocityZ = z
            groundSpeedMetersPerSecond = measuredSpeed
            aircraftVelocityAtNanos = receivedAtNanos
            previous
        }
        flightProfiler.record(
            event = "velocity",
            atNanos = receivedAtNanos,
            details = profileDetails(
                "source" to "listener",
                "x" to x,
                "y" to y,
                "z" to z,
                "groundSpeed" to measuredSpeed,
                "intervalMs" to
                    previousReceivedAtNanos
                        .takeIf { it > 0L }
                        ?.let { (receivedAtNanos - it).coerceAtLeast(0L) / 1_000_000.0 },
            ),
        )
        return true
    }

    private fun usableBodyTravelDirectionDegrees(nowNanos: Long): Double? {
        if (
            aircraftVelocityAtNanos == 0L ||
            nowNanos - aircraftVelocityAtNanos > AIRCRAFT_VELOCITY_STALE_NANOS ||
            groundSpeedMetersPerSecond < MIN_DIRECTIONAL_GROUND_SPEED_METERS_PER_SECOND
        ) {
            return null
        }
        val heading = aircraftHeadingDegrees ?: return null
        val earthTravelDegrees = Math.toDegrees(atan2(aircraftVelocityY, aircraftVelocityX))
        return wrapDegrees(earthTravelDegrees - heading)
    }

    private fun speedFeedbackInputFailure(nowNanos: Long): String? = when {
        aircraftVelocityAtNanos == 0L -> "MISSING_VELOCITY"
        nowNanos - aircraftVelocityAtNanos > AIRCRAFT_VELOCITY_STALE_NANOS -> "VELOCITY_STALE"
        groundSpeedMetersPerSecond < MIN_DIRECTIONAL_GROUND_SPEED_METERS_PER_SECOND -> "SPEED_TOO_LOW"
        aircraftHeadingDegrees == null -> "MISSING_HEADING"
        else -> null
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
            tapeTrackingStartPending || hardwareLatencyStartPending ||
            hardwareLatencyPulseSequence != null || mathematicalCircleStartPending ||
            mathematicalCircleController != null || tiltFlightRun != null || leftStickActive || rightStickActive ||
            straightLineReleasePending || landingAuthorityWaitPending || pendingAcquireCallbacks > 0

    /**
     * All horizontal experiments share the authority pipeline. Straight experiments
     * bypass flight-condition gates; pulse experiments retain their checks.
     */
    private fun toggleHardwareLatencyTest() {
        toggleHorizontalPulseExperiment(HorizontalPulseExperiment.HARDWARE_LATENCY)
    }


    private fun toggleHorizontalPulseExperiment(requested: HorizontalPulseExperiment) {
        if (hardwareLatencyStartPending || hardwareLatencyPulseSequence != null) {
            if (horizontalPulseExperiment == requested) {
                stopHardwareLatencyTest("操作者停止${horizontalPulseName(requested)}")
            } else {
                render("${activeHorizontalPulseName()}正在使用控制權")
            }
            return
        }
        startHorizontalPulseExperiment(requested)
    }

    private fun startHorizontalPulseExperiment(experiment: HorizontalPulseExperiment) {
        (if (isStraightLineTest(experiment)) null else hardwareLatencyStartFailure())?.let { reason ->
            flightLog.write("horizontal pulse refused: $reason")
            render(reason)
            return
        }

        val generation = ++horizontalPulseGeneration
        horizontalPulseExperiment = experiment
        hardwareLatencyStartPending = true
        hardwareLatencyArmedAtNanos = System.nanoTime()
        if (isStraightLineTest(experiment)) {
            velocityReadDiagnostics.start(
                checkNotNull(straightLineAttemptId), "ACQUIRING", straightLineInitialHeading,
            )
            visualVelocityDiagnostics?.start(
                checkNotNull(straightLineAttemptId), "ACQUIRING", straightLineInitialHeading,
            )
            refreshVisualVelocityContext()
            mainHandler.removeCallbacks(velocityReadTickRunnable)
            mainHandler.post(velocityReadTickRunnable)
        }
        when (experiment) {
            HorizontalPulseExperiment.HARDWARE_LATENCY ->
                hardwareLatencyButton.text = "停止硬體延遲脈衝"
            HorizontalPulseExperiment.STRAIGHT_REHEARSAL,
            HorizontalPulseExperiment.STRAIGHT_MEASUREMENT ->
                straightLineButton(experiment).text = "停止"
        }
        setHardwareLatencyMarker(white = false)
        val experimentName = horizontalPulseName(experiment)
        render(
            when (experiment) {
                HorizontalPulseExperiment.HARDWARE_LATENCY ->
                    "正在取得控制權；取得後先懸停 3 秒，再執行 10 組前後脈衝…"
                HorizontalPulseExperiment.STRAIGHT_REHEARSAL ->
                    "取得控制權後直接向前 0.30 m/s；按「停止」結束，最長 20 秒"
                HorizontalPulseExperiment.STRAIGHT_MEASUREMENT ->
                    "取得控制權後直接向前 0.50 m/s；按「停止」結束，最長 20 秒"
            },
        )
        acquireControlLink(
            onFailure = { reason ->
                if (generation != horizontalPulseGeneration) return@acquireControlLink
                if (isStraightLineTest(experiment)) {
                    stopHardwareLatencyTest("$experimentName 未啟動：$reason", release = false)
                    return@acquireControlLink
                }
                hardwareLatencyStartPending = false
                horizontalPulseExperiment = null
                resetHorizontalPulseButtonLabels()
                render("$experimentName 未啟動：$reason")
            },
            shouldContinue = { hardwareLatencyStartPending && generation == horizontalPulseGeneration },
        ) {
            if (!hardwareLatencyStartPending || generation != horizontalPulseGeneration) {
                if (stickOwned && !activityDestroying) {
                    releaseControlLink { error ->
                        render(error?.let { "已取消脈衝，但釋放控制權失敗：$it" } ?: "速度脈衝已取消")
                    }
                }
                return@acquireControlLink
            }
            hardwareLatencyStartPending = false
            (if (isStraightLineTest(experiment)) null else hardwareLatencyStartFailure())?.let { reason ->
                flightLog.write("horizontal pulse aborted after acquire: $reason")
                horizontalPulseExperiment = null
                resetHorizontalPulseButtonLabels()
                releaseControlLink { error ->
                    render(
                        error?.let { "$experimentName 未啟動：$reason（釋放控制權失敗：$it）" }
                            ?: "$experimentName 未啟動：$reason",
                    )
                }
                return@acquireControlLink
            }

            hardwareLatencyPulseSequence =
                when (experiment) {
                    HorizontalPulseExperiment.HARDWARE_LATENCY -> HardwareLatencyPulseSequence()
                    HorizontalPulseExperiment.STRAIGHT_REHEARSAL,
                    HorizontalPulseExperiment.STRAIGHT_MEASUREMENT -> {
                        val config = checkNotNull(straightLineTestConfig)
                        StraightLineSpeedSequence(
                            direction = config.direction,
                            speedMetersPerSecond = config.speedMetersPerSecond,
                            maximumCruiseNanos = config.maximumCruiseSeconds * 1_000_000_000L,
                            baselineNanos = 0L,
                        )
                    }
                }
            hardwareLatencySyncMarker.visibility = if (isStraightLineTest(experiment)) View.GONE else View.VISIBLE
            hardwareLatencyAuthoritySeen = false
            hardwareLatencyArmedAtNanos = System.nanoTime()
            hardwareLatencyStartedAtNanos = 0L
            virtualStick.setHorizontalVelocity(0.0, 0.0)
            virtualStick.setClimbRate(0.0)
            virtualStick.setYawRate(0.0)
            if (isStraightLineTest(experiment)) {
                straightLineInitialHeading?.let { virtualStick.setYawHeading(it) }
            }
            val direction =
                when (experiment) {
                    HorizontalPulseExperiment.HARDWARE_LATENCY -> null
                    HorizontalPulseExperiment.STRAIGHT_REHEARSAL,
                    HorizontalPulseExperiment.STRAIGHT_MEASUREMENT ->
                        checkNotNull(straightLineTestConfig).direction.displayName
                }
            val speed =
                when (experiment) {
                    HorizontalPulseExperiment.HARDWARE_LATENCY ->
                        HardwareLatencyPulseSequence.DEFAULT_SPEED_METERS_PER_SECOND
                    HorizontalPulseExperiment.STRAIGHT_REHEARSAL,
                    HorizontalPulseExperiment.STRAIGHT_MEASUREMENT ->
                        checkNotNull(straightLineTestConfig).speedMetersPerSecond
                }
            val pulseNanos =
                when (experiment) {
                    HorizontalPulseExperiment.HARDWARE_LATENCY ->
                        HardwareLatencyPulseSequence.DEFAULT_PULSE_NANOS
                    HorizontalPulseExperiment.STRAIGHT_REHEARSAL,
                    HorizontalPulseExperiment.STRAIGHT_MEASUREMENT ->
                        checkNotNull(straightLineTestConfig).maximumCruiseSeconds * 1_000_000_000L
                }
            flightProfiler.record(
                event = horizontalPulseEventName(experiment, "armed"),
                atNanos = hardwareLatencyArmedAtNanos,
                details = profileDetails(
                    "attemptId" to straightLineAttemptId.takeIf { isStraightLineTest(experiment) },
                    "direction" to direction,
                    "cycles" to
                        HardwareLatencyPulseSequence.DEFAULT_CYCLE_COUNT.takeIf {
                            experiment == HorizontalPulseExperiment.HARDWARE_LATENCY
                        },
                    "speed" to speed,
                    "pulseMs" to pulseNanos / 1_000_000L,
                ),
            )
            flightLog.write("$experimentName armed direction=$direction speed=$speed")
            render("等待 MSDK 控制權；期間只送零速度…")
            mainHandler.post(hardwareLatencyTickRunnable)
        }
    }

    private fun horizontalPulseName(experiment: HorizontalPulseExperiment): String =
        when (experiment) {
            HorizontalPulseExperiment.HARDWARE_LATENCY -> "硬體延遲脈衝"
            HorizontalPulseExperiment.STRAIGHT_REHEARSAL -> "低速試跑"
            HorizontalPulseExperiment.STRAIGHT_MEASUREMENT -> "直線測速"
        }

    private fun activeHorizontalPulseName(): String =
        horizontalPulseExperiment?.let(::horizontalPulseName) ?: "速度實驗"

    private fun horizontalPulseEventName(
        experiment: HorizontalPulseExperiment,
        suffix: String,
    ): String =
        when (experiment) {
            HorizontalPulseExperiment.HARDWARE_LATENCY -> "latency_test_$suffix"
            HorizontalPulseExperiment.STRAIGHT_REHEARSAL,
            HorizontalPulseExperiment.STRAIGHT_MEASUREMENT -> "straight_test_$suffix"
        }

    private fun resetHorizontalPulseButtonLabels() {
        if (::hardwareLatencyButton.isInitialized) {
            hardwareLatencyButton.text = "硬體延遲脈衝・0.50 m/s"
        }
        if (::straightRehearsalButton.isInitialized) {
            straightRehearsalButton.text = "低速試跑・0.30 m/s"
            straightMeasurementButton.text = "直線測速・0.50 m/s"
        }
    }

    private fun hardwareLatencyStartFailure(nowNanos: Long = System.nanoTime()): String? {
        if (anotherFlightControlActive()) return "請先停止其他飛行控制"
        if (!registered || !aircraftConnected || !flying) {
            return "飛機未在空中，無法執行速度脈衝"
        }
        if (lowCellVoltage || (batteryPercent?.let { it <= BATTERY_CRITICAL_PERCENT } == true)) {
            return "電量狀態危急，不執行速度脈衝"
        }
        if (!avoidance.brakeConfirmed) {
            return avoidance.warning ?: "BRAKE 尚未確認，速度脈衝不啟動"
        }
        val height = usableHeightMeters()
            ?: return "沒有高度資料，速度脈衝不啟動"
        if (height !in HARDWARE_LATENCY_START_HEIGHT_RANGE_METERS) {
            return "請先將高度調整至 0.5–1.0 m（目前 %.2f m）".format(height)
        }
        horizontalActuationStopReason(nowNanos)?.let { return it }
        return null
    }

    private fun hardwareLatencyRuntimeFailure(nowNanos: Long): String? {
        if (isStraightLineTest()) return null
        if (!registered || !aircraftConnected || !flying) return "飛行狀態已結束"
        if (lowCellVoltage || (batteryPercent?.let { it <= BATTERY_CRITICAL_PERCENT } == true)) {
            return "電量狀態危急"
        }
        if (!avoidance.brakeConfirmed) return avoidance.warning ?: "BRAKE 狀態失效"
        val height = usableHeightMeters() ?: return "高度資料中斷"
        if (height !in HARDWARE_LATENCY_FLIGHT_HEIGHT_RANGE_METERS) {
            return "高度超出 0.4–1.1 m 安全範圍（目前 %.2f m）".format(height)
        }
        return horizontalActuationStopReason(nowNanos)
    }

    private val hardwareLatencyTickRunnable = object : Runnable {
        override fun run() {
            val sequence = hardwareLatencyPulseSequence ?: return
            val nowNanos = System.nanoTime()
            hardwareLatencyRuntimeFailure(nowNanos)?.let { reason ->
                stopHardwareLatencyTest("${activeHorizontalPulseName()}安全停止：$reason")
                return
            }
            if (!stickStatus.hasMsdkAuthority) {
                if (hardwareLatencyAuthoritySeen) {
                    stopHardwareLatencyTest(
                        "實體遙控器已接管，${activeHorizontalPulseName()}已停止",
                        release = false,
                    )
                    return
                }
                if (nowNanos - hardwareLatencyArmedAtNanos > AUTHORITY_HANDOVER_TIMEOUT_NANOS) {
                    stopHardwareLatencyTest("等待 MSDK 控制權逾時，${activeHorizontalPulseName()}未啟動")
                    return
                }
                mainHandler.postDelayed(this, HARDWARE_LATENCY_TICK_MS)
                return
            }

            hardwareLatencyAuthoritySeen = true
            val step = if (hardwareLatencyStartedAtNanos == 0L) {
                hardwareLatencyStartedAtNanos = nowNanos
                sequence.start(nowNanos)
            } else {
                sequence.advance(nowNanos)
            }
            if (step != null) {
                if (step.complete) {
                    val experiment = checkNotNull(horizontalPulseExperiment)
                    val message =
                        when (experiment) {
                            HorizontalPulseExperiment.HARDWARE_LATENCY ->
                                "硬體延遲脈衝完成 " +
                                    "${HardwareLatencyPulseSequence.DEFAULT_CYCLE_COUNT} 組；" +
                                    "已歸零並交回遙控器"
                            HorizontalPulseExperiment.STRAIGHT_REHEARSAL,
                            HorizontalPulseExperiment.STRAIGHT_MEASUREMENT ->
                                when ((sequence as StraightLineSpeedSequence).stopReason) {
                                    StraightLineStopReason.OPERATOR -> "操作者結束本趟；歸零紀錄完成，請確認實際停穩"
                                    StraightLineStopReason.CANCELLED_BEFORE_CRUISE -> "操作者取消；未開始移動，不算完成測試"
                                    StraightLineStopReason.TIME_LIMIT -> "命令時間到期中止；未確認通過 B，不算有效區間"
                                    else -> "控制更新中斷；本趟中止，不算有效區間"
                                }
                        }
                    stopHardwareLatencyTest(message,
                        completed = sequence !is StraightLineSpeedSequence ||
                            sequence.stopReason == StraightLineStopReason.OPERATOR)
                    return
                }
                applyHorizontalPulseStep(step)
            }
            if (sequence is StraightLineSpeedSequence) {
                val config = checkNotNull(straightLineTestConfig)
                val cruising = sequence.phase == StraightLineSpeedPhase.CRUISE
                virtualStick.setHorizontalVelocity(
                    if (cruising) config.speedMetersPerSecond * config.direction.forwardSign else 0.0,
                    0.0,
                    validUntilNanos = if (cruising) {
                        minOf(sequence.commandDeadlineNanos, nowNanos + 500_000_000L)
                    } else 0L,
                )
                recordStraightLineSample(nowNanos, sequence)
            }
            mainHandler.postDelayed(this, HARDWARE_LATENCY_TICK_MS)
        }
    }

    private fun applyHorizontalPulseStep(step: HorizontalPulseStep) {
        val requestedAtNanos = System.nanoTime()
        setHardwareLatencyMarker(step.markerWhite)
        val straightSequence = hardwareLatencyPulseSequence as? StraightLineSpeedSequence
        virtualStick.setHorizontalVelocity(
            step.forwardMetersPerSecond,
            step.rightMetersPerSecond,
            validUntilNanos = if (straightSequence?.phase == StraightLineSpeedPhase.CRUISE) {
                minOf(straightSequence.commandDeadlineNanos, requestedAtNanos + 500_000_000L)
            } else 0L,
        )
        val hardwareStep = step as? HardwareLatencyPulseStep
        val straightStep = step as? StraightLineSpeedStep
        if (straightStep != null) velocityReadDiagnostics.updatePhase(straightStep.phase.name)
        if (straightStep != null) {
            straightLineAttemptId?.let { visualVelocityDiagnostics?.updatePhase(straightStep.phase.name, it) }
        }
        val experiment = checkNotNull(horizontalPulseExperiment)
        val event = horizontalPulseEventName(experiment, "command")
        flightProfiler.record(
            event = event,
            atNanos = requestedAtNanos,
            details = profileDetails(
                "cycle" to hardwareStep?.cycle,
                "attemptId" to straightLineAttemptId.takeIf { straightStep != null },
                "stopReason" to straightSequence?.stopReason,
                "phase" to step.phaseName,
                "direction" to straightStep?.direction?.displayName,
                "forward" to step.forwardMetersPerSecond,
                "right" to step.rightMetersPerSecond,
                "groundSpeed" to groundSpeedMetersPerSecond,
                "velocityX" to aircraftVelocityX,
                "velocityY" to aircraftVelocityY,
                "heading" to aircraftHeadingDegrees,
                "marker" to if (straightStep != null) "none_center_clear" else if (step.markerWhite) "white" else "black",
            ),
        )
        flightLog.write(
            "$event phase=${step.phaseName} direction=" +
                "${straightStep?.direction?.displayName} " +
                "forward=${step.forwardMetersPerSecond} right=${step.rightMetersPerSecond}",
        )
        holdStatus =
            when (step) {
                is HardwareLatencyPulseStep -> when (step.phase) {
                    HardwareLatencyPulsePhase.BASELINE -> "硬體延遲脈衝：懸停基線 3 秒"
                    HardwareLatencyPulsePhase.FORWARD ->
                        "硬體延遲脈衝 ${step.cycle}/10：前進 0.50 m/s"
                    HardwareLatencyPulsePhase.SETTLE_AFTER_FORWARD ->
                        "硬體延遲脈衝 ${step.cycle}/10：歸零 2 秒"
                    HardwareLatencyPulsePhase.BACKWARD ->
                        "硬體延遲脈衝 ${step.cycle}/10：後退 0.50 m/s"
                    HardwareLatencyPulsePhase.SETTLE_AFTER_BACKWARD ->
                        "硬體延遲脈衝 ${step.cycle}/10：歸零 2 秒"
                    HardwareLatencyPulsePhase.COMPLETE ->
                        error("complete step is handled before application")
                }
                is StraightLineSpeedStep -> {
                    val config = checkNotNull(straightLineTestConfig)
                    straightLineButton(experiment).text =
                        if (step.phase == StraightLineSpeedPhase.BRAKING) "停止中" else "停止"
                    when (step.phase) {
                        StraightLineSpeedPhase.BASELINE -> "${horizontalPulseName(experiment)}：基線歸零 2 秒"
                        StraightLineSpeedPhase.CRUISE -> String.format(Locale.US,
                            "%s %s・命令 %.2f m/s・上限 %d 秒；過 B 請按停止",
                            straightLineAttemptId, config.direction.displayName,
                            config.speedMetersPerSecond, config.maximumCruiseSeconds)
                        StraightLineSpeedPhase.BRAKING ->
                            "${straightSequence?.stopReason}：已送零速度，續錄 3 秒；不代表已停穩"
                        StraightLineSpeedPhase.COMPLETE -> "歸零記錄結束"
                    }
                }
                else -> error("unsupported horizontal pulse step ${step::class.java.name}")
            }
        render(holdStatus)
    }

    private fun stopHardwareLatencyTest(
        message: String,
        release: Boolean = true,
        completed: Boolean = false,
    ) {
        val wasActive = hardwareLatencyStartPending || hardwareLatencyPulseSequence != null
        val experiment = horizontalPulseExperiment
        val straightSequence = hardwareLatencyPulseSequence as? StraightLineSpeedSequence
        val straight = isStraightLineTest(experiment)
        horizontalPulseGeneration += 1
        hardwareLatencyStartPending = false
        hardwareLatencyPulseSequence = null
        hardwareLatencyAuthoritySeen = false
        mainHandler.removeCallbacks(hardwareLatencyTickRunnable)
        virtualStick.setHorizontalVelocity(0.0, 0.0)
        if (straight) {
            mainHandler.removeCallbacks(velocityReadTickRunnable)
            velocityReadDiagnostics.stop()
            straightLineAttemptId?.let { visualVelocityDiagnostics?.stop(it) }
            refreshVisualVelocityContext()
            virtualStick.setYawRate(0.0)
            virtualStick.setClimbRate(0.0)
        }
        setHardwareLatencyMarker(white = false)
        if (::hardwareLatencySyncMarker.isInitialized) {
            hardwareLatencySyncMarker.visibility = View.GONE
        }
        horizontalPulseExperiment = null
        resetHorizontalPulseButtonLabels()
        if (!wasActive) return

        val completedExperiment = checkNotNull(experiment)
        val nowNanos = System.nanoTime()
        flightProfiler.record(
            event = horizontalPulseEventName(completedExperiment, "stop"),
            atNanos = nowNanos,
            details = profileDetails(
                "completed" to completed,
                "attemptId" to straightLineAttemptId.takeIf { straight },
                "stopReason" to straightSequence?.stopReason,
                "physicalMarkerCrossingVerified" to false,
                "durationMs" to
                    hardwareLatencyStartedAtNanos
                        .takeIf { it > 0L }
                        ?.let { (nowNanos - it).coerceAtLeast(0L) / 1_000_000.0 },
                "reason" to message,
            ),
        )
        flightLog.write(
            "${horizontalPulseName(completedExperiment)} stop completed=$completed reason=$message",
        )
        hardwareLatencyStartedAtNanos = 0L
        hardwareLatencyArmedAtNanos = 0L
        holdStatus = message
        if (straight) {
            straightLineTestConfig = null
            straightLineReleasePending = true
            straightLineTracePending = true
            fun finish(releaseError: String?) {
                flightProfiler.checkpoint { storageError ->
                    runOnUiThread {
                        straightLineTracePending = false
                        if (activityDestroying) return@runOnUiThread
                        holdStatus = message +
                            (releaseError?.let { "；交還控制權失敗：$it，請用實體遙控器接管" } ?: "") +
                            (storageError?.let { "；紀錄寫入失敗：${it.message}" } ?: "；紀錄已刷新至手機")
                        if (straightLineReleasePending) holdStatus += "；確認 RC 控制權前禁止重啟"
                        render(holdStatus)
                    }
                }
            }
            if (release && stickOwned && !stickTransitionPending) {
                releaseControlLink(::finish)
            } else {
                finish(null)
            }
            return
        }
        if (!release || !stickOwned || stickTransitionPending) {
            render(message)
            return
        }
        releaseControlLink { error ->
            holdStatus = error?.let { "$message（釋放控制權失敗：$it）" } ?: message
            render(holdStatus)
        }
    }

    private fun tiltFlightLabel(mode: TiltFlightMode): String =
        when (mode) {
            TiltFlightMode.STRAIGHT -> "無視覺直線・傾角 3°×2秒・20Hz"
            TiltFlightMode.CIRCLE -> "無視覺繞圈・傾角賽車 Ø1.5m・7秒×3・20Hz"
            TiltFlightMode.SKATING_CIRCLE -> "無視覺繞圈・傾角滑冰 Ø1.5m・7秒×3・20Hz"
        }

    private fun resetTiltFlightButtonLabels() {
        tiltStraightButton.text = tiltFlightLabel(TiltFlightMode.STRAIGHT)
        tiltCircleButton.text = tiltFlightLabel(TiltFlightMode.CIRCLE)
        tiltSkatingCircleButton.text = tiltFlightLabel(TiltFlightMode.SKATING_CIRCLE)
    }

    private fun toggleTiltFlight(mode: TiltFlightMode) {
        if (tiltFlightRun != null) {
            stopTiltFlight("操作者停止傾角實驗")
            return
        }
        startTiltFlight(mode)
    }

    private fun usableTiltFlightSpeedMetersPerSecond(nowNanos: Long): Double? =
        synchronized(aircraftVelocityLock) {
            if (aircraftVelocityAtNanos == 0L ||
                nowNanos - aircraftVelocityAtNanos > AIRCRAFT_VELOCITY_STALE_NANOS
            ) null else groundSpeedMetersPerSecond.takeIf(Double::isFinite)
        }

    private fun tiltFlightStartFailure(nowNanos: Long = System.nanoTime()): String? {
        noVisionFlightStartFailure("傾角實驗", nowNanos)?.let { return it }
        if (stickTransitionPending) return "控制權切換尚未完成"
        val speed = usableTiltFlightSpeedMetersPerSecond(nowNanos)
        tiltFlightTelemetryFailure(speed)?.let { return it }
        if (speed != null && speed > TILT_START_MAX_SPEED_MPS) {
            return "請先懸停，水平速度須低於 %.1f m/s".format(TILT_START_MAX_SPEED_MPS)
        }
        return null
    }

    private fun tiltFlightTelemetryFailure(speedMetersPerSecond: Double?): String? {
        if (activityDestroying || !activityForeground) return "測試畫面不在前景"
        if (speedMetersPerSecond != null && speedMetersPerSecond > TILT_FLIGHT_MAX_SPEED_MPS) {
            return "水平速度超出傾角實驗 %.1f m/s 上限".format(TILT_FLIGHT_MAX_SPEED_MPS)
        }
        val pitch = aircraftPitchDegrees?.takeIf(Double::isFinite) ?: return "沒有有效俯仰資料"
        val roll = aircraftRollDegrees?.takeIf(Double::isFinite) ?: return "沒有有效橫滾資料"
        if (abs(pitch) > TILT_FLIGHT_MAX_ATTITUDE_DEGREES ||
            abs(roll) > TILT_FLIGHT_MAX_ATTITUDE_DEGREES
        ) return "機身姿態超出傾角實驗 15° 安全範圍"
        return null
    }

    @Synchronized
    private fun startTiltFlight(mode: TiltFlightMode) {
        tiltFlightStartFailure()?.let { render(it); return }
        if (virtualStick.frameRate() != VirtualStickFrameRate.BASELINE_20 &&
            !virtualStick.selectFrameRate(VirtualStickFrameRate.BASELINE_20)
        ) {
            render("請先交還控制權，再啟動 20 Hz 傾角實驗")
            return
        }
        virtualStickFrameRateButton.text = virtualStickFrameRateLabel()
        val run = TiltFlightRun(mode)
        tiltFlightRun = run
        when (mode) {
            TiltFlightMode.STRAIGHT -> tiltStraightButton.text = "停止無視覺傾角直線"
            TiltFlightMode.CIRCLE -> tiltCircleButton.text = "停止無視覺傾角繞圈"
            TiltFlightMode.SKATING_CIRCLE -> tiltSkatingCircleButton.text = "停止無視覺傾角滑冰"
        }
        mainHandler.removeCallbacks(idleReleaseRunnable)
        render("正在準備無視覺傾角實驗；關閉避障後取得控制權…")
        mainHandler.postDelayed({
            if (tiltFlightRun === run && !run.stopping && run.armedAtNanos == 0L) {
                stopTiltFlight("傾角實驗準備逾時，已取消")
            }
        }, TILT_PREPARATION_TIMEOUT_MS)
        avoidanceCheck.ensureClosed { status ->
            runOnUiThread {
                if (tiltFlightRun !== run || run.stopping || activityDestroying) {
                    if (status.closedConfirmed && !anotherFlightControlActive()) {
                        avoidanceCheck.ensureBrake {}
                    }
                    return@runOnUiThread
                }
                avoidance = status
                flightLog.write("tilt flight avoidance mode=$mode ${status.summary}")
                if (!status.closedConfirmed) {
                    stopTiltFlight("傾角實驗未啟動：${status.detail ?: "無法確認避障關閉"}")
                    return@runOnUiThread
                }
                acquireControlLink(
                    onFailure = { reason ->
                        if (tiltFlightRun === run) stopTiltFlight("傾角實驗未啟動：$reason")
                    },
                    shouldContinue = { tiltFlightRun === run && !run.stopping && activityForeground },
                ) {
                    armTiltFlight(run)
                }
            }
        }
    }

    @Synchronized
    private fun armTiltFlight(run: TiltFlightRun) {
        if (tiltFlightRun !== run || run.stopping) return
        val nowNanos = System.nanoTime()
        (noVisionFlightRuntimeFailure(nowNanos) ?:
            tiltFlightTelemetryFailure(usableTiltFlightSpeedMetersPerSecond(nowNanos)))?.let {
            stopTiltFlight("傾角實驗未啟動：$it")
            return
        }
        virtualStick.setHorizontalVelocity(0.0, 0.0)
        virtualStick.setClimbRate(0.0)
        virtualStick.setYawRate(0.0)
        run.armedAtNanos = nowNanos
        flightProfiler.record(
            event = "tilt_flight_armed",
            atNanos = nowNanos,
            details = profileDetails(
                "mode" to run.mode,
                "rollPitchMode" to "ANGLE",
                "stickHz" to virtualStick.frameRate().hertz,
                "idealSpeed" to run.controller.targetSpeedMetersPerSecond,
                "startupMs" to run.controller.startupDurationNanos / 1_000_000.0,
                "scheduledDurationMs" to run.controller.scheduledDurationNanos / 1_000_000.0,
                "visionUsed" to false,
                "velocityFeedbackUsed" to false,
                "dragCompensation" to "none_unmeasured",
            ),
        )
        flightLog.write("tilt flight armed mode=${run.mode} Hz=20 vision=false velocityFeedback=false")
        holdStatus = "等待 MSDK 控制權；之後先零速度懸停 2 秒"
        render(holdStatus)
    }

    @Synchronized
    private fun checkTiltFlightSafety(
        nowNanos: Long,
        speedMetersPerSecond: Double? = usableTiltFlightSpeedMetersPerSecond(nowNanos),
    ) {
        val run = tiltFlightRun ?: return
        if (run.stopping || run.armedAtNanos == 0L) return
        val failure =
            noVisionFlightRuntimeFailure(nowNanos)
                ?: tiltFlightTelemetryFailure(speedMetersPerSecond)
                ?: when {
                    run.authoritySeen &&
                        (!stickStatus.hasMsdkAuthority || !stickStatus.advancedMode) ->
                        "MSDK 進階控制權已中斷"
                    !run.authoritySeen && nowNanos - run.armedAtNanos > AUTHORITY_HANDOVER_TIMEOUT_NANOS ->
                        "等待 MSDK 控制權逾時"
                    run.lastTickAtNanos > 0L && nowNanos - run.lastTickAtNanos > TILT_COMMAND_LEASE_NANOS ->
                        "20 Hz 傾角控制排程中斷"
                    virtualStick.frameRate() != VirtualStickFrameRate.BASELINE_20 ->
                        "傾角實驗必須使用 20 Hz"
                    else -> null
                }
        if (failure != null) stopTiltFlight("傾角實驗安全停止：$failure")
    }

    @Synchronized
    private fun driveTiltFlight(nowNanos: Long) {
        val run = tiltFlightRun ?: return
        if (run.stopping || run.armedAtNanos == 0L) return
        val speed = usableTiltFlightSpeedMetersPerSecond(nowNanos)
        checkTiltFlightSafety(nowNanos, speed)
        if (tiltFlightRun !== run || run.stopping) return
        if (!stickStatus.hasMsdkAuthority || !stickStatus.advancedMode) return
        run.authoritySeen = true
        run.lastTickAtNanos = nowNanos
        val heading = aircraftHeadingDegrees ?: return
        val command = try {
            if (run.startedAtNanos == 0L) {
                run.startedAtNanos = nowNanos
                run.controller.start(nowNanos, heading)
            } else {
                run.controller.command(nowNanos, heading)
            }
        } catch (error: IllegalArgumentException) {
            stopTiltFlight("傾角排程輸入無效：${error.message}")
            return
        }
        if (command.completed) {
            stopTiltFlight("傾角前饋排程結束，已完成零速度煞停", completed = true, brake = false)
            return
        }
        if (abs(wrapToSignedHeading(command.yawHeadingDegrees - heading)) > TILT_FLIGHT_MAX_HEADING_ERROR_DEGREES) {
            stopTiltFlight("機頭偏離傾角排程超過 45°，已停止")
            return
        }
        if (run.lastPhase == TiltFlightPhase.SETTLE && command.usesAngle &&
            speed != null && speed > TILT_START_MAX_SPEED_MPS
        ) {
            stopTiltFlight("懸停尚未穩定，不進入傾角加速段")
            return
        }
        if (command.usesAngle) {
            if (!virtualStick.setHorizontalAngles(
                    rollDegrees = command.rollDegrees,
                    pitchDegrees = command.pitchDegrees,
                    validUntilNanos = minOf(nowNanos + TILT_COMMAND_LEASE_NANOS, command.phaseEndsAtNanos),
                )
            ) {
                stopTiltFlight("傾角命令無效、超限或過期，已切回零速度")
                return
            }
        } else {
            virtualStick.setHorizontalVelocity(0.0, 0.0)
        }
        virtualStick.setClimbRate(0.0)
        if (!virtualStick.setYawHeading(command.yawHeadingDegrees)) {
            stopTiltFlight("傾角排程航向無效，已停止")
            return
        }
        if (run.lastPhase != command.phase) {
            run.lastPhase = command.phase
            flightLog.write("tilt flight phase mode=${run.mode} phase=${command.phase}")
            flightProfiler.record(
                event = "tilt_flight_phase",
                atNanos = nowNanos,
                details = profileDetails("mode" to run.mode, "phase" to command.phase),
            )
        }
        flightProfiler.recordLazy(event = "tilt_flight_command", atNanos = nowNanos) {
            profileDetails(
                "mode" to run.mode,
                "phase" to command.phase,
                "rollPitchMode" to if (command.usesAngle) "ANGLE" else "VELOCITY",
                "rollDegrees" to command.rollDegrees.takeIf { command.usesAngle },
                "pitchDegrees" to command.pitchDegrees.takeIf { command.usesAngle },
                "yawCommand" to command.yawHeadingDegrees,
                "heading" to heading,
                "groundSpeed" to speed,
                "scheduledLap" to command.lap,
                "scheduledProgressDegrees" to command.progressDegrees,
                "visionUsed" to false,
                "velocityFeedbackUsed" to false,
            )
        }
        if (nowNanos - run.lastRenderedAtNanos >= MATHEMATICAL_CIRCLE_RENDER_PERIOD_NANOS) {
            run.lastRenderedAtNanos = nowNanos
            val phase = when (command.phase) {
                TiltFlightPhase.SETTLE -> "零速度懸停"
                TiltFlightPhase.STRAIGHT -> "直線前傾 3°"
                TiltFlightPhase.STARTUP -> "建立切線速度"
                TiltFlightPhase.CIRCLE ->
                    "左轉${if (run.mode == TiltFlightMode.SKATING_CIRCLE) "滑冰" else "賽車"}排程 " +
                        "${command.lap}/3・%.0f°".format(command.progressDegrees % 360.0)
                TiltFlightPhase.BRAKE -> "零速度煞停"
                TiltFlightPhase.COMPLETE -> "排程結束"
            }
            holdStatus = "無視覺傾角・$phase（20 Hz）" +
                if (speed == null) "；速度資料不可用，略過速度限制" else ""
            render(holdStatus)
        }
    }

    @Synchronized
    private fun stopTiltFlight(
        message: String,
        release: Boolean = true,
        completed: Boolean = false,
        brake: Boolean = true,
    ) {
        val run = tiltFlightRun ?: return
        if (run.stopping && release) return
        val wasStopping = run.stopping
        run.stopping = true
        // Zero ANGLE would coast. Every exit first selects horizontal VELOCITY zero.
        virtualStick.setHorizontalVelocity(0.0, 0.0)
        virtualStick.setClimbRate(0.0)
        virtualStick.setYawRate(0.0)
        if (!wasStopping) {
            flightProfiler.record(
                event = "tilt_flight_stop",
                atNanos = System.nanoTime(),
                details = profileDetails("mode" to run.mode, "completed" to completed, "reason" to message),
            )
        }
        flightLog.write("tilt flight stop mode=${run.mode} completed=$completed reason=$message")
        if (!release || activityDestroying) {
            tiltFlightRun = null
            if (!activityDestroying) runOnUiThread {
                resetTiltFlightButtonLabels()
                restoreBrakeAfterAutonomousControl(message, release = false, logContext = "tilt flight")
            }
            return
        }
        runOnUiThread {
            if (tiltFlightRun !== run) return@runOnUiThread
            tiltStraightButton.text = "傾角實驗煞停收尾中"
            tiltCircleButton.text = "傾角實驗煞停收尾中"
            tiltSkatingCircleButton.text = "傾角實驗煞停收尾中"
            holdStatus = "$message；正在交還控制權"
            render(holdStatus)
            val finish = Runnable {
                if (tiltFlightRun !== run) return@Runnable
                tiltFlightRun = null
                resetTiltFlightButtonLabels()
                restoreBrakeAfterAutonomousControl(message, release = true, logContext = "tilt flight")
            }
            if (brake && run.startedAtNanos != 0L && flying &&
                stickStatus.hasMsdkAuthority
            ) {
                mainHandler.postDelayed(finish, TiltFlightController.BRAKE_DURATION_NANOS / 1_000_000L)
            } else {
                finish.run()
            }
        }
    }

    private fun mathematicalCircleLabel(mode: MathematicalCircleMode): String =
        "數學圓・${mode.displayName} Ø${MathematicalCircleController.DIAMETER_METERS}m・" +
            "${mode.secondsPerLap}秒×${MathematicalCircleController.LAP_COUNT}" +
            if (mode == MathematicalCircleMode.RACING_FAST) "・40Hz" else ""


    private fun armMathematicalCircle(mode: MathematicalCircleMode) {
        if (!mathematicalCircleStartPending) return
        noVisionFlightRuntimeFailure()?.let { reason ->
            flightLog.write("mathematical circle aborted before arm mode=$mode reason=$reason")
            stopMathematicalCircle("數學圓周未啟動：$reason")
            return
        }
        val initialHeading = checkNotNull(aircraftHeadingDegrees)
        val controller = MathematicalCircleController(secondsPerLap = mode.secondsPerLap)
        mathematicalCircleInitialHeadingDegrees = wrapToSignedHeading(initialHeading)
        mathematicalCircleAuthoritySeen = false
        mathematicalCircleArmedAtNanos = System.nanoTime()
        mathematicalCircleStartedAtNanos = 0L
        mathematicalCircleLastLap = 0
        mathematicalCircleLastRenderedAtNanos = 0L
        mathematicalCircleLastTickAtNanos = 0L
        virtualStick.setHorizontalVelocity(0.0, 0.0)
        virtualStick.setClimbRate(0.0)
        virtualStick.setYawRate(0.0)
        mathematicalCircleStartPending = false
        mathematicalCircleController = controller
        flightProfiler.record(
            event = "mathematical_circle_armed",
            atNanos = mathematicalCircleArmedAtNanos,
            details =
                profileDetails(
                    "mode" to mode,
                    "diameterMeters" to MathematicalCircleController.DIAMETER_METERS,
                    "secondsPerLap" to controller.secondsPerLap,
                    "laps" to MathematicalCircleController.LAP_COUNT,
                    "speed" to controller.tangentialSpeedMetersPerSecond,
                    "angularRateDegreesPerSecond" to controller.angularRateDegreesPerSecond,
                    "heading" to initialHeading,
                    "positionSource" to "elapsedTimeSchedule",
                    "visionUsed" to false,
                ),
        )
        flightLog.write(
            mathematicalCircleArmedLogMessage(
                mode = mode.toString(),
                diameterMeters = MathematicalCircleController.DIAMETER_METERS,
                secondsPerLap = controller.secondsPerLap,
                laps = MathematicalCircleController.LAP_COUNT,
                speedMetersPerSecond = controller.tangentialSpeedMetersPerSecond,
                angularRateDegreesPerSecond = controller.angularRateDegreesPerSecond,
                headingDegrees = initialHeading,
            ),
        )
        holdStatus = "等待 MSDK 控制權；期間只送零速度…"
        render(holdStatus)
    }

    private fun toggleMathematicalCircle(mode: MathematicalCircleMode) {
        if (mathematicalCircleStartPending || mathematicalCircleController != null) {
            stopMathematicalCircle("操作者停止數學圓周")
            return
        }
        noVisionFlightStartFailure("數學圓周")?.let { reason ->
            flightLog.write("mathematical circle refused mode=$mode reason=$reason")
            render(reason)
            return
        }
        if (mode == MathematicalCircleMode.RACING_FAST) {
            if (virtualStick.frameRate() != VirtualStickFrameRate.EXPERIMENT_40 &&
                !virtualStick.selectFrameRate(VirtualStickFrameRate.EXPERIMENT_40)
            ) {
                render("請先釋放控制權，再啟動 40 Hz 賽車快版")
                return
            }
            virtualStickFrameRateButton.text = virtualStickFrameRateLabel()
        }

        mathematicalCircleMode = mode
        mathematicalCircleStartPending = true
        val stopLabel = "停止數學圓・${mode.displayName}"
        when (mode) {
            MathematicalCircleMode.SKATING -> mathematicalSkatingCircleButton.text = stopLabel
            MathematicalCircleMode.RACING -> mathematicalRacingCircleButton.text = stopLabel
            MathematicalCircleMode.RACING_FAST -> mathematicalFastRacingCircleButton.text = stopLabel
        }
        render("正在關閉飛機避障；請讓機頭朝向起點切線，圓心位於機身左側…")
        avoidanceCheck.ensureClosed { status ->
            if (activityDestroying) {
                if (status.closedConfirmed) avoidanceCheck.ensureBrake {}
                return@ensureClosed
            }
            runOnUiThread {
                avoidance = status
                flightLog.write(
                    "mathematical circle avoidance mode=$mode ${status.summary} detail=${status.detail}",
                )
                if (!mathematicalCircleStartPending) {
                    if (status.closedConfirmed) avoidanceCheck.ensureBrake {}
                    return@runOnUiThread
                }
                if (!status.closedConfirmed) {
                    mathematicalCircleStartPending = false
                    resetMathematicalCircleButtonLabels()
                    render(
                        status.detail?.let { "數學圓周未啟動：無法關閉避障：$it" }
                            ?: "數學圓周未啟動：無法確認飛機避障已關閉",
                    )
                    readAvoidanceConfiguration()
                    return@runOnUiThread
                }
                render("飛機避障已關閉；正在取得控制權…")
                acquireControlLink(
                    onFailure = { reason ->
                        stopMathematicalCircle("數學圓周未啟動：$reason")
                    },
                    shouldContinue = { mathematicalCircleStartPending },
                ) {
                    if (!mathematicalCircleStartPending) {
                        if (stickOwned && !activityDestroying) {
                            releaseControlLink { error ->
                                render(
                                    error?.let { "已取消數學圓周，但釋放控制權失敗：$it" }
                                        ?: "數學圓周已取消",
                                )
                            }
                        }
                        return@acquireControlLink
                    }
                    noVisionFlightRuntimeFailure()?.let { reason ->
                        flightLog.write(
                            "mathematical circle aborted after acquire mode=$mode reason=$reason",
                        )
                        stopMathematicalCircle("數學圓周未啟動：$reason")
                        return@acquireControlLink
                    }
                    render("控制權已取得；準備純數學圓周排程…")
                    armMathematicalCircle(mode)
                }
            }
        }
    }

    private fun resetMathematicalCircleButtonLabels() {
        if (::mathematicalSkatingCircleButton.isInitialized) {
            mathematicalSkatingCircleButton.text = mathematicalCircleLabel(MathematicalCircleMode.SKATING)
        }
        if (::mathematicalRacingCircleButton.isInitialized) {
            mathematicalRacingCircleButton.text = mathematicalCircleLabel(MathematicalCircleMode.RACING)
        }
        if (::mathematicalFastRacingCircleButton.isInitialized) {
            mathematicalFastRacingCircleButton.text = mathematicalCircleLabel(MathematicalCircleMode.RACING_FAST)
        }
    }

    private fun noVisionFlightStartFailure(name: String, nowNanos: Long = System.nanoTime()): String? {
        if (anotherFlightControlActive()) return "請先停止其他飛行控制"
        if (!registered || !aircraftConnected || !flying) {
            return "飛機未在空中，無法執行$name"
        }
        if (lowCellVoltage || (batteryPercent?.let { it <= BATTERY_CRITICAL_PERCENT } == true)) {
            return "電量狀態危急，不執行$name"
        }
        if (!avoidance.brakeConfirmed) {
            return avoidance.warning ?: "BRAKE 尚未確認，$name 不啟動"
        }
        val height = usableHeightMeters() ?: return "沒有高度資料，$name 不啟動"
        if (height !in MATHEMATICAL_CIRCLE_START_HEIGHT_RANGE_METERS) {
            return "請先將高度調整至 0.5–1.0 m（目前 %.2f m）".format(height)
        }
        if (
            aircraftHeadingDegrees == null ||
            aircraftHeadingAtNanos == 0L ||
            nowNanos - aircraftHeadingAtNanos > MAX_MOVING_HEADING_AGE_NANOS
        ) {
            return "沒有即時機頭方向，$name 不啟動"
        }
        horizontalActuationStopReason(nowNanos)?.let { return it }
        mathematicalCircleObstacleFailure(nowNanos)?.let { return it }
        return null
    }

    private fun noVisionFlightRuntimeFailure(nowNanos: Long = System.nanoTime()): String? {
        if (!registered || !aircraftConnected || !flying) return "飛行狀態已結束"
        if (lowCellVoltage || (batteryPercent?.let { it <= BATTERY_CRITICAL_PERCENT } == true)) {
            return "電量狀態危急"
        }
        if (!avoidance.closedConfirmed) {
            return avoidance.warning ?: "飛機避障關閉狀態失效"
        }
        val height = usableHeightMeters() ?: return "高度資料中斷"
        if (height !in MATHEMATICAL_CIRCLE_FLIGHT_HEIGHT_RANGE_METERS) {
            return "高度超出 0.4–1.1 m 安全範圍（目前 %.2f m）".format(height)
        }
        if (
            aircraftHeadingDegrees == null ||
            aircraftHeadingAtNanos == 0L ||
            nowNanos - aircraftHeadingAtNanos > MAX_MOVING_HEADING_AGE_NANOS
        ) {
            return "機頭方向資料停止更新"
        }
        horizontalActuationStopReason(nowNanos)?.let { return it }
        return mathematicalCircleObstacleFailure(nowNanos)
    }

    private fun mathematicalCircleObstacleFailure(nowNanos: Long): String? {
        val nearest = nearestHorizontalObstacleMm ?: return null
        if (!obstacleSampleValid || obstacleSampleAtNanos == 0L) return null
        val ageMs = (nowNanos - obstacleSampleAtNanos) / 1_000_000L
        if (ageMs > MAX_MANUAL_OBSTACLE_SAMPLE_AGE_MS) return null
        return if (nearest <= AUTONOMOUS_HORIZONTAL_CLEARANCE_MM) {
            "障礙距離 ${nearest}mm（高速門檻 ${AUTONOMOUS_HORIZONTAL_CLEARANCE_MM}mm）"
        } else {
            null
        }
    }

    @Synchronized
    private fun driveMathematicalCircle(nowNanos: Long) {
        val controller = mathematicalCircleController ?: return
        noVisionFlightRuntimeFailure(nowNanos)?.let { reason ->
            stopMathematicalCircle("數學圓周安全停止：$reason")
            return
        }
        if (!stickStatus.hasMsdkAuthority) {
            if (mathematicalCircleAuthoritySeen) {
                stopMathematicalCircle("實體遙控器已接管，數學圓周已停止", release = false)
                return
            }
            virtualStick.setHorizontalVelocity(0.0, 0.0)
            virtualStick.setYawRate(0.0)
            if (nowNanos - mathematicalCircleArmedAtNanos > AUTHORITY_HANDOVER_TIMEOUT_NANOS) {
                stopMathematicalCircle("等待 MSDK 控制權逾時，數學圓周未啟動")
            }
            return
        }

        mathematicalCircleAuthoritySeen = true
        if (
            mathematicalCircleLastTickAtNanos > 0L &&
            nowNanos - mathematicalCircleLastTickAtNanos > MATHEMATICAL_CIRCLE_MAX_TICK_GAP_NANOS
        ) {
            stopMathematicalCircle(
                "${virtualStick.frameRate().hertz} Hz 控制排程中斷，數學圓周已停止",
            )
            return
        }
        mathematicalCircleLastTickAtNanos = nowNanos
        val currentHeading = checkNotNull(aircraftHeadingDegrees)
        val command =
            if (mathematicalCircleStartedAtNanos == 0L) {
                mathematicalCircleStartedAtNanos = nowNanos
                mathematicalCircleLastLap = 1
                controller.start(nowNanos, currentHeading)
            } else {
                controller.command(nowNanos, currentHeading)
            }
        if (command.completed) {
            stopMathematicalCircle("純數學圓周完成 3 圈；已歸零並交回遙控器", completed = true)
            return
        }

        if (command.lap != mathematicalCircleLastLap) {
            val completedLap = command.lap - 1
            flightProfiler.record(
                event = "mathematical_circle_lap",
                atNanos = nowNanos,
                details =
                    profileDetails(
                        "mode" to mathematicalCircleMode,
                        "lap" to completedLap,
                        "scheduledSeconds" to controller.secondsPerLap,
                    ),
            )
            flightLog.write(
                "mathematical circle lap=$completedLap mode=$mathematicalCircleMode " +
                    "elapsedSeconds=%.3f".format(
                        (nowNanos - mathematicalCircleStartedAtNanos) / 1_000_000_000.0,
                    ),
            )
            mathematicalCircleLastLap = command.lap
        }

        virtualStick.setHorizontalVelocity(
            command.forwardMetersPerSecond,
            command.rightMetersPerSecond,
        )
        virtualStick.setClimbRate(0.0)
        val yawCommand = when (mathematicalCircleMode) {
            MathematicalCircleMode.SKATING -> mathematicalCircleInitialHeadingDegrees
            MathematicalCircleMode.RACING,
            MathematicalCircleMode.RACING_FAST -> command.tangentHeadingDegrees
        }
        // Keep the DJI yaw controller identical; only the heading target differs.
        if (!virtualStick.setYawHeading(yawCommand)) {
            stopMathematicalCircle("數學圓周產生無效航向，已停止")
            return
        }
        flightProfiler.recordLazy(
            event = "mathematical_circle_command",
            atNanos = nowNanos,
        ) {
            profileDetails(
                "mode" to mathematicalCircleMode,
                "lap" to command.lap,
                "scheduledProgressDegrees" to command.totalProgressDegrees,
                "forward" to command.forwardMetersPerSecond,
                "right" to command.rightMetersPerSecond,
                "yawCommand" to yawCommand,
                "heading" to currentHeading,
                "groundSpeed" to groundSpeedMetersPerSecond,
                "positionSource" to "elapsedTimeSchedule",
                "visionUsed" to false,
            )
        }
        if (
            nowNanos - mathematicalCircleLastRenderedAtNanos >=
            MATHEMATICAL_CIRCLE_RENDER_PERIOD_NANOS
        ) {
            mathematicalCircleLastRenderedAtNanos = nowNanos
            holdStatus =
                "數學圓・${mathematicalCircleMode.displayName} ${command.lap}/3：" +
                    "排程 %.0f°".format(command.totalProgressDegrees % 360.0)
            render(holdStatus)
        }
    }

    @Synchronized
    private fun stopMathematicalCircle(
        message: String,
        release: Boolean = true,
        completed: Boolean = false,
    ) {
        val wasActive = mathematicalCircleStartPending || mathematicalCircleController != null
        mathematicalCircleStartPending = false
        mathematicalCircleController = null
        mathematicalCircleAuthoritySeen = false
        virtualStick.setHorizontalVelocity(0.0, 0.0)
        virtualStick.setClimbRate(0.0)
        virtualStick.setYawRate(0.0)
        runOnUiThread(::resetMathematicalCircleButtonLabels)
        if (!wasActive) return

        val nowNanos = System.nanoTime()
        flightProfiler.record(
            event = "mathematical_circle_stop",
            atNanos = nowNanos,
            details =
                profileDetails(
                    "mode" to mathematicalCircleMode,
                    "completed" to completed,
                    "durationMs" to
                        mathematicalCircleStartedAtNanos
                            .takeIf { it > 0L }
                            ?.let { (nowNanos - it).coerceAtLeast(0L) / 1_000_000.0 },
                    "reason" to message,
                ),
        )
        flightLog.write(
            "mathematical circle stop mode=$mathematicalCircleMode " +
                "completed=$completed reason=$message",
        )
        mathematicalCircleArmedAtNanos = 0L
        mathematicalCircleStartedAtNanos = 0L
        mathematicalCircleLastTickAtNanos = 0L
        mathematicalCircleLastLap = 0
        mathematicalCircleLastRenderedAtNanos = 0L
        holdStatus = message
        runOnUiThread {
            restoreBrakeAfterAutonomousControl(
                message = message,
                release = release,
                logContext = "mathematical circle",
            )
        }
    }

    private fun toggleYawLapTest() {
        if (yawLapTestActive) {
            finishHeadingTurn("操作者停止航向模式原地旋轉 360°")
            return
        }
        yawLapTestStartFailure()?.let { reason ->
            flightLog.write("yaw lap test refused: $reason")
            render(reason)
            return
        }
        render("正在取得控制權，準備航向模式原地旋轉 360°…")
        acquireControlLink(
            onFailure = { reason -> render("航向模式原地旋轉未啟動：$reason") },
        ) {
            yawLapTestStartFailure()?.let { reason ->
                flightLog.write("yaw lap test aborted after acquire: $reason")
                releaseControlLink { error ->
                    render(
                        if (error == null) {
                            "航向模式原地旋轉未啟動：$reason"
                        } else {
                            "航向模式原地旋轉未啟動：$reason（釋放控制權失敗：$error）"
                        },
                    )
                }
                return@acquireControlLink
            }
            val initialHeading = checkNotNull(aircraftHeadingDegrees)
            val now = System.nanoTime()
            headingTurn = HeadingTurn(initialHeading, YAW_LAP_TARGET_DEGREES)
            yawLapTestActive = true
            turnStartedAtNanos = now
            turnCommandStartedAtNanos = 0L
            turnAuthoritySeen = stickStatus.hasMsdkAuthority
            turnLastRenderedAtNanos = 0L
            turnLastMeasuredAtNanos = 0L
            turnLastMeasuredProgressDegrees = 0.0
            virtualStick.setHorizontalVelocity(0.0, 0.0)
            virtualStick.setClimbRate(0.0)
            virtualStick.setYawRate(0.0)
            yawLapTestButton.text = "停止航向模式旋轉 360°"
            flightLog.write(
                "yaw lap test armed mode=HEADING heading=%.1f target=%.0f lead=%.1f speedLevel=%.3f"
                    .format(
                        initialHeading,
                        YAW_LAP_TARGET_DEGREES,
                        YAW_LAP_HEADING_LEAD_DEGREES,
                        virtualStick.speedLevel(),
                    ),
            )
            mainHandler.post(headingTurnTickRunnable)
        }
    }

    private fun yawLapTestStartFailure(nowNanos: Long = System.nanoTime()): String? {
        if (anotherFlightControlActive()) return "請先停止其他飛行控制"
        if (!registered || !aircraftConnected || !flying) {
            return "飛機未在空中，無法執行原地旋轉 360°"
        }
        if (lowCellVoltage || (batteryPercent?.let { it <= BATTERY_CRITICAL_PERCENT } == true)) {
            return "電量狀態危急，不執行原地旋轉 360°"
        }
        val heading = aircraftHeadingDegrees
        if (
            heading == null ||
            aircraftHeadingAtNanos == 0L ||
            nowNanos - aircraftHeadingAtNanos > MAX_MOVING_HEADING_AGE_NANOS
        ) {
            return "沒有即時機頭方向，原地旋轉 360° 不啟動"
        }
        return null
    }



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
            quarterArcAuthoritySeen = stickStatus.hasMsdkAuthority
            quarterArcLastRenderedAtNanos = 0L
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
        val ownsAuthority = stickStatus.hasMsdkAuthority
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
        yawLapTestActive = false
        turnStartedAtNanos = System.nanoTime()
        turnCommandStartedAtNanos = 0L
        turnAuthoritySeen = false
        turnLastRenderedAtNanos = 0L
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
        val yawLap = yawLapTestActive
        val label = if (yawLap) "航向模式原地旋轉 360°" else "$TURN_LABEL 180°"
        if (!stickOwned) {
            finishHeadingTurn("$label 失去控制權，已停止", release = false)
            return
        }
        val ownsAuthority = stickStatus.hasMsdkAuthority
        if (ownsAuthority) {
            turnAuthoritySeen = true
        } else if (turnAuthoritySeen) {
            finishHeadingTurn("遙控器已接管，$label 已停止", release = false)
            return
        } else {
            virtualStick.setYawRate(0.0)
            val waitedMs = (System.nanoTime() - turnStartedAtNanos) / 1_000_000L
            if (waitedMs > AUTHORITY_HANDOVER_TIMEOUT_MS) {
                finishHeadingTurn("未取得控制權，$label 已取消")
            } else {
                holdStatus = "等待控制權移交後開始$label…"
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
            finishHeadingTurn("機頭方向資料停止更新，$label 已停止")
            return
        }
        val timeoutNanos = if (yawLap) YAW_LAP_TIMEOUT_NANOS else TURN_TIMEOUT_NANOS
        if (now - turnStartedAtNanos > timeoutNanos) {
            finishHeadingTurn("$label 逾時，已轉 %.0f°".format(turn.progressDegrees))
            return
        }
        val remaining = (turn.targetDegrees - turn.progressDegrees).coerceAtLeast(0.0)
        val completed = if (yawLap) remaining == 0.0 else remaining <= TURN_TOLERANCE_DEGREES
        if (completed) {
            if (yawLap) {
                val elapsedSeconds =
                    (now - turnCommandStartedAtNanos).coerceAtLeast(1L) / 1_000_000_000.0
                val meanYawRate = turn.progressDegrees / elapsedSeconds
                finishHeadingTurn(
                    "航向模式原地旋轉 360° 完成：%.2f 秒，平均 %.1f°/秒"
                        .format(elapsedSeconds, meanYawRate),
                    succeeded = true,
                )
            } else {
                finishHeadingTurn(
                    "$TURN_LABEL 180° 完成（%.0f°）".format(turn.progressDegrees),
                    succeeded = true,
                )
            }
            return
        }
        virtualStick.setHorizontalVelocity(0.0, 0.0)
        if (yawLap) {
            val currentHeading = aircraftHeadingDegrees
            if (currentHeading == null || !currentHeading.isFinite()) {
                finishHeadingTurn("沒有有效機頭方向，$label 已停止")
                return
            }
            val targetHeading =
                wrapToSignedHeading(currentHeading + YAW_LAP_HEADING_LEAD_DEGREES)
            if (!virtualStick.setYawHeading(targetHeading)) {
                finishHeadingTurn("航向設定點無效，$label 已停止")
                return
            }
            if (now - turnLastRenderedAtNanos >= TURN_RENDER_PERIOD_NANOS) {
                val measurementElapsedSeconds =
                    turnLastMeasuredAtNanos.takeIf { it != 0L }?.let {
                        (now - it).coerceAtLeast(1L) / 1_000_000_000.0
                    }
                val instantaneousYawRate =
                    measurementElapsedSeconds?.let {
                        (turn.progressDegrees - turnLastMeasuredProgressDegrees) / it
                    }
                turnLastMeasuredAtNanos = now
                turnLastMeasuredProgressDegrees = turn.progressDegrees
                turnLastRenderedAtNanos = now
                val elapsedSeconds =
                    (now - turnCommandStartedAtNanos).coerceAtLeast(1L) / 1_000_000_000.0
                val meanYawRate = turn.progressDegrees / elapsedSeconds
                holdStatus =
                    "航向模式旋轉：%.0f° / 360°，%.2f 秒，平均 %.1f°/秒"
                        .format(turn.progressDegrees, elapsedSeconds, meanYawRate)
                flightLog.write(
                    (
                        "yaw lap progress=%.1f remaining=%.1f elapsed=%.2f mode=HEADING " +
                            "target=%.1f current=%.1f actualInstant=%s actualMean=%.2f speedLevel=%.3f"
                        ).format(
                        turn.progressDegrees,
                        remaining,
                        elapsedSeconds,
                        targetHeading,
                        currentHeading,
                        instantaneousYawRate.logNumber(2),
                        meanYawRate,
                        virtualStick.speedLevel(),
                    ),
                )
                render(holdStatus)
            }
            return
        }

        val speed =
            if (remaining <= TURN_SLOWDOWN_DEGREES) {
                TURN_SLOW_SPEED_DPS
            } else {
                TURN_SPEED_DPS
            }
        virtualStick.setYawRate(speed)
        if (now - turnLastRenderedAtNanos >= TURN_RENDER_PERIOD_NANOS) {
            turnLastRenderedAtNanos = now
            holdStatus = "$TURN_LABEL 180°：已轉 %.0f°，剩 %.0f°"
                .format(turn.progressDegrees, remaining)
            render(holdStatus)
        }
    }

    private fun finishHeadingTurn(
        message: String,
        succeeded: Boolean = false,
        release: Boolean = true,
    ) {
        val turn = headingTurn ?: return
        val yawLap = yawLapTestActive
        val now = System.nanoTime()
        val elapsedSeconds =
            turnCommandStartedAtNanos.takeIf { it != 0L }?.let {
                (now - it).coerceAtLeast(0L) / 1_000_000_000.0
            }
        val meanYawRate =
            elapsedSeconds?.takeIf { it > 0.0 }?.let { turn.progressDegrees / it }
        headingTurn = null
        yawLapTestActive = false
        turnCommandStartedAtNanos = 0L
        turnAuthoritySeen = false
        turnLastRenderedAtNanos = 0L
        turnLastMeasuredAtNanos = 0L
        turnLastMeasuredProgressDegrees = 0.0
        mainHandler.removeCallbacks(headingTurnTickRunnable)
        virtualStick.setYawRate(0.0)
        virtualStick.setHorizontalVelocity(0.0, 0.0)
        if (::yawLapTestButton.isInitialized) {
            yawLapTestButton.text = "航向模式原地旋轉 360°"
        }
        holdStatus = message
        if (yawLap) {
            flightLog.write(
                (
                    "yaw lap stopped success=$succeeded elapsed=${elapsedSeconds.logNumber(2)} " +
                        "progress=%.1f actualMean=${meanYawRate.logNumber(2)} reason=$message"
                    ).format(turn.progressDegrees),
            )
        } else {
            flightLog.write(message)
        }
        if (tapeEndpointTurn) {
            tapeEndpointTurn = false
            tapeDetector?.resetTracking()
            if (succeeded && tapeTracking.enabled && flying && stickOwned) {
                val resumedAt = System.nanoTime()
                tapeTracking.resumeAfterTurn(resumedAt)
                tapeTrackingStartedAtNanos = resumedAt
                tapeTrackingAuthoritySeen = stickStatus.hasMsdkAuthority
                renderedTapeTrackingPhase = TapeTrackingPhase.TURNING
                flightLog.write("tape endpoint turn complete; detection resumed")
                mainHandler.post(tapeTrackingPeriodicRunnable)
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
        if (landingAuthorityWaitPending) {
            render("正在等待已取消的控制權取得與回交收尾，請勿重複操作")
            return
        }
        if (!aircraftConnected) {
            render("飛機未連線，無法降落")
            return
        }
        if (holdingHeight) finishHeightHold("降落操作取消定高", release = false)
        if (headingTurn != null) {
            finishHeadingTurn(
                if (yawLapTestActive) {
                    "降落操作取消航向模式原地旋轉"
                } else {
                    "降落操作取消 180° 旋轉"
                },
                release = false,
            )
        }
        if (quarterArcController != null) {
            finishQuarterArc("降落操作取消無視覺 1/4 圈", release = false)
        }
        if (tapeTracking.enabled) {
            stopTapeTracking("降落操作取消黑膠帶追蹤", release = false)
        }
        if (hardwareLatencyStartPending || hardwareLatencyPulseSequence != null) {
            stopHardwareLatencyTest(
                "降落操作取消${activeHorizontalPulseName()}",
                release = false,
            )
        }
        if (mathematicalCircleStartPending || mathematicalCircleController != null) {
            stopMathematicalCircle("降落操作取消數學圓周", release = false)
        }
        stopTiltFlight("降落操作取消傾角實驗", release = false)
        holdStatus = ""
        if (straightLineReleasePending || pendingAcquireCallbacks > 0) {
            awaitCancelledAcquireBeforeLanding()
            return
        }
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

    private fun awaitCancelledAcquireBeforeLanding() {
        landingAuthorityWaitPending = true
        val deadline = System.nanoTime() +
            (TRANSITION_TIMEOUT_MS + AUTHORITY_HANDOVER_TIMEOUT_MS) * 1_000_000L
        var releaseRequested = false
        fun poll() {
            if (!landingAuthorityWaitPending || activityDestroying || !aircraftConnected || !flying) {
                landingAuthorityWaitPending = false
                return
            }
            if (System.nanoTime() >= deadline) {
                landingAuthorityWaitPending = false
                flightLog.write("landing withheld: cancelled acquire/release or RC authority unconfirmed")
                render("控制權收尾或 RC 回交未確認；未送降落，請用實體遙控器降落")
                return
            }
            if (pendingAcquireCallbacks == 0 && !stickTransitionPending) {
                if (stickStatus.isReleased && !stickOwned &&
                    lastStickStatusAtNanos >= lastReleaseRequestedAtNanos
                ) {
                    landingAuthorityWaitPending = false
                    sendLandingCommand()
                    return
                }
                if (!releaseRequested) {
                    releaseRequested = true
                    releaseControlLink { error ->
                        if (error != null) {
                            landingAuthorityWaitPending = false
                            render("交還控制權失敗，未送降落；請用實體遙控器降落：$error")
                        } else {
                            poll()
                        }
                    }
                    return
                }
            }
            render("等待取得控制權的取消收尾，再確認 RC 控制權…")
            mainHandler.postDelayed(::poll, AUTHORITY_POLL_MS)
        }
        poll()
    }

    /** Waits for RC ownership, or disabled virtual stick when the owner stays UNKNOWN. */
    private fun awaitRemoteAuthority(onRemote: () -> Unit) {
        val deadline = System.nanoTime() + AUTHORITY_HANDOVER_TIMEOUT_MS * 1_000_000L
        fun poll() {
            if (stickStatus.isReleased) {
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
        if (registered && aircraftConnected && (flying || isStraightLineTest())) return
        if (headingTurn != null) {
            finishHeadingTurn("飛行狀態結束，旋轉已停止", release = false)
        }
        if (quarterArcController != null) {
            finishQuarterArc("飛行狀態結束，無視覺 1/4 圈已停止", release = false)
        }
        if (tapeTracking.enabled) {
            stopTapeTracking("飛行狀態結束，黑膠帶追蹤已停止", release = false)
        }
        if (hardwareLatencyStartPending || hardwareLatencyPulseSequence != null) {
            stopHardwareLatencyTest(
                "飛行狀態結束，${activeHorizontalPulseName()}已停止",
                release = false,
            )
        }
        if (mathematicalCircleStartPending || mathematicalCircleController != null) {
            stopMathematicalCircle("飛行狀態結束，數學圓周已停止", release = false)
        }
        stopTiltFlight("飛行狀態結束，傾角實驗已停止", release = false)
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
        if (hardwareLatencyStartPending || hardwareLatencyPulseSequence != null) {
            stopHardwareLatencyTest(
                "控制權釋放，${activeHorizontalPulseName()}已停止",
                release = false,
            )
        }
        if (mathematicalCircleStartPending || mathematicalCircleController != null) {
            stopMathematicalCircle("控制權釋放，數學圓周已停止", release = false)
        }
        stopTiltFlight("控制權釋放，傾角實驗已停止", release = false)
        val generation = beginTransition("release") {
            onDone("釋放控制權無回應")
        }
        heightTargetMeters = null
        mainHandler.removeCallbacks(holdTickRunnable)
        virtualStick.setClimbRate(0.0)
        holdStatus = ""
        if (headingTurn != null) {
            headingTurn = null
            yawLapTestActive = false
            turnAuthoritySeen = false
            turnCommandStartedAtNanos = 0L
            turnLastRenderedAtNanos = 0L
            turnLastMeasuredAtNanos = 0L
            turnLastMeasuredProgressDegrees = 0.0
            mainHandler.removeCallbacks(headingTurnTickRunnable)
            virtualStick.setYawRate(0.0)
            if (::yawLapTestButton.isInitialized) {
                yawLapTestButton.text = "航向模式原地旋轉 360°"
            }
        }
        lastReleaseRequestedAtNanos = System.nanoTime()
        virtualStick.disable { error ->
            runOnUiThread {
                if (!endTransition(generation)) {
                    if (error == null && generation == transitionGeneration) stickOwned = false
                    flightLog.write("late release callback ignored generation=$generation error=$error")
                    if (!activityDestroying) render("控制權回交回應：${error ?: "已接受"}")
                    return@runOnUiThread
                }
                if (error == null) stickOwned = false
                flightLog.write("release done error=$error")
                onDone(error)
            }
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
        shouldContinue: () -> Boolean = { true },
        onOwned: () -> Unit,
    ) {
        if (!shouldContinue()) {
            acquireAttempts = 0
            onFailure?.invoke("取得控制權已取消")
            return
        }
        if (pendingAcquireCallbacks > 0 || landingAuthorityWaitPending) {
            onFailure?.invoke("先前的控制權操作尚未收尾，不重新取得")
            return
        }
        if (stickOwned) {
            onOwned()
            return
        }
        if (!registered || !aircraftConnected || (!isStraightLineTest() && !flying)) {
            val reason = "SDK 或飛機連線未就緒，或目前模式要求先起飛，無法取得控制權"
            flightLog.write("acquire refused: registered=$registered connected=$aircraftConnected flying=$flying")
            render(reason)
            onFailure?.invoke(reason)
            return
        }
        if (stickTransitionPending) {
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
        pendingAcquireCallbacks += 1
        virtualStick.enable { error ->
            runOnUiThread {
                val currentTransition = endTransition(generation)
                if (!currentTransition || !shouldContinue() || activityDestroying || landingAuthorityWaitPending) {
                    if (error == null) {
                        stickOwned = true
                        // Keep the pending count until the compensating disable really answers.
                        lastReleaseRequestedAtNanos = System.nanoTime()
                        virtualStick.disable { releaseError ->
                            runOnUiThread {
                                pendingAcquireCallbacks -= 1
                                if (releaseError == null) stickOwned = false
                                if (!activityDestroying) {
                                    render(releaseError?.let { "取消收尾失敗：$it，請用實體遙控器接管" }
                                        ?: "已取消取得控制權；等待 RC 回交確認")
                                }
                            }
                        }
                    } else {
                        pendingAcquireCallbacks -= 1
                        if (!activityDestroying) render("取得控制權的取消已完成")
                    }
                    if (currentTransition && !activityDestroying) onFailure?.invoke("取得控制權已取消")
                    return@runOnUiThread
                }
                pendingAcquireCallbacks -= 1
                stickOwned = error == null
                flightLog.write("acquire attempt=$attempt owned=$stickOwned error=$error")
                when {
                    stickOwned -> {
                        acquireAttempts = 0
                        onOwned()
                    }
                    attempt < MAX_ACQUIRE_ATTEMPTS && flying && shouldContinue() -> {
                        render("等待起飛完成才能取得控制權（第 $attempt 次）：$error")
                        mainHandler.postDelayed(
                            { acquireControlLink(onFailure, shouldContinue, onOwned) },
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
            !isStraightLineTest() &&
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
            percent != null && percent <= BATTERY_CRITICAL_PERCENT && !isStraightLineTest() ->
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
            render("旋轉進行中，無法同時調整高度")
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
        if (hardwareLatencyStartPending || hardwareLatencyPulseSequence != null) {
            render("${activeHorizontalPulseName()}進行中，無法同時調整高度")
            return
        }
        if (mathematicalCircleStartPending || mathematicalCircleController != null) {
            render("數學圓周進行中，無法同時調整高度")
            return
        }
        if (tiltFlightRun != null) {
            render("傾角實驗進行中，無法同時調整高度")
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
        val ownsAuthority = stickStatus.hasMsdkAuthority
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
        /** Heading-setpoint experiment, stopped by measured heading after one revolution. */
        const val YAW_LAP_TARGET_DEGREES = 360.0
        const val YAW_LAP_HEADING_LEAD_DEGREES = 100.0
        const val YAW_LAP_TIMEOUT_NANOS = 8_000_000_000L
        const val TURN_RENDER_PERIOD_NANOS = 250_000_000L
        const val MAX_MOVING_HEADING_AGE_NANOS = 1_000_000_000L
        const val AIRCRAFT_VELOCITY_STALE_NANOS = 1_000_000_000L
        const val MIN_DIRECTIONAL_GROUND_SPEED_METERS_PER_SECOND = 0.12
        /**
         * Historical movement gate for controller turn-cycle diagnostics. This value is not the
         * measured track perimeter and diagnostic cycles must never be reported as physical laps.
         */
        const val MIN_DIAGNOSTIC_TURN_CYCLE_DISTANCE_METERS = 6.5

        /** Fixed-radius quarter-arc experiment gates and closed-heading endpoint. */
        const val QUARTER_ARC_START_MIN_HEIGHT_METERS = 0.55
        const val QUARTER_ARC_START_MAX_HEIGHT_METERS = 0.85
        const val QUARTER_ARC_FLIGHT_MIN_HEIGHT_METERS = 0.45
        const val QUARTER_ARC_FLIGHT_MAX_HEIGHT_METERS = 0.95
        const val QUARTER_ARC_TOLERANCE_DEGREES = 2.0
        const val QUARTER_ARC_TICK_MS = 50L
        const val QUARTER_ARC_TIMEOUT_NANOS = 20_000_000_000L
        const val QUARTER_ARC_RENDER_PERIOD_NANOS = 250_000_000L

        /** Low indoor envelope for the bounded forward/backward latency experiment. */
        val HARDWARE_LATENCY_START_HEIGHT_RANGE_METERS = 0.5..1.0
        val HARDWARE_LATENCY_FLIGHT_HEIGHT_RANGE_METERS = 0.4..1.1

        /** Elapsed-time circle schedule; velocity feedback and vision never alter its path. */
        val MATHEMATICAL_CIRCLE_START_HEIGHT_RANGE_METERS = 0.5..1.0
        val MATHEMATICAL_CIRCLE_FLIGHT_HEIGHT_RANGE_METERS = 0.4..1.1
        const val MATHEMATICAL_CIRCLE_RENDER_PERIOD_NANOS = 250_000_000L
        const val MATHEMATICAL_CIRCLE_MAX_TICK_GAP_NANOS = 150_000_000L

        /** Additional bounds for the two no-vision horizontal ANGLE experiments. */
        const val TILT_COMMAND_LEASE_NANOS = 150_000_000L
        const val TILT_PREPARATION_TIMEOUT_MS = 15_000L
        const val TILT_START_MAX_SPEED_MPS = 0.2
        const val TILT_FLIGHT_MAX_SPEED_MPS = 1.2
        const val TILT_FLIGHT_MAX_ATTITUDE_DEGREES = 15.0
        const val TILT_FLIGHT_MAX_HEADING_ERROR_DEGREES = 45.0


        /** Grace period before centred sticks hand the aircraft back to the RC. */
        const val STICK_IDLE_RELEASE_MS = 3_000L

        /**
         * Auto-takeoff climbs for a couple of seconds and refuses virtual stick
         * until it finishes, so acquisition retries for ~12 s before giving up.
         */
        const val MAX_ACQUIRE_ATTEMPTS = 24
        const val ACQUIRE_RETRY_MS = 500L

        /** Main-thread cadence for command transitions and immediate authority-loss detection. */
        const val HARDWARE_LATENCY_TICK_MS = 10L

        /**
         * How long the aircraft may take to report enabled MSDK/UNKNOWN authority
         * after enableVirtualStick succeeds. Mini 4 Pro normally names MSDK in
         * ~60 ms; Mini 3 Pro may keep reporting UNKNOWN.
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
        /** Safety fallback and command-ramp cadence; fresh vision still triggers immediate control. */
        const val TAPE_TRACKING_TICK_MS = 50L
        const val TAPE_CAPTURE_DIRECTORY = "tape-captures"
        const val FLIGHT_PROFILE_FILE_PREFIX = "flight-profile-"
        const val FLIGHT_PROFILE_FILE_SUFFIX = ".tsv"
        /** Lets the recorder receive its four 4 Hz post-loss frames before disarming. */
        const val AUTO_CAPTURE_DISARM_DELAY_MS = 1_500L
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
