# 10 秒一圈雙方案實作計劃：方案 B／方案 C 各一個按鈕

> 交付對象：負責實作的 AI agent。
> 背景：`docs/ten-second-lap-problem-statement.md`（問題）、`docs/ten-second-lap-plan.md`（可行性與方案分析）。
> 基線：`main` = `e382d74`。

---

## 0. 目的與不可違反的原則

**目的**：讓下週一的實機時間能在同一個 APK 裡直接切換兩種 10 秒方案，各自獨立啟停、獨立 log，現場依 Stage 0 量測結果決定先飛哪個。

- **按鈕 C「切線高速循跡」**：機頭沿切線，yaw 前饋＋回授，小角度 crab 補償，聯合速度規劃。需要機體 yaw ≥ 36°/s。
- **按鈕 B「定向滑行循跡」**：機頭固定不轉，純 body-frame 速度向量繞圈。不需要 yaw，但需要全方位的影像幾何。

**原則（實作時逐條自檢）**

1. **只加不改**：既有 `STRAIGHT`／`CIRCULAR`／`CURVED_OUT_AND_BACK` 三個模式、其常數、狀態機與測試**一行都不動**。B 與 C 各是新模式或新控制器，用新按鈕啟動。這是唯一能保證「已驗證的 34 秒版本仍在 APK 裡可回退」的做法。
2. **手動搖桿語意不動**：`setStick` 仍用 `MAX_HORIZONTAL_MPS = 0.5`、`MAX_YAW_DEGREES_PER_SECOND = 20`，手動障礙門檻仍 150 mm。
3. **不放寬任何安全停止**：失偵停止、障礙停止、authority 接管、負高度停止、VirtualStick 失敗停止全部保留；新模式只能**更早**停，不能更晚。
4. **任何高速常數都是實驗旗標**：B／C 的速度與 yaw 上限必須能在不重裝 APK 的情況下降階（至少用 UI 可選的階梯；見第 2.5 節）。
5. **先量後信**：沒有 `KeyAircraftVelocity` 實測地速與 `LapTimer` 記錄的圈時，不宣稱任何成績。

---

## 1. UI 與操作流程

### 1.1 按鈕

放在 `buildExperimentActionRow()`（`MainActivity.kt` ≈ 480 行）第二列，與「無視覺右轉 1/4 圈」同列，樣式沿用 `PillButton`：

| 變數 | 文字（閒置／進行中） | 顏色 | 動作 |
|---|---|---|---|
| `fastTangentLapButton` | 「切線高速循跡」／「停止切線高速循跡」 | CYAN | `toggleTapeTracking(TapeTrackingMode.CIRCULAR_FAST)` |
| `fixedHeadingLapButton` | 「定向滑行循跡」／「停止定向滑行循跡」 | CYAN | `toggleFixedHeadingLap()` |
| `lapSpeedStageButton`（小） | 「階：20s」循環 → 「15s」→「12s」→「10s」→「20s」 | AMBER | 切換第 2.5 節的速度階梯，兩個方案共用；循跡進行中不可切換 |

`available` 條件與「圓形黑膠帶追蹤」完全相同（`ready && flying && !turning && !quarterArcActive && !holdingHeight`，且任一模式進行中時只有該模式的按鈕可按）。`anotherFlightControlActive()` 加入 `fixedHeadingLap != null`。

### 1.2 起始程序（兩方案相同，與現行圓形循跡一致）

操作者手動把飛機懸停在膠帶上方、約 1.2 m、機頭大致沿膠帶方向（膠帶在畫面中由下往上）。按下按鈕後：取得控制權 → 既有對正／重取流程 → 進入 TRACKING → `LapTimer` 在「穩定 TRACKING ≥ 1 s 且 `v_cmd ≥ 0.8·v_profile`」時起算。

### 1.3 停止

任一既有停止條件、操作者按同一按鈕、任一搖桿偏移、實體遙控器接管，均走既有 `stopTapeTracking` 路徑（B 走自己的 `stopFixedHeadingLap`，語意相同：歸零水平與 yaw、釋放或保留控制權、寫 log）。

### 1.4 狀態列

既有 telemetry 文字加一行：`lap=<n> t=<s> gs=<m/s> yawRate=<°/s> limit=<reason> θ=<°> ex=<frac>`。

---

## 2. 共用基礎（Phase 0，兩方案都依賴，先做）

### 2.1 `VirtualStickSession.kt`：自主夾限與手動夾限分離

```kotlin
const val MAX_HORIZONTAL_MPS = 0.5               // 手動，不動
const val MAX_YAW_DEGREES_PER_SECOND = 20.0      // 手動，不動
const val AUTONOMOUS_MAX_HORIZONTAL_MPS = 0.8
const val AUTONOMOUS_MAX_YAW_DEGREES_PER_SECOND = 60.0   // 硬上限；實際使用值由階梯決定
```

- `setStick` 不變。
- `setHorizontalVelocity`／`setYawRate`（自主呼叫者）改用 `AUTONOMOUS_*` 夾限。
- `setForwardOnly` 保持手動夾限（它是手動脈衝用）。
- 每秒 frame summary 加上目前 `|yaw|`、`|v|` 的最大值，作為「送出值」證據。
- 既有 `TapeTrackingController.CIRCULAR_MAX_YAW_RATE_DEGREES_PER_SECOND = VirtualStickSession.MAX_YAW_DEGREES_PER_SECOND` **維持引用手動值**（`CIRCULAR` 模式行為不變）。

### 2.2 機體速度與 yaw rate 遙測（`MainActivity.kt`）

- 訂閱 `FlightControllerKey.KeyAircraftVelocity`（`Velocity3D`，NED，m/s）→ `aircraftVelocityNorth/East/Down`、`groundSpeedMetersPerSecond = hypot(N, E)`、`aircraftVelocityAtNanos`，listener 包 `runOnUiThread`（與 attitude 一致）。
- 由 `publishAircraftAttitude` 的連續 heading 差分（`HeadingTurn.shortestAngularDelta`）算 `aircraftYawRateDegreesPerSecond`，用 100 ms 以上的視窗避免量化噪訊。
- 兩者納入 `usableTelemetry` 判定：任一 > 1 s 未更新 → B／C 停止（新增 abort 條件，不影響舊模式）。

### 2.3 `LapTimer`（新檔 `LapTimer.kt`，JVM 可測）

```kotlin
internal class LapTimer(private val degreesPerLap: Double = 360.0) {
    fun arm(nowNanos: Long, initialAngleDegrees: Double)      // 起算條件成立時呼叫
    fun update(nowNanos: Long, angleDegrees: Double): LapEvent? // 累計有向位移，跨 ±180° 用 HeadingTurn.shortestAngularDelta
    val lapCount: Int; val currentLapElapsedSeconds: Double; val progressDegrees: Double
}
data class LapEvent(val lapIndex: Int, val lapSeconds: Double)
```

- C 餵 `aircraftHeadingDegrees`；B 餵虛擬航向 ψ_v（見 4.3）。
- 方向：有向位移取絕對值累計（順／逆時針皆可），但**一旦起算就鎖定方向**，反向位移不計入（避免來回擺動灌水）。
- 每圈完成寫 `flightLog`：`lap n=<i> seconds=<…> meanGs=<…> meanCmd=<…> yawSatFrac=<…> fullPathFrac=<…> staleEvents=<…>`。

### 2.4 每 tick 同步 log（兩方案共用格式）

在套用決策處（`MainActivity.kt` ≈ 864–900 行，B 在自己的 tick 內）每 100 ms 一行，同一 `now`：

```text
lapTick mode=<CIRCULAR_FAST|FIXED_HEADING> stage=<20|15|12|10> phase=<…> q=<FULL|NEAR|LOST> conf=<…>
  theta=<deg> ex=<frac> kappa=<1/m> lookahead=(<x>,<y>) dxL=<frac>
  vPlan=<m/s> limit=<profile|yaw|lateral|vision|stale|align> vFwdCmd=<…> vRightCmd=<…> yawCmd=<deg/s>
  gs=<m/s> yawRateAct=<deg/s> heading=<deg> psiV=<deg|n/a> h=<m>
  detMs=<…> obsAgeMs=<…> front=<mm> side=<mm> rear=<mm>
  lap=<n> lapElapsed=<s> lapProgress=<deg>
```

另每秒一行摘要（yaw 飽和比例、FULL_PATH 比例、stale 事件、`acceptedHz`、`callbacks/maxGapMs`）。`TapeCapture` 的 metadata 同步寫入 `mode`、`stage`、`psiV`、`vPlan`。

### 2.5 速度階梯（`LapSpeedStage` 列舉，新檔）

| 階 | 目標圈時 | `vProfile` (m/s) | `yawMax` (°/s) | `accelUp` (m/s²) | `degradedSpeed` |
|---|---|---|---|---|---|
| S20 | 20 s | 0.32 | 30 | 0.25 | 0.15 |
| S15 | 15 s | 0.42 | 40 | 0.30 | 0.18 |
| S12 | 12 s | 0.52 | 45 | 0.35 | 0.20 |
| S10 | 10 s | 0.62 | 50 | 0.40 | 0.22 |

B 不使用 `yawMax`（其 yaw 指令恆為 0），其他欄位共用。階梯值必須 ≤ 2.1 節硬上限，以單元測試鎖定。**預設 S20**。

### 2.6 方向性障礙（`HorizontalObstacleFilter.kt` + `ObstacleRanges.kt`）

- 新增 `sectorNearest(distancesMm, intervalDegrees, zeroIndexAzimuthDegrees, travelAzimuthDegrees)` → `front`（±45°）、`side`（45–135° 兩側）、`rear` 三個最近值（套用既有地板抑制）。
- 新常數 `OBSTACLE_INDEX_ZERO_AZIMUTH_DEGREES: Double?`（預設 `null`）。**為 null 時退回現行全向 500 mm**；現場校準後（機頭對牆、log 中最小 index）才填值。
- 前向門檻 `frontStopMm(v) = 300 + v·600 + v²/2·1000`（0.6 m/s ≈ 840 mm）；側 350 mm；後 300 mm。
- C 的行進方位 = 機頭 + 小 crab 角；B 的行進方位 = ψ_v（body frame）。
- freshness watchdog **仍不加入**（維持 PR #3 結論，等 `callbacks/maxGapMs` 量測）。

### 2.7 Stage 0 量測按鈕（選配，但強烈建議）

`stepTestButton`「階躍量測」：懸停時依序送 yaw 20→30→40→50→60 °/s 各 2 s（間隔 1 s 歸零），再 forward 0.3／0.5／0.7 m/s 各 1.5 s（需操作者確認 3 m 淨空後才執行速度段；速度段前彈出確認 toast 並要求再按一次）。log 前綴 `step yaw=…`／`step vel=…`，每 50 ms 記 `yawRateAct`／`gs`。任一搖桿偏移立即中止。

---

## 3. 方案 C：`TapeTrackingMode.CIRCULAR_FAST`

### 3.1 定位

在 `TapeTrackingController` 內新增 `CIRCULAR_FAST(followsCurvedPath = true)`。共用 `CIRCULAR` 的觀測、相位、重取、對正、端點（`endpointTurnEnabled = false`）全部邏輯；**只有**速度規劃、yaw 律、右移律、加速度、stale 窗五處依 `mode == CIRCULAR_FAST` 分支。既有 `CIRCULAR` 路徑的行為與測試不變。

### 3.2 控制律（每 tick，`TRACKING` 相位、`FULL_PATH`）

輸入：θ = `controlledAngleDegrees`（右正）、e_x = `controlledHorizontalOffsetFraction`（右正）、κ = `desiredPurePursuitCurvaturePerMeter()`（右彎正）、階梯參數 S。

```text
// 速度規劃
v_yaw     = toRadians(S.yawMax − 2°/s) / |κ|                // 保留 2°/s 回授餘裕
v_lateral = sqrt(A_LAT_MAX / |κ|)                            // A_LAT_MAX = 0.5 m/s²
v_vision  = (q == FULL_PATH && conf ≥ 0.6) ? ∞ : S.degradedSpeed
v_align   = |θ| > 35° ? S.degradedSpeed : ∞
v_target  = min(S.vProfile, v_yaw, v_lateral, v_vision, v_align)
v_cmd     = rampUp(v_target, S.accelUp·dt)；下降立即（沿用 applyForwardAccelerationLimit 的不對稱語意）
limitReason = 取到最小值的那一項名稱（寫入決策供 log）

// yaw：前饋 + 回授（現行是二選一，這裡改為相加，只在 CIRCULAR_FAST）
ω_ff  = toDegrees(v_cmd · κ)
ω_fb  = K_P_FAST · deadzone(θ, 1.5°) + K_D_FAST · dθ/dt      // K_P_FAST = 0.4, K_D_FAST = 0.05（起調值）
ω_cmd = clamp(ω_ff + ω_fb, ±S.yawMax)，再經 applyOutputLimits 的 yaw 斜率限制（CIRCULAR_MAX_YAW_ACCELERATION = 30°/s² 沿用）

// crab 補償 + 小 centering
v_forward = v_cmd · cos θ
v_right   = v_cmd · sin θ + clamp(K_LAT_FAST · deadzone(e_x, 0.05), ±0.08)   // K_LAT_FAST = 0.15
```

- 符號：影像右 = 正 yaw = 正 right（已由真機證據與既有測試 `image horizontal direction maps directly to DJI yaw sign` 鎖定）；新測試必須沿用同一慣例。
- `|θ| > 50°` 持續 300 ms → 進入既有 `ALIGNING_CURVE`（沿用 `CURVE_ALIGNMENT_ENTER_ANGLE_DEGREES` 的機制，但 FAST 用自己的門檻常數）。
- `NEAR_FIELD_ONLY`／`LOST`：沿用 `CIRCULAR` 的處理（不前進、只對正／重取）。

### 3.3 stale 窗改為距離式（只在 CIRCULAR_FAST）

```text
staleNanos(v) = clamp( D_BLIND_MAX / max(v_cmd, 0.05) , 150 ms , 400 ms )     // D_BLIND_MAX = 0.08 m
```

- `tick()` 的 `nowNanos - lastDetectionAtNanos > DETECTION_COMMAND_STALE_NANOS` 判定，在 FAST 模式改呼叫 `staleNanos(appliedForwardSpeed)`。
- gap 期間（`circularDetectionGap == true`）`v_target` 以 `S.degradedSpeed` 為上限且不再上升（現行 `CIRCULAR` 允許 gap 中加速的行為**不帶進** FAST）。

### 3.4 新常數（全部 `CIRCULAR_FAST_` 前綴，放在 companion 尾端獨立區塊）

`A_LAT_MAX`、`K_P_FAST`、`K_D_FAST`、`K_LAT_FAST`、`MAX_LATERAL_TRIM_MPS = 0.08`、`CRAB_DEGRADE_ANGLE = 35`、`CRAB_ALIGN_ANGLE = 50`、`D_BLIND_MAX = 0.08`、`STALE_MIN_NANOS = 150 ms`、`STALE_MAX_NANOS = 400 ms`、`MIN_CONFIDENCE = 0.6`。階梯值不放這裡，由 `LapSpeedStage` 注入（`start(now, mode, endpointTurnEnabled, stage)`）。

### 3.5 `TapeTrackingDecision` 擴充

新增 `plannedSpeedMetersPerSecond`、`speedLimitReason: String?`、`curvaturePerMeter: Double?`、`feedforwardYawRate`、`feedbackYawRate`（皆有預設值，不影響既有建構）。

### 3.6 修改點清單

| 檔案 | 改動 |
|---|---|
| `TapeTrackingController.kt` | 列舉加 `CIRCULAR_FAST`；`start()` 加 `stage` 參數（預設 S20）；`tick()` 的 TRACKING 分支在 FAST 時走 3.2／3.3；新私有函式 `planFastForwardSpeed()`、`fastYawRate()`、`fastRightSpeed()`、`staleNanosFor(v)`；`decision()` 填新欄位 |
| `MainActivity.kt` | 按鈕；`tapeTrackingName()` 加「切線高速循跡」；`startTapeTracking` 在 FAST 時傳 `stage`；套用決策處寫 2.4 log；`LapTimer` 餵 `aircraftHeadingDegrees`；`tapeTrackingStopReason` 加入 2.2 的遙測中斷與 2.6 的方向性扇區 |
| `TapeTrackingControllerTest.kt` | 新增 FAST 測試組（見 6.1），**不改動任何既有測試** |

---

## 4. 方案 B：`FixedHeadingLapController`（新檔）

### 4.1 定位與關鍵幾何事實

- 機頭不轉：`yaw = 0`。行進方向由**虛擬航向 ψ_v**（body frame，0° = 機頭、順時針正）表示，速度向量 = `v·(sin ψ_v, cos ψ_v)` = (right, forward)。
- **影像參考點必須改到畫面中心附近**：現行 `TRACKING_TARGET_Y_FRACTION = 0.94` 把參考點放在底緣、整個畫面視為「前方」，這在 B 不成立（行進方向會轉過 360°）。B 使用 `OMNI_REFERENCE_X = 0.5`、`OMNI_REFERENCE_Y = 0.55`（-90° 相機、機身 CG 在鏡頭後約 5 cm、1.2 m 高度 → 約 5.5% 畫面高度；**現場以懸停在膠帶正上方時膠帶穿過的點校正**，提供一個常數可改）。
- 1.2 m 高度下，中心向上下各約 0.45 m、左右各約 0.8 m 可見：B 的 lookahead 距離 L 以 0.35 m 為目標（`OMNI_LOOKAHEAD_METERS`），對 R = 0.75 m 的圓 PP 仍精確。

### 4.2 detector 端：`TapeDetectionMode.OMNIDIRECTIONAL`

新增一個 detection mode，**只**改變以下與「由下往上」假設有關的地方（其餘 Otsu、色度、地板、形態學全部沿用 `PATH`）：

1. **錨點**：不取最靠近底緣的點，改取中心線上**最接近 `(OMNI_REFERENCE_X, OMNI_REFERENCE_Y)`** 的點（垂足）。
2. **近場／遠場與 `longSideFraction`**：以「錨點兩側的弧長」取代「由底緣向上的弧長」。
3. **所有以 `bounds.bottom`／底緣接觸為條件的接受或拒絕**（例：`ENDPOINT_NEAR_EDGE`、route start、near-field-only 判定）在 OMNI 模式改為「中心線最小距參考點 ≤ `OMNI_MAX_REFERENCE_DISTANCE`（0.25 畫面高度）」。實作前先 `grep -n "bottom\|anchor\|routeStart\|nearField" BlackTapeDetector.kt`（目前約 27 處）逐一判定是否屬於此類，逐條在 PR 說明中列出「動／不動」與理由。
4. **輸出**：`TapeDetection.centerline` 必須帶完整有序點（已有）；新增 `TapeDetection.omni: OmniCenterlineMeasurement?`（OMNI 模式才非 null），欄位見 4.3。
5. `lookahead` 在 OMNI 模式由 4.3 的量測給出（依行進方向），不再由 `CenterlineMeasurement.measure()` 的「由錨點向上」規則給。`quality` 語意不變：有可用 lookahead = `FULL_PATH`。

### 4.3 幾何量測：`OmniCenterlineMeasurement`（新檔，JVM 可測）

```kotlin
internal data class OmniCenterlineMeasurement(
    val footX: Double, val footY: Double,                 // 參考點在中心線上的垂足（影像分數座標）
    val lateralOffsetFraction: Double,                    // 參考點到中心線的有號垂距（以 ψ_v 的右手邊為正）
    val tangentDegrees: Double,                           // 垂足處切線方向，body frame 0..360，已用 ψ_v 消歧
    val lookahead: TapeLookahead?,                        // 自垂足沿 +tangent 方向走弧長 L 的點
    val curvaturePerMeter: Double?,                       // 垂足–lookahead 的 PP 弦曲率（度量，用高度與 FOV）
    val arcAheadFraction: Double, val arcBehindFraction: Double,
)
internal object OmniCenterline {
    fun measure(points: List<CenterlinePoint>, frameWidth: Int, frameHeight: Int,
                referenceX: Double, referenceY: Double,
                travelDirectionDegrees: Double,           // ψ_v，用於消除 ±180° 歧義
                heightMeters: Double?, lookaheadMeters: Double): OmniCenterlineMeasurement?
}
```

- 切線方向由垂足兩側各 `TANGENT_SPAN` 點做最小平方；兩個候選方向取與 `travelDirectionDegrees` 夾角較小者。
- 若兩候選與 ψ_v 夾角都 > 70°（路徑幾乎橫切於行進方向）→ 回傳 `null`（視為不可用，控制器會減速／停止）。
- 度量換算沿用 `TapeTrackingController` 既有的「高度 × 對角 FOV」公式，抽成共用函式 `GroundProjection.metersPerFraction(height, aspect)`（**抽出時不得改變 `CIRCULAR` 的數值結果**，用既有測試鎖定）。

### 4.4 控制器：`FixedHeadingLapController`（新檔，JVM 可測）

狀態：`DISABLED → ACQUIRING → TRACKING → (COASTING) → TRACKING | STOPPED`，刻意簡單，不含端點／回轉。

```text
輸入：OmniCenterlineMeasurement?（每幀）、h、stage S、nowNanos
狀態量：ψ_v（初值 0 = 機頭方向）、v_cmd、lastDetectionAtNanos、濾波後的 θ_rel、e_x、κ

θ_rel = wrap(tangent − ψ_v)                            // 行進方向相對切線的誤差，右正

// 虛擬航向更新（取代 yaw）
ω_v   = toDegrees(v_cmd·κ) + K_P_OMNI·θ_rel + K_D_OMNI·dθ_rel/dt
ψ_v  += clamp(ω_v, ±OMNI_MAX_TURN_RATE)·dt           // OMNI_MAX_TURN_RATE = 90°/s（純軟體，無機體限制）

// 速度規劃（與 C 相同結構，少了 v_yaw）
v_target = min(S.vProfile, sqrt(A_LAT_MAX/|κ|), v_vision, |θ_rel|>35° ? S.degradedSpeed : ∞)
v_cmd    = rampUp(v_target, S.accelUp·dt)；下降立即

// 輸出（body frame）
forward = v_cmd·cos ψ_v − K_LAT_OMNI·e_x·sin ψ_v       // 橫向修正沿切線的法向施加
right   = v_cmd·sin ψ_v + K_LAT_OMNI·e_x·cos ψ_v
yaw     = 0
```

- **ACQUIRING**：第一筆 `FULL_PATH` 且 `|θ_rel| ≤ 20°`、`|e_x| ≤ 0.15` 持續 500 ms 才進 TRACKING（與現行對正語意對齊，但不旋轉機頭；不符合時 v = 0，等待操作者微調或逾時 8 s 停止）。
- **COASTING**：失偵時維持 ψ_v 不變、v 以 `accelUp` 兩倍速率衰減；超過 `staleNanos(v)`（與 3.3 同式）→ STOPPED。
- **方向一致性**：`tangent` 消歧失敗（4.3 回 null）連續 300 ms → STOPPED（寧停勿反向）。
- `LapTimer` 餵 ψ_v。

### 4.5 `MainActivity.kt` 整合

- 新欄位 `fixedHeadingLap: FixedHeadingLapController?`、`fixedHeadingLapTickRunnable`（100 ms）、`toggleFixedHeadingLap()`、`startFixedHeadingLap()`、`stopFixedHeadingLap(reason, release)`，結構比照 `toggleQuarterArc()`／`driveQuarterArc()`／`finishQuarterArc()`（含 authority 握手、逾時、接管）。
- 啟動時 `tapeDetector.detectionMode = OMNIDIRECTIONAL`；停止時還原先前模式。`handleTapeDetection` 在 B 啟用時把 `detection.omni` 餵給 B 控制器，**不**餵 `tapeTracking`。
- 套用決策：`virtualStick.setYawRate(0.0)`、`setHorizontalVelocity(forward, right)`；前置 `tapeTrackingStopReason()`（行進方位 = ψ_v）。
- 所有既有全域停止點（降落、飛行結束、控制權釋放、搖桿偏移、實體遙控器）加入 `if (fixedHeadingLap != null) stopFixedHeadingLap(...)`，與 quarter-arc 的加法方式相同。
- Overlay：在 `TapeOverlayView` 畫出參考點、垂足、lookahead 與 ψ_v 箭頭（B 啟用時），便於現場目視除錯。

---

## 5. 安全與 abort（兩方案共用，新模式專用，不影響舊模式）

| 條件 | 動作 |
|---|---|
| `|e_x| > 0.35` 持續 300 ms、或 `|θ|`/`|θ_rel|` > 50° 持續 300 ms | C：進 ALIGNING_CURVE；B：停止 |
| 失偵超過 `staleNanos(v)`；conf < 0.6 連續 3 幀 | 停止（hover） |
| 障礙：前扇區 ≤ `frontStopMm(v)`、側 ≤ 350、後 ≤ 300（未校準時全向 500） | 停止 |
| 高度／姿態／速度遙測 > 1 s 未更新 | 停止 |
| VirtualStick frame 失敗、authority 非 MSDK | 既有接管 |
| 命令速度與 `gs` 差 > 0.25 m/s 持續 1 s；C 的 yaw 飽和連續 > 1 s | 自動降一階（log 記錄），再發生則停止 |
| 搖桿偏移／實體遙控器／降落／飛行結束／控制權釋放 | 既有接管（B 比照 quarter-arc 加法） |

---

## 6. 測試與驗證

### 6.1 JVM 單元測試（必須）

- `LapTimerTest`：跨 ±180°、方向鎖定、反向不計、兩圈計時。
- `LapSpeedStageTest`：每階值 ≤ 自主硬上限；預設 S20。
- `TapeTrackingControllerTest`（新增區塊，不改舊測試）：FAST 模式下 (a) 速度規劃取最小值且 `speedLimitReason` 正確；(b) yaw = 前饋+回授且 sign 與既有慣例一致；(c) crab 分配 `forward = v cos θ`、`right = v sin θ`；(d) stale 窗隨速度縮短至 150 ms 下限；(e) gap 期間不加速；(f) `CIRCULAR` 模式在同一輸入下輸出與修改前**逐值相同**（回歸鎖）。
- `OmniCenterlineTest`：直線四個方向＋兩個斜向的 tangent／offset／lookahead；消歧選擇；橫切回 null；度量曲率對合成圓弧的誤差 < 5%。
- `FixedHeadingLapControllerTest`：合成圓弧序列下 ψ_v 單調累積到 360°、`LapTimer` 觸發；失偵衰減與停止；ACQUIRING 門檻。
- `HorizontalObstacleFilterTest`：扇區切分與 `zeroIndexAzimuth = null` 的退回。
- `VirtualStickSession` 夾限：手動仍 0.5／20，自主 0.8／60。

### 6.2 Capture replay（instrumented，必須）

- 既有 `PATH` 模式 replay 全部維持通過。
- 新增：把既有 capture 影格**旋轉 90°／180°／270°** 合成 OMNI 測資，驗證 OMNI detector 在四個方向都給出 `FULL_PATH` 且 tangent 正確。這是 B 在週一前唯一能離線驗證「全方位」的方法。

### 6.3 真機（週一，見第 7 節）

---

## 7. 實作順序與 commit 切分

1. **Phase 0**（一個 PR）：2.1 夾限分離 → 2.2 遙測 → 2.3 LapTimer → 2.4 log → 2.5 階梯 → 2.6 扇區（含 null 退回）→ 2.7 量測按鈕。此 PR 不改任何循跡行為，只加量測與基礎。
2. **方案 C**（一個 PR）：3.x 全部 + 按鈕 + 測試。
3. **方案 B**（一個 PR，可與 C 平行）：4.2 detector OMNI → 4.3 幾何 → 4.4 控制器 → 4.5 整合 → 6.2 旋轉 replay。
4. 每個 PR：`testDebugUnitTest` + `lintDebug` + `assembleDebug` + `connectedDebugAndroidTest`（有裝置時）通過；PR 說明列出「動了哪些與舊模式共用的程式碼、如何證明舊模式輸出不變」。

**週一現場順序**：Stage 0（量測按鈕）→ 重飛既有「圓形黑膠帶追蹤」取 34 s 對照 → C 在 S20 → 依 yaw 量測決定 C 能走到哪一階 → 若 yaw < 40°/s 或 C 在 S15 以上不穩，切 B 從 S20 開始 → 兩者各自只在「連續 2 圈、零接管、FULL_PATH > 85%、無 stale 停止」時升階。

---

## 8. 不得做的事

1. 修改 `STRAIGHT`／`CIRCULAR`／`CURVED_OUT_AND_BACK` 的任何常數、相位轉換或測試斷言。
2. 把自主夾限套到 `setStick`。
3. 用固定半徑前饋（除非董事長確認只飛這一個圓；即便如此也要作為可開關旗標）。
4. 為了減少停頓放寬 stale 窗到 400 ms 以上、或移除任何停止條件。
5. 在沒有 `gs` 與 `LapTimer` 的 flight log 之前宣稱圈時。
6. 在 `OBSTACLE_INDEX_ZERO_AZIMUTH_DEGREES` 未校準時啟用方向性扇區（必須退回全向 500 mm）。
7. 加入 freshness watchdog（等量測）。

---

## 9. 開放問題（實作中遇到請記錄在 PR，不要自行假設）

1. `OMNI_REFERENCE_Y` 的實際值（需現場以膠帶正上方懸停校正）。
2. `OBSTACLE_INDEX_ZERO_AZIMUTH_DEGREES`（需現場對牆校正）。
3. MSDK 是否接受 > 50°/s yaw（Stage 0 量測）。
4. OMNI 模式下 detector 在膠帶橫向穿過畫面時的 `confidence` 分布（旋轉 replay 可先看一部分）。
5. 董事長對「機頭是否必須沿切線」與「計時起點」的裁定——決定週一先飛 C 還是 B。
