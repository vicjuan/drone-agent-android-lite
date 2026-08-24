# 10 秒一圈：可行性分析與實作方案

回應 `docs/ten-second-lap-problem-statement.md`。基線：`main` = `e382d74`（PR #3 已合併，尚未重飛）。

---

## 0. 一句話結論

**10 秒一圈在物理上可行，但不是「調速度」問題，而是「yaw 權限」問題。** 只要機頭沿切線飛，一圈就是機頭轉 360°，10 秒即 **平均 36°/s**——與圓圈半徑無關。目前 App 夾限 20°/s，從量級上不可能。整個方案的第一道關卡是**在懸停中量測 Mini 4 Pro 經 MSDK 實際能接受與執行的 yaw rate**；這個數字量出來之前，任何圈速承諾都沒有根據。

其餘工程（速度規劃、crab 補償、stale 窗、方向性避障、lap 計時）都是讓 36°/s 能**安全且可重複**地用出來。

---

## 1. 可行性判斷

### 1.1 由程式碼與幾何可直接得出的結論

| 項目 | 結論 | 依據 |
|---|---|---|
| 平均 yaw 需求 | 36°/s（10 s）、30°/s（12 s）、24°/s（15 s）、18°/s（20 s） | 360° ÷ 圈時；與半徑無關 |
| 目前 yaw 夾限 | 20°/s → 最快約 18 s 一圈（含零餘裕） | `VirtualStickSession.MAX_YAW_DEGREES_PER_SECOND` |
| 平均前進速度需求 | 周長 ÷ 10 s。R=0.75 m → 0.47 m/s；R=1.0 m → 0.63 m/s；R=1.25 m → 0.79 m/s | 半徑未知，需現場量 |
| 目前水平夾限 | 0.5 m/s → R ≥ 0.8 m 時不夠 | `MAX_HORIZONTAL_MPS` |
| 向心加速度 | R=0.75、v=0.47 → 0.30 m/s²；R=1.0、v=0.63 → 0.39 m/s²（傾角 < 2.5°） | v²/R；機體完全無壓力 |
| 加速時間 | 0→0.6 m/s 在 0.10 m/s² 需 6 s；在 0.4 m/s² 需 1.5 s（行進 0.45 m） | 現行 `MAX_FORWARD_ACCELERATION = 0.10` 必須提高 |
| 地面足跡（h=1.2 m） | 約 1.60 m 寬 × 0.90 m 高；640 px 分析寬度 → 2.5 mm/px；48 mm 膠帶 ≈ 19 px | 75° 對角 FOV、16:9 |
| 每幀位移 | 0.6 m/s @ 10 Hz → 6 cm/幀；切線每幀轉 3.6° | — |
| Motion blur | 0.6 m/s：1/60 s 曝光 → 10 mm（4 px）、1/30 s → 20 mm（8 px，接近膠帶半寬） | 需快門 ≥ 1/120 或提高 ISO |
| 度量曲率 | **已存在**：`desiredPurePursuitCurvaturePerMeter()` 以高度 + FOV 投影，κ = 2x_L/L²；對圓弧此公式為精確解 | `TapeTrackingController.kt:1060-1071` |
| 實際地速 | **完全沒有**：未訂閱 `KeyAircraftVelocity`；現有「0.183 m/s」只是命令積分 | `MainActivity.kt` 只訂閱 Attitude/Altitude |

### 1.2 必須真機量測才能回答

1. MSDK 接受的最大 yaw rate、實際達到的角速度、上升時間、overshoot（懸停階躍測試）。
2. 速度階躍的追蹤延遲與 overshoot（0.3／0.5／0.7 m/s 短脈衝）。
3. 端到端延遲：camera callback → detector 完成 → 指令送出 → 機體開始響應。
4. 障礙 callback cadence（`callbacks=N maxGapMs=M` 已埋好）、避障關閉時的實際行為、障礙環 index-0 的方位。
5. 路徑中心線半徑／周長、紙板淨空、牆面距離。
6. 高速下 detector 的信心、FULL_PATH 比例、lookahead 跳變幅度、曝光自動調整後的 blur。

### 1.3 最關鍵的限制（依阻擋程度）

1. **yaw 權限**（硬需求 ≥ 36°/s 平均，建議可用上限 50°/s）。若 MSDK/機體在室內低空做不到 ≥ 40°/s 穩定 yaw，10 秒只能靠「固定機頭 crab」架構（方案 B），那不是下週一能完成的。
2. **端到端延遲 × 速度 = 盲飛距離與停止距離**。0.6 m/s 下每 100 ms 延遲就是 6 cm；這決定 stale 窗、前向淨空與是否還能在會議室飛。
3. **半徑未知**：決定需要的 v 是否超過 0.5 m/s、是否需要放寬水平夾限與 FOV 是否夠用。

---

## 2. 控制方案

### 方案 A：切線對準 + 放寬 yaw（最小改動）

- 架構：維持現行「機頭沿切線、body-forward 前進」。ω = v·κ（PP 前饋）+ K_p·θ（角度回授），v 受 ω_max/|κ| 限制。
- 改動：提高自主 yaw／速度夾限、提高加速度、其餘不動。
- 優點：改動最小，測試基礎可沿用。
- 缺點：**全部押在 yaw 上**。yaw 一飽和、延遲一大，θ 就發散，路徑出畫面；現行 `CIRCULAR_MAX_CENTERING_SPEED = 0.02 m/s` 沒有任何側向救援能力。10 s 時任何 >10% 的 yaw 飽和都會導致失敗。

### 方案 B：固定機頭、純速度向量（crab）循跡

- 架構：yaw ≈ 0，body-frame 速度向量 = v·(sin θ, cos θ)（右, 前），θ 為影像中切線與機頭夾角，涵蓋全 360°。
- 優點：**徹底移除 yaw 瓶頸**；畫面不旋轉，blur 更低；Mini 4 Pro 側飛無物理問題。
- 缺點：detector／控制器目前假設「影像上方 = 前進、近場錨點在下緣」；切線方向在全 360° 下有 180° 歧義（往哪邊繞），需要新的路徑方向記憶與 lookahead 語意；`TapeTrackingController` 的 phase／品質契約要大改；且「機頭沿切線」可能是董事長期待的視覺效果。**不是下週一可交付的工程量**，但它是唯一把 yaw 移出關鍵路徑的架構——若 1.2 項量出 yaw 不足，它是 10 秒的唯一出路。

### 方案 C（推薦）：切線 yaw 前饋＋回授，加小角度 crab 補償與聯合速度規劃

在方案 A 的骨架上，把「θ 偏差」同時用在兩條路：yaw 回授去消除它，body 速度向量旋轉去**容忍**它。yaw 仍需平均 36°/s（crab 吸收的是暫態與延遲，不是平均值），但系統不再「yaw 一落後就出軌」。

---

## 3. 推薦方案 C 的具體設計

### 3.1 符號與輸入

每個控制 tick（100 ms）從現有控制量取得：

- θ：`controlledAngleDegrees`（切線相對影像垂直軸；影像右偏為正，對應 DJI 正 yaw／正 right，已由真機證據確認）
- e_x：`controlledHorizontalOffsetFraction`（近場橫向偏移，右正）
- κ：`desiredPurePursuitCurvaturePerMeter()`（/m，右彎為正）
- h：`heightAboveGroundMeters`
- q：`pathQuality`

### 3.2 速度規劃（取各限制的最小值）

```text
v_profile   = 階梯目標（0.32 → 0.42 → 0.52 → 0.62 m/s，見第 5 節）
v_yaw       = ω_budget / |κ|                         // 已有 limitForwardSpeedForYaw
v_lateral   = sqrt(a_lat_max / |κ|)                   // 新：側向加速度上限，a_lat_max ≈ 0.5 m/s²
v_vision    = q == FULL_PATH 且 confidence ≥ c_min ? ∞ : v_degraded   // 新：品質降級立即減速
v_target    = min(v_profile, v_yaw, v_lateral, v_vision)
v_cmd       = 加速度限制（上升 ≤ a_up = 0.40 m/s²；下降立即）   // 改：MAX_FORWARD_ACCELERATION 0.10 → 0.40
```

### 3.3 yaw：前饋 + 回授（不是二選一）

現行 `desiredYawRate()` 是 `purePursuitYawRate ?: angleFeedback`。改為：

```text
ω_cmd = clamp( v_cmd·κ  +  K_p·θ  +  K_d·dθ/dt ,  ±ω_auto_max )
```

- 前饋 v·κ 負責平均 36°/s；回授只收拾延遲與估計誤差，增益可以比現在小（K_p 0.70 → 0.4 起調）。
- `ω_auto_max` 是**新的自主夾限**（50°/s 為目標，由 Stage 0 量測決定），**手動搖桿的 20°/s 不動**。

### 3.4 crab 補償：把 θ 分配到 body 速度向量

```text
v_forward = v_cmd · cos θ
v_right   = v_cmd · sin θ  +  K_lat · deadzone(e_x)        // K_lat 維持小，最大 ±0.08 m/s
```

- θ 在 ±35° 內時，機體沿**切線**前進而不是沿機頭前進，yaw 落後不再直接變成橫向誤差。
- |θ| > 35°（持續 > 300 ms）→ 降速到 v_degraded；|θ| > 50° → 進入既有 `ALIGNING_CURVE`（原地對正），維持「不可接受路徑就不前進」的既有安全語意。

### 3.5 stale 窗：用距離而非固定時間

```text
t_stale = clamp( d_blind_max / v_cmd , 150 ms , 400 ms )     // d_blind_max = 0.08 m
gap 期間 v_cmd 以 a_down 衰減，不再維持或加速
```

0.6 m/s 下約 130 ms（取下限 150 ms）→ 盲飛 ≤ 9 cm；低速時仍回到 400 ms，與現行行為一致。不是只靠時間：`confidence`、曲率連續性、offset rate 任一異常也提早結束 coasting。

### 3.6 方向性障礙

`data.horizontalObstacleDistance` 有 `horizontalAngleInterval`，index → 方位可算。以**指令速度向量方向**為前方：

```text
d_front_min = 0.30 + v_cmd·t_latency(量測, 預設 0.6 s) + v_cmd²/(2·a_brake, 預設 1.0)   // 0.6 m/s ≈ 0.85 m
前扇區 ±45°：nearest ≤ d_front_min → 停
側扇區：≤ 0.35 m → 停
後扇區：≤ 0.30 m → 停（純保底）
```

**前提**：index-0 方位必須現場量（機頭對牆、看哪個 index 最小）；cadence 必須由 `callbacks/maxGapMs` 量到，freshness watchdog 才決定是否加入（延續 PR #3 的結論）。

### 3.7 lap 計時與驗收訊號

重用 `HeadingTurn`（累計有向機頭位移，已有跨 ±180° 處理）：`targetDegrees = 360`，從「穩定 TRACKING ≥ 1 s 且 v_cmd ≥ 0.8·v_profile」起算，累計 360° 即一圈，log 寫 `lap n time=… meanSpeed=… yawSatFraction=…`。因為機頭沿切線，這與「繞圓一圈」等價，且可由 flight log 回查。可選：在膠帶起點貼一段白色標記作為影像交叉驗證（非必要）。

### 3.8 必須保留的既有行為

detector 與色度遲滯、路徑品質契約、`ALIGNING_CURVE`／重取／端點狀態機、負高度安全停止、authority 接管／釋放、低配置緩衝、TapeCapture、手動搖桿的 0.5 m/s／20°/s／150 mm 語意。

---

## 4. 程式碼修改地圖

| 檔案 | 修改 | 保持 |
|---|---|---|
| `VirtualStickSession.kt` | 新增 `AUTONOMOUS_MAX_HORIZONTAL_MPS`（0.8）與 `AUTONOMOUS_MAX_YAW_DEGREES_PER_SECOND`（Stage 0 量測後設定，上限 60）；`setHorizontalVelocity`／`setYawRate` 改用自主夾限；`setStick` 維持 `MAX_*` 手動夾限 | 20 Hz 發送、frame summary、authority 流程 |
| `TapeTrackingController.kt` | (1) `desiredYawRate`：前饋+回授；(2) 新 `planForwardSpeed()` 取代 `desiredForwardSpeed`+`limitForwardSpeedForYaw` 串接，加入 `v_lateral`、`v_vision`；(3) 圓形分支 `desiredRightSpeed` 改為 crab + 小 centering；(4) `MAX_FORWARD_ACCELERATION` 0.10→0.40（僅圓形）；(5) 動態 stale 窗 `staleWindowNanos(v)` 取代常數 400 ms 的圓形用法；(6) `TapeTrackingDecision` 新增 `plannedSpeedLimitReason`（profile/yaw/lateral/vision/stale）供 log | 直線模式所有常數與路徑、狀態機、測試的 sign 慣例 |
| `MainActivity.kt` | 訂閱 `KeyAircraftVelocity`（NED→地速）；由 attitude 差分算實際 yaw rate；新 `LapTimer`（包 `HeadingTurn(…, 360.0)`）；每 tick 一行同步 log（3.7／第 7 節 schema）；`tapeTrackingStopReason` 改呼叫方向性扇區判定 | 既有 stop 原因順序、手動路徑 |
| `HorizontalObstacleFilter.kt` | 新增 `sectorSummary(distances, intervalDeg, headingSectorCenters)` 回傳前／側／後最近值 | `summarize()` 與地板抑制 |
| `ObstacleRanges.kt` | 新常數：前向停止距離公式參數、側／後門檻 | 手動 150 mm |
| `BlackTapeDetector.kt` | 只加量測：`detectMs` p50/p95 累計、lookahead 跳變量（幀間 Δx_L） | 演算法不動 |
| 相機 | 嘗試固定快門 ≥ 1/120（`KeyShutterSpeed`／曝光模式），作為**實驗旗標**可回退 | — |
| 測試 | JVM：速度規劃各限制的最小值選擇、crab 分配的 sign、stale 窗隨速度縮短、LapTimer 360° 跨界；替換現有「forward == 0.24」類斷言為「≤ 規劃上限」 | 既有 sign 測試 |

---

## 5. 實驗階梯（每階的前提、設定、通過條件）

**Stage 0 — 懸停／地面量測（不循跡，必做，約 30 分鐘）**

1. 障礙 log 10 s：讀 `callbacks` 與 `maxGapMs`；機頭對牆確定 index-0 方位。
2. yaw 階躍：懸停，依序命令 20 → 30 → 40 → 50 → 60°/s，每段 2 s；記錄 attitude 差分的實際 yaw rate、達到 90% 的時間、停止後 overshoot。**若 50°/s 段實際 < 40°/s 或 overshoot > 15°，10 秒目標今天停在「不可行」。**
3. 速度階躍（需 ≥ 3 m 直線淨空）：0.3／0.5／0.7 m/s 各 1.5 s，記錄 `KeyAircraftVelocity`；得出追蹤延遲與停止距離。
4. 影像：懸停於膠帶上方，記錄 10 s 的 `detectMs` p50/p95、`acceptedHz`、callback→指令延遲。

**Stage 1 — 重飛 `main`（34 s 基線）**：常數不變。通過：≥ 2 圈、零接管、無 stale 停止、`lap time` 由 LapTimer 記錄而非目視。這是所有後續階段的對照組。

| Stage | 目標圈時 | v_profile | ω_auto_max | a_up | 通過條件（全部成立才可升階） |
|---|---|---|---|---|---|
| 2 | 20 s | 0.32 | 30 | 0.25 | 連續 2 圈；yaw 飽和時間比 < 20%；\|e_x\| p95 < 0.20；FULL_PATH 比例 > 85%；零接管 |
| 3 | 15 s | 0.42 | 40 | 0.30 | 同上，且 \|θ\| p95 < 25°；stale 事件 = 0 |
| 4 | 12 s | 0.52 | 45 | 0.35 | 同上；前向淨空條件 ≥ 0.8 m 已確認 |
| 5 | 10 s | 0.62（或周長/10） | 50 | 0.40 | 同上；並取得 3 次中 ≥ 2 次 ≤ 10.0 s |

**不得升階的情況**：任何一階出現操作者接管、`ALIGNING_CURVE` 在穩定圈中觸發、obstacle 停止、detector FULL_PATH 比例 < 80%、或 Stage 0 量到的 yaw 上限低於下一階的 ω_auto_max + 20% 餘裕。

---

## 6. 安全與 abort 條件（全部立即歸零水平與 yaw，並依既有流程釋放或保留控制權）

| 條件 | 判定 | 動作 |
|---|---|---|
| 路徑偏差 | \|e_x\| > 0.35 持續 300 ms 或 \|θ\| > 50° | 降速→對正；再 1 s 未恢復 → 停止 |
| 失偵 | 超過 `t_stale(v)`；或 confidence 連續 3 幀 < c_min | 停止（hover） |
| 障礙 | 前扇區 ≤ d_front_min；側 ≤ 0.35；後 ≤ 0.30；callback 中斷（若 cadence 已量測確認） | 停止 |
| 遙測失效 | 高度或姿態 > 1 s 未更新；`KeyAircraftVelocity` 中斷 > 1 s | 停止 |
| VirtualStick | 任何 frame 送出失敗、authority 非 MSDK | 既有接管流程 |
| 指令異常 | 命令 yaw 飽和 > 1 s 連續；命令速度與實測地速差 > 0.25 m/s 持續 1 s | 降一階速度並記錄 |
| 操作者 | 任一搖桿偏移、實體遙控器 | 既有接管 |

---

## 7. 量測與 log 設計

每控制 tick 一行（100 ms），同一 `System.nanoTime()` 時戳，欄位固定順序：

```text
lap t=<ns> phase=<…> q=<FULL/NEAR/LOST> conf=<…>
  theta=<deg> ex=<frac> kappa=<1/m> lookahead=(<x>,<y>) dxL=<frac>
  vPlan=<m/s> limit=<profile|yaw|lateral|vision|stale> vFwdCmd=<…> vRightCmd=<…> yawCmd=<deg/s>
  gs=<m/s 由 KeyAircraftVelocity> yawRate=<deg/s 由 attitude 差分> heading=<deg> h=<m>
  detMs=<…> obsAgeMs=<…> front=<mm> side=<mm> rear=<mm>
  lap=<n> lapElapsed=<s> lapProgress=<deg>
```

另每秒一行摘要：yaw 飽和比例、FULL_PATH 比例、stale 事件數、`acceptedHz`、`callbacks/maxGapMs`。Stage 0 的階躍測試用獨立前綴 `step yaw=…`／`step vel=…`。

---

## 8. 驗收標準（暫定，需董事長確認的項目以 ★ 標記）

- ★ 計時起點：穩定 TRACKING ≥ 1 s 且速度達目標 80% 後起算（不含起始對正與加速）。
- 一圈定義：LapTimer 累計機頭 360°（等價於沿切線繞圓一圈），由 flight log 回查。
- ★ 容許偏差：全程 |e_x| ≤ 0.35（約 ±0.28 m @ 1.2 m）且不離開紙板；圈時 ≤ 10.0 s。
- ★ 重複性：連續 3 次啟動中 ≥ 2 次達成；每次 ≥ 2 圈不接管。
- 證據：flight log、TapeCapture、螢幕錄影、APK SHA-256、現場幾何量測記錄（半徑、淨空、牆距）。
- ★ 路徑幾何：是否只需支援這一個圓圈（影響是否允許用量到的半徑做前饋 A/B）。

---

## 9. 下週一現場清單

**起飛前（可離線準備的先做）**
- 量膠帶中心線半徑／周長、紙板邊界、四周牆面／家具／人員距離（畫平面圖）。
- 計算該半徑下 10 s 所需 v；若 > 0.5 m/s 確認水平夾限已分離為自主專用。
- 確認 APK SHA-256、log 目錄乾淨、錄影就緒。
- 相機：確認 -90°、試固定快門 1/120 並看曝光是否可接受。

**地面／懸停 actuator 測試（Stage 0，先做，不可跳過）**
- 障礙 callback cadence 與 index-0 方位。
- yaw 階躍 20→60°/s；速度階躍 0.3→0.7 m/s；影像延遲與 detectMs。
- 依量到的 yaw 上限**當場決定最高可挑戰的階**。

**飛行順序**：Stage 1（34 s 基線）→ 2 → 3 → 4 → 5，每階至少 2 圈、完成即落地檢視 log 再升階。

**何時不得繼續提速**：第 5 節「不得升階的情況」任一成立；或現場人員距路徑 < 2 m；或電量 < 40%。

---

## 10. 風險排序

**會阻止 10 秒達成**
1. yaw 權限量測結果 < 40°/s（→ 只剩方案 B，非下週一可交付）。
2. 半徑 ≥ 1.0 m 使 v ≥ 0.63 m/s，FOV 前視約 0.6–0.7 m → 前瞻時間 ≈ 1 s，加上延遲後 lookahead 跳變導致 κ 噪訊過大。
3. 高速 blur／曝光使 FULL_PATH 比例跌破 80%，觸發降速或 ALIGNING_CURVE，吃掉 10 秒預算。
4. 起始對正不可重複（34 s 實驗前兩次即因此接管）。
5. 計時定義若包含加速與對正，10 秒幾乎沒有穩態時間。

**可能造成安全事故**
1. 0.6 m/s × 未量測的端到端延遲 → 停止距離 ≈ 0.5 m；會議室前向淨空若 < 1 m 不可飛第 4、5 階。
2. 放寬夾限時誤動手動路徑（必須分離常數並以測試鎖定）。
3. 高 yaw rate 的 overshoot 在室內低空使機身掃過紙板邊界外。
4. stale 期間仍用過期幾何前進（已以距離式 stale 窗與 gap 衰減處理，但依賴正確的 v 估計）。
5. 障礙 cadence 未量測即加入 freshness watchdog（維持 PR #3 結論：先量後加）。

---

## 11. 需要董事長／操作者確認的事項（彙整）

1. 計時起點與是否含加速／對正（★）。
2. 容許路徑偏差與「不碰撞」的具體邊界（★）。
3. 一次成功 vs 3 次中 2 次（★）。
4. 是否只支援這一個固定圓圈（影響前饋與 FOV 設計）（★）。
5. 機頭是否必須沿切線（若否，方案 B 的長期價值大增）。
6. 會議室能否清出 ≥ 1 m 前向淨空與 ≥ 2 m 人員距離。
