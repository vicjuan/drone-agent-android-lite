# 正式滑冰模式圈速結構性上限：分析與解法交接

## 目的

前一份交接文件（`docs/fixed-heading-smoothness-review-handoff.md`）處理的是「短暫失路時不要頓挫、不要停機」。本文件處理的是下一個問題：**為什麼圈速停在 12.5 秒，而不是目標速度對應的約 9.5 秒**。

結論先講：`FixedHeadingLapController` 內有一個結構性的限速機制。在圓形路徑上，
純追蹤（pure pursuit）幾何必然產生的方向落後與橫向偏移，被 `trackingAuthority`
當成「追蹤誤差」而持續打折速度。目標速度 0.90 m/s 從未真正生效；
可達到的巡航上限約 0.72 m/s，與實飛 12.5 秒圈時吻合。

請閱讀本文件後，修正根因並依「驗證要求」一節完成驗證。

## 適用範圍與邊界

- 只修改 `app/src/main/kotlin/com/durendal/droneagent/lite/FixedHeadingLapController.kt`
  與 `app/src/test/kotlin/com/durendal/droneagent/lite/FixedHeadingLapControllerTest.kt`。
- 不要動 detector、賽車模式、UI。
- `docs/fixed-heading-smoothness-review-handoff.md` 中「必須維持的安全不變量」九條全部繼續成立。
- 不接受用「放寬門檻讓數字變好看」的修法：直接把 `SLOWDOWN_GUIDANCE_ERROR_DEGREES`
  或 `SLOWDOWN_LATERAL_OFFSET_METERS` 調大不是解法，那只是把真實誤差的煞車一起關掉。
  正確方向是把「幾何必然的量」從誤差中扣除，只對「超出預期的殘差」減速。

## 現場參數（推導都以此為準）

- 圓形路徑：直徑 2.5 m → 半徑 R = 1.25 m，周長約 7.85 m。
- 快速模式目標速度 v = 0.90 m/s（`TARGET_SPEED_METERS_PER_SECOND`）。
- 快速模式前視距離 L = 0.35 m（`FAST_LOOKAHEAD_METERS`）。
- 控制 tick 與 detector callback 都約 10 Hz。
- 理想圈時：7.85 / 0.90 ≈ 8.7 s，加上加速段，實務最佳約 9.5–10 s。
- 實測圈時：12.5 s → 平均速度約 0.63 m/s。

## 根因分析：trackingAuthority 把幾何當成誤差

`tick()` 的 TRACKING 分支中：

```kotlin
val trackingAuthority = minOf(
    1.0 - abs(guidanceError) / SLOWDOWN_GUIDANCE_ERROR_DEGREES,   // /35°
    1.0 - abs(measurement.lateralOffsetMeters) / SLOWDOWN_LATERAL_OFFSET_METERS, // /0.25m
).coerceIn(0.0, 1.0)
val authorityLimitedSpeed = confidenceLimitedSpeed * trackingAuthority
```

在圓上，這兩個量都有**不會歸零的穩態值**：

1. **方向落後**。沿半徑 1.25 m 的圓以 0.90 m/s 飛行，虛擬方向必須以
   v/R ≈ 0.72 rad/s ≈ 41°/s 持續旋轉。10 Hz 下每個 tick 之間 desired heading
   前進約 4.1°，而 `guidanceError` 是在轉向動作前量測的，因此每個 tick 讀到的
   誤差穩態約 4.1°。這不是飛歪，是圓本來就在轉。
   → authority ≈ 1 − 4.1/35 ≈ **0.88**。

2. **橫向偏移**。純追蹤沿圓的穩態偏移約 L²/(2R) = 0.35²/2.5 ≈ 0.049 m（向圓內），
   幾何必然，且目前移動中沒有任何積分或直接橫向回授去消除它
   （`correction` 只在 `pathSpeedMetersPerSecond == 0.0` 時作用）。
   → authority ≈ 1 − 0.049/0.25 ≈ **0.80**。

取 min 後穩態 authority ≈ 0.80，巡航上限 ≈ 0.90 × 0.80 ≈ **0.72 m/s**，
對應圈時 ≈ 10.9 s。再加上下述次要損耗，即為實測的 12.5 s。

**關鍵佐證**：`OmniCenterlineMeasurement.curvaturePerMeter` 已經在
`measurementForProjection()` 中算出（`2 * lateralLookahead / lookaheadDistanceSquared`），
`FixedHeadingLapDecision` 也帶著它，但控制器**從頭到尾沒有使用這個曲率**。
路徑的彎曲程度是已知量，卻被當成未知誤差來懲罰。

## 主要解法：曲率前饋 + 殘差減速

分兩件事做，建議都做：

### A1. 虛擬轉向加入曲率前饋

目前轉向是純回授：

```kotlin
val turnDegrees = guidanceError.coerceIn(-maximumTurnDegrees, maximumTurnDegrees)
```

改為前饋 + 回授：

```kotlin
feedforwardTurnRate = v * κ          // rad/s，κ = measurement.curvaturePerMeter
turn = (feedforwardTurnRate * dt) + residualCorrection
```

其中 `residualCorrection` 是扣除前饋後的殘差回授，總轉率仍受
`MAX_VIRTUAL_TURN_RATE_DEGREES_PER_SECOND`（60°/s）限制。
前饋讓 heading 不再每個 tick 落後 4.1°，穩態 `guidanceError` 趨近 0。

κ 來自單幀量測，會有雜訊，必須防禦：

- 對 κ 做限幅：例如 |κ| ≤ 1.2 (1/m)（對應最小半徑約 0.83 m；本場地 κ = 0.8）。
- 對 κ 做低通（時間常數約 0.2–0.3 s），避免單幀雜訊直接進轉向。
- COASTING 期間不得使用前饋（沒有可信量測）；恢復 TRACKING 後由低通重新收斂。
- 70° 追蹤誤差停機檢查維持不變，且必須用「未扣前饋的原始誤差」判斷。

### A2. 減速公式只看殘差

`trackingAuthority` 的兩個輸入改為扣除幾何預期值後的殘差：

- 方向項：`guidanceError` 扣除純追蹤幾何角 κ·L/2（弧度）。本場地為
  0.8 × 0.35 / 2 = 0.14 rad ≈ 8°。若 A1 做得好，殘差自然變小，此項可以簡化，
  但顯式扣除能讓兩個修正互相獨立、單獨可測。
- 橫向項：扣除純追蹤穩態偏移 κ·L²/2（本場地約 0.049 m），或在 A1 之外
  加入移動中的小增益橫向回授把穩態偏移真正消掉（擇一，不要都做）。

**驗收標準**：合成圓形路徑的閉迴路模擬中，收斂後巡航速度 ≥ 0.85 m/s，
橫向偏移絕對值 ≤ 0.08 m，且直線路徑行為與現狀相同（κ ≈ 0 時前饋自動消失，
這也是回歸保護：直線案例不得有任何行為變化）。

## 次要問題（依影響排序）

### B. 信心值 0.60 是速度懸崖

```kotlin
val confidenceLimitedSpeed =
    if (latestConfidence >= MIN_CONFIDENCE) TARGET_SPEED else DEGRADED_SPEED  // 0.90 vs 0.22
```

信心在 0.6 附近徘徊時，目標速度在 0.90↔0.22 之間大幅擺盪。即使有加速度與
jerk 平滑，平均速度仍被拖低。改為連續映射，例如信心 0.45→0.22、0.75→0.90
線性內插（端點外飽和）。注意 `MIN_CONFIDENCE` 同時用於 recovery candidate
的接受門檻（`recoveryMeasurement()`），**那個用途必須維持 0.60 不變**——
把兩個用途拆成不同常數。

### C. 減速快、回速慢的單向棘輪

`updatePathSpeed()` 中 `desiredAcceleration` 的範圍是
[−1.20, +0.60]，且回升還要受 jerk 2.0 的斜坡。authority 是逐幀從有雜訊的
量測算出來的：任何一次向下抖動立刻扣速度，恢復卻要慢慢爬。
建議對 `targetSpeed`（或 authority）做短時間常數（約 0.3 s）的低通，
讓單幀雜訊不進速度迴路；持續性的真實減速原因（真的偏了、信心真的掉了）
仍會在幾個 tick 內生效。緊急煞車路徑（`emergencyBrake`）不得被低通延遲。

### D. 沉默門檻與影格週期只差 11 ms

0.90 m/s 時 `staleWindowNanos()` = 0.10/0.90 ≈ 111 ms，而 detector 週期約
97–100 ms。影格只要晚到十幾毫秒（並非漏判），tick 就會進入 COASTING 減速
0.15 m/s，再受 jerk 限制慢慢爬回——每次都是純損耗的圈時，也正是原本要消除
的頓挫。這是設計取捨，不是單純 bug：把窗加大等於把盲行距離契約從 0.10 m
放寬。**這一項需要操作者決策**，請在文件或 PR 中明確提出兩個選項讓使用者選：

1. 顯式放寬 `MAX_BLIND_DISTANCE_METERS`（例如 0.18 m → 窗 200 ms），
   並更新安全不變量第 6 條的文字；或
2. 量測實際影格間隔（滾動中位數），沉默減速改在
   「預期下一幀時間 + 安全餘裕」才開始，距離上限另行硬性檢查。

不要在未經使用者同意下自行放寬盲行距離。

## 必須先落地的正確性修正（與圈速無直接關係，但已確認）

1. **`MAX_BLIND_DECELERATION` 一個常數兩種用途**：4.5→1.5 的調整同時弱化了
   `updatePathSpeed(emergencyBrake = true)`——追蹤中方向誤差達 35° 或橫向偏移
   達 0.25 m 時的煞停，距離從 0.09 m 變 0.27 m。拆成兩個常數：
   盲行減速維持 1.5，追蹤中緊急煞車另立常數（建議恢復 4.5，或明文說明理由）。
2. **進入 COASTING 時 `lastVirtualTurnRateDegreesPerSecond` 未歸零**：
   COASTING 期間 heading 凍結，但恢復幀的 `capturedAtNanos` 補償仍用失路前的
   轉率回推，最壞 0.2 s × 60°/s = 12° 的方向誤估，直接影響 35° 恢復門檻的判斷。
   進入 COASTING 時歸零即可。
3. **測試缺口**：現有 20 個測試沒有涵蓋
   （a）完全沉默——追蹤中之後只呼叫 `tick()`、完全不呼叫 `observe()`，
   應立即受控減速（無保持窗）並在 1.2 s 停止；
   （b）連續不安全 recovery candidates 持續到 1.2 s，仍必須停止；
   （c）跨 ±180° 方向接縫的失路與恢復（現有測試 heading 都在 0°/90° 附近）。

## 建議的執行順序

1. 先落地上節三項正確性修正與缺口測試（行為風險最低，先建立安全網）。
2. 實作 A1 + A2（曲率前饋與殘差減速），附閉迴路圓形模擬測試。
3. 實作 B（信心連續映射）與 C（authority 低通）。
4. D 整理成選項，交由操作者決策後才實作。

每一步都保持既有 20 個測試通過；A1/A2 若造成既有測試失敗，先確認是測試斷言
依賴了「幾何誤差被當成誤差」的舊行為，才可以更新斷言，並在 commit message 說明。

## 驗證要求

- 本機 Gradle 8.7 與預設 JDK 25 不相容，必須指定 JDK 17：

```bash
JAVA_HOME=$(/usr/libexec/java_home -v 17) ./gradlew :app:testDebugUnitTest :app:lintDebug
```

- 新增「合成圓形路徑閉迴路模擬」測試：以 R = 1.25 m 的圓生成逐幀
  centerline（可參考現有 `curvedPath()` 的產生方式），10 Hz 交錯呼叫
  `observe()` / `tick()`，斷言收斂後速度 ≥ 0.85 m/s、橫向偏移 ≤ 0.08 m、
  全程 `stopRequested == false`。
- 直線路徑回歸：κ ≈ 0 時所有輸出與修改前一致（前饋必須自動退化為零）。
- 實飛預期：圈時應由 12.5 s 降至約 10 s 上下。實飛時記錄
  `guidanceError`、`trackingAuthority`、`targetSpeed` 逐 tick 數值，
  確認穩態 authority 由 ~0.80 升到 ~0.95 以上；若沒有，表示前饋未生效，
  回頭查 κ 的量測品質，不要改用放寬門檻補救。
- 實飛證據照舊誠實記錄：單圈成績不得自行擴張為「長時間穩定」。

## 已知風險與不要做的事

- κ 前饋放大了對單幀量測的信任：限幅與低通不是可選項。
- 不要為了讓模擬測試達標而調大 `MAX_VIRTUAL_TURN_RATE_DEGREES_PER_SECOND`；
  60°/s 對穩態 41°/s 已有餘裕，若不夠，先確認是前饋收斂問題。
- 不要移除或弱化 70° 停機、1.2 s 停機、雙幀 recovery 確認——那是前一輪
  修正的安全底線。
- `DEGRADED_SPEED_METERS_PER_SECOND` 的下限語意（信心很低時的保守速度）
  必須保留，B 項只是把階梯換成斜坡。
