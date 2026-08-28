# 正式滑冰模式流暢性修正：AI Agent 審查交接

## 目的

請審查目前尚未提交的正式滑冰模式控制修正，重點確認：

1. 單幀鏡面反光造成短暫失路時，飛機是否能維持流暢，而不會立刻急煞。
2. 短暫容錯是否仍有明確距離與時間上限，不會演變成盲飛。
3. 路徑重新出現時，控制器是否能恢復追蹤，又不會被反光造成的跳變路徑帶往錯誤方向。
4. 加減速、轉向與狀態轉移是否仍符合既有安全契約。

本文件描述的是 `main` 上兩個**尚未提交**檔案的目前工作樹，不要把它誤認為提交 `94e17c6` 已包含的內容。

## 工作樹狀態

目前 `main` 與 `origin/main` 指向同一個提交；另有兩個 tracked files 尚未提交：

- `app/src/main/kotlin/com/durendal/droneagent/lite/FixedHeadingLapController.kt`
- `app/src/test/kotlin/com/durendal/droneagent/lite/FixedHeadingLapControllerTest.kt`

目前差異統計：

```text
2 files changed, 254 insertions(+), 29 deletions(-)
```

不要在審查過程中順手改 detector、賽車模式或 UI。這次範圍只有正式滑冰模式在短暫失路時的控制連續性與安全恢復。

## 模式定義

正式滑冰模式在程式中是：

- `TapeTrackingMode.FIXED_HEADING`
- 主要控制器：`FixedHeadingLapController`

飛機機頭維持啟動時的固定方向。控制器沿辨識出的中心線計算地面行進方向，再將速度投影為 DJI body-frame 的：

- `forwardMetersPerSecond`
- `rightMetersPerSecond`

因此這不是讓機頭沿圓周旋轉的賽車模式。此處的 `virtualHeadingDegrees` 是地面速度向量的方向，不是無人機實際 yaw。

## 問題證據

十圈壓力測試在完成兩個診斷圈後停止。逐幀對齊自動保存的 capture 與控制紀錄後，關鍵序列是：

```text
#2521：正常辨識，FULL_PATH，TRACKING
#2522：鏡面反光造成單幀中心線漏判，進入 COASTING
#2523：下一張影格重新辨識到路徑
```

失路到重新辨識約 97 ms。膠帶並未真的離開畫面；問題是鏡面高光讓一張影格無法產生可信中心線。

舊行為在短暫漏判後太快減速，造成影片中可見的頓挫。若圓形路徑每圈都經過同一個反光位置，這種單幀掉速也會增加後續完全停機的機率。

需要同時避免兩種錯誤：

- **太敏感**：一張約 97 ms 的漏判就急煞。
- **太寬鬆**：路徑真的消失後仍長時間沿舊方向盲飛。

## 昨晚的控制修改

### 1. 明確失路後先保留上一個可信命令

新增狀態：

```kotlin
private var lossObservedAtNanos = 0L
```

`observe()` 收到 `centerline == null` 或無法形成有效量測時：

- `TRACKING → COASTING`
- 記錄明確失路影格的到達時間。
- 將加速度狀態歸零。
- 不立即把路徑速度歸零。

`tick()` 會依目前速度計算保持窗：

```kotlin
hold = MAX_BLIND_DISTANCE_METERS / currentSpeed
hold ∈ [80 ms, 150 ms]
MAX_BLIND_DISTANCE_METERS = 0.10 m
```

保持窗內沿用上一個可信速度與地面方向，不減速、不停止，也不接受未確認的跳變路徑。

這個設計的核心限制是**距離**，不是任意等待固定秒數：速度越高，允許等待的時間越短；最壞情況仍限制在約 10 cm。

### 2. 保持窗結束後改為受控減速

超過短暫保持窗仍未恢復時，維持 `COASTING`，但開始將速度往零收斂：

```kotlin
MAX_BLIND_DECELERATION_METERS_PER_SECOND_SQUARED = 1.50
```

舊值為 `4.50 m/s²`，反光漏判後的減速過於突然。新值降低為 `1.50 m/s²`，目的是讓真正的短暫失路不產生明顯急煞，同時仍比一般舒適減速更積極。

持續失路達下列上限仍會停止：

```kotlin
REACQUISITION_TIMEOUT_NANOS = 1_200_000_000L // 1.2 s
```

停止時速度與加速度歸零，並回傳 `stopRequested = true`。

### 3. 相機或 detector 完全停止回報時不額外等待

短暫保持只適用於 `observe()` 明確收到一張失路結果的情況。

若相機或 detector 完全沒有再呼叫 `observe()`，控制器仍依 `lastDetectionAtNanos` 判斷有效量測年齡；超過速度對應的保持窗後直接進入受控減速，不會再從「發現資料流沉默的時間」重新多等一輪。

這個差異很重要：

- 一張反光漏判：容許跨過單幀。
- 資料流停止：依最後有效影格時間開始減速。

### 4. 跳變路徑必須連續兩次一致才可重建基準

新增狀態：

```kotlin
private var pendingRecoveryMeasurement: OmniCenterlineMeasurement? = null
```

當 `OmniCenterline.measure(... previousMeasurement = latestMeasurement)` 因為路徑跳變而拒絕候選時，不再立即把新候選當作真實路徑。`recoveryMeasurement()` 要求：

1. 信心值至少為 `MIN_CONFIDENCE = 0.60`。
2. 第一個恢復候選只暫存，不輸出控制命令。
3. 第二個候選必須能與第一個候選形成連續量測。
4. 目標方向相對既有行進方向的誤差不超過 `35°`。
5. 橫向偏移不超過 `0.20 m`。

只有兩個候選都通過才更新 `latestMeasurement` 並回到 `TRACKING`。

目的：鏡面高光可能把同一條膠帶重畫成幾何位置差很多的候選；一張孤立影格可以讓飛機暫停接受新方向，但不能把飛機導向錯誤路徑。

### 5. 量測時間補償

`observe()` 使用 `capturedAtNanos` 回推影格擷取當下的虛擬行進方向：

```kotlin
virtualHeadingAtCapture =
    virtualHeadingDegrees - lastVirtualTurnRateDegreesPerSecond * measurementAgeSeconds
```

補償最多 `0.20 s`。這避免用「現在的行進方向」解讀較早擷取的影格，降低轉彎時因處理延遲造成的假方向跳變。

### 6. 流暢性相關常數

目前正式滑冰快速模式的主要常數：

```text
目標速度                         0.90 m/s
一般最大加速度                   0.60 m/s²
一般最大減速度                   1.20 m/s²
失路後最大減速度                 1.50 m/s²
最大 jerk                        2.00 m/s³
虛擬行進方向最大轉率             60°/s
最大盲行距離                     0.10 m
短暫保持窗                       80–150 ms（依速度換算）
重新取得逾時                     1.20 s
恢復方向誤差上限                 35°
恢復橫向偏移上限                 0.20 m
快速模式前視距離                 0.35 m
```

一般信心下降、前視縮短或偏差增加時，速度仍經 `updatePathSpeed()` 的加速度與 jerk 限制平滑調整，不應直接跳到較低速度。

## 必須維持的安全不變量

審查時請確認下列條件沒有被短暫保持邏輯破壞：

1. 從未取得過有效路徑時，不得輸出水平速度。
2. 初始取得超過 8 秒仍失敗，必須停止。
3. 有效路徑持續消失超過 1.2 秒，必須停止。
4. 追蹤方向誤差超過 `70°`，必須停止，不得用保持窗掩蓋。
5. 跳變候選未連續確認前，不得改變飛行方向。
6. 保持窗內最多沿上一個可信方向前進 10 cm。
7. 相機或 detector 沉默時，不得重新起算另一個保持窗。
8. `start()` 與 `stop()` 必須清除所有恢復候選、失路時間、速度與加速度狀態。
9. `stopRequested = true` 時輸出速度必須為零。

## 新增與更新的回歸測試

`FixedHeadingLapControllerTest` 目前共 20 個測試。與本次修改直接相關的測試包括：

### `fixed heading smooths a confidence limited speed reduction`

信心值由正常降到較低時，速度必須下降，但單一步進降幅仍受 jerk 限制；不得直接跳到 degraded speed。

### `fixed heading holds speed for one reflection frame before braking`

- 明確失路後的第一個保持窗內，速度完全不變。
- 超過保持窗後才以 `1.50 m/s²` 減速。
- 失路 0.5 秒時仍在 `COASTING`，尚未要求停止。
- 超過 1.2 秒後要求停止。

### `fixed heading crosses one dropped reflection frame without slowing`

測試時序刻意貼近 capture：

```text
失路影格：前一有效影格後 84 ms
恢復影格：再過 97 ms
```

契約：保持窗內速度不變；下一個有效影格到達後恢復 `TRACKING`，且 `stopRequested == false`。

### `fixed heading resumes after a half second reflection gap`

半秒失路後仍在安全等待／減速階段；可信路徑重新出現後可恢復 `TRACKING`，不需重新啟動整個模式。

### `fixed heading rebases only after two consistent recovery measurements`

第一個安全但與舊路徑不連續的候選只能進入待確認狀態；第二個互相連續的候選到達後才可恢復追蹤。

## 已執行驗證

昨晚在這份工作樹上執行：

```bash
./gradlew :app:testDebugUnitTest :app:connectedDebugAndroidTest :app:lintDebug
```

結果：

```text
BUILD SUCCESSFUL in 1m 42s
77 actionable tasks
```

最終 XML 證據：

- JVM tests：200／200，0 failures，0 errors。
- Pixel 8 Pro instrumented tests：57／57，0 failures，0 errors。

另外已執行：

```bash
./gradlew :app:assembleDebug :app:installDebug
adb -s 39121FDJG009Y6 shell am start -W \
  -n com.durendal.droneagent.app/com.durendal.droneagent.lite.MainActivity
```

APK 安裝成功，`MainActivity` 回報 `Status: ok`。

## 實飛證據邊界

- 修正前，操作者確認最後兩個完整圈平均約 11.5 秒，但後續十圈壓力測試在單幀反光後停止。
- 2026-08-27 操作者回報正式滑冰模式完成 12.5 秒一圈。
- 12.5 秒是目前的實飛圈時成果；除非另有完整圈數與不中斷紀錄，不要自行擴張成「已連續完成十圈」或「已證明長時間穩定」。

## 請下一位 AI Agent 特別審查

1. `lossObservedAtNanos` 與 `lastDetectionAtNanos` 的雙時間來源，是否在所有 `observe()`／`tick()` 排序下都不會多給一個保持窗。
2. 在 `COASTING` 期間收到不安全 recovery candidate 時，是否可能無限延後 1.2 秒停止條件。
3. 第二個 recovery candidate 成功後，速度與 acceleration state 是否可能造成恢復瞬間加速突跳。
4. 80–150 ms 的保持窗在 10 Hz 控制 tick 與實際約 10 Hz detector callback 下，是否存在 off-by-one tick。
5. `capturedAtNanos` 補償與 `lastVirtualTurnRateDegreesPerSecond` 在 `COASTING`／恢復後是否仍代表正確時間區間。
6. `MAX_BLIND_DECELERATION_METERS_PER_SECOND_SQUARED = 1.50` 是否在最壞速度與 1.2 秒停止上限下維持可接受的額外位移。
7. 現有測試是否缺少「資料流完全沉默」、「連續不安全 recovery candidates」或「跨 ±180° 恢復」等會推翻安全契約的案例。

如果發現問題，應修正根因並維持上述不變量；不要只延長 timeout、增加盲行距離或放寬方向／偏移門檻來讓測試通過。
