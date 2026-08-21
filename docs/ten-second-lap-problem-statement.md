# DJI Mini 4 Pro 黑膠帶循跡：10 秒一圈問題描述

## 任務背景

董事長要求在 **2026-08-24（下週一）** 的實機實驗中，讓 DJI Mini 4 Pro 沿地面黑膠帶圓形路徑，在約 1.2 m 飛行高度下完成 **10 秒一圈**。

本文件的用途是把問題、現有證據、限制、未知數與可能風險完整交給另一個 AI agent。請先做可行性與控制設計分析，再提出實作方案；不要把問題簡化成只調高一個速度常數。

## 工作目標

董事長的原始目標是「飛機能夠十秒繞一圈」。在取得更精確驗收定義前，暫採以下工作定義：

1. 從已對準黑膠帶、飛機已穩定懸停的狀態開始計時。
2. 沿同一條黑膠帶圓形路徑完成 360° 一圈，時間不超過 10 秒。
3. 過程不需要操作者接管。
4. 不離開可接受的紙板／膠帶路徑範圍，不碰撞、不失控。
5. VirtualStick 指令持續成功，App 不崩潰。
6. 結果必須可由 flight log、影像或遙測回查，不能只靠目視宣稱。

AI agent 應指出這個工作定義中仍需董事長或操作者確認的部分，例如計時起點、容許偏差、是否只要求一次成功、路徑實際尺寸與安全邊界。

## Repository 與版本基線

Repository：`vicjuan/drone-agent-android-lite`

目前 `main`：

- `28d34ca`：Merge pull request #3 from `experiment/circular-under-60s`
- PR #3：<https://github.com/vicjuan/drone-agent-android-lite/pull/3>

最後有完整圓形真機成功證據的版本：

- `e901490`：`將圓形循跡圈速縮短至約三十四秒`
- 里程碑說明：<https://github.com/vicjuan/drone-agent-android-lite/pull/3#issuecomment-5369524821>

最後成功實驗的設備與結果：

- Pixel 8 Pro
- RC-N3
- DJI Mini 4 Pro
- 飛行高度約 1.2 m
- 單圈約 34 秒
- 穩定循跡約 72.1 秒，約兩圈
- 平均前進命令：0.183 m/s
- 最高前進命令：0.220 m/s
- 有效影像分析率：約 8.9 Hz
- 平均辨識信心：0.791
- VirtualStick 送出失敗：0

重要版本邊界：

- 34 秒／72.1 秒成績只屬於 `e901490`。
- PR #3 後續加入安全與時序修正；目前 `main` 尚未完成同場景圓形重飛。
- 不得把 `e901490` 的實飛成績直接宣稱為目前 `main` 的成績。

## 現行系統概觀

目前 App 使用：

1. DJI MSDK v5 Advanced Virtual Stick。
2. 向下 `-90°` 相機畫面。
3. OpenCV 黑膠帶／瓦楞紙板分割。
4. 中心線提取與路徑品質判定。
5. Image-space Pure Pursuit、橫向偏移修正與 yaw 控制。
6. 失去可信路徑時停止、重新取得、端點確認與回轉等狀態機。
7. 約 10 Hz 的影像接受節流；最後成功實飛有效分析率約 8.9 Hz。
8. 黑膠帶自主循跡時關閉 DJI avoidance，由 App 自己處理水平障礙停止。

相關主要檔案：

- `app/src/main/kotlin/com/durendal/droneagent/lite/BlackTapeDetector.kt`
- `app/src/main/kotlin/com/durendal/droneagent/lite/TapeTrackingController.kt`
- `app/src/main/kotlin/com/durendal/droneagent/lite/MainActivity.kt`
- `app/src/main/kotlin/com/durendal/droneagent/lite/VirtualStickSession.kt`
- `app/src/main/kotlin/com/durendal/droneagent/lite/ObstacleRanges.kt`
- `app/src/main/kotlin/com/durendal/droneagent/lite/CameraPreview.kt`
- `docs/vision-path-following-literature.md`
- `docs/vision-path-following-literature-technical-review.md`
- `docs/circular-path-endpoint-detection.md`

## 現行重要參數

截至 `28d34ca`：

- `CIRCULAR_TRACKING_FORWARD_SPEED_METERS_PER_SECOND = 0.24`
- `CIRCULAR_CORRECTION_FORWARD_SPEED_METERS_PER_SECOND = 0.20`
- `CIRCULAR_MAX_CENTERING_SPEED_METERS_PER_SECOND = 0.02`
- `MAX_FORWARD_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.10`
- `MAX_LATERAL_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.20`
- `CIRCULAR_MAX_YAW_ACCELERATION_DEGREES_PER_SECOND_SQUARED = 30.0`
- `VirtualStickSession.MAX_HORIZONTAL_MPS = 0.5`
- `VirtualStickSession.MAX_YAW_DEGREES_PER_SECOND = 20.0`
- `CIRCULAR_YAW_SPEED_BUDGET_DEGREES_PER_SECOND = 19.0`
- `DETECTION_COMMAND_STALE_NANOS = 400 ms`
- `AUTONOMOUS_HORIZONTAL_CLEARANCE_MM = 500`
- 影像接受間隔目標約 `100 ms`
- 控制 tick 約 `100 ms`

`20°/s` 是目前 App 內刻意保守的夾限，不應直接假設為 Mini 4 Pro 或 DJI MSDK 的硬體極限。更高 yaw 指令是否被接受、實際角速度、overshoot 與室內安全性都需要真機量測。

## 10 秒目標的量級估算

最後成功實驗的平均前進命令為 0.183 m/s、圈時約 34 秒。以命令積分粗估：

```text
0.183 m/s × 34 s ≈ 6.22 m
```

若相同路徑要在 10 秒完成，平均速度量級約為：

```text
6.22 m ÷ 10 s ≈ 0.62 m/s
```

這只是由命令推估，不是實測地速或精確路徑周長；實際路徑尺寸與 aircraft velocity telemetry 必須補量。但它已顯示目前 `0.24 m/s` 巡航目標不可能直接達到 10 秒。

若機頭大致沿路徑切線完成一圈，平均 yaw 量級至少為：

```text
360° ÷ 10 s = 36°/s
```

目前 App yaw 上限為 20°/s。若不能安全提高實際 yaw authority，就必須考慮允許更大的 lateral velocity、降低機頭切線對齊需求，或採用不同的速度向量控制方式。

粗略里程碑量級：

| 圈時 | 推估平均速度 | 一圈平均 yaw |
|---:|---:|---:|
| 34 秒 | 0.183 m/s | 10.6°/s |
| 20 秒 | 0.31 m/s | 18°/s |
| 15 秒 | 0.41 m/s | 24°/s |
| 12 秒 | 0.52 m/s | 30°/s |
| 10 秒 | 0.62 m/s | 36°/s |

## PR #3 後與最後成功實飛不同的行為

目前 `main` 相較 `e901490` 的關鍵差異：

1. 偵測命令 stale window 從 1.25 秒縮短到 400 ms。
2. 控制器 yaw 預算從錯誤的 28°/s 改為符合 App 實際夾限的 19°/s。
3. 多個依影格次數判定的防抖改為實際時間門檻。
4. 自主循跡障礙停止門檻由 150 mm 提高為 500 mm；手動控制仍為 150 mm。
5. 新增紙板色度遲滯，降低曝光／白平衡漂移造成的整段失偵。
6. 負高度樣本改為安全停止，不再可能讓定高流程拋出未捕捉例外。
7. Aircraft attitude callback 改由 UI thread 序列化處理。
8. 全解析度 RGBA 影格改用重用緩衝，降低約 10 Hz 下的大量配置與 GC 壓力。
9. 障礙 log 新增 `callbacks` 與 `maxGapMs`，但尚未取得設備量測結果。

這些變更已通過測試，但尚未取得新的完整圓形實飛證據。

## 可能面臨的核心問題

### 1. 現行速度與加速度 envelope 不足

目前圓形巡航目標只有 0.24 m/s，前進加速度只有 0.10 m/s²。即使把最高速度提高到約 0.62 m/s，從靜止加速到該速度也需要約 6.2 秒；若計時包含加速，10 秒任務幾乎沒有穩態巡航時間。

需要回答：

- 計時是否從已進入穩定巡航後開始？
- 需要多高的最高速度與加速度，才能讓平均速度達標？
- Mini 4 Pro 在 1.2 m 室內低空的安全加速／減速能力是多少？
- Virtual Stick velocity mode 的真實追蹤延遲與 overshoot 是多少？

### 2. yaw authority 與 body-frame 速度耦合

目前前進與右移命令使用 body coordinate，路徑方向快速旋轉時，yaw 能力會限制 body-forward 命令能否沿切線飛行。

需要回答：

- App／MSDK／機體可安全接受的最大 yaw rate 是多少？
- 更高 yaw rate 下，相機畫面 motion blur 與 detector 穩定性如何？
- 是否應主要依賴 yaw 對準，或改用較大的 forward/right 速度向量追蹤？
- 速度規劃是否應聯合限制 yaw rate、lateral acceleration 與路徑曲率？

### 3. Image-space 曲率缺少真實尺度

目前控制器主要使用 image-space 路徑資訊。高速時必須知道某段路徑在地面座標中的曲率，才能計算所需角速度：

```text
ω = v × κ
```

需要研究：

- 能否利用向下相機、飛行高度、FOV／intrinsics，把中心線投影到地面公尺座標？
- 高度誤差、相機姿態、鏡頭畸變與地面不平會造成多少曲率誤差？
- 如何在不預先假設路徑一定是固定圓的前提下取得 metric curvature？
- 是否需要 horizon-free homography、相機標定或 DJI attitude 補償？

### 4. 影像頻率與端到端延遲可能不足

目前有效分析率約 8.9 Hz，每幀間隔約 112 ms。若速度為 0.62 m/s，每次有效觀測之間約移動 7 cm；10 秒一圈時，每次觀測間路徑方向改變約 4°。

需要量測的不只是 Hz：

- Camera callback 到 detector 完成的延遲分布。
- Detector 到 VirtualStick 命令生效的延遲。
- busy/drop/throttle 比例。
- OpenCV 處理 p50／p95／最大時間。
- 1080p 輸入是否應降採樣、裁切 ROI、平行化或改用更低延遲管線。
- 是否需要穩定 15–20 Hz，或使用短期路徑／運動預測補足觀測間隔。

### 5. 高速造成 motion blur、曝光與視野問題

速度和 yaw 提高後，地面紋理在畫面中的位移增加，可能導致：

- 黑膠帶邊緣模糊。
- Otsu／中心線品質下降。
- 紙板色度改變。
- 近場路徑快速離開畫面。
- Pure Pursuit lookahead 在單幀之間跳變。
- 彎道外側路徑離開 FOV。

需要評估相機曝光、ISO、解析度、影格率、分析 ROI、lookahead 與濾波延遲的取捨。

### 6. `400 ms` stale window 在高速下仍可能太長，但縮短會更常停

在 0.62 m/s 下，400 ms 約可移動 25 cm。恢復舊 1.25 秒則可能盲走約 78 cm，不可接受。

核心矛盾：

- 放寬 stale window 可減少停頓，但增加使用過期幾何盲飛距離。
- 縮短 stale window 可提高安全性，但 detector 小幅抖動就可能讓任務失敗。

AI agent 應考慮 confidence、曲率連續性、offset rate、實際速度與失偵原因，而不是只用固定時間決定是否保留舊命令。

### 7. 時間型防抖與狀態轉換延遲

目前對正、路徑重取、可信路徑建立、端點確認與回轉後恢復都使用數百毫秒時間窗。它們不會降低穩定 TRACKING 階段的速度，但一旦觸發就會吃掉 10 秒預算。

需要判斷：

- 10 秒目標是否只計穩定循跡的一圈，不含起始對正？
- 圓形閉環路徑是否需要端點／180° 回轉邏輯？
- 是否可用不同模式或明確 lap detector，避免與 out-and-back 狀態混用？
- 防抖時間能否依速度、信心與觀測頻率自適應？

### 8. 360° 全向障礙最近值不適合高速方向性控制

目前自主循跡在任何方向出現 `≤500 mm` 可行動障礙時停止。高速前進需要更大的前向停止距離，但後方牆面不應阻止向前移動。

需要設計：

- 依實際速度向量選取前方障礙扇區。
- 前／側／後方不同門檻。
- 隨速度與估計停止距離調整門檻。
- 感測 callback cadence 與 freshness watchdog。
- 避障關閉時 PerceptionManager 的實際行為。

目前已有 `callbacks=N maxGapMs=M` 診斷；設備可用後必須先量測，不能猜 callback 週期。

### 9. 停止距離與室內安全空間

10 秒目標可能需要約 0.6 m/s 或更高最高速度。停止距離包含：

1. 相機曝光與傳輸延遲。
2. Detector 延遲。
3. 控制 tick 延遲。
4. MSDK 指令傳輸延遲。
5. Flight controller velocity response。
6. 機體實際減速度與慣性。

目前沒有這整條鏈的實測 p95／worst-case。不能只用 `距離 = 速度 × callback timeout` 判斷安全。

### 10. 路徑幾何與場地資料不足

目前缺少可供計算的精確資料：

- 圓形膠帶中心線半徑／周長。
- 紙板可用淨空。
- 飛機與槳葉到路徑邊界的距離。
- 牆面、家具與人的位置。
- DJI obstacle ring 各角度對應。
- 相機內參與實際畫面裁切。

AI agent 應列出下週一實驗前可離線準備，以及到現場後必須立即量測的資料。

### 11. 命令值不等於實際飛行速度

最後成功實驗只有平均／最高「命令」摘要，不能假設等於 ground speed。高速設計必須區分：

- commanded forward/right velocity
- aircraft reported velocity
- yaw command
- aircraft actual yaw rate
- image-space path motion
- 實際 lap time

需要提出同步時間戳與 log schema，避免再用未夾限命令推論機體能力。

### 12. 一次成功與可重複成功的差異

最後 34 秒實驗的前兩次啟動曾因路徑重取與原地對正由操作者接管，第三次起始角度合理後才穩定完成約兩圈。

AI agent 應明確區分：

- 一次展示成功。
- 多次可重複成功。
- 不同起始角度成功。
- 速度達標但路徑偏差過大。

若時間只允許下週一展示一次，仍需設計安全、可重複的起始程序，而不是依賴操作者碰運氣對正。

## 不接受的簡化方案

以下做法不能單獨視為完整解法：

1. 只把 `CIRCULAR_TRACKING_FORWARD_SPEED_METERS_PER_SECOND` 改成 0.6。
2. 讓控制器假設有較高 yaw，卻不提高並驗證 `VirtualStickSession` 實際輸出上限。
3. 把 stale window 改回 1.25 秒以避免停頓。
4. 關閉路徑失效停止、障礙停止或 VirtualStick 安全釋放。
5. 只用模擬或 JVM 測試宣稱 10 秒實飛成功。
6. 只報 commanded speed，不記錄實際 lap time 與 aircraft response。
7. 為固定圓硬編路徑，卻沒有先確認董事長是否允許只支援該固定幾何。
8. 在沒有真機 evidence 的情況下宣稱高 yaw／高速度安全。

## 可保留的現有基礎

提出新方案時，除非有具體證據，不應重做以下已存在能力：

- 黑膠帶／瓦楞紙板 detector。
- 中心線與路徑品質契約。
- Pure Pursuit 的資料結構與測試基礎。
- 色度遲滯。
- 時間型狀態機。
- 負高度安全停止。
- callback UI thread 序列化。
- VirtualStick authority 接管／釋放。
- 低配置 RGBA 緩衝。
- TapeCapture 錄影與 replay evidence。
- Flight log、障礙 callback 診斷。

可以替換或擴充的是速度規劃、metric geometry、yaw／lateral 聯合控制、方向性障礙判定與高速實驗流程。

## 希望另一個 AI agent 交付的內容

請針對上述問題輸出一份可執行方案，至少包含：

1. **可行性判斷**
   - 10 秒是否可能在現有 Mini 4 Pro、場地與相機架構下完成。
   - 最關鍵的物理或 MSDK 限制。
   - 哪些結論可由程式碼得出，哪些必須真機量測。

2. **至少兩種控制方案**
   - 各自的控制架構、資料需求、優缺點與風險。
   - 是否保留機頭沿切線、是否使用 lateral velocity、是否需要 metric projection。

3. **推薦方案**
   - 清楚說明為何選它。
   - 不要只提出抽象方向；需描述主要狀態、輸入、輸出與公式。

4. **程式碼修改地圖**
   - 指出需修改的檔案、類別、常數與資料結構。
   - 說明哪些既有行為必須保持。

5. **實驗階梯**
   - 建議 `20 → 15 → 12 → 10 秒` 或更合理的分段。
   - 每一階的速度、yaw、加速度、停止條件與觀察指標。
   - 不可一次跳到未驗證最高值。

6. **安全與 abort 條件**
   - 路徑偏差、失偵、障礙、遙測失效、VirtualStick failure、過大 yaw／速度時如何立即停止或接管。

7. **量測與 log 設計**
   - 實際速度、yaw rate、延遲、曲率、路徑偏差、lap time、callback cadence 的同步記錄方式。

8. **驗收標準**
   - 一圈如何計時。
   - 容許偏差。
   - 成功重複次數。
   - 需要保存的 evidence。

9. **下週一現場操作清單**
   - 起飛前量測。
   - 地面／懸停 actuator 測試。
   - 每階段飛行順序。
   - 何時不得繼續提速。

10. **風險排序**
    - 依「會阻止 10 秒達成」與「可能造成安全事故」分開排序。

## 當前已知結論

1. PR #3 已完成審查並合併，但目前 `main` 尚未取得審查後的完整圓形重飛證據。
2. 最後有完整實飛證據的版本是 `e901490`，約 34 秒一圈。
3. 目前 `0.24 m/s`、`20°/s` 的 App envelope 從量級上不足以達成 10 秒。
4. 10 秒目標不能靠移除安全停止或延長盲飛時間達成。
5. 真正需要解決的是 metric curvature、水平／yaw authority、端到端延遲、視覺可靠度、方向性避障與可控的實驗階梯。
6. 下週一設備可用後，第一批必要資料是實際路徑尺寸、aircraft velocity／yaw response、影像延遲，以及 obstacle `callbacks`／`maxGapMs`。
