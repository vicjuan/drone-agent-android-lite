# 立即執行：完成新視覺管線與安全飛控切換

不要停在 shadow、overlay、報告或分析。不要再修改 capture infrastructure。完成所有不需要人工操作飛機的部分；遇到測試失敗就修根因並繼續，不要在中間階段停下來回報。

## 最終目標

1. 新 `CenterlineExtractor` 成為正式黑膠帶幾何來源。
2. 移除 row/column 方向偏見與 `HORIZONTAL_FALLBACK`。
3. 正式產生 `FULL_PATH / NEAR_FIELD_ONLY / LOST`。
4. 控制器依品質安全控制 forward/right/yaw。
5. 新中心線與品質顯示在 preview。
6. 使用既有真實影像素材與 Pixel 8 Pro 完成驗證。
7. connected tests 後重新安裝 APK、冷啟動並確認程序存活。
8. 不宣稱沒有執行的 Mini 4 Pro 實飛結果。

## 一、使用既有真實素材，不准因為沒有新 capture 停工

先搜尋並使用 repository 既有的：

- `app/src/androidTest/assets` 真實場景圖片。
- 先前從 Mini 4 Pro／Pixel 錄影抽出的影格。
- `final-cardboard-tape.jpg`、loop、glare、dark-floor 等既有素材。
- 先前保存的影片、flight log 與 detector diagnostics。

至少涵蓋：直線、曲線、反光斷裂、深色／灰色地板、紙板邊界、末端、短水平殘片、分支、多候選、無路徑、遺失後重新取得。

螢幕錄影影格不是 detector 原始 bytes，必須標記限制；但它仍是真實場景證據，不能因不完美就不用。

不要建立新的 capture 格式、queue、event model、下載工具或資料管理框架。

## 二、完成新幾何語義

### 拆開兩種曲率

- `lookaheadHeadingChangeDegrees`：near-field 到 lookahead 的局部轉向，用於近期 steering／Pure Pursuit 分析。
- `totalPathTurnDegrees`：整條可見中心線首尾切線差，用於曲線分類，替代舊 estimator 的整體曲率語義。
- `turnConsistency`：若仍需要平滑度，明確表示轉向是否同向且分布合理。

補直線、左曲線、右曲線、S 形、短殘片與 hairpin 測試。不要調常數讓新舊數字看似相同。

### 正式消費 topology

- `branchCount > 0`：不得產生 `FULL_PATH`；近場可信可為 `NEAR_FIELD_ONLY`，近場也有歧義則為 `LOST`。
- `closedLoop`：在 `CIRCULAR` 模式是合法路徑，不得一律拒絕，也不得產生一般末端判定。
- `distalTerminus == INSIDE_FRAME`：可成為膠帶末端候選。
- `distalTerminus == AT_FRAME_BORDER`：代表路徑離開畫面，不是末端。
- `distalTerminus == NONE`：依 closed-loop 與路徑長度解釋，不得假裝成末端。

## 三、建立正式路徑品質

### `FULL_PATH`

必須具備可信 near-field centerline、anchor、足夠弧長、完整 lookahead，且沒有未解分支歧義。

### `NEAR_FIELD_ONLY`

near-field 方向與 anchor 可信，但 lookahead 不存在或前方有分支／斷裂／不足。不得製造 bounding-box lookahead，也不得沿用上一幀 lookahead。

### `LOST`

near-field 不可信，沒有可安全對準的中心線。資料模型只能有一個一致表示，不能半套。

先採保守門檻：寧可停止，不可讓短殘片取得前進權。門檻必須由合成與既有真實素材共同測試。

## 四、讓新中心線成為正式 detector 幾何來源

目前 shadow 只處理舊 estimator 選出的 winner，這不算完整切換。完成真正 cutover：

1. 保留現行 segmentation、morphology、floor/board context 與便宜的 area/chroma 前置篩選。
2. 對通過前置篩選的候選使用新中心線幾何。
3. 以新 geometry、topology、continuity 與 quality 決定拒絕與勝者。
4. 不得依賴舊 estimator 選好 winner 後才使用新中心線。
5. 移除 `TapePathDirectionEstimator` 的 production caller。
6. 移除 `HORIZONTAL_FALLBACK`。
7. 移除 temporary shadow A/B、shadow-only log 與舊 overlay；正式 overlay 只顯示實際控制使用的中心線。
8. 若效能不合格，先修重複配置、mask 轉換或不必要的候選計算；不要永久保留雙軌。

## 五、同一批完成控制器安全行為

### `FULL_PATH`

- 才允許 forward、right 與 Pure Pursuit。
- lookahead 必須來自本幀。
- stale observation 到期立即歸零。

### `NEAR_FIELD_ONLY`

- `forward = 0`
- `right = 0`
- Pure Pursuit = 0
- 只允許 bounded in-place yaw alignment。
- observation stale 時 yaw 也歸零。

### `LOST`

- `yaw = 0`
- `forward = 0`
- `right = 0`
- 清除濾波後 anchor、lookahead 與 offset。
- 進入重新取得，不得使用上一幀控制量。

直接測試最終 `TapeTrackingDecision`：

- FULL_PATH 正常循跡與曲線 Pure Pursuit。
- FULL_PATH → NEAR_FIELD_ONLY 當幀停止平移。
- NEAR_FIELD_ONLY 只 yaw。
- NEAR_FIELD_ONLY → LOST 全零。
- FULL_PATH → LOST 全零。
- LOST 後重新取得不得復用 stale lookahead。
- branch path 不前進。
- endpoint 與 frame-border terminus 不混淆。
- closed-loop 在 circular mode 不被誤判為末端。

## 六、正式可見功能

Preview 必須顯示真正驅動控制器的：

- 中心線 polyline。
- anchor。
- lookahead。
- `FULL_PATH / NEAR_FIELD_ONLY / LOST`。
- branch／closed-loop／endpoint 簡短狀態。

不做 UI 美化。Pixel 8 Pro 上實際開啟 App 確認畫面存在，不只測 Canvas 呼叫。

## 七、效能與驗證

完成後執行：

1. JVM 單元測試。
2. 所有 Android connected tests。
3. `lintDebug`。
4. 使用既有真實素材量 p50、p95、p99、max 與 busy/drop count。
5. 確認完整 production pipeline 在 250 ms intake interval 內保留合理餘裕。
6. 確認沒有每幀不必要的大型配置或無界 queue。
7. connected tests 後重新 assemble、安裝、cold launch，驗證 `pm path`、PID 與無 `AndroidRuntime` fatal。
8. 保存一張 Pixel 8 Pro 正式中心線 overlay 畫面作為可見成果。

## 八、提交要求

任何可能被安裝的最終 head 都不得存在：

- 新 producer 已生效、controller gating 未生效。
- quality 已產生、控制器仍忽略。
- 舊 estimator 已移除、新 candidate selection 尚未完成。
- 測試綠但 APK 未重新安裝。
- overlay 與實際控制路徑不一致。

不要再新增規劃文件或階段報告。程式碼、測試、Pixel 畫面與可執行 APK 才是交付。

## 九、硬體邊界

沒有 Mini 4 Pro／RC-N3 操作者時：

- 不得假造 capture 或宣稱實飛。
- 不得為了完成而放寬品質門檻。
- 不得讓未驗證的新控制 build 自動起飛。
- 不得因此停止其他可執行工作。

完成所有軟體、既有真實素材、Pixel 8 Pro 與安裝驗證後，只留下：

- 需要使用者執行的飛行步驟。
- 每一步要觀察的畫面與飛行行為。
- 通過／失敗判準。
- 若失敗應取回的 capture 與 log。

## 完成定義

- 新中心線正式產生 detection。
- topology 正式被消費。
- 品質正式控制命令。
- 舊 estimator 與 horizontal fallback 已移除。
- Pixel 8 Pro 顯示正式中心線。
- 單元、connected、lint、效能與冷啟動驗證通過。
- APK 最後確實安裝並可啟動。
- 只剩 Mini 4 Pro 實飛這個無法由軟體 agent 單獨完成的外部驗證。

現在開始。不要停在中途詢問、報告或等待；完成所有可執行工作後再一次交付結果。
