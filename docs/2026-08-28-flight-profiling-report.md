# 系統 Profiling 報告：2026-08-28 真機飛行（第二輪實測）

本報告是 `docs/system-profiling-report.md`（第二輪細分量測計畫）對應的實測結果。所有統計均由本趟循跡飛行及後續專用硬體延遲脈衝的 `flight-profile.tsv` 重新計算，非引用先前估計值。

## 證據等級定義

| 標記 | 意義 |
|---|---|
| **measured** | 直接來自 trace / log / 影片畫素的數值 |
| **derived** | 由 measured 數值經明確演算法計算而得 |
| **operator observation** | 操作者現場肉眼觀察，無 trace 佐證 |
| **inference** | 合理推論，尚無獨立證據 |

## 實驗範圍與證據

- 裝置與版本（measured，`session_start`）：Google Pixel 8 Pro、Android 16、MSDK 5.18.0、OpenCV 4.14.0；目標頻率 visionHz=20、controlHz=10、stickHz=20。
- 飛行 App session：13:53:56.256 profiling start（log），TSV `elapsedMs` 以此為零點；起飛命令 13:54:10.560 送出、13:54:10.688 accepted；操作者 13:55:49.337 要求降落、13:55:52.422 降落確認（高度 0.6→0.1 m）。飛行高度約 1.20 m、gimbal pitch −90°（俯視）。
- 證據檔案：
  - `/tmp/drone-latest-flight-20260828/flight-profile.tsv`（9,956 列事件）
  - `/tmp/drone-latest-flight-20260828/flight-log.txt`
  - `/tmp/drone-latest-flight-20260828/screen-20260828-135535.mp4`（螢幕錄影 87.003 s、2244×1008 H.264）
  - `video-contact-sheet.jpg`、`tracking-contact-sheet.jpg`、`tracking-stop-contact-sheet.jpg`（由錄影抽幀）
  - `tape-captures/`：40 個 path-lost 擷取目錄（編號 000000056–000000096，缺 000000078），每個含 1920×1080×4 原始幀與 4 張 640×360 遮罩及 `capture.txt`；log 記錄 saved=96、dropped=1、104,612,669 bytes（保留策略只留最後 40 個，measured）。
  - `/tmp/drone-hardware-latency-20260828/flight-profile.tsv`（專用硬體延遲脈衝 session）
  - `/tmp/drone-hardware-latency-20260828/flight-log.txt`
  - `/tmp/drone-hardware-latency-20260828/screen-20260828-141845.mp4`（螢幕錄影 99.815 s、2244×1008 H.264、6,719 幀）
  - 上述三檔 SHA-256 依序為 `50fa50a65c7417c9d88c4d451ef26117517c776effe4c194b7ae36d8adbb94ab`、`6790ce48f415cbc94a3bae30bfe4eb994f230fae752e919ae8b17861ce51bf2f`、`01973895675c26ac8a9f629ff6fd09e773ded68d4064022e36f167b756899fbc`。

## 循跡區段邊界（measured）

| 事件 | elapsedMs | 牆鐘時間 | 內容 |
|---|---|---|---|
| `tracking_start` | 54,271 | 13:54:50.50 | mode=FIXED_HEADING、circularSpeed=FAST、yawControl=RATE、fixedSpeed=BOOST |
| `tracking_stop` | 86,221 | 13:55:22.45 | elapsedMs=31,949.498，reason=「方案 B 失去有效路徑或取得逾時，已停止」 |

循跡段長 **31.949 s**。啟動時避障切為 CLOSE（13:54:50.441），停止後還原 BRAKE（13:55:22.483）並釋放控制權回 RC（13:55:22.663）。HUD 顯示速度檔為「花冰高速・1.25 m/s（Pure Pursuit 前視點導引）」，與命令上限（|forward| max 1.25 m/s，measured）一致。

### 循跡窗內事件樣本數（measured；括號為整個 session）

| 事件 | 窗內 n | （全程 n） |
|---|---|---|
| `vision` | 395 | (1,698) |
| `detection_ui` | 395 | (1,698) |
| `control` | 304 | (304) |
| `virtual_stick` | 636 | (4,747) |
| `velocity` | 264 | (383) |
| `attitude` | 318 | (1,123) |

窗內偵測成功率 375/395 = **94.9%**；有效視覺吞吐 13.0 fps（394 個間隔 / 30.35 s，derived；目標 20 Hz，未達成主因是單幀處理平均 61.8 ms 加上 throttle/busy 丟幀）。相機 RGBA callback 累計約 52–54 Hz（log，measured）。

以下所有分段統計除特別註明外，均只取循跡窗（monotonic 時戳落在 `tracking_start`..`tracking_stop` 之間）；百分位數採排序後線性內插。

## 端到端軟體路徑（measured / derived）

以每筆 `newFrame=true` 的 control tick 為單位，將 `vision.frameNanos`（相機幀時戳）與該 tick 之後第一筆 `virtual_stick` 逐筆 join，得到「相機幀擷取 → MSDK 呼叫返回」的真實端到端分布（n=292，derived-join over measured timestamps）：

**mean 140.7 ms、P50 136.9 ms、P95 201.8 ms、P99 254.7 ms、max 662.4 ms、min 59.1 ms**

```text
相機影格擷取
  │ 1.1 ms   複製與排隊 queueMs
  ▼
OpenCV 全管線（見下表細分）
  │ 60.7 ms  totalMs − queueMs
  ▼
偵測回呼進 UI 執行緒
  │ 4.3 ms   visionToCallbackMs
  ▼
等待 10 Hz 控制 tick
  │ 41.9 ms  uiToTickMs（newFrame=true，n=292）
  ▼
控制器決策與命令更新
  │ 0.2 ms   preDecision+decision+decisionToCommand
  ▼
等待下一個 20 Hz Virtual Stick 幀
  │ 25.8 ms  逐命令對齊（見下節）
  ▼
sendVirtualStickAdvancedParam 同步返回
  │ 0.8 ms   sendCallMs
  ▼
（本趟觀測邊界；機體反應未量測，見「未量測項目」）
```

各階段平均相加為 134.6 ms，與逐幀 join 的 140.7 ms 差約 6 ms：uiToTick 只在 newFrame tick 有值、而「等待搖桿幀」是對全部 304 個 tick 平均，兩者取樣集合不同（derived 說明）。max 662 ms 的單一離群值發生在 85,183 ms——正是相機串流中斷開始後控制迴圈消化最後一張舊幀的時刻（見中斷一節）。

對照：第一輪報告的軟體段為 77.9 + 59 + 25 ≈ 162 ms（不同場地與日期，僅供參考，inference 級比較）。

## OpenCV 子階段統計（循跡窗，n=395，單位 ms，measured）

| 子階段 | mean | P50 | P95 | max | 佔 totalMs |
|---|---:|---:|---:|---:|---:|
| queueMs | 1.09 | 0.97 | 1.79 | 4.98 | 1.8% |
| preprocessMs | 16.44 | 16.80 | 23.39 | 42.17 | 26.6% |
| thresholdMs | 0.84 | 0.72 | 1.31 | 7.11 | 1.4% |
| floorContextMs | 3.29 | 2.88 | 5.16 | 51.31 | 5.3% |
| morphologyContoursMs | 9.92 | 9.52 | 13.12 | 27.04 | 16.1% |
| **candidateMs** | **29.15** | **21.85** | **67.61** | **102.34** | **47.2%** |
| cleanupMs | 0.99 | 0.80 | 1.33 | 44.00 | 1.6% |
| otherMs | 0.03 | 0.02 | 0.06 | 0.28 | <0.1% |
| **totalMs**（含 queue） | **61.76** | **53.81** | **100.93** | **137.82** | 100% |

第一輪「77.9 ms 整包視覺處理」現已拆開：候選輪廓／中心線／幾何評分（candidateMs）以 47% 佔比為最大項且尾部最長（P95 67.6 ms），其次是前處理 27% 與形態學+輪廓 16%。Otsu 門檻與地板判斷合計 <7%。session 第一幀冷啟動 493 ms（含初始化）不在窗內，未計入。

## 偵測之後：UI、控制 tick、決策（measured）

| 分段 | n | mean | P50 | P95 | max |
|---|---:|---:|---:|---:|---:|
| visionToCallbackMs | 395 | 4.31 | 4.19 | 5.63 | 26.90 |
| uiToTickMs（newFrame） | 292 | 41.88 | 41.19 | 77.48 | 179.75 |
| preDecisionMs | 304 | 0.02 | 0.02 | 0.05 | 0.18 |
| decisionMs | 304 | 0.12 | 0.10 | 0.27 | 0.90 |
| decisionToCommandMs | 304 | 0.04 | 0.03 | 0.07 | 1.19 |
| 控制 tick 間隔 | 303 | 105.08 | 100.99 | 119.96 | 429.93 |

- 控制迴圈實際跑在約 9.5–10 Hz（304 tick / 31.95 s）；292/304 tick 拿到新幀，12 個 tick 沿用舊幀（newFrame=false，多數集中在串流中斷段）。
- 第一輪的「59 ms」現在確認幾乎全是**等待固定 10 Hz tick**（平均 41.9 ms，理論期望值 ≈ 半個 tick 週期 50 ms），控制器本身計算 <0.2 ms。
- 唯一 >200 ms 的 tick 間隔（429.9 ms，84,570→85,000）發生在相機串流中斷開始的同一刻（measured 同時性；主執行緒短暫卡頓的因果屬 inference）。

## 命令 → 下一個 Virtual Stick 幀：對齊方法（重要）

TSV 中 `virtual_stick.commandToFrameMs` 記的是「該幀距**目前命令設定時刻**多久」。20 Hz 發送迴圈會把同一命令重送到後續每一幀，所以這個欄位隨重送次數線性膨脹（窗內原始值 mean 54.8 ms、max 395.9 ms；session 最後一筆高達 175,297 ms）。**直接平均全部 `commandToFrameMs` 是錯的。**

正確算法（derived）：對每筆 `control` 事件，以 monotonic 時戳二分搜尋其後第一筆 `virtual_stick`，取時間差：

| 統計（n=304） | 值 |
|---|---:|
| mean | **25.80 ms** |
| P50 | 23.24 ms |
| P95 | 47.14 ms |
| P99 | 49.63 ms |
| max | 185.16 ms |

P50≈23 ms、P95≈47 ms 與「10 Hz 命令對 20 Hz 幀、相位自由」的理論分布（0–50 ms 均勻、期望 25 ms）吻合，證實此段純粹是取樣等待，無額外排隊。max 185 ms 是循跡啟動第一筆命令（54,280 ms）的一次性暫態。

## Virtual Stick 20 Hz 與 MSDK 呼叫（measured）

- **636/636 幀 success=true**，無任何失敗；log 之 `frames=… fails=0` 全程一致。
- 幀間隔：mean 50.00 ms、P50 50.01、P95 50.51、max 65.06、min 35.11 ms → 實測 **20.000 Hz**。即使在相機串流中斷期間，最大幀間隔也只有 65.1 ms（84,815 ms 處）。
- `sendVirtualStickAdvancedParam` 同步呼叫 `sendCallMs`：mean 0.80 ms、P50 0.62、P95 1.85、P99 3.58、max 11.26 ms。MSDK 入口不是瓶頸。

## 遙測：KeyAircraftVelocity 與地速（measured / derived）

- callback 間隔（窗內，排除跨窗缺口樣本，n=263）：mean 117.9 ms、P50 101.6、P95 218.3、max 446.0 ms → 有效約 8.5 Hz，並非嚴格 10 Hz。
- **16.6 s 遙測缺口**：窗內第一筆 velocity（55,113 ms）的 `intervalMs=16,648.4`，前一筆在 38,465 ms（起飛後手動段）。即 KeyAircraftVelocity 曾停送 16.6 s，跨越循跡開始。任何以速度序列做的相關性分析（含第一輪 380 ms 估計法）都必須先處理這類缺口，否則會被插值假象污染（後半句為 inference 警示）。
- attitude callback 間隔：mean 100.3 ms、P50 100.1、max 445.9 ms（n=317），約 10 Hz。
- 地速分布（n=264）：mean 0.45、P05 0.20、P25 0.30、P50 0.42、P75 0.58、P95 0.72、**max 0.81 m/s**；無任何樣本 >1.0 m/s。命令上限 1.25 m/s，實際地速遠低於命令值——固定機頭繞圈時命令向量方向持續旋轉，機體加速度跟不上（解讀屬 inference）。
- 水平位移積分（velocity 樣本梯形法，derived）：約 **14.1 m**／31.1 s 有樣本區間。受 8.5 Hz 取樣與缺口影響，屬粗估。

## 固定機頭穩定度（measured）

循跡窗內 heading（n=318）：mean −153.58°、母體標準差 **0.475°**、全距 2.8°（−155.1°..−152.3°）、P05–P95 跨度 1.51°。命令 |yawRate| P95 僅 0.79°/s、max 1.20°/s。FIXED_HEADING 模式將機頭鎖定在起始朝向 ≈ −153.6°，本趟成立。

## 圈數：只能記為 operator observation

- **operator observation**：操作者肉眼認為飛機在 10 秒內完成一圈。
- **trace 事實（measured）**：本趟 TSV 與 log **沒有任何 lap 事件**。13:54:50.710 只有 `diagnostic turn cycle armed…notPhysicalLap=true`，其後 31.9 s 內無任何 turn-cycle 完成事件；固定地標自動計圈仍未實作。
- **derived 佐證（非圈數量測）**：控制器 `travel` 角度三次經過同值點（+18.4°@13:55:00.40、+18.5°@13:55:10.05、+18.7°@13:55:19.34），間隔 9.64 s 與 9.29 s。此為視覺推算的角度進度回繞，與物理圈認定無對應保證，**不得**寫成圈速量測結論。

## 4.627 秒相機串流中斷與安全停止（根因證據鏈）

時間線（measured；牆鐘由 elapsedMs + 13:53:56.256 換算）：

| 時刻 | 證據 |
|---|---|
| 13:55:20.92（84,660） | 最後一張正常視覺幀 seq 1122 完成，**detected=true**（其前 5 幀 1117–1121 也全部 detected=true） |
| 13:55:21.34 | log `callbackAgeMs=500`、cameraCallbacks=4400 → 相機 RGBA callback 已停 0.5 s |
| 84,570→85,000 | 控制 tick 出現全窗唯一 429.9 ms 停頓 |
| 13:55:21.5–22.2 | 控制器 `rawAngle` 凍結在 +319.4°，forward 1.24→0.83→0.48→0.22→**0.00** 斜坡歸零（log 逐 tick 可見） |
| 13:55:22.446（86,221） | `tracking_stop`：「方案 B 失去有效路徑或取得逾時，已停止」＝最後一次新幀消化（85,183）後約 1.04 s 逾時觸發 |
| 13:55:22.48–22.66 | 避障還原 BRAKE、Virtual Stick 釋放、控制權回 RC；飛機懸停 |
| 13:55:23.68 | log `RGBA frame stream stale; detector reset` |
| 13:55:25.55（89,287） | 下一張視覺幀 seq 1123 完成；**seq 1122→1123 間隔 4,626.8 ms**；cameraCallbacks 僅 4400→4405 |

結論（依證據分級）：

- **measured**：中斷發生在相機 RGBA callback 層（callback 計數凍結、callbackAgeMs 飆升），不是偵測演算法失敗——中斷前最後 6 幀全部成功偵測到膠帶。停止是「取得新幀逾時」保護，不是「路徑消失」。
- **measured**：安全機制全部按設計動作——命令先斜坡歸零、逾時後停止循跡、還原 BRAKE、交還 RC，期間 20 Hz 幀從未失敗。
- **獨立佐證（measured，影片）**：螢幕錄影在 t≈72.7–77.5 s 為全黑幀（64×36 降採樣平均亮度 = 0.0），換算牆鐘 13:55:20.7–13:55:25.5，與 trace 中斷區間吻合在 0.3 s 內。
- **inference（根因未定）**：callback 停止的上游原因無 log 錯誤可查。候選：MSDK 影像解碼／傳輸暫停、系統 I/O 壓力（自動擷取已寫入 ~105 MB）、或 App 主執行緒卡頓（與 429.9 ms tick 停頓同時）。需在下一趟以 MSDK 影像層診斷或 systrace 釐清，本報告不下結論。

## 錄影與 trace 對齊方式與限制

- 錨點：mp4 `creation_time = 2026-08-28T05:55:35Z`（13:55:35 本地，**秒級解析度**）為錄影**結束**時刻；起點 = 13:55:35.000 − 87.003 s ≈ 13:54:08.0。
- 交叉驗證（measured）：(1) 影片開頭第一格 contact sheet 即「起飛 指令已被飛機接受」（log 13:54:10.69，落在影片 t≈2.7 s）；(2) t=42.5 s 抽幀 HUD 顯示控制權=RC、avoidance=CLOSE、heading=−153.6°，正是 13:54:50.44–50.69 的循跡啟動過渡窗；(3) 黑幀區間與 trace 串流中斷差 <0.3 s。
- 限制：`creation_time` 只有秒級，且結束時刻與最後一幀的關係未經器材級校準，因此**全部影片↔trace 對齊只宣稱 ±1 s**。凡需要毫秒級的結論（圈速、硬體延遲）都不能用本錄影建立。

## 後續專用硬體延遲脈衝（measured）

原循跡 session 沒有 `latency_test_*` 事件，故前述循跡資料本身不能量硬體反應；14:16:53 啟動的後續專用 session 已補做完整實驗。控制權取得後，App 固定機頭、以機體座標前後速度 `+0.50 / −0.50 m/s` 各維持 1 秒，中間歸零 2 秒，重複 10 組；畫面中央 SYNC 方塊在非零脈衝時變白。

- trace 完整性：`latency_test_armed` 1 筆、`latency_test_command` 41 筆（baseline 1、非零 20、歸零 20）、`latency_test_stop` 1 筆，`completed=true`；總長 63,380 ms。20 段非零命令平均維持 1,005.1 ms，20 段歸零平均維持 2,004.8 ms。
- MSDK 發送：實驗窗內 `virtual_stick` 1,265 / 1,265 成功。每筆 control event 對齊「下一筆」20 Hz virtual-stick 幀，命令→幀為 mean 23.6、P50 24.7、P95 43.3、max 49.8 ms；不可平均同一命令的後續重送幀。
- 反應判定：將 `KeyAircraftVelocity` 的地理座標 x/y 依最近一筆 heading 投影到機體前向；每個非零命令取方向一致且首次達 `|bodyForward| ≥ 0.08 m/s` 的 velocity callback。原始速度量化至 0.1 m/s，因此此門檻實際代表第一筆方向正確的 ±0.1 m/s 回報。

| 邊界（20 次非零脈衝） | min | mean | P50 | P95 | P99 | max |
|---|---:|---:|---:|---:|---:|---:|
| control 命令→首次地速反映 | 274.5 ms | **351.8 ms** | **351.8 ms** | **406.5 ms** | 419.7 ms | **423.0 ms** |
| 首次 virtual-stick 幀→首次地速反映 | 245.3 ms | 328.2 ms | 344.7 ms | 401.9 ms | 404.7 ms | 405.4 ms |
| MSDK 呼叫返回→首次地速反映 | 240.8 ms | 327.2 ms | 344.2 ms | 398.9 ms | 402.5 ms | 403.4 ms |

正向 10 次的 control→地速 mean 360.0、P50 358.0、max 423.0 ms；反向 10 次 mean 343.6、P50 351.4、max 405.7 ms。20 次中 16 次不超過 380 ms；原先「約 380 ms」比本次 mean/P50 高約 28 ms，但落在實測分布內，量級獲得支持。歸零命令至首次 `groundSpeed < 0.05 m/s` 回報為 mean 519.8、P50 499.9、P95 708.0、max 982.1 ms；此值包含飛機減速物理過程。

影片獨立驗證：以 60 fps 解碼後，畫素直接找到 20 次黑→白 SYNC 邊緣；白色持續 mean 1,005.0 ms，邊緣間隔 mean 3,010.5 ms。以第一個白邊作錨點後，其餘 19 個白邊相對 trace 的殘差為 mean 9.2、P95 18.2、max 21.4 ms，符合錄影幀量化；20 次白色脈衝皆可見俯視畫面交替位移。影片證明「標記、命令、可見移動」同屬一個節奏，但沒有第二支相機直接拍機體，且 DJI 相機、無線下行、解碼與螢幕錄影本身都有延遲，故不以畫面位移幀宣稱純機體起動時間。

量測限制：351.8 ms 是「App control event → MSDK KeyAircraftVelocity 首次回報 ±0.1 m/s」的端到端上界，不是馬達／槳葉開始加速時間。它包含最多約一個 20 Hz 搖桿幀等待、飛控與機體反應、速度估計器，以及約 10 Hz velocity callback 的取樣量化；真正物理起動應早於首次遙測回報。仍無器材級 240 fps 外拍可進一步拆出純機體反應。

- 無 lap 事件（見圈數一節），圈速不可宣稱。

## 結論與下一步優先順序

軟體路徑已完整拆解且量到端（相機幀 → MSDK 返回）：**P50 136.9 ms / P95 201.8 ms**。組成中可優化空間排序：

1. **candidateMs（mean 29.2 ms、P95 67.6 ms、佔 47%）**——路徑候選評分是最大且尾部最長的計算項，優先剖析其內部（輪廓數與 region 大小相關性在 log 可見）。
2. **uiToTickMs（mean 41.9 ms）**——純粹是 10 Hz 控制 tick 的相位等待；把控制迴圈提到 20 Hz 或改為偵測驅動，理論上平均可省 ~25–40 ms（inference，需驗證控制穩定性）。
3. **等待搖桿幀（mean 25.8 ms）**——受 20 Hz 幀率相位限制，命令即時插發或提高 stickHz 才能壓縮。
4. **preprocessMs（16.4 ms）**——縮放/色彩轉換，可評估解析度或 NEON/GPU 路徑。
5. 視覺吞吐 13.0 fps < 20 Hz 目標：處理時間壓下來後 throttle/busy 丟幀自然減少。

穩定性議題優先於效能：**相機串流 4.6 s 中斷的根因診斷**（MSDK 影像層 log + systrace + I/O 監控）應排最前，因為它直接終止了循跡。硬體延遲脈衝已完成，control event→首次速度回報的實測邊界為 P50 351.8 / P95 406.5 ms；若需把飛控／機體與遙測取樣拆開，再補 240 fps 外拍。其後依序補固定地標自動計圈，以及 KeyAircraftVelocity 缺口告警。
