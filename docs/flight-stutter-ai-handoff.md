# 求助：Mini 4 Pro 持續控制下的急衝急停／頓挫

請獨立分析根因，提出最少、能區分假說的驗證方式。你可以讀取下列本機程式、影片及日誌；先分析，不直接修改程式、更新韌體或發送飛行指令。

## 核心現象

- **滑冰模式會有明顯「急衝 → 急停／明顯減速 → 再急衝」現象。**這裡的滑冰指固定機頭方向，靠前後、左右平移繞圈；原本視覺循線的 B4 實飛尤其明顯，不只是畫面或辨識線抖動。
- **剛剛的純數學圓「賽車模式」也有較輕微的同類跡象：持續前進、左轉時，仍會短暫頓一下再加速。**不要因為模式不同就忽略這個共同現象，也不要直接假定兩者必然同根因。
- 最新賽車快版確實在約 **21 秒內實際繞三圈，約 7 秒一圈**。這是使用者明確指出、影片可見的成果，不只是 UI 排程。SDK 平均線速度增幅小，不能用來否定圈速成果；實際圈徑是否精確維持 1.5 m 尚未量測。

## 環境與已知事實

- 專案：`/Users/vic/git/drone-agnet-android-lite`；Android MSDK **5.18.0**；飛機 **DJI Mini 4 Pro**。
- 已從硬體讀到：飛機韌體 **01.00.1100**、遙控器韌體 **01.01.0300**。不要再把「升到 01.00.1100」當成尚未做的解法。
- 目前影片是在室內桌椅旁，存在桌面／地板高度差，尚非空曠平坦場地對照。
- 數學圓不使用辨線結果控制：按時間產生地球座標速度，再依實際機頭方向轉成機身座標。滑冰保持初始航向；賽車讓航向沿圓的切線向左轉，兩者都用 yaw `ANGLE`。
- 最新賽車快版：設定直徑 1.5 m、7.0 秒×3圈、**40 Hz**；主要向前、帶少量左移，同時持續更新左轉目標航向。**合成速度命令固定 0.673 m/s，不代表每一軸數值固定。**
- 最新有效運行有 841 個非零 ANGLE 發送樣本，間隔中位約 25 ms、最大約 33.7 ms，沒有反覆歸零或約一秒的 App 停送。SDK 呼叫成功不等於已證明飛控收到並正確執行。
- 賽車運行時避障設定為 **`CLOSE confirmed=true`**，畫面 `MANUAL · NO OA`；排程結束後才恢復 `BRAKE`。設定關閉不代表可排除飛機端的避障異常，見下方官方案例。
- 數學圓的 20／40 Hz、滑冰／賽車四組都有速度起伏，約每秒一次；目前沒有一致的頻率優劣。40 Hz 高於 DJI 文件建議的 5–25 Hz，但 20 Hz 也有現象，不能只憑頻率定罪。

## 本機證據：建議依此順序看

所有飛行資料根目錄：`/Users/vic/drone-flights/`。

1. **最新數學圓賽車快版**：`2026-09-11-fast-racing/`
   - 影片：`screen-20260911-170847.mp4`，**看第 10–32 秒**；約 10–11 秒是啟動前等待，不要算成運行中的急停。
   - TSV：`flight-profile-1789117579709-541272777073087.tsv`。
   - `review-results.json`、`flight-log-snapshot.txt`、`review-01.jpg` 至 `review-07.jpg`。已有影片／TSV 複本校驗紀錄。
2. **滑冰／固定機頭 B4 急衝急停**：
   - 影片：`2026-09-11-circle/131539/screen-20260911-131539.mp4`，從第 15 秒看。
   - 分析：`2026-09-11-circle/sprint-stop/findings.json`、`signals.json`、`command-versus-motion.svg`、`constant-command-comparison.json`。
   - 有命令仍很強、視覺回饋有效，但速度下降並伴隨反向傾斜煞車的片段；另有少數 App 確實收速的片段，不能混為一談。
   - **舊分析中「硬體韌體尚未確認」已過期，以本檔的實讀版本為準。**
3. **四組數學圓對照**：`2026-09-11-mathematical-circle/`，均為 7.5 秒排程、合成速度約 0.628 m/s。

   | 影片檔名 | 指定區間 | 模式／發送頻率 |
   |---|---|---|
   | `screen-20260911-160730.mp4` | 6–22 秒 | 賽車 20 Hz |
   | `screen-20260911-160952.mp4` | 44–66 秒 | 賽車 40 Hz |
   | `screen-20260911-161345.mp4` | 28–50 秒 | 滑冰 20 Hz |
   | `screen-20260911-161518.mp4` | 4–26 秒 | 滑冰 40 Hz |

   同資料夾有配對 TSV、`review-results.json`、`matched-window-comparison.json`、`matched-speed-comparison.png`。DJI 速度分量量化至 0.1 m/s；機載線速度、畫面位移、實際圈速需分開解讀。TSV 以 `monotonicNanos` 對時，勿直接用影片檔名時間推算起飛／控制開始時間。

程式入口位於專案的 `app/src/main/kotlin/com/durendal/droneagent/lite/`：`MainActivity.kt`（數學圓啟停與 drive）、`MathematicalCircleController.kt`、`VirtualStickSession.kt`；原本循線另看 `FixedHeadingLapController.kt`、`TapeTrackingController.kt`。

## 已找到的官方相似案例

- [DJI GitHub #594 官方回覆，2025-07-10](https://github.com/dji-sdk/Mobile-SDK-Android-V5/issues/594#issuecomment-3056344439)：承認 Mini 4 Pro 在狹窄環境，即使避障已停用，仍可能觸發而停止移動或晃動。原回報同樣是室內桌椅、Advanced Virtual Stick、低於 1 m/s。
- [原回報者後續，2025-11-11](https://github.com/dji-sdk/Mobile-SDK-Android-V5/issues/594#issuecomment-3514637396)：表示新韌體似乎已修好。
- [DJI #670 官方回覆，2025-11-19](https://github.com/dji-sdk/Mobile-SDK-Android-V5/issues/670#issuecomment-3550495042)：01.00.1100 會減少 Virtual Stick 抖動。我們已是此版；案例高度相關，但尚未證明本次就是相同缺陷。

## 希望你回答

1. 最可能是 App 指令／控制耦合、Virtual Stick 執行、飛控／定位／避障異常，還是其他原因？請列證據與反證，不預設 DJI 或 App 無責。
2. 哪個最小、低風險的對照最能區分原因？避免只重複已做的「持續發送、改 20 Hz、關閉避障、升至上述韌體」。
3. 如何在保留約 7 秒一圈成果下減少頓挫？若需要改程式，先提出具體理由與最小改動，不以降低速度掩蓋問題。
