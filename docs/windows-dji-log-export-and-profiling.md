# Windows：Mini 4 Pro 紀錄匯出與 Profiling 重分析

## 目的

在 Windows 電腦完成兩件事：

1. 從 DJI Mini 4 Pro 飛行器本體匯出原始飛行紀錄，確認檔案是否能解析。
2. 從 Pixel 8 Pro 取得本專案 App 的 `flight-profile.tsv`，獨立重做 timestamp profiling，驗證各階段耗時與約 380 ms 的「命令送出至地速遙測反映」結果。

> App trace 可以獨立重做 profiling，因為同一個 App session 內的 `monotonicNanos`、`frameNanos` 與各事件時間都使用 Pixel 的單調時鐘 `System.nanoTime()`。DJI 飛行器紀錄使用另一個時鐘；除非成功解析並完成事件對時，不能直接與 App timestamp 相減。

## 重要限制

- 新版為每次 `MainActivity` session 建立唯一的 `flight-profile-<wall-clock-ms>-<monotonic-nanos>.tsv`，避免 Activity 重建時覆寫或共用 writer。以 Flight Log 的 `profiling start session=... trace=...` 選取該次實飛檔案；不要只憑檔案修改時間猜測。
- 舊 trace 只能重算當時已記錄的大區段，不能事後產生不存在的 OpenCV 子階段時間戳。
- 新版 trace 才包含 `preprocessMs`、`thresholdMs`、`floorContextMs`、`morphologyContoursMs`、`candidateMs`、`cleanupMs`、`otherMs`、`visionToCallbackMs`、`uiToTickMs`、`decisionMs`、`decisionToCommandMs`、`commandToFrameMs` 與 `intervalMs`。
- DJI Assistant 2 能匯出紀錄，不代表紀錄可由第三方工具解析。不能猜測欄位、解密內容或杜撰飛控內部耗時。

## 是否需要 Git 程式碼

重算既有 `drone-latest-flight-profile.tsv` 與匯出 DJI 紀錄時，**不需要 Git repo**。必要輸入只有本文件、原始 `.tsv` 與對應 `flight-log.txt`。程式碼只在下列情況才需要：

- 要稽核每個 trace 欄位由哪一行 App 程式產生；
- 要重建或修改 Android App；
- 要分析新版細分 trace，並核對產生該 trace 的確切程式版本。

舊 trace 必須依其實際欄位分析，不能拿最新版程式新增的欄位回填。此工作站目前另有尚未提交的變更，因此單純 `git clone` 遠端 `main` 也不等於產生新版 trace 的完整程式；若之後需要重建 App，必須另外交付確切 commit 加工作樹變更，或先建立可識別的正式 commit。

## 一、建立證據資料夾

以 PowerShell 執行：

```powershell
$Stamp = Get-Date -Format "yyyyMMdd-HHmmss"
$Root = "$env:USERPROFILE\Desktop\mini4-profile-$Stamp"
New-Item -ItemType Directory -Force `
  "$Root\app", `
  "$Root\dji-aircraft-original", `
  "$Root\analysis", `
  "$Root\screenshots" | Out-Null
$Root
```

後續所有原始檔與分析結果都放在 `$Root`。不要修改 `dji-aircraft-original` 內的檔案。

## 二、安裝 Windows 版 DJI Assistant 2

使用 **DJI Assistant 2（Consumer Drones Series）**，不要使用 Enterprise、Mavic 或 FPV 版本。

- 官方下載頁：<https://www.dji.com/downloads/softwares/dji-assistant-2-consumer-drones-series>
- 2026-04-23 官方 Windows 2.1.40：<https://dl.djicdn.com/downloads/dji_assistant/20260423/DJI%20Assistant%202(Consumer%20Drones%20Series)%202.1.40.exe>

下載後先驗證 Windows 簽章：

```powershell
$Installer = Get-ChildItem "$env:USERPROFILE\Downloads" -Filter "*DJI*Assistant*2*.exe" |
  Sort-Object LastWriteTime -Descending |
  Select-Object -First 1
Get-AuthenticodeSignature $Installer.FullName |
  Format-List Status,StatusMessage,SignerCertificate
```

只有 `Status` 為 `Valid`，且簽署者確實是 DJI 時才繼續。否則停止安裝並記錄結果。

安裝完成後，記錄：

- DJI Assistant 2 完整版本；
- Windows 版本；
- Mini 4 Pro 韌體版本；
- 匯出日期、時區及預計實飛時間。

## 三、匯出 Mini 4 Pro 飛行器紀錄

1. 拆下螺旋槳或確保飛行器不可能啟動，電池保有足夠電量。
2. 用 USB-C 將 **Mini 4 Pro 飛行器本體直接連到 Windows 電腦**；不要連 RC-N3 代替飛行器。
3. 開啟飛行器電源。
4. 啟動 DJI Assistant 2 並登入 DJI 帳號。
5. 選擇辨識到的 Mini 4 Pro。
6. 開啟 **Log Export**。
7. 選取涵蓋目標實飛前後至少各 10 分鐘的紀錄。
8. 選擇 **Save to Local**，直接輸出到 `$Root\dji-aircraft-original`。
9. 截圖裝置資訊、韌體版本、紀錄清單、所選時間與匯出完成畫面，放入 `$Root\screenshots`。
10. 匯出後不要用編輯器開啟並另存原始檔；分析時使用複本。

產生原始檔清單與 SHA-256：

```powershell
Get-ChildItem "$Root\dji-aircraft-original" -Recurse -File |
  Sort-Object FullName |
  ForEach-Object {
    $Hash = Get-FileHash $_.FullName -Algorithm SHA256
    [PSCustomObject]@{
      RelativePath = $_.FullName.Substring($Root.Length + 1)
      Bytes        = $_.Length
      Modified     = $_.LastWriteTime.ToString("o")
      SHA256       = $Hash.Hash
    }
  } | Export-Csv "$Root\dji-aircraft-files.csv" -NoTypeInformation -Encoding UTF8
```

### DJI 紀錄判定

依序檢查：

1. 先保留原始目錄及 hash。
2. 複製一份到 `$Root\analysis\dji-working-copy` 再解壓縮或檢查檔頭。
3. 記錄副檔名、檔案大小、magic bytes、是否為 ZIP／SQLite／XOR 或其他容器。
4. 可用 DatCon／CsvView 嘗試開啟，但必須記錄工具版本、輸入檔及完整錯誤。
5. 工具無法辨識即判定「目前不可自行解析」，不得把空白輸出或猜測欄位當成成功。
6. 若紀錄加密或格式不公開，保留原檔，後續交由 DJI Support，要求提供欄位定義、取樣率、timestamp 時鐘來源及可讀 CSV。

## 四、從 Pixel 8 Pro 複製 App trace

安裝 Android SDK Platform Tools，開啟 Pixel 的 USB debugging，使用原飛行手機連線。

```powershell
adb devices -l
adb pull "/sdcard/Android/data/com.durendal.droneagent.app/files/flight-log.txt" "$Root\app\flight-log.txt"
adb shell "ls -lt /sdcard/Android/data/com.durendal.droneagent.app/files/flight-profile-*.tsv"
# 將 <session-id> 換成 flight-log.txt 中該次實飛的 profiling start session。
$Profile = "flight-profile-<session-id>.tsv"
adb pull "/sdcard/Android/data/com.durendal.droneagent.app/files/$Profile" "$Root\app\$Profile"
```

若路徑不存在，先執行：

```powershell
adb shell "find /sdcard/Android/data/com.durendal.droneagent.app/files -maxdepth 2 -type f"
```

複製完成後計算 hash：

```powershell
Get-FileHash "$Root\app\$Profile","$Root\app\flight-log.txt" -Algorithm SHA256 |
  Format-Table Path,Hash -AutoSize |
  Out-File "$Root\app\SHA256.txt" -Encoding UTF8
```

確認所選 `flight-profile-<session-id>.tsv` 第一列必須是：

```text
elapsedMs	monotonicNanos	event	frameNanos	durationMs	details
```

## 五、App timestamp 的正確語意

| 欄位 | 語意 |
|---|---|
| `elapsedMs` | 相對本次 App profiling 啟動時間；毫秒整數，只適合閱讀 |
| `monotonicNanos` | Pixel 開機後單調時鐘；精確分析以此欄為準 |
| `event` | `vision`、`detection_ui`、`control`、`virtual_stick`、`velocity` 等事件 |
| `frameNanos` | 影格進入 App 的單調時間；用來配對同一影格的 vision／UI／control |
| `durationMs` | 該事件自行量到的區段耗時；舊版可能只保留整數毫秒 |
| `details` | 空白分隔的 `name=value` 欄位；新版細分耗時主要在這裡 |

規則：

- 只能在同一個 `session_start` 所屬 session 內相減。
- 不得把 `System.nanoTime()` 當成日期時間或直接對 DJI log 的時間戳。
- 精確配對優先使用 `frameNanos`，不要用「兩列剛好相鄰」代替。
- `control` 只使用 `newFrame=true` 的列計算新影格到首次控制命令延遲。
- 遇到 `null`、缺欄位或舊版 trace，要標成不可用，不得補零。

## 六、重做 App profiling

使用 Python 3.11 以上。可使用 `pandas`、`numpy`、`matplotlib`；分析程式與套件版本一併保留在 `$Root\analysis`。

### 6.1 完整性檢查

先輸出：

- 檔案 SHA-256、列數及檔案大小；
- `session_start`／`session_end` 數量；
- 裝置、Android、MSDK、OpenCV、visionHz、controlHz、stickHz；
- 每種 event 的筆數與實際頻率；
- `monotonicNanos` 是否單調；
- 缺欄位、重複 frame、負值 duration、超過 1 秒的時間缺口；
- 飛行前後 trace 是否完整。

若沒有 `session_end`，須註明 App 可能非正常關閉；檔案每秒 flush，末段仍可能不完整。

### 6.2 統計格式

每項耗時都輸出：

```text
樣本數 N、平均、P50、P95、P99、最大值
```

同時分組比較：

- `detected=true` 與 `detected=false`；
- 不同 tracking `mode`／`phase`；
- 實飛移動區間與起飛前／降落後靜止區間。

平均值不能取代 P95；所有百分位必須註明樣本數。

### 6.3 OpenCV 細分

對每一列 `event=vision` 分析：

- `queueMs`
- `preprocessMs`
- `thresholdMs`
- `floorContextMs`
- `morphologyContoursMs`
- `candidateMs`
- `cleanupMs`
- `otherMs`
- `totalMs`

驗證：

```text
queueMs + preprocessMs + thresholdMs + floorContextMs
+ morphologyContoursMs + candidateMs + cleanupMs + otherMs
≈ totalMs
```

允許浮點誤差，但若系統性缺少數毫秒，必須列為未歸類成本。輸出各階段占 vision processing 的比例，以及 detected／miss 的差異。

舊 trace 沒有上述子欄位時，只能重算：

```text
OpenCV processing ≈ totalMs - queueMs
```

並明確標記「無法回溯拆分」。

### 6.4 偵測完成至控制命令

以相同 `frameNanos` 配對：

```text
vision → detection_ui → control(newFrame=true)
```

分析：

- `visionToCallbackMs`
- `detection_ui.durationMs`：callback 等待 UI 執行緒
- `uiToTickMs`
- `preDecisionMs`
- `decisionMs`
- `decisionToCommandMs`
- `control.monotonicNanos - vision.monotonicNanos`：完整偵測完成至命令更新

若 `uiToTickMs` 占主要部分，才支持「固定 10 Hz tick 是主要延遲」；若主要時間在 UI queue 或 decision，結論必須跟隨量測結果修改。

### 6.5 控制命令至 MSDK 搖桿幀

分析 `event=virtual_stick`：

- `commandToFrameMs`
- `sendCallMs`：`sendVirtualStickAdvancedParam()` 同步呼叫耗時，保留小數毫秒；`durationMs` 在舊格式可能只有整數，不用於次毫秒統計
- 實際送出頻率及相鄰搖桿幀 interval
- `success=false` 次數

舊 trace 沒有 `commandToFrameMs` 時，可以用每筆 `control` 尋找其後第一筆命令值相符的 `virtual_stick` 作為估計，但必須另外標示為配對推估，不能與新版直接量測混在一起。

### 6.6 地速遙測與約 380 ms 重算

`velocity` 的地速為：

```text
groundSpeed = sqrt(x^2 + y^2)
```

先分析：

- velocity callback 實際頻率；
- 新版 `intervalMs` 的 P50／P95／最大值；
- hover 時地速雜訊與量化；
- callback 是否有長時間缺口。

再以 `virtual_stick` 的水平命令大小：

```text
commandSpeed = sqrt(forward^2 + right^2)
```

與 `velocity.groundSpeed` 建立時間序列。使用零階保持重採樣到固定時間格，測試 0–1000 ms 的 lag，至少輸出：

1. 每個 lag 的相關係數曲線；
2. 最大相關性的 lag；
3. 峰值附近的寬度，而非只報單一數字；
4. 以速度命令明顯改變事件進行 step-response 交叉驗證；
5. 不同實飛區段的結果是否一致。

報告用語必須是：

> 命令送出至飛機回報地速反映命令的遙測延遲。

不得寫成馬達起轉時間或純機體反應時間。若 correlation 峰值很寬、不同區段不一致，應回報範圍與不確定性，不能強行保留 380 ms。

## 七、若 DJI 紀錄可以解析：跨時鐘對齊

不要用檔案修改時間或牆鐘直接對齊。應在 DJI 與 App 都有的訊號中尋找多個明顯事件，例如：

- 水平速度開始／停止；
- 明顯加減速；
- 大角度轉向；
- 起飛及降落狀態轉換。

至少選擇分散在整段飛行中的多個 anchor，擬合：

```text
t_app = a × t_dji + b
```

其中 `b` 是時鐘偏移，`a` 用來檢查時鐘漂移。輸出每個 anchor 的 residual、P50／P95 residual 與最大值。

只能推論大於對時誤差的階段。若對時 residual P95 為 30 ms，就不能宣稱已可靠拆出 10 ms 的飛控步驟。

若無法取得 DJI timestamp 定義或無法穩定對時，停止跨資料源相減，只保留 App 內部 profiling 與 DJI 原始檔。

## 八、交付物

Windows 電腦最後應產生：

```text
mini4-profile-YYYYMMDD-HHMMSS/
├─ app/
│  ├─ flight-profile.tsv
│  ├─ flight-log.txt
│  └─ SHA256.txt
├─ dji-aircraft-original/
├─ analysis/
│  ├─ analyze_flight_profile.py
│  ├─ requirements.txt
│  ├─ stage-summary.csv
│  ├─ event-counts.csv
│  ├─ lag-correlation.csv
│  ├─ profiling-analysis.md
│  └─ plots/
├─ screenshots/
└─ dji-aircraft-files.csv
```

`profiling-analysis.md` 必須清楚分成：

1. **Measured**：trace 直接量到的數值；
2. **Derived**：由 timestamp 配對或相關性計算得到；
3. **Inferred**：尚未由實驗直接證明；
4. 資料缺口與不能回答的問題；
5. 對原本 77.9、59、25、380、542 ms 的獨立重算結果；
6. 新版細分 profiling 的 P50／P95 及下一個應優化的單一瓶頸。

## 九、完成條件

只有同時符合以下條件才算完成：

- DJI 原始匯出檔、版本資訊及 SHA-256 已保存；
- App trace 在重新開啟 App 前成功複製；
- 所有計算可由保存的 Python 程式重跑；
- 報告沒有把地速 callback 說成馬達起轉；
- 舊 trace 缺少的欄位沒有被臆測補值；
- 380 ms 已被獨立重算，或明確說明為何資料不足；
- DJI 紀錄無法解析時有保留完整錯誤與原始檔，而不是宣稱失敗檔案沒有價值。
