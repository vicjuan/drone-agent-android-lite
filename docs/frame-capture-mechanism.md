# 影格證據錄製機制

## 定位

「錄製影格證據」是黑膠帶 detector 的事件式診斷工具，不是連續影片錄影。

操作者沒有按按鈕，不代表沒有錄製。啟動以下循跡模式時，App 會自動 arm recorder：

- `TapeTrackingMode.CIRCULAR`
- `TapeTrackingMode.FIXED_HEADING`

賽車模式實飛使用 `CIRCULAR + FAST`，因此也會自動錄製。Flight Log 應出現：

```text
tape capture armed ...
automatic tape capture armed mode=CIRCULAR
```

畫面上的「錄製影格證據」按鈕提供手動 arm/disarm；自動模式不需要操作者按它。

## 事件錄製流程

Recorder arm 後，detector 處理過的影格先進入記憶體 rolling buffer。偵測器原本有路徑、下一幀首次漏檢時，`BlackTapeDetector` 會觸發：

```kotlin
captureRecorder?.trigger("path-lost")
```

預設保存窗口：

- 事件前 4 幀
- 漏檢當下 1 幀
- 事件後 4 幀

連續漏檢會延長 trailing window，而不是錄成影片。若整場沒有 `path-lost`，即使 recorder 已 arm，也可能不產生任何 capture。

自動錄製在循跡結束後會再保留約 1.5 秒，讓事件後影格寫完，再自動 disarm。若 recorder 原本由操作者手動 arm，循跡模式只沿用它，不應自動關閉。

## Capture 格式

每個 capture 是一個影格目錄，包含：

- 原始全解析度 RGBA 影格
- `blackMask`
- `floorMask`
- `bridgedBlackMask`
- `cleanedBlackMask`
- `capture.txt` metadata
  - detector mode
  - acceptance/rejection
  - detector diagnostics
  - frame timestamp
  - 雲台角度
  - 飛行高度與來源
  - 其他飛行上下文

裝置儲存位置：

```text
/sdcard/Android/data/com.durendal.droneagent.app/files/tape-captures/
```

目錄依序號與事件位置命名：

```text
000000051-path-lost-before
000000052-path-lost-at
000000053-path-lost-after
```

## 容量與計數

`TapeCaptureStore` 預設限制：

- 最多 40 個 capture 目錄
- 最多 256 MiB
- 非同步 writer backlog 最多 8 個 capture

超過容量時會先淘汰最舊資料。Flight Log 的 `saved=80` 表示 session 中曾成功寫入 80 個影格，不表示結束時磁碟仍保留 80 個。

判讀錄製結果時應同時檢查：

```text
saved=...
dropped=...
failed=...
```

## 已有真機使用紀錄

2026-09-04 圓形循跡 Flight Log：

```text
automatic tape capture armed mode=CIRCULAR
...
tape capture disarmed saved=80 dropped=0 failed=0 bytes=101865993
```

該次實飛最後保留序號 `40–79`。我們使用其中：

- capture `52`、`53`：分析中段短暫減速
- capture `68–71`：分析終段持續漏檢

影格與遮罩證明膠帶幾何及雙側板色仍有效，但 coarse floor context 降到 `0.000`，舊版因此錯誤拒絕路徑。

修正後的完整 tracking-session replay 中，capture `52`、`53`、`68–71` 全部恢復為 `FULL_PATH`；capture `51→52` 已轉成永久回歸測試。

## 相關程式

- `MainActivity.kt`：初始化、按鈕、自動 arm/disarm
- `TapeTrackingController.kt`：各模式的 `automaticallyCapturesEvidence` 設定
- `BlackTapeDetector.kt`：產生影格、遮罩、metadata 及 `path-lost` trigger
- `TapeCaptureRecorder.kt`：rolling buffer、事件窗口、非同步寫入
- `TapeCapture.kt`：codec、儲存限制與淘汰
- `TapeCaptureReplayInstrumentedTest.kt`：離線重播

## Agent 判讀規則

> 操作者沒按「錄製影格證據」，不能推論沒有錄製。先檢查 Flight Log 的 `automatic tape capture armed`，再檢查 `saved/dropped/failed` 與裝置上的 `tape-captures` 目錄。這是事件式 detector 證據錄製，不是連續影片錄影。
