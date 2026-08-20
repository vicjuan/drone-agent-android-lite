# 中心線 shadow 管線：階段一、二成果

**日期**：2026-08-20
**對應**：[`tape-vision-pipeline-redesign.md`](tape-vision-pipeline-redesign.md) 階段二第 3 步
**commit**：`4bdd372`（前置：`bb63359`）
**驗證裝置**：Pixel 8 Pro / Android 16

---

## 一、shadow 接法

新中心線與現行 estimator 並行處理**同一份候選遮罩**，只寫 flight log，不影響回傳的 `TapeDetection`。

- winner 選定後以它自己的 `contourIndex` 重畫遮罩，**不重用** `scoreCandidate` 留下的 `candidateMaskBytes`——那是「最後一個被評分的候選」而非勝出者。
- segmentation、morphology、floor mask、候選評分與勝者判定全部沿用現行路徑，沒有第二套。
- 每幀一行 flat log，帶 `seq` 可與 capture 的 `frame.sequence` 對齊重播。

新增 `CenterlineMeasurement`：有序中心線 → anchor / 弧長前視 / 近場切線角 / 曲率 / 弧長 / 中位寬度。刻意沿用 estimator 既有的弧長規則（0.40 弧長比例、上限 0.40 短邊），否則比的是兩套前視慣例而不是幾何。

`lookahead == null` 即「這條路徑不足以瞄準」——是量測自身的判定，不是待填缺值。這正是階段三 `FULL_PATH` / `NEAR_FIELD_ONLY` 的分界，階段三不需另立判準。

## 二、七個場景結果（全通過）

| 場景 | 舊 estimator | 新中心線 |
|---|---|---|
| straight / curved / glare-break / dark-floor / endpoint / lose-then-reacquire | 成功 | 全部 `FULL_PATH` |
| no-path | 無輸出 | 無輸出 |

**一致性**：`dAnchorX ≤ 0.002`、`dAngle 0.0`、`dLookaheadX ≤ 0.008`、`dLookaheadY ≤ 0.046`

**不可比的欄位：曲率。** 舊的是 estimator 自有定義，新的是近場與前視切線的軸向夾角；同一幀舊 `36.9°` / 新 `20.6°`。這不是誤差，是兩個不同的量，切換時須一併決定採用哪個定義。

## 三、效能

```
shadow 單獨   p50 47.3   p95 54.4 ms
整幀          p50 126.9  p95 150.8 ms    ← 250 ms intake interval 的 60%
冷啟第一幀    301.5 ms                    ← 每次 app 啟動一次，超過單幀間隔
```

**修正我上一輪的數字。** 先前報中心線「p50 11 ms、約佔預算 5%」，那是連續 40 次迭代讓 CPU 升頻的結果。以 4 Hz 節流、幀間有閒置的實際節奏量是 **47 ms**，差 4 倍。結論方向未變（整幀仍在 60%），但當時「效能上階段三沒有阻礙」的支撐比我說的弱；正確的支撐是上面這組整幀數字。

附帶：上一輪看似 polish 的壓縮修正（`store.save` 758 → 98 ms）其實是核心路徑前提。在那之前寫入端 1.32 captures/s 對 detector 4 frames/s，錄製必然丟棄，「用真實 capture 做 replay」拿不到完整素材。

## 四、過程中我自己弄壞的兩處

**1. 重用 `SegmentationMask` 物件 → extractor 永遠看到空遮罩。**
`tapePixelCount` 是建構時算好的 eager val，重用實例等於凍結在建構當下的計數。七個場景全變 `NO_CENTERLINE`。抓到它的不是突變驗證，是場景測試的功能斷言。

**2. 效能斷言用了我在量測前自己編的 40 ms 子預算。**
要求是完整管線對 250 ms 並保留餘裕，不是某個子預算。已改為量整幀、上限 75%。這看起來像為了讓測試變綠而移動目標，所以講明：新斷言在整幀超過 187.5 ms 時仍會紅，而 40 ms 從頭到尾沒有依據。

## 五、尚未成立

場景畫面**全是合成的**。真實地板輪廓數多得多，所以每個 detect 數字都是**下限**。

「使用已保存的真實 capture 做 replay」目前做不到——一筆真實 capture 都還沒有。replay 進入點、格式、frame identifier 對齊皆已就緒，第一次帶著錄製起飛即可把同一套 harness 指向真機影格。

## 六、下一步的三個答案

| 問題 | 答案 |
|---|---|
| 哪個真實輸入證明這一步成立 | **目前沒有**。合成七場景證明幾何在我們控制的形狀上成立，不是在真實地板上成立。 |
| 哪個測試會在它失效時變紅 | `TapeShadowPipelineInstrumentedTest`：新中心線在任一舊 estimator 成功的場景失敗即紅，整幀超過 187.5 ms 亦紅。今天已紅過兩次，各逼出一個真缺陷。 |
| 哪個 build 可能被裝上飛機 | 分支上每一個。shadow 只寫 log，飛控不變式維持：無 producer 輸出非 `FULL_PATH`，控制器亦無 gating，兩邊語義一致。 |

## 七、一個待判斷的順序問題

階段三要定 `NEAR_FIELD_ONLY` / `LOST` 的門檻（近場可信度、最短弧長、分支歧義）。用合成畫面定值等於再一次讓設計跑在證據前面，而這批門檻直接決定飛機何時停止平移。

**建議**：階段三之前插一次錄製飛行。帶現有 build 起飛並開啟錄製，取回真實 capture 後再定門檻，同時滿足「用真實 capture 做 replay」的要求。

**替代**：若順序不該打斷，就以合成畫面定初值，階段五實飛前再以真機 capture 重校——代價是門檻會有一段時間沒有真憑據。

---

**驗證**：單元測試 127 / 0 failures · 實機 `connectedDebugAndroidTest` 30 / 0 failures · `lintDebug` 通過
