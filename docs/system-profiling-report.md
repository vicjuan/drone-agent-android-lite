# 系統 Profiling 報告：軟體與硬體耗時

## 第一輪量測結果

```text
相機影格
  │ 77.9 ms（整包視覺處理）
  ▼
OpenCV 偵測完成
  │ 59 ms（回呼、UI 執行緒與固定控制 tick）
  ▼
循跡控制器更新速度與轉向命令
  │ 25 ms（等待下一個 20 Hz 搖桿幀）
  ▼
搖桿幀進入 MSDK
  │ 約 380 ms（目前只能由命令與地速遙測的相關性估計）
  ▼
飛機回報地速反映命令變化
```

第一輪只能得到四個大區段。77.9 ms 不只是單一 OpenCV 函式，而是包含影像縮放、色彩轉換、二值化、地板與陰影判斷、形態學、輪廓搜尋、中心線評分及資源釋放。380 ms 也不是純機體反應時間。

## 第二輪細分量測

程式已加入以下時間點；數值必須由下一次實飛產生，不能用第一輪資料回推：

| 區段 | 新增量測欄位 |
|---|---|
| 影格進入 | 複製與排隊 `queueMs` |
| OpenCV 前處理 | Mat 建立、縮放、RGB／灰階及模糊 `preprocessMs` |
| 黑色分割 | Otsu、門檻修正及明暗分離 `thresholdMs` |
| 地板判斷 | Lab、陰影移除及 flood fill `floorContextMs` |
| 路徑形成 | close／open 及輪廓搜尋 `morphologyContoursMs` |
| 路徑判定 | 候選輪廓、中心線及幾何評分 `candidateMs` |
| 收尾 | OpenCV Mat／輪廓釋放 `cleanupMs`；未歸類成本 `otherMs` |
| 偵測至控制 | detector 回呼、UI 排隊、等待控制 tick、控制器計算、安全判斷與命令更新 |
| 控制至搖桿幀 | `commandToFrameMs` |
| MSDK 呼叫 | `sendVirtualStickAdvancedParam` 同步呼叫耗時 |
| 地速遙測 | `KeyAircraftVelocity` 每筆 callback 間隔 `intervalMs` |

這能直接回答 77.9 ms 究竟花在哪一個演算法階段，也能確認 59 ms 是 UI 排隊、固定 tick，還是其他控制邏輯。

## 380 ms 能拆到什麼程度

App 現在能分開量到：

1. 控制命令更新至下一個搖桿幀；
2. App 呼叫 MSDK 所花的時間；
3. MSDK 呼叫返回至地速 callback 的剩餘時間；
4. 地速 callback 的取樣間隔，因此能估計遙測取樣造成的時間不確定性。

但「RC 上行傳輸、飛控收到命令、馬達／機體開始反應、飛機產生遙測、RC 下行傳輸」沒有共同且公開的 MSDK 時間戳，無法只靠 App 再細分。Mini 4 Pro 可由 DJI Assistant 2 匯出飛行器紀錄，但 DJI 沒有提供這類消費級飛行器紀錄的公開欄位格式與時鐘定義；除非 DJI Support 回傳可解析資料，專案不能把匯出的檔案當成可自行分析的證據。因此目前可獨立執行的物理反應量測方案是同步高速攝影，不能把 380 ms 任意分攤。

> **地速如何判定？** App 透過 DJI MSDK 的 `KeyAircraftVelocity` 取得飛機回報的水平速度，以 $\sqrt{x^2+y^2}$ 計算地速，再比較搖桿命令與地速兩條時間序列。約 380 ms 是兩者相關性最高的位移量，不是馬達起轉的直接時間戳。

## 已備妥的同步高速攝影實驗

Pixel 8 Pro 上的 App 已加入「硬體延遲脈衝・0.50 m/s」。取得 MSDK 控制權後，程式固定執行：

```text
懸停 3 秒
→ 前進 0.50 m/s 1 秒
→ 歸零 2 秒
→ 後退 0.50 m/s 1 秒
→ 歸零 2 秒
→ 共 10 組
```

每個非零速度脈衝開始時，畫面中央的 `SYNC` 方塊由黑轉白；歸零時轉黑。App 同時在 `flight-profile.tsv` 寫入 `latency_test_armed`、`latency_test_command` 與 `latency_test_stop`。既有 `virtual_stick` 列會提供 `commandToFrameMs`、`sendCallMs`，`velocity` 列則提供地速與 callback `intervalMs`。因此同一份 trace 可直接對齊：

1. 同步標記／速度命令請求；
2. 下一個 20 Hz Virtual Stick 幀；
3. MSDK 同步呼叫返回；
4. `KeyAircraftVelocity` 遙測反映。

實驗必須在空曠場地、BRAKE 避障確認成功且電量非危急時啟動。任一障礙剎車、近距離障礙、斷線、飛行結束、低電量、實體 RC 接管或畫面搖桿輸入都會立即歸零；完成後自動交回 RC。若要取得「機體首次可見移動」時間，第二支相機需以 240 fps 同時拍到飛機和 Pixel 畫面的 `SYNC` 方塊。畫面實際變白仍受 Pixel 顯示器下一次 vsync 限制，必須把這一個顯示幀的不確定性列入結果。

## 結論

第一輪平均端到端耗時約 **542 ms（0.54 秒）**，但目前只能把它視為基準，不應據此直接決定優化項目。第二輪已把可由 App 觀測的軟體階段拆開；完成下一次相同路線實飛後，應依各階段 P50、P95 與占比選擇優化點。

380 ms 的內部傳輸與物理反應仍是觀測邊界。App 可以縮小其估計誤差，但不能宣稱已分離飛控、機體與遙測各自耗時。
