# 圓形路徑短終點辨識設計

## 文件目的

本文件定義 `CURVED_OUT_AND_BACK`／圓形循跡模式如何辨識「原本可靠且完整的路徑突然縮短，遠端端點落在畫面內」的終點情境。

本文只記錄設計、測試與真機驗收條件，**目前不修改程式碼**。

核心決策：

> 圓形模式應將「同一路徑由可靠長路徑持續縮短，且可見遠端端點位於畫面內」納入終點候選；候選只會進入既有 `VERIFYING_ENDPOINT` 低速確認，不會直接觸發 180°。

---

## 真機失效案例

來源影片：`screen-20260820-165108.mp4`，片長 `93.909` 秒。

影片最後幾秒，第二片瓦楞紙板被吹走，使原本連續的黑膠帶突然形成實際終點。無人機沒有原地旋轉 180°，而是繼續留在 `TRACKING`。

對應飛行日誌顯示：

- 紙板移動前，`longSideFraction` 曾達約 `1.53–1.63`。
- 紙板移動後，`longSideFraction` 降至約 `0.79–0.95`；最低值相對先前可靠長度約為 `0.79 / 1.63 = 0.48`。
- 最短 observation 的 `bounds.top=0.561`、`bounds.bottom=1.0`，表示可見路徑只剩畫面靠近無人機的下方約 44%。
- 偵測角度曾由 `+53.1°` 跳到 `-43.8°`，但偵測器仍回報 `black tape detected`，信心約 `0.81–0.92`。
- 控制器沒有進入 `VERIFYING_ENDPOINT` 或 `TURNING`；直到影片結束仍為 `TRACKING`，並持續輸出約 `forward=0.03`、`yaw=-6.0`。

根因不是 180° 動作失效，而是圓形模式沒有將「可靠長路徑突然變成畫面內短路徑」視為終點候選。只要偵測器仍產生 observation，現有控制器就可能繼續追蹤。

---

## 現況

### 直線模式

`TapeTrackingController.observe()` 已具備相對長度終點判斷：

1. 以 `longestObservedTapeFraction` 保存本段可靠長度基準。
2. 先由足夠長的 observation 啟用 `endpointQualificationArmed`。
3. 要求 observation 為畫面內端點候選、不是閉環、方向合理。
4. 要求目前長度低於 `longestObservedTapeFraction * ENDPOINT_LENGTH_RATIO`。
5. 連續數幀且持續足夠時間後，進入 `VERIFYING_ENDPOINT`。
6. 低速前探期間確認路徑消失，才進入 `TURNING`。

因此直線模式不是用「短」作絕對判斷，而是結合長度歷史、畫面內端點、拓撲、方向與時間確認。

### 圓形模式

`observe()` 在 `mode.followsCurvedPath` 時直接呼叫 `observeCircularPath()` 並返回，因此不會執行直線模式的相對長度終點邏輯。

現有圓形終點流程主要依賴：

- 先建立 `reliableCircularPathEstablished`。
- `TRACKING` 中連續收到 `observation == null`。
- 或偵測中斷後，新 observation 無法構成合理連續路徑，也不是可信的重新取得路徑。

這適合「膠帶進入墊子下方而完全消失」的既有案例，但無法處理本次「實體路徑突然縮短，偵測器仍能看到近場短段」的案例。

---

## 正確的終點語意

圓形模式的短路徑終點候選必須同時具備以下證據：

1. **可靠長路徑歷史**  
   本段已連續觀察到足夠長、可安全控制的路徑；不能在剛啟動、重新取得或轉向恢復期間用短片段自行建立終點基準。

2. **相對縮短**  
   目前路徑相對本段可靠基準明顯縮短。不能只用固定絕對長度，因高度、透視與曲率會改變畫面中的弧長。

3. **近場連續**  
   路徑近端仍接近畫面底部，anchor、bounds 與前一條已接受路徑具合理連續性。這證明短段仍是無人機正在跟隨的同一路徑，而不是遠處無關黑影。

4. **遠端端點位於畫面內**  
   路徑的遠端確實在畫面內終止，不是被上、左或右邊界裁切。只看 bounding box 長度不足以證明這一點；應優先使用 detector／中心線輸出的 `endpointCandidate` 或等價拓撲資訊。

5. **不是閉環**  
   `closedLoop` observation 不能成為終點候選。

6. **時間一致性**  
   候選必須連續出現數幀並維持最短時間，避免反光、遮蔽、紙板晃動或單幀 segmentation 破碎造成誤轉。

滿足以上條件代表「可信終點候選」，不是立即確認終點。

---

## 狀態機設計

### `TRACKING` → `VERIFYING_ENDPOINT`

在 `observeCircularPath()` 的 `TRACKING` 分支增加短終點候選累積。概念條件如下：

```text
reliable circular path already established
AND current observation exists
AND current observation belongs to the tracked near-field path
AND current observation has an in-frame far terminus
AND current observation is not a closed loop
AND current length <= reliable length baseline * circular endpoint length ratio
AND condition persists for N frames and minimum duration
```

達成後呼叫既有 `beginCircularEndpointVerification(nowNanos)`。

不得在以下狀態建立短終點候選：

- `RECENTERING`
- `ALIGNING_CURVE`
- `REACQUIRING_PATH`
- `RECOVERING_AFTER_TURN`
- `TURNING`

這些狀態本來就可能只看到近場短片段；若允許它們建立終點，起飛後或 180° 後很容易立即再次誤轉。

### `VERIFYING_ENDPOINT`

沿用既有安全流程：

- 清除先前控制量。
- 以 `CIRCULAR_ENDPOINT_PROBE_SPEED_METERS_PER_SECOND` 低速前探。
- 限制 yaw 與橫移，不沿舊命令高速前進。
- 路徑持續縮短或消失並累積足夠 miss 後，才進入 `TURNING`。
- 若可靠完整路徑恢復，取消候選並回到 `TRACKING`。
- 超過既有 verification timeout 時 fail-safe 停止，不得無限前探。

### 候選取消

任一情況發生時，清除短終點候選累積：

- 路徑長度恢復到退出比例以上。
- observation 不再是畫面內端點。
- 近場 anchor／offset 與原路徑不連續。
- observation 變成閉環。
- 進入重新對準或重新取得狀態。
- observation 短暫消失；此時交由既有「可靠路徑消失」流程處理，避免兩組計數器競爭。

---

## 長度基準

不應直接把直線模式的 `longestObservedTapeFraction` 原封不動共用到圓形模式。

建議新增圓形段落專用的可靠長度基準，例如 `longestReliableCircularPathFraction`，僅在以下情況更新：

- phase 為 `TRACKING`。
- observation 已通過圓形路徑連續性判斷。
- observation 品質足以安全前進。
- observation 不是短終點候選。

基準應在以下時機重設：

- 新循跡 session。
- 完成 180° 並成功重新建立回程可靠路徑。
- 明確切換模式。
- 停止循跡。

不能讓上一段去程的尺度永久支配回程，亦不能讓一次異常超長 false positive 抬高整段基準。實作前應以既有 capture replay 比較「本段最大可靠長度」與「近期可靠長度高分位／平滑峰值」；選擇能通過現有正常彎道資料且資料結構最簡單的方案。

---

## 門檻原則

第一版應新增圓形模式專用名稱，不把直線常數硬套到曲線語意：

- `CIRCULAR_ENDPOINT_LENGTH_RATIO`
- `CIRCULAR_ENDPOINT_CONFIRMATION_COUNT`
- `CIRCULAR_ENDPOINT_CONFIRMATION_NANOS`
- 必要時增加遠端端點距畫面邊緣的 margin

初始長度比例可先以本次證據和現有 replay 離線選定。本次失效比例約 `0.48`，所以門檻必須能接納此案例；但不能只根據單次飛行任意固定數值。

直線模式的 `ALIGNMENT_TOLERANCE_DEGREES` 不適用於曲線。圓形終點不能要求整條路徑接近垂直，應改用：

- 有序路徑的近場連續性。
- 遠端端點是否位於畫面內。
- 與上一個已接受 observation 的 bounds／anchor／中心線重疊。
- 現有 `isPlausibleCircularContinuation()` 或其最小必要子條件。

---

## Detector 與控制器契約

控制器不應由 bounding box 猜測真正的拓撲端點。`TapeTrackingObservation.endpointCandidate` 必須具有明確語意：

- 近端由畫面下緣進入或位於無人機近場。
- 遠端中心線節點位於畫面內，與影像邊界保持指定 margin。
- 遠端不是 morphology 人工橋接後產生的虛假末端。
- 閉環沒有遠端終點。

若目前診斷與 capture 尚未保存 `endpointCandidate`，實作時應將它加入診斷及 capture metadata，讓本次失效可以離線重播驗證；不能只靠人工觀看畫面推測控制器當時收到的布林值。

---

## 不採用的方案

### 只要 `longSideFraction` 小於固定值就轉向

拒絕。高度、透視、彎道、剛起步及轉向恢復都可能正常產生短路徑。

### 路徑一變短就立即執行 180°

拒絕。反光、短暫遮蔽及 segmentation 斷裂都可能產生單幀假端點。必須先進入 `VERIFYING_ENDPOINT`。

### 降低 detector 門檻，要求終點後一定輸出 `null`

拒絕。這會把控制器需要的狀態語意推回像素門檻，並增加背景黑影成為路徑的風險。畫面內短端點本身就是有效幾何證據，不應強迫它消失。

### 完全取代現有「可靠路徑消失」終點流程

拒絕。膠帶進入墊子下方時可能沒有可見端點，既有 disappearance 路徑仍是必要且互補的終點來源。

---

## 測試計畫

### JVM 控制器測試

在 `TapeTrackingControllerTest` 增加可觀察狀態轉移測試：

1. **可靠圓形長路徑持續縮短且遠端位於畫面內**  
   建立可靠長路徑，連續輸入同一路徑的短端點 observation；預期 `TRACKING → VERIFYING_ENDPOINT`。

2. **短端點只有單幀**  
   下一幀恢復長路徑；預期維持／回復 `TRACKING`，不可進入 `TURNING`。

3. **短路徑仍被畫面邊緣裁切**  
   即使長度比例低，也不得成為終點候選。

4. **短路徑不是近場同一路徑**  
   anchor、offset 或 overlap 不連續；預期走重新取得／安全停止流程，不得判為終點。

5. **閉環變短**  
   `closedLoop=true` 時不得成為終點。

6. **重新取得期間只看到短片段**  
   不得建立終點候選。

7. **180° 後只看到近場短片段**  
   `RECOVERING_AFTER_TURN` 不得立即再次觸發終點。

8. **短終點經低速前探後消失**  
   預期 `VERIFYING_ENDPOINT → TURNING`，只產生一次 turn action。

9. **短終點在前探時重新延長**  
   預期取消候選、回到 `TRACKING`，不得轉向。

10. **既有完整路徑消失案例**  
    現有 null-observation 終點流程必須保持不變。

### Capture replay

至少重播：

- 本次 `screen-20260820-165108.mp4` 對應、紙板被吹走前後的 capture。
- 正常連續繞圈且沒有終點的 capture。
- 反光造成短暫路徑斷裂的 capture。
- 180° 後重新取得近場短段的 capture。
- 膠帶進入墊子下方、沒有畫面內端點的既有成功案例。

必要驗證：

- 本次案例能進入 `VERIFYING_ENDPOINT`。
- 正常繞圈不增加誤判終點。
- glare／shadow／短 fragment 不觸發 `TURNING`。
- 既有 disappearance 終點仍能觸發。

### 真機驗收

回到辦公室後分階段進行：

1. 固定紙板，正常繞圈多圈；不得誤判終點。
2. 固定製作畫面內短終點；應先低速確認，再原地旋轉 180°。
3. 循跡中移走第二片紙板，重現本次突發縮短；應進入 `VERIFYING_ENDPOINT`，不得繼續高速追蹤背景。
4. 短暫遮住遠端後移開；不得誤轉。
5. 驗證 180° 後能重新取得回程路徑，且不會因近場短片段立即再次旋轉。

每次保留：

- 完整 `flight-log.txt`
- 對應 `tape-captures`
- 螢幕錄影
- APK SHA-256
- 測試時間與實際紙板配置

---

## 實作順序

後續實作時依序進行：

1. 先用 capture 確認本次短路徑 observation 的 `endpointCandidate`、`closedLoop`、bounds、中心線端點與品質欄位。
2. 若 capture 缺少必要欄位，先補診斷／序列化契約與 replay 讀取。
3. 在圓形 `TRACKING` 增加可靠長度基準及短終點候選累積。
4. 候選成立後只接到既有 `beginCircularEndpointVerification()`。
5. 保持既有 disappearance 終點及 post-turn recovery 行為。
6. 執行 targeted JVM tests、完整既有 controller tests、capture replay。
7. 最後才進行真機低速驗證。

---

## 完成條件

此修改只有在以下條件全部成立時才算完成：

- 紙板被吹走的短終點 replay 能進入 `VERIFYING_ENDPOINT`。
- 低速確認後才產生 180° action，不由單幀直接轉向。
- 正常圓形路徑長度波動不會誤判終點。
- glare、shadow、背景暗邊與重新取得短片段不會觸發 180°。
- 沒有可見端點的「可靠路徑消失」案例仍正常運作。
- 180° 後不會立即再次判定終點。
- 真機日誌可明確看出候選來源、長度基準、比例、端點拓撲、確認與取消原因。
