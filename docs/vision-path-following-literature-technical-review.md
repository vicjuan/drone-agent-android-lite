# 《視覺循線飛行的相關論文調查》技術審閱與修正

**日期**：2026-08-18  
**審閱對象**：[`vision-path-following-literature.md`](vision-path-following-literature.md)  
**審閱範圍**：論文主張、數學單位、目前程式碼資料流，以及建議改動是否能安全落地。

---

## 一、審閱結論

原文件成功找到幾個相關研究方向：下視相機循線、image-based feedback、Pure Pursuit、vector field、illumination-robust segmentation，以及參數空間追蹤。這些文獻適合用來建立後續研究地圖。

但原文件把「概念相似」多次寫成「目前實作已等價於該方法」，並提出一項量綱不成立的控制改動：直接把 `TapePathEstimate.curvatureDegrees` 用於 `ω = v / R` 曲率前饋。這不能照做。現有值既不是 `1 / R`，也沒有方向符號或公尺尺度，而且目前根本沒有傳入控制器。

因此：

- **可保留**：文獻清單、研究方向、現有角度／偏移回授與相關研究的概念對照。
- **必須修正**：曲率前饋、look-ahead 單位、IBVS 定性、Kalman filter 能取代 endpoint counters、兩篇核心論文的實驗證據範圍。
- **不能據此直接改程式**：原文件第七節的四項建議都還不是可執行規格。

---

## 二、正確且值得保留的部分

### 1. 現有控制器確實使用兩個重要影像特徵

`TapeTrackingObservation` 包含：

- `angleFromVerticalDegrees`：路徑方向相對 image-up 的角度。
- `nearFieldOffsetFraction`：近場 anchor 相對畫面中心的橫向偏移。

這與循線文獻常見的 orientation error 和 lateral displacement error 在概念上相符。原文件拿 Brandão et al. 的位置／方向誤差作對照，方向正確。

### 2. `axialExponentialAverage()` 正確處理無方向直線的角度接縫

目前直線角度定義在 `[-90°, 90°]`，而同一條無方向直線相差 `180°` 應視為相同。函式把差值 wrap 到 `[-90°, 90°]` 再做 EMA，數學語意合理。這項正面評價可以保留。

### 3. 固定前視尺度的確有曲線追蹤取捨

前視太短通常會放大量測雜訊並增加振盪，太長則可能造成曲線內切。這是合理的研究問題；目前圓形模式以較低速度、不同 yaw gain 與安全閘門控制行為，也確實反映直線與曲線的不同需求。

但這只能支持「值得研究 adaptive look-ahead」，不能直接推出「目前九個 `CIRCULAR_*` 常數都因 fixed look-ahead 而存在」。九個常數還包含最大 yaw rate、橫移速度、允許移動的角度／偏移安全界線及兩段前進速度，不是單靠 adaptive look-ahead 就能刪除。

### 4. 偵測文獻方向合理

Otsu、亮度上限、色彩平衡、floor context、tracking gate 與 crop-row／power-line detection 的問題類型相近。這些文獻適合用來比較 illumination robustness 與 temporal gating；只是不能在未逐篇核對演算法和資料集前，宣稱現行閘門就是某篇方法的簡化等價物。

---

## 三、必須修正的技術問題

### 1. `curvatureDegrees` 不是曲率，不能直接做 `ω = v / R` 前饋

原文件寫：

> `TapePathEstimate.curvatureDegrees` 已經算出來了，但目前完全沒有用在控制上，這是最有性價比的改進點。

這個結論錯誤。

目前 `TapePathDirectionEstimator.curvatureDegrees()` 的計算是：

1. 在 sampled centerline 近端與遠端各取一條局部切線。
2. 算兩條切線的角度。
3. 回傳兩者的**絕對角度差**，範圍約為 `[0°, 90°]`。

也就是：

\[
C_{\mathrm{current}} = \left|\psi_{\mathrm{far}} - \psi_{\mathrm{near}}\right|
\]

它是「目前可見片段的總轉角」，不是幾何曲率：

\[
\kappa = \frac{d\psi}{ds} = \frac{1}{R}
\]

兩者差異如下：

| 性質 | 現有 `curvatureDegrees` | 前饋需要的 `κ` |
| --- | --- | --- |
| 單位 | degree | `m⁻¹` |
| 是否除以弧長 | 否 | 是 |
| 左／右彎符號 | 沒有，程式使用 `abs` | 必須有 signed curvature，或由已知方向另行提供 |
| 是否具地面尺度 | 否，來源為 image pixels | 必須是地面公尺，或由已知半徑直接提供 |
| 對可見片段長度的敏感性 | 高 | 理想圓弧上應近似固定 |
| 對飛行高度／相機投影的敏感性 | 高 | metric curvature 經正確轉換後不應隨高度任意變動 |

正確的 yaw feedforward 是：

\[
\omega_{\mathrm{rad/s}} = v_{\mathrm{m/s}}\,\kappa_{\mathrm{m}^{-1}}
\]

若 DJI 命令使用 degree/s：

\[
\omega_{\mathrm{deg/s}} = \operatorname{radToDeg}(v\kappa)
\]

直接把 degree 塞進 `v / R` 會造成量綱錯誤。

#### 還有一個資料流事實

`curvatureDegrees` 目前只留在 detector 的 `Candidate` 和 diagnostics：

- `TapeDetection` 沒有 curvature 欄位。
- `TapeTrackingObservation` 沒有 curvature 欄位。
- `TapeTrackingController` 收不到 curvature。

所以這也不是「接一條線就完成」的改動。至少要先定義量測語意、座標轉換、有效性條件、資料模型、濾波、失效策略與測試。

#### 可行的兩條路

**A. 實體圓圈半徑固定且已知**

設定 `radiusMeters` 與方向，直接用 `v / radiusMeters` 做前饋。影像角度與偏移只做修正。這才是低工程量方案。

**B. 半徑未知，需要從影像估測**

先取得 signed image curvature，再透過相機內參、飛行高度、姿態與地面 homography 轉成 metric curvature。若無法穩定取得地面尺度，就不能宣稱算出了 `1 / R`。

---

### 2. `LOOKAHEAD_ARC_FRACTION = 0.40` 不是文獻中的 look-ahead distance

目前 estimator 使用：

```kotlin
val targetArc = tracedArcLengthPixels * LOOKAHEAD_ARC_FRACTION
```

`0.40` 的語意是「本幀已追蹤影像弧長的 40%」。它是 dimensionless fraction；被乘數是 pixels。

原文件建議：

```text
L = k * v + L_min
```

這裡的 `L` 通常是空間距離。若 `v` 是 `m/s`，`L` 應是 meters。不能把 meter 直接取代 `0.40`，也不能和 pixel arc length 混算。

在目前架構下要做 adaptive look-ahead，至少要選定一種明確設計：

1. **Metric look-ahead**：先把 centerline 投影到地面座標，再以 meters 取 target point。
2. **Image-space look-ahead**：定義 pixels 或 normalized-frame distance，並明確建模它和速度、高度、相機 FOV 的關係。
3. **Fractional look-ahead**：仍輸出 fraction，但以有界函式從速度、可見弧長與 confidence 推導；這不等價於文獻中的 metric `L`，必須如實命名。

此外，`LOOKAHEAD_ARC_FRACTION` 位於 detector，而速度由 controller 決定。若要讓前視距離依速度改變，就需要明確的 detector/controller configuration flow，不能讓兩層互相偷偷讀取狀態。

---

### 3. Terlizzi et al. 與目前控制器只有概念相似，不是同一種控制律

原文件說該篇對目前 look-ahead 設計最有直接參考價值，並暗示 look-ahead 選擇有理論依據而非經驗值。核對原文後，需要加上以下限制：

- 論文使用 image-space annular mask 找 Virtual Target Point；表格中的 inner/outer radius 是固定 `26/28 pixels`。
- 論文明確假設固定高度 `1 m`。
- 路徑規劃器參數 `a`、`b` 是 heuristic tuning。
- 作者利用多旋翼可側向平移的特性，明確表示 path follower **不控制 heading**。
- 目前 repo 則用 body-forward velocity、yaw-to-tangent 與 lateral correction；控制結構不同。
- 論文結果是 MATLAB/Virtual Reality/Simulink 的 numerical simulation；future work 才是 field experiments。

因此可寫成：

> Terlizzi et al. 證明 image-space VTP 對多旋翼路徑跟隨可行，並提供固定像素 annulus 的模擬案例；它支持研究 target-point guidance，但不直接提供目前 yaw-based controller 的 look-ahead tuning law。

不能寫成「目前 40% 就是該篇 carrot point 的等價實作」，也不能說該篇已給出可直接套用的速度自適應 look-ahead 理論。

---

### 4. Brandão et al. 的實驗證據被放大了

原文件正確指出 Brandão et al. 使用位置誤差和方向誤差，並對簡化閉迴路模型做 Lyapunov 分析。但原文的證據範圍必須完整描述：

- 穩定性推導假設平坦地面與內層控制器保持高度。
- 相機畫面需要與地面平行；作者討論 gimbal 補償或 holonomic vehicle。
- 無量測誤差時證明漸近穩定；有位置／角度感測誤差時只證明誤差有界，界線取決於量測誤差。
- 實驗部分使用真實飛行影片驗證**線偵測器**能輸出平均位置與方向。
- 論文結論把「將 controller 與 detector 整合成自主飛行」列為下一步；它沒有提供完整閉迴路自主循線的實飛證據。

這仍是一篇有價值的控制參考，但不能把它描述成與本 repo 相同場景、已完成閉迴路實飛驗證的「學術版本」。

---

### 5. 現有系統應稱為 `IBVS-inspired`，不是完整 IBVS

目前系統確實用 image features 閉迴路控制，因此可以說受到 IBVS 類型方法啟發。但若要正式宣稱 IBVS，通常需要明確定義：

- feature vector `s`
- desired feature `s*`
- feature error `e = s - s*`
- interaction matrix `L_s`
- camera velocity 與 aircraft/body command 的映射
- depth、ground-plane、camera calibration 或其他可觀測性假設

目前程式是：

- angle error → yaw P control
- offset error/rate → lateral PD control
- dead zone、hysteresis、slew limit、安全閘門

這是合理的 engineering image-feedback controller，但尚未建立 interaction matrix。建議原文件把「目前已是 IBVS」改成：

> 目前是 IBVS-inspired image-feature feedback；使用與 line-feature IBVS 相近的方向與偏移特徵，但尚未建立正式 interaction matrix 或穩定性分析。

另外，「目前 P/PD 是正式 IBVS 或 Brandão 非線性控制律的線性化特例」也未經推導，不能只憑形式相似就下結論。

---

### 6. Kalman filter 不能直接取代 endpoint counters

Kalman filter 可以改善：

- angle／offset 的狀態估計
- covariance 與 innovation gating
- 短時間漏檢時的預測
- 根據不確定度調整速度或量測信任度

但 `consecutiveEndpointDetections`、`consecutiveEndpointMisses`、`endpointPending` 與 `VERIFYING_ENDPOINT` 不只是雜訊濾波；它們編碼飛行狀態機的 temporal confirmation、hysteresis 與動作觸發語意。

即使加入 Kalman filter，仍可能需要：

- 連續多幀確認
- 最短持續時間
- 狀態轉移 guard
- 誤觸發後的 recovery

比較精確的建議是：

> Kalman covariance 可加入 endpoint evidence gate，取代部分固定閾值或調整確認時間；能否刪除 counters 必須由 false-positive／false-negative 實驗決定。

在缺少可靠 process model、measurement covariance 與 ground truth 時，加入 Kalman filter 只會把可見的幾個常數換成較難解釋的 `Q`、`R` 與初始 covariance，不一定降低複雜度。

---

### 7. LGVF 引用與 circular-orbit 主張需要重新核對

原文件把 DOI [`10.2514/1.G008056`](https://doi.org/10.2514/1.G008056) 描述為「Lyapunov Guidance Vector Field 的 circular orbit 部分」。Crossref 書目顯示該 DOI 是：

> Harinarayana, Krishnan, Hota, *Lyapunov Guidance Vector Field-Based Waypoint Following by Unmanned Aerial Vehicles*, Journal of Guidance, Control, and Dynamics, 48(1), 192–202, 2025；2024-12-04 online publication。

標題是 waypoint following，不足以單獨證明原文件所說的 circular-orbit 內容。沒有核對全文前，應改成候選參考，而不是已確認方案。

若要支持 circular loiter／orbit 的 globally stable vector field，應引用真正包含 circular-path 方程、假設與穩定性結果的原始論文，並區分：

- fixed-wing loiter guidance
- holonomic multirotor translation
- 本 repo 的 body-forward + yaw 控制

三者的控制輸入和運動學模型不同。

---

## 四、逐項修正原文件第七節建議

| 原建議 | 審閱結果 | 修正版 |
| --- | --- | --- |
| 1. 直接以 `curvatureDegrees` 加 `v/R` 前饋 | **不可實作；量綱錯誤，且資料未進 controller** | 已知實體半徑則用 configured `R`; 未知則先建立 signed metric curvature estimator |
| 2. 把固定 `0.40` 改為 `L = kv + L_min` | **方向合理，規格不完整** | 先定義 `L` 的座標系與單位，再建立 detector/controller configuration flow |
| 3. Kalman filter 取代 endpoint counters | **結論過度** | 先用錄影／flight log 建噪聲模型；Kalman 只作 state estimate 與 evidence gate，FSM guard 另行保留與驗證 |
| 4. 正規化為 IBVS 或非線性控制律 | **研究價值高，但不是重構工作** | 另立研究分支，先寫相機／地面／機體模型和可驗證 acceptance criteria，再談 interaction matrix 或 Lyapunov proof |

---

## 五、建議的安全研究順序

### Priority 0：先保留並驗證現有 baseline

在任何文獻導向控制改動前，先對目前 PR head 完成：

1. `connectedDebugAndroidTest`。
2. 地面影像串流與 detector smoke test。
3. 低高度、可立即人工接管的直線循跡。
4. 低高度圓形循跡。
5. 保存 APK SHA、commit、flight log、同步錄影及飛行條件。

沒有 baseline，就無法判斷後續理論改動改善或惡化了什麼。

### Priority 1：補齊量測，不改控制律

先記錄：

- raw/controlled angle
- raw/controlled offset
- commanded yaw/right/forward
- altitude
- detector path sample count
- visible arc length
- **signed** near/far tangent change
- frame timestamp 與 command timestamp

目標是量出穩態偏差、振盪頻率、漏檢率與控制飽和比例。

### Priority 2：若圓圈半徑已知，做固定半徑前饋 A/B test

保持 detector 不變，以實體 `R_meters` 和已知方向產生 feedforward：

\[
\omega = v/R + K_p e_\psi
\]

必須保留：

- yaw rate clamp
- stale detection fail-closed
- offset safety gate
- operator takeover
- 前饋 enable flag，讓同一天可回退 baseline

### Priority 3：只有在半徑未知時，研究 metric curvature

這需要 camera calibration、ground-plane projection 與高度／姿態資料。應另開功能 PR，不和 cleanup 或 Kalman filter 混在一起。

### Priority 4：最後才評估 adaptive look-ahead 與 Kalman filter

這兩項都需要 baseline log 才能定參數與驗證；沒有量測先加入，只是把手調常數搬到另一套公式。

---

## 六、原文件建議改寫的總結論

可將原文件第一節改成下列較準確版本：

> 現有系統屬於以影像方向與偏移為特徵的工程式閉迴路控制，與 line-following、Pure-Pursuit/VTP 及 IBVS 文獻有明確概念交集，但尚未建立正式 interaction matrix、metric ground-plane geometry 或穩定性分析。現有 `LOOKAHEAD_ARC_FRACTION` 是可見影像弧長比例，不是公尺前視距離；`curvatureDegrees` 是無符號總轉角，不是 `1/R`。因此文獻可用來提出下一輪假設，但每項控制改動仍需先補齊單位、資料流、失效策略與真機 A/B 驗證。

---

## 七、核對來源

1. Brandão, Martins, Soneguetti, *A Vision-based Line Following Strategy for an Autonomous UAV*, ICINCO 2015.  
   https://www.scitepress.org/papers/2015/55439/55439.pdf
2. Terlizzi et al., *A Vision-Based Algorithm for a Path Following Problem*, arXiv:2302.04742.  
   https://arxiv.org/abs/2302.04742
3. Crossref metadata for DOI `10.2514/1.G008056`.  
   https://api.crossref.org/works/10.2514/1.G008056
4. Repo implementation inspected for this review:
   - `TapePathDirectionEstimator.curvatureDegrees()`
   - `TapePathDirectionEstimator.LOOKAHEAD_ARC_FRACTION`
   - `TapeDetection`
   - `TapeTrackingObservation`
   - `TapeTrackingController.desiredYawRate()`
   - `TapeTrackingController.desiredRightSpeed()`
   - `TapeTrackingController.desiredForwardSpeed()`

---

## 八、後續修訂紀錄（2026-08-19 追加）

> 以下由後續討論產生，**不修改本審閱正文** —— 正文保留為當時審閱意見的原始紀錄。
> 本節僅標示其中兩項結論已被取代，避免單獨閱讀本文件時誤用。

### 修訂 A：第三節第 1 項與第四節建議 1 的「已知半徑」方案已作廢

本審閱正確指出原文件的 `curvatureDegrees` → `ω = v/R` 前饋量綱錯誤，
並提出「**A. 實體圓圈半徑固定且已知** → 設定 `radiusMeters` 直接前饋」為低工程量方案。

**該方案隱含了「只飛固定圓圈」的前提，而該前提未經確認。**
實際需求為：**系統必須在不預先知道路徑形狀的前提下運作**（直線、圓弧、S 形皆須可跟隨）。

**取代方案：完整實作 Pure Pursuit。**

- **新事實 [已驗程式碼]**：`TapeDetection` 已含 `lookaheadXFraction` / `lookaheadYFraction`，
  但 `MainActivity.kt:343` 建構 `TapeTrackingObservation` 時未傳入，
  前視點座標**僅寫入 flight log，從未進入控制器**。
  現行實作等於「挑了 look-ahead point 卻只取其切線角度、丟棄其位置」——
  只完成 Pure Pursuit 的一半，而丟棄的那一半正是產生曲率的部分。
- `κ = 2·x_L / L²`、`ω = v·κ`。形狀為輸出而非輸入：
  直線 `x_L→0`；等曲率弧 `x_L` 為定值；S 形 `x_L` 自行變號。
- **本審閱第三節第 1 項所列的三項缺陷，Pure Pursuit 自帶前兩項**：
  `L²` 即弧長歸一化，`x_L` 本身有號。**僅餘 metric scale**，
  可由高度與 FOV 近似求得 ground sample distance；
  其誤差僅表現為前饋增益誤差，由既有角度回授吸收。
- 審閱關於「圓弧方向已由 `b53714e` 固定為逆時針，符號為已知常數」的補充，
  在 Pure Pursuit 下**不再需要** —— 符號由量測給出。

### 修訂 B：第三節第 5 項與第四節建議 4 對 IBVS 的定性偏低

本審閱將 IBVS 定性為「研究價值高，但不是重構工作」，並要求先建立完整模型與 acceptance criteria。
形式上正確，但遺漏了一項**已存在於程式中的工程事實 [已驗程式碼]**：

- `TapeTrackingController.kt:276` 的註解明言必須「先決定橫移、再決定 yaw」，
  否則偏移的載具會「原地打轉而非平移過去」。
- `desiredYawRate()` 在 `lateralCorrectionActive` 為真時，
  將 yaw 上限壓至 `ANCHOR_ACQUISITION_MAX_YAW_RATE_DEGREES_PER_SECOND = 2.0`。

亦即：**yaw ↔ lateral 的耦合，現況是以執行順序、互相壓制與遲滯手工解決的，
而壓制值為經驗調參**。交互矩陣正是描述此耦合的正規工具，因此 IBVS 對應的是
一個現存的工程問題，而非僅是論文形式化。規模上為 2 特徵 × 2 自由度 → 2×2 矩陣，
前置需求僅相機焦距與飛行高度。

**排序結論不變**（仍在 Pure Pursuit 之後），但理由改變：
不是「它太學術」，而是 Pure Pursuit 將控制由「事後糾正」改為「事先預判」後，
所需修正量下降，耦合的實務影響可能隨之縮小。
**是否投入應由 `lateralCorrectionActive` 的實際觸發率與持續時間決定，不由推測決定。**

### 未受影響、仍然成立的審閱結論

- 第三節第 1 項：`curvatureDegrees` 不是 `κ`（量綱、符號、尺度三項差異）—— **完全成立**。
- 第三節第 2 項：`LOOKAHEAD_ARC_FRACTION` 是影像弧長比例而非 metric 距離 —— **完全成立**。
- 第三節第 3、4 項：Terlizzi 與 Brandão 的證據範圍限制 —— **完全成立**。
- 第三節第 6 項：Kalman filter 不能取代 endpoint FSM 的 temporal confirmation —— **完全成立**。
- 第三節第 7 項：LGVF 引用需重新核對 —— **完全成立**。
- 第五節 Priority 0／1：先建立 baseline 與量測再改控制律 —— **完全成立，且優先於上述所有改動**。
