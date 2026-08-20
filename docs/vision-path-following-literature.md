# 視覺循線飛行的相關論文調查

**建立日期**：2026-08-18
**修訂 1**：2026-08-18　依 [技術審閱](vision-path-following-literature-technical-review.md) 修正量綱、引用與定性錯誤。
**修訂 2**：2026-08-19　**需求釐清：系統必須在不預先知道路徑形狀的前提下運作。**
第七節建議 1 因此從「已知半徑前饋」改為 Pure Pursuit（與形狀無關）。
**修訂 3**：2026-08-19　修正對 IBVS 的低估 —— 它對應的是現有程式中
yaw↔lateral 的手工排序與壓制常數，屬工程問題而非論文形式化。見第一節與第七節建議 4。
**修訂 4**：2026-08-19　已實作 Pure Pursuit 實驗版：前視點座標進入控制器，以高度與
Mini 4 Pro 的 16:9 video FOV 近似地面尺度，計算 `κ = 2x_L/L²` 與 `ω = vκ`；尚待真機 A/B 驗證。
**目的**：為 repo 中「無人機看著地上黑膠帶飛行」的兩個功能（直線 / 圓弧）尋找學術文獻依據，
並釐清目前實作與文獻主流做法的差距。

> **本文件的引用可信度分級**
> 全文以三種標記區分證據強度，請勿混用：
> - **[已驗程式碼]**：本人直接讀過 repo 原始碼確認。
> - **[已讀摘要]**：讀過論文摘要或出版頁，未讀全文。
> - **[未讀全文．引自審閱]**：來自技術審閱者對全文的閱讀，本人未獨立核對。
> - **[憑既有知識]**：未在本次調查中開啟原文，正式引用前務必自行核對。

---

## 一、總結論

現有系統是**Pure Pursuit 前視曲率前饋 + 影像方向／偏移回授**的混合閉迴路控制。
它已建立近似的 metric ground-plane geometry，但仍沒有正式 interaction matrix、
相機姿態補償或穩定性分析。

兩個必須先講清楚、否則會導致錯誤實作的事實 **[已驗程式碼]**：

1. `LOOKAHEAD_ARC_FRACTION = 0.40` 是**本幀可見影像弧長的比例**（被乘數為 pixels），
   **不是**文獻中以公尺為單位的 look-ahead distance。
2. `TapePathEstimate.curvatureDegrees` 是**無號的可見片段總轉角（度）**，
   **不是**幾何曲率 `κ = dψ/ds = 1/R`。

因此本文件的文獻可用來**提出下一輪研究假設**，但每一項控制改動都仍需先補齊
單位、資料流、失效策略與真機 A/B 驗證。

### 現有程式碼與文獻概念的對照（是「相近」，不是「等價」）

| 現有程式碼 | 文獻中概念相近的對象 | 差距 |
| --- | --- | --- |
| `TapeTrackingObservation.angleFromVerticalDegrees` | line-following 的 orientation error | 相近，語意一致 |
| `TapeTrackingObservation.nearFieldOffsetFraction` | line-following 的 lateral displacement error | 相近，但為 normalized frame 比例，非地面公尺 |
| `lookaheadXFraction/YFraction` + 高度/FOV 尺度換算 | Pure Pursuit / VTP 的 look-ahead point | 已實作 `κ = 2x_L/L²`；相機姿態未量測，地面投影仍是近似 |
| `desiredYawRate()` | Pure Pursuit yaw 前饋 + 方向角比例回授 | 已實作混合控制，但無穩定性推導 |
| `desiredRightSpeed()` 的 PD | 橫向誤差 PD 控制 | 相近 |
| Otsu + 門檻上限截斷 + 相對亮度 | 作物行偵測的 illumination-robust segmentation | 問題類型相近，演算法未逐篇比對 |
| `previousBounds` 的 overlap 檢查 | temporal tracking gate | 相近；文獻常用參數空間 Kalman filter |
| `axialExponentialAverage()` 處理 ±90° 接縫 | 無方向直線（axial line）的角度平均 | **作法正確**，可保留 |

### 關於 IBVS 的正確定性

目前系統確實以 image features 閉迴路控制，因此可稱為 **IBVS-inspired image-feature feedback**。
程式也已明確建立 Pure Pursuit 所需的前視點、飛行高度與相機 FOV 地面尺度假設。
但要正式宣稱是 IBVS，仍需要 feature vector `s`、desired feature `s*`、
error `e = s - s*`、interaction matrix `L_s` 與 camera velocity 到機體指令的完整映射。

**不可宣稱**「現有 P/PD 是正式 IBVS 或 Brandão 非線性控制律的線性化特例」——
這只是形式相似，未經推導。

**但也不可反向低估其工程價值 [已驗程式碼]**：IBVS 的交互矩陣正是在描述
「各控制輸入分別使各影像特徵變化多少」，亦即**耦合**。而現有程式已經在手工處理這個耦合：

- `TapeTrackingController.kt:276` 的註解明言必須「先決定橫移、再決定 yaw」，
  否則偏移的載具會「原地打轉而非平移過去」。
- `desiredYawRate()` 在 `lateralCorrectionActive` 為真時，
  將 yaw 上限壓至 `ANCHOR_ACQUISITION_MAX_YAW_RATE_DEGREES_PER_SECOND = 2.0`。

也就是說，yaw ↔ lateral 的耦合現況是靠**執行順序、互相壓制與遲滯**解決的，
而那些壓制值是經驗調出來的。**IBVS 是這個問題的正規解，不只是論文形式化。**
規模上，2 個特徵對 2 個自由度 → 交互矩陣為 2×2，求逆平凡；
真正的前置需求只有相機焦距與飛行高度（depth）。詳見第七節建議 4。

---

## 二、最直接對應「直線膠帶」的兩篇

### 1. Brandão, Martins, Soneguetti (2015) — *A Vision-based Line Following Strategy for an Autonomous UAV*

- ICINCO 2015，[SCITEPRESS PDF](https://www.scitepress.org/papers/2015/55439/55439.pdf)
- **偵測** **[已讀摘要]**：下視相機 → Gaussian filter → Sobel 邊緣 → Hough transform，
  取出主線的平均位置與平均方向。
- **控制** **[已讀摘要]**：非線性 path-following 控制器，附 Lyapunov 分析。

**證據範圍（務必完整描述，不可放大）** **[未讀全文．引自審閱]**：

- 穩定性推導假設**平坦地面**，且由內層控制器維持高度。
- 相機畫面需與地面平行；作者討論 gimbal 補償或 holonomic vehicle。
- **無量測誤差時**證明漸近穩定；有位置／角度感測誤差時**只證明誤差有界**，界線取決於量測誤差。
- 實驗部分以真實飛行影片驗證的是**線偵測器**能輸出平均位置與方向。
- 論文把「將 controller 與 detector 整合為自主飛行」列為**下一步工作**，
  **未提供完整閉迴路自主循線的實飛證據**。

**因此不可寫成**「這是本 repo 的學術版本、已完成閉迴路實飛驗證」。
它是一篇有價值的**控制設計參考**，其誤差定義與本 repo 的角度／偏移回授概念相符。

### 2. Terlizzi, Silano, Russo, Aatif, Basiri, Mariani, Iannelli, Glielmo (2021) — *A Vision-Based Algorithm for a Path Following Problem*

- [arXiv:2302.04742](https://arxiv.org/abs/2302.04742)（2021 發表，2023 上傳 arXiv）
- 把 Pure Pursuit 與 IBVS 結合，下視相機做四旋翼路徑跟隨；為某 UAV 競賽獲獎演算法，**有開源程式碼**。

**證據範圍** **[已讀摘要]**：驗證方式為 **MATLAB / Virtual Reality toolbox / Simulink 的數值模擬**，
field experiments 列為 future work。

**細節限制** **[未讀全文．引自審閱]**：

- 以 image-space **annular mask** 尋找 Virtual Target Point，inner/outer radius 為**固定 26/28 pixels**。
- 明確假設**固定高度 1 m**。
- 路徑規劃器參數 `a`、`b` 為 heuristic tuning。
- 作者利用多旋翼可側向平移的特性，明確表示 path follower **不控制 heading**；
  本 repo 則使用 body-forward velocity + yaw-to-tangent + lateral correction，**控制結構不同**。

**正確的引用方式**：

> Terlizzi et al. 證明 image-space VTP 對多旋翼路徑跟隨可行，並提供固定像素 annulus 的模擬案例；
> 它支持研究 target-point guidance，但**不直接提供**目前 yaw-based controller 的 look-ahead tuning law。

**不可寫成**「目前的 40% 就是該篇 carrot point 的等價實作」，
也**不可說**該篇已給出可直接套用的速度自適應 look-ahead 理論。

---

## 三、幾何導引法的理論根據

### Sujit, Saripalli, Sousa (2014) — *UAV Path Following: A Survey and Analysis of Algorithms*

- IEEE Control Systems Magazine, 34(1):42–59。
  [ASU 頁面](https://asu.elsevierpure.com/en/publications/unmanned-aerial-vehicle-path-following-a-survey-and-analysis-of-a/)、
  [Semantic Scholar](https://www.semanticscholar.org/paper/An-evaluation-of-UAV-path-following-algorithms-Sujit-Saripalli/578d40f2ba1bc5733d8c319da68dc27e9b7f851f)
- 系統性比較 Carrot-Chasing、Nonlinear Guidance Law、Pure Pursuit + LOS、Vector Field。
- **一般性結論** **[已讀摘要]**：前視太短會放大量測雜訊、增加振盪；太長則可能在曲線上內切（cut corner）。

**可以主張的**：這是一個真實的取捨，值得研究 adaptive look-ahead；
目前圓形模式以較低速度、不同 yaw gain 與安全閘門來因應曲線，確實反映直線與曲線的不同需求。

**不可主張的**：不能推論「九個 `CIRCULAR_*` 常數都因 fixed look-ahead 而存在」。
**[已驗程式碼]** 那九個常數是 dead zone、yaw gain、max yaw rate、兩段前進速度、
max centering speed、stable angle，以及允許移動的角度／偏移安全界線 ——
**adaptive look-ahead 無法單獨刪除它們**。

### Nelson, Barber, McLain, Beard (2007) — *Vector Field Path Following for Miniature Air Vehicles*

- IEEE Transactions on Robotics。**[憑既有知識]**
- 一般認為 vector field 法在曲線路徑上的追蹤誤差小於 carrot-chasing。**引用前請自行核對原文。**

### Rezende et al. (2019) — *A Survey of Path Following Control Strategies for UAVs Focused on Quadrotors*

- [Journal of Intelligent & Robotic Systems](https://link.springer.com/article/10.1007/s10846-019-01085-z) **[已讀摘要]**
- 四旋翼專用綜述，適合當作日後論文 related work 的骨架。

---

## 四、IBVS 的理論基礎

- **Chaumette & Hutchinson**, *Visual Servo Control, Part I / Part II*, IEEE RAM 2006 / 2007。
  **[憑既有知識]** interaction matrix 的標準定義來源。
- **Azinheira & Rives** — *Image-Based Visual Servoing for Vanishing Features and Ground Lines
  Tracking: Application to a UAV*，
  [INRIA PDF](http://www-sop.inria.fr/members/Patrick.Rives/Publications/IJOP-azinheira-rives-08.pdf)
  **[已讀摘要]** 場景最貼近：地面線 + 消失點的 IBVS，應用於 UAV。
- **Bourquardez, Mahony, Guenard, Chaumette, Hamel, Eck** — *Image-Based Visual Servo Control of
  the Translation Kinematics of a Quadrotor Aerial Vehicle*，
  [HAL 全文](https://inria.hal.science/inria-00436722v1/document) **[已讀摘要]**
  以 spherical image moments 做四旋翼 IBVS，含穩定性分析。

---

## 五、偵測端的相關文獻

近期 commit（`以相對色彩辨識環境光下黑膠帶`、`排除僅在端點彎折的筆直背景`）處理的問題，
在文獻上主要出現於兩個社群。**問題類型相近可以主張；但在未逐篇核對演算法與資料集前，
不可宣稱現行閘門是某篇方法的簡化等價物。**

### 作物行偵測（illumination robustness）

- [Crop Row Segmentation Based on Treble-Classification Otsu and Double-Dimensional Clustering
  (Remote Sensing 2021)](https://doi.org/10.3390/rs13050901) **[已讀摘要]**
- [A review of vision-based crop row detection method (Comput. Electron. Agric. 2024)](https://www.sciencedirect.com/science/article/abs/pii/S0168169924004770) **[已讀摘要]**
- 「Otsu 門檻加上限截斷、避免亮牆使整塊棕色紙板被歸入暗類」在該領域是常見議題。

### 電力線巡檢（line detection + tracking）

- **Zhang et al.** — *High Speed Automatic Power Line Detection and Tracking for a UAV-Based
  Inspection*，[ResearchGate](https://www.researchgate.net/publication/261166400_High_Speed_Automatic_Power_Line_Detection_and_Tracking_for_a_UAV-Based_Inspection) **[已讀摘要]**
  流程：Hough 取線段 → Hough 參數空間 K-means 過濾 → Kalman filter 在參數空間追蹤。
  可作為 temporal gating 的比較對象。
- [Powerline Tracking with Event Cameras (arXiv:2108.00515)](https://arxiv.org/pdf/2108.00515)
- [LS-Net: Fast Single-Shot Line-Segment Detector (arXiv:1912.09532)](https://arxiv.org/pdf/1912.09532)

---

## 六、關於「圓圈膠帶」

**未找到專門處理「無人機跟隨地面圓形標記」的論文。**

**但這其實是把問題設定錯了。** 需求是「不預先知道形狀，看到什麼就跟什麼飛」，
因此不該去找「圓形跟隨」的專門解法，而應該回到**與形狀無關的 path-following 幾何**——
Pure Pursuit 本來就不需要路徑的解析形式，見第七節。

其餘曾檢視的路線：

1. **Lyapunov Guidance Vector Field**。原先引用的
   DOI [`10.2514/1.G008056`](https://doi.org/10.2514/1.G008056) 經核對，
   實際標題為 Harinarayana, Krishnan, Hota, *Lyapunov Guidance Vector Field-Based
   **Waypoint Following** by Unmanned Aerial Vehicles*, JGCD 48(1):192–202, 2025。
   **標題是 waypoint following，不足以單獨支持 circular-orbit 主張**，
   目前僅列為**候選參考**。若要引用 circular loiter／orbit 的 globally stable vector field，
   應找到真正包含 circular-path 方程、假設與穩定性結果的原始論文，並區分
   fixed-wing loiter guidance、holonomic multirotor translation，
   以及本 repo 的 body-forward + yaw 控制 —— 三者運動學模型與控制輸入皆不同。

其他曾檢視、相關性較低者：
- [Drone Path-Following in GPS-Denied Environments using Convolutional Networks (arXiv:1905.01658)](https://arxiv.org/pdf/1905.01658)
  end-to-end CNN 直接輸出轉向指令，路線與本 repo 的幾何式做法不同，可作對照組。

---

## 七、改動建議

> **本節已修訂三次。**
> 第一版建議直接以 `curvatureDegrees` 做 `v/R` 前饋 —— **量綱錯誤**，已刪除。
> 第二版依技術審閱改為「實體圓圈半徑已知時用 configured `radiusMeters`」——
> **該方案隱含了「只飛固定圓圈」的前提，與實際需求（不預先知道形狀）不符**，已刪除。
> 目前版本為 Pure Pursuit，與形狀無關。

### 建議 1（已實作實驗版）：以完整前視點位置執行 Pure Pursuit

**目前的程式碼事實 [已驗程式碼]**：`MainActivity` 已將
`TapeDetection.lookaheadXFraction` / `lookaheadYFraction`、來源影像尺寸與可用飛行高度
傳入 `TapeTrackingObservation`。`TapeTrackingController` 以畫面上的飛機參考點
`(0.5, 0.94)` 為原點，使用來源影像長寬比、Mini 4 Pro 的 `75°` 16:9 video FOV 與高度，
把前視點近似投影到地面公尺，再計算有號曲率和 yaw rate 前饋。

這條前饋與既有方向角比例回授相加，最後仍受各模式的 yaw 上限、橫移期間 yaw 壓制及
輸出加速度限制。高度不存在、前視點不在影像前方或 metric look-ahead 小於 `0.10 m` 時，
Pure Pursuit 前饋回傳零，保留既有方向角回授作 fallback。

Pure Pursuit 的標準結果為

```
κ = 2·x_L / L²
```

其中 `x_L` 為前視點在機體橫向的有號偏移、`L` 為到前視點的距離；
再由 `ω = v·κ` 得到 yaw rate。

**為何與形狀無關**：路徑形狀不是這條式子的輸入，而是輸出。
直線時 `x_L → 0` 故 `κ → 0`；等曲率圓弧上 `x_L` 為定值故 `κ` 為定值；
S 形上 `x_L` 自行變號故 `κ` 自行反向。**不需要任何路徑的解析形式或先驗參數。**

**與第八節「陷阱」的關係**：`2x/L²` 這條式子**自帶弧長歸一化（`L²`）與符號（`x` 有正負）**，
因此 `curvatureDegrees` 缺的三項中，前兩項不需另外補。
**metric scale 的目前實作**：以飛行高度、來源影像長寬比與相機對角 FOV 推算每個 frame
高度對應的地面公尺。這比拿 degree 當 `κ` 具有正確量綱，但仍假設地面平坦且相機朝下。
App 不再強迫或監控雲台姿態，因此操作者必須手動把相機置於適合看地面的角度；
相機偏離正下方時會產生投影尺度誤差，目前由 yaw 上限與既有方向角回授限制影響。

**已知風險**：單一前視點對偵測雜訊敏感，且 `75°` video FOV 模型未包含鏡頭畸變與
雲台姿態。若實測抖動過大，下一步是依 flight log 量測誤差，再決定校正投影模型或對
整條 traced centerline 做最小平方圓弧擬合；不得先用未量測的濾波掩蓋問題。

### 為什麼 `curvatureDegrees` 不能直接做前饋 **[已驗程式碼]**

`TapePathDirectionEstimator.curvatureDegrees()` 的實作是：取 centerline 近端與遠端各一條
局部切線，計算兩者角度後回傳

```
min(|ψ_far - ψ_near|, 180° - |ψ_far - ψ_near|)
```

即「目前可見片段的**總轉角**」，範圍約 `[0°, 90°]`。與前饋所需的 `κ` 差異：

| 性質 | 現有 `curvatureDegrees` | 前饋需要的 `κ` |
| --- | --- | --- |
| 單位 | degree | `m⁻¹` |
| 是否除以弧長 | 否 | 是 |
| 左／右彎符號 | **無**（程式使用 `abs`） | 必須 signed |
| 地面尺度 | 無（來源為 image pixels） | 必須為地面公尺 |
| 對可見片段長度的敏感性 | 高 | 理想圓弧上應近似固定 |
| 對飛行高度／相機投影的敏感性 | 高 | 正確轉換後不應隨高度任意變動 |

正確式子為 `ω_rad/s = v_m/s · κ_m⁻¹`；若指令為 degree/s 則需 `radToDeg(v·κ)`。
把 degree 直接塞進 `v/R` 是量綱錯誤。

**另一個資料流事實 [已驗程式碼]**：`curvatureDegrees` 目前只存在於 detector 的 candidate 與
diagnostics。`TapeDetection` 與 `TapeTrackingObservation` **都沒有 curvature 欄位**，
`TapeTrackingController` 收不到。這不是「接一條線」的改動。

**兩點補充 [已驗程式碼]**：

- `curvatureSmoothness()` 內部已累積 `signedTurn`，只是僅用於計算
  `directionConsistency` 比值後即丟棄。影像空間的有號轉角離取得只差一步，
  但仍缺弧長歸一化與 metric scale。
- commit `b53714e 固定圓弧路徑為逆時針方向` 曾使方向成為已知常數。
  **採用 Pure Pursuit 後不再需要這個假設** —— `x_L` 自帶正負號，
  左彎右彎由量測決定，這也是它能處理 S 形路徑的原因。

### 修正後的四項建議

| # | 建議 | 審閱結果 | 修正版 |
| --- | --- | --- | --- |
| 1 | 曲率前饋 | 第一版量綱錯誤；第二版（已知半徑）與「不預知形狀」的需求不符 | **已實作實驗版**：`lookaheadXFraction/YFraction` 已接進控制器，以高度、影像比例與 FOV 近似 metric `x_L` 與 `L`，計算 `κ = 2x_L/L²`、`ω = vκ`；尚待真機 A/B 驗證相機投影與增益 |
| 2 | adaptive look-ahead | 方向合理，**規格不完整** | 先擇定 `L` 的座標系與單位（metric／image-space／fractional 三選一並如實命名），再建立明確的 detector↔controller configuration flow，不可讓兩層互相偷讀狀態 |
| 3 | Kalman filter | 「取代 endpoint counters」**結論過度** | 先用錄影／flight log 建噪聲模型；Kalman 只做 state estimate 與 evidence gate，FSM 的 temporal confirmation／hysteresis／guard 另行保留並驗證 |
| 4 | 正規化為 IBVS | 審閱定性為「研究價值高但不是重構工作」——**低估了它的工程價值**（見下） | 目標不是形式化，而是**用交互矩陣取代現有的 yaw↔lateral 手工排序與壓制常數**。前置需求僅相機焦距與高度；矩陣為 2×2。排序在 Pure Pursuit 之後，並以壓制邏輯的實際觸發率決定是否值得做 |

**關於建議 4 的補充**：審閱把 IBVS 視為純學術形式化，因而排在最後並標為「非重構工作」。
這個定性遺漏了一件事 —— 現有程式已經在手工解 IBVS 該解的問題（yaw↔lateral 耦合，
見第一節「關於 IBVS 的正確定性」所列的兩處程式碼）。因此：

- **它不是「只有寫論文才需要」**：交互矩陣直接對應那些試出來的壓制常數。
- **它仍排在 Pure Pursuit 之後**，理由是 Pure Pursuit 將控制從「事後糾正」改為
  「事先預判」後，所需修正量下降，耦合的實務影響可能隨之縮小。
- **判準應為量測而非推測**：以 `lateralCorrectionActive` 的觸發率與持續時間決定是否值得投入。
  若 Pure Pursuit 上線後該路徑已極少觸發，即不必做。

**關於建議 3 的補充**：`consecutiveEndpointDetections`、`consecutiveEndpointMisses`、
`endpointPending` 與 `VERIFYING_ENDPOINT` 編碼的是飛行狀態機的 temporal confirmation、
hysteresis 與動作觸發語意，**不只是雜訊濾波**。在缺少可靠 process model、
measurement covariance 與 ground truth 時導入 Kalman filter，
只會把幾個可讀的常數換成較難解釋的 `Q`、`R` 與初始 covariance，**未必降低複雜度**。

---

## 八、建議的研究順序（來自技術審閱；屬流程建議而非技術更正）

1. **Priority 0 — 先保留並驗證現有 baseline**：`connectedDebugAndroidTest`、
   地面 detector smoke test、低高度可人工接管的直線與圓形循跡，
   並保存 APK SHA、commit、flight log、同步錄影與飛行條件。沒有 baseline 就無法判斷改動的好壞。
2. **Priority 1 — 補齊量測，不改控制律**：記錄 raw/controlled angle 與 offset、
   commanded yaw/right/forward、altitude、path sample count、visible arc length、
   **signed** near/far tangent change、frame 與 command timestamp。
   目標是量出穩態偏差、振盪頻率、漏檢率與控制飽和比例。
3. **Priority 2 — Pure Pursuit 前饋 A/B test**：保持 detector 不變
   （前視點已算好，只需接進 `TapeTrackingObservation`），
   `ω = v·(2x_L/L²) + K_p·e_ψ`，ground sample distance 由高度與 FOV 近似。
   必須保留 yaw rate clamp、stale detection fail-closed、offset safety gate、
   operator takeover，以及可當天回退的前饋 enable flag。
4. **Priority 3 — 若單點雜訊過大，改用 centerline 圓弧擬合**：對整條 traced centerline
   做最小平方圓弧擬合取得 signed image curvature，同樣與形狀無關但工程量較大，應另開功能 PR。
5. **Priority 4 — 評估 adaptive look-ahead 與 Kalman filter**：
   兩者都需 baseline log 才能定參數與驗證。
6. **Priority 5 — 最後以量測決定是否導入 IBVS**：檢視 `lateralCorrectionActive`
   的觸發率與持續時間。若 yaw↔lateral 的手工壓制仍頻繁作用，代表耦合問題未消，
   此時導入 2×2 交互矩陣才划算；若已極少觸發則不必做。**以飛行資料判定，不以推測判定。**

---

## 九、可用於後續搜尋的關鍵詞

- `image-based visual servoing line following UAV`
- `carrot chasing` / `pure pursuit` / `nonlinear guidance law` / `vector field path following`
- `crop row detection illumination robust Otsu`
- `power line detection tracking Kalman Hough parameter space`
- `signed curvature estimation ground plane homography monocular`
