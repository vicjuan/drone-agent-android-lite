# 黑膠帶路徑視覺管線重構設計

## 文件目的

本文件記錄 `drone-agnet-android-lite` 下一階段的黑膠帶視覺辨識設計，供後續實作、審查與真機驗證使用。

設計不是單純調整 OpenCV 門檻，也不是移除 OpenCV。核心改變是：

> OpenCV 負責從影像產生有來源可追蹤的膠帶候選遮罩；移植 `~/git/drone-agent-android` 的中心線量測能力，從候選遮罩抽取並驗證有序路徑；控制器只使用具有明確品質等級的路徑資料。

目標是解決目前真機畫面中觀察到的三類失效：

1. 螢光反射讓黑膠帶局部變白，路徑被切成數段。
2. 完整路徑消失後，短水平殘片或背景暗邊可能成為 `HORIZONTAL_FALLBACK`。
3. 視覺資料已不足以安全前進時，控制器仍可能取得偏移、角度或前視點並繼續平移。

本文件只定義視覺量測及其與控制器的契約。Pure Pursuit 控制律的完整修改另行實作，但兩者的介面在此明確定義。

---

## 現況的正確描述

目前 `BlackTapeDetector` 並不是「看到黑色就直接認定為膠帶」。它已具備多層過濾，包括：

- 灰階與 Lab 色彩判斷。
- 黑色亮度上限。
- 色彩通道平衡檢查。
- 面積、長度、寬度及外形限制。
- 紙板／地板背景比例。
- 路徑兩側背景檢查。
- 曲率、方向及平滑度條件。
- 與上一幀 bounds、anchor、寬度的連續性。
- 畫面邊緣、水平候選與其他異常候選的拒絕條件。
- 多方向 morphology，用來修補反光造成的短缺口。

真正的結構性限制是：目前主要候選單位仍是「修補後的黑色區域／輪廓」，之後再由 `TapePathDirectionEstimator` 以 row/column 路徑估計及 `HORIZONTAL_FALLBACK` 推導方向。

這會造成以下問題：

1. Morphology 先將區域接起來後，後續很難區分真實黑色像素與推測補上的橋。
2. Bounding box 與 contour 適合描述物體外觀，不足以表示路徑的順序、分支、閉環和真實終點。
3. Row/column 走訪帶有方向偏好，曲線、U 型、圓環及畫面邊緣路徑需要特殊處理。
4. 上一幀只保留框、anchor 與少量統計，無法完整表示曲線身份。
5. 短殘片可能通過局部條件，卻不具備可安全前進的完整前視路徑。

---

## 已有能力與移植來源

優先重用 `~/git/drone-agent-android` 的既有量測層，不在 Lite 平行發明第二套中心線架構。

主要來源：

- `vision/.../segment/BoardRelativeSegmenter.kt`
- `vision/.../segment/TapeSegmenter.kt`
- `vision/.../centerline/CenterlineExtractor.kt`
- `vision/.../imagespace/ImageSpaceErrorEstimator.kt`
- `docs/tape-following-design.md`

其中可重用的設計包括：

1. **紙板相對分割**：以局部紙板背景與候選像素的相對亮度辨識膠帶，而非完全依賴全域 Otsu。
2. **方向無關中心線**：distance transform、Zhang–Suen thinning、8-connected skeleton graph、寬度加權 Dijkstra。
3. **有序中心線**：由靠近畫面底部的近場端開始排列中心線點。
4. **弧長參數化**：沿中心線弧長選擇 lookahead，並計算切線與曲率。
5. **拓撲資訊**：區分畫面內終點、畫面邊界截斷、閉合環與可信分支。
6. **可拆解信心分數**：support、width consistency、continuity、fit residual 等可觀察成分。

移植時應保持演算法語意及測試資料結構；允許將高成本像素運算改由 Lite 已有的 OpenCV Android runtime 執行，但不得建立另一套不相容的路徑模型。

---

## 設計原則

### 1. 候選遮罩不等於確認路徑

任何 threshold、morphology 或 contour 結果都只能提供候選證據。只有通過中心線、幾何、背景、拓撲及時間驗證的結果，才能成為可控制路徑。

### 2. 新證據優先於歷史推測

上一幀路徑可以協助候選關聯及短缺口判斷，但不能在持續看不到膠帶時讓飛機沿歷史路徑盲飛。

### 3. 真實像素與修補像素必須分開

原始分割支持和 morphology／幾何補洞支持必須保留不同來源。修補比例越高，路徑可信度越低。

### 4. 所有形狀使用同一條路徑模型

直線、圓弧、U 型與閉合圓環都是中心線幾何，不應以 `STRAIGHT`、`CIRCULAR` 或 `HORIZONTAL_FALLBACK` 改變視覺抽取方法。

### 5. 無足夠前視資料時停止平移

不能計算可信 Pure Pursuit 路徑，不代表應將近場角度當成零，也不代表可以沿最後命令前進。視覺品質必須直接限制控制權限。

### 6. 不用放寬門檻掩蓋模型缺陷

不能以降低紙板兩側、寬度、曲率或跨幀一致性要求來追求「每幀都有結果」。拒絕不可信結果比錯誤控制飛機安全。

---

## 目標架構

```text
Camera RGBA frame
        │
        ▼
OpenCV 影像前處理與候選產生
- resize / color conversion
- 局部紙板背景估計
- 相對暗色遮罩
- 色彩 veto
- 原始候選元件
- 多方向 bridge 提議
        │
        ▼
路徑幾何層（移植主專案）
- distance transform
- Zhang–Suen thinning
- 8-connected skeleton graph
- width-aware path extraction
- ordered centerline
- branch / loop / terminus topology
        │
        ▼
時間與品質驗證
- 與上一幀中心線關聯
- bridge 驗證
- width / board-side / curvature continuity
- evidence ratio / confidence
        │
        ▼
路徑量測層
- near-field anchor and tangent
- arc-length lookahead
- image-space heading / curvature
- ground projection（姿態有效時）
        │
        ▼
PathQuality
- FULL_PATH
- NEAR_FIELD_ONLY
- LOST
        │
        ▼
控制器
- Pure Pursuit
- 原地近場對準
- 懸停並重新取得
```

---

## 第一層：OpenCV 影像前處理與候選遮罩

Lite 已依賴：

```kotlin
implementation("org.opencv:opencv:4.14.0")
```

OpenCV 應保留，專注於矩陣與像素運算：

- RGBA → RGB／gray／Lab。
- 縮放與 Gaussian blur。
- threshold 與遮罩布林運算。
- morphology opening／closing。
- connected components／contours。
- distance transform（若與移植演算法輸入契約一致）。
- 診斷遮罩輸出。

### 紙板相對分割

從主專案 `BoardRelativeSegmenter` 移植判斷語意：

1. 將影像降採樣到固定工作解析度。
2. 以 tile 估計當地背景亮度。
3. 背景低於紙板最低亮度的 tile 視為紙板外，不產生膠帶候選。
4. 像素比當地紙板背景暗到一定程度時，才成為候選。
5. 有 chroma 資訊時，以高彩度作為 veto，排除彩色物體；低彩度本身不構成膠帶證據。
6. 候選元件需符合面積、寬度、延展性與近場可達性。
7. 候選法線方向兩側需具有平衡且較亮的紙板支持。

可保留目前 Lite 的固定黑色亮度上限，作為額外安全條件，而不是唯一分割依據。

### 必須保留的遮罩來源

每幀至少保留以下邏輯遮罩：

- `rawDarkMask`：只包含原始影像直接支持的暗色候選。
- `boardMask`：可信紙板區域。
- `appearanceCandidateMask`：通過局部對比與色彩 veto 的候選。
- `bridgeProposalMask`：OpenCV morphology 建議加入的像素。
- `acceptedPathMask`：經幾何與時間驗證後的最終路徑遮罩。

不得只保留最後合成結果，否則無法計算修補比例或診斷錯誤連接。

---

## 第二層：方向無關的有序中心線

以主專案 `CenterlineExtractor` 為基礎：

```text
SegmentationMask
→ 降採樣工作網格
→ 3-4 chamfer distance transform
→ Zhang–Suen thinning
→ 8-connected skeleton
→ 寬度加權 Dijkstra
→ ordered centerline
```

中心線抽取不得預設膠帶方向。輸出至少包含：

```kotlin
data class CenterlinePoint(
    val x: Double,
    val y: Double,
    val widthPixels: Double,
)

data class CenterlineTopology(
    val distalTerminus: CenterlineTerminus,
    val distalBorderDistancePixels: Double,
    val branchCount: Int,
    val closedLoop: Boolean,
)
```

### 為何取代 row/column 與 `HORIZONTAL_FALLBACK`

骨架圖直接表示路徑連通性，因此：

- 水平、垂直及對角線沒有不同權限。
- 圓弧與 U 型不需要切換抽取模式。
- 閉環可以被辨認為無自然終點。
- 畫面內真實終點與畫面邊界截斷可以分開處理。
- 短水平殘片不會因方向特殊而直接升格成控制路徑。

完成切換後，應刪除已被取代的 row/column 特殊路徑與 `HORIZONTAL_FALLBACK` 控制分支，不保留雙軌實作。

---

## 第三層：受約束的反光缺口修補

目前多方向 morphology 可以保留，但輸出只作為 bridge proposal。任何 bridge 都必須通過以下條件：

1. **寬度一致**：兩端局部膠帶寬度在允許比例內。
2. **缺口有限**：缺口長度不超過局部膠帶寬度的有限倍數；實際倍數需由回歸素材決定，不在未量測前硬編常數。
3. **切線延續**：兩端切線朝向彼此，連接後方向變化平滑。
4. **曲率連續**：不得產生單點尖角或不可能的曲率跳變。
5. **紙板支持**：bridge 法線方向兩側仍為可信紙板。
6. **歷史相容**：bridge 位於上一幀中心線的有限預測走廊內。
7. **近場連接**：至少一側應連到本幀已確認的近場路徑，避免遠端兩個無關暗物體自行組成路徑。
8. **修補比例限制**：人工補出的弧長或面積不能占完整路徑過高比例。

連接失敗時必須安全降級為 `NEAR_FIELD_ONLY` 或 `LOST`，不可為了維持偵測率降低所有條件。

### 不採用的方案

- 不使用單一巨大 morphology kernel 無條件跨越反光。
- 不只依兩個 bounding box 距離決定連接。
- 不沿上一幀中心線永久補畫看不見的路徑。
- 不把 bridge 像素與原始黑色像素視為等價證據。

---

## 第四層：跨幀中心線關聯

目前歷史資料以 bounds、anchor 與寬度為主。新版本應保留上一筆已確認的有序中心線及其時間戳。

由上一幀中心線形成有限搜尋走廊，協助：

- 在多個候選中維持原路徑身份。
- 判斷反光前後片段是否應連接。
- 拒絕突然跳到畫面另一側的背景邊緣。
- 驗證中心線位移、方向與曲率變化是否符合幀間運動。
- 在短暫干擾後重新取得同一路徑。

### 時間資料不得做的事

- 本幀沒有新證據時，不得繼續輸出上一幀的 `FULL_PATH`。
- 不得用歷史中心線偽造 endpoint 或 closed-loop。
- 不得只因候選與上一個 bounds 重疊就跳過紙板、寬度及拓撲驗證。

### 重新取得

進入失去路徑狀態後，第一個新候選不得直接控制飛機。候選需連續多幀具備：

- 近場可見。
- 寬度一致。
- 方向一致。
- 位移有限。
- 紙板兩側成立。
- 與遺失前路徑或合法搜尋區域相容。

確認幀數與容許值需由錄製素材及真機測試決定，不能沿用沒有證據的任意數值。

---

## 第五層：弧長前視與路徑量測

沿用主專案 `ImageSpaceErrorEstimator` 的弧長參數化概念。

從同一條有序中心線產生：

- `anchor`：靠近畫面底部的近場中心線點。
- `nearFieldTangent`：近場方向，用於停止平移後的原地對準。
- `lookahead`：沿中心線總弧長的設定比例取得，不以固定影像高度選點。
- `heading`：lookahead 附近的平滑切線角度。
- `curvature`：切線角度相對弧長的變化率。
- `arcLength`：本幀可見中心線長度。

近場角度與 Pure Pursuit 前視點必須來自同一條中心線，避免兩套路徑估計對同一畫面給出互相矛盾的方向。

---

## 第六層：路徑信心與品質契約

建議的控制邊界資料：

```kotlin
enum class PathQuality {
    FULL_PATH,
    NEAR_FIELD_ONLY,
    LOST,
}

data class PathConfidenceComponents(
    val imageSupport: Double,
    val widthConsistency: Double,
    val continuity: Double,
    val curveSmoothness: Double,
    val boardSideSupport: Double,
    val temporalConsistency: Double,
    val directEvidenceFraction: Double,
)

data class TrackedPath(
    val centerline: List<CenterlinePoint>,
    val anchor: CenterlinePoint,
    val nearFieldTangentRadians: Double?,
    val lookahead: CenterlinePoint?,
    val curvaturePerPixel: Double?,
    val confidence: Double,
    val confidenceComponents: PathConfidenceComponents,
    val topology: CenterlineTopology,
    val quality: PathQuality,
)
```

實際命名應配合移植後的既有主專案型別，避免建立重複資料類別；上例用來定義必要語意，不代表必須原樣新增。

### `FULL_PATH`

最低語意：

- 近場中心線可信。
- 中心線具有足夠點數及弧長。
- lookahead 可用。
- 寬度與曲率合理。
- 紙板兩側支持成立。
- 與上一幀路徑連續。
- 原始影像直接支持比例足夠。
- 無未處理的可信分支歧義。

控制權限：允許 Pure Pursuit 計算前進、橫移與 yaw。

### `NEAR_FIELD_ONLY`

最低語意：

- 近場膠帶及切線可信。
- 遠端路徑太短、遭反光遮斷、存在分支歧義，或 lookahead／曲率不可信。

控制權限：

```text
forward = 0
right = 0
只允許受限的近場角度原地對準
```

近場片段不得提供 Pure Pursuit 前視點，也不得觸發「急彎時繼續前進」。

### `LOST`

最低語意：

- 近場中心線不存在或不可信。
- 候選與已追蹤路徑不連續。
- 僅剩紙板／地板／牆面邊緣。
- 路徑主要由推測 bridge 構成。
- 分割或幾何階段明確拒絕結果。

控制權限：

```text
forward = 0
right = 0
yaw = 0
```

進入重新取得流程，不保留上一筆移動命令。

---

## 第七層：相機姿態與地面投影

影像中心線可以在未知姿態下用於 UI 診斷，但 Pure Pursuit 需要地面座標。第一個安全版本應要求：

- gimbal yaw 已命令並 read-back 為 `0°` 附近。
- gimbal pitch 已命令並 read-back 為 `-90°` 附近。
- 飛行高度有效且在允許範圍。
- 相機串流與姿態資料時間差在允許範圍。

條件未成立時：

- 可以顯示影像空間中心線。
- 不得將其標為可供地面 Pure Pursuit 使用的 `FULL_PATH`。
- 不得在 `cameraPitch=unknown` 時默認為垂直俯視。

若未來支援 `-60°`：

1. 取得相機內參。
2. 使用機身姿態、gimbal yaw/pitch 與飛行高度。
3. 建立影像到地平面的 homography 或射線／地平面交點。
4. 將中心線點轉為機身座標後再計算 Pure Pursuit 曲率。

不能直接用目前垂直俯視公式處理 `-60°` 或未知 pitch。

---

## 視覺與控制的狀態對應

```text
FULL_PATH
→ Pure Pursuit 是主要 yaw／路徑控制器
→ 近場角度不以相近權重持續相加

NEAR_FIELD_ONLY
→ 停止 forward 與 right
→ 使用近場角度低速原地對準
→ 等待完整前視路徑恢復

LOST
→ 所有運動命令歸零
→ 進入重新取得
```

這份契約必須由單一狀態轉換點實作，不能讓 UI、detector 與 controller 各自推導不同品質狀態。

---

## 終點、畫面截斷與閉環

不能再只以「路徑變短」判定物理終點。至少要區分：

### 真實終點候選

- 遠端中心線 terminus 位於畫面內，距離各邊界超過基於局部膠帶寬度的 margin。
- 近場仍有膠帶支持。
- 遠端寬度沒有縮成髮絲或不合理尖端。
- 終點位置連續多幀穩定。

### 畫面外截斷

- 中心線遠端接近畫面邊界。
- 代表路徑可能仍在畫面外延續，不得觸發 180° 回轉。

### 分割失敗或反光缺口

- 沒有完整遠端 terminus 證據。
- 應降級、懸停或重新取得，不得解釋為終點。

### 閉合環

- 骨架沒有自然 endpoint，拓撲判定為 closed loop。
- 不觸發終點回轉。

若出現可信分支，預設 fail-safe；在尚未定義岔路選擇規則前，不得靜默選任一分支繼續飛行。

---

## 診斷與證據輸出

每次進入 `NEAR_FIELD_ONLY`、`LOST` 或重新取得失敗時，應保存足以離線重播的原始資料，而不是只保存帶 UI 疊層的螢幕錄影。

建議保存：

- 原始 RGBA／YUV 影格。
- frame timestamp。
- gimbal yaw/pitch 及其 timestamp。
- 飛行高度與姿態。
- `rawDarkMask`。
- `boardMask`。
- `appearanceCandidateMask`。
- `bridgeProposalMask`。
- `acceptedPathMask`。
- 所有候選元件及拒絕原因。
- 最終中心線、anchor、lookahead。
- confidence components。
- topology。
- `PathQuality` 與狀態轉換原因。

日誌不得只寫 `black tape not detected`。至少要能指出失敗階段：

```text
NO_BOARD_BACKGROUND
NO_APPEARANCE_CANDIDATE
NO_NEAR_FIELD_COMPONENT
NO_CENTERLINE
BRIDGE_REJECTED
WIDTH_INCONSISTENT
BOARD_SIDE_UNSUPPORTED
TEMPORAL_DISCONTINUITY
AMBIGUOUS_BRANCH
INSUFFICIENT_LOOKAHEAD
CAMERA_POSE_UNAVAILABLE
```

實際 enum 應重用主專案既有 rejection/state 型別並補足缺口，不建立重複名稱。

---

## 實作順序

### 階段一：固定輸入與可重播證據

1. 在 detector 輸入邊界保存無 UI 疊層的原始影格。
2. 保存目前所有中間遮罩與偵測結果。
3. 從已知失敗影片的對應時刻建立固定回歸素材。
4. 建立離線執行器，確保同一影格可重現同一結果。

完成條件：每個既知失敗點都能由原始影格重播，不需要靠螢幕錄影目視猜測。

### 階段二：移植資料型別與中心線抽取

1. 移植 `SegmentationMask`、tape width 型別及必要的純資料類別。
2. 移植 `CenterlineExtractor` 及原有單元測試。
3. 以目前 `BlackTapeDetector` 的最終 mask 暫時餵入新 extractor，建立新舊中心線的 A/B 診斷。
4. 此階段不改控制輸出。

完成條件：主專案中心線測試在 Lite 專案通過；真實回歸影格可輸出有序中心線與 topology。

### 階段三：替換分割輸入

1. 移植 board-relative 分割語意。
2. 使用 OpenCV 加速色彩轉換、遮罩及元件運算，但維持相同判斷契約。
3. 保留原始遮罩與 bridge proposal 的來源區分。
4. 將既有黑膠帶、灰地板、紙板邊緣、深色物件及強反光素材加入回歸。

完成條件：不得降低現有已通過場景的辨識能力；已知灰地板／紙板邊緣誤判仍被拒絕。

### 階段四：受約束 bridge 與跨幀關聯

1. 將 morphology 結果改為 proposal。
2. 加入寬度、端點切線、曲率、紙板兩側與歷史走廊驗證。
3. 保存上一筆確認中心線，而非只保存 bounds。
4. 加入重新取得的連續確認規則。

完成條件：反光短暫切斷時能維持同一路徑或安全降級；不得連接無關暗邊。

### 階段五：導入品質契約

1. 產生 `FULL_PATH`、`NEAR_FIELD_ONLY`、`LOST`。
2. 將 UI 疊層及 flight log 改為顯示品質與拒絕原因。
3. 控制器先以 shadow mode 記錄「若套用新契約會輸出什麼」，不立即驅動真機。
4. 對比現行控制輸出與新契約。

完成條件：所有視覺輸出都有明確品質；沒有 null、fallback 或最後命令可以繞過品質閘門。

### 階段六：切換控制器

1. `FULL_PATH` 才允許 Pure Pursuit。
2. `NEAR_FIELD_ONLY` 停止平移並原地對準。
3. `LOST` 全部歸零並重新取得。
4. 移除舊 row/column、`HORIZONTAL_FALLBACK` 及雙 yaw 相加的控制路徑。
5. 相機姿態 read-back 不成立時拒絕地面 Pure Pursuit。

完成條件：所有舊 caller 已遷移，無相容 shim、雙軌 detector 或舊 fallback 殘留。

---

## 驗證計畫

### 單元測試

至少覆蓋：

1. 直線、左彎、右彎、U 型與閉合環的有序中心線。
2. 路徑方向不受水平／垂直／對角方向影響。
3. 畫面內終點與畫面邊界截斷的區分。
4. 細小 skeleton spur 不被當成可信分支。
5. 真實岔路輸出 ambiguous branch。
6. 有限反光缺口可在全部條件成立時連接。
7. 寬度不符、切線不連續或紙板兩側不足時拒絕 bridge。
8. 歷史路徑不能在沒有新影像支持時維持 `FULL_PATH`。
9. 短近場片段只能得到 `NEAR_FIELD_ONLY`。
10. 無近場證據得到 `LOST`。
11. 未知 gimbal pose 不得產生可控制的地面路徑。

### 錄製影格回歸

使用無 UI 疊層的原始影格，涵蓋：

- 目前完整繞圈影片的正常區段。
- 每一個反光中斷前、當下及恢復後影格。
- 灰地板、紙板邊緣與牆邊暗線。
- 短水平殘片。
- 路徑接近畫面左／右邊界。
- 終點、回轉後重新取得。
- 人工改變 gimbal pitch 前後。

每筆素材應驗證最終 `PathQuality`、中心線 topology、bridge 接受／拒絕原因，而不是只驗證「非 null」。

### Android 真機影像驗證

在不啟動槳的情況下，以真實 DJI 串流確認：

- OpenCV Android runtime 載入成功。
- 每幀處理時間及配置是否符合目標更新率。
- 長時間處理無持續 Mat／ByteArray 配置與 native memory 成長。
- 原始影格、遮罩與中心線 timestamp 對齊。
- gimbal pose 不符合時品質正確降級。

### 飛行驗證順序

1. 槳關閉／地面影像 shadow mode。
2. 懸停但控制輸出保持 neutral，只記錄建議命令。
3. 低速直線，驗證品質切換會立即停止平移。
4. 一般彎道。
5. 已知反光區。
6. 完整圓環。
7. 終點回轉與重新取得。

任何階段若出現 `LOST` 後仍有非零平移命令，視為阻斷缺陷，不進入下一階段。

---

## 效能與記憶體要求

影像處理是固定頻率長時間執行路徑，應避免每幀配置大型物件。

- 重用 OpenCV `Mat`、kernel 與 byte buffers。
- 重用中心線 extractor 的 primitive workspaces。
- 不在每幀建立大量 `List`、臨時 contour copies 或全尺寸遮罩副本；診斷保存只在事件觸發時複製。
- 明確 `release()` 不再使用的 native `Mat`。
- OpenCV 與 Kotlin 陣列間的轉換只放在清楚的階段邊界，避免反覆來回複製。
- 在真機量測處理延遲、allocation 及 native heap，再決定是否將 distance transform 等步驟改為 OpenCV；不可只憑假設最佳化。

標準 OpenCV Android AAR 不應假設包含 `opencv_contrib` 的 `ximgproc.thinning`。若 API 不存在，沿用主專案的 Zhang–Suen 純 Kotlin實作，不為此新增另一個大型依賴。

---

## 主要風險

### 1. 移植後短期內新舊輸出不同

這是預期結果，但不能直接在真機控制中切換。先以 shadow mode 比較中心線、品質與建議命令，再移除舊路徑。

### 2. Morphology 過度連接

透過來源分離、bridge 幾何驗證及修補比例限制處理。不能只縮小 kernel，因為那只會讓反光問題重新出現。

### 3. 時間關聯變成盲目追蹤

每個 `FULL_PATH` 必須有本幀直接影像支持；歷史只增加或降低候選可信度，不能替代本幀觀測。

### 4. 固定 `-90°` 相機降低操作彈性

這是第一版安全約束。支援斜視相機需要正確 ground projection，不應在缺少內參與姿態驗證時提前開放。

### 5. 同時保留兩套 detector 造成長期分歧

A/B 階段只用於驗證。切換完成後應刪除舊 extractor、舊 fallback、過時常數及其測試，不提供永久 compatibility mode。

---

## 非目標

本次不包含：

- 任意材質、任意顏色的通用道路辨識。
- 深度學習模型或新增神經網路 runtime。
- 未校正相機下的完整 3D 場景理解。
- 自動選擇岔路方向。
- 在沒有本幀影像證據時預測並持續前進。
- 以放寬安全門檻換取表面上的高偵測率。

---

## 完成定義

視覺重構只有在以下條件全部成立時才算完成：

1. Lite 使用方向無關的有序中心線作為唯一視覺路徑表示。
2. OpenCV 僅產生及處理候選影像證據，不直接決定控制路徑。
3. 原始像素與修補像素的支持比例可觀察。
4. 反光缺口只能經受約束 bridge 驗證連接。
5. 上一幀完整中心線用於關聯，但無新證據時不會盲飛。
6. 視覺明確輸出 `FULL_PATH`、`NEAR_FIELD_ONLY`、`LOST` 或等價型別。
7. 只有完整可信路徑能啟用 Pure Pursuit。
8. 短片段不能觸發前進或急彎控制。
9. 真實終點、畫面截斷、分割失敗與閉合環可以區分。
10. 未知或未確認的相機姿態不能啟用地面 Pure Pursuit。
11. 既知反光、灰地板、紙板邊緣、完整彎道及終點素材均有固定回歸。
12. 真機發生 `LOST` 時，下一個控制週期內所有平移輸出歸零。
13. 舊 row/column 特殊路徑、`HORIZONTAL_FALLBACK` 及永久雙 detector 已移除。

最終系統應回答的問題不再是：

> 這一塊黑色區域看起來像不像膠帶？

而是：

> 本幀的直接影像證據，能否與可信紙板背景、膠帶寬度、幾何拓撲及跨幀連續性共同構成一條從飛機近場開始、足以安全控制的有序路徑？
