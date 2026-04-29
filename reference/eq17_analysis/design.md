# Design: Eq.17 控制律 + 5-state Reduced EKF

**Branch**: `test/eq17-5state-ekf`
**Base**: `origin/main` (5681455)
**Date**: 2026-04-28
**Status**: Initial design snapshot — captures discussion accumulated up to branch creation

本檔案是新分支的 entry-point design，記錄當前對「paper 2023 Eq.17 控制律 + reduced-state EKF」這條主線的設計決策。後續實作前若有重大設計變更，應新增 `design_v2.md`，本檔不刪。

---

## 1. Motivation 與 Scope

### 為什麼從 paper 2025 風格切到 paper 2023 Eq.17 + 自設 7-state

qr 分支上的 EKF 採 paper 2025 Meng/Long/Menq 的「Eq.6 控制律 + 7-state EKF」組合，累積了已知的 Q/R 設計問題：

| 問題（qr branch parameter audit） | 來源 |
|---|---|
| R(1,1) 非 per-axis（260×–30000× 過估） | EKF 把全軸共用一個 R |
| Q(3,3) linear vs quadratic in a_r 模糊 | δx 三胞胎 + thermal 模型耦合 |
| Q(7,7) = 1e-8 vs 0 frozen behavior 衝突 | δa_x random walk 與 a_x random walk 互相牽動 |
| a_hat std x-axis 5–7× under-prediction | per-axis EKF coupling 未解 |

本路線**換控制律但保留 7-state estimator**：採 paper 2023 Eq.(17) 的 d-step delay-compensated 控制律（不需要估 δx，自帶延遲補償），estimator 維持 7-state integrated random walk 結構（[δx 三胞胎, x_D, δx_D, a_x, δa_x]）以利平滑追蹤動態 a_x(t)。控制律與 estimator 解耦後可重新設計 Q/R，繞過上述 audit 問題。

### Scope 範圍

In scope：
- Plant 模型不變（paper 2023 Eq.13–14）
- 控制律：paper 2023 Eq.(17) d-step delay-compensated 形式
- Estimator: 7-state integrated random walk，position+velocity Markov for both x_D 與 a_x
- LTV observability 分析
- Q/R 第一性推導（不靠試誤）
- 時變矩陣處理（F_e[k]、Q[k]、R[k] 全部 [k]-indexed）

Out of scope：
- 殘磁/磁滯補償（simulation 環境無此項）
- 退化版本（5-state 1st-order random walk）— 已驗證可觀但對動態軌跡有 lag，作為 task01 歷史記錄保留

---

## 2. Plant 模型

採用 1-D 形式（paper 2023 §III.A）：

```
x[k+1] = x[k] + a_x[k] · f_d[k] + a_x[k] · f_T[k] + x_D[k]
```

其中：
- `a_x[k] = Δt / γ(h[k]/R)` — motion gain（時變，未知）
- `γ(h/R) = γ_N · C(h/R)` — 與壁面距離相關的拖曳係數
  - `γ_x = γ_y = γ_N · C_∥(h/R)`
  - `γ_z = γ_N · C_⊥(h/R)`
- `f_T[k]` — thermal force, 白噪聲，方差 `σ²_T = 4 k_B T γ / Δt`
- `x_D[k]` — lumped disturbance（位置單位），吸收所有低頻偏差

**量測**：
```
y[k] = δx_m[k] = δx[k − d] + n_x[k],   d = 2
```
2-step delay 來自 vision-based tracking pipeline（paper 2023 §III.A 描述）。

---

## 3. 控制律：Paper 2023 Eq.(17)

### 公式

```
f_d[k] = (1/â_x[k]) · { x_d[k+1] − λ_c · x_d[k] − (1−λ_c) · x_d[k−d]
                       + (1−λ_c) · δx_m[k] − x̂_D[k] }
```

### 各項用途分類

| 項 | 角色 | 估測依賴 |
|---|---|---|
| `1/â_x[k]` | 增益反向補償（讓控制力被 plant 的 a_x 抵消後留下乾淨閉迴路） | â_x（必須估） |
| `x_d[k+1]` | one-step preview feedforward | — |
| `−λ_c · x_d[k]` | closed-loop pole 設計 | — |
| `−(1−λ_c) · x_d[k−d]` | d-step 延遲補償的核心：與 δx_m[k] 配合產生 δx[k+1] = λ_c·δx[k] − ε[k] | — |
| `(1−λ_c) · δx_m[k]` | feedback，使用**延遲量測直接餵回** | — |
| `−x̂_D[k]` | lumped disturbance 補償（取代 paper 2023 原公式的 `−(1−λ_c)·Σf_T[k−i]`） | x̂_D（必須估） |

### 為什麼不需要 δx̂

paper 2023 Eq.(17) 的核心優雅之處：**用延遲量測 `δx_m[k]` 直接餵回，不必估「當前」δx[k]**。代入 plant 後的代數運算（Eq.18 → Eq.19）會自動產生：

```
δx[k+1] = λ_c · δx[k] − ε[k]
```

其中 ε[k] 是零均值白噪組合（thermal + measurement）。這代表**控制律本身內建延遲補償**，不需要在 estimator 估 δx[k]，省下 paper 2025 風格的 δx 三胞胎（在控制律使用層面）。

### x̂_D 補償項的位置設計

把 `−x̂_D` 放在大括號**內**（與其他項共享 `1/â_x` 前因子），而非放在大括號**外**做獨立力補償，理由：

- 確保補償量單位一致（位置單位 → 透過 `1/â_x` 轉成力單位）
- 線性化後 `(a_x/â_x) − 1 ≈ ẽ_a/â_x` 直接乘整個大括號，把 ẽ_a 和 ẽ_xD 的耦合保持線性，便於 EKF 線性化

---

## 4. State Vector 推導

### 設計原則

> 「要算 Eq.17 的 f_d[k]，每一個未知量都是 KF 的 state 候選。不需要估的、可量測的、可查表的都不該進 state vector。」

### 候選分析

| 量 | 屬性 | 是否需要估 |
|---|---|---|
| `x_d[k+1], x_d[k], x_d[k−d]` | reference | ✗ 已知 |
| `δx_m[k]` | 量測 | ✗ 直接讀 |
| `a_x[k]` | 時變 motion gain | **✓ 需估** |
| `Σ f_T[k−i]` | 過去 d 步 thermal 史 | ✗ 白噪聲零均值，吃進 ε[k] |
| `x_D[k]` | lumped disturbance | **✓ 需估** |

對動態軌跡（如 1Hz osc 上的平滑 a_x 變化），motion gain 與 disturbance 用**整合隨機漫步（integrated random walk）**模型 — 即 [位置, 速度] 兩個 state — 比純隨機漫步更貼合物理。位置-速度模型的「速度」可預測平滑變化，避免 KF 跟蹤滯後。

加上量測方程的延遲約束（`y[k] = δx[k−2] + n_x`）需要 δx 三胞胎做 buffer，最終 state vector：

```
x_e[k] = [ δx_1[k], δx_2[k], δx_3[k], x_D[k], δx_D[k], a_x[k], δa_x[k] ]ᵀ      (7 維)
```

| Index | State | 物理意義 | 隨機模型 |
|---|---|---|---|
| 1 | `δx_1[k]` | δx[k−2]（最舊，量測對應） | (deterministic shift) |
| 2 | `δx_2[k]` | δx[k−1] | (deterministic shift) |
| 3 | `δx_3[k]` | δx[k]（當前） | closed-loop dynamic |
| 4 | `x_D[k]` | lumped disturbance（位置單位） | x_D[k+1] = x_D[k] + δx_D[k] |
| 5 | `δx_D[k]` | x_D 增量速率 | random walk |
| 6 | `a_x[k]` | motion gain | a_x[k+1] = a_x[k] + δa_x[k] |
| 7 | `δa_x[k]` | a_x 增量速率 | random walk |

### State 命名約定

- `δx_1[k] = δx[k−2]` ← 量測對應位置（最舊）
- `δx_2[k] = δx[k−1]`
- `δx_3[k] = δx[k]`     ← 當前

注意：本檔約定與 paper 2025 寫作慣例**相反**（paper 把當前狀態放第 3 位、最舊放第 1 位）。實作上保持本檔約定，避免與 H 矩陣對應位置混淆。

### 隨機模型

```
δx_D[k+1] = δx_D[k] + w_xD[k]                (純白噪驅動 x_D 速率)
x_D[k+1]  = x_D[k] + δx_D[k]                  (x_D 由速率累積)

δa_x[k+1] = δa_x[k] + w_a[k]                  (純白噪驅動 a_x 速率)
a_x[k+1]  = a_x[k] + δa_x[k]                  (a_x 由速率累積)
```

對應到物理：a_x(t) = Δt/{γ_N · C(h(t)/R)} 對 1Hz osc 是平滑訊號，δa_x 是有意義的「a_x 變化率」。Q 矩陣只需驅動 δa_x（和 δx_D），主 state 方程是純積分，Q_aa = Q_xD,xD = 0。

---

## 5. F_e 矩陣推導

### Row 1, 2：delay buffer

```
δx_1[k+1] = δx_2[k]              → F_e(1,2) = 1
δx_2[k+1] = δx_3[k]              → F_e(2,3) = 1
```

### Row 3：closed-loop tracking dynamics

從 plant 出發：
```
δx[k+1] = δx[k] + (x_d[k+1] − x_d[k]) − a_x · f_d − a_x · f_T − x_D
```

代入控制律後（在 â_x = a_x、x̂_D = x_D 處線性化）：
```
δx[k+1] = λ_c · δx[k]                              ← paper Eq.18 主項
        − (a_x − â_x) · f_d[k]                      ← gain estimation 誤差
        − (x_D − x̂_D)                              ← disturbance estimation 誤差
        − a_x · f_T − (1−λ_c) · n_x                ← noise
```

對 state 線性化（δx_D 與 δa_x 不直接進入 δx[k+1]，只透過下一步 x_D, a_x 的更新間接影響）：
```
∂δx_3[k+1]/∂δx_3[k] = λ_c
∂δx_3[k+1]/∂x_D[k]  = −1
∂δx_3[k+1]/∂a_x[k]  = −f_d[k]
∂δx_3[k+1]/∂δx_D[k] = 0
∂δx_3[k+1]/∂δa_x[k] = 0
其他 = 0
```

→ Row 3：`F_e(3,:) = [ 0, 0, λ_c, −1, 0, −f_d[k], 0 ]`

**附註：上述為 Eq.19 form 結果，並非直接 Jacobian**。直接代入 Eq.(17) 進 plant 先得 **Eq.18 form**：
```
δx[k+1] = δx[k] − (1−λ_c)·δx[k−d] − (1−λ_c)·n_x[k] − a_x·f_T[k]
```
（regulation 簡化下；F_e Row 3 在 Eq.18 form 為 `[−(1−λ_c), 0, 1, −1, 0, −f_d, 0]`）

再用代數重排 `δx[k] − (1−λ_c)·δx[k−d] = λ_c·δx[k] + (1−λ_c)·{δx[k] − δx[k−d]}` 並把括號項用 Eq.18 遞迴展開吸收進 ε[k]，得到 **Eq.19 form** `δx[k+1] = λ_c·δx[k] − ε[k]`，其中 ε 變 MA(2) cross-step correlated（見 §8.2 Step 1）。

兩 form 描述同一閉迴路，但 representation 不同：
- Eq.18：noise 白，但 n_x 同進 Q 與 R（Q-R cross 違反）
- **Eq.19**：noise 切割乾淨（Q 拿白部分、R 拿 n_x），但 ε 非白（KF P 預測 ~50% 真實 σ²_δx）

design.md 全程採 **Eq.19 form**，trade-off 詳見 §8.2 Step 5。

### Row 4, 5：x_D 與 δx_D（integrated random walk）

```
x_D[k+1]  = x_D[k] + δx_D[k]    → F_e(4,4) = 1, F_e(4,5) = 1
δx_D[k+1] = δx_D[k] + w_xD[k]   → F_e(5,5) = 1
```

### Row 6, 7：a_x 與 δa_x（integrated random walk）

```
a_x[k+1]  = a_x[k] + δa_x[k]    → F_e(6,6) = 1, F_e(6,7) = 1
δa_x[k+1] = δa_x[k] + w_a[k]    → F_e(7,7) = 1
```

### 完整 F_e（7×7 時變）

```
            δx_1  δx_2  δx_3  x_D   δx_D  a_x   δa_x
        ┌                                                ┐
δx_1   │   0     1     0     0     0     0     0         │
δx_2   │   0     0     1     0     0     0     0         │
δx_3   │   0     0    λ_c   −1     0    −f_d   0         │   ← 時變
x_D    │   0     0     0     1     1     0     0         │
δx_D   │   0     0     0     0     1     0     0         │
a_x    │   0     0     0     0     0     1     1         │
δa_x   │   0     0     0     0     0     0     1         │
        └                                                ┘
```

**唯一時變項**在 (3,6) = `−f_d[k]`。其他 48 個元素皆為常數。

### F_e_state vs F_e_err（重要觀察）

對 paper 2023 Eq.(17) 控制律（用 δx_m 直接 feedback），**state 系統 F_e 與 error 系統 F_e_err 完全相同**：

從 plant + 控制律得：δx[k+1] = λ_c·δx[k] − ε[k]
KF predict：δx̂[k+1|k] = λ_c·δx̂[k|k]
誤差動態：e_δx[k+1] = λ_c·δx[k] − ε[k] − λ_c·δx̂[k|k] = λ_c·e_δx[k] − ε[k]

→ F_e_err(3,3) = λ_c（與 F_e_state 相同）。

對比 paper 2025 Eq.(6) 控制律（用 δx̂ feedback），其 F_e_err(3,3) = 1（兩個 λ_c 抵銷），需與 F_e_state 區分。

**含義**：本主線 error 系統內含 λ_c<1 的衰減 → 不靠 KF 增益 L 都會收斂；paper 2025 error 系統完全靠 KF 收斂。

---

## 6. 量測架構（雙回授，含 a_xm 2-step 延遲）

KF 使用兩條量測通道：raw 延遲量測 + IIR 推導的 motion gain 量測。

```
y[k] = [ δx_m[k] ]      ← raw 延遲量測：δx_m[k] = δx[k−d] + n_x[k]
       [ a_xm[k]  ]      ← IIR 推導：由 σ²_δxr 反解的 motion gain
```

### a_xm 內生延遲：a_xm[k] 對應 a_x[k−d]

paper 2025 §II.F 末尾明示：a_xm[k] 實際對應 a_x[k−2]（不是 a_x[k]）。原因：
- δx_m[k] 本身延遲 d=2 步（量測管線）
- IIR 處理 δx_m 來算 σ²_δxr，繼承同樣延遲

正確處理：把 a_x[k−d] 用當前 state 線性表達。從 integrated random walk model 反推：
```
a_x[k−1] = a_x[k] − δa_x[k] + w_a[k−1]
a_x[k−2] = a_x[k] − 2·δa_x[k] + 2·w_a[k−1] + w_a[k−2]
```

對 d=2 推廣為一般式（d 步反推）：
```
a_x[k−d] = a_x[k] − d·δa_x[k] + Σ_{j=1}^{d} (d−j+1) · w_a[k−j]
                                └────────┬─────────┘
                                  過去 process noise，非當前 state
```

### 量測方程（含延遲）

```
y_2[k] = a_x[k−d] + n_a[k]
       = a_x[k] − d·δa_x[k] + (Σ_{j=1}^{d} (d−j+1)·w_a[k−j]) + n_a[k]
         └──────────┬──────────┘    └────────────┬────────────┘  └─┬─┘
              current state                    past process noise   sensor
```

### H 矩陣（含延遲修正）

```
H = [ 1  0  0  0  0   0    0 ]    ← y_1: δx_m → δx_1 = δx[k−2]
    [ 0  0  0  0  0   1   −d ]    ← y_2: a_xm → a_x − d·δa_x
```

對 d=2：H(2,7) = `−2`。

### Effective R_2（包含延遲傳導項）

過去 process noise 項 `Σ(d−j+1)·w_a[k−j]` 對 KF 是未追蹤的隨機輸入，要併入 effective measurement noise：

```
R_2_effective = R_2_intrinsic + Σ_{j=1}^{d} (d−j+1)² · Q77

對 d=2:  R_2_effective = R_2_intrinsic + (4 + 1)·Q77 = R_2_intrinsic + 5·Q77
```

R_2_intrinsic 是 IIR 推導的 a_xm 內生雜訊（Task 03 推導）。

### 對可觀性的影響

H_2 = [0,...,1,−d]，繞 F_e 累乘時 δa_x 係數每步 +1：m=0 是 −d，m=1 是 −(d−1)，… 連續量測差直接給 δa_x。**rank=7 仍成立**，無 PE 需求。

### IIR 不在控制 feedback 路徑上

控制律 Eq.(17) 中 `δx_m[k]` 用**原始量測**（不是 IIR 過濾後）。理由：
- IIR 動態若進入 closed-loop，會破壞 paper Eq.18 的 λ_c·δx[k] 形式
- IIR 純粹當 estimator 的 pre-processor，產出 a_xm 餵進 KF measurement update

### a_xm 由 IIR 推導（paper 2025 Eq.9–13，**線性形式**）

```
δx̄_m[k+1]   = (1−a_var)·δx̄_m[k] + a_var·δx_m[k+1]                         (Eq.9 LP mean)
δx_r[k]     = δx_m[k] − δx̄_m[k]                                            (random component)
σ̂²_δxr[k+1] = (1−a_cov)·σ̂²_δxr[k] + a_cov·(δx_r²[k+1] − δx̄_r²[k+1])         (Eq.10 variance)

a_xm[k] = (σ̂²_δxr[k] − C_n·σ²_n_s) / (C_dpmr · 4kBT)                       (Eq.13)

其中（粗糙版）：
  C_dpmr = 2 + 1/(1−λ_c²)
  C_n    = 2/(1+λ_c)
```

**注意**：a_xm 公式是**線性反解**（σ²_δxr 是 a_x 的線性函數，所以反解也線性）。**不是 sqrt**。

a_xm 視為 a_x 的「噪聲量測」（含 d 步延遲傳導），加入 KF 後 a_x 與 δa_x 都可由量測差分識別。

---

## 7. Observability 分析框架

### LTV 觀測矩陣（標準流程）

對窗口 `[k_0, k_0+N−1]`：
```
       ┌ H · Φ(k_0,   k_0) ┐    ┌ H               ┐
O   =  │ H · Φ(k_0+1, k_0) │  = │ H · F_e[k_0]    │
       │ H · Φ(k_0+2, k_0) │    │ H · F_e[k_0+1]·F_e[k_0] │
       │ ⋮                  │    │ ⋮                │
       └ H · Φ(k_0+N−1, k_0)┘   └ ⋮                ┘
```

State transition matrix（**新時間在左**）：
```
Φ(k_0+m, k_0) = F_e[k_0+m−1] · F_e[k_0+m−2] · ⋯ · F_e[k_0]
```

判斷準則：rank(O) = n_state = 7 ⇔ 在該窗口可觀。

### Window length

- m_out = 2（雙量測），n_state = 7 → N ≥ ⌈7/2⌉ = 4，**保險取 N = 5–7**
- 多出兩個速度 state（δx_D, δa_x），需要多一步觀測才能識別 δx_D（m=4 row）

### 結論：rank(O) = 7 與 f_d 無關

| State | 識別來源 | 需要 f_d ≠ 0？ |
|---|---|---|
| δx_1, δx_2, δx_3 | y_1 前三步直接量 | ✗ |
| a_x | y_2 任一步直接量 | ✗ |
| **δa_x** | y_2 連續兩步差（量到 a_x + δa_x − 量到 a_x） | **✗** |
| x_D | y_1 第 m=3 步觀測扣 δx_3 與 a_x（係數 −1 與 f_d 無關） | ✗ |
| **δx_D** | y_1 第 m=4 步觀測扣已識別量（係數 −1） | **✗** |

→ **7 個 state 全部可獨立識別，f_d 任何行為（包括 hold 階段 f_d=0）下都成立**

完整逐步推導與 MATLAB rank 驗證見 [task01_math_observability_report.md](task01_math_observability_report.md)。

### 「永遠可觀」要分兩層

| 層次 | 結論 | 條件 |
|---|---|---|
| 數學 rank | 永遠 = 7 | a_xm 量測存在且模型有效 |
| 實務 Gramian λ_min | 視 R_2[k] 而定 | warm-up 過後 + 訊噪比夠 + a_x 變化夠慢 |

### IIR 崩塌條件（a_xm 暫時失效時）

當下列任一條件發生，y_2 訊息品質急遽下降，需要將 R_2[k] → ∞ 把該量測暫時關掉：

1. **Warm-up 期**（前 ~50 ms @ a_var=0.05）：σ²_δxr 還沒收斂
2. **Thermal-to-noise 比例太低**：分子 `σ²_δxr − C_n·σ²_n` < 0 → a_xm = NaN
3. **a_x 變化快過 IIR 窗**：a_xm 有延遲與 bias
4. **接近 wall（h_bar ~ 1.1）**：C_⊥ lookup 誤差大，a_xm 模型失準

→ 設計時 R_2[k] 必須**自適應**。實務 Gramian σ_min(t) 的數值評估屬於 Task 02 範疇。

---

## 8. Q 矩陣推導（unified version）— Task 02

### 8.1 Q 矩陣的稀疏結構與對角性

state vector `[δx_1, δx_2, δx_3, x_D, δx_D, a_x, δa_x]ᵀ` 中，shift state（δx_1, δx_2）與 integrator state（x_D, a_x）是**確定性映射**，沒有驅動噪聲，Q 對角項為 0。**只有三個非零 Q 項**：Q33（δx_3）、Q55（δx_D）、Q77（δa_x）。

```
       δx_1  δx_2  δx_3       x_D  δx_D    a_x  δa_x
   ┌                                                          ┐
   │  0     0     0           0    0       0    0             │
   │  0     0     0           0    0       0    0             │
   │  0     0    Q33,i[k]    0    0       0    0             │   ← Path C strict (Eq.19 form)
Q_i = │  0     0     0           0    0       0    0             │
   │  0     0     0           0   Q55      0    0             │   ← simulation 設 0
   │  0     0     0           0    0       0    0             │
   │  0     0     0           0    0       0    Q77,i[k]      │   ← adaptive
   └                                                          ┘
```

per-axis i ∈ {x, y, z}（三個獨立 KF）。

**Off-diagonal 嚴格 0** 的前提（A1–A4 四個獨立性假設）：
1. f_T 與 n_x 獨立（thermal vs sensor，物理隔離）
2. w_xD 與 (f_T, n_x) 獨立（modeling 選擇）
3. w_a 與 (f_T, n_x, w_xD) 獨立（modeling 選擇）
4. 三者皆白噪聲（時間上獨立）

對 simulation 場景全部成立。

**⚠️ 空間 vs 時間 cross 區分**：single-time Q 的 off-diagonal = 0，但 ε[k] 在**時間上** MA(d) 自相關 ρ(1)≈33%, ρ(2)≈25%（由 Eq.17 控制律的 Σf_T 結構造成）。後者由 Q33 的 Path A′ inflation 部分補償（見 §8.2）。

### 8.2 Q33[k] — 嚴格 KF formalism 推導（Path C）

#### Step 1：ε[k] 完整形式（paper 2023 Eq.19）

Eq.(17) 控制律代入 plant 後：
```
δx[k+1] = λ_c · δx[k] − ε[k]
ε[k] = a_x · f_T[k]                       ← (i) 當步 thermal
     + (1−λ_c) · a_x · f_T[k−1]            ← (ii) 上一步 thermal 殘留
     + (1−λ_c) · a_x · f_T[k−2]            ← (iii) 上上步 thermal 殘留
     + (1−λ_c) · n_x[k]                    ← (iv) sensor noise 注入
```

#### Step 2：KF 對 Q 的數學要求

KF formalism：
```
x[k+1] = F·x[k] + w[k],     E[w[k]·w[j]ᵀ] = Q · δ(k−j)     ← 白噪 cross-step uncorrelated
y[k]   = H·x[k] + v[k],     E[w[k]·v[k]ᵀ]  = 0              ← Q 與 R 無 cross
```

進 Q(3,3) 的「資格」必須**全數通過**五個條件：
1. 進 δx 動態
2. **當步**（不是過去步）
3. **白噪**（cross-step 獨立）
4. exogenous（不是其他 state propagation）
5. **未在 R 中**（避免 Q-R cross）

#### Step 3：對 ε[k] 四個 component 逐項過濾

| 項 | 進 δx | 當步 | 白 cross-step | exog | 不在 R | 進 Q? |
|---|---|---|---|---|---|---|
| (i)   `a_x·f_T[k]` | ✓ | ✓ | ✓ | ✓ | ✓ | **✓** |
| (ii)  `(1−λ_c)·a_x·f_T[k−1]` | ✓ | ✗ 上一步 | ✗ 跨步相關 | ✓ | ✓ | ✗ |
| (iii) `(1−λ_c)·a_x·f_T[k−2]` | ✓ | ✗ 上上步 | ✗ 跨步相關 | ✓ | ✓ | ✗ |
| (iv)  `(1−λ_c)·n_x[k]` | ✓ | ✓ | ✓ | ✓ | **✗ 已在 R** | ✗ |

→ **只有 (i) 通過**。

#### Step 4：(ii)(iii)(iv) 不消失，只是不在 Q

| 項 | 在哪裡被處理 |
|---|---|
| (ii) f_T[k−1] 殘留 | paper 2023 Eq.(22) 閉迴路 σ²_δx 公式（透過 closed-loop 累積） |
| (iii) f_T[k−2] 殘留 | 同上 |
| (iv) n_x | R(1,1) = σ²_n_s（KF measurement update 自然納入） |

#### 最終公式

```
┌──────────────────────────────────────────────────────────────┐
│  Q33,i[k] = Var( a_x,i[k] · f_T,i[k] )                       │
│           = a_x,i²[k] · σ²_fT,i[k]                            │
│           = 4·k_B·T · â_x,i[k]      (linear in â_x，per-axis) │
└──────────────────────────────────────────────────────────────┘
```

**沒有 inflation factor，沒有 sensor noise 項，純粹當步 thermal 白噪變異數**。

#### Step 5：KF 行為的含義

```
σ²_δx,KF_預測 = Q33 / (1−λ_c²)   = 4kBT·â_x / (1−λ_c²)         ≈ 1.96·4kBT·â_x  (λ_c=0.7)
σ²_δx,真實    = (2 + 1/(1−λ_c²))·σ²_δxT + ...                    ≈ 3.96·4kBT·â_x  (paper Eq.22)

→ KF 預測 ≈ 50% 的真實值（KF 認為自己估得比實際更準）
```

這是 standard KF 處理 correlated process noise 的**已知 trade-off**：
- KF 預測 P(3,3) 偏小，但 KF 仍穩定收斂
- 實際 estimation 誤差不會差 50%，因為 KF 透過 measurement update 仍會修正
- 完全消除偏差**只能用 Path B**（augment state with f_T history → 9-state, Q non-diagonal）

#### 屬性

| 性質 | 值 |
|---|---|
| Per-axis | ✓ 透過 â_x,i |
| 時變 | ✓ 透過 â_x,i[k] |
| 是否 â_x 函數？ | ✓ Adaptive EKF / SDRE 結構 |
| KF formalism 嚴謹性 | ✓ 嚴格滿足白噪、Q-R 獨立 |
| KF P 預測 vs 真實 σ²_δx | △ 約 50%（standard KF correlated-noise trade-off） |

### 8.3 Q55 — Disturbance velocity（simulation 場景）

x_D 是 lumped disturbance（殘磁、磁滯、modeling error）。我們的 simulation **無這些干擾源**：

```
┌──────────────────────────────────────┐
│  Q55 = 0    (simulation 場景)         │
└──────────────────────────────────────┘
```

**實機部署 補丁**：若有殘磁，需從磁通變化率物理模型推。
**工程 safety**：實作可設 floor `Q55 = 1e-12` 避免數值鎖死，**不影響理論**。

### 8.4 Q77,i[k] — Adaptive，per-axis 時變（Path B 嚴格）

#### 物理推導鏈：從 Var(w_a) = Δt⁴·Var(ä_x)

KF integrated random walk 模型 `δa_x[k+1] = δa_x[k] + w_a[k]` 要求 `Q77 = Var(w_a)`。對 trajectory 真實訊號：
```
w_a_true[k] = δa_x_true[k+1] − δa_x_true[k]
            = a_x_true[k+2] − 2·a_x_true[k+1] + a_x_true[k]    (二階差分)
            ≈ Δt² · ä_x_true                                    (二階導)

Q77 = Var(w_a_true) = Δt⁴ · Var(ä_x_true)
```

**注意**：Q77 是 **Var(w_a) = Var(Δt²·ä_x)**，不是 Var(δa_x) = Var(Δt·ȧ_x)。差一階導 + 一個 Δt 因子。

#### Step 1：a_x 對 h̄ 的偏導
```
∂a_x/∂h̄  = − a_x · K_h(h̄)              K_h := (1/C)·(dC/dh̄)
∂²a_x/∂h̄² = a_x · ( K_h² − K_h' )       K_h' := dK_h/dh̄ = C''/C − K_h²
```

#### Step 2：時間二階導（鏈式 + product rule）
```
ȧ_x = − a_x · K_h · (ḣ/R)
ä_x = a_x · ( K_h² − K_h' )·(ḣ/R)²  −  a_x · K_h ·(ḧ/R)
      └────────────┬────────────┘     └─────────┬─────────┘
            Term A: ḣ² 主導                 Term B: ḧ 貢獻
```

#### Step 3：對 trajectory 算 Var(ä_x)（一週期平均）

對 sinusoidal `h(t) = h_0 + A·cos(ωt)`：
```
Var(ḣ²) = ḣ_max⁴/8        (cos²(ωt) 的 var)
Var(ḧ)  = ḧ_max²/2        (cos(ωt) 的 var)
Cov(ḣ², ḧ) = 0            (Fourier 正交：ḣ² 含 2ω 諧波，ḧ 含 ω 基頻)

Var(ä_x) = a_x² · (K_h² − K_h')² · ḣ_max⁴/(8·R⁴)
         + a_x² · K_h²            · ḧ_max² /(2·R²)
```

#### 最終公式

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Q77,i[k] = Δt⁴ · â_x,i²[k] · {                                          │
│              ( K_h,i²(h̄[k]) − K_h,i'(h̄[k]) )² · ḣ_max⁴ / (8·R⁴)         │
│            +   K_h,i²(h̄[k])                  · ḧ_max² / (2·R²)           │
│           }                                                              │
└─────────────────────────────────────────────────────────────────────────┘

K_h,i(h̄)  := (1/C_i(h̄)) · (dC_i/dh̄)              一階壁面敏感度
K_h,i'(h̄) := dK_h,i/dh̄ = C_i''/C_i − K_h,i²       二階敏感度修正

ḣ_max := A·ω = 2π·f·A             trajectory ḣ 上界（對 1Hz/A=10: 62.8 μm/s）
ḧ_max := A·ω² = (2π·f)²·A         trajectory ḧ 上界（對 1Hz/A=10: 395 μm/s²）
```

per-axis：x, y 用 C_∥；z 用 C_⊥。

#### Term A vs Term B 的相對量級（osc trajectory, z 軸）

對 c_⊥(h̄) ≈ 1+9/(8h̄)（near-wall leading order）：

| 位置 h̄ | K_h² | K_h' | Term A / Term B |
|---|---|---|---|
| 22 (hold) | 4.9e−6 | 2.0e−4 | 0.04 (Term B 主導) |
| 5.6 (descent) | 0.013 | 0.008 | 0.6 (各半) |
| **1.11 (wall)** | **0.205** | **0.612** | **4.0** |

→ **過 wall 時 Term A 是 Term B 的 4 倍**。完整公式必須含兩項，不能只取 Term B（這是為什麼 Route III 簡化會 under-estimate ~80% — 見 §8.8）。

#### 屬性

| 性質 | 值 |
|---|---|
| Per-axis | ✓ K_h,i, K_h,i' 是 per-axis（C_∥ vs C_⊥） |
| 時變 | ✓ 雙重依賴：直接 (â_x²) 與間接 (K_h, K_h' 透過 h̄[k]) |
| 是否 â_x 函數？ | ✓ Adaptive EKF / SDRE |
| 是否需 inflation? | ✗ w_a 是 designed white driver，無 MA 問題 |
| 函數變數 | h̄[k]（直接從量測算，非 KF 估），無 bias amplification loop |

#### 實作前置：K_h 函數

需在 `calc_correction_functions(h_bar)` 增加 `dC_∥/dh̄`、`dC_⊥/dh̄` 輸出（解析式或數值差分），Task 04 實作時處理。

### 8.5 時變性與更新時機總結

```
每一步 KF update 計算量：
  Q33[k] = 4kBT · â_x[k]                                              (1 個 ×)
  Q77[k] = Δt⁴·â_x²·{(K_h²−K_h')²·ḣ_max⁴/(8R⁴) + K_h²·ḧ_max²/(2R²)}
                                                       (lookup K_h, K_h' + 多次 ×)
  Q55    = 0  (常數，不更新)

離線一次性算（常數）：
  ḣ_max, ḧ_max 從 trajectory 算 (A·ω, A·ω²)
  常數因子 4kBT, Δt⁴, 1/(8R⁴), 1/(2R²)
```

| 項 | 時變？ | 來源 |
|---|---|---|
| Q33,i[k] | ✓ 時變 | 透過 â_x,i[k] |
| Q55 | ✗ 常數 (=0) | 設計選擇 |
| Q77,i[k] | ✓ 時變 | 雙重依賴 â_x,i[k] 與 h̄[k]（K_h, K_h'） |

### 8.6 全部 caveats

```
[Q33] 嚴格 KF formalism：Q33 只含當步白噪 thermal (i)，
       不含 (ii)(iii) thermal 歷史 (cross-step correlated) 與
       (iv) n_x (在 R 中)。後者三項影響由 paper 2023 Eq.(22)
       的 closed-loop 公式分別處理。
       
       代價：KF 預測 P(3,3) ≈ 真實 σ²_δx 的 50% (1/(3-2λ_c²))。
       這是 standard KF 處理 correlated process noise 的
       已知 trade-off。要消除須用 Path B (9-state augmented)。

[Q55] = 0 假設：simulation 無殘磁。實機需重新推導 from
       磁通變化率物理模型。建議加 floor=1e-12 避免數值鎖死。

[Q77] 公式 Var(w_a) = Δt⁴·Var(ä_x_true)，含 Term A (ḣ²) 與
      Term B (ḧ) 兩項。Term A 過 wall 時主導 (4×Term B at h̄=1.11)，
      因此完整公式必須含兩項。早期 Var(δa_x) 形式（少一個 Δt
      因子加上一階導 vs 二階導）已退役。

[Q77] σ²_ḣ_max, σ²_ḧ_max 用 trajectory 上界（一週期統計），
      是保守選擇。對非常急的 transient 軌跡可能略偏。

[Q77] K_h, K_h' 函數需要 calc_correction_functions 加 dC/dh̄ 與
      d²C/dh̄² (或 K_h') 輸出。Task 04 實作時處理。

[Q]  對角化前提：A1-A4 四個獨立性假設。對 simulation 全部成立；
     實機若 thermal/sensor 有相關性需重評估。
```

### 8.7 對比 paper 2025 / qr 分支

| 項 | paper 2025 (qr 分支) | 本主線 (paper 2023 Eq.17 + Path C strict) |
|---|---|---|
| Q33 thermal 公式 | `4kBT·a_x`（嚴格） | `4kBT·â_x`（嚴格，數學上一致） |
| 對應控制律 | Eq.6（用 δx̂ feedback） | Eq.17（用 δx_m feedback） |
| ε 結構差異 | ε 不含 Σf_T | ε 含 Σf_T（不入 Q，由 Eq.22 處理） |
| KF P 預測 vs 真實 | 接近 1.0× | ~0.5×（Eq.17 closed-loop dynamics 的 trade-off） |
| Q77 | random walk 常數 | adaptive (â_x², K_h²) |
| 對動態軌跡的反應 | 需手動調 Q | 自動跟著 trajectory 形狀走 |

**重要**：兩個方案的 Q33 公式長得**一樣**（`4kBT·a_x`），但**KF 行為不同**。原因在控制律不同 → ε 結構不同 → 真實 σ²_δx 不同 → KF 對 P 的「正確值」認知不同。Q33 數值對齊只是表面，深層結構差異透過 closed-loop variance 體現。

### 8.8 Route III 評估（已駁回，記錄理由）

曾考慮把 Q77 寫成 â_x 的多項式（用 c_⊥ ≈ 1+9/(8h̄) 近壁近似反代 K_h → â_x）：
```
Route III: Q(7,7)(â_x) = (32/81) · (a_nom − â_x)⁴ · A_h² · ω⁴ · Δt⁴ / (a_nom²·R²)
```

**評估結論：駁回，採 Path B (§8.4)**。

#### Route III 的本質

數學上，Route III ≡ Path B 的 **Term B 單獨**（即 ḧ 主導項）+ K_h 用近壁近似反代成 â_x 多項式。**不是新方法，是 Path B 的真子集**。

#### 駁回理由

| 維度 | Path B (§8.4) | Route III |
|---|---|---|
| 包含項 | Term A + Term B（完整） | **Term B only** |
| 過 wall 精度 | ✓ Term A 主導 4× | ✗ **under-estimate ~80%** |
| 函數變數 | h̄[k]（直接量測） | â_x[k]（KF 估，含 bias） |
| Bias amplification | ✗ 無 | ✓ **隱式遞迴需 1-step lag mitigate** |
| h_init 依賴 | ✗ 不依賴 | ✗ 不依賴（兩者一樣） |
| Adaptive | ✓ 透過 K_h(h̄[k]) | ✓ 透過 (a_nom − â_x)⁴ |
| 高階 c 修正擴展性 | ✓ 換 K_h, K_h' lookup | ✗ 需重推解析多項式 |

#### Route III 唯一實質優勢

「閉式多項式，不需 K_h' lookup」。但這個優勢的代價是過 wall 80% 精度損失與 bias loop，**不划算**。

#### Route III 的價值（保留作 sanity check）

Task 04 實作後，可在「遠離 wall + Term B 主導區」用 Route III 多項式跟 Path B 完整公式對算 Q77，驗證 Path B 在該區域實作正確。**不採作主要設計**。

---

## 9. R 矩陣推導（Task 03，進行中）

R 矩陣 per-axis 結構（雙量測對角，cross-channel 假設獨立）：

```
       y_1: δx_m     y_2: a_xm
     ┌                          ┐
y_1  │  R(1,1)                  │     R(1,1) = σ²_n_s,i           (per-axis sensor)
R_i =│                          │
y_2  │             R(2,2)       │     R(2,2) = R_2_eff,i[k]       (主推導目標)
     └                          ┘

R_2_eff,i[k] = R_2_intrinsic,i[k] + 5·Q77,i[k]
              └─────┬─────┘     └────┬────┘
        IIR-induced           延遲傳導項 (d=2, §6 已推)
        Step 3.2-3.3 推
```

### 9.1 Step 3.1 — 閉迴路 δx_r 統計（DONE）

#### δx_r 定義（paper 2025 Eq.9）

```
δx̄_m[k+1] = (1−a_var)·δx̄_m[k] + a_var·δx_m[k+1]      (IIR LP mean)
δx_r[k]   = δx_m[k] − δx̄_m[k]                          (centered random component)
```

穩態下：δx_r[k] ≈ δx[k−d] + n_x[k] − E[δx[k−d]]，positioning hold 時 E=0。

#### σ²_δxr — 從 Eq.19 form 嚴格推導

從閉迴路 δx[k+1] = λ_c·δx[k] − ε[k] 出發，ε[k] 是 MA(2) cross-step correlated（§8.2 Step 1）。

**ARMA inflation 推導**（thermal 部分）：
```
Var(ε)_thermal = [1+2(1−λ_c)²]·â_x²·σ²_fT
ρ_ε(1) = (1−λ_c)(2−λ_c)/[1+2(1−λ_c)²]    對 λ_c=0.7：≈ 0.331
ρ_ε(2) = (1−λ_c)         /[1+2(1−λ_c)²]    對 λ_c=0.7：≈ 0.254
ρ_ε(τ≥3) = 0

σ²_δx,thermal = Var(ε,thermal)/(1−λ_c²) · [1 + 2λ_c·ρ_ε(1) + 2λ_c²·ρ_ε(2)]
              = [2 + 1/(1−λ_c²)] · σ²_δxT
              = [2 + 1/(1−λ_c²)] · 4kBT·â_x
```

n_x 部分（白噪 leak 進 ε，再加量測時直接的 sensor noise）：
```
σ²_δx,sensor   = (1−λ_c)²·σ²_n_s/(1−λ_c²) = (1−λ_c)/(1+λ_c)·σ²_n_s
σ²_δxr,sensor  = σ²_δx,sensor + σ²_n_s = 2/(1+λ_c)·σ²_n_s
```

**最終公式**：

```
┌──────────────────────────────────────────────────────────────────┐
│  σ²_δxr,i[k] = C_dpmr · 4kBT · â_x,i[k]  +  C_n · σ²_n_s,i        │
│                                                                  │
│  C_dpmr = 2 + 1/(1−λ_c²)         對 λ_c=0.7：≈ 3.96               │
│  C_n    = 2/(1+λ_c)               對 λ_c=0.7：≈ 1.18               │
└──────────────────────────────────────────────────────────────────┘
```

**重要**：C_dpmr 與 C_n **不是粗糙近似**，是 paper 2023 Eq.17 + Eq.19 form 嚴格閉式推導（與 paper 2025 Eq.11/12 同形式，因兩個控制律的 ε 都有同樣 MA(2) tail 結構）。

#### ρ_δxr(τ) 自相關函數

```
ρ_δxr(τ) = ρ_δx(τ) · σ²_δx / σ²_δxr      (τ ≥ 1)
ρ_δxr(0) = 1
```

ρ_δx(τ) 兩種選擇與 inflation factor IF_var：

| Option | ρ_δx(1) | ρ_δx(2) | ρ_δx(τ≥3) | IF_var (thermal-dominated, λ_c=0.7) |
|---|---|---|---|---|
| **A: MA(2) full**  ✓ | 0.852 | 0.672 | λ_c·ρ_δx(τ−1) | ≈ 4.2 |
| B: AR(1) approx | 0.700 | 0.490 | λ_c^τ | (1+λ_c²)/(1−λ_c²) ≈ 2.92 |

**選 Option A**（與 §8.2 Eq.19 form Path C 一致；Option B 對 IF_var under-estimate ~30%）。

σ²_δx/σ²_δxr 比值：
- thermal-dominated (4kBT·â_x >> σ²_n_s):  → 1
- sensor-dominated (4kBT·â_x << σ²_n_s):   → (1−λ_c)/2 ≈ 0.15

#### IF_var inflation factor（自相關對 IIR 統計的影響）

```
IF_var = 1 + 2·Σ_{τ=1}^∞ [ρ_δxr(τ)]²
```

對 thermal-dominated case，IF_var ≈ 4.2（Option A）。**離線一次計算，per-axis 共用**。

### 9.2 Step 3.1 三決定（已敲定）

| 決定 | 選擇 | 理由 |
|---|---|---|
| ρ_δx(τ) approximation | Option A (MA(2) full) | 與 §8.2 Eq.19 form 一致 |
| σ²_δxr 用 a_x 哪個 | â_x[k] (per-step 時變) | 與 Q33/Q77 同樣 adaptive |
| σ²_n_s 處理 | 常數 per-axis (config.meas_noise_std) | sensor noise 與位置無物理耦合 |

### 9.3 Step 3.2 — IIR variance estimator chi-squared 分析（DONE）

#### 設定 + 穩態簡化

paper 2025 Eq.10 的 IIR：
```
σ̂²_δxr[k+1] = (1−a_cov)·σ̂²_δxr[k] + a_cov·(δx_r²[k+1] − δ̄x_r²[k+1])
                                       └────────┬────────┘
                                          input Z[k+1]
```

δx_r 已中心化 → E[δx_r] = 0 → 穩態 δ̄x_r → 0 → **Z[k] ≈ δx_r²[k]**。

**Caveat**：嚴格上 δ̄x_r 自身有 Var(δ̄x_r) ≈ a_var/(2−a_var)·σ²_δxr，σ̂²_δxr 因此有 small bias ~ −Var(δ̄x_r)。對 a_var=0.05 是 ~2.5%，可接受，本推導忽略。

#### δx_r 高斯性 + Wick 處理

δx_r = Σ Gaussian noises（thermal + sensor）線性組合 → δx_r ~ N(0, σ²_δxr) 嚴格高斯。可用 Isserlis (Wick) 算高階矩：

```
Var(δx_r²)                 = 2·(σ²_δxr)²                            (E[X⁴]=3σ⁴)
Cov(δx_r²[k], δx_r²[k+τ])  = 2·(σ²_δxr)² · ρ_δxr²(τ)                 (Wick)
ρ_{δx_r²}(τ)               = ρ_δxr²(τ)                               ← 高斯特例
```

**關鍵**：高斯下平方訊號自相關 = 原訊號自相關平方。配合 Step 3.1 的 ρ_δxr(τ) 立即得 ρ_{δx_r²}(τ)。

#### EWMA 線性方差傳遞

把 EWMA 寫成 weight sum (s := 1−a_cov)：
```
σ̂²_δxr[k+1] = a_cov · Σ_{i=0}^∞ s^i · Z[k+1−i]

Var(σ̂²_δxr) = a_cov² · Var(Z) · S(s)
S(s) = (1/(1−s²)) · IF_eff(s)
IF_eff(s) := 1 + 2·Σ_{m=1}^∞ ρ_δxr²(m) · s^m         ← effective inflation
```

小 a_cov 極限：
- 1/(1−s²) ≈ 1/(2·a_cov)
- IF_eff(s→1) → IF_var = 1 + 2·Σ ρ_δxr²(τ)（與 Step 3.1 同 IF_var）

```
┌──────────────────────────────────────────────────────────────────┐
│  Var(σ̂²_δxr,i[k]) ≈ a_cov · IF_var · (σ²_δxr,i[k])²              │
│                                                                  │
│  IF_var ≈ 4.24  (Option A, λ_c=0.7, thermal-dominated)            │
└──────────────────────────────────────────────────────────────────┘
```

#### 精度檢核：IF_eff vs IF_var

對 a_cov=0.05, λ_c=0.7, Option A：
- IF_eff(s=0.95) ≈ 3.90
- IF_var (s→1)  ≈ 4.24
- **小 a_cov 近似 over-estimate ~9%**，可接受

對更大 a_cov（如 0.1）誤差會升到 ~17%，到時切回 IF_eff(s) 完整公式。

#### 數值（thermal-dominated, λ_c=0.7, a_cov=0.05）

| Option | Var(σ̂²_δxr) / (σ²_δxr)² | σ(σ̂²_δxr) / σ²_δxr |
|---|---|---|
| **A: MA(2) full** | a_cov·4.24 ≈ 0.212 | √0.212 ≈ **46%** |
| B: AR(1) approx | a_cov·2.92 ≈ 0.146 | √0.146 ≈ **38%** |

→ Option A 預測 σ̂²_δxr 標準差 ~46% 真實值，B 約 38%。A 比 B 高 ~21%。

### 9.4 Step 3.3 — R_2_intrinsic 閉式（DONE）

線性反解 a_xm = (σ̂²_δxr − C_n·σ²_n_s)/(C_dpmr·4kBT)：
```
σ²_δxr,i / (C_dpmr·4kBT) = â_x,i + ξ_i
ξ_i := (C_n/C_dpmr) · σ²_n_s,i / (4kBT)        per-axis sensor-induced offset
```

```
┌──────────────────────────────────────────────────────────────────┐
│  R_2_intrinsic,i[k] = a_cov · IF_var · {â_x,i[k] + ξ_i}²          │
│                                                                  │
│  ξ_i = (C_n/C_dpmr) · σ²_n_s,i / (4kBT)        per-axis const     │
└──────────────────────────────────────────────────────────────────┘
```

**結構**：純二次多項式 in â_x，per-axis 常數 ξ_i 平移（sensor-noise floor）。

**兩極限**：
- **sensor-dominated** (â_x << ξ)：R_2_intrinsic ≈ a_cov·IF_var·ξ_i²，常數 floor
- **thermal-dominated** (â_x >> ξ)：R_2_intrinsic ≈ a_cov·IF_var·â_x²，與 Q33 同 â_x 級數

### 9.5 Step 3.4 — R_2_eff 完整公式（DONE）

從 §6 量測架構推導，d=2 步延遲傳導：
```
R_2_eff,i[k] = R_2_intrinsic,i[k] + Σ_{j=1}^{d}(d−j+1)² · Q77,i[k]
             = R_2_intrinsic,i[k] + 5 · Q77,i[k]    (d=2)
```

完整公式：

```
┌──────────────────────────────────────────────────────────────────┐
│                                                                  │
│  R_2_eff,i[k] = a_cov · IF_var · {â_x,i[k] + ξ_i}²                │
│              + 5 · Δt⁴ · â_x,i²[k] · {                            │
│                  (K_h,i² − K_h,i')² · ḣ_max⁴/(8R⁴)                │
│                + K_h,i²            · ḧ_max²/(2R²)                 │
│                }                                                  │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

依賴：â_x,i[k] (EKF state), h̄[k] (measurement), per-axis 常數 ξ_i, K_h, K_h' lookup。

### 9.6 R 矩陣完整函數性質總表

```
        y_1: δx_m       y_2: a_xm
      ┌                              ┐
y_1   │  R(1,1) = σ²_n_s,i           │   per-axis 常數
R_i = │                              │
y_2   │              R(2,2) = R_2_eff │   per-axis 時變
      └                              ┘
```

| Element | 公式 | 變數 | 類型 | 時變？ |
|---|---|---|---|---|
| R(1,1),i | `σ²_n_s,i` | sensor config | scalar 常數 | ✗ |
| R(2,2),i intrinsic | `a_cov·IF_var·(â_x,i+ξ_i)²` | â_x,i[k] | scalar (二次 in â_x) | ✓ |
| R(2,2),i delay | `5·Q77,i[k]` | â_x,i[k], h̄[k] | scalar (â_x² × wall fcn) | ✓ |

**離線常數**：C_dpmr, C_n, IF_var, ξ_i, 5, ḣ_max, ḧ_max, R, Δt, kBT。  
**線上每步 i ∈ {x,y,z}**：以 (â_x,i[k], h̄[k]) 算 R_i[k]（off-diagonal=0）。

### 9.7 Q vs R 對照（與 Task 02 並列）

| Matrix element | 公式 | 變數 | 物理 |
|---|---|---|---|
| Q33,i[k] | `4kBT·â_x` | â_x,i[k] | 當步 thermal 白噪變異 |
| Q55 | 0 | — | simulation 假設 |
| Q77,i[k] | `Δt⁴·â_x²·{...K_h, K_h'...}` | â_x,i[k], h̄[k] | 軌跡誘發 a_x 二階變化 |
| R(1,1) | `σ²_n_s,i` | const | sensor noise |
| **R(2,2),i[k]** | **`a_cov·IF_var·(â_x+ξ_i)² + 5·Q77`** | â_x,i[k], h̄[k] | IIR 統計噪聲 + 延遲傳導 |

→ Q33 與 R(2,2) 都是 â_x 的多項式（Q33 一次、R(2,2) 二次），都是 EKF / SDRE 結構的 adaptive 矩陣。

### 9.8 各 Step 精度限制總表

| Source | 偏差 | 處理 |
|---|---|---|
| Eq.19 form ε MA(2) trade-off (Q33) | KF P 預測 ~50% 真實 σ²_δx | 接受（§8.2 Step 5） |
| 穩態 δ̄x_r ≈ 0 假設 | σ̂²_δxr bias ~2.5% | 忽略 |
| Small a_cov (IF_eff ≈ IF_var) | IF_var over-estimate ~9% | 忽略 (a_cov=0.05) |
| Option A vs B (ρ_δx 模型) | IF_var Option B under ~30% | 已選 A |
| Thermal-dominated ρ_δxr ≈ ρ_δx | sensor-dominated 時 ρ_δxr 偏小 | 後續精化 |
| **合計** | **~10–15% 各方向偏差** | 端到端驗證再回頭精化 |

### 9.9 Step 3.5 — 自適應 R_2[k] gate（待推導）

四個 IIR 崩塌條件，發生時 R_2 → ∞ 把 y_2 通道暫時關閉：

| 條件 | 物理 | 偵測 |
|---|---|---|
| Warm-up | IIR 還沒收斂 | k < N_warm（從 a_var 算） |
| 低 SNR | thermal 不足 | σ̂²_δxr − C_n·σ²_n < 0 |
| a_x 動 太快 | IIR 跟不上 | \|dâ_x/dt\| > threshold |
| 過 wall | C_⊥ lookup 誤差大 | h̄ < 1.2 |

實作可用單一 boolean gate：滿足任一 → R_2 = 1e10。

### 9.10 實作對應（規劃）

| 檔案 / 函數 | 用途 | 性質 |
|---|---|---|
| `build_eq17_constants(lambda_c, option)` | 離線算 C_dpmr, C_n, IF_var, delay_R2_factor | 一次性 scalar |
| `compute_rho_dx_MA2(lambda_c)` | Option A 的 ρ_δx(τ) array (τ=0..30) 與 IF_var | 一次性 vector |
| `update_R_2(â_x, σ²_n_s, Q77, ctrl_const)` | 線上每步算 R_2_eff | inline scalar |
| `calc_correction_functions(h_bar, want_derivs)` | 擴充輸出 K_h, K_h'（Q77 用） | scalar polynomial |

**Q77 與 R_2 都不需要 lookup table**，唯一的 table 是 C_∥(h̄), C_⊥(h̄) 與其導數（已在既有 `calc_correction_functions`，待擴充導數輸出）。

### 9.11 下一步

進 **Step 3.5**：自適應 R_2[k] gate 設計（warm-up / 低 SNR / 過 wall / a_x 變化太快 四條件偵測 + 觸發策略 + hysteresis），詳細討論點見 §9.9 outline。

---

## 10. Notation 約定

完整 notation 對照表見 [`agent_docs/eq17-architecture.md`](../../agent_docs/eq17-architecture.md) §6。MATLAB 變數名遵循 `.claude/rules/matlab-conventions.md` 的 snake_case 規範。

---

## 11. 開放問題（暫保留，等驗證再決定）

- a_xm 在 IIR warm-up 期的 fallback 策略 — **task 3 處理**
- 是否需要把 IIR 內部 state（δx̄_m、σ²_δxr）也納入 augmented state — **目前傾向不需要**，待 Gramian 數值結果驗證
- 控制律中 x̂_D 補償項的縮放因子（`−x̂_D` vs `−(1−λ_c)·x̂_D`）— 已選前者全量補償，task 5 Lyapunov 驗證
- C_dpmr/C_n 粗糙版（paper Eq.13 閉式）vs effective 版（augmented Lyapunov）— **起步用粗糙版**，端到端驗證若不準再升級

---

## 12. 待辦項目（後續 task）

依優先順序：

1. **Observability 數值驗證** ✓ DONE (Task 01)
   - `test_script/check_observability_eq17.m`：5-state（10 cases）+ 7-state（3 cases）共 13 case 全 PASS
   - 報告：[`task01_math_observability_report.md`](task01_math_observability_report.md)

2. **Q 矩陣推導（Path C strict + Path B adaptive Q77）** ✓ DONE (Task 02)
   - 結果：見 §8 unified version
   - Q33: Path C strict `4kBT·â_x` (Eq.19 form，accept ~50% P undershoot trade-off)
   - Q55: 0 (simulation)；Q77: Path B `Δt⁴·â_x²·{(K_h²−K_h')²·ḣ_max⁴/(8R⁴) + K_h²·ḧ_max²/(2R²)}`

3. **R 矩陣設計** IN-PROGRESS（§9 詳推）
   - R_1: per-axis σ²_n_s（直接從 config.meas_noise_std）
   - R_2[k]: a_xm 噪聲方差（閉迴路推導：IIR variance estimator + chi-squared + autocorrelation 修正 + 5·Q77 延遲傳導項）
   - **Step 3.1-3.4 ✓ DONE** (§9.1-9.5)：
     - σ²_δxr = C_dpmr·4kBT·â_x + C_n·σ²_n_s（C_dpmr=3.96, C_n=1.18, IF_var≈4.24 for λ_c=0.7, Option A）
     - Var(σ̂²_δxr) = a_cov·IF_var·(σ²_δxr)²（從 Wick + EWMA 線性傳遞）
     - **R_2_eff,i[k] = a_cov·IF_var·(â_x,i+ξ_i)² + 5·Q77,i[k]**（純二次多項式 in â_x + Q77 wall 動態）
     - ξ_i = (C_n/C_dpmr)·σ²_n_s,i/(4kBT) 是 per-axis sensor floor
   - **Step 3.5 待續**：自適應 R_2 gate 設計（warm-up / 低 SNR / 過 wall / a_x 變化太快 四條件偵測 + 觸發 + hysteresis）

4. **7-state EKF 實作**
   - `model/controller/motion_control_law_eq17_7state.m`
   - 含 K_h 函數擴充（calc_correction_functions 加 dC/dh̄）

5. **Closed-loop variance Lyapunov**
   - 7-state augmented Lyapunov 解 closed-loop 穩態方差
   - 對比 paper 2023 Eq.22 在無估測誤差極限下的閉式

6. **Trajectory 上的端到端驗證**
   - 跑 `run_simulation.m`（osc trajectory）
   - 對比 controller_type=eq17_7state vs controller_type=7（qr 版本）

---

## References

- Meng, T.-M., & Menq, C.-H. (2023). *Ultra-Precise High-Speed Untethered Manipulation of Magnetic Scanning Microprobe in Aqueous Solutions*. IEEE/ASME T-Mech, vol. 28, no. 1. — Eq.(17) 控制律來源
- Meng, T.-M., Long, F., & Menq, C.-H. (2025). *Near-Wall Ultra-precise Motion Control of a Magnetically Driven Scanning Microprobe in Aqueous Solutions*. IEEE TIE, vol. 72, no. 1. — IIR + 7-state EKF 來源（本路線只借用 IIR 的 a_xm 推導，不採其控制律）
- qr 分支保留的相關文件：
  - `agent_docs/ekf-qr-analysis.md`（main 上版本）— EKF 矩陣推導參考
  - `agent_docs/math-model.md`（main 上版本）— 座標、單位、γ 計算
