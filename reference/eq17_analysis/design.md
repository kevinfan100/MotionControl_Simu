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

## 8. 待辦項目（後續 task）

依優先順序：

1. **Observability 數值驗證** ✓ DONE (Task 01)
   - `test_script/check_observability_eq17.m`：5-state（10 cases）+ 7-state（3 cases）共 13 case 全 PASS
   - rank=7 對任意 f_d 確認（含 f_d=0 最嚴苛條件）
   - 報告：[`task01_math_observability_report.md`](task01_math_observability_report.md)
   - 後續 Gramian σ_min(t) 量化評估暫不做，需要時新開 task

2. **Q 矩陣第一性推導** ✓ DONE (Task 02, 純代數開迴路)
   - 結果：見 §8（Q 矩陣 open-loop 推導）
   - Q33[k] from paper 2023 Eq.19；Q55 = 0；Q77 from trajectory 差分方差

3. **R[k] 設計**
   - R_1: per-axis σ²_n（直接從 config.meas_noise_std）
   - R_2[k]: a_xm 噪聲方差，含 IIR-induced 結構（warm-up + thermal-to-noise 監測）

4. **7-state EKF 實作**
   - `model/controller/motion_control_law_eq17_7state.m`
   - 控制律 + EKF 一體化（與既存 motion_control_law_7state.m 並列；前者用 paper 2023 Eq.17，後者用 paper 2025 Eq.6）

5. **Closed-loop variance Lyapunov**
   - 7-state augmented Lyapunov 解 closed-loop 穩態方差
   - 對比 paper 2023 Eq.22 在無估測誤差極限下的閉式

6. **Trajectory 上的端到端驗證**
   - 跑 `run_simulation.m`（osc trajectory）
   - 對比 controller_type=eq17_7state vs controller_type=7（qr 版本）
   - tracking error std、a_hat 收斂行為

---

## 8. Q 矩陣推導（純代數，純開迴路）— Task 02

### 8.1 Q 矩陣的稀疏結構

state vector `[δx_1, δx_2, δx_3, x_D, δx_D, a_x, δa_x]ᵀ` 中，shift state（δx_1, δx_2）與 integrator state（x_D, a_x）是**確定性映射**，沒有驅動噪聲，Q 對角項為 0。**只有三個非零 Q 項**：Q33（δx_3）、Q55（δx_D）、Q77（δa_x）。

```
       δx_1  δx_2  δx_3       x_D  δx_D    a_x  δa_x
   ┌                                                          ┐
   │  0     0     0           0    0       0    0             │
   │  0     0     0           0    0       0    0             │
   │  0     0    Q33,i[k]    0    0       0    0             │   ← thermal+sensor
Q_i = │  0     0     0           0    0       0    0             │
   │  0     0     0           0   Q55      0    0             │   ← disturbance velocity
   │  0     0     0           0    0       0    0             │
   │  0     0     0           0    0       0    Q77,i        │   ← gain velocity
   └                                                          ┘
```

per-axis i ∈ {x, y, z}。

### 8.2 Q33[k] — 從 paper 2023 Eq.(19) 推

paper 2023 Eq.(19) 寫出 ε[k] 的完整形式（per-axis 略 i index）：
```
ε[k] = a_x[k]·f_T[k] + (1−λ_c)·a_x[k]·Σ_{j=1}^{d} f_T[k−j] + (1−λ_c)·n_x[k]
```

各項來源都是**開迴路物理量**：
- `f_T[k]`：thermal force（白噪聲），`σ²_fT[k] = 4kBT·γ(h[k])/Δt`
- `f_T[k−j]` for j=1..d：過去 d 個獨立 thermal samples
- `n_x[k]`：sensor noise（白噪聲），`σ²_n_s`

各項獨立加性方差：
```
σ²_ε[k] = a²_x[k]·σ²_fT[k]              ← 當前 thermal
        + (1−λ_c)²·a²_x[k]·d·σ²_fT[k]    ← 過去 d 步 thermal 史（d 個獨立白噪和）
        + (1−λ_c)²·σ²_n_s                ← sensor noise
```

化簡 `a²_x·σ²_fT = (Δt/γ)²·(4kBT·γ/Δt) = 4kBT·a_x`（線性 in a_x！）：

```
┌─────────────────────────────────────────────────────────────────────┐
│  Q33,i[k] = 4kBT · a_x,i[k] · [ 1 + d·(1−λ_c)² ]                    │
│                + (1−λ_c)² · σ²_n_s,i                                │
└─────────────────────────────────────────────────────────────────────┘
```

per-axis 變化來自 `a_x,i[k]`（透過 `γ_i(h[k])`）與 sensor 規格差。**時變來源只有 a_x[k]**，其他都是常數。

### 8.3 Q55 — 物理設計參數

x_D 是 lumped disturbance（殘磁、磁滯、modeling error）。我們的 simulation 環境**無這些干擾源**，故：
```
Q55 = 0    (simulation 場景)
```

實機部署若有殘磁，需從殘磁磁通變化率的物理模型補上。

### 8.4 Q77,i — 從軌跡差分方差推

`a_x_true,i(t) = Δt / { γ_N · C_i(h(t)/R) }` 對 prescribed trajectory 是 h(t) 的確定函數。Q77 反映「a_x 速度的不可預測度」：

```
┌──────────────────────────────────────────────────────────┐
│  Q77,i = Var_{t ∈ trajectory}( δa_x_true,i(t) )           │
│                                                          │
│  其中  δa_x_true,i(t) := a_x_true,i(t+Δt) − a_x_true,i(t) │
└──────────────────────────────────────────────────────────┘
```

**離線一次性計算**：用 `trajectory_generator.m` 產 a_x_true(t) 全段序列 → 算一階差分 → 取 sample variance。實作時填入 params。

對 osc trajectory（h: 50 → 12.5±10 1Hz）：
- z 軸（c_⊥(h/R) 主導）：過 wall 時 |δa_x| 最大，Q77,z 應顯著大於 Q77,x、Q77,y
- x, y 軸（c_∥）：c_∥ 變化幅度小於 c_⊥，Q77 較小

per-axis 數值留待 Task 04 實作時填入。

### 8.5 時變性總結

| 項 | 時變？ | 來源 |
|---|---|---|
| Q33,i[k] | ✓ 時變 | 透過 a_x,i[k] 的時變性 |
| Q55 | ✗ 常數 (=0) | 設計選擇 |
| Q77,i | ✗ 軌跡常數 | 離線從整段軌跡算 |

→ 實作時 Q[k] 每步只需更新 Q33（一次乘法 + 一次加法），Q55 / Q77 預先固定。

### 8.6 對比 paper 2025 / qr 分支的差異

paper 2025 Eq.(21) 給的 Q(δx_3) 是 `4kBT·a_x[k]`（直接，**沒有 1+d(1−λ_c)² 修正**）。差異來源：
- paper 2025 的控制律用 δx̂ feedback，ε[k] 不含 `(1−λ_c)·a_x·Σf_T` 項
- 我們的 Eq.(17) 控制律有此項（過去 thermal 史經 (1−λ_c) 加權進入 ε）
- 對 d=2, λ_c=0.7：修正因子 1 + 2·0.09 = **1.18**（18% 增量）

→ 直接套 paper 2025 的 Q 會 18% under-estimate。我們的推導更精確。

---

## 9. Notation 約定

完整 notation 對照表見 [`agent_docs/eq17-architecture.md`](../../agent_docs/eq17-architecture.md) §6。MATLAB 變數名遵循 `.claude/rules/matlab-conventions.md` 的 snake_case 規範。

---

## 10. 開放問題（暫保留，等驗證再決定）

- a_xm 在 IIR warm-up 期的 fallback 策略 — **task 3 處理**
- 是否需要把 IIR 內部 state（δx̄_m、σ²_δxr）也納入 augmented state — **目前傾向不需要**，待 Gramian 數值結果驗證
- 控制律中 x̂_D 補償項的縮放因子（`−x̂_D` vs `−(1−λ_c)·x̂_D`）— 已選前者全量補償，task 5 Lyapunov 驗證
- C_dpmr/C_n 粗糙版（paper Eq.13 閉式）vs effective 版（augmented Lyapunov）— **起步用粗糙版**，端到端驗證若不準再升級

---

## References

- Meng, T.-M., & Menq, C.-H. (2023). *Ultra-Precise High-Speed Untethered Manipulation of Magnetic Scanning Microprobe in Aqueous Solutions*. IEEE/ASME T-Mech, vol. 28, no. 1. — Eq.(17) 控制律來源
- Meng, T.-M., Long, F., & Menq, C.-H. (2025). *Near-Wall Ultra-precise Motion Control of a Magnetically Driven Scanning Microprobe in Aqueous Solutions*. IEEE TIE, vol. 72, no. 1. — IIR + 7-state EKF 來源（本路線只借用 IIR 的 a_xm 推導，不採其控制律）
- qr 分支保留的相關文件：
  - `agent_docs/ekf-qr-analysis.md`（main 上版本）— EKF 矩陣推導參考
  - `agent_docs/math-model.md`（main 上版本）— 座標、單位、γ 計算
