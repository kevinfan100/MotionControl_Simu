# Eq.17 Reduced-State EKF Architecture

**永久技術參考**。本檔描述 paper 2023 Eq.(17) 控制律 + reduced-state EKF 主線的核心架構，跨 session 不變。實際推導記錄請見 [`../reference/eq17_analysis/design.md`](../reference/eq17_analysis/design.md)。

對照組：[`ekf-qr-analysis.md`](ekf-qr-analysis.md)（7-state EKF, paper 2025 風格）。兩條路線**並列**，不相互取代。

---

## 1. System Equations

### Plant（1-D，paper 2023 Eq.13–14）

```
γ(h(t)) ẋ(t) = f_d(t) + f_T(t)                    (連續)
x[k+1] = x[k] + a_x[k] · (f_d[k] + f_T[k]) + x_D[k]   (離散 + lumped disturbance)
```

| 符號 | 定義 | 單位 |
|---|---|---|
| `Δt` | 取樣間隔 | sec |
| `a_x[k]` | motion gain `Δt / γ(h[k]/R)` | μm/pN |
| `γ(h/R)` | 位置相關拖曳係數 `γ_N · C(h/R)` | pN·sec/μm |
| `f_d[k]` | desired control force | pN |
| `f_T[k]` | thermal force（白噪，方差 `σ²_T = 4 k_B T γ / Δt`） | pN |
| `x_D[k]` | lumped disturbance | μm |

Per-axis：
- x, y axes: `γ = γ_N · C_∥(h/R)`
- z axis:    `γ = γ_N · C_⊥(h/R)`

### 量測（d-step delay）

```
y[k] = δx_m[k] = δx[k − d] + n_x[k],   d = 2
```

`n_x[k]` per-axis 白噪聲，方差 `σ²_n,i`（i ∈ {x, y, z}）。

### 控制律（paper 2023 Eq.(17)）

```
f_d[k] = (1 / â_x[k]) · { x_d[k+1]
                          − λ_c · x_d[k]
                          − (1−λ_c) · x_d[k−d]
                          + (1−λ_c) · δx_m[k]
                          − x̂_D[k] }
```

Plug-in 來源：
- `x_d[k+1], x_d[k], x_d[k−d]`：reference trajectory（已知）
- `δx_m[k]`：raw 量測（不過 IIR）
- `â_x[k], x̂_D[k]`：EKF 估計值

不需要的：`δx̂[k]`（Eq.17 自帶延遲補償，不需估「當前」δx）。

### 理想閉迴路（paper 2023 Eq.18）

當 `â_x = a_x` 且 `x̂_D = x_D` 時：
```
δx[k+1] = λ_c · δx[k] − ε[k]
```
ε[k] 為零均值白噪組合（thermal + measurement）。

---

## 2. State Vector Definition

### 7-state（integrated random walk）— 鎖定架構

```
x_e[k] = [ δx_1[k], δx_2[k], δx_3[k], x_D[k], δx_D[k], a_x[k], δa_x[k] ]ᵀ
```

| Index | State | 物理意義 | 隨機模型 |
|---|---|---|---|
| 1 | `δx_1[k]` | δx[k−2]（最舊，量測對應） | (deterministic shift) |
| 2 | `δx_2[k]` | δx[k−1] | (deterministic shift) |
| 3 | `δx_3[k]` | δx[k]（當前） | closed-loop dynamic |
| 4 | `x_D[k]` | lumped disturbance（位置） | x_D[k+1] = x_D[k] + δx_D[k] |
| 5 | `δx_D[k]` | x_D 增量速率 | random walk: δx_D[k+1] = δx_D[k] + w_xD |
| 6 | `a_x[k]` | motion gain | a_x[k+1] = a_x[k] + δa_x[k] |
| 7 | `δa_x[k]` | a_x 增量速率 | random walk: δa_x[k+1] = δa_x[k] + w_a |

設計理由：a_x(t) 對動態軌跡（1Hz osc）是平滑訊號，「速度」δa_x 是有意義量。Integrated random walk 模型與訊號吻合，KF 跟蹤無相位 lag；對應 Q 矩陣只需在速度 state 設驅動，Q_aa = Q_xD,xD = 0。

### 歷史：5-state 1st-order random walk

Task 01 baseline（已驗證可觀但對動態 a_x(t) 跟蹤有 lag），結構：
```
x_e[k] = [ δx_1, δx_2, δx_3, x_D, a_x ]ᵀ      (5 維)
x_D, a_x 直接 random walk
```
作為歷史記錄保留於 `test_script/check_observability_eq17.m`。實作時不採用此版本。

---

## 3. F_e 矩陣（7×7 時變）

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

**時變元素**：僅 `F_e(3,6) = −f_d[k]`。其餘 48 個元素皆為常數。

**Cross-couplings**（速度進入位置的 1）：
- F_e(4,5) = 1：x_D[k+1] = x_D[k] + δx_D[k]
- F_e(6,7) = 1：a_x[k+1] = a_x[k] + δa_x[k]

### Row 3 推導摘要

```
δx[k+1] = λ_c · δx[k]                           (paper Eq.18 主項)
        − (a_x − â_x) · f_d[k]                   (gain 估測誤差)
        − (x_D − x̂_D)                           (disturbance 估測誤差)
        + zero-mean noise
```

對 state 線性化（δx_D 與 δa_x 不直接進入 δx[k+1]，只透過下一步間接）：
- ∂δx_3[k+1] / ∂δx_3[k] = λ_c
- ∂δx_3[k+1] / ∂x_D[k]  = −1
- ∂δx_3[k+1] / ∂a_x[k]  = −f_d[k]
- ∂δx_3[k+1] / ∂δx_D[k] = 0
- ∂δx_3[k+1] / ∂δa_x[k] = 0

完整推導見 design.md §5。

---

## 4. H 矩陣（雙量測，含 a_xm 內生 d=2 延遲）

paper 2025 §II.F 末尾明示：a_xm[k] 對應 a_x[k−d]（d=2 步延遲，繼承自 δx_m）。正確處理是把 a_x[k−d] 用當前 state 線性表達：

```
a_x[k−d] = a_x[k] − d·δa_x[k] + Σ_{j=1}^{d} (d−j+1)·w_a[k−j]
                                └────────┬─────────┘
                                  past process noise → effective R_2
```

對應的 H 矩陣與 effective measurement noise：

```
y[k] = [ δx_m[k] ]      ← raw 延遲量測 = δx[k−d] + n_x
       [ a_xm[k]  ]      ← IIR 推導（含 d 步延遲傳導）

H = [ 1  0  0  0  0   0    0 ]    (2 × 7)
    [ 0  0  0  0  0   1   −d ]    ← (2,7) = −d, 對 d=2 為 −2

R_2_effective = R_2_intrinsic + (Σ_{j=1}^{d}(d−j+1)²) · Q77
              = R_2_intrinsic + 5·Q77   (對 d=2)
```

第 1 列觀測 δx_1（state index 1），第 2 列觀測 a_x − d·δa_x（含 δa_x 耦合）。δx_D 與 δa_x 透過時序差分識別。

**可觀性**（數學層）：rank 永遠 = 7，與 f_d 任何行為無關（Task 01 驗證）。
**可觀性**（實務層）：受 R_2[k] 影響，IIR 崩塌時 R_2 → ∞，y_2 通道退化但結構性可觀仍由 y_1 鏈承擔（在 PE 滿足時）。

### IIR 推導 a_xm（paper 2025 Eq.9–13，**線性反解**）

```
δx̄_m[k+1]   = (1−a_var) · δx̄_m[k] + a_var · δx_m[k+1]                       (Eq.9)
δx_r[k]     = δx_m[k] − δx̄_m[k]                                              (random component)
σ̂²_δxr[k+1] = (1−a_cov) · σ̂²_δxr[k] + a_cov · (δx_r²[k+1] − δx̄_r²[k+1])      (Eq.10)

a_xm[k] = (σ̂²_δxr[k] − C_n · σ²_n_s) / (C_dpmr · 4kBT)                      (Eq.13)

  C_dpmr = 2 + 1/(1−λ_c²)      (paper 2025 Eq.11/12 閉式，粗糙版)
  C_n    = 2/(1+λ_c)
```

**重要**：a_xm 是**線性反解**（σ²_δxr 是 a_x 的線性函數）。**不是 sqrt**。a_xm 視為 a_x 的「噪聲量測」（含 d 步延遲傳導），加入 KF。

---

## 5. Q 矩陣（unified version，Task 02）

7×7 對角結構，三個非零項，全部為 â_x（與 h̄）函數：

```
       δx_1  δx_2  δx_3       x_D  δx_D    a_x  δa_x
   ┌                                                          ┐
   │  0     0     0           0    0       0    0             │
   │  0     0     0           0    0       0    0             │
   │  0     0    Q33,i[k]    0    0       0    0             │   ← Path A′ inflation
Q_i = │  0     0     0           0    0       0    0             │
   │  0     0     0           0   Q55      0    0             │   ← simulation 設 0
   │  0     0     0           0    0       0    0             │
   │  0     0     0           0    0       0    Q77,i[k]      │   ← adaptive
   └                                                          ┘
```

per-axis i ∈ {x, y, z}。

### Q 對角性的前提（A1–A4）

1. f_T 與 n_x 獨立（thermal vs sensor，物理隔離）
2. w_xD 與 (f_T, n_x) 獨立（modeling 選擇）
3. w_a 與 (f_T, n_x, w_xD) 獨立（modeling 選擇）
4. 三者皆白噪聲（時間上獨立）

對 simulation 全部成立。**single-time Q off-diagonal 嚴格 0**，但 ε[k] 在時間上 MA(d) 自相關（由 Path A′ inflation 補償）。

### Q33,i[k]（thermal + sensor，Path A′ inflation 形式）

```
┌─────────────────────────────────────────────────────────────────────┐
│  Q33,i[k] = (3 − 2·λ_c²) · 4kBT · â_x,i[k]  +  (1 − λ_c)² · σ²_n_s,i │
└─────────────────────────────────────────────────────────────────────┘
```

對 λ_c=0.7：因子 (3−2λ_c²) = **2.02**（比 marginal Var(ε) 的 1.18 多 1.71×）。

**為什麼用 inflation factor**：marginal Var(ε) 是 ε 的「平均強度」，但 ε 是 MA(d=2) 過程，自相關 33% (lag1) / 25% (lag2)。直接用 marginal 餵 KF 會 underestimate 穩態 Var(δx)。Inflation 讓 KF 用「等效白噪」假設下的穩態預測匹配 paper 2023 Eq.(22) 真值。

殘餘 5–15% gap 在 KF gain 結構次優（無法靠單參數補償），可接受。詳細推導見 design.md §8.2。

### Q55（disturbance velocity，simulation 場景）

```
Q55 = 0    (simulation 無殘磁)
```

實機部署若有殘磁，需從殘磁磁通變化率推。可加 floor=1e-12 避免數值鎖死。

### Q77,i[k]（gain velocity，adaptive per-axis 時變）

從 a_x(h(t)) 對 prescribed trajectory 的二階導推：
```
┌──────────────────────────────────────────────────────────────────┐
│  Q77,i[k] = (â_x,i[k])² · K_h,i(h̄[k])² · (Δt/R)² · σ²_ḣ_max     │
└──────────────────────────────────────────────────────────────────┘

K_h,i(h̄) := (1/C_i(h̄)) · (dC_i/dh̄)        per-axis 壁面敏感度
σ²_ḣ_max  := max_{t} ḣ²(t)                 trajectory 上界（離線算）
```

對 osc trajectory（A=10μm, f=1Hz）：σ²_ḣ_max = (2π·10)² ≈ 3947 (μm/s)²。

**對動態軌跡的價值**：Q77 在過 wall (h̄~1.1) 時自動 inflate ~600× 給 KF agility；遠離 wall 時 Q77 縮回避免雜訊敏感。

### 時變性與更新

| 項 | 時變？ | 每步計算量 |
|---|---|---|
| Q33,i[k] | ✓ | 1 次 × |
| Q55 | ✗ (=0) | 不更新 |
| Q77,i[k] | ✓ (â_x², h̄ 雙重) | lookup + 2 次 × |

詳細推導與 caveats 見 design.md §8。

---

## 6. Notation 對照表

| 符號（本架構） | Paper 2023 | Paper 2025 | qr writeup_architecture.tex |
|---|---|---|---|
| Δt | Δt | Δt | Ts |
| λ_c | λ_c | λ_c | λ_c |
| a_x | (Δt/γ) | a_x | a_x |
| δx_1, δx_2, δx_3 | δx (scalar) | δx_1, δx_2, δx_3（順序相反） | δx_1, δx_2, δx_3 |
| x_D | (not defined) | x_D | x_D |
| f_T | f_T | f_T | f_T |
| n_x | n_x | n_xs | n_x |
| δx_m | δx_m | δx_m | δx_m |
| a_xm | (not defined) | a_xm | a_xm |
| δx_D, δa_x | (not used) | δx_D, δa_x | δx_D, δa_x |
| C_n, C_dpmr | (closed form Eq.22 內) | (not defined) | C_n, C_dpmr |

注意：本主線**不**使用 qr writeup 的 `C_dpmr` / `C_n` 抽象（那是為了 paper 2025 控制律 + 7-state EKF 寫的）；本主線控制律不同，靠 7-state augmented Lyapunov 直接算閉迴路 variance。

---

## 7. 與 paper 2025 7-state EKF 的關鍵差異

兩條路線都用 7-state integrated random walk，差別在控制律：

| 項 | qr 路線 (paper 2025) | 本主線 (paper 2023 Eq.17) |
|---|---|---|
| 控制律來源 | paper 2025 Eq.6 | paper 2023 Eq.17 |
| 控制律延遲處理 | 用 estimator 預測 δx̂_3，控制律使用估計值 | Eq.17 自帶延遲補償，用 δx_m 直接餵回 |
| δx̂ 是否進入控制律 | ✓ 是（feedback term 用 δx̂） | ✗ 不需要 |
| IIR 角色 | δx̄_m 進控制 feedback + a_xm 進 KF | 只 a_xm 進 KF（不進控制 feedback） |
| Closed-loop variance | 11-state augmented Lyapunov + C_dpmr/C_n 抽象 | 7-state augmented Lyapunov 直接解 |
| 時變處理 | F_e[k] 含 f_dx[k] | F_e[k] 含 f_d[k]（位置略有不同：(3,6) vs (3,7)） |
| 已知 audit 問題 | R(1,1) per-axis、Q(3,3) linear/quadratic、Q(7,7) frozen behavior | 預期繞過（不同控制律 → 不同 Q/R 推導路徑） |

---

## 8. 實作對應檔案（規劃，尚未建立）

| 模組 | 檔案路徑 |
|---|---|
| 控制律 + EKF 一體化 | `model/controller/motion_control_law_eq17_7state.m` |
| 參數計算 | `model/controller/calc_ctrl_params.m`（既存，需擴充） |
| 觀測性檢查 | `test_script/check_observability_eq17.m` ✓（5+7-state 各驗證） |
| Q/R 設計 | `test_script/compute_qr_eq17.m` |
| 端到端驗證 | `test_script/verify_eq17_7state.m` |

對應 simulation 設定變更：`config.controller_type = 'eq17_7state'`（命名待定）。
