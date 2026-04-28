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

### 5-state 最簡形式（random walk 起點）

```
x_e[k] = [ δx_1[k], δx_2[k], δx_3[k], x_D[k], a_x[k] ]ᵀ
```

| Index | State | 物理意義 | 隨機模型 |
|---|---|---|---|
| 1 | `δx_1[k]` | δx[k−2]（最舊，量測對應） | (deterministic shift) |
| 2 | `δx_2[k]` | δx[k−1] | (deterministic shift) |
| 3 | `δx_3[k]` | δx[k]（當前） | closed-loop dynamic |
| 4 | `x_D[k]` | lumped disturbance | random walk |
| 5 | `a_x[k]` | motion gain | random walk |

### 後續延伸（integrated random walk, 7-state）

預留升級空間：
```
x_e[k] = [ δx_1, δx_2, δx_3, x_D, δx_D, a_x, δa_x ]ᵀ
```
其中 `δx_D, δa_x` 為對應 state 的「速度」項，主 state 改為 integration of velocity（paper 2025 風格）。**升級時機**：當 5-state random walk 在 1Hz osc 上跟蹤誤差過大時。

---

## 3. F_e 矩陣（5×5 時變）

```
         δx_1   δx_2   δx_3   x_D    a_x
        ┌                                      ┐
δx_1   │  0     1      0      0      0          │
δx_2   │  0     0      1      0      0          │
δx_3   │  0     0     λ_c    −1     −f_d[k]     │  ← 唯一時變項
x_D    │  0     0      0      1      0          │
a_x    │  0     0      0      0      1          │
        └                                      ┘
```

**時變元素**：僅 `F_e(3,5) = −f_d[k]`。其餘元素皆為常數。

### Row 3 推導摘要

```
δx[k+1] = λ_c · δx[k]                           (paper Eq.18 主項)
        − (a_x − â_x) · f_d[k]                   (gain 估測誤差)
        − (x_D − x̂_D)                           (disturbance 估測誤差)
        + zero-mean noise
```

對 state 線性化：
- ∂δx_3[k+1] / ∂δx_3[k] = λ_c
- ∂δx_3[k+1] / ∂x_D[k]  = −1
- ∂δx_3[k+1] / ∂a_x[k]  = −f_d[k]

完整推導見 design.md §5。

---

## 4. H 矩陣（兩個量測配置）

### 配置 A — 單量測（無 IIR）

```
y[k] = δx_m[k]
H_A  = [ 1  0  0  0  0 ]    (1 × 5)
```

**可觀性**：需要 PE — `f_d` 必須在窗口內變動。靜止 positioning 永遠不可觀。

### 配置 B — 雙量測（含 IIR-derived a_xm）

```
y[k] = [ δx_m[k] ]      ← raw 延遲量測
       [ a_xm[k]  ]      ← IIR 副產品

H_B = [ 1  0  0  0  0 ]    (2 × 5)
      [ 0  0  0  0  1 ]
```

**可觀性**（數學層）：rank 永遠 = 5，無 PE 需求。
**可觀性**（實務層）：受 R_2[k] 影響，IIR 崩塌時退回配置 A。

### IIR 推導 a_xm（paper 2025 Eq.9–13）

```
δx̄_m[k+1]   = (1−a_var) · δx_m[k+1] + a_var · δx̄_m[k]
σ²_δxr[k+1] = (1−a_var) · (δx_m[k+1] − δx̄_m[k+1])² + a_var · σ²_δxr[k]
a_xm[k]     = sqrt( (σ²_δxr[k] − C_n · σ²_n) / (C_dpmr · σ²_T,nominal[k]) )
```

a_xm 視為 a_x 的「噪聲量測」，加入 KF。

---

## 5. Notation 對照表

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
| C_n, C_dpmr | (closed form Eq.22 內) | (not defined) | C_n, C_dpmr |

注意：本主線**不**使用 qr writeup 的 `C_dpmr` / `C_n` 抽象（那是為了 7-state EKF 寫的）；本主線靠 5-state Lyapunov 直接算閉迴路 variance。

---

## 6. 與 7-state EKF 的關鍵差異

| 項 | 7-state (qr) | 5-state (本主線) |
|---|---|---|
| State 數 | 7 | 5 |
| Q 矩陣旋鈕 | 5+ 個獨立可調 | 3 個 |
| 控制律延遲處理 | 透過 estimator 預測 δx̂_3 | Eq.17 自帶（用 δx_m 直接餵回） |
| Closed-loop variance | 11-state augmented Lyapunov + C_dpmr/C_n 抽象 | 5-state Lyapunov 直接解 |
| IIR 角色 | 提供 δx̄_m + a_xm（前者進控制 feedback） | 只提供 a_xm（不進控制 feedback） |
| 時變處理 | F_e[k] 含 f_dx[k]（與本主線同） | F_e[k] 含 f_d[k] |
| 已知 audit 問題 | R(1,1) per-axis、Q(3,3)、Q(7,7) 三個 | 由架構簡化避免 |

---

## 7. 實作對應檔案（規劃，尚未建立）

| 模組 | 檔案路徑 |
|---|---|
| 控制律 + EKF 一體化 | `model/controller/motion_control_law_eq17_5state.m` |
| 參數計算 | `model/controller/calc_ctrl_params.m`（既存，需擴充） |
| 觀測性檢查 | `test_script/check_observability_eq17.m` |
| Q/R 設計 | `test_script/compute_qr_eq17.m` |
| 端到端驗證 | `test_script/verify_eq17_5state.m` |

對應 simulation 設定變更：`config.controller_type = 'eq17_5state'`（命名待定）。
