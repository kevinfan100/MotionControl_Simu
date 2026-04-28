# Design: Eq.17 控制律 + 5-state Reduced EKF

**Branch**: `test/eq17-5state-ekf`
**Base**: `origin/main` (5681455)
**Date**: 2026-04-28
**Status**: Initial design snapshot — captures discussion accumulated up to branch creation

本檔案是新分支的 entry-point design，記錄當前對「paper 2023 Eq.17 控制律 + reduced-state EKF」這條主線的設計決策。後續實作前若有重大設計變更，應新增 `design_v2.md`，本檔不刪。

---

## 1. Motivation 與 Scope

### 為什麼從 7-state EKF 切到 Eq.17 + reduced-state

qr 分支上的 7-state EKF（paper 2025 Meng/Long/Menq 風格）累積了已知的 Q/R 設計問題，部分根本原因來自架構本身：

| 問題（qr branch parameter audit） | 來源 |
|---|---|
| R(1,1) 非 per-axis（260×–30000× 過估） | EKF 把全軸共用一個 R |
| Q(3,3) linear vs quadratic in a_r 模糊 | δx 三胞胎 + thermal 模型耦合 |
| Q(7,7) = 1e-8 vs 0 frozen behavior 衝突 | δa_x random walk 與 a_x random walk 互相牽動 |
| a_hat std x-axis 5–7× under-prediction | per-axis EKF coupling 未解 |

切換到 Eq.17 + reduced-state 後，這些問題的**根本來源**有的直接消失（無 R/Q 矩陣某些塊），有的因架構簡化而變得可分析。

### Scope 範圍

In scope：
- Plant 模型不變（paper 2023 Eq.13–14）
- 控制律改為 paper 2023 Eq.(17) d-step delay-compensated 形式
- Estimator: 5-state 最簡 random walk 起點，後續可升級
- LTV observability 分析（含 Gramian σ_min(t) 視覺化）
- Q/R 第一性推導（不靠試誤）
- 時變矩陣處理（F_e[k]、Q[k]、R[k] 全部 [k]-indexed）

Out of scope：
- 7-state EKF 章節（保留在 qr archive，本分支不使用）
- 殘磁/磁滯補償（simulation 環境無此項）

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
| `a_x[k]` | 時變 motion gain | **✓ 需估** → state #1 |
| `Σ f_T[k−i]` | 過去 d 步 thermal 史 | ✗ 白噪聲零均值，吃進 ε[k] |
| `x_D[k]` | lumped disturbance | **✓ 需估** → state #2 |

加上量測方程的延遲約束（`y[k] = δx[k−2] + n_x`），需要 δx 三胞胎做 buffer：

```
x_e[k] = [ δx_1[k], δx_2[k], δx_3[k], x_D[k], a_x[k] ]ᵀ      (5 維)
```

### State 命名約定

- `δx_1[k] = δx[k−2]` ← 量測對應位置（最舊）
- `δx_2[k] = δx[k−1]`
- `δx_3[k] = δx[k]`     ← 當前

注意：本檔約定與 paper 2025 寫作慣例**相反**（paper 把當前狀態放第 3 位、最舊放第 1 位）。實作上保持本檔約定，避免與 H 矩陣對應位置混淆。

### 隨機模型（最簡起點）

```
a_x[k+1] = a_x[k] + w_a[k]      ← random walk
x_D[k+1] = x_D[k] + w_xD[k]    ← random walk
```

兩者皆採最簡 random walk。後續若需提升精度，可升級成 integrated random walk（paper 2025 風格 `[a_x, δa_x]` 與 `[x_D, δx_D]` 4 維）。

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

對 state 線性化（state 為 a_x 和 x_D 的真值，估計值 â_x、x̂_D 視為已知輸入）：
```
∂δx_3[k+1]/∂δx_3[k] = λ_c
∂δx_3[k+1]/∂x_D[k]  = −1
∂δx_3[k+1]/∂a_x[k]  = −f_d[k]
其他 = 0
```

→ Row 3：`F_e(3,:) = [ 0, 0, λ_c, −1, −f_d[k] ]`

### Row 4, 5：random walk

```
F_e(4,:) = [ 0, 0, 0, 1, 0 ]
F_e(5,:) = [ 0, 0, 0, 0, 1 ]
```

### 完整 F_e（5×5 時變）

```
         δx_1   δx_2   δx_3   x_D    a_x
        ┌                                      ┐
δx_1   │  0     1      0      0      0          │
δx_2   │  0     0      1      0      0          │
δx_3   │  0     0     λ_c    −1     −f_d[k]     │
x_D    │  0     0      0      1      0          │
a_x    │  0     0      0      0      1          │
        └                                      ┘
```

**唯一時變項**在 (3,5) = `−f_d[k]`。其他元素皆為常數。

---

## 6. 量測架構（雙回授）

KF 使用兩條量測通道：raw 延遲量測 + IIR 推導的 motion gain 量測。

```
y[k] = [ δx_m[k] ]      ← raw 延遲量測：δx[k−2] + n_x[k]
       [ a_xm[k]  ]      ← IIR 副產品：由 σ²_δxr 反推的 motion gain 量測

H = [ 1  0  0  0  0 ]
    [ 0  0  0  0  1 ]
```

### IIR 不在控制 feedback 路徑上

控制律 Eq.(17) 中 `δx_m[k]` 用**原始量測**（不是 IIR 過濾後）。理由：
- IIR 動態若進入 closed-loop，會破壞 paper Eq.18 的 λ_c·δx[k] 形式
- IIR 純粹當 estimator 的 pre-processor，產出 a_xm 餵進 KF measurement update

### a_xm 由 IIR 推導

paper 2023/2025 的 IIR 機制：
```
δx̄_m[k+1]   = (1−a_var) · δx_m[k+1] + a_var · δx̄_m[k]              (LP mean)
σ²_δxr[k+1] = (1−a_var) · (δx_m[k+1] − δx̄_m[k+1])² + a_var · σ²_δxr[k]   (variance)
a_xm[k]     = sqrt( (σ²_δxr[k] − C_n · σ²_n) / (C_dpmr · 4kBT γ(C(h))/Δt) )
```

a_xm 視為 a_x 的「噪聲量測」，加入 KF 後直接給 a_x 一個觀測通道。

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

判斷準則：rank(O) = n_state = 5 ⇔ 在該窗口可觀。

### Window length

- m_out = 2（雙量測），n_state = 5 → N ≥ ⌈5/2⌉ = 3，保險取 **N = 4**

### 結論：rank(O) = 5 與 f_d 無關

- a_xm 量測（H 第 2 列 = `[0,0,0,0,1]`）直接識別 a_x
- δx 三胞胎透過前三步觀測直接識別 (δx_1, δx_2, δx_3)
- x_D 透過 m=3 觀測 row（係數 −1，**與 f_d 完全無關**）+ a_x 已知 → 解出
- → **5 個 state 全部可獨立識別，f_d 任何行為（包括 hold 階段 f_d=0）下都成立**

完整逐步推導與 MATLAB rank 驗證見 [task01_math_observability_report.md](task01_math_observability_report.md)。

### 「永遠可觀」要分兩層

| 層次 | 結論 | 條件 |
|---|---|---|
| 數學 rank | 永遠 = 5 | a_xm 量測存在且模型有效 |
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
   - `test_script/check_observability_eq17.m`：rank=5 對任意 f_d 確認
   - 報告：[`task01_math_observability_report.md`](task01_math_observability_report.md)
   - 後續 Gramian σ_min(t) 量化評估暫不做，需要時新開 task

2. **Q 矩陣第一性推導**
   - Q_thermal[k]: thermal force 方差直接代入 (a_x, h(t))
   - Q_a: a_x random walk 強度，從 trajectory 動態強度推
   - Q_xD: lumped disturbance random walk 強度，從預期殘磁/建模誤差幅度推
   - 對比 qr branch 既存的 Q33 / Q66 / Q77 設計，標出差異

3. **R[k] 設計**
   - R_1: per-axis σ²_n（直接從 config.meas_noise_std）
   - R_2[k]: a_xm 噪聲方差，含 IIR-induced 結構（warm-up + thermal-to-noise 監測）

4. **5-state EKF 實作**
   - `model/controller/motion_control_law_eq17_5state.m`
   - 控制律 + EKF 一體化（與既存 motion_control_law_7state.m 並列，不取代）

5. **Closed-loop variance Lyapunov**
   - 5-state augmented Lyapunov 解 closed-loop 穩態方差
   - 對比 paper 2023 Eq.22 在無估測誤差極限下的閉式

6. **Trajectory 上的端到端驗證**
   - 跑 `run_simulation.m`（osc trajectory）
   - 對比 controller_type=eq17_5state vs controller_type=7
   - tracking error std、a_hat 收斂行為

7. **升級 integrated random walk**（後期）
   - state 擴成 7 維 [δx_1, δx_2, δx_3, x_D, δx_D, a_x, δa_x]
   - 重做 observability 與 Q/R 推導

---

## 9. Notation 約定

完整 notation 對照表見 [`agent_docs/eq17-architecture.md`](../../agent_docs/eq17-architecture.md) §5。MATLAB 變數名遵循 `.claude/rules/matlab-conventions.md` 的 snake_case 規範。

---

## 10. 開放問題（暫保留，等驗證再決定）

- Q_a 和 Q_xD 的物理單位推導（從第一性原理，不靠試誤）— **task 2 處理**
- a_xm 在 IIR warm-up 期的 fallback 策略 — **task 3 處理**
- 是否需要把 IIR 內部 state（δx̄_m、σ²_δxr）也納入 augmented state — **目前傾向不需要**，待 Gramian 數值結果驗證
- 控制律中 x̂_D 補償項的縮放因子（`−x̂_D` vs `−(1−λ_c)·x̂_D`）— 已選前者全量補償，task 5 Lyapunov 驗證

---

## References

- Meng, T.-M., & Menq, C.-H. (2023). *Ultra-Precise High-Speed Untethered Manipulation of Magnetic Scanning Microprobe in Aqueous Solutions*. IEEE/ASME T-Mech, vol. 28, no. 1. — Eq.(17) 控制律來源
- Meng, T.-M., Long, F., & Menq, C.-H. (2025). *Near-Wall Ultra-precise Motion Control of a Magnetically Driven Scanning Microprobe in Aqueous Solutions*. IEEE TIE, vol. 72, no. 1. — IIR + 7-state EKF 來源（本路線只借用 IIR 的 a_xm 推導，不採其控制律）
- qr 分支保留的相關文件：
  - `agent_docs/ekf-qr-analysis.md`（main 上版本）— EKF 矩陣推導參考
  - `agent_docs/math-model.md`（main 上版本）— 座標、單位、γ 計算
