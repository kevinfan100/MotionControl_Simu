# Eq.17 + 5-state EKF Verification Notes

**累積式驗證紀錄**。本檔追蹤 `test/eq17-5state-ekf` 分支上每個驗證 task 的結果、與 paper 對照、以及跨 task 的關鍵發現。

對應架構文件：[`eq17-architecture.md`](eq17-architecture.md)
對應 design 來源：[`../reference/eq17_analysis/design.md`](../reference/eq17_analysis/design.md)

---

## Verification Milestones

每完成一個 task 在此追加一列。

| # | Task | Date | Status | Report 連結 | 關鍵結論 |
|---|---|---|---|---|---|
| 01 | Math-layer observability rank test | 2026-04-28 | DONE | [task01_math_observability_report.md](../reference/eq17_analysis/task01_math_observability_report.md) | 10/10 PASS；發現 PE 條件精確邊界：N 窗口下只有 f_d_seq(1..N-3) 影響 (x_D,a_x) 子塊 rank。Config B (含 a_xm) 任意 f_d 下 rank=5 |
| 02 | Observability Gramian σ_min(t) on osc trajectory | — | TODO | — | — |
| 03 | Q matrix first-principles derivation | — | TODO | — | — |
| 04 | R matrix design (incl. IIR-induced σ²_n_axm) | — | TODO | — | — |
| 05 | 5-state EKF MATLAB implementation | — | TODO | — | — |
| 06 | Closed-loop variance Lyapunov | — | TODO | — | — |
| 07 | End-to-end vs 7-state EKF comparison | — | TODO | — | — |
| 08 | Integrated random walk upgrade | — | TODO | — | — |

`Status` 用：TODO / IN-PROGRESS / DONE / BLOCKED。

---

## Key Findings

跨 task 累積的重要發現，等驗證實際展開後填入。

### 架構層級

- **PE-window 精確條件 (Task 01)**：對窗口 N，Config A 的 (x_D, a_x) 子塊只受 f_d_seq(1..N−3) 影響。窗口尾端 N−2 之後的變動**不**改變 rank。Task 02 的 Gramian 窗口設計需要套用此條件。
- **Config B 的 IIR 「保險」效果 (Task 01)**：a_xm 量測直接給 a_x 通道，rank=5 與 f_d 無關。在 hold 階段（f_d≈0）配置 A 嚴格不可觀，配置 B 仍 rank=5（受 IIR 收斂速度限制但結構可觀）。

### 數值結果

- **Task 01**：所有 10 個 case rank 預測與 MATLAB 計算精確一致（含 A5/A6/A7 邊界 sweep）

### 與 paper 對照

- _(待累積)_

---

## 預期 Paper Benchmarks

### Paper 2023 (Meng & Menq, T-Mech vol.28)

| Figure | 內容 | 我們複現方式 | 預期吻合度 |
|---|---|---|---|
| Fig.9 | Tracking std vs λ_c (paper experiment) | 5-state Lyapunov 解析 | < 5% |
| Fig.10 | 2-D sine wave 5Hz tracking | osc trajectory simulation | (待 task 06) |
| Fig.11 | 2-D triangle wave tracking | (out of scope) | — |
| Fig.13 | PSD comparison (PI vs model-based) | controller_type=eq17_5state vs PI baseline | (待 task 06) |

### Paper 2025 (Meng/Long/Menq, IEEE TIE)

| 對照項 | 我們的對應 | 預期 |
|---|---|---|
| 7-state EKF Q 設計 (paper §IV) | 5-state EKF Q (本主線 task 02) | 數量級相同，公式更簡 |
| a_hat std with IIR | 配置 B 的 σ_min(t) Gramian 預測 | (待 task 01) |
| 雙量測 PE-free 可觀 | 配置 B 數學 rank 證明 | 已在 design.md 完成 |

---

## 已知 Caveats

### IIR 崩塌條件（影響配置 B 實務可觀性）

1. **Warm-up 期** (~50 ms @ a_var=0.05)：σ²_δxr 還沒收斂
2. **Thermal-to-noise 比例太低**：分子 σ²_δxr − C_n·σ²_n < 0
3. **a_x 變化快過 IIR 窗** (>5 Hz)：a_xm 有延遲與 bias
4. **接近 wall** (h_bar ~ 1.1)：C_⊥ lookup 誤差大

設計時 R_2[k] 必須自適應（task 03 處理）。

### Quasi-static 假設邊界（影響時變 γ 處理）

控制律時間常數 (1−λ_c)·Δt = 2 ms (λ_c=0.7)。
γ(h(t)) 變化時間常數對 1Hz osc ≈ 160 ms。
比值 80×，quasi-static 在 control-loop 層級成立。
但 estimator 收斂時間若 > 50 ms（如 frozen Q），會跟不上 γ̇。

→ Q[k] 必須跟 trajectory 動態強度走，不能 frozen。

---

## Cross-References to qr Branch（lessons learned）

qr 分支累積的相關發現，**部分可借用、部分不適用**：

| qr findings | 對本主線的相關性 |
|---|---|
| `R(1,1) per-axis` audit issue | 不存在（5-state 沒有那個 R(1,1) 結構） |
| `Q(3,3) linear vs quadratic` | 不存在（5-state Q_thermal 直接從 a_x²·γ(h) 算） |
| `Q(7,7) frozen behavior` | 不存在（5-state 無 δa_x） |
| `frozen_correct preset` (a_hat 4%) | 不適用（dynamic trajectory 必須 dynamic Q） |
| `Level 1–6 chi-sq chain for a_hat std` | 大幅簡化（5-state 沒有 a_x estimation 的 cascaded LP 結構） |
| `wall-aware initial a_hat`（paper 2025 風格） | 借用（5-state EKF 初始化也應 wall-aware） |
| `per-axis Einstein scaling for thermal` | 借用（task 02 的 Q_thermal 推導要 per-axis） |
| `compute_rho_a_rigorous.m` | 不適用（5-state 無此項） |

---

## Update Log

每次重要更新追加一列，格式：日期 | commit hash | 摘要。

| Date | Commit | Update |
|---|---|---|
| 2026-04-28 | (init) | 建立 skeleton |
| 2026-04-28 | (task01) | Task 01 DONE — math-layer observability ranks confirmed + PE-window finding |
