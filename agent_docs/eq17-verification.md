# Eq.17 + 7-state EKF Verification Notes

**累積式驗證紀錄**。本檔追蹤 `test/eq17-5state-ekf` 分支上每個驗證 task 的結果、與 paper 對照、以及跨 task 的關鍵發現。

對應架構文件：[`eq17-architecture.md`](eq17-architecture.md)
對應 design 來源：[`../reference/eq17_analysis/design.md`](../reference/eq17_analysis/design.md)

---

## Verification Milestones

每完成一個 task 在此追加一列。

| # | Task | Date | Status | Report 連結 | 關鍵結論 |
|---|---|---|---|---|---|
| 01 | Math-layer observability rank test | 2026-04-28 | DONE | [task01_math_observability_report.md](../reference/eq17_analysis/task01_math_observability_report.md) | 5-state 10/10 + 7-state 3/3 = 13/13 PASS。雙回授下 7-state rank=7 對任意 f_d 成立（含 f_d=0）。發現 Config A 5-state PE 條件精確邊界：N 窗口下只有 f_d_seq(1..N-3) 影響 (x_D,a_x) 子塊 rank。 |
| 02 | Q matrix first-principles derivation | 2026-04-28 | DONE | design.md §8 | Q33,i[k] = 4kBT·a_x,i[k]·[1+d·(1−λ_c)²] + (1−λ_c)²·σ²_n_s,i（純開迴路，比 paper 2025 多 18% 修正）；Q55=0；Q77,i = Var(δa_x_true,i) over trajectory。同時更正 a_xm 公式為線性反解（不是 sqrt），並加入 a_xm 2-step 延遲到 H 矩陣 (col 7 = −d)。 |
| 03 | R matrix design (incl. IIR-induced σ²_n_axm) | — | TODO | — | — |
| 04 | 7-state EKF MATLAB implementation | — | TODO | — | — |
| 05 | Closed-loop variance Lyapunov | — | TODO | — | — |
| 06 | End-to-end vs qr 7-state EKF comparison | — | TODO | — | — |

`Status` 用：TODO / IN-PROGRESS / DONE / BLOCKED。

---

## Key Findings

### 架構層級

- **7-state Config B 的數學可觀性 (Task 01)**：rank=7 與 f_d 任何行為無關（含 f_d=0 hold 階段）。a_x、δa_x 透過 a_xm 量測直接 / 差分識別；x_D、δx_D 透過 y_1 鏈 m=3, m=4 row 結合 −1 係數識別。**整套 KF 不需要 PE 也能識別所有 7 個 state**（前提：a_xm 健康）。
- **5-state historical (Task 01 baseline)**：1st-order random walk 版本可觀但對動態 a_x(t) 跟蹤有 lag。Config A 路線額外發現 PE 視窗 = 前 N−3 個樣本的精確邊界。已退役為歷史記錄。

### 數值結果

- **Task 01 (5-state)**：10 個 case 全 PASS，含 A5/A6/A7 邊界 sweep
- **Task 01 (7-state)**：3 個 case 全 PASS（f_d=0、f_d=2 const、f_d ramp），rank=7 確認

### 推導結果

- **Task 02 — Q 矩陣（純代數，純開迴路）**：
  - **Q33,i[k]** = 4kBT · a_x,i[k] · [1 + d·(1−λ_c)²] + (1−λ_c)² · σ²_n_s,i
    - 比 paper 2025 Eq.(21) 多一個 d·(1−λ_c)² 修正（18% @ d=2, λ_c=0.7）
    - 來源差異：我們的 Eq.17 控制律含過去 d 步 thermal 史的 (1−λ_c) 加權項
  - **Q55** = 0（simulation 場景無殘磁）
  - **Q77,i** = Var_{t}( δa_x_true,i(t) ) — 離線從軌跡計算
  - 時變項只有 Q33（透過 a_x[k]）

- **a_xm 公式更正**：從錯誤的 sqrt 形式改為 paper 2025 Eq.(13) 線性形式
  - `a_xm[k] = (σ²_δxr[k] − C_n·σ²_n_s) / (C_dpmr · 4kBT)`
  - C_dpmr = 2 + 1/(1−λ_c²), C_n = 2/(1+λ_c)（粗糙版）

- **a_xm 2-step 延遲處理**：H 矩陣 col 7 從 0 改為 −d
  - H = [[1,0,0,0,0,0,0]; [0,0,0,0,0,1,−d]]
  - Effective R_2 多 5·Q77（d=2）的延遲傳導項

### 與 paper 對照

- _(待累積)_

---

## 預期 Paper Benchmarks

### Paper 2023 (Meng & Menq, T-Mech vol.28)

| Figure | 內容 | 我們複現方式 | 預期吻合度 |
|---|---|---|---|
| Fig.9 | Tracking std vs λ_c (paper experiment) | 7-state Lyapunov 解析 | < 5% |
| Fig.10 | 2-D sine wave 5Hz tracking | osc trajectory simulation | (待 task 06) |
| Fig.11 | 2-D triangle wave tracking | (out of scope) | — |
| Fig.13 | PSD comparison (PI vs model-based) | controller_type=eq17_7state vs PI baseline | (待 task 06) |

### Paper 2025 (Meng/Long/Menq, IEEE TIE)

| 對照項 | 我們的對應 | 預期 |
|---|---|---|
| 7-state EKF Q 設計 (paper §IV) | 7-state EKF Q (本主線 task 02) | 結構相似，但繞過 paper 2025 audit 三大問題 |
| a_hat std with IIR | 雙回授 a_xm 量測直接識別 a_x | 由 7-state Lyapunov 解析（task 05） |
| 雙量測 PE-free 可觀 | 數學 rank=7 證明 | 已在 task 01 驗證 |

---

## 已知 Caveats

### IIR 崩塌條件（影響雙回授實務可觀性）

1. **Warm-up 期** (~50 ms @ a_var=0.05)：σ²_δxr 還沒收斂
2. **Thermal-to-noise 比例太低**：分子 σ²_δxr − C_n·σ²_n < 0
3. **a_x 變化快過 IIR 窗** (>5 Hz)：a_xm 有延遲與 bias
4. **接近 wall** (h_bar ~ 1.1)：C_⊥ lookup 誤差大

設計時 R_2[k] 必須自適應（task 03 處理）。當 a_xm 失效，y_2 通道 R_2 → ∞，系統退化但仍由 y_1 鏈承擔（在 PE 滿足時）。

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
| `R(1,1) per-axis` audit issue | 借鑑 — 我們也採 per-axis R_1，避開原問題 |
| `Q(3,3) linear vs quadratic` | 待重新推導 — 控制律不同，Q_thermal 公式可能不同 (task 02) |
| `Q(7,7) frozen behavior` | 待測試 — 我們也有 δa_x state，但動態軌跡 Q 不能 frozen |
| `frozen_correct preset` (a_hat 4%) | 不適用 — dynamic trajectory 必須 dynamic Q |
| `Level 1–6 chi-sq chain for a_hat std` | 部分適用 — 7-state 結構類似 paper 2025，但 closed-loop dynamics 不同 |
| `wall-aware initial a_hat`（paper 2025 風格） | 借用 — 7-state EKF 初始化也應 wall-aware |
| `per-axis Einstein scaling for thermal` | 借用 — task 02 的 Q_thermal 推導要 per-axis |
| `compute_rho_a_rigorous.m` | 待評估 — paper 2023 控制律下 a_x autocorrelation 結構不同 |

---

## Update Log

每次重要更新追加一列，格式：日期 | commit hash | 摘要。

| Date | Commit | Update |
|---|---|---|
| 2026-04-28 | (init) | 建立 skeleton |
| 2026-04-28 | (task01) | Task 01 DONE — math-layer observability ranks confirmed + PE-window finding |
| 2026-04-28 | (lock-7state) | 7-state architecture locked in；agent_docs 同步更新 |
| 2026-04-28 | (task02-Q) | Task 02 DONE — Q 矩陣三項代數推導（Q33[k], Q55=0, Q77）；a_xm 公式改線性；H 加入 d-step 延遲修正 (col 7 = −d) |
