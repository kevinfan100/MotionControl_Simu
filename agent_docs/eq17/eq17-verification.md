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
| 02 | Q matrix derivation (Path C strict Q33 + Path B Var(w_a) Q77) | 2026-04-29 | DONE | design.md §8 | Q33,i[k] = 4kBT·â_x,i[k] (Path C 嚴格 KF formalism)；Q55=0；Q77,i[k] = Δt⁴·â_x²·{(K_h²−K_h')²·ḣ_max⁴/(8R⁴) + K_h²·ḧ_max²/(2R²)} (Path B 完整 Var(w_a) 含 Term A+B；過 wall Term A 主導 4×Term B)。Route III 簡化版 (Q ∝ (a_nom-â_x)⁴) 評估後駁回（=Term B only，過 wall under-estimate ~80%）。需要實作 K_h, K_h' lookup（Task 04 處理）。 |
| 03 | R matrix design (incl. IIR-induced σ²_n_axm + 3-guard adaptive) | 2026-04-29 | DONE | design.md §9 | R_2_eff,i[k] = a_cov·IF_var·(â_x,i+ξ_i)² + 5·Q77,i[k]，純二次多項式 in â_x + Q77 wall 動態（per-axis sensor floor ξ_i=(C_n/C_dpmr)·σ²_n_s/(4kBT)）。常數 C_dpmr=3.96, C_n=1.18, IF_var≈4.24 (Option A, λ_c=0.7)。Step 3.5 採 3-guard 簡化版（trajectory-aware）：t < t_warmup_kf (0.2 sec) / σ²_δxr ≤ C_n·σ²_n_s (NaN guard) / h̄ < h̄_safe (1.5) → R_2 = 1e10；無 hysteresis、無 ramp-down（先別預期問題）。 |
| 04 | 7-state EKF MATLAB implementation + pure-MATLAB driver (Phase 4a-c + Phase 0) | 2026-04-29 | IN-PROGRESS | model/controller/motion_control_law_eq17_7state.m, model/pure_matlab/, test_script/unit_tests/ | **Implementation 完成 + 端到端驗證跑過**：Phase 4a-c 三個 .m 檔 + Phase 0 dual-track infra + 4 個 unit test 共 23 個測試全 PASS；S1/S2 (positioning) + F3 (osc) end-to-end 跑通，tracking std xy=24 nm, z=24-76 nm；3 bugs fixed (sequential KF update / thermal_enable gate / test reference precision)；**Open issue: a_hat 系統性偏低 29-44% — self-consistent equilibrium 問題**（非 bug，design.md §11 列為 open）；design.md C_dpmr=3.96 公式 Lyapunov 驗證正確；Pf_init 對 bias 無影響。Phase 4d/4e/4f Simulink 整合後置 |
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

- **Task 02 — Q 矩陣（Path C 嚴格 KF formalism + adaptive Q77）**：
  - **Q33,i[k]** = 4 · k_B · T · â_x,i[k]
    - **嚴格依 KF formalism**：只含當步白噪 thermal，不含其他項
    - (ii)(iii) thermal 史 (cross-step correlated)：由 paper 2023 Eq.(22) closed-loop 公式分別處理
    - (iv) n_x：在 R(1,1) 中，避免 Q-R cross
    - 代價：KF 預測 P(3,3) ≈ 真實 σ²_δx 的 50%（standard KF 對 correlated noise 的 trade-off）
    - 之前推過 Path A′ inflation 形式 (3−2λ_c²)·4kBT·â_x + (1−λ_c)²·σ²_n_s，**已退役**（理由：違反 KF Q-R 獨立與白噪 cross-step 假設，是 hack 不是推導）
  - **Q55** = 0（simulation 場景無殘磁；可加 floor=1e-12 防數值鎖死）
  - **Q77,i[k]** = Δt⁴·â_x²·{(K_h²−K_h')²·ḣ_max⁴/(8R⁴) + K_h²·ḧ_max²/(2R²)}
    - **Path B 完整 Var(w_a) 公式**，從 Var(ä_x_true) 的鏈式推導
    - Term A (ḣ² 項，2ω 諧波)：過 wall (h̄=1.11) 主導，4× Term B
    - Term B (ḧ 項，ω 基頻)：遠離 wall 主導
    - 早期 (â_x²·K_h²·(Δt/R)²·σ²_ḣ_max) 形式對應 Var(δa_x) 不是 Var(w_a)，over-estimate ~3300×，已退役
    - K_h,i, K_h,i' 函數需要 calc_correction_functions 增加 dC/dh̄ 與 d²C/dh̄² 輸出（Task 04 處理）
    - 用 measured h̄[k]（不用 KF 估的 â_x），避開 bias amplification loop
    - w_a 是 designed white driver，本來就是白噪，無 Q33 的 correlated-noise 問題

- **Route III 評估（已駁回）**：
  - 提案：Q77 = (32/81)·(a_nom−â_x)⁴·A_h²·ω⁴·Δt⁴/(a_nom²·R²)
  - 本質：Path B 的 Term B 單獨 + K_h 用近壁 c_⊥≈1+9/(8h̄) 反代成 â_x 多項式
  - 駁回理由：
    (1) 過 wall under-estimate ~80%（缺 Term A，那裡 Term A 是主導項）
    (2) bias amplification loop（Q 用 KF 估的 â_x 而非 measured h̄）
    (3) 高階 c 修正擴展性差（換 c 公式要重推多項式）
  - 保留價值：可作 sanity check（Term B 主導區，遠離 wall），對驗 Path B 數值正確性
  - **Q 對角化確認**：4 個獨立性假設下 single-time Q off-diagonal 嚴格 0
  - **完整 ε 結構處理**：(i) 進 Q33，(ii)(iii) 進 Eq.22 closed-loop，(iv) 進 R(1,1)

- **a_xm 公式更正**：從錯誤的 sqrt 形式改為 paper 2025 Eq.(13) 線性形式
  - `a_xm[k] = (σ²_δxr[k] − C_n·σ²_n_s) / (C_dpmr · 4kBT)`
  - C_dpmr = 2 + 1/(1−λ_c²), C_n = 2/(1+λ_c)（粗糙版）

- **a_xm 2-step 延遲處理**：H 矩陣 col 7 從 0 改為 −d
  - H = [[1,0,0,0,0,0,0]; [0,0,0,0,0,1,−d]]
  - Effective R_2 多 5·Q77（d=2）的延遲傳導項

- **Task 03 Step 3.1 — δx_r 閉迴路統計（Eq.19 form 嚴格推導）**：
  - **σ²_δxr,i[k] = C_dpmr · 4kBT · â_x,i[k] + C_n · σ²_n_s,i**
    - C_dpmr = 2 + 1/(1−λ_c²) ≈ 3.96 (λ_c=0.7)
    - C_n = 2/(1+λ_c) ≈ 1.18 (λ_c=0.7)
    - **嚴格從 ε MA(2) tail 結構算 ARMA inflation 得**（不是粗糙近似），結構上與 paper 2025 Eq.11/12 同形式（因兩控制律 ε 都有同 MA(2) tail）
  - **ρ_δxr(τ)** 選 Option A (MA(2) full)：ρ_δx(1)≈0.85, ρ_δx(2)≈0.67, τ≥3 衰減 λ_c
    - IF_var ≈ 4.2 (thermal-dominated, λ_c=0.7)
    - 拒絕 Option B (AR(1) approx, IF_var≈2.92)：under-estimate IF_var ~30%，與 §8.2 Eq.19 form 不一致
  - **三決定**敲定：ρ_δx 用 Option A、σ²_δxr 用 â_x[k] adaptive、σ²_n_s 用常數 per-axis
  - **F_e Row 3 的 Eq.18 vs Eq.19 form 釐清**：之前 §5 derivation 看似直接 Jacobian，實際是 Eq.18→Eq.19 代數重排把 −(1−λ_c)·δx[k−d] 項用遞迴吸收進 ε。Eq.18 form: F_e(3,1)=−(1−λ_c), (3,3)=1, ε 白但 Q-R cross；**Eq.19 form (採用): F_e(3,1)=0, (3,3)=λ_c, ε MA(2)**；trade-off 同 §8.2 Step 5
  - 下一步：**Step 3.2 chi-squared** — IIR EWMA 對 (δx_r² − δ̄x_r²) 的方差傳遞，目標 Var(σ̂²_δxr) ≈ a_cov·IF_var·(σ²_δxr)² 公式驗證

- **Task 03 Step 3.2-3.4 — Var(σ̂²_δxr) + R_2 閉式（DONE）**：
  - **Step 3.2** Var(σ̂²_δxr) = a_cov · IF_var · (σ²_δxr)²
    - IIR EWMA 線性方差傳遞 + Wick 定理（δx_r ~ N(0,σ²) 嚴格高斯，平方訊號自相關 = 原訊號自相關平方）
    - 假設：δ̄x_r ≈ 0 穩態（bias ~2.5%）+ small a_cov（IF_eff vs IF_var ~9% over-estimate）
    - 數值：σ(σ̂²_δxr)/σ²_δxr ≈ 46% (Option A, λ_c=0.7, a_cov=0.05)
  - **Step 3.3** R_2_intrinsic,i[k] = a_cov · IF_var · {â_x,i + ξ_i}²
    - 線性反解 a_xm 後得二次多項式
    - ξ_i := (C_n/C_dpmr)·σ²_n_s,i/(4kBT) 是 per-axis sensor-induced offset
    - 兩極限：sensor-dominated → 常數 noise floor；thermal-dominated → â_x² 主導
  - **Step 3.4** R_2_eff,i[k] = R_2_intrinsic,i[k] + 5·Q77,i[k]（d=2 延遲傳導）
  - **R 矩陣作為 scalar 函數**：R_i(â_x,i[k], h̄[k])，每 step per-axis 算 2×2 matrix（off-diag=0）
  - **與 Q 並列**：Q33 一次 in â_x，R(2,2) 二次 in â_x，都是 EKF/SDRE adaptive 矩陣
  - **合計精度限制**：~10–15% 各方向偏差（Eq.19 form 50% P undershoot + δ̄x_r 2.5% bias + IF_eff 9% + thermal/sensor regime crossover），合在 design.md §9.8 表
  - 下一步：**Step 3.5 自適應 R_2 gate** — warm-up / 低 SNR / 過 wall / a_x 變化太快 四條件偵測

- **Task 03 Step 3.5 — 自適應 R_2 gate（DONE，3-guard 簡化版）**：
  - **核心策略**：利用既有軌跡結構（hold→descend→oscillation）天然提供乾淨 warm-up 期；EKF 內只加 3 個獨立 guard，純 OR 邏輯
  - **4 條件 → 3 guards 整併**：
    - Guard 1: `t < t_warmup_kf` (0.2 sec)，covers warm-up
    - Guard 2: `σ²_δxr ≤ C_n·σ²_n_s` (NaN guard, prevents a_xm divergence)
    - Guard 3: `h̄ < h̄_safe` (1.5)，covers wall + a_x rate（兩者過 wall 同步觸發）
  - **觸發時 R_2 = 1e10**（KF 自然忽略 y_2，退化單回授）
  - **退化保護**：rank(O) 仍=7（Task 01 單回授 + PE 滿足下成立），â_x 改由 y_1 鏈間接識別
  - **不加 hysteresis / ramp-down**：MVP 簡化，端到端驗證有問題再加
  - **驗證項目**（轉到 Task 04 端到端）：每 guard 獨立觸發測試 + â_x 連續性檢查
  - **Task 03 Step 3.1-3.5 全部 DONE，Task 03 完整收斂**

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
| 2026-04-28 | (task02-unified) | Q 升級到 unified version: Path A′ inflation Q33 (3−2λ_c²) + adaptive Q77 (含 K_h 壁面敏感度)；補上 Q 對角化的 A1–A4 假設 |
| 2026-04-28 | (task02-strict) | Q33 退役 Path A′ inflation，改回 Path C 嚴格形式 4kBT·â_x。理由：Path A′ inflation 違反 KF Q-R 獨立與 Q 白噪 cross-step 假設，雖然能讓 KF 預測 σ²_δx 對齊 Eq.22 但是 hack 不是推導。Path C 數學嚴格但 KF P 預測偏小 ~50%（standard KF 對 correlated noise 的已知 trade-off），可接受。 |
| 2026-04-29 | (task02-q77-fix) | Q77 從 Var(δa_x) 形式 (â_x²·K_h²·(Δt/R)²·σ²_ḣ_max) 升級為 Var(w_a) 完整 Path B 形式 Δt⁴·â_x²·{(K_h²−K_h')²·ḣ_max⁴/(8R⁴)+K_h²·ḧ_max²/(2R²)}。理由：原式對應一階差分 Var(δa_x)，但 KF 模型要求二階差分 Var(w_a) = Δt⁴·Var(ä_x)，原式 over-estimate ~3300×。同時評估 Route III (â_x 多項式形式) 後駁回（過 wall under-estimate 80%, bias loop 風險）。 |
| 2026-04-29 | (task03-step1) | Task 03 Step 3.1 ✓ DONE — 閉迴路 δx_r 統計嚴格推導（從 Eq.19 form ARMA inflation）：σ²_δxr = C_dpmr·4kBT·â_x + C_n·σ²_n_s（C_dpmr=3.96, C_n=1.18 for λ_c=0.7，與 paper 2025 Eq.11/12 同形式，非粗糙近似）；ρ_δxr(τ) 選 Option A (MA(2) full)，IF_var≈4.2；三決定敲定（ρ_δx Option A、σ²_δxr 用 â_x[k]、σ²_n_s 常數 per-axis）；Step 3.2 chi-squared 推 Var(σ̂²_δxr) 待續。同步補修 design.md §5 Row 3 註解明確 Eq.18 vs Eq.19 form 兩階段（之前只跳到 Eq.19 form 看似直接 Jacobian），fix §8.1 + agent_docs/eq17-architecture.md §5 caption "Path A′ inflation" → "Path C strict" 對齊 §8.2 嚴格形式；新增 design.md §9 R 矩陣推導章節（Step 3.1-3.5 進度與決定總覽），renumber §9-11 → §10-12。 |
| 2026-04-29 | (task03-step234) | Task 03 Step 3.2-3.4 ✓ DONE — Var(σ̂²_δxr) 從 IIR EWMA + Wick 推得 a_cov·IF_var·(σ²_δxr)²（用 δx_r ~ N(0,σ²) 高斯特例 ρ_{δx_r²}=ρ_δxr²；穩態 δ̄x_r≈0 簡化；small a_cov 極限 IF_eff≈IF_var 誤差 ~9%）。R_2_intrinsic,i[k] = a_cov·IF_var·{â_x,i+ξ_i}² 純二次多項式 (per-axis ξ_i = (C_n/C_dpmr)·σ²_n_s,i/(4kBT) sensor floor)。R_2_eff,i[k] = R_2_intrinsic + 5·Q77,i[k] 含延遲傳導。R 矩陣作為 scalar 函數 R(â_x,i[k], h̄[k])，每 step per-axis 算 2×2 matrix (off-diag=0)。design.md §9.3-9.8 全部 expand 完整推導 + 函數性質總表 + 合計精度 ~10-15%。下一步 Step 3.5 自適應 R_2 gate 設計。 |
| 2026-04-29 | (task03-step5) | Task 03 Step 3.5 ✓ DONE — 自適應 R_2 gate 採 trajectory-aware 3-guard 簡化版（拒絕原 §9.5 outline 的 4-gate + hysteresis + ramp 複雜設計）：軌跡的 hold(0.5s) → descend → oscillation 結構天然提供 IIR 收斂期；EKF 內只加 (1) `t < t_warmup_kf=0.2s` warm-up gate (2) `σ²_δxr ≤ C_n·σ²_n_s` NaN guard (3) `h̄ < h̄_safe=1.5` wall gate，純 OR 觸發 R_2=1e10，無 hysteresis、無 ramp-down。Guard 3 同時 cover 過 wall 與 a_x 動太快兩條件（兩者物理上同步）。觸發時 KF 退化單回授，由 y_1 鏈 + PE 結構維持 rank(O)=7（Task 01 結論支持）。Task 03 全部完成（Step 3.1-3.5 五階段）。同時整合既有 dual-track simulation design（agent_docs/dual-track-simulation-design.md，從 Downloads/ 移入），規劃 Task 04 走 dual-track（pure-MATLAB driver + Simulink SSOT + verify_equivalence gate），詳見 design.md §12 task 04。 |
| 2026-04-29 | (task04-step0) | Task 04 Step 0 ✓ DONE — pre-Phase 4a 環境準備：(1) **緩和軌跡**：run_simulation.m 改 h_bottom 2.5→4.5 μm（h̄_bottom 1.11→2.0 > h̄_safe=1.5），h_min 1.1·R→1.5·R，避開 gate-on/off 邊界複雜度，Task 04 初期 KF 全程 dual-feedback；(2) **§8.4 caveat**：design.md 加 leading-order vs full polynomial 差異說明 — 實作用 full poly（與既有 calc_correction_functions 一致），near wall (h̄<2) full poly K_h² 比 leading order 大 50-380×（lubrication 發散），緩和軌跡確保 Q77 不需 cap；(3) **Task 04 短期目標重定**：純 MATLAB only（不動 Simulink），4 phases (4a 擴充 K_h, K_h' / 4b build_eq17_constants / 4c motion_control_law_eq17_7state / 4g unit tests)，Phase 4d/4e/4f 後置；(4) **執行原則**：agent 只做文字操作不佔 MCP，主執行緒用 MCP 做靜態分析，動態驗證由使用者本地執行。 |
| 2026-04-29 | (task04-end2end) | Task 04 端到端驗證（Stage 1+2+3） — **全 23 unit tests PASS**（Phase 4a 6, 4b 7, 4c 6, Phase 0 4），**S1/S2/F3 端到端跑通**：S1 (positioning, no noise, 1s) f_d=8e-13≈0 / p_m=p_d 完美；S2 (positioning, full noise, 5s) tracking std 24 nm 三軸均勻 mean<1 nm；F3 (osc, h_bar∈[1.98, 22.25], 4.8s) tracking xy=23 nm z=76 nm。**3 bugs fixed (commit c807ad2)**：(A) motion_control_law_eq17_7state EKF update 改 sequential 1D（gate 觸發時 R=1e10 造成 joint S 矩陣 cond ~1e14 RCOND 警告，現改成 H_use=H(1,:) 跳過 y_2）；(B) run_pure_simulation 加 P.thermal.enable gate（calc_thermal_force 不看 enable flag，driver 必須在外面 gate）；(C) test_calc_correction_functions reference values 從 3 sig fig 升到 5-6 sig fig 精度。**a_hat 系統性偏低 29-44% — open issue（design.md §11 加註）**：診斷結果（1）design.md C_dpmr=3.96 公式 Lyapunov (Eq.18 form 直接) 驗證正確（Lyap=3.89, 差 2%）；（2）Pf_init 改 250× 對 bias 無影響（穩態 P 與 init 無關）；（3）observed σ²_δx ≈ 63% × design.md prediction；（4）KF 完美 track IIR 給的 a_xm；（5）self-consistent equilibrium：a_hat 變化改 effective feedback gain → 改 σ²_δx → 改 a_xm，迴圈穩在 0.6×a_true 平衡點；（6）動態軌跡 (osc) bias 較小 (29% z-axis) 因軌跡提供 excitation 幫識別。**結論**：tracking 完全可用（與既有 7-state 同等級 24 nm），但 a_hat 不能直接當 motion gain truth；Task 06 對照時注意。 |
| 2026-04-29 | (task04-bias-diag2) | Task 04 a_hat bias 深度診斷 — bias 比例**不是固定**：(1) **λ_c sweep**：λ=0.5 ratio=0.84, λ=0.7=0.61, λ=0.8=0.50, λ=0.9=0.39。觀察到的 σ²_δx 在 5-7e-4 範圍內被「夾住」(self-stabilizing)，因為 a_hat 自我調整改 effective feedback gain。(2) **Test 1 鎖 a_hat near a_nom**：σ²_δx ratio 仍 0.59-0.70 (T_sim=0.15 sec 太短可能未平衡，但顯示鎖 a_hat 不能完全恢復 design 預測)。(3) **F3 osc 動態軌跡**：a_hat **跟蹤 a_true 形狀** (Pearson corr=0.795 for z-axis)，但整體 scale 偏離。底部 a_hat_z=2.78e-3 vs a_true_z=6.82e-3 (ratio 0.41)。**最可能 root cause**：F_e Row 3 用 Eq.19 form (3,3)=λ_c 假設 a_hat=a_true；當 a_hat≠a_true（g=a_true/a_hat≠1），實際 closed-loop 是 δx[k+1] = δx[k] − g·(1−λ_c)·δx[k−d] − ε，與 EKF 預測不一致 → model mismatch 造成 KF 在錯誤平衡點。對比：既有 7-state (paper 2025 Eq.6) reference bias 僅 +2.46%，可能因 δx̂ feedback 提供 a_hat-a_true 抵消機制。**待驗證 hypothesis**：(E1) 改 F_e Row 3 為 Eq.18 form `[-(1-λ_c),0,1,-1,0,-f_d,0]` 是否消 bias；(E2) 對照既有 7-state 同軌跡，找結構性差異。 |
| 2026-04-29 | (task04-phase4a) | Task 04 Phase 4a ✓ DONE — calc_correction_functions.m 擴充 K_h, K_h' 解析導數 + 6 個 unit test。Backward-compat 保留（既有 2-output 1-input call 不破）。新增 derivs struct (nargout>=3 自動算)：dc_para_dh, dc_perp_dh, K_h_para, K_h_perp, K_h_prime_para, K_h_prime_perp。解析導數逐項從 D_para(u), D_perp(u) 微分，d(u^n)/dh_bar=-n·u^(n+1)。驗證點 h_bar=[1.10,1.50,22.0] reference values，FD 比對 rel.err<1e-5。checkcode 0 issues 兩檔。 |
| 2026-04-29 | (task04-phase4b) | Task 04 Phase 4b ✓ DONE — build_eq17_constants.m 離線常數 + 7 個 unit test。ctrl_const struct 含 lambda_c, C_dpmr (=2+1/(1-λ²)=3.96), C_n (=2/(1+λ)=1.18), option, IF_var (Option A: 4.224 / Option B: 2.922 for λ=0.7), xi_per_axis (3x1), delay_R2_factor (=5 for d=2), t_warmup_kf, h_bar_safe, d, a_cov, meta。Option A IF_var 用 ε MA(2) ARMA 閉式：rho_e_1, rho_e_2, Var(δx)/σ²_ε, rho_dx_1, rho_dx_2，幾何 tail summed for τ≥3。輸入驗證完整。checkcode 0 issues 兩檔。 |
| 2026-04-29 | (task04-phase4c) | Task 04 Phase 4c ✓ DONE — motion_control_law_eq17_7state.m 主控制器 (388 lines) + 6 個 unit test (240 lines)。Per-axis × 3 EKF：(1) Eq.17 控制律 f_d=(1/â_x)·{pd[k+1]-λ_c·pd[k]-(1-λ_c)·pd[k-d]+(1-λ_c)·δx_m-x̂_D}; (2) IIR a_xm via paper 2025 Eq.9-13 (dx_bar_m EWMA, sigma2_dxr_hat EWMA, linear inversion); (3) Q matrix 三項 active (Q33=4kBT·â_x, Q55=0, Q77=Δt^4·â_x²·{...K_h, K_h'...}); (4) R(1,1)=σ²_n_s, R(2,2)=R_2_intrinsic+5·Q77 with 3-guard (G1:t<0.2s, G2:σ²_δxr≤C_n·σ²_n_s, G3:h̄<1.5)→R_OFF=1e10; (5) F_e (7x7) 唯一時變 (3,6)=-f_d, H (2x7) (2,7)=-d=2; (6) EKF predict+update with Joseph-form symmetrization。Persistent state（x_e_per_axis 7x3, P_per_axis cell{3} of 7x7, IIR states, pd delay buffer, k_step）。Function signature: [f_d, ekf_out] = motion_control_law_eq17_7state(del_pd, pd, p_m, params, ctrl_const)，匹配既有 motion_control_law_7state convention 但加 ctrl_const 第五個引數。checkcode 0 issues 兩檔。**Pending 使用者本地跑 test_motion_control_law_eq17_7state 驗證**。 |
| 2026-04-29 | (task04-buffer-deeper) | **Task 04 sensor delay buffer 深度診斷 — bug fix 比預期更深**：先前 (527b89e) 已找到 buffer 長度應為 d+1=3 (而非 d=2)。本 session 進一步用 4-way 矩陣 (buffer × F_e form) 驗證 + a_hat freeze 對照得出：**(1) buffer=3 alone 不夠**：產線換 buffer=3 (任 F_e form: eq19 或 eq18) 都會 a_hat 早期飛走 (~iter 6 後 a_x sign flip 到負/正大數)，造成位置發散到 mm 量級或撞 wall。**(2) freeze a_hat=a_nom + buffer=3 完全穩**：std_x/y/z = 30-32 nm，與 Lyap d=2 prediction (30.4 nm) 高度吻合 ✓ → 確認 buffer=3 closed-loop dynamics 本身對的；發散源是 EKF a_hat update。**(3) Pf_init[6] sweep**：current 2.16e-3 (10·a_nom²) 過大 → 小 P_init (Pa_scale=0.0001 對應 P[6,6]_init=2.16e-7) 在 buffer=3 下 stable，T=2s std=30 nm ✓；但中間值 (Pa=0.001~0.01) 邊緣穩 (T=0.5s 看似穩，T=2s 發散) → 不可靠。**(4) F_e Row 3 form**：加了 `ctrl_const.F_e_form='eq18'` debug flag (預設 eq19 backward-compat)。實驗發現 form 切換對 buffer=3 穩定性影響小，主要瓶頸是 P_init/P_pred 太放縱 a_hat 更新。**結論**：buffer fix 必須與 EKF a_hat update 重新設計搭配；單獨修 buffer 會把先前看起來正常的 24 nm tracking 變成 mm 級失控。**Production 維持 buffer=2 不動**，等 EKF 重設計 (Pf_init 緊縮 / 暖機期 freeze a_hat / Q matrix retune) 後一起做。**Decision**：buffer fix 從 task #31 的 single-line 改成 future "EKF a_hat 重設計" task。已留 `ctrl_const.F_e_form` flag 在 controller 為 future debug 用 (default 'eq19' backward-compat)。**Diagnostic harness** (temp_diag_buffer.m, parameterized buffer × F_e_form × a_hat_freeze × Pa_scale) 已用完刪除，可從本 entry 描述快速重建。**Sensitivity caveat**：production buffer=2 在 cross-experiment session 出現 first-run 偶發 a_hat blow-up (subsequent runs 穩)，顯示 EKF 對 persistent state / RNG ordering 敏感；屬 EKF design fragility 一部分。 |
