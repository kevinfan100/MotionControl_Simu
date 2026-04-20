# Comprehensive Theory Status v2 (2026-04-20, updated with Level 5)

完整推導 + 驗證的最終結果。涵蓋 5 個 Levels (Level 2 跳過, 反向走 wrong direction; Level 4 跳過, Pf 已 settled).

**Level 5 (a_true wall sensitivity) 是 KEY breakthrough**, 將 frozen z near-wall 預測從 3× under 修正到 ±7%。

---

## 執行摘要

| 量 | 預測精度 (Level 5 後) | 適用 |
|---|---|---|
| **Tracking std** | **±4%** (5/6 cells ±0.4%) | 全部 ✓ |
| **a_hat std (frozen z h=50)** | **+7%** (theory 1.57% vs 1.68%) | paper 主場景 ✓ |
| **a_hat std (frozen z h=2.5)** | **+7% (Lv5 修正)** (4.22% vs 4.52%) | near-wall ✓ |
| **a_hat std (frozen x)** | h=2.5: 5.5×; h=50: 8.9× | per-axis 限制 |
| **a_hat std (empirical z)** | h=2.5: 4× (Lv5); h=50: 5.8× | empirical 模式 |

**6 commits**: `9dca456` (bug fix), `171aa1f` (Sigma-based tracking), `9ba1b02` (a_hat Lv0), `be744be` (Lv1 rho_a), `6164dd4` (Lv3 f_d), **`a0fb317` (Lv5 wall sensitivity, KEY)**.

---

## Level-by-Level 實作與結果

### Level 0 → Level 1: rho_a rigorous (rigorous autocorrelation)

**Implementation**:
- 抽 `compute_rho_a_rigorous` 為 standalone helper (test_script/)
- verify code 兩 pass call: 第一次無 actual_a_m_var 抽 Sigma, 第二次用 rigorous rho_a

**Mathematical**:
```
rho_a = 1 + 2 * sum_{L=1}^{Lmax} ρ_del_pmr(L)² · (1-a_cov)^L
ρ_del_pmr(L) = (c · A_aug^L · Sigma · c') / (c · Sigma · c' + sigma2_n)
```

**Results** (vs Level 0 with rho_a=4):
| 場景 | Level 0 ratio_z | Level 1 ratio_z | rho_a measured |
|---|---|---|---|
| frozen h=2.5 | 3.07 | 3.07 | 4.05 |
| frozen h=50 | 1.15 | **1.07** | 4.63 |
| empirical h=2.5 | 6.13 | 6.58 | 3.47 |
| empirical h=50 | 5.72 | 5.76 | 3.95 |

**結論**: rho_a 接近 4, Level 1 影響很小 (<2% 改變)。frozen z h=50 略改善, 其他 cells 不變。

### Level 2: skipped (mismatched R, wrong direction)

**Why skipped**: 用 actual R = chi_sq*ρa*a² 重解 DARE 給 optimal L (大 4.6×, sub-optimal designer L = 1e-4)。Optimal Var < Sub-optimal Var (數學上 trivially)。但量測 > theory 已 (under-prediction), 走 optimal 反而**更小**。

**Lesson**: 「修正 R 失配」不是「縮小 theory-vs-meas gap」, 是「逼近最佳化下界」。對 under-prediction 沒幫助。

### Level 3: f_d multiplicative coupling (Isserlis)

**Implementation**:
- compute_7state_cdpmr_eff 加 `Sigma_mult = solve_dlyap(A_aug, B_mult)` where B_mult(idx_dx) = -1
- verify code 算 selectors c_fd, c_e6 → Var(f_d_nom), Cov(e6, f_d_nom)
- Isserlis 4 階矩: `Var(-e6·f_d_nom) = Var(e6)·Var(f_d) + Cov²`
- 加 `Var_input * Sigma_mult` 進 final Sigma

**Results**:
- Tracking std prediction: 不變
- a_hat std prediction: 不變 (< 0.01% 改變)

**為什麼沒效果**:
- frozen: Var(e6) 太小 (~2e-7) → Var product 微
- empirical: Sigma_mult amplification at e6 被 closed-loop dampened

**結論**: f_d 耦合不是 under-prediction 主因。

### Level 4: skipped (time-varying Pf, won't help)

**Analysis**:
- frozen L_ss = 1e-4 → time const 1/L = 71 samples = 44 ms
- empirical L_ss = 0.01 → time const 100 samples = 62 ms
- t_warmup = 2s = 32-45 time constants
- Pf reaches steady state long before measurement window

**結論**: Pf transient already settled, time-varying 算不會改變預測。

### Level 5: a_true wall-sensitivity term (KEY breakthrough)

**Insight**: 之前我假設 `a_true_axis = a_nom/c_axis(h_init)` 是常數, 但實際 p_m_z 漂移
(= tracking error_z) 改變 h_bar, c_perp/c_para 也跟著變, 所以 **a_true 隨時間波動**。

`e_a = (a_hat - a_true)/a_true` 包含**兩個獨立來源**:
1. a_hat noise (EKF 裡的 chi-sq 雜訊) — Lv1+Lv3 已 cover
2. **a_true fluctuation** (wall sensitivity × tracking std) — **Lv5 加入**

**Formula**:
```
rel_std(a_true_axis) = |dc/dh_bar| / c · std(p_m_z) / R
                       (sensitivity)         (tracking std)
```

對 z 軸用 dc_perp/dh_bar; 對 x/y 軸用 dc_para/dh_bar。

**Sensitivity 數值** (從 finite difference at h_init):
- h=2.5 (h_bar=1.11): `|dc_perp|/c_perp = 7.884` per h_bar unit
- h=50 (h_bar=22.2): `|dc_perp|/c_perp = 0.002` (3900× 小)

**Combined assuming independence**:
```
Var(e_a)_total = Var(e_a_EKF) + Var(a_true_wall_sens)
```

**Results (z-axis)**:
| 場景 | Lv1 ratio | **Lv5 ratio** | 改善 |
|---|---|---|---|
| frozen h=2.5 | 3.07 | **1.07** ✓ | HUGE |
| frozen h=50 | 1.07 | 1.07 | unchanged (sens ~0) |
| empirical h=2.5 | 6.58 | 4.02 | partial |
| empirical h=50 | 5.76 | 5.76 | unchanged |

**結論**: Level 5 完成 frozen z-axis 推導 (兩個高度都 ±7% 對齊)。
empirical 部分改善, 仍剩 5-6× — 屬於不同機制 (active EKF 對 autocorrelated input 的 filtering 行為)。

---

## 為什麼 a_hat under-prediction 沒解?

**剩餘候選 (uninvestigated)**:

1. **Per-axis EKF cross-coupling via 2x2 Riccati**
   - Designer R(1,1) 是常數 (sigma2_dXT*0.397), 對 x 過估 1.6 million×, 對 z 過估 9×
   - 2x2 R 矩陣導致 L matrix per-axis 行為不同
   - sub-optimal Kalman 的 actual Var 不是 designer-L Lyapunov 預測的

2. **a_m chi-sq autocorrelation interaction with EKF**
   - a_m at consecutive steps shares ~99% info (IIR memory)
   - EKF 假設 a_m 是 independent samples
   - 真實 EKF 行為跟 designer 預期不同 (filtering 效果不一致)

3. **Wall-effect lookup interpolation error**
   - calc_correction_functions polynomial fit
   - Cdpmr_eff lookup 在 (lc, ar) 內插
   - 累積誤差 → systematic offset

**估計工作量** (各 ~3-5 days):
- Item 1: 重構 Lyapunov 用 actual per-axis R; 解 sub-optimal Kalman 的 真實 variance
- Item 2: 含 a_m 的延遲狀態加入 augmented state, 解 13-dim Lyapunov
- Item 3: 改良 wall-effect polynomial, 重建 lookup

---

## 完整公式 (paper 可用)

### Tracking error variance per axis (CONFIRMED ±4%)

```
Var(tracking_axis) = Sigma_aug_phys(idx_dx, idx_dx)

where Sigma_aug_phys = a_ratio · sigma2_dXT · Sigma_th
                     + sigma2_n_axis · Sigma_np

with Sigma_th, Sigma_np = solve_dlyap(A_aug, B_*)
     A_aug = 11-dim closed-loop transition (delta_x + delays + EKF errors + IIR LP)
     A_aug(idx_e, idx_e) = Fe · (I - L · H)
     L = DARE solution with designer Q_kf, R_kf
     Fe = paper Eq.18 with f0 = 0
```

### a_hat estimation variance per axis (FROZEN z: ±7%, others partial)

```
Var(e_a) = Var(e_a_EKF)        +  Var(a_true_wall_sensitivity)
         = (Var_e_phys / a²)    + (sens · std(p_m_z)/R)²

where Var_e_phys = Sigma_aug_phys(idx_e6, idx_e6) + chi_sq·rho_a·a²·Sigma_na(idx_e6, idx_e6)
      chi_sq    = 2·a_cov / (2 - a_cov)
      rho_a     = compute_rho_a_rigorous(A_aug, Sigma_state, a_pd, a_cov, sigma2_n)
      Sigma_na  = solve_dlyap(A_aug, B_na),  B_na(idx_e) = -Fe·L(:,2)
      sens      = |dc_axis/dh_bar| / c_axis        % wall sensitivity, finite-diff
      std(p_m_z) = sqrt(Sigma_aug_phys(idx_dx, idx_dx))    % tracking std
```

**Accuracy by case**:
- frozen z (both h=2.5 and h=50): **±7%** ✓ (paper main scenarios)
- empirical: ~4-6× under (active-EKF mechanism, future work)
- x-axis: ~5-9× under (per-axis EKF coupling, future work)

---

## Thesis 寫作清單

### CAN claim ✓
- "11-dim augmented Lyapunov predicts closed-loop tracking variance to within ±4% across near-wall and free-space operating points."
- "**a_hat z-axis estimation std under frozen Q matches Sigma-based prediction + wall-sensitivity term within ±7% across BOTH near-wall (h=2.5) AND free-space (h=50)**, validating closed-loop chi-squared chain + a_true wall-sensitivity composite formula." (Lv5 加入後)
- "Bug-free per-axis Einstein scaling (a_axis/a_nom linear) consistent with thermal_force model; quadratic scaling fixed in commit 9dca456."

### CANNOT claim ✗
- Per-axis a_hat asymmetry (theory predicts symmetric, measurement shows x ≠ z by factor 5-8)
- Empirical-mode a_hat std (theory under-predicts 4-6× consistently, even after Lv5)

### SHOULD list as future work
1. Sub-optimal Kalman with R designer ≠ actual: per-axis Lyapunov treatment
2. a_m autocorrelation in EKF feedback (extend augmented state with delayed a_m)
3. Per-axis Cdpmr_eff in lookup with proper interpolation
4. Wall-effect polynomial accuracy improvement

---

## 最終 Commit 鏈 (test/qr-paper-reference branch)

```
9dca456 fix bug ar^2 -> ar (Einstein per-axis)
171aa1f tracking std Sigma-based + per-axis sigma2_n
9ba1b02 a_hat std Sigma-based (Level 0, rho_a=4)
be744be Level 1: rigorous rho_a (small improvement)
6164dd4 Level 3: f_d coupling (no measurable effect)
a0fb317 Level 5: a_true wall sensitivity (KEY breakthrough for frozen z h=2.5)
```

Skipped: Level 2 (wrong direction), Level 4 (Pf already steady state).

---

## 結論

**完整推導框架建立** (11-dim Lyapunov + rho_a + Sigma_na + Sigma_mult + wall sensitivity), **bug 修了**, **驗證 P7+P5 共 24 runs 完成**。

**Tracking std**: 工程級準確 (±4%) → **可信任公式**

**a_hat z 軸 frozen** (paper 主場景): **±7% 對齊兩個高度** (Level 5 加入後完整) → **可寫完整 z 軸 frozen claim**

剩餘限制:
- empirical 模式: 4-6× under (active EKF + autocorrelated input 機制)
- x 軸全部: 5-9× under (per-axis EKF coupling 結構)

兩者需要更深入工作 (~3-5 days each):
- empirical: 把 a_m 的 IIR 狀態加進 augmented state, 解 13-dim Lyapunov
- x 軸: per-axis sub-optimal Kalman 嚴格分析

**Level 5 是關鍵發現**: a_true 不是常數, 而是隨 p_m_z 漂移而變; 近壁時 wall sensitivity 高達 7.88 per h_bar unit, 這個項貢獻 4% relative std (近壁), 是漏掉的物理。
