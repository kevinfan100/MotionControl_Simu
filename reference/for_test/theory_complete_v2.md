# Comprehensive Theory Status v2 (2026-04-20)

完整推導 + 驗證的最終結果。涵蓋 4 個 Levels (Level 2 跳過, 反向走 wrong direction)。

---

## 執行摘要

| 量 | 預測精度 (Level 3 後) | 適用 |
|---|---|---|
| **Tracking std** | **±4%** (5/6 cells ±0.4%) | 全部 ✓ |
| **a_hat std (frozen z h=50)** | **+7%** (theory 1.57% vs 1.68%) | paper 主場景 ✓ |
| **a_hat std (frozen z h=2.5)** | 3× under | near-wall, partial |
| **a_hat std (frozen x)** | 9× under | per-axis 限制 |
| **a_hat std (empirical)** | 6× under (consistent) | empirical 模式 |

**3 commits**: `9dca456` (bug fix), `171aa1f` (Sigma-based tracking), `9ba1b02` (a_hat Lv0), `be744be` (Lv1 rho_a), `6164dd4` (Lv3 f_d).

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

### a_hat estimation variance per axis (PARTIAL accuracy)

```
Var(a_hat - a_true) ≈ Sigma_aug_phys(idx_e6, idx_e6) + chi_sq · rho_a · a²·Sigma_na(idx_e6, idx_e6)

where chi_sq = 2·a_cov / (2 - a_cov)
      rho_a  = compute_rho_a_rigorous(A_aug, Sigma_state, a_pd, a_cov, sigma2_n)
      Sigma_na = solve_dlyap(A_aug, B_na)
      B_na(idx_e) = -Fe · L(:, 2)  (gain channel innovation)
```

**Limitation**: 對 frozen z (free-space) 準 (±7%), 對其他 cells 給 order-of-magnitude only.

---

## Thesis 寫作清單

### CAN claim ✓
- "11-dim augmented Lyapunov predicts closed-loop tracking variance to within ±4% across near-wall and free-space operating points."
- "a_hat estimation std under frozen Q in free-space positioning matches Sigma-based prediction within +7% (1.57% theory vs 1.68% measured), validating chi-squared chain mechanism + closed-loop suppression."
- "Bug-free per-axis Einstein scaling (a_axis/a_nom linear) consistent with thermal_force model; quadratic scaling fixed in commit 9dca456."

### CANNOT claim ✗
- a_hat std for ALL operating points (theory under-predicts 3-9× in non-paper-main scenarios)
- Per-axis a_hat asymmetry (theory predicts symmetric, measurement shows x ≠ z by factor 8 for h=50 frozen)
- Empirical-mode a_hat std (theory under-predicts 6× consistently)

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
```

Skipped: Level 2 (wrong direction), Level 4 (Pf already steady state).

---

## 結論

**完整推導框架建立** (11-dim Lyapunov + rho_a + Sigma_na + Sigma_mult), **bug 修了**, **驗證 P6 12 runs 完成**。

**Tracking std**: 工程級準確 (±4%) → **可信任公式**
**a_hat std**: 部分場景準 (frozen z h=50 ±7%) → **可寫主場景, 其他列限制**

剩 6× under-prediction 的根因 (empirical 全部, frozen x 全部) **未解**, 可能源於 per-axis sub-optimal Kalman 結構 (R designer ≠ actual)。需更深入 closed-loop sub-optimal 理論才能解。
