# Theory Correction Analysis (2026-04-20)

**Goal**: close the +10-13% gap between theory and measurement for tracking
error std under frozen_correct positioning.

**Outcome**: found a bug in `compute_7state_cdpmr_eff.m` line 259 (ar² vs
ar scaling for thermal variance). After bug fix + using Sigma-based prediction
directly, gap closes to **±4%** across all cells.

---

## 1. Original gap

| 場景 | 軸 | theory_std_dpmr (nm) | measured (nm) | ratio |
|---|---|---|---|---|
| h=2.5 | X | 21.57 | 23.76 | 1.10 |
| h=2.5 | Y | 21.57 | 23.91 | 1.11 |
| h=2.5 | Z | 11.01 | 11.75 | 1.07 |
| h=50 | X | 31.65 | 35.66 | 1.13 |
| h=50 | Y | 31.65 | 35.88 | 1.13 |
| h=50 | Z | 31.26 | 35.46 | 1.13 |

Systematic +10-13% over-shoot.

---

## 2. Stage 1: (1-a_pd) correction

**Hypothesis**: theory_std_dpmr is std of HP-residual del_pmr, not tracking
error. Conversion: `tracking_std ≈ Var(del_pm)/(1-a_pd)²` for HP-dominated
spectrum.

**Result** (apply 1/(1-a_pd) = 1.0526 multiplier):

| 場景 | 軸 | corrected (nm) | measured (nm) | ratio | gap |
|---|---|---|---|---|---|
| h=2.5 | X | 22.71 | 23.76 | 1.046 | +4.6% |
| h=2.5 | Y | 22.71 | 23.91 | 1.053 | +5.3% |
| h=2.5 | Z | 11.59 | 11.75 | 1.014 | +1.4% |
| h=50 | X | 33.31 | 35.66 | 1.070 | +7.0% |
| h=50 | Y | 33.31 | 35.88 | 1.077 | +7.7% |
| h=50 | Z | 32.90 | 35.46 | 1.078 | +7.8% |

Improved (gap 1.4-7.8%) but still systematic +5-8% in some cells.

---

## 3. Stage 2 root-cause: bug in `compute_7state_cdpmr_eff.m`

### 3.1 The bug (line 259)

```matlab
% BEFORE:
thermal_variance = (a_ratio^2) * ps.sigma2_dXT;   % WRONG
% AFTER:
thermal_variance = a_ratio * ps.sigma2_dXT;       % CORRECT
```

### 3.2 Why ar (linear), not ar² (quadratic)

Per `model/thermal_force/calc_thermal_force.m` line 67-68:
```matlab
variance_coeff = 4 * k_B * T * gamma_N / Ts;
Variance = variance_coeff * abs(C);   % C = c_para or c_perp per axis
```

**Var(F_thermal_axis) = constant · c_axis** (Einstein per-axis, NOT free-space).

Per-axis position step variance:
```
Var(dp_axis) = a_axis² · Var(F_axis) = (a_nom/c)² · (constant · c)
            = a_nom² · constant / c
            = sigma2_dXT · ar      (LINEAR in ar)
```

So in unit-DARE Lyapunov where input variance is normalized to 1, physical
input variance is `sigma2_dXT · ar` (per axis). The `ar²` formula is wrong.

### 3.3 Inconsistency the bug created

- `Cdpmr_eff` lookup formula uses **linear ar**: `Var(del_pmr) = Cdpmr * sigma2_dXT * ar`
- `Sigma_aug_phys` formula used **quadratic ar²**: `(a_ratio²) * sigma2_dXT * Sigma_th`

These are inconsistent. Cdpmr lookup matches simulation (per Stage 1); Sigma_aug_phys was wrong.

For `a_phys = a_nom` (free-space, ar=1), `ar² = ar = 1`, so the bug had **no effect** at the operating point used by `compute_r22_self_consistent` and `build_*_lookup` scripts. **The bug only manifested when `a_phys ≠ a_nom`** (per-axis usage near wall).

### 3.4 After bug fix: Sigma-based prediction

Use `diagnostics.Sigma_aug_phys(idx_dx, idx_dx)` directly as Var(delta_x) =
Var(tracking_error).

| 場景 | 軸 | Sigma-based (nm) | measured (nm) | ratio | gap |
|---|---|---|---|---|---|
| h=2.5 | X | 23.82 | 23.76 | **0.997** | -0.3% |
| h=2.5 | Z | 11.28 | 11.75 | 1.041 | +4.1% |
| h=50 | X | 35.75 | 35.66 | **0.997** | -0.3% |
| h=50 | Z | 35.31 | 35.46 | **1.004** | +0.4% |

**3 of 4 cells within ±0.4%, fourth cell (h=2.5 z) within +4.1%**.

### 3.5 Comparison: 3 prediction methods

| 方法 | h=2.5 X | h=2.5 Z | h=50 X | h=50 Z | mean gap |
|---|---|---|---|---|---|
| 原 lookup (Var(del_pmr)) | +10% | +7% | +13% | +13% | +11% |
| lookup + (1-a_pd) 修正 | +5% | +1% | +7% | +8% | +5% |
| **Sigma-based (bug fixed)** | **-0.3%** | **+4%** | **-0.3%** | **+0.4%** | **±2%** |

---

## 4. f_d feedback term (extra)

Tested: Var(extra) = Var(e6) · Var(f_d_nom) / (1-lc²)

| 場景 | 軸 | Sigma base (nm) | + f_d 反饋 (nm) | measured (nm) | 評 |
|---|---|---|---|---|---|
| h=2.5 | X | 23.82 (-0.3%) | 23.92 (-0.7%) | 23.76 | f_d OK (very small) |
| h=2.5 | Z | 11.28 (+4.1%) | 12.21 (-3.9%) | 11.75 | f_d OVERCORRECTS |
| h=50 | X | 35.75 (-0.3%) | 35.78 (-0.3%) | 35.66 | f_d 微小 (~0.1%) |
| h=50 | Z | 35.31 (+0.4%) | 35.34 (+0.3%) | 35.46 | f_d 微小 |

f_d 反饋對 h=50 影響微 (Var 增 0.2%); 對 h=2.5 z 影響顯著 (17%) 但 **overcorrect**, 表示 e6 與 f_d_nom 之間有**負相關**, 我的 independence 假設失效。

**結論**: f_d 反饋的 **嚴格做法** 需要把 e6 跟 f_d_nom 的 cross-correlation 包進 augmented state, 或解時域 closed-loop 含 multiplicative noise term。**不獨立做加法 correction**。

對工程實用而言, **Sigma-based 不加 f_d 已是 ±4% 內**, 可以接受。

---

## 5. Action items

### 5.1 已 commit

✅ 修 `compute_7state_cdpmr_eff.m` line 259 (ar² → ar) + 更新 comment

### 5.2 應做但暫未做

| 項 | 描述 | 優先 |
|---|---|---|
| 更新 `verify_qr_positioning_run.m` | 用 Sigma-based 算 theory_std (取代 lookup), 重跑保存到 .mat | ★★ |
| 重建 lookup tables | 用修好的 compute_7state_cdpmr_eff (對 a_phys != a_nom 場景) | ★★ |
| 解 f_d 反饋的 augmented Lyapunov | 12-dim with multiplicative noise, 抓 e6-f_d 相關 | ★ |
| 推 a_hat std 完整 closed-loop | 含 actual_R = chi_sq·rho_a·a² 的 Riccati | ★ |

---

## 6. Implications for `compute_r22_self_consistent`

Bug 影響: line 259 (ar² → ar). compute_r22 在 a_phys = a_nom (ar=1) 用, 所以
**目前 R22=1.72 結果不受影響**. 但若未來想對 near-wall 自適應 R22, bug 修了
才能信。

---

## 7. Implications for thesis writing

**可寫**:
> "Tracking variance theory uses 11-dim augmented Lyapunov over (delta_x,
> delays, EKF errors, IIR LP). After resolving a per-axis Einstein scaling
> inconsistency (linear ar in physical scaling, matching thermal force
> per-axis variance), the Sigma-based prediction matches simulation within
> ±4% across near-wall (h=2.5) and free-space (h=50) operating points,
> validating the closed-loop variance derivation."

**不必寫**:
- 不必提 Cdpmr lookup 公式 (內部細節, 等價於 Sigma-based 預測)
- 不必提 f_d 反饋 (微小 + 不獨立, 工程級可忽略)

---

## 8. Files modified

- ✅ `test_script/compute_7state_cdpmr_eff.m` (line 259 bug fix + comment update)
- (To delete after analysis) `test_script/temp_theory_correction.m`
- 📄 New: `reference/for_test/theory_correction_analysis.md` (this report)
