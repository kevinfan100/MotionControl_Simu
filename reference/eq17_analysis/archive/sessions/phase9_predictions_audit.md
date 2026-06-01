# Phase 9 Stage 0 — R(2,2) Intrinsic Predictions Audit

**Date**: 2026-05-04
**Branch**: `test/eq17-5state-ekf`
**Plan**: `reference/eq17_analysis/phase9_R22_validation_plan.md` (commit 698949c)
**Stage 11 baseline**: commit 6add6da

---

## 1. Constants Table

| Symbol | Value | Source (file:line) | Units | Sanity |
|--------|-------|--------------------|-------|--------|
| `lambda_c` | 0.7 | `model/config/user_config.m:65` | — | per plan |
| `a_cov`    | 0.05 | `model/config/user_config.m:80` | — | per plan |
| `a_pd`     | 0.05 | `model/config/user_config.m:78` | — | per plan |
| `R` (radius) | 2.25 | `model/config/physical_constants.m:13` | µm | — |
| `gamma_N` | 0.0425 | `model/config/physical_constants.m:14` | pN·s/µm | — |
| `Ts` | 1/1600 = 6.25e-4 | `model/config/physical_constants.m:15` | s | — |
| `k_B` | 1.3806503e-5 | `model/config/physical_constants.m:16` | pN·µm/K | — |
| `T` | 310.15 (37 °C) | `model/config/physical_constants.m:17` | K | — |
| `4·k_B·T` | 1.712835e-2 | derived | pN·µm | Phase 6 used 1.646e-2 (T=298K); ratio 1.040 |
| `h_init` | 50 | per plan | µm | — |
| `h_bar = h/R` | 22.2222 | derived | — | — |
| `c_para(h_bar)` | 1.025959 | `calc_correction_functions.m:75` | — | ≈1 (far from wall) |
| `c_perp(h_bar)` | 1.053277 | `calc_correction_functions.m:81` | — | > c_para ✓ |
| `a_freespace = Ts/gamma_N` | 1.470588e-2 | derived | µm/pN | — |
| `a_true_per_axis` | [1.4334; 1.4334; 1.3962]·1e-2 | a_freespace ./ [c_para; c_para; c_perp] | µm/pN | — |
| `meas_noise_std` | [6.2e-4; 5.7e-5; 3.31e-3] | `test_script/run_simulation.m:63`, `run_v2_h50_e2e.m:109` | µm | lab spec (production) |
| `sigma2_n_per_axis` | [3.844e-7; 3.249e-9; 1.0956e-5] | derived | µm² | — |
| `IF_var` (Option A MA(2) full) | **4.219824** | `build_eq17_constants.m:208` | — | doc ref ≈4.224 ✓ |
| `IF_var` (Option B AR(1)) | 2.9216 | `build_eq17_constants.m:212` | — | doc ref ✓ |
| `C_dpmr_paper = 2 + 1/(1-λ²)` | 3.960784 | `build_eq17_constants.m:178` | — | ✓ |
| `C_n_paper    = 2/(1+λ)` | 1.176471 | `build_eq17_constants.m:181` | — | ✓ |
| `C_dpmr_eff_per_axis` (Lyapunov) | [3.1909; 3.1907; 3.1966] | `compute_7state_cdpmr_eff_v2.m:202`, called from `calc_ctrl_params.m:149` | — | ratio to paper 0.806 |
| `C_n_eff_per_axis`   (Lyapunov) | [0.9552; 0.9552; 0.9542] | `compute_7state_cdpmr_eff_v2.m:207` | — | ratio to paper 0.812 |
| `C_dpmr_closed_form` (production override) | 3.160954 | `calc_ctrl_params.m:175` | — | further -0.9% vs Lyapunov |

**Production override note**: `user_config.m:100` sets `cdpmr_method = 'closed_form'`, so `calc_ctrl_params.m:177` overwrites `C_dpmr_eff_per_axis` with `C_dpmr_closed = 3.1610 * ones(3,1)`. `C_n_eff_per_axis` is **always** the Lyapunov solution (per `calc_ctrl_params.m:166` comment "Leaves C_np_eff_per_axis untouched").

---

## 2. Dimensional Audit Verdict

**Question**: Is `C_dpmr_eff` (and `C_n_eff`) the same kind of dimensionless ratio as paper `C_dpmr` (resp. `C_n`)?

**Verdict**: **PASS — `C_eff` is the same quantity as `C_paper`.** Both are pure dimensionless ratios of the form:

- `C_dpmr` ≡ Var(δx_r) / (4·k_B·T·a) where `a` = single-axis mobility µm/pN
- `C_n`    ≡ Var(δx_r) / σ²_n where `σ²_n` = sensor noise variance µm²

**Evidence:**

1. **Paper closed-forms** (`build_eq17_constants.m:178, 181`):
   - `C_dpmr = 2 + 1/(1-λ²)` — pure function of λ_c, dimensionless.
   - `C_n = 2/(1+λ)` — pure function of λ_c, dimensionless.

2. **Lyapunov computations** (`compute_7state_cdpmr_eff_v2.m:80–207`):
   - DARE solved on `(F_e, H, Q_kf, R_kf)` with **unit-scaled `sigma2_dXT_unit = 1`** (line 85). Gain `L` is therefore a function of dimensionless `(Q_scale, R_scale)` only.
   - Thermal noise driver `B_th` has unit magnitude (`B_th(idx_dx) = -1`, line 167). The Lyapunov solve `Sigma_th = solve_dlyap_robust(A_aug, B_th)` returns Var per **unit thermal forcing**.
   - Output `C_dpmr_eff = gain_sq * var_state_th` (line 202) is therefore exactly Var(δx_r) per unit thermal driver = Var(δx_r) / (σ²_thermal_unit) = Var(δx_r) / (4·k_B·T·a) with a built into the simulation context. **Same slot, same dimensions as paper `C_dpmr`.**
   - Same logic for `C_n_eff` with unit sensor-noise driver `B_np` (line 173).

3. **Same formula slot in R(2,2)**: both paper and eff are used identically as the multiplier of `(a + ξ)²` (eff in calc_ctrl_params local design, paper in motion_control_law_eq17_7state.m:530). They occupy the same algebraic position with the same units.

**Conclusion**: V6 (paper-vs-eff comparison) is **dimensionally well-defined**. Proceed.

---

## 3. Per-Axis ξ: Paper vs Effective

| Axis | ξ_paper [µm/pN] | ξ_eff [µm/pN] | ratio eff/paper |
|------|-----------------|---------------|-----------------|
| x | 6.666e-6 | 6.718e-6 | 1.0078 |
| y | 5.634e-8 | 5.679e-8 | 1.0079 |
| z | 1.900e-4 | 1.909e-4 | 1.0050 |

**Comparison vs Phase 6 §4.2 reference** (computed at T=298 K with 4·k_B·T = 1.646e-2):

| Axis | Phase 6 reference | This audit (T=310.15K, T-rescaled to 298K) | Match |
|------|-------------------|---------------------------------------------|-------|
| x | 6.93e-6 | 6.666e-6 · (1.713/1.646) = 6.937e-6 | ✓ exact |
| y | 5.86e-8 | 5.634e-8 · 1.0407 = 5.864e-8 | ✓ exact |
| z | 1.98e-4 | 1.900e-4 · 1.0407 = 1.978e-4 | ✓ exact |

(Phase 6 used T=298K convention; this audit uses production T=310.15K. After temperature rescale, all match Phase 6 numbers exactly.)

---

## 4. V6 Discriminating Power Assessment — **FLAG**

The eff/paper ξ ratio is **1.005–1.008 across all three axes** — i.e. ξ shifts by **< 1%** when switching from paper `C_n/C_dpmr = 0.297` to per-axis effective values (Lyapunov ≈ 0.299, closed-form ≈ 0.302).

Implication for V6: `R22_pred_eff` and `R22_pred_paper` differ by **< 1% per axis at h=50µm**:

| Axis | R22_pred_paper | R22_pred_eff | rel diff |
|------|----------------|--------------|----------|
| x | 4.339e-5 | 4.339e-5 | +7e-6 (negligible) |
| y | 4.335e-5 | 4.335e-5 | <1e-7 |
| z | 4.226e-5 | 4.226e-5 | +1.4e-4 (negligible) |

Interpretation: at this operating point, the `(a + ξ)` sum is dominated by `a ≈ 1.4e-2` and ξ contributes only ~0.05% (x), ~4e-6 (y), ~1.4% (z). **A few-percent shift in ξ produces only ~0.001–0.03% shift in R(2,2)**.

**Therefore the empirical V6 test will not distinguish paper vs eff at h=50µm with these noise stds**. The two predictions are functionally identical and any empirical R(2,2) signal will agree with both within experimental uncertainty.

**Recommendation**: Stage 0 should still record both predictions for the audit trail, but downstream V6 should either:
- (a) move to a regime where ξ dominates (z-axis with much larger σ²_n_z, or a closer wall where a shrinks), or
- (b) re-scope the validation to focus on the **a_cov sweep (V2)** and **σ²_n sweep (V3)** which both have clean order-of-magnitude signal.

---

## 5. Production `meas_noise_std` Resolution

| Source | Value [µm] | Notes |
|--------|------------|-------|
| `model/config/user_config.m:68` baseline | `[0.01; 0.01; 0.01]` | default fallback only |
| `test_script/run_simulation.m:63` | `[0.00062; 0.000057; 0.00331]` | **production main** |
| `test_script/run_v2_h50_e2e.m:109` | `[0.00062; 0.000057; 0.00331]` | h=50 e2e driver |
| `test_script/verify_7state_free_space.m:103` | `[0; 0; 0]` | free-space verification only |
| Phase 6 §3.2 spec | `[0.62; 0.057; 3.31] nm` = `[6.2e-4; 5.7e-5; 3.31e-3] µm` | matches production scripts |

**Production-current**: `meas_noise_std = [6.2e-4; 5.7e-5; 3.31e-3] µm` (lab spec). Used in this audit.

**Phase 6 §8.1 σ_n_y suspicion note**: The `5.7e-5 µm = 0.057 nm` y-axis std is ~10× smaller than expected and was flagged for verification. `phase8_settings_audit.md:200` documents this as "confirmed real, not a typo — project-wide spec for the lab sensor's tangential-y axis".

---

## 6. Outputs Saved

`reference/eq17_analysis/phase9_predictions.mat` — fields:
- Scalars: `lambda_c, a_cov, a_pd, h_init, R, gamma_N, Ts, k_B, T, kBT, h_bar, c_para, c_perp, a_freespace, IF_var, C_dpmr_paper, C_n_paper, C_dpmr_closed, rho_dx_1, rho_dx_2, rho_e_1, rho_e_2, Var_dx_over_sig_e`
- 3×1 vectors: `a_true_per_axis, meas_noise_std_per_axis, sigma2_n_per_axis, C_dpmr_eff_per_axis, C_n_eff_per_axis, xi_paper_per_axis, xi_eff_per_axis, R22_pred_paper_per_axis, R22_pred_eff_per_axis`
- Sweeps: `a_cov_sweep [5x1]`, `V2_R22_pred_paper [5x3]`, `V2_R22_pred_eff [5x3]`; `sigma2_n_factor_sweep [4x1]`, `V3_R22_pred_paper [4x3]`, `V3_R22_pred_eff [4x3]`
- `V4_ACF_predicted_optionA [51x1]` — δx autocorrelation, lags 0–50, Option A MA(2) full closed-form. Lag 0 = 1, lag 1 = 0.8515, lag 2 = 0.6718; tail geometric with ratio λ_c=0.7.

ACF self-consistency check: `1 + 2·Σ_{τ≥1} ρ²(τ)` with infinite tail = 4.2198 (matches IF_var exactly).

---

## 7. Stop Conditions

None triggered. Dimensional audit PASS. Predictions saved.

**Soft flag**: V6 paper-vs-eff comparison has < 1% discriminating power at h=50µm with current noise spec. Discuss with user before running V6 simulations; consider operating-point change or alternative scope.
