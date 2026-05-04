# Phase 9: R(2,2) Intrinsic Formula Validation Plan

**Pre-Phase-9 commit**: `0d9cd37` (Phase 8 Wave 1 complete — h=50 results v4)
**Date**: 2026-05-04
**Status**: Validation plan. Awaiting user approval before script implementation.
**Branch**: `claude/cdpmr-eff-calculation-PnWIq`

This phase validates the closed-form intrinsic formula for R(2,2) — the EKF measurement noise variance on the `a_xm` channel. The plan isolates Layer 1 of the three-layer R(2,2) structure and provides a clean empirical test under positioning conditions.

---

## 0. Executive Summary

**Goal**: Empirically validate the closed-form intrinsic R(2,2) formula
```
R(2,2)_intrinsic = a_cov · IF_var · (a + ξ)²
```
under controlled positioning conditions where Layer 2 (delay correction) and Layer 3 (3-guard) are inactive.

**Approach**: Run positioning simulation with `Q_77 = 0` and `a_hat` frozen, sample `a_xm[k]` in steady state, compare empirical Var(a_xm) against predicted R(2,2)_intrinsic.

**Pass criterion**: Empirical / predicted ratio ∈ [0.95, 1.05] across all three axes, plus monotonic linear scaling under `a_cov` sweep.

**Deliverable**: Validation report (this doc) → script (Phase 9 Wave 1) → numerical results (Phase 9 Wave 2).

---

## 1. Background

### 1.1 R(2,2) three-layer structure (from Phase 6)

Reference: `model/controller/motion_control_law_eq17_7state.m:500-518`, `reference/eq17_analysis/phase6_R_matrix_derivation.md`

```
R(2,2) = (G1 ∨ G2 ∨ G3) ? R_OFF : (R_intrinsic + delay_R2_factor · Q_77)

  Layer 1 (Intrinsic):  R_intrinsic = a_cov · IF_var · (a_hat + ξ)²
  Layer 2 (Delay):      + delay_R2_factor · Q_77         (Phase 6 §4.3)
  Layer 3 (3-guard):    G1 = warmup, G2 = low SNR, G3 = near-wall
```

The intrinsic formula is the foundation; layers 2 and 3 are corrections / safety overrides. Validating Layer 1 in isolation is necessary before trusting the composite formula.

### 1.2 Why validate now

- Stage 11 Option I introduced per-axis effective `C_dpmr_eff / C_np_eff` (commit `6add6da`)
- The R(2,2) formula uses paper closed-form `C_n / C_dpmr` (3.96, 1.18) for `ξ` computation, not the per-axis effective values
- This inconsistency may produce 5-10% R(2,2) bias, which could explain residual `a_hat` bias seen in Wave 4 v3 (commit `4e2d7de`)
- Empirical validation will quantify the actual mismatch and inform whether Layer 1 needs revision

### 1.3 Relationship to Stage 11 calibration

```
Stage 11 (commit 417edc):  C_dpmr_eff_per_axis  →  calibrates E[a_xm]      (DC bias)
Phase 6 (commit 8b468aa):  R(2,2) = a_cov·IF_var·(a+ξ)²  →  Var(a_xm)     (AC variance)
                                                              (Layer 1 here)
```

These two are independent and complementary. Stage 11 rectifies the mean; Phase 9 validates the variance prediction.

---

## 2. Validation Goals

### Primary

V1. **Variance check**: empirical Var(a_xm) ≈ predicted R(2,2)_intrinsic across x, y, z.

### Secondary (factor decomposition)

V2. **a_cov scaling**: linear relation Var(a_xm) ∝ a_cov, with slope = IF_var · (a+ξ)².
V3. **σ²_n scaling**: Var(a_xm) follows (a + ξ(σ²_n))² as σ²_n is varied.
V4. **IF_var assumption**: empirical autocorrelation of dx_r matches λ_c^|m| approximation (validates IF_var = (1+λ_c²)/(1-λ_c²) closed form).

### Tertiary (consistency checks)

V5. **DC unbiased**: empirical Mean(a_xm) ≈ a_true (cross-check Stage 11).
V6. **ξ definition**: compare predictions using paper `C_n/C_dpmr` (current) vs effective `C_n_eff/C_dpmr_eff` for ξ.

---

## 3. Experimental Setup

### 3.1 Physical scenario: positioning

```
config.amplitude     = 0          # No trajectory oscillation → ḣ = 0
config.frequency     = 0          # → ḧ = 0
config.h_init        = 50         # Far from wall → G3 (near-wall) inactive
config.simulation_time = 30       # 30 sec (≈ 48000 steps at 1600 Hz)
```

Rationale:
- `amplitude = 0` → trajectory is stationary → ä_x = 0 in truth → naturally consistent with Q_77 = 0
- `h_init = 50 µm` → h_bar = 20, well above any reasonable `h_bar_safe` threshold

### 3.2 Disable Q_77 (and its floor)

```
config.sigma2_w_fA   = 0          # Disable Q_77 floor (Phase 5 §5.5)
config.sigma2_w_fD   = 0          # (already 0 in baseline; keeps Q_55 = 0)
```

This forces `Q_77 = 0` per axis (since `h_dot_max = 0`, `h_ddot_max = 0`, and floor is 0).
Consequence: `delay_R2_factor · Q_77 = 0` → Layer 2 contribution exactly zero.

### 3.3 Freeze a_hat (avoid closed-loop coupling)

```
ctrl_const.a_hat_freeze = a_true_per_axis    # 3x1 wall-aware ground truth
```

Reference: `motion_control_law_eq17_7state.m:541-544`

Rationale: with `a_hat` frozen, R(2,2) computation uses the true `a_x` (not a noisy estimate), eliminating the `a_hat → R(2,2) → a_hat` self-reference loop. This isolates the formula from EKF estimation transients.

### 3.4 Warmup and sampling window

```
config.t_warmup_kf   = 1.0 sec    # Standard EKF warmup
N_skip               = ceil(t_warmup_kf / Ts) + 1.0 sec buffer  → ≈ 3200 steps
N_sample             = 25 sec × 1600 Hz = 40000 steps
```

Rationale: IIR EWMA time constant ≈ 1/a_cov ≈ 200 steps for a_cov = 0.005. Skipping 3200 steps ensures > 15 time constants of EWMA settling, giving < 1% transient bias on σ̂² mean.

### 3.5 Measurement noise level

Use baseline per-axis `sigma2_n_s` from sensor spec (`σ_n_s = 0.62 nm` for x, etc.). For V3 (σ²_n sweep), scan `[0.5x, 1x, 2x, 4x]` baseline.

---

## 4. Why This Setup Is "Clean"

### 4.1 Layer 2 elimination

```
Q_77_setup = T_s⁴ · a² · {(K_h²−K_h')² · ḣ_max⁴/(8R⁴) + K_h² · ḧ_max²/(2R²)}
           = T_s⁴ · a² · {0 + 0}    (because ḣ_max = ḧ_max = 0)
           = 0
sigma2_w_fA floor = 0
→ Q_77 = 0 → delay_R2_factor · Q_77 = 0
```

Layer 2 contribution is **identically zero**, not just small. R(2,2)_eff = R(2,2)_intrinsic exactly.

### 4.2 Layer 3 elimination

| Guard | Trigger | Why inactive in this setup |
|---|---|---|
| G1 (warmup) | `t < t_warmup_kf` | Sampling skips first 3200 steps > t_warmup_kf · 1600 |
| G2 (low SNR) | `σ̂² ≤ C_np_eff · σ²_n` | Thermal driver provides nominal SNR; positioning noise floor maintained |
| G3 (near-wall) | `h_bar < h_bar_safe` | h = 50 µm → h_bar = 20 ≫ h_bar_safe (typically ~3) |

All three guards are **structurally inactive** under this setup → R(2,2) = R(2,2)_intrinsic always.

### 4.3 Closed-loop decoupling

Without `a_hat_freeze`:
```
σ̂²[k] (random) → a_xm[k] → KF update → a_hat[k] (random)
                                              ↓
              R(2,2)[k] = a_cov · IF_var · (a_hat[k] + ξ)²  ← noisy
                                              ↓
                            K_a recomputed each step  ← time-varying
```
With `a_hat_freeze`:
```
R(2,2) = a_cov · IF_var · (a_true + ξ)²    ← constant, deterministic
```
The frozen `a_hat` removes 3-axis self-coupling and makes R(2,2) a single fixed value per axis, simplifying both prediction and empirical comparison.

---

## 5. Validation Procedure

### Step 1: Configure scenario

```matlab
% In test_script/run_R22_validation.m (Phase 9 Wave 1 deliverable)
config = baseline_config();
config.amplitude       = 0;
config.frequency       = 0;
config.h_init          = 50;
config.sigma2_w_fA     = 0;
config.sigma2_w_fD     = 0;
config.simulation_time = 30;
config.t_warmup_kf     = 1.0;
```

### Step 2: Compute a_true per axis and freeze

```matlab
a_freespace = config.Ts / constants.gamma_N;
h_bar = config.h_init / constants.R;
[c_para, c_perp] = calc_correction_functions(h_bar);
a_true = [a_freespace/c_para; a_freespace/c_para; a_freespace/c_perp];

ctrl_const.a_hat_freeze = a_true;
```

### Step 3: Run simulation

```matlab
sim_result = run_pure_simulation(config, ctrl_const);
% Extract a_xm log per axis from sim_result.diag.a_xm_log (3 × N_total)
```

### Step 4: Trim warmup and compute statistics

```matlab
N_skip = round(2.0 / config.Ts);   % 2 sec safety margin > t_warmup_kf
a_xm_steady = sim_result.a_xm_log(:, N_skip:end);   % 3 × N_sample

emp_mean = mean(a_xm_steady, 2);                    % 3×1
emp_var  = var(a_xm_steady, 0, 2);                  % 3×1
```

### Step 5: Compute predictions and compare

```matlab
% Paper closed-form ξ (current implementation)
xi_paper = (C_n_paper / C_dpmr_paper) * config.meas_noise_std.^2 / (4 * kBT);
IF_var = (1 + lambda_c^2) / (1 - lambda_c^2);
R22_pred_paper = config.a_cov * IF_var * (a_true + xi_paper).^2;

% Effective ξ (consistency probe — V6)
xi_eff = (C_n_eff_per_axis ./ C_dpmr_eff_per_axis) .* config.meas_noise_std.^2 / (4 * kBT);
R22_pred_eff = config.a_cov * IF_var * (a_true + xi_eff).^2;

% Print
fprintf('Axis | a_true   | emp_mean | emp_var    | R22_paper  | R22_eff    | ratio_paper | ratio_eff\n');
for ax = 1:3
    fprintf('  %s  | %.3e | %.3e | %.3e | %.3e | %.3e | %.3f       | %.3f\n', ...
        'XYZ'(ax), a_true(ax), emp_mean(ax), emp_var(ax), ...
        R22_pred_paper(ax), R22_pred_eff(ax), ...
        emp_var(ax)/R22_pred_paper(ax), emp_var(ax)/R22_pred_eff(ax));
end
```

---

## 6. Validation Metrics

### 6.1 V1 — Variance ratio (core)

| Quantity | Formula | Target |
|---|---|---|
| ratio_paper | emp_var / R22_pred_paper | 0.95 ≤ ratio ≤ 1.05 |
| ratio_eff   | emp_var / R22_pred_eff   | 0.95 ≤ ratio ≤ 1.05 |

If `ratio_paper` deviates but `ratio_eff` matches → **ξ should use effective C_n_eff/C_dpmr_eff** (Phase 9 outcome: file Issue / fix).

### 6.2 V5 — DC bias check (Stage 11 cross-validation)

| Quantity | Formula | Target |
|---|---|---|
| bias_pct | (emp_mean − a_true) / a_true · 100% | within ±2% |

If bias > 2% → Stage 11 calibration may have residual error.

### 6.3 V2 — a_cov sweep

Run for `a_cov ∈ {0.001, 0.0025, 0.005, 0.01, 0.02}` (5 points), each with same physical setup:

```
Expectation:   emp_var ≈ a_cov · [IF_var · (a + ξ)²]   (linear through origin)
Slope check:   linear fit slope ≈ IF_var · (a + ξ)²    within ±5%
R² check:      linear fit R² > 0.99
```

### 6.4 V3 — σ²_n sweep

Run for `σ²_n_factor ∈ {0.5, 1, 2, 4}` × baseline:

```
Expectation:  emp_var(σ²_n) follows quadratic in (a + ξ(σ²_n))
              where ξ ∝ σ²_n
Plot:         log(emp_var) vs log(a + ξ)  → slope ≈ 2
```

### 6.5 V4 — dx_r autocorrelation

Extract `dx_r[k]` log from steady-state segment, compute sample ACF:

```matlab
acf = autocorr(dx_r_log_per_axis, 50);      % lags 0..50
acf_predicted = lambda_c .^ (0:50);         % paper assumption

plot(0:50, acf, 0:50, acf_predicted);
% Compute integrated autocorrelation:
IF_var_emp = 1 + 2 * sum(acf(2:end).^2);
fprintf('IF_var: closed-form = %.3f, empirical = %.3f, ratio = %.3f\n', ...
        IF_var, IF_var_emp, IF_var_emp/IF_var);
```

If `IF_var_emp / IF_var_pred` deviates significantly from 1, the `λ_c^|m|` autocorr assumption needs revisiting (could explain residual R(2,2) error).

---

## 7. Acceptance Criteria

### Pass (Phase 9 complete)

All of:
- V1: |ratio − 1| ≤ 0.05 for all three axes (paper or eff version)
- V5: |bias_pct| ≤ 2% for all three axes
- V2: linear fit slope within ±5% of predicted, R² ≥ 0.99
- V4: |IF_var_emp / IF_var_pred − 1| ≤ 0.10

### Conditional pass (paper-vs-eff revision needed)

- V1 fails for `ratio_paper` but passes for `ratio_eff`
- → File issue: "Replace paper C_n/C_dpmr in ξ with C_n_eff/C_dpmr_eff in motion_control_law_eq17_7state.m:504"
- → Re-run validation with fixed code → expect Pass

### Fail (Layer 1 formula needs revision)

- V1 fails both versions, OR V2 nonlinear, OR V4 shows large IF_var mismatch
- → Phase 9 Wave 2: full formula re-derivation under positioning conditions
- → Possible suspects: H matrix delay term `−d·δa_x` not zero in transient, EWMA bias correction missing, dx_r non-Gaussianity

---

## 8. Pitfalls and Diagnostics

### 8.1 a_var / a_pd consistency

`config.a_pd` (used in `compute_7state_cdpmr_eff_v2` for offline calibration) MUST equal the `a_var` used in `motion_control_law_eq17_7state.m` IIR LP-1.

Check: `config.a_pd === ctrl_const.a_var`. Inconsistency biases C_dpmr_eff calibration → biases E[a_xm] → inflates emp_var via `Var = E[X²] − (E[X])²` decomposition.

### 8.2 ξ definition mismatch (V6)

Current implementation uses `xi_per_axis = (C_n_paper / C_dpmr_paper) · σ²_n / (4kBT)` — paper closed form.

Phase 9 should compute both `R22_pred_paper` and `R22_pred_eff` and report both ratios. If `eff` consistently better, this is a code fix candidate.

### 8.3 Sampling length and statistical error

EWMA autocorrelation time ≈ 1/a_cov steps. Effective independent samples:
```
N_eff ≈ N_sample · a_cov
```

For a_cov = 0.005 and N_sample = 40000:
```
N_eff ≈ 200
SE(emp_var) / emp_var ≈ √(2/N_eff) ≈ 10%
```

To reach ±5% confidence, need `N_eff ≥ 800` → `N_sample · a_cov ≥ 800` → `N_sample ≥ 160000` for a_cov = 0.005.

**Action**: extend `simulation_time` to 100 sec for V2 sweep at small a_cov, or use bootstrap CI.

### 8.4 IIR not fully converged

Check IIR persistent state at sampling start:
```matlab
% sigma2_dxr_hat at end of warmup
sigma2_dxr_hat_warmup_end = sim_result.diag.sigma2_dxr_hat_log(:, N_skip);
sigma2_dxr_hat_steady     = mean(sim_result.diag.sigma2_dxr_hat_log(:, N_skip:end), 2);
ratio_warmup_to_steady    = sigma2_dxr_hat_warmup_end ./ sigma2_dxr_hat_steady;
% Should be within 0.95-1.05; otherwise extend t_warmup_kf
```

### 8.5 H matrix delay term

H(2,7) = −d = −2 means y_2 measures `a_x − 2·δa_x`, not pure `a_x`. With `a_hat_freeze`, EKF internal state e7 is still being updated; if it drifts, predicted measurement deviates from a_true.

**Mitigation**: also freeze e7 (δa_x = 0) by setting `Q_77_floor = 0` AND seeding `x_e_per_axis(7,:) = 0` at warmup end. Or accept small (≪ 1%) bias from e7 dynamics.

### 8.6 Frozen a_hat side effects

`a_hat_freeze` only overrides `a_hat` used for control law `f_d = (1/a_hat)·...`. It does NOT freeze the EKF internal state e6. Verify whether the freeze mechanism in `motion_control_law_eq17_7state.m:541-544` covers what we need for clean validation, or if additional freeze on `x_e_per_axis(6,:)` is needed.

---

## 9. Optional Extensions (Phase 9 Wave 3+)

### 9.1 Bootstrap confidence intervals

For each `emp_var`, resample with replacement to compute 95% CI. More robust than Gaussian SE for non-Gaussian a_xm distributions.

### 9.2 Spectrum of a_xm

Compute PSD of a_xm[k] log. Should show low-frequency content matching EWMA bandwidth. Anomalies (peaks, residual coupling) indicate unmodeled effects.

### 9.3 Per-axis decomposition

Currently both x and y use c_para → identical R(2,2). z uses c_perp → different. Verify all three independently before claiming "validation passes".

### 9.4 Wall sweep

Extend to `h_init ∈ {50, 30, 20, 10, 5}` to see how R(2,2) tracks (a + ξ)² as a changes via wall correction. This validates the `(a + ξ)²` factor's a-dependence directly.

---

## 10. Next Steps Branching

### Path A — All criteria pass

- Phase 9 deliverable: validation report + script
- Confidence in R(2,2) Layer 1 formula → can extend trust to Layers 2 and 3 in trajectory scenarios
- Move to Phase 10 (e2e validation under trajectory)

### Path B — Conditional pass (paper vs eff)

- File code issue: change `xi_per_axis` to use effective C ratios
- Implement fix in `motion_control_law_eq17_7state.m`
- Re-run Phase 9 → expect Path A
- Document discovery in Phase 9 Wave 2 results

### Path C — Layer 1 formula incorrect

- Trigger formula re-derivation Phase 9 Wave 2
- Suspects to investigate (in order):
  1. H matrix delay term contribution (e7 dynamics)
  2. EWMA finite-sample bias
  3. IF_var approximation vs true autocorr
  4. Closed-loop pole structure beyond λ_c^|m|
- Do NOT proceed to Phase 10 until Layer 1 confirmed

---

## 11. Deliverables

| Item | Location | Status |
|---|---|---|
| Validation plan (this doc) | `reference/eq17_analysis/phase9_R22_validation_plan.md` | Awaiting approval |
| Validation script | `test_script/run_R22_validation.m` | Phase 9 Wave 1 |
| Validation results report | `reference/eq17_analysis/phase9_R22_validation_results.md` | Phase 9 Wave 2 |
| (If Path C) Re-derivation | `reference/eq17_analysis/phase9_R22_revision.md` | Phase 9 Wave 2 |

---

## 12. References

- `reference/eq17_analysis/phase6_R_matrix_derivation.md` — R matrix derivation source
- `reference/eq17_analysis/phase2_C_dpmr_C_n_derivation.md` — C_dpmr / C_n closed forms
- `reference/eq17_analysis/phase2_IF_var_dpr_derivation.md` — IF_var derivation
- `reference/eq17_analysis/phase8_e2e_h50_results_v4.md` — Stage 11 latest h=50 results
- `model/controller/motion_control_law_eq17_7state.m:500-518` — R(2,2) implementation
- `model/controller/calc_ctrl_params.m:79-138` — Stage 11 per-axis C_dpmr_eff calibration
- `test_script/compute_7state_cdpmr_eff_v2.m` — Augmented Lyapunov for C_dpmr_eff
