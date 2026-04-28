# Phase 4 Final Report: Autonomous Research Session Results

## Executive Summary

This report documents an autonomous research session on the MotionControl_Simu
project, addressing the user's core question: **"How to estimate motion gain
a[k] from feedback del_pm for the 7-state EKF, with math that's analytically
derivable and only involves (lc, Q, R, delay, a_pd)?"**

**Main achievement**: derived and validated the correct `C_dpmr_eff` and
`C_np_eff` formulas for the 7-state EKF via augmented Lyapunov analysis.
Variance prediction accuracy improved **from 26% to 4.5%** mean error vs
Simulink (~6x improvement). Integration is complete, backward-compatible,
and regression-tested.

**Gap to paper precision remaining**: the 7-state EKF's a_hat estimate has
~10-20% rel std in static free-space (vs paper's ~5-10%). This gap is
dominated by chi-squared statistical noise + temporal autocorrelation, and
a small residual ~7-10% bias that requires further investigation.

## Original Goal (from user)

> 「我要怎麼在知道 thermal motion 的情況下知道 thermal force variance ... 這個是建立
> 在控制器架構好後我未知現在的 motion gain 需要估測 ... 轉換過程應該只會涉及到 lc & Q & R
> & two step delay」

Translated and refined through the initial discussion:
1. **Input**: del_pm feedback (tracking error)
2. **Output**: real-time `a[k]` estimate, fed into the 7-state KF
3. **Precision target**: match the paper "Near-Wall Ultra-precise Motion Control..."
4. **Constraint**: no wall distance prior, math fully derivable, involves only
   (lc, Q, R, two-step delay, a_pd)
5. **Execution**: autonomous (user away), no user intervention required

## Phase 1: C_dpmr_eff Correction (COMPLETED, integrated)

### Approach

Built an 11-dimensional augmented state:
```
x_aug = [delta_x, dx_d1, dx_d2, e1..e7, pmd_prev]
```
and solved two steady-state Lyapunov equations (thermal-only and pos-noise-only)
to extract:
```
Var(del_pmr) = C_dpmr_eff * (4*k_B*T*a) + C_np_eff * sigma2_n
```

### Mathematical derivation

- `Fe` (7x7) from paper Eq.18 with f_0 = 0 (steady-state free-space)
- `A_e = Fe*(I - L*H)` where L is the steady-state Kalman gain from iterative DARE
- `A_aug` (11x11) couples delta_x dynamics (row 1 with lc, (1-lc)*e3, -e4)
  to the delay chain (rows 2, 3), error dynamics (rows 4-10 = A_e), and IIR
  LP state (row 11: `(1-a_pd)*pmd_prev + a_pd*dx_d2`)
- Extraction: `C_dpmr_eff = (1-a_pd)^2 * (S(3,3)+S(11,11)-2*S(3,11))` from the
  unit-thermal Lyapunov solve

### Numerical challenges

- Iterative DARE takes ~5000-30000 iterations (marginal eigenvalues at 1)
- Built-in `dare()` fails (Symplectic spectrum too close to unit circle)
- `dlyap()` fails on the resulting ill-conditioned A_aug
- Solution: iterative Riccati with `Pf_init = Q + I` and `dlyapchol` for the
  Lyapunov stage (automatic fallback to `dlyap` and then fixed-point iteration)

### Results

| Metric | Before (K=2) | After (new) | Improvement |
|--------|-------------|-------------|-------------|
| `C_dpmr(0.7)` | 3.161 | 3.924 | +24% |
| Mean Var prediction error (6 test points) | 26.2% | **4.5%** | **~6x** |
| Max Var prediction error | 37.7% | 7.6% | ~5x |

All 6 Simulink verification points pass the gate (mean < 5%, max < 10%).

### Files created / modified

- **Core**: `test_script/compute_7state_cdpmr_eff.m`
- **Lookup**: `test_script/build_cdpmr_eff_lookup.m` → `test_results/verify/cdpmr_eff_lookup.mat`
- **Tests**: `verify_cdpmr_eff_sanity.m`, `verify_cdpmr_eff_simulink.m`, `dryrun_7state_with_cdpmr_eff.m`, `regression_7state_new_cdpmr.m`
- **Integration**: `model/controller/calc_ctrl_params.m`, `model/controller/motion_control_law_7state.m`, `model/calc_simulation_params.m` (CtrlBus)

## Phase 2: Statistical Floor Characterization (COMPLETED)

### Experiment 2A: chi-squared noise baseline

30-second free-space run at `lc=0.7`, IIR smoother with `a_cov=0.05`.

| Axis | mean(a_m)/a_nom | bias | rel. std |
|------|-----------------|------|----------|
| x    | 89.9%           | -10.1% | 44.1% |
| y    | 88.4%           | -11.6% | 42.2% |
| z    | 92.5%           | -7.5%  | 43.6% |

Chi-squared theory prediction: `sqrt(2/N_eff) = 31.6%` for `N_eff = 20`.
Actual ~44% is **~38% higher than theory due to temporal autocorrelation**
(successive del_pmr samples are correlated through the closed-loop dynamics,
giving ~1/4 independent samples).

### Experiment 2C: paper-like benchmark

1 Hz oscillation, amplitude 2.5 um, descending to h=2.5 um (near wall), lc=0.4
(matching paper).

| axis | mean err | std err |
|------|---------|---------|
| x    | 0.0 nm  | 27.6 nm |
| y    | 0.2 nm  | 27.7 nm |
| z    | 2.2 nm  | 22.6 nm |

- **3D RMSE = 41.2 nm** (comparable to paper's rough 30-50 nm)
- a_hat_z median error vs a_true: **17.4%** (paper Fig 6 visually ~5-10%)

### Diagnostic conclusion

- Bias: ~8-10% (independent of a_cov, investigation needed)
- Variance: ~43% rel std on raw a_m, smoothed to ~17-22% by EKF
- Tracking error is comparable to paper; **a_hat precision is ~2x worse**

## Phase 3: Advanced Methods (EXPLORED)

### 3b-1: Multi-lag autocorrelation

**Theory**: `gamma_L = c' * A_aug^L * Sigma_th_unit * c * a` — each lag gives
a linear measurement of `a`. Theoretically, multi-lag LS should improve precision.

**Result**: In practice, adding lags HURT precision (due to correlated lag
estimates + diminishing SNR at higher lags). Single-lag (block sample variance)
is the best from this family.

### 3b-2: a_cov sweep (offline IIR re-processing)

Recomputed IIR with various a_cov on Phase 2A data:

| a_cov | rel std | bias | response |
|-------|---------|------|----------|
| 0.100 | 60.1%   | -7.5% | 6.25 ms |
| 0.050 (default) | 43.6% | -7.5% | 12.5 ms |
| 0.020 | 30.1%   | -7.4% | 31.2 ms |
| 0.010 | 23.8%   | -7.3% | 62.5 ms |
| 0.005 | 19.6%   | -7.3% | 125.0 ms |

**Key finding**: smaller `a_cov` gives better precision but more lag. Bias is
essentially independent of a_cov.

### 3b-3: Simulink closed-loop validation of a_cov change

Ran actual Simulink with a_cov ∈ {0.05, 0.01, 0.005}, free-space positioning:

| a_cov | a_hat_x rel std | a_hat_z rel std | 3D tracking RMSE |
|-------|-----------------|-----------------|------------------|
| 0.050 | 20.9%           | 22.5%           | 61.9 nm          |
| 0.010 | 14.2%           | 17.3%           | 60.9 nm          |
| 0.005 | 10.7%           | 11.2%           | 60.8 nm          |

**Finding**: reducing a_cov gives **~2x better a_hat precision**, with
**unchanged tracking error**.

### 3b-4: Near-wall dynamic re-test with a_cov=0.005

Re-ran the paper benchmark with a_cov=0.005. Result: a_hat dynamic tracking
error actually **worsened** (21.8% vs 17.4% at a_cov=0.05), because the added
lag dominates in the near-wall time-varying scenario.

**Conclusion**: the optimal a_cov is **scenario-dependent**. 0.05 is the
best single value for mixed scenarios. An adaptive a_cov (small when a is
slowly varying, larger when near wall) would be superior.

## Phase 4: Final Integration

### Current state of the controller

- `motion_control_law_7state.m` now uses `C_dpmr_eff` and `C_np_eff` from the
  lookup table (lookup_path: `test_results/verify/cdpmr_eff_lookup.mat`)
- `a_cov` reverted to 0.05 (the robust cross-scenario default)
- Backward compatible: falls back to K=2 if lookup missing
- Regression tests pass (free-space a_hat within 25% of a_nom)

### Final tracking performance (paper benchmark)

- **3D RMSE = 41.6 nm** (unchanged by a_cov tuning)
- **a_hat_z median error = 17-22%** vs a_true (depends on a_cov)

This is 2x worse than the paper's ~5-10% precision target, so the goal is
**not fully met**, but the Phase 1 correction (C_dpmr_eff) is a substantial
first-principles improvement and the remaining gap is clearly characterized.

## Remaining Work for Paper Precision

### 1. Investigate the persistent 7-10% bias

The bias is present in ALL configurations (different a_cov, different lc)
and is independent of the statistical window. Candidates:

- `f_0 = 0` assumption: the real controller has transient f_d during
  thermal "kicks". Solving the Lyapunov with `f_0 != 0` may shift C_dpmr_eff.
- `Pf_init` convergence: iterative DARE with `Q + I` init may converge to a
  slightly different fixed point than the actual controller's runtime EKF.
- Higher-order statistical bias: IIR smoothing of `del_pmr^2` has a O(1/N)
  negative bias in the finite-sample mean.

**Concrete investigation path**:
1. Run Simulink with a deterministic f_d test signal; measure realized L_ss vs
   the iterative-DARE L_ss used to build the lookup
2. Retry augmented Lyapunov with `f_0 = mean(f_d)` over the trajectory
3. Quantify the finite-sample bias of the IIR smoother analytically

### 2. Adaptive a_cov or hybrid estimator

The scenario-dependent optimal a_cov suggests:
- **Slow regime** (far from wall): use small a_cov (long window)
- **Fast regime** (near wall, rapid a change): use large a_cov
- **Hybrid**: two parallel variance estimators, weighted by a "speed indicator"

A trigger could be `|d(a_hat)/dt| > threshold` → switch to fast IIR.

### 3. Model-based separation (Phase 3a fallback)

If the IIR HP filter's bias is irreducible, try:
- Predict deterministic del_pm from the KF's own state and f_d history
- Subtract this model prediction from raw del_pm
- The residual should be cleaner (less bias)

## File Inventory (session artifacts)

### Scripts created

| File | Purpose |
|------|---------|
| `test_script/compute_7state_cdpmr_eff.m` | Core augmented Lyapunov function |
| `test_script/verify_cdpmr_eff_sanity.m` | 5 unit tests |
| `test_script/build_cdpmr_eff_lookup.m` | Offline lookup builder |
| `test_script/verify_cdpmr_eff_simulink.m` | Simulink variance verification |
| `test_script/dryrun_7state_with_cdpmr_eff.m` | Advisory pre-integration check |
| `test_script/regression_7state_new_cdpmr.m` | Post-integration regression |
| `test_script/phase2_chisquared_mc.m` | Chi-squared noise baseline |
| `test_script/phase2_paper_benchmark.m` | Paper-like 3D benchmark |
| `test_script/phase3b_multilag_autocorr.m` | Multi-lag theory + test |
| `test_script/phase3b_acov_sweep.m` | Offline a_cov sweep |
| `test_script/phase3_acov_validate.m` | Simulink closed-loop a_cov validation |

### Modified files

| File | Change |
|------|--------|
| `model/controller/calc_ctrl_params.m` | Load cdpmr_eff_lookup.mat, interpolate over lc, store ctrl.C_dpmr_eff/C_np_eff |
| `model/controller/motion_control_law_7state.m` | Use persistent C_dpmr_eff_const / C_np_eff_const in Eq.13 (fallback to K=2) |
| `model/calc_simulation_params.m` | Added C_dpmr_eff, C_np_eff to CtrlBus (elements 26, 27) |
| `model/config/user_config.m` | a_cov documented as Phase 3 tested parameter (kept at 0.05) |

### Data files (gitignored in test_results/verify/)

- `cdpmr_eff_lookup.mat` — THE lookup table (6x21 grid)
- `verify_cdpmr_eff_sanity.mat`
- `verify_cdpmr_eff_simulink_results.mat`
- `dryrun_7state_cdpmr_eff.mat`
- `regression_7state_new_cdpmr.mat`
- `phase2_chisquared_mc.mat`
- `phase2_paper_benchmark.mat`
- `phase3b_multilag_autocorr.mat`
- `phase3b_acov_sweep.mat`
- `phase3_acov_validate.mat`

### Reports (reference/for_test/)

- `phase1_report.md` — augmented Lyapunov derivation + verification
- `phase2_report.md` — statistical floor characterization
- `phase4_final_report.md` — this document

### Figures (reference/for_test/)

- `fig_cdpmr_eff_lookup.png`
- `fig_cdpmr_eff_verification.png`
- `fig_phase2_chisquared.png`
- `fig_phase2_paper_benchmark.png`
- `fig_phase3b_multilag.png`

## Summary: what was solved, what remains

### Solved (ready to use)

1. **Correct `C_dpmr` for the 7-state EKF**, derived via augmented Lyapunov.
   ~6x improvement in variance prediction accuracy (26% → 4.5%).
2. **`noise_corr_dpmr`** now uses `C_np_eff` (the correct analytic form,
   previously was using the wrong del_pm formula).
3. **Integration**: the controller transparently uses the new values via
   lookup table; falls back to K=2 if lookup is missing.
4. **Statistical floor characterized**: chi-squared + autocorrelation gives
   ~43% raw noise, which the EKF smooths to ~17-22% a_hat rel std.

### Remaining for paper precision

1. The **~8% systematic bias** in a_m needs investigation (possibly
   `f_0 != 0` or finite-sample IIR bias).
2. **a_hat precision is 2x worse than paper** (~17% vs ~5-10%).
3. Potential solutions identified but not implemented:
   - Adaptive a_cov
   - Augmented Lyapunov with non-zero f_0
   - Model-based separator instead of IIR HP filter
   - Multi-axis joint estimation (unused cross-correlation between x/y/z)

### Recommendations for next session

When the user returns, suggest:
1. Read `phase4_final_report.md` (this file) for the full picture
2. Decide if the 41 nm 3D RMSE is sufficient, or if deeper work is needed
3. If more precision needed: start with investigating the 8% bias
   (it's the lowest-hanging fruit and may give big improvements)
4. The Phase 1 `C_dpmr_eff` correction is ALREADY integrated and is a
   permanent improvement to the controller baseline
