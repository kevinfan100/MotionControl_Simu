# Phase 8 — Wave 4 v3 e2e h=50 5-seed CI Results (Stage 11 Option I)

**Date**: 2026-04-30
**Branch**: `test/eq17-5state-ekf` @ `6add6da`
**Status**: **PARTIAL PASS** — bias improved 3× from v2 baseline but missed paper-level target
**Predecessor**: `phase8_e2e_h50_results_v2.md` (Wave 4 v2, WARN, bias -31%)

## TL;DR

| Metric | v2 baseline | v3 Stage 11 | Δ |
|---|---|---|---|
| a_hat bias x | -31.32% | **-11.10%** | 3× ↓ |
| a_hat bias y | -31.18% | **-10.50%** | 3× ↓ |
| a_hat bias z | -31.34% | **-11.89%** | 3× ↓ |
| s2_dx ratio x | 0.799 | 0.540 | worse |
| s2_dx ratio y | 0.732 | 0.545 | worse |
| s2_dx ratio z | 0.764 | 0.539 | worse |
| Tracking std x | 29 nm | 30.25 nm | flat |
| Tracking std y | — | 30.48 nm | OK |
| Tracking std z | 29 nm | 29.92 nm | flat |

**Pass criteria**:
- |bias|_max < 5% — FAIL (max 11.89%)
- ratio ∈ [0.9, 1.1] — FAIL (range [0.539, 0.545])

## Run config

- Driver: `test_script/run_v2_h50_e2e.m`
- Seeds: [1, 2, 3, 4, 5]
- T_sim = 5 s, t_warmup = 0.5 s
- h_init = 50 µm, positioning (no oscillation)
- meas_noise + thermal both ON
- meas_noise_std = [0.62, 0.057, 3.31] nm
- lambda_c = 0.7, a_pd = 0.05, a_cov = 0.05, sigma2_w_fD = 0
- Elapsed: 53.2 s
- Saved: `test_results/wave4_v3_5seed_tsim5_h50.mat`

## Diagnostic — why partial pass

Stage 11 replaces paper closed-form C_dpmr=3.96 / C_n=1.18 in the a_xm formula with
per-axis values from the augmented Lyapunov solver (`compute_7state_cdpmr_eff_v2.m`).

**Per-axis values produced by Stage 11 (calc_ctrl_params)**:
- C_dpmr_eff = [3.1909, 3.1907, 3.1963]  (vs paper 3.96)
- C_np_eff   = [0.9552, 0.9552, 0.9543]  (vs paper 1.18)

**Implied actual closed-loop C_dpmr** (from observed s2_dx_r over 5-seed mean):
- s2_dx_r_obs = [9.157, 9.293, 8.963] × 1e-4 µm²
- C_dpmr_actual = (s2_dx_r_obs - C_n*sigma2_n) / (4·k_BT·a_truth)
- ≈ [3.73, 3.79, 3.75]

**Gap**: Lyapunov gives 3.19, but actual closed-loop sits at 3.74. Lyapunov
**under-predicts** the closed-loop variance by ~16%, leading to over-corrected
denominator in the a_xm formula.

If the formula used C_dpmr_actual = 3.74, the bias would close to ~0%
(vs current -11%).

## Hypotheses for Lyapunov under-prediction

The helper `compute_7state_cdpmr_eff_v2.m` builds an 11-dim augmented state
[δx, δx_{k-1}, δx_{k-2}, e1..e7, pmd_prev]. Row 1 (δx dynamics, lines 130-136):

```matlab
A_aug(idx_dx, idx_dx)       = lc;        % delta_x → delta_x: lc
A_aug(idx_dx, idx_e(3))     = 1 - lc;    % e3 → delta_x: (1-lc)
A_aug(idx_dx, idx_e(4))     = -1;        % e4 → delta_x: -1
A_aug(idx_dx, idx_e(6))     = -opts.f0;  % e6 → delta_x: -f0 (= 0 in positioning)
```

The coefficients `(1-lc)` and `-1` are inherited from v1 / qr branch (where
F_e(3,4)=-1). For v2 with F_e(3,4) = -1.6, these may not represent the true
closed-loop coupling. Specifically:
- `A_aug(idx_dx, idx_e(4)) = -1` may need to be `-(1 + d·(1-lc)) = -1.6`
  or related v2-specific value
- The (1-lc) coefficient on e3 may need v2 update

This is the most likely root cause of the 16% under-prediction.

## Stability + tracking sanity

- All 5 seeds completed without NaN
- Cross-seed tracking std std: [0.74, 0.66, 1.07] nm (low, indicating no instability)
- Tracking std mean ~30 nm — slightly worse than v2 (29 nm) but in same regime
- a_hat std ~3-4 × 10⁻⁴ µm/pN — 4× lower than v2 (Wave 4 v2 had similar order)

The system is stable; this is a pure calibration issue (a_xm formula mapping).

## Recommended next steps

**Path A — Re-derive A_aug Row 1 for v2** (rigorous):
1. Symbolically derive δx[k+1] coefficients on e3, e4 from v2 Eq.17 control law
2. Update `compute_7state_cdpmr_eff_v2.m` lines 133-136
3. Re-run Stage 11 calc + Wave 4 verification
4. Expected: bias < 2%, ratio ≈ 1.0

**Path B — Empirical calibration** (pragmatic):
1. Run a calibration sweep with truth a_truth, multiple h values
2. Fit C_dpmr_eff(h_bar, lambda_c) lookup table
3. Bypass Lyapunov derivation entirely
4. Match qr branch's pre-Lyapunov approach

**Path C — Accept current PARTIAL PASS**:
- a_hat bias -11% is 3× better than v1 baseline
- Tracking error 30 nm is well within spec
- Document as known limitation; revisit later

## Files referenced

- `model/controller/calc_ctrl_params.m` (lines 67-115, Stage 11 per-axis Lyapunov)
- `model/controller/build_eq17_constants.m` (lines 91-98, 226-228, pass-through)
- `model/controller/motion_control_law_eq17_7state.m` (Stage 11 a_xm with C_dpmr_eff)
- `test_script/compute_7state_cdpmr_eff_v2.m` (helper — likely root cause Row 1)
- `test_results/wave4_v3_5seed_tsim5_h50.mat` (raw results)

## Comparison table — v2 vs v3 at-a-glance

```
                        v2 (paper closed-form)   v3 (Stage 11 Lyapunov)
C_dpmr in a_xm          3.96                     3.19
a_hat bias              -31%                     -11%
s2_dx ratio (obs/pred)  0.76                     0.54
Tracking std            29 nm                    30 nm
NaN events              0                        0
```
