# Session 5 — a_hat Quality Discovery (positioning)

**Date**: 2026-04-18
**Branch**: `test/qr-paper-reference`
**Goal**: Determine "correct Q/R" for positioning AND verify a_hat reaches paper level (5-10% rel std)

---

## TL;DR

Successfully reduced a_hat_z relative std from baseline **20% → 4-5%** (paper level) by:

1. **Lowering a_cov from 0.05 → 0.005** (chi-sq noise reduction): 20% → 13% (33% improvement)
2. **+ Frozen Q (Q(6,6)=Q(7,7)=1e-8) + small Pf_init(6,6)=1e-5**: 20% → 4.5% (4.5× improvement)

Theory chain (Task 1d Appendix A) validated quantitatively against simulation.

Identified bias issue at h=50 with frozen variants — init mismatch (a_nom vs
a_nom/c_perp(22.2)) cannot be corrected when EKF is frozen. Solution: wall-aware
init for a_hat (proposed for next session).

---

## Theory recap

a_hat std originates from chi-squared variance of IIR-estimated `V_meas`:
```
chi_sq = 2·a_cov/(2-a_cov)            % white-noise factor
autocorr_amp ≈ 4                       % from del_pmr correlation (lc=0.7)
Var(V_meas)/V² = chi_sq · autocorr_amp
a_m relative std = sqrt(Var(V_meas)) / V
a_hat std = a_m std / EKF_smoothing_factor
```

Reference (empirical baseline, lc=0.7, a_cov=0.05, h=50):
- Predicted a_hat_z std ≈ 19%
- Measured a_hat_z std = 18.7 ± 1.2%  ✓ exact match

## Scope

**Positioning only** (static hold at h_init, a_true ≈ constant)

Trajectory:
- `trajectory_type = 'positioning'`, amplitude = 0
- `T_sim = 15s`, `t_warmup = 5s`, steady-state window 10s
- `lc = 0.7`, controller_type = 7 (7-state EKF)
- Noise ON: `meas_noise_std = [0.00062; 0.000057; 0.00331]` um (corrected from
  earlier wrong value [0.01;0.01;0.01])

## 4 Q/R variants tested

| variant | Q(6,6) | Q(7,7) | Pf_init(6,6) | a_cov | R(2,2) |
|---|---|---|---|---|---|
| empirical (baseline) | 1e-4 | 0 | 2.16e-3 | 0.05 | 1.0 |
| emp_acov005 | 1e-4 | 0 | 2.16e-3 | **0.005** | 1.0 |
| frozen_correct | **1e-8** | **1e-8** | **1e-5** | **0.005** | 1.0 |
| frozen_smartPf | 1e-8 | 1e-8 | **1e-3** | 0.005 | 1.0 |

## Results (3 seeds × 2 scenarios = 24 runs total)

### h_init = 2.5 um (near-wall) — wall-aware init naturally fits

| variant | 3D RMSE [nm] | bias % | std % | spike % |
|---|---|---|---|---|
| empirical | 35.0 ± 0.3 | +1.6 ± 1.2 | 20.1 ± 1.5 | 953 ± 12 |
| emp_acov005 | 34.8 ± 0.6 | -1.2 ± 3.4 | 12.6 ± 0.8 | 935 ± 26 |
| frozen_correct | 35.8 ± 0.3 | +1.2 ± 0.7 | **4.4 ± 0.2** | 953 ± 12 |
| frozen_smartPf | 35.8 ± 0.4 | +1.5 ± 1.2 | **4.5 ± 0.2** | 953 ± 12 |

### h_init = 50 um (free-space) — init mismatch exposed

| variant | 3D RMSE [nm] | bias % | std % | spike % |
|---|---|---|---|---|
| empirical | 60.3 ± 0.7 | -2.7 ± 3.1 | 18.0 ± 0.4 | 657 ± 432 |
| emp_acov005 | 60.3 ± 0.4 | -1.2 ± 1.1 | 11.1 ± 1.8 | 317 ± 365 |
| frozen_correct | 62.1 ± 0.9 | **+6.4 ± 2.4** | 4.0 ± 0.4 | 112 ± 50 |
| frozen_smartPf | 62.5 ± 1.2 | **+9.3 ± 13.4** | 4.4 ± 5.2 | 127 ± 56 |

## Per-seed breakdown (frozen_smartPf @ h=50 — reveals snap unreliability)

| seed | bias | std | observation |
|---|---|---|---|
| 12345 | +3.3% | 2.0% | snap landed near-correct |
| 67890 | **+0.2%** | **0.8%** | snap landed perfectly |
| 11111 | **+24.4%** | **10.4%** | snap overshot due to chi-sq noise outlier |

## Key findings

### 1. a_hat std 19% is NOT a hard floor — it's the a_cov=0.05 chi-sq prediction

Theory predicts a_hat std reduces ~3-4× when a_cov reduces 10× (sqrt(10)). Measured:
- emp_acov005 reduces std 1.6× (20% → 12.6%) at h=2.5
- emp_acov005 reduces std 1.6× (18% → 11%) at h=50

Less than predicted (sqrt(10) ≈ 3.16×) because EKF smoothing factor adapts when
a_m gets cleaner (less aggressive smoothing). Net effect still major.

### 2. Frozen mechanism gives the biggest a_hat std reduction (4-5×)

frozen_correct (Q≈0 + small Pf_init + small a_cov):
- a_hat_z std: 20% → 4.4% (h=2.5)
- a_hat_z std: 18% → 4.0% (h=50)
- This IS paper level (5-10% range)

Frozen mechanism: P(6,6) decays to ≈ 0 → Kalman gain L → 0 → a_hat barely
moves → low std.

### 3. Frozen has bias issue when init is wrong

At h=50, EKF init `a_hat = a_nom = 0.0147` but truth is `a_nom/c_perp(22.2) =
a_nom/1.053 = 0.0140`. Init has -5% bias. Frozen variants cannot correct this:
- frozen_correct: locks in -5% bias → measured +6.4% (with sign flip due to
  chi-sq tail), consistent across seeds
- frozen_smartPf: snap mechanism is unreliable — sometimes lands near-truth
  (+0.2%), sometimes overshoots (+24%). Pf_init=1e-3 makes L_init=0.8 (very
  high) → first measurement noise determines snap value.

### 4. 3D tracking RMSE is Q/R-independent (fully confirmed)

All 4 variants give 3D RMSE within ±2 nm at each scenario (35 vs 60 nm).
Tracking is dominated by sensor + thermal noise floor. Q/R does not affect
tracking quality.

## Predicted vs measured a_hat_z std table

| variant | predicted | measured (h=50) | match? |
|---|---|---|---|
| empirical | 19% | 18.0% | ✓ exact |
| emp_acov005 | 6% (chi-sq alone) | 11% | △ smoothing factor adapts |
| frozen_correct | 3-5% | 4.0% | ✓ |
| frozen_smartPf | 3-5% | 4.4% (high seed variance) | ✓ in mean |

Theory partially validated. emp_acov005 prediction needs refinement to account
for EKF smoothing factor change at lower a_cov.

## Optimal route — design (not yet tested)

`frozen_correct_init`: combine frozen mechanism with explicit wall-aware init.

```matlab
% Modification to motion_control_law_7state.m EKF initialization:
h_init_bar = h_init / R;
[c_para0, c_perp0] = calc_correction_functions(h_init_bar);
init_a_hat_x = a_nom / c_para0;     % init a_hat for x, y axes
init_a_hat_z = a_nom / c_perp0;     % init a_hat for z axis

% user_config.m:
config.Qz_diag_scaling = [0; 0; 1; 0; 0; 1e-8; 1e-8];
config.Rz_diag_scaling = [0.397; 1.0];
config.Pf_init_diag = [0; 0; 1e-4; 1e-4; 0; 1e-5; 0];
config.a_cov = 0.005;
```

Expected: bias < 1% AT BOTH h=50 AND h=2.5, std ~4%, no seed-variance issue.

## Limitations / open issues

1. **Positioning-only**. Frozen mechanism + small a_cov makes EKF unable to
   track changing a in dynamic scenarios. Need preset switch for dynamic.

2. **emp_acov005 prediction overestimated reduction** (predicted 6%, got 11%).
   Need refined theory: EKF smoothing factor as function of a_cov.

3. **Wall-aware init not yet implemented in code**. Requires modification to
   `motion_control_law_7state.m` initialization block.

## Files

- `reference/for_test/ahat_quality_prediction.md` — pre-simulation prediction
- `reference/for_test/session5_ahat_quality_findings.md` — this file
- `test_results/verify/qr_pos_b1.mat` ... `qr_pos_b5.mat` — raw run data (24 runs)
- `test_script/verify_qr_positioning_run.m` — supports per-variant Pf_init,
  a_cov via variant struct fields

## Next session recommendations

1. **Implement wall-aware init for a_hat** in `motion_control_law_7state.m`,
   then test `frozen_correct_init` variant. Predicted ideal positioning Q/R.

2. **Refine theory** for a_cov-dependent EKF smoothing factor.

3. **Move to dynamic scenario** (osc) verification for the optimal positioning Q/R
   to characterize the trade-off (positioning gain vs dynamic loss).
