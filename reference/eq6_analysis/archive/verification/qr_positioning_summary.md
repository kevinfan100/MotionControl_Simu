# Q/R Positioning Verification — Summary

**Latest update**: 2026-04-18 16:57 — corrected sensor noise spec
**Branch**: `test/qr-paper-reference`

## Important: this report uses corrected sensor noise spec

Earlier 5-variant runs used `meas_noise_std = [0.01; 0.01; 0.01]` um.
Main project `run_simulation.m` uses **`[0.00062; 0.000057; 0.00331]` um**
(per-axis hardware spec). Z-axis noise is **3x smaller** than the round 0.01.

After correction, `empirical` variant in this branch matches main project's
documented Task P2 baseline (a_hat_z bias ~0%, std ~19% at lc=0.7).

---

## Setup (canonical positioning)

- Trajectory: `'positioning'` (static hold at h_init)
- T_sim = 15 s, t_warmup = 5 s, steady-state window 10 s
- lc = 0.7, controller_type = 7 (7-state EKF)
- a_pd = a_prd = a_cov = 0.05
- meas_noise_enable = true, std = `[0.00062; 0.000057; 0.00331]` um
- thermal_enable = true
- 3 seeds: 12345, 67890, 11111

---

## Final 12-run comparison (corrected noise spec, 2026-04-18)

### h_init = 2.5 um (near-wall)

| variant | 3D RMSE [nm] | a_hat_z bias % | a_hat_z std % | Notes |
|---|---|---|---|---|
| empirical | 35.0 ± 0.3 | +1.6 ± 1.2 | 20.1 ± 1.5 | matches main project baseline |
| beta | 35.7 ± 0.5 | +1.9 ± 1.3 | **4.2 ± 0.1** | a_hat FROZEN (low std) |

### h_init = 50 um (free-space)

| variant | 3D RMSE [nm] | a_hat_z bias % | a_hat_z std % | Notes |
|---|---|---|---|---|
| empirical | 60.6 ± 0.5 | -0.2 ± 1.0 | 18.7 ± 1.2 | matches main project baseline |
| beta | 62.0 ± 0.8 | **+6.9 ± 3.7** | **2.5 ± 0.3** | FROZEN, biased high |

### Comparison vs main project Task P2 baseline (lc=0.7, noise OFF)

Per-axis std (z-axis only) for sanity:
- h=2.5: this branch empirical 3D RMSE 35.0nm => sqrt(21²+21²+10²) ≈ 32nm matches P2 (z=10nm)
- h=50: this branch empirical 3D RMSE 60.6nm => sqrt(35²+35²+35²) ≈ 60.6nm matches P2 (z=35nm)
- a_hat_z bias and std match within seed variability

**Verdict: empirical baseline RECONSTRUCTED in qr branch.** Q/R variants tested
under the same conditions, only the Q/R differs.

---

## Key takeaways from positioning verification

### 1. 3D tracking RMSE is essentially Q/R-independent
- empirical and beta give same RMSE (within ±2 nm out of 35-60 nm baseline)
- Tracking error is dominated by sensor + thermal noise floor, not EKF tuning
- This confirms: Q/R choice doesn't improve positioning tracking

### 2. a_hat_z std differs dramatically
- empirical: ~19% std — EKF actively updating, normal behavior
- beta: ~3-5% std — a_hat is FROZEN (essentially constant)
- "Frozen" means EKF treats a as nearly constant → measurement updates have
  negligible Kalman gain → a_hat sits near init value

### 3. Frozen estimator is useless in dynamic scenarios
- In positioning (a static), frozen looks "clean" (low std)
- In dynamic (a varies with h_bar), frozen cannot track → a_hat → wrong value
- Hence beta is unsuitable for general use despite low static std

### 4. Earlier 5-variant comparison findings (with wrong noise spec)
Prior data (qr_positioning_combined.mat, 27 runs):
- B' (Bprime) caused 1/3 seed crash at near-wall (particle hit wall)
- B'_Remp seemed safe in 2/2 seeds tested but 1 missing due to crash
- Qemp_Rderived stable everywhere
- Pattern is consistent: B' physical Q values lack the regularization that
  empirical's Q(3,3)=1e4 provides.

These earlier 5-variant findings (B' crash etc.) are valid in their own right
but should be re-verified with corrected noise spec for direct comparison
against baseline.

---

## Files

- `test_script/verify_qr_positioning_run.m` — worker function (configurable subset)
- `test_script/verify_qr_positioning_aggregate.m` — combine batches into reports
- `test_results/verify/qr_pos_b1.mat`, `qr_pos_b2.mat` — latest 12-run data (corrected noise)
- `test_results/verify/qr_positioning_combined.mat` — older 27-run aggregate (wrong noise)
- `reference/for_test/fig_qr_positioning_summary.png` — plot (older 5-variant data)
- This file: `reference/for_test/qr_positioning_summary.md`
