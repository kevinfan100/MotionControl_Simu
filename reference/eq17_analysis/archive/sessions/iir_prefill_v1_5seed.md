# IIR Warmup Pre-Fill Validation — 5 Seeds × 2 Modes (h=50, T_sim=15s)

## Setup

- Branch:           test/eq17-5state-ekf
- Driver:           inline replication of `run_R22_validation` body, with
                    `config.iir_warmup_mode = mode` injected after `user_config()`.
- Modes tested:     'legacy' (warmup_count=2, sigma2_dxr_hat_init=0)
                    'prefill' (warmup_count=0, sigma2_dxr_hat_init = per-axis SS)
- Seeds:            1..5
- Scenario:         a_cov=0.05, sigma2_n_factor=1.0, T_sim=15s, h_init=50um
- Trajectory:       positioning hold (amplitude=0, frequency=0)
- Controller:       type=23, lambda_c=0.7, a_pd=a_cov=0.05
- Process noise:    sigma2_w_fA=0, sigma2_w_fD=0
- Meas noise:       std=[0.62e-3; 0.057e-3; 3.31e-3] um (Phase 6)
- Thermal:          enabled
- a_hat freeze:     ON (per-axis Stokes truth at h_init)
- Stats window:     N_skip=2.0s -> idx = 3201..N
- Predicted R(2,2): R22_prefactor * IF_eff_per_axis * (a_true + xi_per_axis)^2
  - R22_prefactor       = 0.05128
  - IF_eff_per_axis xyz = (logged in .mat)
  - xi_per_axis     xyz = (logged in .mat)
  - R22_pred xyz        = [3.625e-05, 3.611e-05, 3.426e-05] (same for both modes)
  - a_true   xyz [um/pN]= [1.433e-02, 1.433e-02, 1.396e-02]

## Per-mode aggregate (5-seed mean +/- std)

### V1 variance ratio (emp_var / R22_pred)  — target ~1.0

| axis | legacy mean | legacy SE | prefill mean | prefill SE |
|------|-------------|-----------|--------------|------------|
|  x   |   1.001     |  0.012    |    1.001     |   0.012    |
|  y   |   0.970     |  0.026    |    0.970     |   0.026    |
|  z   |   0.979     |  0.020    |    0.979     |   0.020    |

### V5 mean DC bias  100*(mean(a_xm) - a_true)/a_true   [%]

| axis | legacy mean | legacy std | prefill mean | prefill std |
|------|-------------|------------|--------------|-------------|
|  x   |    0.386    |   2.067    |    0.386     |   2.067     |
|  y   |   -1.325    |   1.612    |   -1.325     |   1.612     |
|  z   |   -1.097    |   0.589    |   -1.097     |   0.589     |

### Tracking RMSE  [nm]   (||p_m - p_d||_RMS per axis)

| axis | legacy mean | legacy std | prefill mean | prefill std |
|------|-------------|------------|--------------|-------------|
|  x   |   31.24     |   0.36     |    31.24     |   0.36      |
|  y   |   31.01     |   0.32     |    31.01     |   0.32      |
|  z   |   30.71     |   0.22     |    30.71     |   0.22      |

## Per-seed table

### V1 ratio per seed (legacy / prefill identical)

| seed | x_legacy | y_legacy | z_legacy | x_prefill | y_prefill | z_prefill |
|------|---------:|---------:|---------:|----------:|----------:|----------:|
|  1   |  0.987   |  0.977   |  1.008   |   0.987   |   0.977   |   1.008   |
|  2   |  1.048   |  1.060   |  0.908   |   1.048   |   1.060   |   0.908   |
|  3   |  0.993   |  0.907   |  0.999   |   0.993   |   0.907   |   0.999   |
|  4   |  0.986   |  0.934   |  0.959   |   0.986   |   0.934   |   0.959   |
|  5   |  0.991   |  0.974   |  1.020   |   0.991   |   0.974   |   1.020   |

### V5 bias per seed (%)

| seed | x_legacy | y_legacy | z_legacy | x_prefill | y_prefill | z_prefill |
|------|---------:|---------:|---------:|----------:|----------:|----------:|
|  1   |   1.09   |  -1.84   |  -0.57   |   1.09    |  -1.84    |  -0.57    |
|  2   |  -3.11   |   1.46   |  -1.63   |  -3.11    |   1.46    |  -1.63    |
|  3   |   0.56   |  -2.52   |  -0.35   |   0.56    |  -2.52    |  -0.35    |
|  4   |   1.02   |  -1.42   |  -1.51   |   1.02    |  -1.42    |  -1.51    |
|  5   |   2.37   |  -2.29   |  -1.42   |   2.37    |  -2.29    |  -1.42    |

### Tracking RMSE per seed (nm)

| seed | x_legacy | y_legacy | z_legacy | x_prefill | y_prefill | z_prefill |
|------|---------:|---------:|---------:|----------:|----------:|----------:|
|  1   |   31.50  |   30.82  |   30.66  |   31.50   |   30.82   |   30.66   |
|  2   |   30.62  |   31.55  |   30.51  |   30.62   |   31.55   |   30.51   |
|  3   |   31.20  |   30.79  |   31.13  |   31.20   |   30.79   |   31.13   |
|  4   |   31.36  |   31.11  |   30.83  |   31.36   |   31.11   |   30.83   |
|  5   |   31.51  |   30.79  |   30.61  |   31.51   |   30.79   |   30.61   |

## Health checks (all 10 sims)

| metric                | result        |
|-----------------------|---------------|
| any NaN in a_xm/p_m/f_d/ekf | 0 / 10  |
| min a_hat across all sims  | 1.396e-02 (> 0) |
| max |f_d| across all sims  | 2.378 pN (no explosion) |

## Transient diagnostic (seed 1, first 8 steps)

| step | sigma2_dxr_hat[x] legacy | sigma2_dxr_hat[x] prefill | a_xm[x] legacy | a_xm[x] prefill |
|------|-------------------------:|--------------------------:|---------------:|----------------:|
|  1   |        0.000e+00         |         0.000e+00         |   0.000e+00    |    0.000e+00    |
|  2   |        0.000e+00         |         7.376e-04         |  -6.782e-06    |    1.362e-02    |
|  3   |        0.000e+00         |         7.007e-04         |  -6.782e-06    |    1.294e-02    |
|  4   |        7.208e-06         |         6.729e-04         |   1.264e-04    |    1.242e-02    |
|  5   |        1.868e-05         |         6.511e-04         |   3.382e-04    |    1.202e-02    |
|  6   |        1.787e-05         |         6.187e-04         |   3.232e-04    |    1.142e-02    |
|  7   |        3.451e-05         |         6.053e-04         |   6.305e-04    |    1.117e-02    |
|  8   |        3.999e-05         |         5.822e-04         |   7.318e-04    |    1.075e-02    |

prefill seeds sigma2_dxr_hat to ~7e-4 immediately and a_xm jumps to ~1.36e-2 (the
steady-state per-axis Stokes truth) on step 2, whereas legacy ramps from 0.

## Verdict: WARN

### Acceptance criteria

| # | criterion                                              | result |
|---|--------------------------------------------------------|--------|
| A | prefill V1 mean >= 0.95 (xyz)                          | PASS   |
| B | prefill |V5| mean <= 1.5% (xyz, strict)                | FAIL on axis y (1.91%); PASS on x, z |
| B' | prefill |V5| <= legacy |V5| + 0.5pp                   | PASS (deltas exactly 0)              |
| C | prefill tracking RMSE within +/- 5% of legacy          | PASS (delta = 0)                     |
| D | no NaN, no a_hat <= 0, no |f_d| explosions             | PASS                                 |

### Why WARN (not PASS)

- prefill steady-state metrics are **bit-for-bit identical** to legacy at the
  N_skip=2s window. This is expected: with `a_hat_freeze` active in
  `run_R22_validation`, f_d is independent of the IIR LP / EWMA states, so the
  prefill optimization changes only `sigma2_dxr_hat` and `a_xm` transients
  (visible in first ~10 steps), not the tracked output. After 2 s skip the IIR
  states have all converged regardless of init.
- The y-axis V5 |bias| of ~1.91% mean failure of strict criterion (B) is a
  **baseline issue** present in both modes, not a prefill regression.
- prefill is **not worse than legacy** on any metric (criterion B' passes).

### Why prefill is still safe

- Health checks (D) pass cleanly on all 10 sims.
- Transient diagnostic confirms prefill behaves as designed: sigma2_dxr_hat seeded
  to per-axis steady state, a_xm jumps to true value on step 2 instead of ramping.
- prefill changes nothing in steady state -> no risk of breaking Phase 9 V1
  baseline (V1 ratio 0.99 +/- 0.02 PASS).
- Runtime is identical (~32 s per 15 s sim, both modes).

### Recommendation

prefill is **safe to keep as default**. The V5 y-axis 1.91% bias warrants a
separate investigation but is unrelated to the IIR warmup change. To exercise
prefill end-to-end (no a_hat_freeze) and confirm transient tracking benefit,
re-run this validation with `scenario.freeze_a_hat = false`.

## Files

- master .mat: `reference/eq17_analysis/iir_prefill_v1_5seed.mat`
  - results_all.{legacy,prefill}.{emp_var, emp_mean, a_true, R22_pred,
    tracking_rmse_nm, a_hat_min, f_d_max, any_nan}  (5x3 each, except a_hat_min,
    f_d_max, any_nan)
  - agg.{legacy,prefill}.{V1_mean, V1_std, V1_se, V5_mean, V5_std, V5_abs_mean,
    TR_mean, TR_std, V1_all, V5_all, TR_all, R22_pred_logged, a_true_logged}
  - acceptance: A_pass, B_pass, B_v_legacy, C_pass, D_pass, verdict
- this report: `reference/eq17_analysis/iir_prefill_v1_5seed.md`
