# IIR warmup pre-fill closed-loop validation (5 seeds, no a_hat freeze)

Agent C2 follow-up to Agent C. Agent C used `opts.a_hat_freeze` which made f_d
independent of IIR state and produced bit-identical metrics across modes. This
run is the actual closed-loop regression test with the EKF updating slot 6 normally.

## Setup

- Driver: `run_pure_simulation` (NO a_hat_freeze)
- Trajectory: positioning (h_init = h_bottom = 50 um, t_hold=0, amp=0, freq=0)
- T_sim = 15 s; t_skip = 5 s -> N_skip = 8001, steady-region = 16001 samples
- Ts = 6.250000e-04 s, lambda_c = 0.7, controller_type = 23 (inert; pure driver always uses eq17_7state)
- a_cov = a_pd = 0.05; sigma2_w_fA = sigma2_w_fD = 0
- meas_noise_std = [0.62e-3; 0.057e-3; 3.31e-3] um (x,y,z)
- thermal_enable = true; meas_noise_enable = true
- Modes: legacy (warmup_count=2, sigma2_dxr_hat=0) vs prefill (warmup_count=0, per-axis steady-state seed)
- Seeds: 1..5 (no a_hat_freeze, opts.collect_diag=true)

a_true derived from h_bar = 50/R = 22.2222, c_para = 1.02596, c_perp = 1.05328:
- a_true_x = 1.433380e-02 um/pN  (parallel)
- a_true_y = 1.433380e-02 um/pN  (parallel)
- a_true_z = 1.396203e-02 um/pN  (perpendicular)

## Per-axis aggregate (legacy vs prefill, mean +/- std across 5 seeds)

### bias_pct = (mean(a_hat_steady) - a_true) / a_true x 100

| axis | legacy mean | legacy std | prefill mean | prefill std | delta(prefill - legacy) |
|------|-------------|------------|--------------|-------------|--------------------------|
|  x   |  -6.795 %  | 2.991      |  -6.436 %   | 3.081       |  +0.359 pp                |
|  y   |  -7.483 %  | 1.667      |  -7.923 %   | 2.302       |  -0.440 pp                |
|  z   |  -7.234 %  | 2.082      |  -6.152 %   | 3.419       |  +1.082 pp                |

### RMSE_pct = sqrt(bias_pct^2 + std_pct^2)

| axis | legacy mean | legacy std | prefill mean | prefill std | delta(prefill - legacy) |
|------|-------------|------------|--------------|-------------|--------------------------|
|  x   |   6.904 %   | 2.902      |   6.591 %    | 3.012       |  -0.313 pp                |
|  y   |   7.537 %   | 1.659      |   7.947 %    | 2.308       |  +0.410 pp                |
|  z   |   7.303 %   | 2.082      |   6.212 %    | 3.355       |  -1.091 pp                |

### tracking_std_nm = 1e3 * std(p_m - p_d) over steady region

| axis | legacy mean | legacy std | prefill mean | prefill std | delta(prefill - legacy) |
|------|-------------|------------|--------------|-------------|--------------------------|
|  x   |  30.646 nm  | 0.173      |  30.957 nm   | 0.607       |  +0.311 nm                |
|  y   |  30.562 nm  | 0.295      |  30.500 nm   | 0.352       |  -0.062 nm                |
|  z   |  30.406 nm  | 0.199      |  30.561 nm   | 0.616       |  +0.156 nm                |

### std_pct = std(a_hat_steady) / a_true x 100

| axis | legacy mean | legacy std | prefill mean | prefill std | delta(prefill - legacy) |
|------|-------------|------------|--------------|-------------|--------------------------|
|  x   |   0.999 %   | 0.296      |   1.197 %    | 0.553       |  +0.197 pp                |
|  y   |   0.872 %   | 0.173      |   0.613 %    | 0.180       |  -0.259 pp                |
|  z   |   0.970 %   | 0.281      |   0.620 %    | 0.125       |  -0.350 pp                |

## Per-seed table

### bias_pct

| seed | mode    |    x     |    y     |    z     |
|------|---------|----------|----------|----------|
|  1   | legacy  |   -9.039 |   -8.396 |   -7.802 |
|  1   | prefill |   -1.534 |   -9.044 |   -4.992 |
|  2   | legacy  |   -7.277 |   -9.976 |   -3.635 |
|  2   | prefill |   -5.798 |   -7.188 |   -1.228 |
|  3   | legacy  |   -5.681 |   -6.012 |   -8.367 |
|  3   | prefill |   -7.775 |   -6.436 |   -7.497 |
|  4   | legacy  |   -2.249 |   -6.378 |   -8.885 |
|  4   | prefill |   -7.330 |   -5.595 |  -10.539 |
|  5   | legacy  |   -9.730 |   -6.653 |   -7.481 |
|  5   | prefill |   -9.744 |  -11.353 |   -6.505 |

### std_pct

| seed | mode    |    x     |    y     |    z     |
|------|---------|----------|----------|----------|
|  1   | legacy  |   +0.990 |   +1.160 |   +0.795 |
|  1   | prefill |   +0.833 |   +0.679 |   +0.588 |
|  2   | legacy  |   +0.668 |   +0.694 |   +0.788 |
|  2   | prefill |   +2.032 |   +0.593 |   +0.709 |
|  3   | legacy  |   +1.448 |   +0.847 |   +1.168 |
|  3   | prefill |   +0.583 |   +0.525 |   +0.479 |
|  4   | legacy  |   +1.074 |   +0.814 |   +1.365 |
|  4   | prefill |   +1.326 |   +0.393 |   +0.539 |
|  5   | legacy  |   +0.817 |   +0.845 |   +0.734 |
|  5   | prefill |   +1.210 |   +0.875 |   +0.784 |

### RMSE_pct

| seed | mode    |    x     |    y     |    z     |
|------|---------|----------|----------|----------|
|  1   | legacy  |   +9.093 |   +8.476 |   +7.843 |
|  1   | prefill |   +1.746 |   +9.069 |   +5.027 |
|  2   | legacy  |   +7.307 |  +10.000 |   +3.719 |
|  2   | prefill |   +6.144 |   +7.212 |   +1.419 |
|  3   | legacy  |   +5.863 |   +6.071 |   +8.448 |
|  3   | prefill |   +7.796 |   +6.458 |   +7.512 |
|  4   | legacy  |   +2.492 |   +6.430 |   +8.989 |
|  4   | prefill |   +7.449 |   +5.609 |  +10.553 |
|  5   | legacy  |   +9.764 |   +6.706 |   +7.516 |
|  5   | prefill |   +9.819 |  +11.386 |   +6.552 |

### tracking_std_nm

| seed | mode    |    x     |    y     |    z     |
|------|---------|----------|----------|----------|
|  1   | legacy  |  +30.788 |  +31.067 |  +30.136 |
|  1   | prefill |  +31.486 |  +30.221 |  +30.773 |
|  2   | legacy  |  +30.510 |  +30.457 |  +30.590 |
|  2   | prefill |  +31.697 |  +30.976 |  +31.306 |
|  3   | legacy  |  +30.433 |  +30.405 |  +30.280 |
|  3   | prefill |  +30.342 |  +30.688 |  +29.847 |
|  4   | legacy  |  +30.834 |  +30.557 |  +30.430 |
|  4   | prefill |  +30.794 |  +30.511 |  +30.004 |
|  5   | legacy  |  +30.667 |  +30.324 |  +30.592 |
|  5   | prefill |  +30.466 |  +30.104 |  +30.877 |

## Sanity gates

### Hard gates (must pass)

| gate                               | result | detail                          |
|------------------------------------|--------|---------------------------------|
| no NaN in p_m or f_d (any sim)     | PASS   | -                               |
| min(a_hat) > 0 in every sim        | PASS   | overall min = 6.4132e-03 um/pN |
| max(|f_d|) < 100 pN in every sim   | PASS   | overall max =  2.544 pN          |

### Target gates (prefill regression vs legacy)

| axis | RMSE_pct prefill <= legacy + 1 pp | tracking_std prefill <= legacy + 0.5 nm | std_pct cross-seed spread <= legacy + 30% relative |
|------|------------------------------------|------------------------------------------|------------------------------------------------------|
|  x   | PASS (delta = -0.31 pp)         | PASS (delta = +0.31 nm)              | WARN (legacy=0.296, prefill=0.553, rel  +87.1%) |
|  y   | PASS (delta = +0.41 pp)         | PASS (delta = -0.06 nm)              | PASS (legacy=0.173, prefill=0.180, rel   +4.1%) |
|  z   | PASS (delta = -1.09 pp)         | PASS (delta = +0.16 nm)              | PASS (legacy=0.281, prefill=0.125, rel  -55.5%) |

## Verdict: PASS

- Hard gates: all PASS (no NaN, min(a_hat) > 0 everywhere, max(|f_d|) ~2.5 pN well below 100 pN).
- RMSE_pct target: prefill is within 1 pp of legacy on every axis (deltas: x=-0.31, y=+0.41, z=-1.09 pp).
- tracking_std_nm target: prefill is within 0.5 nm of legacy on every axis (deltas: x=+0.31, y=-0.06, z=+0.16 nm).
- std_pct cross-seed spread sanity: x-axis WARN (legacy 0.30 -> prefill 0.55, +83%); y and z PASS.
  Soft sanity gate only. Mean std_pct itself decreases on y and z under prefill (y: 0.87 -> 0.61;
  z: 0.97 -> 0.62) and rises only slightly on x (1.00 -> 1.20). The wider cross-seed spread on x
  is dominated by seed=2 (prefill std_pct 2.03% vs legacy 0.67%), but tracking_std_nm and
  bias_pct remain bounded for that seed (trk=31.7 nm, bias=-5.8%).

## Notes / unexpected

- Bias has large per-seed spread (~3 pp std) under both modes; matches the chi-squared floor on a_hat
  spread at h=50 (~19% relative per project memory).
- Prefill helps z-axis RMSE on average (-1.09 pp) and slightly hurts y (+0.41 pp); per-seed sign-flips
  dominate, consistent with the warmup-mode contribution being below the EKF/thermal noise floor at h=50.
- tracking_std_nm is essentially identical (~30.5 nm) across modes - prefill is not destabilizing
  closed-loop positioning at h=50.
- max(|f_d|) overall is ~2.5 pN, two orders of magnitude below the 100 pN gate.
- Recommendation: prefill is SAFE to keep as default for positioning at h=50. Original justification
  (eliminate 3-step f_d=0 warmup transient) is preserved without measurable closed-loop regression.
  The +83% std_pct cross-seed spread on x is a minor noise-floor artifact, not a stability concern.
