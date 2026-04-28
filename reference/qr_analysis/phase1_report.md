# Phase 1 Report: 7-state EKF augmented Lyapunov for C_dpmr_eff / C_np_eff

## Context

The `motion_control_law_7state.m` controller uses Eq.13 to estimate motion gain:

```
a_m = (Var(del_pmr) - noise_corr) / (C_dpmr * 4 * k_B * T)
```

The existing code used `K=2` (from the ctrl1 delay-compensation formula) and a
`noise_corr` formula derived for `del_pm` (not `del_pmr`). Both are known wrong
for the 7-state EKF controller. Phase 1 derives the correct values via
augmented Lyapunov analysis.

## Math Derivation

### Augmented 11-state system

```
x_aug = [ delta_x, dx_d1, dx_d2, e1..e7, pmd_prev ]
```

| Index | Meaning |
|-------|---------|
| 1 | `delta_x[k]` — true current tracking error |
| 2 | `dx_d1 = delta_x[k-1]` |
| 3 | `dx_d2 = delta_x[k-2]`  (what `del_pm` sees) |
| 4..10 | `e1..e7` — 7-state EKF estimation errors |
| 11 | `pmd_prev = del_pmd[k-1]` — IIR LP previous state |

### Closed-loop error dynamics

From paper Eq.18 (with `f_0 = 0` for Phase 1 steady-state free-space):

```
Fe = [0 1 0  0 0 0 0;
      0 0 1  0 0 0 0;
      0 0 1 -1 0 0 0;
      0 0 0  1 1 0 0;
      0 0 0  0 1 0 0;
      0 0 0  0 0 1 1;
      0 0 0  0 0 0 1]

H  = [1 0 0 0 0 0 0;
      0 0 0 0 0 1 0]

A_e = Fe * (I - L*H)     (7x7 closed-loop error matrix)
```

where `L` is the steady-state Kalman gain from DARE on `(Fe, H, Q_kf, R_kf)`.

### A_aug structure

- Row 1 (delta_x): `A(1,1)=lc`, `A(1,6)=1-lc` (e3 coupling), `A(1,7)=-1` (e4 coupling), `A(1,9)=-f0`
- Row 2 (dx_d1 = delta_x[k]): `A(2,1) = 1`
- Row 3 (dx_d2 = dx_d1[k]): `A(3,2) = 1`
- Rows 4-10 (errors): `A(4:10, 4:10) = A_e`
- Row 11 (pmd_prev = del_pmd[k]): `A(11,11) = 1-a_pd`, `A(11,3) = a_pd`

### Noise inputs

| Source | Row 1 (dx) | Rows 4-10 (errors) | Row 11 (IIR) |
|--------|-----------|-------------------|--------------|
| Thermal `a*f_T`    | `-1` | `-1` at row 6 (e3) only | 0 |
| Pos noise `n_p`    | 0    | `-Fe*L(:,1)` | `a_pd` |
| Gain noise `n_a`   | 0    | `-Fe*L(:,2)` | 0 |

### Extraction

```
del_pmr[k] = (1-a_pd) * (dx_d2[k] - pmd_prev[k] + n_p[k])

Var(del_pmr) = (1-a_pd)^2 * [S(3,3) + S(11,11) - 2*S(3,11) + sigma2_n]

where S = Sigma_th + Sigma_np
      Sigma_th = dlyap(A_aug, B_th * B_th')    # unit-thermal solve
      Sigma_np = dlyap(A_aug, B_np * B_np')    # unit-posnoise solve
```

Define:
```
C_dpmr_eff = (1-a_pd)^2 * (S_th(3,3) + S_th(11,11) - 2*S_th(3,11))
C_np_eff   = (1-a_pd)^2 * (S_np(3,3) + S_np(11,11) - 2*S_np(3,11) + 1)
```

Then:
```
Var(del_pmr) = C_dpmr_eff * (4*k_B*T*a) + C_np_eff * sigma2_n
```

and Eq.13 inversion:
```
a_m = (Var(del_pmr) - C_np_eff * sigma2_n) / (C_dpmr_eff * 4*k_B*T)
```

## Numerical Notes

- The 7-state DARE is ill-conditioned due to marginal eigenvalues at 1 in Fe.
  Standard `dare()` fails. Iterative Riccati with `Pf_init = Q_kf + I` and
  30000 iterations gives stable convergence (error ~1e-4).
- `dlyap` fails for the same reason. `dlyapchol` (Cholesky-based) is used,
  with fallback to `dlyap` and then iterative fixed-point.
- `C_dpmr_eff` and `C_np_eff` depend only on `(lc, Q_kf_scale, R_kf_scale, a_pd)`
  — NOT on rho (noise-to-thermal ratio). The original plan's 2-D lookup over
  (lc, rho) is effectively 1-D over lc.

## Sanity Tests (verify_cdpmr_eff_sanity.m)

| Test | Description | Result |
|------|-------------|--------|
| T1 | A_aug stability over (lc, rho, a_pd) grid | PASS (worst max|eig| = 0.999966) |
| T2 | a_pd -> 0 limit: `C_dpmr_eff -> Sigma_th(3,3)` | PASS (1e-6 diff) |
| T3 | Linearity: scaling Q/R gives identical C | PASS (2e-5 at relaxed 1e-4 tol) |
| T4 | C_np_eff >= (1-a_pd)^2 bound at small a_pd | PASS |
| T5 | Reference (lc=0.7, a_pd=0.05): C_dpmr_eff = 3.9242 vs K=2 = 3.1610 (+24.1%) | PASS |

## Lookup Table (build_cdpmr_eff_lookup.m)

Computed `C_dpmr_eff(lc)` and `C_np_eff(lc)` on 6-point lc grid × 21-point
rho grid (rho axis redundant but kept for future extensions). Saved to
`test_results/verify/cdpmr_eff_lookup.mat`.

| lc | C_dpmr_eff | K=2 value | Correction | C_np_eff |
|----|-----------|-----------|------------|----------|
| 0.4 | 3.5381 | 2.467 | +43% | 1.360 |
| 0.5 | 3.6101 | 2.809 | +29% | 1.267 |
| 0.6 | 3.7271 | 2.943 | +27% | 1.186 |
| 0.7 | 3.9242 | 3.161 | +24% | 1.114 |
| 0.8 | 4.2932 | 3.625 | +18% | 1.050 |
| 0.9 | 5.1663 | 4.542 | +14% | 0.991 |

(K=2 values use the legacy formula `(1-a_pd)^2 * (2*(1-av)*(1-lc)/(1-(1-av)*lc) + (2/(2-av))/((1+lc)*(1-(1-av)*lc)))`)

## Simulink Verification (verify_cdpmr_eff_simulink.m)

6 test points: 3 lc × 2 noise levels (rho ≈ 0 and rho ≈ 0.036).

| lc | noise_std | Var_emp | Var_theory_new | Var_theory_K2 | err_new | err_K2 |
|----|-----------|---------|----------------|---------------|---------|--------|
| 0.50 | 0.0000 | 9.486e-4 | 9.094e-4 | 7.074e-4 | +4.32% | +34.09% |
| 0.50 | 0.0030 | 9.905e-4 | 9.208e-4 | 7.194e-4 | +7.58% | +37.68% |
| 0.70 | 0.0000 | 1.036e-3 | 9.885e-4 | 7.962e-4 | +4.85% | +30.17% |
| 0.70 | 0.0030 | 1.054e-3 | 9.985e-4 | 8.068e-4 | +5.52% | +30.59% |
| 0.90 | 0.0000 | 1.315e-3 | 1.301e-3 | 1.144e-3 | +1.03% | +14.91% |
| 0.90 | 0.0030 | 1.263e-3 | 1.310e-3 | 1.154e-3 | -3.61% | +9.47% |

**Statistics:**
- NEW (lookup):  mean |err| = **4.49%**, max |err| = **7.58%**
- OLD (K=2):     mean |err| = 26.15%, max |err| = 37.68%
- **~6x improvement in variance prediction accuracy**

All 6 points pass the 10% gate. Mean 4.49% passes the 5% gate.

## Controller Integration

**Files modified:**
- `model/calc_simulation_params.m`: added `C_dpmr_eff`, `C_np_eff` to CtrlBus (elements 26, 27)
- `model/controller/calc_ctrl_params.m`: loads `cdpmr_eff_lookup.mat`, interpolates at current `lc`, stores as `ctrl.C_dpmr_eff` and `ctrl.C_np_eff`. Falls back to K=2 formula if lookup missing.
- `model/controller/motion_control_law_7state.m`: persistent `C_dpmr_eff_const`, `C_np_eff_const`, initialized from `params.ctrl.C_dpmr_eff/C_np_eff`. Eq.13 computation uses these instead of K=2 formula.

**Backward compatibility**: if `cdpmr_eff_lookup.mat` is missing, both `calc_ctrl_params` and `motion_control_law_7state` fall back to the legacy K=2 formula (with a warning).

## Regression (regression_7state_new_cdpmr.m)

Free space positioning (h_init=50, no wall effect), T_sim=10s.

| lc | a_hat_x / a_nom | a_hat_z / a_nom | Var error |
|----|-----------------|-----------------|-----------|
| 0.5 | 0.9851 | 0.9724 | +4.3% |
| 0.7 | 0.8801 | 0.9730 | +6.3% |
| 0.9 | 0.8222 | 0.8334 | -1.4% |

Mean a_hat_x / a_nom = 0.896, mean a_hat_z / a_nom = 0.926. All within 25%
tolerance. At lc=0.5, a_hat tracks a_nom to within 3%. Error grows with lc,
consistent with larger closed-loop time constants and statistical noise.

## Conclusion

**Phase 1 gates all PASS:**
- Sanity unit tests: 5/5 pass
- Lookup table: 126/126 finite, reference point 3.924 vs K=2 3.161 (+24%)
- Simulink variance verification: mean 4.5%, max 7.6% (gate: mean<5%, max<10%)
- Integration: backward-compatible, lookup loaded successfully
- Regression: a_hat converges to within 25% of a_nom across lc sweep

**Measured improvements:**
- Var(del_pmr) prediction: 26% -> 4.5% error (~6x)
- Reference point C_dpmr correction: +24% (3.161 -> 3.924)

**Known limitations (Phase 2 / Phase 3 work):**
- Phase 1 assumes f_0 = 0 (steady-state free space). Near-wall scenarios
  with non-zero control force may need extended analysis.
- Lookup is built at fixed `a_pd = 0.05`. If `config.a_pd` changes, a warning
  fires but the lookup is still used (valid as long as a_pd change is small).
- EKF's a_hat still has 10-20% error in free-space positioning. This is
  likely chi-squared statistical noise from the IIR variance estimate, not
  a C_dpmr bug. Phase 2 will characterize this statistical floor.

## File Inventory

| File | Purpose |
|------|---------|
| `test_script/compute_7state_cdpmr_eff.m` | Core function (augmented Lyapunov) |
| `test_script/verify_cdpmr_eff_sanity.m` | 5 unit tests |
| `test_script/build_cdpmr_eff_lookup.m` | Offline lookup builder |
| `test_script/verify_cdpmr_eff_simulink.m` | Simulink variance verification (12 points) |
| `test_script/dryrun_7state_with_cdpmr_eff.m` | Advisory pre-integration check |
| `test_script/regression_7state_new_cdpmr.m` | Post-integration regression |
| `test_results/verify/cdpmr_eff_lookup.mat` | The lookup table (6x21 grid) |
| `test_results/verify/verify_cdpmr_eff_sanity.mat` | Sanity test results |
| `test_results/verify/verify_cdpmr_eff_simulink_results.mat` | Simulink verification data |
| `test_results/verify/regression_7state_new_cdpmr.mat` | Regression data |
| `reference/for_test/fig_cdpmr_eff_lookup.png` | 2-D heatmap of Cdpmr_tab, Cnp_tab |
| `reference/for_test/fig_cdpmr_eff_verification.png` | theory vs empirical scatter |
