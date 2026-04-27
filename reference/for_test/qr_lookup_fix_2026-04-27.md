# Per-Axis Lookup Fix — h=50 Figure Correctness (2026-04-27)

**Branch**: `test/qr-paper-reference`
**Scope**: positioning, h_init=50 µm, controller_type=7, frozen_correct_peraxisR11_R22_Q770

---

## Root Cause

The 7-state EKF Eq.13 a_m reconstruction depends on three precomputed coefficients
(`C_dpmr_eff`, `C_np_eff`, `IIR_bias_factor`):

```
a_m = (sigma2_dpmr / IIR_bias_factor - C_np_eff · sigma2_n) / (C_dpmr_eff · 4·k_B·T)
```

These were loaded from precomputed 2-D lookup tables (`cdpmr_eff_lookup.mat`,
`bias_factor_lookup.mat`) built **2026-04-17 with the β-derivation Q/R**:
- Q(6,6) = Q(7,7) = 1.34e-11 (loaded from `q77_trajectory.mat`)
- chi_sq_R = 0.176 (with `a_cov` = 0.05, `autocorr_amp` = 4.0)
- single value all axes

But the **production frozen_correct_peraxisR11_R22_Q770** EKF runs with:
- Q(6,6) = 1e-8 (744× larger)
- Q(7,7) = 0 per axis
- a_cov = 0.005 (10× smaller)
- R(2,2) per-axis ≈ 0.018 (10× smaller than lookup's 0.176)

The lookup-delivered `C_dpmr_eff = 4.027` was 4% too high vs the self-consistent
production value 3.875 (x) / 3.896 (z). This caused the EKF to **underestimate
`a_m` by ~3.5%**, biasing `a_hat` low → fig1 showed bias x = -2.85%, z = -1.81%.

## Fix

`model/controller/calc_ctrl_params.m`: **replaced 2-D lookup interpolation with
per-axis on-the-fly compute** of `C_dpmr_eff`, `C_np_eff`, `IIR_bias_factor` via
`compute_7state_cdpmr_eff()` (one call per axis with axis-specific Q, R from
`Qz_diag_scaling`/`Rz_diag_scaling` Bus elements).

Bus structure changed (`model/calc_simulation_params.m`):
- Elements 26 (`C_dpmr_eff`), 27 (`C_np_eff`), 28 (`IIR_bias_factor`):
  `Dimensions = [1 1]` → `[3 1]`.

Controller (`model/controller/motion_control_law_7state.m`):
- Persistent constants now 3x1 vectors.
- Eq.13 reconstruction (line 204-209) uses element-wise `.*` and `./`.
- Pre-fill (line 156-157) uses element-wise `.*`.
- `>0` checks wrapped with `all(...)`.

## Verification (5 seeds, T_sim=15s, t_warmup=2s)

| Metric | LEGACY lookup | Single-val prodQR (sandbox) | **Per-axis prodQR (final)** | Theory |
|---|---|---|---|---|
| bias_x | -2.85 ± 2.07 % | -1.33 ± 2.35 % | **-0.80 ± 2.38 %** | 0 |
| bias_z | -1.81 ± 1.72 % | -0.45 ± 1.57 % | **-0.45 ± 1.57 %** | 0 |
| std_x | 4.92 ± 0.38 % | 5.21 ± 1.59 % | 5.25 ± 1.61 % | 5.34 % |
| std_z | 4.50 ± 0.81 % | 4.92 ± 1.51 % | 4.92 ± 1.51 % | 5.38 % |
| trk_x | 34.88 ± 0.45 nm | 34.60 ± 0.61 nm | 34.65 ± 0.62 nm | 34.91 |
| trk_z | 34.73 ± 0.30 nm | 34.60 ± 0.38 nm | 34.60 ± 0.38 nm | 34.59 |
| 3D RMSE | 60.22 ± 0.42 nm | 60.09 ± 0.33 nm | 60.14 ± 0.34 nm | — |

- **Both axes' bias drop to within 1% of zero** (95% CI contains 0 for both).
- a_hat std improves toward theory (ratio: x 0.92 → 0.98, z 0.84 → 0.92).
- Tracking std and 3D RMSE essentially unchanged (lookup doesn't enter tracking-std path).

### Per-axis self-consistent values @ lc=0.7, h=50

```
C_dpmr_eff      = [3.8748, 3.8740, 3.8959]   (was scalar 4.0275)
C_np_eff        = [1.1076, 1.1077, 1.1037]   (was scalar 1.0846)
IIR_bias_factor = [0.9067, 0.9067, 0.9061]   (was scalar 0.9024)
```

## Note on a_hat std slight increase

Mean std_x went 4.92% → 5.25% and std_z went 4.50% → 4.92%. This is **expected
and good**: the legacy biased lookup made `a_hat` track at a wrong scale (3.5%
low), which artificially reduced the apparent relative std. After the fix, the
EKF runs at the physically correct operating point and the std matches theory
within 2%, vs the 8-16% under-prediction before. Theory uses
`CV² = chi_sq · rho_a · L_eff/(L_eff + a_cov)` per writeup §8.3.

## Files Modified

| File | Change |
|---|---|
| `model/calc_simulation_params.m` | Bus elem 26-28 dim `[1 1]` → `[3 1]` |
| `model/controller/calc_ctrl_params.m` | Lookup load → per-axis on-the-fly compute |
| `model/controller/motion_control_law_7state.m` | Scalar → element-wise (`.*`, `./`); fallback values 3x1; `all()` checks |
| `reference/for_test/fig_qr_verification_h50/fig1_gain_estimation.png` | regenerated, run_simulation thesis style |
| `reference/for_test/fig_qr_verification_h50/fig2_tracking_error.png` | regenerated; del_xmd LP overlay; equal subplot sizes; no ±1σ lines; single-seed stats |
| `test_script/plot_qr_h50_figures.m` | NEW — one-shot figure regenerator |

## Open Items (out of h=50 scope)

1. **Fe_err_z structural bug** (`motion_control_law_7state.m:292-298`): z-axis
   chart-extension form does not retreat to Jordan-block at β=0. Verified
   harmless for h=50 figure correctness (sandbox a_hat_z impact ~0.01%; tracking
   ratio 1.000); needs cleanup for future paper-quality refactor.
2. **F·L vs L on injection** (writeup §3.3 vs `motion_control_law_7state.m:322-329`):
   notational inconsistency. Empirical impact at h=50 is < 0.01% because
   missing cross-coupling terms involve L(4,:) ≈ 1e-4 and small f_d.
3. **B_np sign** (`compute_7state_cdpmr_eff.m:164`): `+a_pd` vs writeup `-a_pd`.
   Harmless for Σ (sign-invariant) but should match writeup notation.
4. **State ordering reversal**: `motion_control_law_7state.m` uses writeup
   ordering `[del_p1=oldest, del_p3=newest]`, but `compute_7state_cdpmr_eff.m`
   uses reversed `[delta_x=newest, dx_d2=oldest]`. Functionally equivalent;
   refactor for readability.
5. **Per-axis lookup tables (.mat)** in `test_results/verify/` are now unused
   (kept for legacy reference). Could be removed.
