# Paper Fig 6/9 Comparison Report

**Date**: 2026-05-05  
**Setup**: Q66_physical_mode=true, iir_warmup_mode='prefill', t_warmup_kf=0, calibrated noise, σ²_w_fA=0, σ²_w_a_direct=0, seed=10  
**Branch**: test/eq17-5state-ekf

## Executive Summary

Tested 4 trajectories × 2 λ_c values (= 8 sims) to compare our controller (with physical Q66 derivation) against paper Fig 6/9 expectations. Found **catastrophic KF divergence whenever h_bar < 2 region is entered**, regardless of λ_c. Safe-region trajectories (h_bar > 2.2) achieve paper-grade tracking (~5-8% rstd, 30 nm err). Paper-aligned trajectories fail by 100-100,000× margin.

## Configuration

```
controller_type      = eq17_7state
a_pd                 = 0.05
a_cov                = 0.005
sigma2_w_fA          = 0
sigma2_w_a_direct    = 0
Q66_physical_mode    = true        ← Phase 5 §6.9 NEW physical derivation
cdpmr_method         = lyapunov
iir_warmup_mode      = prefill     ← per feedback memory
t_warmup_kf          = 0           ← per feedback memory
meas_noise_std       = [0.00062; 0.000057; 0.00331] (calibrated)
seed                 = 10
```

R(2,2): IF_eff_per_axis calibrated [3.79, 3.77, 3.66] from phase9_stageI_acf_diagnosis.mat (Phase 9 Stage I X2a active).

## Trajectory definitions

| Tag | Type | h_init | h_bottom | T_sim | Notes |
|---|---|---|---|---|---|
| T1 | ramp_descent | 15 | 2.5 | 10s | Paper Fig 4/6 z-descent (simplified, no x/y sine) |
| T2 | osc | 4.5 | 2.5 | 3.83s | Paper Fig 7/9 near-wall (3Hz, amp=1, n_cycles=9, t_hold=0.2s) |
| T3 | ramp_descent | 50 | 5 | 20s | Our standard ramp (safe region) |
| T4 | positioning | 15 | 15 | 5s | Hold at h=15 (Paper-relevant baseline) |

## Results — 8-condition Summary

```
Trajectory                lc     Cd       bx%       sx%        bz%        sz%      zerr(nm)
T1 paper descent 15→2.5   0.40  2.19   +35.57    13.28    +43098     291003     4294   ★ DIVERGE
T1                        0.70  3.20    +0.59    10.87      +285       1753      163   ★ DIVERGE
T2 paper nearwall osc     0.40  2.19   +83.15    43.01  +2617109    3504867     4155   ★ DIVERGE
T2                        0.70  3.20  +3.9e8   6.8e9   -2919169   11527475      802   ★ DIVERGE
T3 our ramp 50→5          0.40  2.19   +29.23     7.72    +31.21      12.26      28.5
T3                        0.70  3.20    -2.70     4.23     -0.04       7.68      29.8
T4 positioning h=15       0.40  2.19   +27.19     6.51    +31.36      13.69      27.4
T4                        0.70  3.20    +0.53     5.61     +3.11       5.83      29.4
```

## Per-trajectory Findings

### T1 (paper descent 15→2.5)
KF diverges in last ~1s of descent as h crosses h_bar=1.5 (G3 boundary).
Both λ_c values fail. lc=0.7 less dramatic but still 285% bias on a_z, 163 nm tracking.

### T2 (paper near-wall osc dwell at h_bar~1.1-2.0)
Worst case. Whole simulation is in h_bar < 2 region.
lc=0.7 numerically explodes (a_x bias = 4×10⁸%). Total controller breakdown.

### T3 (our standard ramp 50→5, safe region only h_bar > 2.22)
Works as previously shown. lc=0.7 gives bias 0%, std 7.7%, tracking 29.8 nm — paper-grade.
lc=0.4 gives 30% bias (Cd_eff lambda mismatch known issue).

### T4 (positioning h=15, very safe)
Works clean. lc=0.7 gives best result: bias 3.1%, std 5.8%, tracking 29.4 nm.

## Numerical Diagnosis

MATLAB warned `Matrix is close to singular, RCOND ~5e-17` repeatedly during T1/T2 sims (machine ε = 2.2e-16). This is the KF Riccati S = H·P·H' + R inversion failing.

Root cause: Wall correction K_h(h̄) becomes -7.88 at h_bar=1.11. This makes Q66 prefactor (a·K_h/R)² jump 60-fold. P matrix entries get badly scaled (P_66 grows huge, others stay small). S = H·P·H' + R becomes ill-conditioned. S^{-1} amplifies numerical noise by ~10¹⁶, producing garbage state updates → divergence.

## Architectural Gap Identified

**This is NOT a Q tuning issue. It is a KF numerical stability issue at hydrodynamic singularity (h_bar → 1).**

Possible architectural fixes (in order of invasiveness):

1. **Joseph form P-update**: Use `P_post = (I-KH)·P_pred·(I-KH)' + K·R·K'` instead of `(I-KH)·P_pred`. Maintains P symmetric positive-definite under numerical error. Standard KF stability technique.

2. **Square-root KF**: Operate on Cholesky factor U·U' = P instead of P directly. Avoids condition number squaring. More invasive but more robust.

3. **K_h saturation**: Cap |K_h| at some max (e.g., 1.0) when h_bar < threshold. Sacrifices physical accuracy near wall but stabilizes numerics.

4. **Reformulated state-space**: Add h to state vector explicitly, observe h directly (paper Eq.14 may do this — needs verification). Avoids K_h(h̄) appearing in F_e or Q matrix.

## Conclusion

Our derivation of physical Q66 (Phase 5 §6.9 NEW) is conceptually correct but numerically incompatible with paper's near-wall scenarios. The controller works in safe-region (h_bar > 2.2) achieving paper-grade tracking (~30 nm err, ~7% std).

To replicate paper Fig 6/9, architectural changes are needed. Q tuning alone cannot bridge the gap.

## Files

- 8 figures in `reference/eq17_analysis/figures/paper_compare/`
- Diagnosis figure: `Q66_explosion_diagnosis.png`
- This report

## Status

**OPEN**: architectural gap identified, fix path proposed (Joseph form / Square-root KF / K_h cap / reformulate state). User to decide direction.
