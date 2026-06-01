# a_hat Quality — Theoretical Prediction (pre-simulation)

**Generated**: 2026-04-18 (before Session 5 simulation)
**Purpose**: Predict a_hat performance for 3 variants from theory, then compare to simulation.

## Theory chain (Task 1d Appendix A)

a_hat std originates from chi-squared variance of IIR-estimated V_meas:

```
Var(V_meas)/V_meas² = chi_sq · autocorr_amp
chi_sq = 2·a_cov / (2 - a_cov)
autocorr_amp ≈ 4 (rigorous: see compute_r22_self_consistent rho_a)

Then:
a_m relative std = sqrt(chi_sq · autocorr_amp) · factor
a_hat std = a_m std / EKF_smoothing_factor
```

## Reference: empirical at lc=0.7, a_cov=0.05

Measured:
- a_hat_z std at h=50: 19.6 ± 1.5%
- a_hat_z std at h=2.5: 20.1 ± 1.5%

This is the chi-sq floor with current `a_cov=0.05`.

## Predictions for Session 5 variants

### Variant 1: empirical (control, a_cov=0.05)
- Predicted a_hat_z std: **19%** (h=50), **20%** (h=2.5)
- Predicted a_hat_z bias: ~0%

### Variant 2: emp_acov005 (a_cov=0.005, only change a_cov)

chi_sq ratio: (2·0.005/1.995) / (2·0.05/1.95) = 0.005012 / 0.05128 = **0.0978** (~10× smaller)

Var(V_meas) reduces ×0.0978 → Std(V_meas) reduces ×sqrt(0.0978) = **×0.313**

If EKF smoothing unchanged, a_hat std reduces by same factor:
- Predicted a_hat_z std at h=50: **19% × 0.313 = 6.0%**
- Predicted a_hat_z std at h=2.5: **20% × 0.313 = 6.3%**

Bias should remain ~0% (no other change).

⚠️ Caveat: smaller a_cov → IIR slower → some lag in V_meas tracking. Positioning is static so lag is benign.

### Variant 3: frozen_correct (Q≈0, Pf_small, a_cov=0.005)

Configuration:
- Q(6,6) = Q(7,7) = 1e-8 (essentially zero process noise on a)
- Pf_init(6,6) = 1e-5 (small initial uncertainty, vs default 2.16e-3)
- a_cov = 0.005

**Initial Kalman gain analysis** (L(6,2) at t=0):
- R(2,2) absolute = 1.0 × sigma2_dXT = 2.52e-4 (um/pN)²
- Pf(6,6)_init = 1e-5
- L(6,2)_init = 1e-5 / (1e-5 + 2.52e-4) = **0.038** (small, no spike expected)

**P(6,6) evolution**:
- After first measurement update: P_post = 0.962 × 1e-5 = 9.6e-6
- Q(6,6) injection per step: 1e-8 → negligible
- P decays slowly → L decreases over time
- After ~100 steps L → ~0.001 (frozen-like)

**a_hat behavior prediction**:
- Stays near init (which is wall-aware = correct a_true)
- Few updates near beginning (L=0.038, small steps)
- Then frozen near correct value
- **Predicted std: ~3-5%** (very low due to frozen + low a_cov)
- **Predicted bias: ~0%** (init is correct, no spike to push it off)

## Predicted summary table

| variant | a_cov | Pf(6,6) init | predicted a_hat_z bias | predicted a_hat_z std |
|---|---|---|---|---|
| empirical | 0.05 | 2.16e-3 | ~0% | ~19% (matches measurement) |
| emp_acov005 | 0.005 | 2.16e-3 | ~0% | **~6%** (chi-sq reduction) |
| frozen_correct | 0.005 | 1e-5 | ~0% | **~3-5%** (frozen + chi-sq) |

## What this would prove (if matched)

1. **chi-sq chain prediction is quantitatively correct** for a_cov sweep
2. **a_hat std 19% is NOT a hard floor** — smaller a_cov breaks it
3. **a_hat std ~6% is achievable** in positioning with derived theory
4. **Paper's 5-10% target is reachable** by simply adjusting a_cov
5. **frozen_correct = ideal positioning Q/R**: small Q + small Pf_init + small a_cov

## Caveats / risks

- Smaller a_cov makes IIR slow. In **dynamic** scenarios (a varies), this causes lag. But for **positioning**, no signal → no lag penalty.
- Smaller Pf_init means EKF "trusts" init blindly. If wall-aware init is wrong, bias appears.
  - At h=50: c_perp(22.2) ≈ 1.0, init a_hat = a_nom is exactly correct.
  - At h=2.5: c_perp(1.11) ≈ 12, init a_hat = a_nom/c_perp ≈ 1.23e-3 (wall-aware).
- Q ≈ 0 means EKF cannot track if a actually changes. For positioning OK.

## To verify

3 variants × 2 scenarios × 3 seeds = 18 runs (3-parallel batches of 6).
After run, compare:
- Predicted bias vs measured bias
- Predicted std vs measured std

If predictions match within ±20%, theory is validated for positioning a_hat quality.
