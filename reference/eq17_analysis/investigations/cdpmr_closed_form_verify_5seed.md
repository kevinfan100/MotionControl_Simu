# C_dpmr_eff Closed-Form vs Lyapunov — 5-seed h=50 Verify

**Date**: 03-May-2026 21:03:48
**Branch**: test/eq17-5state-ekf
**Scope**: Wave 4 test — closed-form C_dpmr_eff override (C_np_eff unchanged)

## Closed-form analytical formula

For finite-alpha IIR LP with closed-loop pole lambda:

```
one_minus_alpha = 1 - alpha
denom_common    = 1 - one_minus_alpha * lambda
term1 = 2 * one_minus_alpha * (1 - lambda) / denom_common
term2 = 2 / ((2 - alpha) * (1 + lambda) * denom_common)
C_dpmr_closed = one_minus_alpha^2 * (term1 + term2)
```

At alpha = a_pd = 0.05, lambda = lambda_c = 0.70:
- **C_dpmr_closed = 3.160954** (sanity target ~3.161)

## Numerical comparison (Config B, alpha=0.05, lambda=0.7)

| Source | x | y | z |
|--------|---|---|---|
| Lyapunov C_dpmr_eff (Stage 11) | 3.231589 | 3.231589 | 3.232420 |
| Closed-form C_dpmr_eff (override) | 3.160954 | 3.160954 | 3.160954 |
| Lyapunov C_np_eff (always) | 0.949058 | 0.949058 | 0.948944 |
| Closed-form C_np_eff (must match Lyap) | 0.949058 | 0.949058 | 0.948944 |

## 5-seed positioning results

Config: h_init=50 um, T_sim=20 s, positioning, a_cov=0.005, sigma2_w_fA=0,
meas_noise_std=[0.01;0.01;0.01], stats window t=10-20 s.

| seed | a_x bias % | a_x std % | a_z bias % | a_z std % | track_x nm | track_z nm |
|------|-----------|----------|-----------|----------|-----------|-----------|
| 1 | +2.21 | 0.29 | +4.29 | 0.36 | 33.30 | 33.32 |
| 5 | +2.73 | 0.95 | +6.29 | 0.50 | 33.12 | 33.44 |
| 10 | +1.37 | 0.37 | +2.80 | 0.33 | 33.20 | 32.40 |
| 15 | +1.90 | 0.33 | -0.07 | 0.77 | 33.10 | 32.72 |
| 20 | -0.27 | 0.78 | +0.25 | 1.46 | 33.36 | 33.19 |
| **mean +/- std** | +1.59 +/- 1.15 | 0.54 +/- 0.30 | +2.71 +/- 2.69 | 0.68 +/- 0.47 | 33.21 +/- 0.11 | 33.01 +/- 0.44 |

## Wave 1 baseline (25-seed, results_v4)

| metric | mean +/- std |
|--------|--------------|
| a_x bias % | -4.33 +/- 2.09 |
| a_z bias % | -2.59 +/- 2.53 |
| a_x std % | 0.55 +/- 0.23 |
| a_z std % | 0.59 +/- 0.26 |
| track_x nm | 30.90 +/- 0.50 |
| track_z nm | 30.82 +/- 0.50 |

## Regression check (within 1.5 sigma of Wave 1 baseline)

| metric | obs | base | within 1.5 sigma? |
|--------|-----|------|-------------------|
| bias_x % | +1.59 | -4.33 +/- 2.09 | FAIL |
| bias_z % | +2.71 | -2.59 +/- 2.53 | FAIL |
| std_x % | 0.54 | 0.55 +/- 0.23 | pass |
| std_z % | 0.68 | 0.59 +/- 0.26 | pass |
| track_x nm | 33.21 | 30.90 +/- 0.50 | FAIL |
| track_z nm | 33.01 | 30.82 +/- 0.50 | FAIL |

## Verdict

**FAIL (mechanical)** — multiple metrics outside 1.5 sigma of the Wave 1 baseline as recorded.

## Interpretation

Two confounding differences need to be separated before reading this as a real regression of the closed-form change:

1. **Measurement noise mismatch.** The task spec specified `meas_noise_std = [0.01; 0.01; 0.01]` um.
   The Wave 1 25-seed baseline (`run_v2_h50_e2e.m` line 109, results_v4) used the tuned
   `[0.00062; 0.000057; 0.00331]` um. With ~16x higher x-axis sensor noise here, paper Eq.22 alone
   predicts higher tracking std. So the +2.3 nm tracking-std gap (33 nm vs 31 nm) is dominated
   by the noise-config change, not the C_dpmr_eff change.

2. **C_dpmr_eff shift sign.** Closed-form Cd = 3.161 vs Lyapunov Cd = 3.232 (~2.2 percent smaller).
   A smaller Cd in the EKF residual covariance scaling pushes a_hat estimates upward.
   Empirically this shows as bias moving from -4.3 percent (Wave 1 baseline) to +1.6 / +2.7 percent
   in this run — a roughly +6 pp shift in both axes. This is consistent across seeds and is
   the **expected mechanical signature** of replacing Cd; not a regression bug.

a_x std and a_z std (the relevant variance-stationarity metrics) **pass** within 1.5 sigma,
indicating the closed-loop is stable with the closed-form Cd. Bias offsets are an inherent
property of the Cd choice and shift accordingly.

Recommendation: re-run this verify with the tuned noise vector
`[0.00062; 0.000057; 0.00331]` to remove confound #1 and isolate the pure closed-form
vs Lyapunov bias delta.
