# Phase 9 Stage 0: R(2,2) Validation Report

Generated: 2026-05-04 19:59:39

## Sanity Gates (V1 seed 1)

| Gate | Result |
|---|---|
| max\|delta_a_hat\| < 1e-30 | PASS |
| range(a_hat steady) < 1e-30 | PASS |
| Q77 == 0 everywhere | PASS |
| guards inactive in steady state | PASS |

## Constants

- Ts = 0.000625 s, N_skip = 3200 (t_warmup = 2.0 s)
- IF_var (predicted) = 4.2198
- a_true per axis [x, y, z] = [1.4334e-02, 1.4334e-02, 1.3962e-02] um/pN
- xi_paper per axis [x, y, z] = [6.666e-06, 5.634e-08, 1.900e-04]

## Phase 9 outcomes

Phase 9 Stage 0 has four possible verdicts on the paper-vs-eff comparison (V6):

1. **paper wins** — paper R(2,2) prediction matches V1 closer than eff on informative axes.
2. **eff wins** — per-axis effective C_dpmr/C_n prediction matches V1 closer than paper.
3. **INDISTINGUISHABLE** — at h=50 production conditions, xi_eff/xi_paper ratio is ~1.005-1.008 across all 3 axes (per Agent X audit), so paper and eff predictions differ by less than 1% in R(2,2). |dist_paper - dist_eff| sits below the empirical noise floor and V6 cannot pick a winner. This is the **expected** Wave 1 outcome at h=50.
4. **mixed** — informative axes disagree (rare).

When V6 is INDISTINGUISHABLE and V1-V5 all pass, Path A still applies. The Wave 4 v3 a_hat bias is therefore NOT explained by the paper-vs-eff xi choice and a separate mechanism must be investigated. Reference: `reference/eq17_analysis/phase9_predictions_audit.md`.

## V1 — Variance ratio (5 seeds)

| Axis | emp_var mean | ratio_paper mean +/- SE | pass paper | ratio_eff mean +/- SE | pass eff |
|---|---|---|---|---|---|
| x | 3.5868e-05 | 0.989 +/- 0.023 | PASS | 0.989 +/- 0.023 | PASS |
| y | 3.5534e-05 | 0.984 +/- 0.015 | PASS | 0.984 +/- 0.015 | PASS |
| z | 3.3283e-05 | 0.972 +/- 0.018 | PASS | 0.971 +/- 0.018 | PASS |

## V2 — a_cov linearity

| Axis | slope | pred slope (paper) | slope/pred (paper) | pred slope (eff) | slope/pred (eff) | R^2 | pass paper | pass eff |
|---|---|---|---|---|---|---|---|---|
| x | 5.393e-04 | 7.155e-04 | 0.754 | 7.155e-04 | 0.754 | 0.9810 | FAIL | FAIL |
| y | 5.574e-04 | 7.123e-04 | 0.783 | 7.123e-04 | 0.783 | 0.9859 | FAIL | FAIL |
| z | 5.415e-04 | 6.761e-04 | 0.801 | 6.761e-04 | 0.801 | 0.9882 | FAIL | FAIL |

## V3 — sigma2_n factor sweep

Factors: 0.5, 1, 2, 4. Note: y axis xi_y ~ 0 -> V3 weakly informative on y.

| Axis | informative | max\|ratio_paper-1\| | max\|ratio_eff-1\| | pass paper | pass eff |
|---|---|---|---|---|---|
| x | PASS | 0.026 | 0.026 | PASS | PASS |
| y | FAIL | 0.014 | 0.014 | PASS | PASS |
| z | PASS | 0.030 | 0.031 | PASS | PASS |

## V4 — ACF (lags 0-50, dx_r, V1 seed 1)

| Axis | max\|ACF dev\| (lags 2-51) | IF_var emp / IF_var pred | pass |
|---|---|---|---|
| x | 0.114 | 0.929 | FAIL |
| y | 0.133 | 0.912 | FAIL |
| z | 0.133 | 0.898 | FAIL |

## V5 — DC bias (5-seed aggregate)

| Axis | mean(a_xm) | a_true | bias % | pass |
|---|---|---|---|---|
| x | 1.4281e-02 | 1.4334e-02 | -0.37 | PASS |
| y | 1.4337e-02 | 1.4334e-02 | 0.02 | PASS |
| z | 1.3814e-02 | 1.3962e-02 | -1.06 | PASS |

## V6 — paper vs eff (informative on x, z)

| Axis | informative | dist_paper | dist_eff | |dist_paper-dist_eff| | verdict |
|---|---|---|---|---|---|
| x | PASS | 0.011 | 0.011 | 0.000 | indistinguishable |
| y | PASS | 0.016 | 0.016 | 0.000 | indistinguishable |
| z | PASS | 0.028 | 0.029 | 0.000 | indistinguishable |

Aggregate verdict: INDISTINGUISHABLE — paper/eff difference below noise floor (|dist_paper - dist_eff| <= 0.010) on all informative axes; formula choice has no measurable impact at this operating point.

Note: y-axis xi_y is structurally near zero, so paper and eff predictions are nearly identical for y; y is excluded from V6 verdict.

Reference: Agent X predictions audit at `reference/eq17_analysis/phase9_predictions_audit.md` (xi_eff/xi_paper ~ 1.005-1.008 at h=50 production conditions, predicting V6 INDISTINGUISHABLE).

## Path Verdict: C

  - V2 fails on axis x (slope ratio paper=0.754, eff=0.754, R^2=0.981). Linearity in a_cov broken; suspect IF_var or (a_true+xi)^2.
  - V4 fails on axis x (max ACF dev=0.114, IF_var ratio=0.929). Inspect MA(2) closed-form coeffs vs lambda_c=0.
  - V2 fails on axis y (slope ratio paper=0.783, eff=0.783, R^2=0.986). Linearity in a_cov broken; suspect IF_var or (a_true+xi)^2.
  - V4 fails on axis y (max ACF dev=0.133, IF_var ratio=0.912). Inspect MA(2) closed-form coeffs vs lambda_c=0.
  - V2 fails on axis z (slope ratio paper=0.801, eff=0.801, R^2=0.988). Linearity in a_cov broken; suspect IF_var or (a_true+xi)^2.
  - V4 fails on axis z (max ACF dev=0.133, IF_var ratio=0.898). Inspect MA(2) closed-form coeffs vs lambda_c=0.

### Diagnostic notes (Path C)

Recheck:
- Phase 6 R(2,2) closed-form derivation
- IF_var theoretical vs empirical
- xi_paper / xi_eff sign and scaling with sigma2_n
- a_xm formula constants (C_n*sigma2_n_s subtraction)

