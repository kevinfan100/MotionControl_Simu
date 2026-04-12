# Phase 2 Report: Statistical Floor Characterization

## Context

Phase 1 fixed the `C_dpmr_eff` bias (27% → 4.5% mean error vs Simulink).
Phase 2 quantifies the remaining RMSE contribution from chi-squared statistical
noise on the Eq.13 variance estimator, and benchmarks current performance vs
the paper's precision.

## Experiment 2A: Chi-squared baseline (fixed a)

**Setup**: 30-second free-space positioning run, lc=0.7, no wall effect, no
measurement noise. 32002-sample steady-state window (after 10s warmup).
Per-step Eq.13 applied with IIR-smoothed variance (`a_cov = 0.05`, N_eff ≈ 20).

### Results: a_m distribution

| axis | mean(a_m)/a_nom | bias | std(a_m) | rel. std |
|------|-----------------|------|----------|----------|
| x    | 89.9%           | -10.1% | 5.83e-3 | **44.1%** |
| y    | 88.4%           | -11.6% | 5.49e-3 | **42.2%** |
| z    | 92.5%           | -7.5%  | 5.93e-3 | **43.6%** |

**Theoretical chi-squared floor** (IIR with `a_cov=0.05`):
`std(a_m)/mean(a_m) ≈ sqrt(2/N_eff) = sqrt(0.1) = 31.6%`

**Observation**: actual rel. std is ~38% higher than the chi-squared(N_eff)
theory (43.6% vs 31.6%). The discrepancy comes from **temporal autocorrelation**:
successive `del_pmr[k]` samples are correlated through the closed-loop dynamics,
so the effective degrees of freedom is smaller than N_eff = 20.

### Window-based sample variance (sanity check for autocorrelation)

| window length | rel. std of Var(del_pmr_z) | chi-sq(wl) theory | excess |
|---------------|----------------------------|-------------------|--------|
| 100 samples   | 32.1%                      | 14.2%             | 2.26x |
| 500           | 16.5%                      | 6.3%              | 2.62x |
| 1000          | 12.5%                      | 4.5%              | 2.79x |
| 2000          | 6.8%                       | 3.2%              | 2.15x |
| 5000          | 2.9%                       | 2.0%              | 1.48x |

**Excess ratio ~2-3x** confirms: each Var estimator effectively uses ~1/4 of
its nominal samples due to autocorrelation. This places a fundamental limit on
how much an IIR smoother can reduce the statistical noise.

### Bias analysis

All three axes show a systematic **bias of -7% to -11%** (a_m below a_nom).
This is smaller than the Phase 1 raw dryrun (which showed similar magnitudes
but opposite signs for the x-axis) and is a residual from:
- The closed-loop dynamics assumption (f_0 = 0) used in the Lyapunov derivation
- EKF non-steady-state transients (the 10s warmup may not fully eliminate
  start-up effects)
- Subtle 3D vs per-axis coupling

## Experiment 2C: Paper-like benchmark

**Setup**: approximately paper Fig 5 configuration — 1 Hz oscillation,
amplitude 2.5 um, descending from h=50 to h=2.5 um, `lc = 0.4` (paper uses
`λc = 0.4`), no measurement noise, wall effect ON, T_sim = 12s.

### Tracking error statistics (mean, std, max)

| axis | mean       | std       | max\|err\| |
|------|------------|-----------|------------|
| x    | +0.020 nm  | 27.6 nm   | 101.4 nm   |
| y    | +0.175 nm  | 27.7 nm   | 119.2 nm   |
| z    | +2.230 nm  | 22.6 nm   | 106.2 nm   |

- **3D RMSE = 41.2 nm**
- **3D max = 134.2 nm**

### del_pm / del_pmr std (for paper Fig 6 comparison)

| axis | std(del_pm) | std(del_pmr) |
|------|-------------|--------------|
| x    | 27.6 nm     | 25.3 nm      |
| y    | 27.7 nm     | 25.4 nm      |
| z    | 26.3 nm     | 20.5 nm      |

### a_hat tracking (z-axis only, since x/y comparison needs c_para reference)

- `|a_hat_z - a_true|/a_true`: median **17.4%**, mean **21.6%**, max 164%

### Comparison with paper precision

Paper Fig 5 shows tracking error "close to random error with zero mean" —
no specific nm numbers quoted, but visually:
- 3D tracking: ~30-50 nm range (our 41 nm is within this)
- a_hat tracking: ~5-10% median error (our 17% is ~2x worse)

**The 41 nm 3D tracking is comparable to the paper**, but the **a_hat estimate
is noticeably noisier** (2x worse than paper).

## Analysis: where is the gap?

The gap between current performance and paper precision comes primarily from
a_hat estimation accuracy, NOT from tracking error. The reason is:

1. The **statistical floor on Eq.13** (a_m from IIR-smoothed variance) is
   ~43% relative std due to chi-squared + autocorrelation. This is a
   fundamental property of single-lag variance estimation.
2. The **7-state EKF smooths a_m** via its own state recursion, reducing
   the effective noise. This brings the a_hat tracking error down to ~17%
   (vs raw 43%).
3. **The paper apparently achieves ~5-10% a_hat precision**, implying either:
   (a) better Eq.13 / separation / statistical method, or
   (b) different Q/R tuning that better extracts info from dynamics channel.

## Decision: Phase 3 is needed

The Phase 1 gate is passed (C_dpmr_eff correct, Var prediction accurate), but
the **statistical floor from Eq.13 is the remaining bottleneck**. To close the
gap to paper precision, we need:

- **Phase 3b (statistical): multi-lag autocorrelation or ML estimation**
  — the most promising direction, since it directly addresses the effective-
  sample-size limitation by utilizing temporal structure of del_pmr

- Optional **Phase 3a (separation): alternative filter** if autocorrelation
  alone doesn't close the gap (e.g., KF innovation or model-based residual)

The plan calls for Phase 3a vs 3b based on diagnostic:
- If bias dominates → Phase 3a (separation)
- If noise dominates → Phase 3b (statistics)
- **Current finding: noise dominates (43% std vs 10% bias). → Phase 3b**

## Files

| File | Purpose |
|------|---------|
| `test_script/phase2_chisquared_mc.m` | Experiment 2A (fixed-a chi-squared) |
| `test_script/phase2_paper_benchmark.m` | Experiment 2C (paper config) |
| `test_results/verify/phase2_chisquared_mc.mat` | 2A data |
| `test_results/verify/phase2_paper_benchmark.mat` | 2C data |
| `reference/for_test/fig_phase2_chisquared.png` | a_m distribution + del_pmr |
| `reference/for_test/fig_phase2_paper_benchmark.png` | tracking error time series |

## Phase 3 Roadmap

**Primary direction (Phase 3b)**: multi-lag autocorrelation of del_pmr.

Theoretical foundation: the 11-state augmented Lyapunov gives the full
Sigma_aug matrix, including off-diagonal elements. `Sigma_aug(3,3)` is
Var(del_pmr) (the 0-lag autocorrelation). `Sigma_aug(3,2)` and higher off-
diagonals give lagged autocorrelations `E[del_pmr[k]*del_pmr[k-L]]`. All
of these are linear functions of `a`. Using L lags simultaneously provides
more information per sample batch, effectively multiplying the sample size
by ~L.

### Phase 3b implementation sketch

1. Derive the lagged covariances from the augmented Lyapunov
2. Sample autocorrelation of del_pmr at multiple lags in the controller
3. Least-squares fit to extract a (over-determined system)
4. Verify via MC: precision improvement vs single-lag IIR
5. Integrate into 7-state EKF as a replacement y2 measurement

### Fall-back (Phase 3a)

If Phase 3b doesn't close the gap, try:
- KF innovation as separation signal (but watch for self-reference)
- Smaller a_cov (longer IIR window, accept more lag)
- Adaptive R22 matched to the chi-squared variance scaling
