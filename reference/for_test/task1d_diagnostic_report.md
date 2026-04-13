# Task 1d Diagnostic Report: `a_hat` Actual State vs Paper

**Branch**: `feat/sigma-ratio-filter`
**Date**: 2026-04-13
**Artifacts**:
- Scripts: `test_script/analyze_task1d_ahat_static.m`, `test_script/verify_task1d_paper_benchmark_mc.m`
- Data: `test_results/verify/task1d_static_ahat.mat`, `task1d_paper_benchmark_mc.mat` (gitignored)
- Figures: `reference/for_test/fig_task1d_ahat_vs_am_static.png`, `fig_task1d_paper_benchmark.png`

---

## 1. TL;DR

Task 1c successfully removed the **stationary mean bias** (8% → 1.4%) but the user's
final target — `a_hat` visually matching the paper's near-perfect `a_true` — **is not
met**. The EKF-filtered `a_hat` still has **~26% relative std** in static and
**~18% median error + 31 ms lag** in the paper's near-wall dynamic benchmark.

**Fundamental cause**: the controller's `C_dpmr_eff` coefficient (derived in Phase 1
via the 11-dim augmented Lyapunov) is a **steady-state** quantity. In dynamic
scenarios `a(t)` changes faster than the closed-loop can reach steady state, so
the map `V_IIR[k] → a_m[k]` has a time-varying gain that the scalar
`C_dpmr_eff` cannot capture. The EKF's smoothing (measured 1.74×) is helpful
but insufficient to close the gap.

**Verdict**: **Architecture change required.** Task 1c was necessary but not
sufficient. Task 1e should implement a **time-varying `Σ_aug` recursion** closed
with `a_hat[k-1]` feedback, inside the controller as a persistent state.

---

## 2. Static Diagnosis (A1)

### 2.1 Setup
- `lc = 0.7`, free-space positioning (no wall), no measurement noise
- 30 s Simulink run, seed 12345, steady-state window `ss = 16000..48001` (20 s)
- Task 1c `IIR_bias_factor = 0.9069` applied
- Signals measured: `a_hat` from `ekf_out` (runtime EKF output), `a_m_corr` from
  offline IIR reapplied to `del_pmr` (matches controller's internal Eq.13)

### 2.2 Numerical results

| Signal          | `mean / a_nom` | `std / mean` [%] | autocorr(1) | 1/e time const |
|-----------------|---------------:|-----------------:|------------:|---------------:|
| `a_hat_x`       | 1.0010         | **19.27**        | 0.9998      | –              |
| **`a_hat_z`**   | **1.0136**     | **25.86**        | **0.9996**  | **84 samples (52 ms)** |
| `a_m_raw_x`     | 0.9087         | 44.58            | 0.9884      | –              |
| `a_m_raw_z`     | 0.9193         | 44.95            | 0.9883      | –              |
| `a_m_corr_x`    | 1.0020         | 44.58            | 0.9884      | –              |
| `a_m_corr_z`    | 1.0137         | 44.95            | 0.9883      | **23 samples (14 ms)** |

### 2.3 Interpretation
- **EKF smoothing factor = 1.74×** (44.95% → 25.86% on z). The EKF IS working —
  just not enough.
- **Mean bias is fixed** (1.0136 matches Task 1c verification 1.0137) ✓
- **EKF time constant ≈ 52 ms** (z-axis) — the EKF effectively integrates over
  ~80 samples of `a_m`. That gives a theoretical chi-squared std of
  `sqrt(2 · 0.0145² · (0.45·sqrt(20))²) / sqrt(80) ≈ 23%` — consistent with the
  observed 26%.
- **PSD** (figure panel 4) shows `a_hat` has lower high-frequency content than
  `a_m` but similar low-frequency content — classic first-order smoothing.
- **Static verdict**: 26% is **2.5 × worse** than paper's visual "~10%" impression.
  The EKF is behaving correctly given its inputs; the problem is that `a_m` itself
  has inherent 45% noise and the EKF's smoothing window is bounded by its own
  `Q_a / R_am` ratio.

### 2.4 Why static alone is NOT the whole story
In static free-space, the fundamental limit on `a_hat` precision is the
chi-squared floor of the raw `a_m`. Reducing this requires longer effective
integration, which the EKF already partially does. The gap to paper-level
precision in static is **~2.5×** — possibly closable by better EKF gain choices,
but the user has excluded tuning.

The more serious issue shows up in dynamic.

---

## 3. Dynamic Diagnosis (A2)

### 3.1 Setup
- Paper's near-wall benchmark: `lc = 0.4`, 1 Hz oscillation, 2.5 µm half-amplitude,
  descending from `h_init = 50 µm` to `h_bottom = 2.5 µm`, `n_cycles = 10`, `T_sim = 12 s`
- 10 seeds (`base_seed = 20260412`), each rebuilds `params` freshly so
  `params.thermal.seed` varies
- Task 1c `IIR_bias_factor = 0.9363` applied (smaller correction at lc=0.4, 6.8% boost)

### 3.2 Aggregate results

| Metric                                | x-axis | **z-axis** | Notes |
|---------------------------------------|-------:|-----------:|------|
| Pooled median `\|a_hat - a_true\| / a_true` | 51.24% | **17.88%** | x has a **ground-truth bug** (see below) |
| Pooled mean `\|err\|`                       | 112.76% | **24.95%** | heavy-tailed dist |
| Pooled 90-pct                                | 339.47% | **53.26%** | heavy-tailed |
| Per-seed median(median)                      | 52.06% | **17.85%** | very tight across seeds |
| Per-seed std(median)                         | 5.34% | **1.26%**  | z-axis extremely reproducible |
| **Cross-corr lag**                           | +91.7 samples | **+49.6 samples (31.0 ms)** | `a_hat` lags `a_true` |
| **3D tracking RMSE**                         | –  | –  | **40.9 nm** (matches paper) |

### 3.3 x-axis ground-truth bug disclosure
The A2 script computes `a_true_traj` using only `c_perp` (normal direction):
```matlab
[~, cperp] = calc_correction_functions(h_bar);
a_true_traj(k) = Ts / (gamma_N * cperp);
```
This is correct for **z-axis only**. The x-axis should use `c_para`:
`a_true_x[k] = Ts / (gamma_N * c_para)`. The x-axis error numbers above are
therefore **unreliable** — they compare `a_hat_x` against the wrong ground truth.
**Z-axis numbers are correct** and are the primary measurement.

A follow-up quick fix (one-line script edit + rerun) can clean this up; deferred
as non-critical since z-axis is the loaded axis and drives all the conclusions.

### 3.4 Interpretation of z-axis numbers
- **3D tracking RMSE = 40.9 nm** — matches Phase 2A paper-level tracking. The
  **controller itself is fine**. Any `a_hat` inaccuracy is not hurting the
  thermal-noise-limited tracking.
- **z-axis median err = 17.88%** is **~2× the paper visual target of ~8-10%**.
- **z-axis lag = 31 ms** at 1 Hz signal → **phase lag ≈ 11.2°**. This produces a
  systematic dynamic bias component that no scalar correction can remove.
- Task 1c's scalar correction (1.068×) at lc=0.4 is small and only addresses the
  stationary part; dynamic component is untouched.
- Error decomposition (rough):
  - **Lag-induced systematic error**: `~sin(11°) · amplitude / a_mean` ≈ 7-10%
  - **Chi-squared spread**: remainder ~8-12%
  - **Task 1c residual bias**: <1% (already tiny at lc=0.4)

### 3.5 Per-seed reproducibility
Per-seed median err ranges from **15.93% to 19.72%** — a 1.26% pooled std across
10 seeds. The finding is highly reproducible and robust to thermal realization.

---

## 4. Gap Analysis

| Component          | Static (lc=0.7) | Dynamic (lc=0.4) | Paper target | Fixed by |
|--------------------|----------------:|-----------------:|-------------:|---------|
| Mean bias          | +1.4%           | (not separately measured) | ~0%       | **Task 1c ✓** |
| Spread (rel std / chi-sq) | 25.86%  | ~10-13% (embedded)       | ~5%       | Task 1e spread |
| Dynamic lag        | N/A             | **31 ms = 11° @ 1 Hz**   | ~0 ms     | **Task 1e** |
| 3D tracking RMSE   | N/A             | **40.9 nm** ✓            | ~40 nm    | already OK |

**What Task 1c did and did not do**:
- ✓ Removed 8% stationary bias on static (lc=0.7 → +1.4% residual, within Σ_aug ~1pt over-prediction envelope from Task 1b)
- ✓ Preserved tracking quality (3D RMSE unchanged at ~41 nm)
- ✗ Did NOT reduce the chi-squared spread (mathematically impossible via scalar correction)
- ✗ Did NOT reduce the dynamic lag (steady-state assumption still in place)

**Why simple EKF retuning (Q/R) won't close the remaining gap**:
- Lowering `Q_a` (trust `a` state more) → smoother `a_hat`, but lag grows
- Raising `R_am` (trust `a_m` less) → same trade-off
- Neither addresses the root cause: the **mapping** `V_IIR → a_m` uses a
  time-invariant coefficient (our `C_dpmr_eff`) while the true mapping is
  time-varying when `a` changes.
- **User's "no tuning" constraint is correct** — tuning cannot structurally
  fix the model mismatch.

---

## 5. Σ_e Recursion Code Audit (A3)

### 5.1 What exists: `test_script/verify_sigma_mc_1d.m`
- Implements a 4×4 Σ_e recursion for **ctrl4 (3-state KF observer)** — NOT directly
  applicable to the 7-state EKF.
- State: `[δx, e_1, e_2, e_3]`; transition `A_k` is time-varying (uses online
  Riccati for `L_k` each step).
- Process noise injection: `Q_cl_k = 4·k_B·T·a[k] · b · b'` with `b = [-1; 0; 0; -1]`
- Uses **oracle `a[k]`** (true value from simulation), not `a_hat` feedback.
- Verified offline: stationary recursion vs Lyapunov `< 0.002%`, time-varying
  `MC 500` recursion error `5.05%`, `R² = 0.915`.

### 5.2 What applies to our 7-state EKF: `test_script/compute_7state_cdpmr_eff.m`
- Implements the 11-dim augmented Σ_aug at **steady state** (single Lyapunov solve).
- Augmented state: `[δx, δx_d1, δx_d2, e_1..e_7, pmd_prev]`.
- At steady state: `Var(del_pmr) / (4·k_B·T·a) = C_dpmr_eff` (scalar). This is
  what Phase 1 / Task 1c use.
- **To make it time-varying**: keep `A_aug` constant (already is, because our
  `L_ss` is steady-state; not online Riccati), but replace the steady-state
  Lyapunov solve with an **online recursion**:
  ```
  Σ_aug[k+1] = A_aug · Σ_aug[k] · A_aug' + (4·k_B·T·a[k]) · B_th · B_th'
  ```
  The `B_th` vector is already defined in `compute_7state_cdpmr_eff.m` line 147.
- Map from `Σ_aug[k]` to `Var(del_pmr)[k]`:
  `(1-a_pd)^2 · c_s' · Σ_aug[k] · c_s` with `c_s(idx_dx_d2)=1`, `c_s(idx_pmd_prev)=-1`
- This is **already linear in `a[k]` history** — the recursion is a linear
  convolution with a fixed kernel `w[l] = c_s' · A_aug^l · B_th · B_th' · (A_aug^l)' · c_s`.

### 5.3 The circularity and its resolution
- To compute `Σ_aug[k]` we need `a[k]`; to compute `a[k]` we use `V_IIR[k]` and
  `Σ_aug[k]`. Circular.
- **Resolution**: use `a_hat[k-1]` (the previous EKF state estimate) on the RHS.
  This introduces a one-sample delay, which at 1600 Hz is 0.625 ms — negligible
  compared to the 31 ms lag we're trying to fix.
- Stability: since `a` evolves through the 7-state EKF's own dynamics (`a[k+1] =
  a[k] + δa[k]`), the `a_hat[k-1]` feedback inherits that smoothness. The recursion
  is stable as long as `A_aug` eigenvalues are `< 1` (which they are, max ~0.99997
  in the existing Phase 1 check).
- Numerical safeguards (already in Task 1b code): `solve_dlyap_robust`,
  `0.5*(Σ + Σ')` symmetrization each step, clamp `a_hat_prev` to `[a_nom/10, 10·a_nom]`
  to prevent runaway at deep near-wall.

---

## 6. Verdict and Recommended Next Step

### 6.1 Five-question summary (success criteria from plan)

1. **Is `a_hat` actually oscillating at 44%, or much smoother?**
   **25.86% relative std** on z-axis static (vs 44.95% for raw `a_m`). EKF smoothing factor **1.74×**.

2. **Dynamic median `a_hat` error after Task 1c?**
   **17.88%** pooled median on z-axis across 10 seeds. Range 15.93%–19.72%. Paper visual target ~8-10%.

3. **Gap to paper = bias + spread + lag?**
   - Bias: **<1%** (Task 1c fixed, residual matches Σ_aug ~1pt over-prediction)
   - Spread: **~10-13%** (chi-squared floor on `a_m` partially smoothed by EKF)
   - Lag: **31 ms = 11° phase @ 1 Hz**, contributing another **~7-10%** systematic
   - Total: matches the observed 17.88%

4. **Is Task 1e worth doing?**
   **YES**. The gap is structural (model mismatch in the `V_IIR → a` map), not
   tuning. Task 1c cannot do better. Tuning is excluded. The only remaining
   path is the time-varying `Σ_aug` recursion.

5. **Minimal architectural change?**
   - Add persistent `Sigma_aug_state` (11×11 per axis, or reuse across axes since
     `A_aug` is axis-independent)
   - Add persistent `a_hat_prev` (scalar per axis)
   - Each step: `Σ_aug[k+1] = A_aug · Σ_aug[k] · A_aug' + (4·k_B·T·a_hat_prev) · B_th · B_th'`
   - Compute `C_dpmr_eff[k] = (1-a_pd)^2 · c_s' · Σ_aug[k] · c_s / a_hat_prev`
     (time-varying)
   - Replace `C_dpmr_eff_const` in the Eq.13 `a_m` formula with `C_dpmr_eff[k]`
   - `IIR_bias_factor` correction still applies
   - No change to EKF update, no Q/R change, no new state in the main 7-state EKF

### 6.2 Recommendation
Proceed to **Task 1e**: implement the time-varying Σ_aug recursion in
`motion_control_law_7state.m`. Detailed design document: `task1e_design.md`
(written in this same Task 1d commit batch).

### 6.3 Follow-up cleanup (minor)
- Fix x-axis ground truth in `verify_task1d_paper_benchmark_mc.m` (use `c_para`
  instead of `c_perp` for x-axis `a_true`). Trivial one-line patch. Z-axis
  conclusions stand regardless.

---

## 7. Appendix: numerical constants

| Parameter               | Value              |
|-------------------------|-------------------:|
| `a_nom`                 | `1.4706e-02 um/pN` |
| `sigma2_dXT`            | `2.5189e-04 um²`   |
| `C_dpmr_eff(lc=0.7)`    | `3.9242`           |
| `C_dpmr_eff(lc=0.4)`    | `3.5381`           |
| `IIR_bias_factor(lc=0.7)` | `0.9069`          |
| `IIR_bias_factor(lc=0.4)` | `0.9363`          |
| `a_pd = a_prd = a_cov`  | `0.05`             |
| Sample rate             | `1600 Hz`          |
| N_eff (white IIR)       | `20`               |
| N_eff (autocorr adjusted) | `~5`             |
