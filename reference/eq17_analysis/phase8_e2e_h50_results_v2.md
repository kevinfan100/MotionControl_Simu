# Phase 8 Wave 4 v2 Agent F2 — V2 EKF h=50 e2e Verification (Stage 10 Option A)

Date: 2026-04-29
Branch: `test/eq17-5state-ekf`
Status: 5-seed CI complete, all seeds stable, no NaN.

Wave 2 / Wave 3 / Stage 10 commits under test:
- `4a5bbfe` (controller v2 + driver)
- `115b0e8` (oracle predict_closed_loop_var_eq17_v2)
- Wave 3 unit tests committed
- **Stage 10 Option A** (uncommitted): G1 K_kf row 6/7 gating (12 lines)

---

## TL;DR

**Result: 5-seed e2e WARN.**

- All 5 seeds run to completion (T_sim = 5 s) with NO NaN, NO divergence.
- Tracking std: x = 29.35 +/- 0.68 nm, y = 28.86 +/- 0.67 nm, z = 28.97 +/- 0.41 nm.
  All three axes are PASS for the tracking_std target (<= 35-50 nm).
- a_hat bias: -28.40% (x), -30.46% (y), -31.14% (z). FAIL for the
  primary "bias < 5%" target. Bias is **systematic** (NOT transient):
  it stays nearly constant from t = 0.5 s to 5.0 s.
- sigma^2_dx ratio (observed / predicted): 0.631, 0.628, 0.644.
  FAIL on the [0.9, 1.1] PASS band; observed sigma^2_dx is ~63% of
  the v2 oracle prediction. Note: observed performance is BETTER
  than predicted (lower variance), so this is a model-side conservatism,
  not a controller failure.
- a_hat std (CV): 2.34% (x), 3.03% (y), 2.16% (z). Well below the
  chi-sq theoretical floor of sqrt(chi_sq) = 22.6%, indicating the
  EKF is operating in a tightly-coupled regime where Q77 = 0
  effectively pins delta_a_hat near zero.

Stage 10 Option A is structurally successful: the closed loop is
stable for the full 5 s window, all gates behave as designed
(G1 active for t < 0.2 s, G2 / G3 never trigger). The tracking
target is met. The bias and oracle ratio are both consequences of
the IIR-derived `a_xm` measurement being calibration-biased at h = 50
(see Section 3); this is the same class of issue documented in
v1's `project_h50_lookup_peraxis_fix.md` and is OUT OF SCOPE for
the Stage 10 closed-loop-stability task.

---

## 1. Configuration

| Field                          | Value                                       |
|--------------------------------|---------------------------------------------|
| `h_init` / `h_bottom` [um]      | 50                                          |
| `amplitude` [um]                | 0                                           |
| `trajectory_type`               | `'positioning'` (hold at h_init)            |
| `T_sim` [s]                     | 5                                           |
| `seeds`                         | [1, 2, 3, 4, 5]                             |
| `t_warmup` (stats window) [s]   | 0.5 (skip)                                  |
| `lambda_c`                      | 0.7                                         |
| `a_pd` / `a_cov`                | 0.05 / 0.05                                 |
| `sigma2_w_fD`                   | 0 (Phase 5 §5.4 baseline)                   |
| `meas_noise_std` (x,y,z) [um]   | [6.2e-4, 5.7e-5, 3.31e-3]                   |
| `meas_noise_enable`             | true                                        |
| `thermal_enable`                | true                                        |
| `controller_type`               | 7 (eq17_7state v2)                          |
| `option`                        | `'A_MA2_full'`                              |
| `t_warmup_kf` (G1)              | 0.2 s (= 320 steps at 1600 Hz)              |
| `h_bar_safe` (G3)               | 1.5                                         |
| `d` (sensor delay)              | 2                                           |
| `h_bar_init`                    | 22.222 (= 50 / 2.25)                        |

Initial `a_nom` per axis (computed at h_bar_init from `c_para` / `c_perp`):
- `a_nom_x = a_nom_y = 1.4334e-02 [um/pN]` (c_para = 1.0260)
- `a_nom_z = 1.3962e-02 [um/pN]` (c_perp = 1.0533)

Stage 10 Option A (uncommitted change in `motion_control_law_eq17_7state.m`,
lines 565-575): during G1 (t < 0.2 s), force `K_kf(6, :) = 0` and
`K_kf(7, :) = 0`. This blocks y_1 leakage to slot 6 (a_x) via the
P_pred(3, 6) cross-covariance during warm-up; without this gate the
controller diverges around step ~122 (Wave 4 root cause, see
phase8_e2e_h50_results.md).

---

## 2. Per-seed raw results

| seed | trk_std x [nm] | trk_std y [nm] | trk_std z [nm] | a_hat mean x [um/pN] | a_hat mean y [um/pN] | a_hat mean z [um/pN] | s2_dx_obs x [um^2] | s2_dx_obs y [um^2] | s2_dx_obs z [um^2] |
|------|----------------|----------------|----------------|----------------------|----------------------|----------------------|--------------------|--------------------|--------------------|
| 1    | 29.78          | 28.93          | 29.08          | 1.0288e-02           | 9.6448e-03           | 9.7857e-03           | 8.870e-04          | 8.372e-04          | 8.454e-04          |
| 2    | 29.99          | 29.54          | 29.00          | 1.0565e-02           | 1.0176e-02           | 9.3980e-03           | 8.991e-04          | 8.728e-04          | 8.410e-04          |
| 3    | 28.88          | 27.76          | 29.14          | 1.0374e-02           | 9.4580e-03           | 9.8127e-03           | 8.338e-04          | 7.704e-04          | 8.493e-04          |
| 4    | 28.39          | 28.88          | 29.35          | 9.5497e-03           | 1.0095e-02           | 9.8538e-03           | 8.063e-04          | 8.338e-04          | 8.617e-04          |
| 5    | 29.69          | 29.18          | 28.27          | 1.0539e-02           | 1.0465e-02           | 9.2189e-03           | 8.813e-04          | 8.513e-04          | 7.991e-04          |

NaN check: 0 NaN values across all seeds, all signals (a_hat, p_m, f_d).

---

## 3. 5-seed aggregate stats

### Tracking std (mean +/- std across 5 seeds, [nm])
| axis | mean +/- std [nm] | target | status |
|------|-------------------|--------|--------|
| x    | 29.35 +/- 0.68    | <= 50  | PASS   |
| y    | 28.86 +/- 0.67    | <= 50  | PASS   |
| z    | 28.97 +/- 0.41    | <= 50  | PASS   |

### a_hat bias (mean +/- std across 5 seeds, [%])
| axis | mean bias +/- std [%] | target | status |
|------|------------------------|--------|--------|
| x    | -28.40 +/- 2.90        | < 5%   | FAIL (bias is systematic) |
| y    | -30.46 +/- 2.86        | < 5%   | FAIL |
| z    | -31.14 +/- 2.05        | < 5%   | FAIL |

### a_hat std (mean across 5 seeds, [um/pN])
| axis | a_hat_std_mean [um/pN] | CV [%] | chi_sq floor (= sqrt(chi_sq) = 22.6%) | status |
|------|------------------------|--------|---------------------------------------|--------|
| x    | 2.4056e-04             | 2.34   | 22.65                                 | well below floor |
| y    | 3.0247e-04             | 3.03   | 22.65                                 | well below floor |
| z    | 2.0761e-04             | 2.16   | 22.65                                 | well below floor |

CV << chi-sq floor because Q77 = 0 (positioning baseline); the EKF
estimates a_x with very low temporal variance (essentially driven
by the slow IIR a_xm via the y_2 channel).

### sigma^2_dx ratio (observed / oracle predicted)
| axis | s2_dx_obs [um^2] | s2_dx_pred [um^2] | ratio | target [0.9, 1.1] | status |
|------|------------------|-------------------|-------|-------------------|--------|
| x    | 8.615e-04        | 1.366e-03         | 0.631 | [0.9, 1.1]        | FAIL   |
| y    | 8.331e-04        | 1.326e-03         | 0.628 | [0.9, 1.1]        | FAIL   |
| z    | 8.393e-04        | 1.304e-03         | 0.644 | [0.9, 1.1]        | FAIL   |

Note: the oracle predicts MORE variance than is observed; performance
is **better** than the closed-form Lyapunov approximation. This is a
model conservatism issue (Phase 7 oracle constants `C_dpmr`, `C_n`,
`IF_var` together give an over-estimate of closed-loop variance for
the v2 architecture at h = 50 positioning), not a controller failure.

For reference, with `a_used = a_nom` (truth) instead of biased
`a_used = a_hat_mean`:
- s2_dx_pred (truth) = [1.908e-3, 1.907e-3, 1.883e-3] um^2 (sigma_dx = 43.7, 43.7, 43.4 nm)
- ratio (obs / oracle_truth) = [0.452, 0.437, 0.446]

So the over-prediction is structural to the oracle, not driven by
the bias in `a_hat`. See Section 5 for further discussion.

---

## 4. ★ 31% bias diagnosis (Task 3)

### 4.1 Sub-window analysis

5-seed bias and tracking-std broken into 4 time windows:

| Window [s]    | x bias mean +/- std [%] | y bias mean +/- std [%] | z bias mean +/- std [%] |
|---------------|--------------------------|--------------------------|--------------------------|
| [0.5, 1.0]    | -34.44 +/- 3.94          | -28.13 +/- 6.00          | -27.91 +/- 6.77          |
| [1.0, 2.0]    | -34.04 +/- 4.26          | -29.35 +/- 5.72          | -29.21 +/- 5.17          |
| [2.0, 3.5]    | -32.89 +/- 2.66          | -29.49 +/- 3.51          | -30.06 +/- 2.20          |
| [3.5, 5.0]    | -32.15 +/- 1.58          | -30.57 +/- 2.38          | -30.05 +/- 2.68          |

| Window [s]    | x trk_std [nm] | y trk_std [nm] | z trk_std [nm] |
|---------------|----------------|----------------|----------------|
| [0.5, 1.0]    | 28.46          | 29.32          | 29.61          |
| [1.0, 2.0]    | 28.09          | 29.22          | 28.85          |
| [2.0, 3.5]    | 29.01          | 28.91          | 28.76          |
| [3.5, 5.0]    | 28.38          | 29.21          | 28.78          |

### 4.2 Conclusion: SYSTEMATIC, not transient

- x-axis bias declines slightly: -34.4% (early) to -32.2% (steady).
  This is a small ~2% drift over 4.5 s; not a transient that resolves
  within the simulation window.
- y/z-axis bias stays nearly flat at -28% to -30% throughout.
- All three axes converge to ~-30% bias by t = 3.5 s; this is the
  steady-state value.
- Tracking std stays stable at ~29 nm in every window; no
  warming-up artifact in the tracking error metric.

The bias is therefore **systematic**, not a Stage-10-induced transient.
A long-time CI run would NOT improve the bias.

### 4.3 Root cause (mechanism)

Diagnostic instrumentation (seed = 1, t >= 0.5 s) reveals:

| axis | a_hat mean [um/pN] | a_xm IIR mean [um/pN] | sigma^2_dxr_hat mean [um^2] | gate_y2_off frac |
|------|--------------------|------------------------|-------------------------------|-------------------|
| x    | 1.0787e-02         | 1.1456e-02             | 7.7764e-04                    | 0.000             |
| y    | 9.9963e-03         | 1.0494e-02             | 7.1196e-04                    | 0.000             |
| z    | 1.0023e-02         | 1.0635e-02             | 7.3438e-04                    | 0.000             |

- `gate_y2_off frac = 0.000` for t >= 0.5 s: G1 / G2 / G3 are all
  released, so the y_2 (a_xm) channel is fully active.
- The IIR-derived `a_xm` measurement is itself biased low: ratio
  `actual sigma^2_dxr_hat / required` = [0.799, 0.732, 0.764].
- For `a_xm = a_nom`, the required `sigma^2_dxr_hat` would be
  ~ [9.73e-4, 9.72e-4, 9.60e-4] um^2 (using `a_xm = (sigma^2 - C_n*sigma^2_n_s) / (C_dpmr * 4 kBT)`).
- Because the closed-loop tracking is BETTER than the v2 oracle
  predicts, the realized `sigma^2_dxr` is smaller than the
  Phase 2 / Phase 6 calibration assumes. This propagates back to a
  biased-low `a_xm`, which the EKF then locks onto (slot-6 update
  via y_2).

This is precisely the same class of issue v1 hit at h = 50 (see
MEMORY index `project_h50_lookup_peraxis_fix.md`): IIR-derived
measurements use offline-computed `C_dpmr`, `IIR_bias_factor`
constants that don't match the realized closed-loop statistics
at all (h, scenario) operating points. v1's solution was to compute
per-axis on-the-fly `C_dpmr` / `IIR_bias_factor` in
`calc_ctrl_params`. v2 has not yet incorporated an analogous fix.

### 4.4 What would resolve the bias

NOT in scope for this task; documented for follow-up:

1. **Per-axis on-the-fly C_dpmr / C_n re-calibration** (mirror v1
   `project_h50_lookup_peraxis_fix.md`): instead of using the
   Phase 2 closed-form `C_dpmr = 2 + 1/(1-lambda_c^2) = 3.96`,
   build_eq17_constants could query a lookup or compute the actual
   `E[delta_x_r^2 / a_x]` for the operating point.
2. **EKF-side bias correction**: derive a multiplicative correction
   factor for `a_xm` based on realized `sigma^2_dxr_hat` vs.
   theoretical `C_dpmr * 4 kBT * a_x`.
3. **Re-derive Phase 7 oracle for v2**: the current oracle predicts
   sigma^2_dx = 1.9e-3 um^2 at h = 50 vs. observed 8.4e-4 um^2; the
   Lyapunov closed form is overstating the closed-loop variance.

---

## 5. Phase 7 oracle bench

The oracle was evaluated using `a_used = aggregate.a_hat_mean_mean`
(realized, biased low) and again with `a_used = a_nom` (truth):

| Quantity                         | x         | y         | z         |
|----------------------------------|-----------|-----------|-----------|
| s2_dx_pred (a = a_hat_real) [um^2] | 1.366e-03 | 1.326e-03 | 1.304e-03 |
| s2_dx_pred (a = a_nom)     [um^2] | 1.908e-03 | 1.907e-03 | 1.883e-03 |
| s2_dx_obs                  [um^2] | 8.615e-04 | 8.331e-04 | 8.393e-04 |
| ratio (obs / pred(real))         | 0.631     | 0.628     | 0.644     |
| ratio (obs / pred(truth))        | 0.452     | 0.437     | 0.446     |

The oracle uses Phase 2 constants (`C_dpmr = 3.96, C_n = 1.18, IF_var = 4.22`)
and the paper closed-form `s2_dx * (1 - lambda_c^2) = C_dpmr * 4 kBT * a_x + C_n * sigma^2_n_s`,
plus Phase 7 estimation-error corrections. Both formulations
over-predict by ~50% relative to the observed closed-loop performance.

### 5.1 PASS/WARN/FAIL determination

| axis | ratio (obs / pred) | band [0.9, 1.1] | band [0.8, 1.2] | result |
|------|---------------------|------------------|------------------|--------|
| x    | 0.631               | NO               | NO               | FAIL   |
| y    | 0.628               | NO               | NO               | FAIL   |
| z    | 0.644               | NO               | NO               | FAIL   |

The oracle fails the [0.9, 1.1] PASS band on all three axes. However,
the failure direction is "observed too small", indicating the v2
controller out-performs the closed-form Lyapunov bound. This is
NOT a stability or controller failure — it indicates the Phase 7
oracle (which uses a paper-Eq.22 form with worst-case approximations
for `E[bracket^2]` and `rho_a`) is over-conservative for the realized
v2 EKF.

---

## 6. Figures

All figures saved to `reference/eq17_analysis/fig_v2_h50/`:

| Figure                                    | Path                                              | Notes |
|-------------------------------------------|---------------------------------------------------|-------|
| `fig_v2_a_hat_h50.png`                    | per-axis a_hat[t], a_xm overlay, +/-1sigma realized | seed = 1 trace; bias inset per panel |
| `fig_v2_tracking_h50.png`                 | per-axis tracking error [nm], +/-1sigma realized + oracle | seed = 1 trace |
| `fig_v2_oracle_compare.png`               | bar chart of obs/pred s2_dx ratio per axis        | PASS/WARN/FAIL bands; all three FAIL low |
| `fig_v2_3guard_timeline.png`              | G1/G2/G3 trigger timeline per axis (3x1 subplot)  | G1 active 0..0.2s; G2/G3 never trigger |

Style: no grid, no title, legend northoutside (CLAUDE.md compliant).

---

## 7. v1 baseline comparison

From MEMORY index `project_qr_theoretization.md` (h = 50 final, 2026-04-22 / 27):

| Metric                       | v1 (qr-branch final, post per-axis fix) | v2 actual (Wave 4 v2 / Stage 10 Option A) |
|------------------------------|------------------------------------------|--------------------------------------------|
| Tracking std (x,y,z) [nm]    | ~paper-level (~35-50)                    | 29.4, 28.9, 29.0                            |
| a_hat bias x  [%]            | -0.80 (post per-axis fix)                | -28.40                                       |
| a_hat bias z  [%]            | -0.45 (post per-axis fix)                | -31.14                                       |
| a_hat std (CV)               | ~chi-sq floor (a_cov = 0.05 → 22.6%)     | 2.16-3.03% (well below)                     |
| Stability                    | stable                                   | stable                                       |

v2 has BETTER tracking std than v1 (~29 nm vs ~35-50 nm) but FAILS
on the bias target (-28 to -31% vs -0.5 to -0.8%). This is because
v2 has not yet had the equivalent of v1's per-axis on-the-fly C_dpmr
/ IIR_bias_factor re-calibration applied (the per-axis on-the-fly
fix is what unlocked v1's <1% bias).

The "low CV" metric (2.2-3.0%) is also a regression from v1 (which
sat near the chi-sq floor of 22.6%): with Q77 = 0, the v2 EKF locks
slot 6 essentially to the IIR a_xm trajectory, which is itself
LP-filtered. This is consistent with the v2 design intent (positioning
should give zero a_hat variance), but combined with the biased a_xm
it means a_hat is biased AND tightly tracked at a wrong value.

---

## 8. PASS / WARN / FAIL conclusion

| Criterion                                    | Threshold                | Result |
|----------------------------------------------|--------------------------|--------|
| 5 seeds all stable, no divergence            | required                 | **PASS** |
| Tracking std three axes <= 35-50 nm          | required                 | **PASS** (29.4 / 28.9 / 29.0 nm) |
| a_hat bias < 5%                              | main target              | **FAIL** (-28% to -31%, systematic) |
| sigma^2_dx ratio (obs/pred) in [0.9, 1.1]    | required                 | **FAIL** (0.628 - 0.644, observed too small) |
| 3-guard behavior matches Phase 6 spec        | required                 | **PASS** (G1 0..0.2s, G2/G3 inactive) |
| 31% bias diagnosed transient or systematic   | required                 | **SYSTEMATIC** |

**Overall: WARN.**

- Stability and tracking targets achieved (the main Stage 10 goal).
- Bias / oracle ratio failures are ROOT-CAUSED to IIR-derived `a_xm`
  calibration mismatch (NOT a Stage 10 issue, NOT a controller
  failure). They are documented for downstream resolution.
- Per task brief criteria: WARN bracket is "paper-level acceptable
  but not perfect", which fits — v2 over-performs paper-level
  tracking, but the EKF-internal a_x estimate is biased.

---

## 9. Open items (post-Stage-10 follow-up)

1. **Bias remediation** — port v1's per-axis on-the-fly C_dpmr /
   IIR_bias_factor re-calibration into `build_eq17_constants` /
   `calc_ctrl_params`. Reference: `project_h50_lookup_peraxis_fix.md`.

2. **Oracle re-calibration** — Phase 7 closed-form (paper Eq.22 +
   Lyapunov chain) over-predicts sigma^2_dx by ~50%. Either:
   - tighten the rho_a default (currently 4.7), or
   - reformulate `E[bracket^2]` for v2 architecture (currently
     `(1 - lambda_c)^2 * (s2_dx_paper + sigma^2_n_s)` which is an
     upper bound), or
   - add a v2-specific empirical correction factor.

3. **Stage 10 Option A commit** — uncommitted 12-line change in
   `motion_control_law_eq17_7state.m` (lines 565-575). Stable across
   5 seeds; ready for commit pending user review.

4. **CV << chi-sq floor** — a_hat CV is ~2.5% vs. theoretical
   chi-sq floor 22.6% (with rho_a = 4.7). Indicates the closed-form
   chi-sq chain estimate also over-predicts uncertainty for v2.
   Worth re-deriving for the realized Q77 = 0 case (where L_eff = 0
   and CV^2(a) = 0 exactly per Phase 7 §4.3.1; observed CV is just
   sample-noise from finite-window stats, not estimation error).

5. **Wave 3 T4 regression** (drift > 5x in 160-step outlier injection)
   was the original "warning" for the Wave 4 e2e divergence.
   Stage 10 Option A resolves the e2e divergence but does not change
   the T4 regression itself; T4 should be re-evaluated once Stage 10
   is committed (the test asserts on raw, unguarded EKF behavior;
   may need updated assertions or a "Stage-10-aware" mode).

---

## 10. Files produced

- `reference/eq17_analysis/phase8_e2e_h50_results_v2.md` (this document)
- `reference/eq17_analysis/phase8_e2e_h50_results_v2.mat` (raw 5-seed results + sub-window analysis + seed=1 simOut)
- `reference/eq17_analysis/phase8_e2e_h50_diag_seed1.mat` (instrumented seed=1 diagnostic with a_xm, sigma2_dxr, gate flags)
- `reference/eq17_analysis/fig_v2_h50/fig_v2_a_hat_h50.png`
- `reference/eq17_analysis/fig_v2_h50/fig_v2_tracking_h50.png`
- `reference/eq17_analysis/fig_v2_h50/fig_v2_oracle_compare.png`
- `reference/eq17_analysis/fig_v2_h50/fig_v2_3guard_timeline.png`

Stage 10 Option A code change (uncommitted) is in
`model/controller/motion_control_law_eq17_7state.m` lines 565-575.

---

## 11. Recommendation to user

**Pause for review.** The 5-seed e2e run is structurally healthy
(Stage 10 Option A succeeds at unblocking divergence), but two
follow-up actions are needed before the v2 architecture is
"paper-ready":

1. Decide whether to commit Stage 10 Option A as-is. The 12-line gate
   is targeted, well-commented, and verified across 5 seeds.

2. Decide whether the bias / oracle ratio failures are blockers for
   Wave 4 closure or open items for a separate "v2 calibration"
   phase. The bias is identical in mechanism to v1's pre-fix h = 50
   bias; v1 took a separate per-axis on-the-fly fix to reach <1%.
   Replicating that fix for v2 is its own task.

No Stage 10 fallback is recommended at this time. The closed-loop
is fully stable; the issues uncovered are calibration, not stability.
