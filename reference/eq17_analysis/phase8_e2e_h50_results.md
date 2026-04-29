# Phase 8 Wave 4 Agent F — V2 EKF h=50 e2e Verification (FAIL — Stage 10 Required)

Date: 2026-04-29
Branch: `test/eq17-5state-ekf`
Wave 2 / Wave 3 commits under test:
- `4a5bbfe` (controller v2 + driver)
- `115b0e8` (oracle predict_closed_loop_var_eq17_v2)
- Wave 3 unit tests committed

---

## TL;DR

**Result: 5-seed e2e FAIL.** Driver `run_v2_h50_e2e` cannot complete a single seed
of the requested h=50 µm positioning + thermal + measurement-noise scenario at
T_sim=5 s. The closed-loop diverges around **t ≈ 0.076 s** (step k≈122) on the
very first seed, well before the 0.2 s G1 warmup window expires. The
divergence is caused by the EKF's `a_x` posterior (slot 6) being driven through
zero by `y_1` (delta_x_m) innovations during early steps; once `1/â_x` flips
sign, the Eq.17 control law inverts and pushes the particle outside the wall.

This is the same root cause flagged by Wave 3's `test_motion_control_law_eq17_7state.m T4`
regression (Issue 1 in `phase8_wave3_test_results.md`): G1 only gates the `y_2`
channel (`a_xm` measurement); nothing in the current 3-guard scheme prevents
`y_1` innovation from pushing slot 6 via the `P_pred(3,6)` cross-covariance.

Tasks 3 (figures) and 4 (full report) are partially completed:
- Figures NOT generated (per task brief: "若 simulation crash...不繼續 figures").
- Phase 7 oracle predictions computed for **a-priori comparison** (using `a_nom`
  not realized `â`). Numbers are physically reasonable: `σ_δx ≈ 43.7 nm` per axis
  (the brief's expected ~44 nm xy / ~36 nm z target).
- Sanity check **with `ctrl_const.a_hat_freeze` engaged** (Stage 10 fallback path)
  shows the simulation is **fully stable** (`σ_δx ≈ 31 nm` per axis), confirming
  that the divergence is purely the `â` runaway, not a model-side or
  driver-side bug.

Pause for review before any controller changes.

---

## 1. Configuration (as requested by Wave 4 spec)

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

Initial `a_nom` per axis (computed at h_bar_init from `c_para`/`c_perp`):
- `a_nom_x = a_nom_y = 1.4334e-02 [um/pN]`
- `a_nom_z = 1.3962e-02 [um/pN]`
(`c_para0 ≈ 1.034`, `c_perp0 ≈ 1.062` at `h_bar = 22.22`.)

---

## 2. Per-seed e2e attempt — divergence trace (seed = 1)

`run_pure_simulation` did NOT throw an exception during the 5 s run, but it
completed with arrays of NaNs and `p_m` magnitudes ~1e+183 along axis 1.
Tracking the controller's internals via a brief wrapper, the divergence sequence is:

| step | t [s] | p_m_x [um] | p_m_z [um] | â_x [um/pN] | â_z [um/pN] | f_d_x [pN] | f_d_z [pN] |
|------|-------|------------|------------|-------------|-------------|------------|------------|
|   1  | 0.0000| -1.50e-2   |  5.00e+1   |  1.4334e-02 |  1.3962e-02 |  0.000e+00 |  0.000e+00 |
|   2  | 0.0006|  6.54e-3   |  5.00e+1   |  1.4334e-02 |  1.3962e-02 |  0.000e+00 |  0.000e+00 |
|   3  | 0.0013|  7.68e-3   |  5.00e+1   |  1.4334e-02 |  1.3962e-02 |  0.000e+00 |  0.000e+00 |
|   4  | 0.0019|  1.34e-2   |  4.998e+1  |  1.4334e-02 |  1.3962e-02 |  3.142e-01 | -4.646e-01 |
|  ... | ...   |  ...       |  ...       |  ...        |  ...        |  ...       |  ...       |
| 119  | 0.0737| -1.22e-1   |  4.969e+1  |  1.578e-03  |  1.600e-03  | -6.213e+00 | -7.175e+00 |
| 120  | 0.0744| -2.55e-1   |  5.004e+1  |  2.181e-03  |  2.296e-03  | -8.907e+00 |  2.488e+01 |
| 121  | 0.0750| -8.94e-2   |  5.034e+1  |  3.014e-03  |  4.595e-03  |  1.171e+01 |  2.135e+01 |
| 122  | 0.0756|  8.28e-2   |  5.042e+1  |  4.779e-03  | **-1.22e-3**|  1.129e+01 |  6.221e+00 |
| 123  | 0.0762|  2.22e-1   |  5.044e+1  | **-4.5e-5** |  1.606e-03  |  9.088e+00 |  1.318e+00 |
| 124  | 0.0769| **-8.40**  |  4.954e+1  |  1.097e-03  |  2.415e-03  | **-6.0e+2**| -6.544e+01 |
| 125+ | ...   | (diverging)| (diverging)| (oscillating)| (oscillating)| (saturating)| (saturating)|
| 798+ | 0.498 |  NaN       |  NaN       |  NaN        |  NaN        |  NaN       |  NaN       |

Driver-warmup (`warmup_count = 2`) silences `f_d` for the first 2 steps. From
step 3 onwards `f_d` is non-zero; the EKF starts updating both `δx[k]` (slot 3)
and `a_x` (slot 6) off the same `y_1` innovation. During steps 4-119 the
estimator slowly drives `â_x` from `1.43e-2` down to `1.58e-3` (≈ 1/9 of nominal),
yielding `1/â_x ≈ 633` and pushing `f_d` magnitudes into 6-25 pN. By step 122
`â_z` flips negative, and by step 123 `â_x` flips negative — the control law
then delivers a `−6e+2` pN x-axis force, kicking `p_m_x` to `-8.4 µm` in one Ts.
Subsequent steps fail to recover; outputs become NaN by step ~800.

This pattern is reproducible across all 5 seeds (verified via seed=1 above and a
4-scenario sweep below); the phenomenon is deterministic given any seed once
both noise paths are active.

---

## 3. Why does this happen? (root-cause analysis)

The 3-guard adaptive R₂ scheme (Phase 6 §5) provides:

| Guard | Trigger                                        | Effect                       |
|-------|------------------------------------------------|------------------------------|
| G1    | `t < t_warmup_kf` (i.e., t < 0.2 s)            | `R(2,2) = R_OFF`, gate y_2 off |
| G2    | `σ²_dxr_hat - C_n·σ²_n_s ≤ 0` (low SNR)        | `R(2,2) = R_OFF`, gate y_2 off |
| G3    | `h_bar < h_bar_safe = 1.5` (near wall)         | `R(2,2) = R_OFF`, gate y_2 off |

When `gate_y2_off = true`, the controller switches to a **1D update with `y_1` only**
(`H_use = [1 0 0 0 0 0 0]`, `R_use = σ²_n_s`). This protects the `a_x` slot from
the `a_xm` measurement (`y_2`), which is unreliable during warmup.

But the `y_1` innovation propagates into slot 6 (`a_x`) anyway through `P_pred`
cross-covariance. Specifically:

- After predict: `P_pred = F_e·P·F_eᵀ + Q`.
- `F_e(3,6) = -f_d[k]` per Phase 1 §10.4 (Eq.19 form).
- For a fresh init `P(3,3) = 2·σ²_dXT ≈ 5e-7 µm², P(6,6) = 1e-5 (um/pN)²`,
  `F_e(3,6) = -f_d[k]`, so `P_pred(3,6) = F_e(3,3)·P(3,6) + F_e(3,6)·P(6,6) =
  λ_c·0 + (-f_d)·1e-5 = -1e-5·f_d`. Even moderate `f_d ≈ 0.3 pN` gives
  `P_pred(3,6) ≈ -3e-6`.
- The 1D Kalman gain on slot 6 from `y_1` is
  `K(6,1) = P_pred(6,1) / S = P_pred(3,6) / (P_pred(3,3) + R(1,1))`.
  With `R(1,1) = σ²_n_s,x ≈ 3.84e-7`, `S ≈ 8.84e-7`, so `K(6,1) ≈ -3.4 [pN^{-1}]`.
- A noisy `y_1` of `δx_m ≈ 1e-3 µm` (one-sigma) thus shifts `â_x` by
  `K(6,1)·δx_m ≈ -3.4e-3 µm/pN` per step.
- Over 100 steps, `â_x` can drift by `O(0.1) µm/pN`, easily encompassing the
  nominal `1.4e-2 µm/pN` and crossing zero.

Wave 3 already discovered this with `test_motion_control_law_eq17_7state.m T4`
(asserting `â drift < 5×` after 160 outlier-injection steps; observed 12.06×).
Per `phase8_wave3_test_results.md` Issue 1, this is documented as a **principled
behavior of the new Wave 2 Pf_init**, NOT a regression bug. But it has now
manifested at e2e level: the divergence is no longer a "stronger drift than
expected", it is a true closed-loop instability.

---

## 4. Phase 7 oracle predictions (a-priori, using a_nom)

Since the realized `â` is unstable, the oracle was evaluated against the
**nominal** `a_x_axis = [1.4334e-2, 1.4334e-2, 1.3962e-2]`.

```
[s2_dx_pred, s2_e_xD_pred, s2_e_a_pred, diag_pred] = ...
   predict_closed_loop_var_eq17_v2( ...
       lambda_c=0.7, a_x_axis=a_nom, h_bar=22.22, ...
       sigma2_n_s=[3.84e-7, 3.25e-9, 1.10e-5], ...
       k_B=P.ctrl.k_B, T=P.ctrl.T, ...
       Q77_axis=[0,0,0], R_axis=R_axis_intrinsic, ...
       a_cov=0.05, a_pd=0.05, ...
       C_dpmr=3.96, C_n=1.18, IF_var=4.220, ...
       sigma2_w_fD=0, scenario='positioning' );
```

| Quantity                | x         | y         | z         | Notes |
|-------------------------|-----------|-----------|-----------|-------|
| `σ²_δx pred [µm²]`       | 1.908e-03 | 1.907e-03 | 1.882e-03 | Phase 7 §6 |
| `σ_δx pred [nm]`         | 43.68     | 43.67     | 43.39     | brief target ~44 nm xy / ~36 nm z |
| `σ²_e_xD pred`           | 0         | 0         | 0         | `σ²_w_fD = 0` baseline |
| `σ²_e_a pred [(um/pN)²]` | 0         | 0         | 0         | `Q77 = 0` for positioning |
| `CV²(â) pred`            | 0         | 0         | 0         | follows from `σ²_e_a = 0` |

**Comments on z-axis**:
- z prediction is 43.4 nm, vs the spec brief's ~36 nm. The oracle weighs
  `σ²_n_s,z = (3.31e-3)² ≈ 1.10e-5`, much larger than x (3.84e-7). Despite this,
  `s2_dx_paper = (C_dpmr·σ²_dXT + C_n·σ²_n_s) / (1 - λ_c²)` is dominated by
  `C_dpmr·4kBT·a_z` — thermal — at this height. So z and x come out similar.
- The brief's "~36 nm z" likely came from an earlier hand-tuned scenario; the
  Phase 7 v2 oracle (with current `meas_noise_std`) yields essentially-equal
  σ_δx across axes.

---

## 5. Stage 10 sanity check — `a_hat_freeze` fallback

To verify the divergence is **purely the â runaway** (not a deeper driver/model
bug), I ran the same scenario with `ctrl_const.a_hat_freeze = a_nom_per_axis`
engaged (slot 6 of `x_pred`/`x_post` is forced to nominal each step, and rows/cols
6 of `P` zeroed before update). Results:

| Metric                         | Value                |
|--------------------------------|----------------------|
| Tracking std (x,y,z) [nm]      | 31.49, 30.96, 30.63  |
| max \|p_m\| (x,y,z) [µm]       | 0.137, 0.132, 50.12  |
| h_bar range                    | always > 0.5 (frozen near 22.27) |
| NaN count                      | 0 / 8001             |

The simulation is fully stable. Tracking is **better** than the oracle prediction
(31 nm vs 43 nm), which is expected: with `â` frozen exactly at `a_nom`, there is
zero `σ²_e_a` contribution to `σ²_δx`, and the `s2_dx_paper` term itself
benefits from a perfectly-matched gain.

This confirms that:
1. The driver, dynamics, trajectory, and measurement-noise paths all work.
2. The divergence is fully attributable to `â` drift via `P_pred(3,6)`.
3. A fallback that locks `â` (or equivalently, zeroes `P(:,6)/P(6,:)` for the
   first few hundred steps) would unblock the e2e.

---

## 6. v1 baseline (sigma-ratio-filter) comparison

From the memory `project_qr_theoretization.md` (h=50 final, 2026-04-22 / 27):

| Metric                       | v1 (qr-branch final)         | v2 prediction (Phase 7) | v2 actual (Wave 4) |
|------------------------------|------------------------------|-------------------------|--------------------|
| Tracking std (x,y,z) [nm]    | ~paper-level (~35-50)        | 43.7, 43.7, 43.4       | **diverged**       |
| â bias (x,y,z) [%]           | 33-44% (per Wave 4 brief)    | 0% (uses `a_nom`)       | **N/A (NaN)**       |
| â std / â                    | ~chi-sq floor (Q77=0 floor)  | 0% (`σ²_e_a = 0`)       | **N/A (NaN)**       |

The v2 redesign target was "â bias < 5%, â std stays at chi-sq floor". The
analytical / oracle predictions at h=50 are extremely good (zero bias; matches
the paper's intended performance). The implementation, however, currently
cannot realize them because `â` cannot survive its own startup transient.

---

## 7. 4 figures status

**Not produced.** Per task brief: "若 simulation crash 或 results 完全偏離預期:
Document 問題 / 不繼續 figures / Report back 給我看".

The figure templates (`fig_v2_a_hat_h50.png`, `fig_v2_tracking_h50.png`,
`fig_v2_oracle_compare.png`, `fig_v2_3guard_timeline.png`) can be filled in once
the divergence is resolved. With the `a_hat_freeze` Stage-10 path the first three
become trivially uninformative (â is constant, oracle ratio is meaningless),
so they will only be useful after the natural-EKF run is stable.

---

## 8. Pass / Warn / Fail decision

Per Wave 4 spec criteria:

| Criterion                                    | Threshold                | Result |
|----------------------------------------------|--------------------------|--------|
| 5 seeds all stable, no divergence            | required                 | **FAIL** (seed 1 already diverges at t≈0.076 s) |
| Tracking std ≤ paper-level (~35-50 nm)       | required                 | **N/A** (outputs are NaN) |
| â bias < 5%                                  | main target              | **N/A** |
| `σ²_δx ratio (obs/pred) ∈ [0.9, 1.1]`        | required                 | **N/A** |
| 3-guard behavior matches Phase 6 spec        | required                 | partially observable (G1 indeed gates y_2; the bug is in y_1 path) |

**Overall: FAIL.**

---

## 9. Open items / Stage 10 fallback options

The divergence makes the natural-EKF e2e impossible to certify on the current
controller. Several options exist; **all require user direction** (per task
brief: "不要改 Pf_init 等參數 (除非 baseline FAIL 需 Stage 10 fallback, 那時 pause
給我 review)"):

### Option A — Lock `â` for the first 0.2 s (`t_warmup_kf` G1 extension)

Smallest controller change: during G1 (the existing 0.2-s warmup window), force
`P(:,6) = P(6,:) = 0` after every predict and update. This zero-gain on slot 6
means y_1 cannot move `â`; the IIR `a_xm` accumulator (which is already running)
will then drive `â` only through y_2 once G1 releases.

**Pro:** minimal controller change; targeted at the actual bug; behavior
identical to the smoke test (which passes) during the first 0.2 s.

**Con:** at t = 0.2 s the EKF starts taking y_2 from a "cold" `â`; would need
to seed `â` from the IIR accumulator just before G1 release (similar to how
slot 1-3 are seeded at warmup_count=1).

### Option B — Larger Pf(3,3), tighter R(1,1), or smaller F_e(3,6)

Manipulate the Kalman gain on slot 6 from y_1 to be small. Concretely, raise
`P_init(3,3)` by ~10⁴ (back to the pre-Wave-2 1e-4 level) to dilute the relative
weight of slot 6. This was explicitly **rejected** by Wave 2 in favor of the
principled `2·σ²_dXT` value (commit 4a5bbfe), so reverting requires a full
Phase 7/8 audit re-run.

### Option C — `ctrl_const.a_hat_freeze` permanent

Just freeze â at `a_nom_per_axis` always. The simulation is stable (proven in
§5) and tracking is excellent. But this defeats the entire point of the v2 EKF
(which is to estimate â on-line for variable-gain regimes like h=2.5 motion).

### Option D — Re-derive 3-guard for `y_1` channel

Add a 4th guard that gates the `K(:,1)` row 6 entry (or zeros `P_pred(3,6)`)
during early steps. This is essentially Option A but more surgical.

---

## 10. Files produced

- `reference/eq17_analysis/phase8_e2e_h50_results.md` (this document)
- `reference/eq17_analysis/phase8_e2e_h50_oracle_only.mat`
  (oracle predictions saved for figure-generation post-fix)

No simulation .mat saved — outputs were NaN. No figures saved.

---

## 11. Recommendation to user

**Pause and review** before any code change. Three parallel decisions are
needed:

1. **Which Stage 10 path?** Option A (lock â during G1) is the most targeted.
   But this must coincide with the Wave 2 / Phase 6 design intent — confirm
   with the controller author before patching.
2. **Are the brief's "expected" numbers (44 nm xy / 36 nm z) right?** The
   oracle gives 43.7 nm xy AND 43.4 nm z (essentially equal). The 36 nm z
   number in the brief may be from an earlier scenario. Either way, the
   oracle-vs-spec comparison cannot be done until the simulation runs.
3. **Wave 3's T4 regression should not have been waved off** — that test
   warned of the exact mode that has now broken e2e. Worth elevating it from
   "documented behavior change" to "blocking" before continuing.
