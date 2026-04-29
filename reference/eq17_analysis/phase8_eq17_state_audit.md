# Phase 8 — v1 vs v2 State Audit

**Branch**: `test/eq17-5state-ekf` (HEAD = 4843e6f, post Phase 7 docs)
**Date**: 2026-04-29
**Status**: Read-only audit. No code changes. v2 implementation deferred to Phase 8 task.

This document audits the current v1 Eq.17 7-state EKF implementation against the Phase 1-7 v2 spec. Each file/area is examined for what is present vs what v2 needs, with a summary diff table at the end.

---

## A. `model/controller/motion_control_law_eq17_7state.m`

### A.1 Current control law (lines 240-246)

```matlab
one_minus_lc = 1 - lambda_c;
f_d = (1 ./ a_hat) .* ( ...
        pd_kp1 ...
      - lambda_c * pd ...
      - one_minus_lc * pd_km_d ...
      + one_minus_lc * delta_x_m ...
      - xD_hat_for_ctrl );
```

This implements:
```
f_d[k] = (1/â_x) · { p_d[k+1] − λ_c·p_d[k] − (1−λ_c)·p_d[k−d] + (1−λ_c)·δx_m[k] − x̂_D[k] }
```

### A.2 Diff vs v2 spec

| Aspect | v1 (current) | v2 (paper Eq.17, Phase 1 §2.3) | Diff |
|---|---|---|---|
| Σ_{i=1..d} f_d[k−i] term | **MISSING** | **Required**: `−(1−λ_c)·Σ f_d[k−i]` outside `(1/â_x)` bracket | **medium-high** |
| `x̂_D` placement | Inside bracket (divided by `â_x`) | **Outside** bracket: `−x̂_D / â_x` (Phase 0 v2: `x̂_D` additive, separate from bracket) | low (algebraically equivalent: both divide by â_x once; Phase 0 spec has `x̂_D[k]/â_x[k]` outside the bracket as a separate adaptive term) |
| `f_d[k−i]` history buffer | **NOT PERSISTENT** | Need `f_d_km1`, `f_d_km2` | **add** |
| F_e(3,4) value | Constant `−1` (`build_F_e` line 537) | **`−1.6`** (Eq.19 form, d=2, λ_c=0.7 → `−(1+d(1−λ_c))`) | **medium** |
| F_e(3,1) (Eq.19 default) | 0 | 0 | OK |
| F_e(3,3) (Eq.19 default) | `λ_c` | `λ_c` | OK |
| F_e(3,6) | `−f_d[k]` | `−f_d[k]` | OK |
| F_e form switch | `eq19` default with `eq18` flag | Eq.19 chosen for Phase 1 baseline | OK |

### A.3 Q matrix construction (lines 290-315)

| Entry | v1 | v2 spec (Phase 5) | Diff |
|---|---|---|---|
| Q11, Q22, Q44, Q66 | 0 | 0 | OK |
| Q33 | `4·kBT·â_x` (Path C strict) per axis | `4·kBT·â_x` (Path C strict) per axis | OK ✓ identical |
| Q55 | **0 (hard-coded)** | `a_x²·σ²_w_fD` (default 0) — needs config field | **add config knob** |
| Q77 | `Δt⁴·â_x²·{Term A + Term B}` per axis with K_h, K_h' | Same closed form | OK ✓ identical |

Q matrix is fully **closed form**, no lookup table (good — already matches v2).

### A.4 R matrix construction (lines 317-342)

| Entry | v1 | v2 spec (Phase 6) | Diff |
|---|---|---|---|
| R(1,1) | `σ²_n_s,i` per axis | `σ²_n_s,i` per axis | OK |
| R₂_intrinsic | `a_cov · IF_var · (â_x + ξ)²` | Same | OK |
| ξ per-axis | Computed offline once in `build_eq17_constants` (`ctrl_const.xi_per_axis`) | Phase 6 §4: same closed form `(C_n/C_dpmr)·σ²_n_s/(4kBT)` | OK ✓ |
| Delay propagation | `delay_R2_factor · Q77` (= `5·Q77` for d=2) | `5·Q77` for d=2 | OK ✓ |
| 3-guard logic | G1 warm-up + G2 NaN + G3 wall (line 326-335 + 395-398) | Phase 6 §5: same | OK ✓ |
| `R_OFF` value | `1e10` (line 181) | `1e10` (Phase 6 §5) | OK ✓ |

ξ is computed **once** at init, not per-step. Per Phase 6, ξ depends only on σ²_n_s and constants, so this is correct (no per-step recomputation needed).

### A.5 Persistent state inventory (lines 80-105)

Currently persistent:
- `x_e_per_axis` (7×3 EKF state)
- `P_per_axis` (cell{3} of 7×7 covariance)
- `dx_bar_m`, `sigma2_dxr_hat` (IIR states)
- `pd_km1`, `pd_km2` (trajectory delay buffers)
- `k_step` (step counter)
- Many cached scalars/vectors

**Missing for v2**:
- `f_d_km1`, `f_d_km2` (persistent past control buffer for `Σ f_d[k−i]`)
- `warmup_count` — note: `t_warmup_kf` is used but no integer `warmup_count` step counter

### A.6 Per-axis EKF update (lines 375-447)

- Predict + Update sequential (1D updates when y_2 gated, 2D otherwise) — sound design
- F_e built fresh per axis with `build_F_e(lambda_c, f_d(ax), use_eq18)` — only F_e(3,6) time-varying, F_e(3,4) is hard-coded constant `−1` (Eq.19 default branch line 537) which **needs to become `−1.6`** for v2.
- `a_hat_freeze` override and `suppress_xD` option exist (added 89e6521 / 757b94b for diagnostics).

### A.7 `suppress_xD` flag

Added in commit 89e6521 (line 200-204):
```matlab
if isfield(ctrl_const, 'suppress_xD') && ctrl_const.suppress_xD
    xD_hat_for_ctrl = zeros(3, 1);
else
    xD_hat_for_ctrl = xD_hat;
end
```

This is a v1 diagnostic flag. v2 will keep `x̂_D` always active by spec; this flag can stay for testing but is not part of the v2 contract.

---

## B. `model/controller/build_eq17_constants.m`

### B.1 Current ctrl_const fields

```
lambda_c, C_dpmr, C_n, option, IF_var, xi_per_axis, delay_R2_factor,
t_warmup_kf, h_bar_safe, d, a_cov, meta
```

### B.2 Diff vs v2 spec

| Field | Phase 2/6 spec | v1 status | v2 needs |
|---|---|---|---|
| `C_dpmr` | `2 + 1/(1−λ_c²)` ≈ 3.961 | ✓ computed (line 137) | OK — Phase 2 §6 confirms VALUES preserved from v1 |
| `C_n` | `2/(1+λ_c)` ≈ 1.176 | ✓ computed (line 140) | OK ✓ |
| `IF_var` | MA(2)-full (default) ≈ 4.224 | ✓ computed Option A_MA2_full (line 146-167) | OK ✓ |
| `xi_per_axis` | `(C_n/C_dpmr)·σ²_n_s/(4kBT)` | ✓ computed (line 177) | OK ✓ |
| `delay_R2_factor` | `Σ_{j=1..d}(d−j+1)²` (=5 for d=2) | ✓ computed via loop (line 182-187) | OK ✓ |
| `t_warmup_kf`, `h_bar_safe`, `d`, `a_cov` | Defaults / passthrough | ✓ all present | OK ✓ |
| **`sigma2_w_fD`** | New for v2 (Phase 5 §5.4 baseline 0) | **MISSING** | **Add** as opts field passthrough → ctrl_const for Q55 closed form |

### B.3 Phase 2 conclusion (already audited)

Phase 2 §6 explicitly states "C_dpmr/C_n VALUES were always right; only the controller had to be fixed". So `build_eq17_constants` is **structurally complete for v2** — only sigma2_w_fD needs to be added.

---

## C. `model/pure_matlab/run_pure_simulation.m` (driver)

### C.1 Sensor delay buffer (lines 101-108)

```matlab
d_delay = ctrl_const.d;          % = 2
p_m_buffer = repmat(p0, 1, d_delay);  % [3 x d] — length 2
```

Current: buffer of length 2.

### C.2 Effective delay analysis

Looking at the loop (lines 132-188):
1. (a) Get `pd[k+1]` from trajectory.
2. (b) `pd_k = pd_for_ctrl` — this is `pd[k]` (came from previous loop's `pd_kp1`).
3. (c) `p_m_delayed = p_m_buffer(:, 1)` — reads HEAD of buffer.
4. (d) Controller called with `(del_pd, pd_k, p_m_delayed)`.
5. ...
6. (i) After dynamics, append new `p_m_raw` to TAIL.

Within the controller, `pd_km2` holds `p_d[k−2]` (Per A.5: trajectory delay buffer in motion_control_law_eq17_7state itself shifts internally).

The 2026-04-29 diagnostic note in the file (lines 105-108) explicitly flags:
> "buffer length d effectively implements d-1 delay; tentative fix was buffer length d+1 but caused full-driver crash"

**v2 needs**: buffer length d+1 (=3 for d=2) with proper `pd_km` synchronization (the controller's internal `pd_km1`/`pd_km2` buffers must align with the driver's `p_m_buffer` semantics). Phase 1 spec assumes "δx_m[k] = δx[k−d] + n_x[k]" where d=2, requiring TWO-step delay between sensor read and current step.

### C.3 5-seed CI / batch framework

Search across `test_script/` shows **no 5-seed CI loop** in v1 driver path. Phase 8 verification will need a script that wraps `run_pure_simulation` in a seed sweep for std/mean reporting.

### C.4 Other gates / hooks

| Aspect | v1 | v2 |
|---|---|---|
| `thermal_enable` gate | At driver level, line 151-155 | OK — keep |
| `meas_noise_enable` gate | At driver level, line 164-168 | OK — keep |
| RNG seeding | `rng(opts.seed)` once at top (line 64) | OK — keep |
| `eq17_opts` build | Hard-coded `option='A_MA2_full'`, `t_warmup_kf=0.2`, `h_bar_safe=1.5`, `d=2` | OK; need passthrough for new `sigma2_w_fD` |

### C.5 Comparison vs `run_simulation.m` (Simulink track)

Per Bash diff: `run_simulation.m` on this branch differs from `feat/sigma-ratio-filter` by `h_min` value only (1.1 → 1.5 R for h_bar_safe alignment). The Simulink-driver path is for `controller_type=7`/`23`, NOT `eq17_7state`. **All eq17 work must use `run_pure_simulation.m`**.

---

## D. `model/pure_matlab/step_dynamics.m`

### D.1 Inner step

```matlab
inner_step_target = 10e-6;                         % [sec] design target
N_inner = max(1, round(Ts / inner_step_target));   % integer substeps
```

For Ts = 1/1600 ≈ 625 µs and target 10 µs → N_inner = 63 (so actual h = Ts/63 ≈ 9.92 µs).

### D.2 Diff vs Simulink

| Aspect | Pure-MATLAB (v1) | Simulink (qr branch) |
|---|---|---|
| Solver | ode4 (RK4) classical | ode4 |
| Inner step | 10 µs | 1 µs (Simulink Solver step 1e-6) |
| ZOH | `F_total` constant within Ts (line 38-43) | ZOH block at 1/1600 |

Decision in `agent_docs/dual-track-simulation-design.md` decision 3: 10 µs chosen for ode4 identity at 10× larger step. RMS error vs Simulink expected < 1e-10 for typical traj. **No change needed for v2** — this is integration accuracy, not Eq.17-specific.

---

## E. `model/config/user_config.m`

### E.1 Current EKF-relevant fields

```
ctrl_enable, lambda_c, controller_type=23,
meas_noise_enable=false, meas_noise_std=[0.01;0.01;0.01],
a_pd=0.05, a_prd=0.05, a_cov=0.05, epsilon=0.01,
beta=0.5, lamdaF=1.0,
Pf_init_diag = [0; 0; 1e-4; 1e-4; 0; 10*(0.0147)^2; 0],   % 7x1
Qz_diag_scaling = [0; 0; 1e4; 1e-1; 0; 1e-4; 0],            % 7x1
Rz_diag_scaling = [1e-2; 1e0],                              % 2x1
thermal_enable=true, T_sim=5
```

### E.2 Diff vs v2

| Field | v1 | v2 needs |
|---|---|---|
| `meas_noise_std` per axis | All 0.01 µm | **Per Phase 6 §3.2: x=0.62 nm, y=0.057 nm, z=3.31 nm** (paper-spec sensor values) — need updated default OR keep current and override per-test | likely keep current default, override in test scripts |
| `Pf_init_diag` (7-state) | `[0; 0; 1e-4; 1e-4; 0; 10·(0.0147)²; 0]` | Phase 5 doesn't constrain explicitly; v1's choice is fine |
| `Qz_diag_scaling` | `[0; 0; 1e4; 1e-1; 0; 1e-4; 0]` (7×1) | NOT used by `motion_control_law_eq17_7state` — that controller builds Q from closed form (Phase 5). This field is for legacy `_7state` controller (not eq17). |
| `Rz_diag_scaling` | `[1e-2; 1e0]` (2×1) | NOT used by eq17 controller. Same legacy point. |
| **`sigma2_w_fD`** | **MISSING** | **Add** for Q55 closed form (Phase 5 §5.4); default 0 |
| `lambda_c` | 0.7 | OK |
| `a_cov` | 0.05 | OK |
| `a_pd`, `a_prd` | 0.05 each | OK (both used by IIR; eq17 controller uses `a_var = a_cov` per design.md §6) |

### E.3 Bottom line

`Qz_diag_scaling` and `Rz_diag_scaling` are **dead code** for the eq17 path (the controller computes Q/R online from physics). They are still passed through `calc_ctrl_params` because the legacy `motion_control_law_7state` uses them. v2 leaves them alone.

The only **must-add** field is `sigma2_w_fD`.

---

## F. `model/calc_simulation_params.m` + `calc_ctrl_params.m`

### F.1 Diff vs `feat/sigma-ratio-filter`

Per `git diff feat/sigma-ratio-filter..HEAD`:

| Element | sigma-ratio-filter | test/eq17-5state-ekf | Note |
|---|---|---|---|
| CtrlBus length | 25 elements | 22 elements | qr branch had 3 extra: `kf_R`, `kf_L`, `lambda_e`, plus `Qz_diag_scaling` 9×1, `Rz_diag_scaling` 6×1, `C_dpmr_eff`, `C_np_eff`, `IIR_bias_factor` (per-axis lookup outputs) |
| Bus elements removed in eq17 branch | (kept in qr) `lambda_e`, `kf_R`, `kf_L`, `C_dpmr_eff/C_np_eff/IIR_bias_factor` | Removed | These were qr-branch additions for per-axis lookup; eq17 path doesn't use them |
| `Qz_diag_scaling` dim | 9×1 (qr per-axis) | **7×1** (eq17 single) | eq17 uses 7-state, not per-axis vector form |
| `Rz_diag_scaling` dim | 6×1 (qr per-axis) | **2×1** (eq17 single) | same |

### F.2 Bus elements present (eq17 branch)

22 fields in CtrlBus including `enable`, `lambda_c`, `gamma`, `Ts`, `meas_noise_*`, `a_pd/prd/cov`, `epsilon`, `k_B`, `T`, `sigma2_deltaXT`, `g_cov`, `controller_type`, `beta`, `lamdaF`, `sigma2_noise`, `Pf_init_diag` (7×1), `Qz_diag_scaling` (7×1), `Rz_diag_scaling` (2×1).

### F.3 v2 needs

- **Add** `sigma2_w_fD` to CtrlBus (1×1, double, default 0). Bus length 22 → 23.
- `calc_ctrl_params.m` needs to read `config.sigma2_w_fD` and pass through to `ctrl.sigma2_w_fD`.
- `build_eq17_constants` needs to consume opts.sigma2_w_fD and store in ctrl_const.
- `motion_control_law_eq17_7state` needs to use `ctrl_const.sigma2_w_fD` to set `Q55,i = a_hat_i² · sigma2_w_fD` instead of hard-coded 0.

---

## G. Lookup tables usage

### G.1 Search results

```
.mat files in model/:               none (only test_results/ and slprj/)
load(...mat) calls in model/*.m:    none
cdpmr_eff_lookup / bias_factor_lookup references in model/*.m:  none
```

### G.2 Conclusion

The active eq17 controller path (motion_control_law_eq17_7state, build_eq17_constants, calc_ctrl_params) **does not load any .mat lookup**. C_dpmr, C_n, IF_var, ξ are all computed in closed form from λ_c and σ²_n_s/kBT. K_h, K_h' use `calc_correction_functions.m` (Phase 4a function), not a lookup.

**v2 status**: ✓ already lookup-free, matches Phase 2 + Phase 6 spec.

The `cdpmr_eff_lookup.mat` / `bias_factor_lookup.mat` files in `test_results/verify/` are artifacts from the qr branch's per-axis investigation (per MEMORY index `project_h50_lookup_peraxis_fix.md`). They are NOT in the load path of the eq17 controller.

---

## H. Unit tests current state

### H.1 Existing tests in `test_script/unit_tests/`

| Test | Coverage |
|---|---|
| `test_calc_correction_functions.m` | K_h, K_h' formulas (Phase 4a) |
| `test_step_dynamics.m` | RK4 integrator |
| `test_build_eq17_constants.m` | `ctrl_const` field validity (Phase 4b) |
| `test_motion_control_law_eq17_7state.m` | T1-T6 smoke + warmup gate + steady-state (Phase 4c) |
| `test_predict_closed_loop_var.m` | Lyapunov diagnostic helper (T1-T4 — Eq.18 form g-sweep) |

### H.2 v2 priority tests MISSING

1. **delay_R2_factor d-sweep** — verify `delay_R2_factor(d=1)=1`, `(d=2)=5`, `(d=3)=14`. Phase 6 §4.3 lists these explicitly. Probably trivial to add to `test_build_eq17_constants`.
2. **F_e(3,4) value at λ_c=0.7, d=2** — verify the v2 update `−(1+d(1−λ_c)) = −1.6`. Critical regression test.
3. **F_e time-varying (3,6) entry** — verify F_e(3,6) refreshes as `−f_d[k]` each step.
4. **Q55 closed form** — verify `Q55 = a_x²·σ²_w_fD` (and =0 at baseline).
5. **Σf_d term in control law** — verify `f_d[k]` matches paper Eq.17 with non-zero past `f_d_km1`, `f_d_km2`.
6. **Sensor delay end-to-end** — sweep `d_delay` and verify simulated tracking error matches Phase 6 §4.3 formula.

### H.3 Diagnostic-only files (89e6521, 757b94b commits)

- `model/diag/predict_closed_loop_var.m` — Lyapunov bench helper for v1 Eq.18 g-sweep (used by test T1-T4). Useful as baseline; v2 needs a separate `predict_closed_loop_var_eq17_v2.m` per Phase 7 spec (block-triangular augmented Lyapunov).
- `motion_control_law_eq17_7state.m` `suppress_xD` flag — diagnostic, can stay.

---

## I. Phase-by-phase v2 readiness summary

| Phase | Spec item | v1 code state | Implementation needed |
|---|---|---|---|
| Phase 0 | 7-state EKF + Σf_d + x̂_D additive | EKF + x̂_D yes; **Σf_d MISSING** | Add Σf_d to control law + persistent buffer |
| Phase 1 | F_e(3,4) = −1.6 (Eq.19 form, d=2, λ_c=0.7) | `−1` hard-coded in `build_F_e` line 537 | Replace constant with `−(1+d·(1−λ_c))` |
| Phase 1 | H matrix unchanged | `H = [1 0 0 0 0 0 0; 0 0 0 0 0 1 −d]` | OK ✓ |
| Phase 2 | C_dpmr=3.96, C_n=1.18, IF_var=4.21 | All computed via build_eq17_constants closed form | OK ✓ |
| Phase 5 | Q33=4kBT·â_x | OK ✓ | OK ✓ |
| Phase 5 | Q55=a_nom²·σ²_w_fD (=0 baseline) | hard-coded 0 | Add config.sigma2_w_fD passthrough |
| Phase 5 | Q77 Path B (Term A + Term B) | OK ✓ | OK ✓ |
| Phase 6 | R(2,2)=a_cov·IF_var·(â_x+ξ)²+5·Q77 | OK ✓ | OK ✓ |
| Phase 6 | ξ per-axis | computed in build_eq17_constants | OK ✓ |
| Phase 6 | 3-guard adaptive R_2 | OK ✓ | OK ✓ |
| Phase 7 | predict_closed_loop_var_eq17_v2.m | NOT PRESENT (only g-sweep v1 helper exists) | Implement per Phase 7 §8 spec |

---

## J. Diff summary table

| File | v1 status | v2 needs | Diff scale |
|---|---|---|---|
| `motion_control_law_eq17_7state.m` (control law) | Missing Σf_d term; xD_hat in bracket | Add `−(1−λ_c)·Σ_{i=1..d} f_d[k−i]`; persistent f_d history; xD_hat as separate `−x̂_D/â_x` term outside bracket | **HIGH risk (controller core)** |
| `motion_control_law_eq17_7state.m` (`build_F_e`) | F_e(3,4) = −1 (Eq.19 default branch) | F_e(3,4) = `−(1+d(1−λ_c))` = −1.6 | **MEDIUM** (single number, but propagates to Lyapunov / Riccati) |
| `motion_control_law_eq17_7state.m` (Q55) | hard-coded 0 | `a_hat²·sigma2_w_fD` | LOW (gated by σ²_w_fD=0 baseline) |
| `build_eq17_constants.m` | All Phase 2/6 constants computed | Add `sigma2_w_fD` field passthrough | LOW |
| `run_pure_simulation.m` (sensor buffer) | Length d=2; effective delay = d−1 = 1 (per 2026-04-29 note) | Length d+1 = 3 + sync controller's `pd_km1/pd_km2` semantics | **HIGH risk** (off-by-one breaks all tracking statistics) |
| `step_dynamics.m` | ode4, 10 µs inner step | Unchanged | NONE |
| `user_config.m` | Missing `sigma2_w_fD` | Add field, default 0 | LOW |
| `calc_simulation_params.m` (CtrlBus) | 22 elements | 23 elements (add `sigma2_w_fD`) | LOW |
| `calc_ctrl_params.m` | passes existing config | Add sigma2_w_fD passthrough | LOW |
| `predict_closed_loop_var.m` (diag) | v1 Eq.18 g-sweep helper only | v2: implement `predict_closed_loop_var_eq17_v2.m` per Phase 7 §8 (block-triangular) | MEDIUM (analytical oracle for Phase 8 verify) |
| Unit tests | 5 existing (warmup/smoke/Lyapunov g-sweep) | Add: delay_R2_factor sweep, F_e(3,4) value, Σf_d behavior, Q55 closed form | MEDIUM |

---

## K. Top-3 most-important diffs

These are gating items for any v2 numerical run:

1. **Σf_d term missing in control law** (`motion_control_law_eq17_7state.m` line 240-246): The current `f_d[k]` formula does NOT include the `−(1−λ_c)·Σ_{i=1..d} f_d[k−i]` term that paper Eq.17 specifies. Phase 1 §2.3 makes this explicit. This is the **root cause of the 26-44% a_hat bias documented in commit 89e6521 / agent_docs/eq17-verification.md**. Without Σf_d, the closed-loop dynamics structurally don't match the F_e the EKF assumes, so KF estimation is biased. Fixing this requires (a) adding `f_d_km1`, `f_d_km2` to persistent state, (b) augmenting the bracket-or-outside formula, (c) confirming `xD_hat` placement (Phase 0 spec: outside bracket as `x̂_D/â_x` separate term).

2. **Sensor delay buffer off-by-one** (`run_pure_simulation.m` line 108): Buffer of length `d_delay=2` effectively delivers `p_m[k−1]` (1-step delay) to the controller, not `p_m[k−2]` (2-step delay) as the design assumes. The 2026-04-29 diagnostic comment in the file flags this. Tentative fix (length 3) was tried but caused crash because the controller's `pd_km1`/`pd_km2` internal buffers are aligned to the d=2 assumption. Needs joint fix of driver buffer length AND controller pd_km synchronization. **High risk** because all numerical tracking statistics depend on this delay being consistent. Phase 6 R(2,2) `5·Q77` term (delay_R2_factor=5 for d=2) only holds if d=2 is actually realized.

3. **F_e(3,4) constant value: −1 → −1.6** (`motion_control_law_eq17_7state.m` `build_F_e` line 537): Phase 1 §10.4 derived `F_e(3,4) = −(1+d·(1−λ_c)) = −1.6` for the v2 Eq.19 form (with Σf_d substitution introducing past x_D contributions). The v1 code uses `−1` (the v1 simplified controller value). Once Σf_d is added (item 1), F_e Row 3 must also be updated to match. Low code change (single-line constant), but **medium impact** because it changes the Lyapunov / Riccati steady-state and therefore the predicted closed-loop variances used by Phase 7 oracle and Phase 8 verification.

These three items are tightly coupled: without all three, the v2 test cannot meaningfully isolate the controller-fix improvement from the sensor-delay artifact. Recommended Phase 8 implementation order:
- Step 1: Fix sensor delay buffer + controller pd_km sync (verify simple positioning still passes smoke test).
- Step 2: Add Σf_d term + persistent f_d history (verify f_d magnitudes still bounded, single-seed run completes).
- Step 3: Update F_e(3,4) to −1.6 in build_F_e (verify EKF a_hat estimate stops biasing).
- Step 4: Add sigma2_w_fD knob (default 0; verify Q55=0 reproduces Step 3 baseline).

---

**End of Phase 8 audit. Ready for v2 implementation planning.**
