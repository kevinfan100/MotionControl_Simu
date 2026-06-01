# Phase 8 — Wave 2D: Driver Buffer Fix + Bus Extension

**Date**: 2026-04-29
**Branch**: `test/eq17-5state-ekf`
**Scope**: Wave 4 driver / config / Bus chain that follows Wave 2B (controller v2) and Wave 2C (oracle v2).
**Status**: Implementation complete (no commit, no simulation run).

---

## 1. Tasks delivered

| # | Task | File(s) | LoC | Status |
|---|------|---------|-----|--------|
| 1 | Driver buffer length fix `d → d+1` (true d-step delay) | `model/pure_matlab/run_pure_simulation.m` | +13/-5 | done |
| 1 | Driver pass `eq17_opts.a_pd` + `sigma2_w_fD` | `model/pure_matlab/run_pure_simulation.m` | +7 | done |
| 2 | `build_eq17_constants` accept + propagate `a_pd`, `sigma2_w_fD` | `model/controller/build_eq17_constants.m` | +18 | done |
| 3 | `motion_control_law_eq17_7state` use `ctrl_const.a_pd` for IIR LP | `model/controller/motion_control_law_eq17_7state.m` | +12/-1 (Wave 2D only, on top of Wave 2B base) | done |
| 4 | `calc_ctrl_params` pass `sigma2_w_fD` through | `model/controller/calc_ctrl_params.m` | +10 | done |
| 5 | `calc_simulation_params` extend CtrlBus 22→23 with `sigma2_w_fD` | `model/calc_simulation_params.m` | +5/-1 | done |
| 6 | `user_config` add `sigma2_w_fD = 0` + clarify `a_pd`/`a_cov` semantics | `model/config/user_config.m` | +14/-3 | done |
| 7 | New 5-seed CI wrapper `run_v2_h50_e2e.m` | `test_script/run_v2_h50_e2e.m` | +258 (new file) | done |

Total: 7 files modified/created, ~334 LoC delta.

---

## 2. Buffer fix — root cause + correction

Original code (line 108):
```
p_m_buffer = repmat(p0, 1, d_delay)         % [3 x 2] for d=2
```

Per-step ordering:
```
(i)  p_m_delayed = p_m_buffer(:, 1)        % read OLDEST first
(ii) p_m_buffer  = [p_m_buffer(:, 2:end), p_m_raw]  % shift in newest
```

For d_delay=2, the buffer has 2 columns. At step k (after k-1 prior shifts):
* `p_m_buffer(:,1)` was inserted at step k-1 (one shift ago) — actually p_m at step k-1.

So the read returns p_m at (k - 1), implementing **(d-1)=1-step delay**, not the spec'd **d=2-step delay**. Confirmed by 89e6521 diagnostic (kept the buggy d=2 buffer, observed effective d=1, which Agent B's controller assumes is wrong per Phase 1 §10.4 derivation `F_e(3,4)=-(1+d·(1-λ_c))=-1.6`).

**Fix**: `p_m_buffer = repmat(p0, 1, d_delay + 1)` makes buffer hold 3 columns. After the same per-step ordering, `buffer(:,1)` returns p_m at (k - 2), matching the d=2 design spec.

Agent B's controller `motion_control_law_eq17_7state.m` lines 261-269 already aligns `pd_km2` with `pd[k-2]` and uses it to form `delta_x_m = pd_km_d - p_m`. With the driver buffer fix, the controller now sees a true 2-step delayed `p_m`, matching the y_1 = δx[k-2] measurement equation (Phase 1 §10).

---

## 3. Bus chain — sigma2_w_fD passthrough

Path: `config.sigma2_w_fD` → `params.ctrl.sigma2_w_fD` (CtrlBus elem 23) → `eq17_opts.sigma2_w_fD` → `ctrl_const.sigma2_w_fD` → controller persistent `sigma2_w_fD`.

* `user_config` defaults `sigma2_w_fD = 0` (Phase 5 §5.4 baseline).
* `calc_ctrl_params` reads from config (with default 0 if absent), writes to `ctrl.sigma2_w_fD`.
* `calc_simulation_params` adds CtrlBus element 23 (1×1 double, name `sigma2_w_fD`).
* `run_pure_simulation` reads `config.sigma2_w_fD` and assembles `eq17_opts.sigma2_w_fD`.
* `build_eq17_constants` defaults missing `opts.sigma2_w_fD` to 0, copies into `ctrl_const.sigma2_w_fD`.
* Controller (already in Wave 2B) reads from `ctrl_const.sigma2_w_fD` with isfield/isempty defensive guard, defaults 0.

**Backward compat**: every layer has a default-0 fallback — old callers (legacy 7-state Simulink path that doesn't extend the Bus) won't break. The default Q55 = a_nom_axis² · 0 = 0 reproduces the Phase 5 baseline.

---

## 4. a_pd separation (per Wave 2B §5.6 flag)

Wave 2B §5.6 flagged that `a_var` (LP coefficient for `dx_bar_m`) was hard-aliased to `ctrl_const.a_cov`. v2 design_v2 §6 distinguishes:
* `a_pd`  = LP coefficient for `δp_md` (mean estimation)
* `a_cov` = EWMA coefficient for `σ²_δxr` (variance estimation)

Phase 0 baseline locks both at 0.05, so the existing alias produces identical numerics. But the field separation matters for future tuning (e.g., decoupling LP responsiveness from variance smoothing).

**Wiring**:
* `build_eq17_constants` accepts `opts.a_pd` (default = `opts.a_cov` for backward compat), copies to `ctrl_const.a_pd`.
* `motion_control_law_eq17_7state` lines 146-159 (within the existing initialization block):
  - new persistent `a_pd`
  - reads `ctrl_const.a_pd` if present, else falls back to `a_cov`
  - sets `a_var = a_pd` (LP coefficient consumed by line 298 `dx_bar_m_new = (1 - a_var) * dx_bar_m + a_var * delta_x_m`)
* `run_pure_simulation` adds `eq17_opts.a_pd = config.a_pd`.
* `calc_ctrl_params` already carries `ctrl.a_pd` from previous design (no new field needed there).

User-facing tuning is now `config.a_pd` (LP) and `config.a_cov` (EWMA) — both default 0.05. To diverge them, set them differently in run_simulation.m or a new sweep script.

---

## 5. Trajectory generator — positioning support confirmed

`model/trajectory/trajectory_generator.m` lines 53-66:
```matlab
trajectory_type = params.traj.trajectory_type;
if trajectory_type > 1.5
    h = h_init;
    p_d = (pz + h) * w_hat;
    ...
    return;
end
```

`calc_traj_params.m` maps `'positioning'` → 2 and `'osc'` → 1. So `trajectory_type='positioning'` triggers the early-return positioning branch where `p_d = p0` constant. **No modifications needed.**

For the wrapper: setting `config.trajectory_type = 'positioning'`, `config.h_init = 50`, `config.h_bottom = 50` (no descent), `config.amplitude = 0` (no oscillation) gives a pure constant-position scenario. The wrapper sets all three explicitly.

---

## 6. ekf_out axis order — confirmed and documented

`motion_control_law_eq17_7state.m` line 606:
```matlab
ekf_out = [a_hat_post(1); a_hat_post(3); a_hat_post(2); h_bar_now];
        % col 1 = a_hat_x, col 2 = a_hat_z, col 3 = a_hat_y, col 4 = h_bar
```

The brief had a one-line inconsistency:
* Comment block: `[a_hat_x, a_hat_z, a_hat_y, h_bar]` (matches code)
* Later code: `metrics.a_hat_mean(s, :) = [mean(a_hat_x), mean(a_hat_y), mean(a_hat_z)]` (treats stored axes as [x,y,z])

I resolved this by reading from `simOut.ekf_out(:, 1)`, `(:, 2)`, `(:, 3)` as `a_hat_x`, `a_hat_z`, `a_hat_y` respectively, then re-packing in `[x, y, z]` order in the metrics. This matches per-axis a_nom_per_axis ordering (also `[x, y, z]`). All output reports use the canonical `[x, y, z]` order.

---

## 7. Wrapper expected workflow (`run_v2_h50_e2e.m`)

```
1. Build base config: positioning, h_init=50, T_sim=5, lambda_c=0.7,
                      meas_noise_std=[0.62; 0.057; 3.31] nm,
                      a_pd=a_cov=0.05, sigma2_w_fD=0.
2. Resolve params once → extract a_nom_per_axis (free-space Ts/gamma_N
                          divided by per-axis correction at h_bar=22.2).
3. For each seed in [1,2,3,4,5]:
     simOut = run_pure_simulation(config, opts);
     idx = (n_warmup+1):end;          % skip first 0.5 s
     tracking_std_seed(s,:) = std(simOut.p_d_out(idx,:) - simOut.p_m_out(idx,:));
     a_hat_*_seed(s,:) = stats over simOut.ekf_out(idx, 1..3);
4. Aggregate cross-seed:
     tracking_std_mean / std (1×3)
     a_hat_mean (1×3) → bias % vs a_nom_per_axis
     a_hat_std mean (1×3)
     s2_dx_observed mean (1×3)
5. Optional oracle:
     ctrl_const = build_eq17_constants(eq17_opts);
     R_axis = a_cov * IF_var * (a_used + xi)^2;
     [s2_dx_pred, ...] = predict_closed_loop_var_eq17_v2(oracle_opts);
     ratio = s2_dx_observed / s2_dx_pred;     % target 1.0 for verification
6. Print summary table (tracking std nm, a_hat bias %, a_hat std,
                         observed/predicted s2_dx ratio).
```

Wave 4 Agent F can call:
```matlab
results = run_v2_h50_e2e();                       % default 5 seeds
results = run_v2_h50_e2e(struct('seeds', 1:3));   % 3-seed quick check
```

The wrapper is self-contained; it adds the necessary paths, resolves params, and does NOT spawn Simulink (uses `run_pure_simulation` only).

---

## 8. checkcode results

| File | Result |
|------|--------|
| `model/pure_matlab/run_pure_simulation.m` | 1 pre-existing message about a stale `%#ok<CLFUNC>` annotation on line 72 (unchanged from before this task). Not introduced by me. |
| `model/controller/build_eq17_constants.m` | No issues |
| `model/controller/calc_ctrl_params.m` | No issues |
| `model/controller/motion_control_law_eq17_7state.m` | No issues |
| `model/config/user_config.m` | No issues |
| `model/calc_simulation_params.m` | No issues |
| `test_script/run_v2_h50_e2e.m` | No issues (after replacing `datestr(now)` with `char(datetime('now'))`) |

---

## 9. Implementation questions / flags

### 9.1 Trajectory positioning support
**Confirmed working**, no modifications needed. Branch at `trajectory_generator.m:56` activates when `trajectory_type > 1.5` (i.e., string `'positioning'` mapped to numeric 2 by `calc_traj_params`).

### 9.2 ekf_out axis order
**Confirmed**: `[a_hat_x, a_hat_z, a_hat_y, h_bar]`. Wrapper handles the remap to `[x, y, z]` internally.

### 9.3 controller_type mismatch in user_config
`config.controller_type = 23` is the user_config default. The pure-MATLAB driver `run_pure_simulation` ignores this field (it only invokes `motion_control_law_eq17_7state`). The wrapper sets `config.controller_type = 7` for trace consistency, but it has no functional effect on the eq17 path.

If Wave 4 wants to add a runtime guard `if config.controller_type ~= 7 ... error(...)`, that's a one-line addition and should be confirmed before doing so. Not in this task's scope.

### 9.4 Q77 = 0 assumption in oracle
The wrapper passes `Q77_axis = [0, 0, 0]` to the oracle. This matches Phase 8 §157 settings audit (positioning, frozen a, ḣ=ḧ=0). For h=50 motion or h=2.5 oscillation, this would need to be replaced with the time-mean Q77 from `motion_control_law_eq17_7state` (computed per-step from K_h and h_dot/h_ddot). Out of scope for this Wave; wrapper is positioning-only.

### 9.5 Simulink path side effect
The CtrlBus extension (22→23 elements) means existing Simulink models that use `Bus: CtrlBus` will need to be re-saved or re-validated. This is **only relevant** to the legacy `system_model.slx` Simulink path (controller_type=7 / 23 via `motion_control_law_*.m`); the eq17 path uses pure-MATLAB and doesn't touch the .slx model.

The new field is added at the end (element 23), which preserves indices 1-22 → existing field accesses by name (`params.ctrl.foo`) keep working. But if any Simulink block references `Bus: CtrlBus` by signature/dimension count, that would need a re-validation pass. This is a known cost of extending a Bus and is documented in the Wave 4 plan. Wave 4 Agent F should confirm Simulink re-validation is acceptable.

### 9.6 Pf_init_diag legacy field
Wave 2B §3.7 noted that `params.ctrl.Pf_init_diag` is **no longer read** by the eq17 controller (replaced by physical wall-aware `a_x_init` derivation). The field still exists in CtrlBus as element 20 to keep the v1 path functional. This is intentional and not modified here.

---

## 10. Files NOT modified (deliberate)

| File | Reason |
|------|--------|
| `model/trajectory/trajectory_generator.m` | Positioning mode already supported (line 56-66). Read-only confirmed. |
| `model/trajectory/calc_traj_params.m` | Positioning mode mapping already in place (line 24-28). Read-only confirmed. |
| `model/controller/motion_control_law_7state.m` | v1 legacy controller. Bus extension is backward-compatible (default sigma2_w_fD=0). |
| `model/controller/motion_control_law_23state.m` | v1 legacy controller. Same as above. |
| `model/controller/motion_control_law.m` | Top-level dispatch. Doesn't read sigma2_w_fD; fine. |
| `test_script/unit_tests/test_build_eq17_constants.m` | Tests defaults only; new fields have safe defaults so test passes unchanged. |
| `test_script/unit_tests/test_motion_control_law_eq17_7state.m` | Not inspected; if it provides explicit `ctrl_const`, it must be checked by Wave 4. Note the controller falls back to `a_cov` and `0` defensively, so tests using a struct without these fields should still pass. |

---

## 11. Handoff to Wave 4 (Agent F)

1. Run `test_script/run_v2_h50_e2e.m` with default opts (5 seeds, T_sim=5, h_init=50).
2. Compare `results.aggregate.tracking_std_mean` and `results.oracle.s2_dx_pred` against Phase 7 closed-form prediction.
3. If `ratio_obs_to_pred` is in [0.84, 1.20] for all axes, V2 is verified for h=50 positioning.
4. If significant deviation: check seed-to-seed variance, then iterate on Q/R per-axis tuning (out of scope for Wave 4 Agent D).
5. If `test_motion_control_law_eq17_7state.m` references new ctrl_const fields, run that test first to confirm no regression.

---

## 12. Summary

All Wave 2D tasks complete. checkcode clean (1 pre-existing comment on `run_pure_simulation.m:72` not introduced by this task). No simulations run. No commits. Files ready for Wave 4 Agent F's e2e CI run.

Critical path:
* Driver buffer fix (`d → d+1`) is the high-impact change — combined with Agent B's `pd_km1`/`pd_km2` alignment, the driver+controller now implement true 2-step sensor delay matching Phase 1 §10.4 derivation `F_e(3,4) = -(1 + d·(1-λ_c)) = -1.6`.
* Bus chain extensions are low-risk, fully backward-compatible (every layer has default-0 fallback for `sigma2_w_fD` and default-`a_cov` fallback for `a_pd`).
* `run_v2_h50_e2e.m` is a clean wrapper around `run_pure_simulation` with built-in oracle comparison.
