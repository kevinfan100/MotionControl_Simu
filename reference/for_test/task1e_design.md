# Task 1e Design: Time-Varying Σ_aug Recursion for Dynamic `a_hat`

**Branch**: `feat/sigma-ratio-filter`
**Date**: 2026-04-13
**Status**: DESIGN — not yet implemented. Requires user approval before implementation.
**Prerequisites**: Task 1d diagnostic report (`task1d_diagnostic_report.md`)

---

## 1. Motivation

Task 1d measured:
- Static z-axis: `a_hat` rel std = 25.86% (paper target ~5-10%)
- Dynamic z-axis near-wall: median error 17.88% + **31 ms lag** (paper target ~8-10%, no visible lag)

Task 1c scalar correction cannot address either — both stem from the steady-state
assumption baked into `C_dpmr_eff`. The fix is to make the `V_IIR → a` map
**time-varying**, driven by a running Σ_aug recursion closed with `a_hat[k-1]`.

User has explicitly excluded the tuning path (Q/R).

---

## 2. Mathematical Formulation

### 2.1 Current (post-Task 1c) `a_m` formula

```
a_m[k] = (del_pmr_var[k] / IIR_bias_factor - noise_corr) / (C_dpmr_eff · 4 k_B T)
```

where `C_dpmr_eff` is the **scalar** steady-state coefficient from Phase 1's
augmented Lyapunov:
```
C_dpmr_eff = (1 - a_pd)^2 · c_s' · Σ_aug_ss · c_s
```
with `Σ_aug_ss` the solution of the 11-dim discrete Lyapunov equation
`Σ = A_aug · Σ · A_aug' + B_th · B_th'` (unit thermal input).

### 2.2 Proposed time-varying formulation

Keep `A_aug` constant (our `L_ss` is pre-computed; only thermal magnitude varies)
but make `Σ_aug` a **persistent recursion**:

```
Σ_aug[k+1] = A_aug · Σ_aug[k] · A_aug' + (4 k_B T · a_hat_prev[k]) · B_th · B_th'
```

where `a_hat_prev[k]` is the per-axis previous-step EKF estimate of `a`.
Initial condition: `Σ_aug[0] = Σ_aug_ss · a_nom` (warm-started at steady state).

At each step, derive the **instantaneous** effective coefficient:
```
C_dpmr_eff[k] = (1 - a_pd)^2 · c_s' · Σ_aug[k] · c_s / a_hat_prev[k]
```

Note: the division by `a_hat_prev[k]` normalizes out the thermal scaling so that
`C_dpmr_eff[k]` remains dimensionless and ~O(C_dpmr_eff_ss). When `a_hat_prev`
has been constant for longer than the `A_aug` effective time constant, `Σ_aug[k]`
converges to `Σ_aug_ss · a_hat_prev` and `C_dpmr_eff[k] → C_dpmr_eff_ss`.

### 2.3 Modified `a_m` formula

```
a_m[k] = (del_pmr_var[k] / IIR_bias_factor[k] - noise_corr) / (C_dpmr_eff[k] · 4 k_B T)
```

`IIR_bias_factor[k]` could also become time-varying (it depends on `ρ(L)` which
depends on `Σ_aug`), but for Task 1e **v1** we hold it fixed at the Phase 1
lc-lookup value. Follow-up Task 1f can make this time-varying if needed.

### 2.4 Why `a_hat[k-1]` closure is stable

- `A_aug` has max |eig| ~ 0.99997 (stable, but near unit circle)
- `Σ_aug[k]` is a linear-in-`a_hat_prev` accumulator with effective time constant
  equal to the closed-loop settling time of `A_aug` (~few hundred samples at 1600 Hz)
- `a_hat[k]` is produced by the 7-state EKF which has its own smoothness from
  `Q_a = 1e-4 · σ²_dXT` (process noise on `a` state)
- Inner loop: `Σ_aug ← a_hat_prev`, outer loop: `a_hat ← a_m ← Σ_aug`
- Gain of the outer loop at DC is 1 (steady state recovers `C_dpmr_eff_ss`),
  and it decreases with frequency due to both IIR and EKF filtering
- Closed-loop bandwidth for the `a`-channel is O(few Hz), well below Nyquist (800 Hz)
- **One-sample closure delay** (0.625 ms) is negligible compared to the 31 ms
  lag we want to eliminate

### 2.5 Numerical safeguards

- `a_hat_prev[k]` clamped to `[a_nom/10, 10·a_nom]` before entering the recursion
  (prevents runaway at deep near-wall where `a` can be small)
- `Σ_aug[k]` symmetrized each step: `Σ_aug = 0.5 · (Σ_aug + Σ_aug')`
- `C_dpmr_eff[k]` clamped to `[0.3 · C_dpmr_eff_ss, 3 · C_dpmr_eff_ss]` as a
  sanity fence (should never trip in normal operation; logs warning if it does)
- First call: `Σ_aug[0] = Σ_aug_ss · a_nom` (warm start, avoids cold-start transient)

---

## 3. Integration Points

### 3.1 `motion_control_law_7state.m`

**New persistent state (add to the existing persistent block near line 55)**:
```matlab
persistent Sigma_aug_state            % 11x11
persistent A_aug_const B_th_const     % 11x11 and 11x1
persistent c_s_const                  % 11x1
persistent gain_sq_const              % scalar (1-a_pd)^2
persistent a_hat_prev                 % 3x1, one per axis
persistent C_dpmr_eff_ss              % scalar, for normalization
```

**Initialization (add after the existing `C_dpmr_eff_const` block, ~line 115)**:
```matlab
% Task 1e: load steady-state Σ_aug components from the Phase 1 offline solve
% stored alongside cdpmr_eff_lookup.mat (new fields to add)
if isfield(params.ctrl, 'A_aug') && isfield(params.ctrl, 'B_th') && ...
   isfield(params.ctrl, 'Sigma_aug_ss')
    A_aug_const       = params.ctrl.A_aug;            % 11x11
    B_th_const        = params.ctrl.B_th;             % 11x1
    Sigma_aug_state   = params.ctrl.Sigma_aug_ss * a_nom;  % warm start
    C_dpmr_eff_ss     = params.ctrl.C_dpmr_eff;
    c_s_const         = zeros(11, 1);
    c_s_const(3)      = 1;
    c_s_const(11)     = -1;
    gain_sq_const     = (1 - a_pd)^2;
    a_hat_prev        = [a_nom; a_nom; a_nom];
    use_tv_sigma_aug  = true;
else
    use_tv_sigma_aug  = false;
end
```

**Runtime recursion (replace lines 161-163 `den / a_m` block)**:
```matlab
if use_tv_sigma_aug
    % Clamp a_hat_prev for stability
    a_hat_safe = min(max(a_hat_prev, a_nom / 10), 10 * a_nom);
    % Use MEAN across axes for Sigma_aug (single shared recursion — see 3.3)
    a_hat_mean = mean(a_hat_safe);
    % Recursion
    Sigma_aug_state = A_aug_const * Sigma_aug_state * A_aug_const' ...
                      + (4 * k_B * T_temp * a_hat_mean) * (B_th_const * B_th_const');
    Sigma_aug_state = 0.5 * (Sigma_aug_state + Sigma_aug_state');
    % Time-varying C_dpmr_eff
    Var_tv_unit = gain_sq_const * (c_s_const' * Sigma_aug_state * c_s_const);
    C_dpmr_tv   = Var_tv_unit / a_hat_mean;
    % Sanity fence
    C_dpmr_tv   = min(max(C_dpmr_tv, 0.3 * C_dpmr_eff_ss), 3 * C_dpmr_eff_ss);
    den         = C_dpmr_tv * 4 * k_B * T_temp;
else
    den         = C_dpmr_eff_const * 4 * k_B * T_temp;
end
noise_corr = C_np_eff_const * sigma2_noise;
del_pmr_var_unbiased = del_pmr_var / IIR_bias_factor_const;
a_m        = max((del_pmr_var_unbiased - noise_corr) / den, 0);
```

**State update at the end of the step (just before `ekf_out` assignment ~line 330)**:
```matlab
a_hat_prev = a_hat;   % 3x1, feeds next step's Sigma_aug recursion
```

### 3.2 New offline precomputation

Extend `test_script/build_cdpmr_eff_lookup.m` (or create a new
`test_script/build_sigma_aug_components.m`) to:
1. For each `lc` in the grid, call `compute_7state_cdpmr_eff`
2. Extract `A_aug`, `B_th`, `Sigma_aug_ss = Sigma_th` (already returned in `diag_out`)
3. Save alongside `Cdpmr_tab` / `Cnp_tab` as new fields in `cdpmr_eff_lookup.mat`

### 3.3 Per-axis vs shared Σ_aug — design choice

**Option (a)** — single shared recursion using `mean(a_hat)` across axes:
- Pro: one 11×11 state, fast
- Pro: works well in free-space (all axes see same `a`)
- Con: in near-wall, `a_hat_z` can differ from `a_hat_x/y` by up to 5× (parallel vs normal)
- Con: map error when axes are decoupled

**Option (b)** — three independent recursions (one per axis):
- Pro: each axis tracks its own `a` accurately
- Pro: matches the physics (three different `a` values in wall frame)
- Con: 3× state and compute (still trivial at 1600 Hz)
- Con: code complexity slightly higher

**Recommendation**: start with **Option (b)** since the MC results showed
axes differ dramatically near wall. Compute cost is negligible (11×11 matmul
per axis per step = 3×1331 multiplies ≈ 4000 flops at 1600 Hz = 6.4 Mflops,
trivial).

### 3.4 Bus and params flow

Add three new fields to `calc_ctrl_params.m`:
- `ctrl.A_aug` (11×11)
- `ctrl.B_th` (11×1)
- `ctrl.Sigma_aug_ss` (11×11)

Add to `calc_simulation_params.m` CtrlBus:
- Element 29: `A_aug` (11×11 double)
- Element 30: `B_th` (11×1 double)
- Element 31: `Sigma_aug_ss` (11×11 double)

(CtrlBus already has elements 1-28 with IIR_bias_factor as 28.)

---

## 4. Verification Strategy

### 4.1 Regression test 1: static Phase 2A (must still pass)
- Rerun `verify_task1c_correction.m` style with time-varying Σ_aug enabled
- Expected: `mean(a_m_z) / a_nom ≈ 1.0137 ± 0.02` (same as Task 1c — static should
  converge to the same answer)
- If static regresses, Σ_aug warm start is wrong

### 4.2 Regression test 2: dynamic MC (the target)
- Rerun `verify_task1d_paper_benchmark_mc.m` with time-varying Σ_aug enabled
- **Success criterion**: z-axis median error drops from 17.88% → `< 12%`
- **Stretch**: z-axis lag drops from 31 ms → `< 10 ms`
- If lag unchanged, the recursion is not responding fast enough (check `A_aug`
  time constant vs 1 Hz signal)

### 4.3 Unit test: convergence to Task 1b closed form at steady state
- Drive `Sigma_aug_state` with constant `a_hat_prev = a_nom` for ~1000 steps
- Check `C_dpmr_tv → C_dpmr_eff_ss` within machine precision
- Confirms recursion + map math match Phase 1 / Task 1b

### 4.4 Unit test: known time-varying response
- Offline, drive the recursion with a step change in `a` from `a_nom` to `0.5·a_nom`
- Verify `C_dpmr_tv` decays exponentially to the new steady state with time
  constant matching `A_aug` dominant eigenvalue
- Compare to the analytic `verify_sigma_mc_1d.m` prediction (after porting from
  4×4 ctrl4 to 11×11 augmented 7-state form)

---

## 5. Effort Estimate

| Step                                    | Time     |
|-----------------------------------------|----------|
| Offline: add Σ_aug components to lookup | 1 h      |
| Modify `calc_ctrl_params.m` + Bus       | 1 h      |
| Modify `motion_control_law_7state.m`    | 2 h      |
| Unit tests (4.3, 4.4)                   | 1 h      |
| Regression 4.1 static Phase 2A          | 0.5 h run + analysis |
| Regression 4.2 dynamic MC               | 1 h run + analysis   |
| Report + commit                         | 1 h      |
| **Total**                               | **~7-8 hours (1 focused day)** |

---

## 6. Risk and Fallback

### 6.1 Risk: dynamic lag unchanged
If the recursion's time constant is dominated by `A_aug`'s ~0.99997 eigenvalue
(~30000 samples = 18 seconds), it'll be as slow as the current steady-state
assumption. The fix may be ineffective.

**Fallback**: truncate the history with a faster forgetting factor:
```
Σ_aug[k+1] = λ · (A_aug · Σ_aug[k] · A_aug') + (4 k_B T · a_hat_prev) · B_th · B_th'
```
where `λ < 1` (e.g., 0.99) introduces forgetting. This is ad-hoc but may be
necessary. Evaluate only after 4.2 shows no improvement.

### 6.2 Risk: feedback instability
If `a_hat_prev → 0` the recursion still has the `A_aug^k` memory term but no
fresh thermal input — `Σ_aug` decays. This is the right behavior. No instability
expected. The clamp at `a_nom/10` is belt-and-suspenders.

### 6.3 Risk: static regression
The time-varying recursion at true steady state should converge to the same
numerical answer as Phase 1's scalar `C_dpmr_eff`. If `static a_m_z/a_nom`
drifts from 1.0137 more than ±1%, there's a bug in the warm-start or the
normalization. Fix: verify 4.3 unit test.

### 6.4 Risk: computational cost
11×11 matmul per step per axis = 3 × 1331 mults × 1600 Hz = 6.4 Mflops. Trivial.
Simulink's overhead per block call is much larger.

### 6.5 Risk: Σ_aug drift due to floating point
Symmetrization each step + eigenvalue `< 1` should keep it bounded. If not,
add a trace check and a reset to steady state if trace grows beyond a threshold.

---

## 7. Out of Scope for Task 1e

- Q/R retuning (excluded by user)
- Multi-axis joint estimation (Phase 3 already rejected)
- Replacing the IIR variance estimator (would require re-deriving everything)
- Time-varying `IIR_bias_factor` (deferred to Task 1f if still needed)
- Measurement noise case (`C_np_eff` side) — still uses the scalar value
- EKF structure changes (still 7-state)

---

## 8. Go/No-Go for Implementation

**Go criteria (user decision)**:
1. User accepts the ~1-day implementation effort
2. User accepts the small added complexity in the controller (~30 lines)
3. User agrees the Task 1d findings justify the change

**No-go fallbacks**:
- If the 17.88% dynamic median error is "close enough for the user's actual use case"
  → skip Task 1e, document current state as the shipping baseline.
- If a simpler alternative surfaces (e.g., the paper's own implementation has a
  detail we missed that can be ported more cheaply) → reassess.

---

## 9. Critical Files to Modify (when implementation is approved)

- `model/controller/motion_control_law_7state.m` — add persistent state + recursion
- `model/controller/calc_ctrl_params.m` — load new Σ_aug components
- `model/calc_simulation_params.m` — add 3 CtrlBus elements
- `test_script/build_cdpmr_eff_lookup.m` — extend to save Σ_aug components
- `test_results/verify/cdpmr_eff_lookup.mat` — regenerate with new fields

## 10. Critical Files to Reuse

- `test_script/compute_7state_cdpmr_eff.m` — already produces `A_aug`, `B_th`, `Sigma_th`
- `test_script/verify_sigma_mc_1d.m` — template for recursion verification (port to 11×11)
- `test_script/analyze_task1d_ahat_static.m` — regression harness (4.1)
- `test_script/verify_task1d_paper_benchmark_mc.m` — target benchmark (4.2)
