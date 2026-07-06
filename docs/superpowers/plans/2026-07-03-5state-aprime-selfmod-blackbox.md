# 5-State a'_x Self-Modulation + Honest Q(a') Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an opt-in `use_selfmod` mode to the existing `motion_control_law_eq17_5state_aprime.m` controller that (a) restores hold-observability of the gain slope `a'_x` via the thermal self-dither coupling, and (b) replaces the wall-peeking Q(5,5) formula with the honest, dimensional-anchor-derived one — then directly test, on the real controller, whether this resolves the statistically significant wrong-sign result found by the external regression test this session (`verify_aprime_blackbox_v1_ensemble.m`).

**Architecture:** Extend the existing controller with one new boolean toggle (`ctrl_const.use_selfmod`, default `false` = bit-identical to current behavior), matching this codebase's established pattern (e.g. the 4-state's `use_deblur`, `use_q44_ar1` toggles). No new controller file. Reuses `build_eq17_6state_constants` and `run_pure_simulation` unchanged except for one new pass-through field.

**Tech Stack:** MATLAB (project convention: `checkcode` static analysis, `run_pure_simulation.m` dual-track driver, `verify_*.m` integration test scripts in `test_script/integration/`).

## Global Constraints

- Every code change must default to `use_selfmod=false` and reproduce the CURRENT baseline bit-for-bit (regression safety) — this is a production controller path, not throwaway research code.
- No `calc_correction_functions` (real wall model `c(h̄)`) may be introduced anywhere in the NEW `use_selfmod=true` code path. The whole point is a black-box design; the F_e(3,4)/Q33/Q44/R22 paths that ALREADY call `calc_correction_functions` (wall-peeking) are explicitly OUT OF SCOPE for this plan and are left untouched — see `reference/eq17_analysis/derivation/5state_aprime_unified.tex` Section 7 note and the "Known limitations" section below.
- Follow `.claude/rules/matlab-conventions.md`: `checkcode` must report 0 issues on every modified file before it is considered done.
- Do not commit until the CLAUDE.md rule is satisfied ("只有功能完整、測試通過才能 commit") — this plan's last task is the point where that becomes true; do not commit mid-plan.
- Source derivation: `reference/eq17_analysis/derivation/5state_aprime_unified.tex` Sections 3–7 (state/F_e/H equations already fixed; do not re-derive).

---

### Task 1: Add the self-mod coupling term to `build_F_e_5state_aprime.m`

**Files:**
- Modify: `model/controller/build_F_e_5state_aprime.m`
- Test: `test_script/integration/verify_eq17_5state_aprime_selfmod_L0.m` (created in Task 5; this task only touches the matrix builder)

**Interfaces:**
- Consumes: nothing new (pure function, no state).
- Produces: `build_F_e_5state_aprime(lambda_c, f_d_i, F_1_i, dFh_i, Delta_h_d, dxh3_selfmod)` — a NEW 6th optional argument. Callers passing 5 arguments (the entire rest of the codebase, today) are unaffected: `dxh3_selfmod` defaults to `0`, which makes `F_e(4,5) = Delta_h_d` exactly as before.

- [ ] **Step 1: Read the current file to confirm line numbers before editing**

Run: `sed -n '1,28p' model/controller/build_F_e_5state_aprime.m`
Expected: matches the version already known (20 lines of code, `F_e = [0 1 0 0 0; ...]` as the last statement).

- [ ] **Step 2: Replace the file contents**

Replace the entire file with:

```matlab
function F_e = build_F_e_5state_aprime(lambda_c, f_d_i, F_1_i, dFh_i, Delta_h_d, dxh3_selfmod)
%BUILD_F_E_5STATE_APRIME  5x5 error-dynamics matrix (per axis); slot 5 = a'_x.
%   Standalone so the L0 Jacobian check can exercise the EXACT matrix the
%   controller uses. See reference/eq17_analysis/derivation/5state_est_aprime.tex
%   (baseline) and reference/eq17_analysis/derivation/5state_aprime_unified.tex
%   Section 3-5 (self-mod extension).
%
%   F_e = build_F_e_5state_aprime(lambda_c, f_d_i, F_1_i, dFh_i, Delta_h_d)
%   F_e = build_F_e_5state_aprime(lambda_c, f_d_i, F_1_i, dFh_i, Delta_h_d, dxh3_selfmod)
%
%   cols: 1=dx1 2=dx2 3=dx3 4=a_x 5=a'_x
%   Row 3 = [0 0 lc -F_dx dF_dx^h]
%       F_dx    = f_d[k] + (1-lc)*F_1_i                     (col 4, a_x)
%       dF_dx^h = (1-lc)*sum_i (h_d[k]-h_d[k-i]) f_d[k-i]   (col 5, a'_x; passed as dFh_i)
%   Row 4 = [0 0 0 1 Delta_h_d + (1-lc)*dxh3_selfmod]
%       a_x[k+1] = a_x[k] + a'_x[k]*[Delta_h_d[k] + (1-lc)*delta_x_hat_3[k]] + ...
%       dxh3_selfmod (optional, default 0) is the CURRENT delta_x_hat_3 estimate
%       (the filtered tracking-error-3 state, NOT the raw dx_r residual --
%       see 5state_aprime_unified.tex Section 6). When 0 (default), this row
%       reduces EXACTLY to the baseline F_e(4,5) = Delta_h_d (motion-only
%       observability). When nonzero (use_selfmod=true in the controller),
%       the thermal self-dither term restores hold-observability
%       (5state_aprime_selfmod.tex: rank(O_N) 4 -> 5 on a height hold).
%   Row 5 = [0 0 0 0 1]            (a'_x random walk)
%
%   vs the rate 5-state build_F_e_5state: F_e(4,5) is Delta_h_d (was the
%   constant 1) and F_e(3,5) is dF_dx^h (was (1-lc)*sum_i i*f_d[k-i]).
%
%   See also: motion_control_law_eq17_5state_aprime, build_F_e_5state

    if nargin < 6 || isempty(dxh3_selfmod)
        dxh3_selfmod = 0;
    end
    one_minus_lc = 1 - lambda_c;
    Fe3_a  = -f_d_i - one_minus_lc * F_1_i;     % col 4 (a_x): -F_dx
    Fe3_ap = dFh_i;                             % col 5 (a'_x): dF_dx^h
    Fe4_ap = Delta_h_d + one_minus_lc * dxh3_selfmod;   % col 5 (a'_x), row 4 (a_x)
    F_e = [0 1 0        0      0; ...
           0 0 1        0      0; ...
           0 0 lambda_c Fe3_a  Fe3_ap; ...
           0 0 0        1      Fe4_ap; ...
           0 0 0        0      1];
end
```

- [ ] **Step 3: Static-analysis check**

Run: `/Applications/MATLAB_R2025b.app/bin/matlab -batch "checkcode('model/controller/build_F_e_5state_aprime.m')"`
Expected: no output (0 issues). If `mcp__matlab__check_matlab_code` is available in your session, prefer that tool instead.

- [ ] **Step 4: Manual backward-compatibility check (5-arg call unaffected)**

Run:
```
/Applications/MATLAB_R2025b.app/bin/matlab -batch "addpath(genpath('model')); F1 = build_F_e_5state_aprime(0.7, 1.0, 0.5, 0.02, 0.01); F2 = build_F_e_5state_aprime(0.7, 1.0, 0.5, 0.02, 0.01, 0); disp(isequal(F1,F2))"
```
Expected: `1` (the 5-arg call and the 6-arg call with `dxh3_selfmod=0` produce identical matrices).

- [ ] **Step 5: Do not commit yet** (this file alone is not a working feature; commit happens at Task 7).

---

### Task 2: Add the `use_selfmod` toggle to `motion_control_law_eq17_5state_aprime.m`

**Files:**
- Modify: `model/controller/motion_control_law_eq17_5state_aprime.m`

**Interfaces:**
- Consumes: `build_F_e_5state_aprime(...)` new 6th argument (Task 1).
- Produces: `ctrl_const.use_selfmod` (new optional boolean field, default `false`). No change to the function's own signature or its 3 outputs (`f_d`, `ekf_out`, `diag`).

This task has 5 sub-edits to the same file. Each is independently a small, exact diff.

- [ ] **Step 1: Add the persistent variable and parse the new flag**

Find this block (around line 103-107):
```matlab
    persistent initialized
    persistent lambda_c d_delay Ts kBT R_radius gamma_N_p
    persistent a_pd a_cov C_dpmr C_n K_var IF_abc xi_per_axis var_da_inc_factor
    persistent t_warmup_kf h_bar_safe R_OFF use_am_lpf a_det amlpf_var_factor
    persistent sigma2_n_s a_x_init enable_wall w_hat_n pz_wall
    persistent Q_aprime_factor freeze_aprime aprime_init
```
Replace with:
```matlab
    persistent initialized
    persistent lambda_c d_delay Ts kBT R_radius gamma_N_p a_nom_p
    persistent a_pd a_cov C_dpmr C_n K_var IF_abc xi_per_axis var_da_inc_factor
    persistent t_warmup_kf h_bar_safe R_OFF use_am_lpf a_det amlpf_var_factor
    persistent sigma2_n_s a_x_init enable_wall w_hat_n pz_wall
    persistent Q_aprime_factor freeze_aprime aprime_init use_selfmod
```
(Two additions: `a_nom_p` and `use_selfmod`.)

- [ ] **Step 2: Parse `use_selfmod` from `ctrl_const` and make `a_nom` persistent**

Find this block (around line 138-142):
```matlab
        % --- aprime-specific knobs ---
        Q_aprime_factor  = get_field_default(ctrl_const, 'Q_aprime_factor', 1);
        Pf_aprime_scale  = get_field_default(ctrl_const, 'Pf_aprime_scale', 1);
        freeze_aprime    = logical(get_field_default(ctrl_const, 'freeze_aprime', false));
        % NOTE: ctrl_const.suppress_xD is ignored (no disturbance term exists).
```
Replace with:
```matlab
        % --- aprime-specific knobs ---
        Q_aprime_factor  = get_field_default(ctrl_const, 'Q_aprime_factor', 1);
        Pf_aprime_scale  = get_field_default(ctrl_const, 'Pf_aprime_scale', 1);
        freeze_aprime    = logical(get_field_default(ctrl_const, 'freeze_aprime', false));
        % Self-modulation (hold-observability) + honest Q55, per
        % reference/eq17_analysis/derivation/5state_aprime_unified.tex Sec.3-5,7.
        % Default false: bit-identical to the pre-existing baseline.
        use_selfmod      = logical(get_field_default(ctrl_const, 'use_selfmod', false));
        % NOTE: ctrl_const.suppress_xD is ignored (no disturbance term exists).
```

Find this block (around line 155-157):
```matlab
        % --- 0E. Wall-aware a_x[0] + a'_x[0] seeding (one-time nominal; the
        %         ongoing estimation never re-reads c(h_bar)) ---
        a_nom = Ts / gamma_N_p;
```
Replace with:
```matlab
        % --- 0E. Wall-aware a_x[0] + a'_x[0] seeding (one-time nominal; the
        %         ongoing estimation never re-reads c(h_bar)) ---
        a_nom_p = Ts / gamma_N_p;      % persistent: also needed every step by the
                                       % honest Q55 dimensional anchor (Step 4 below)
        a_nom = a_nom_p;               % local alias, keeps the rest of this block unchanged
```

- [ ] **Step 3: Reorder `x_curr` before the `F_e` call, and pass the self-mod term**

Find this block (around line 464-476, inside `for ax = 1:3` in section `[5] EKF predict + update per axis`):
```matlab
    for ax = 1:3
        % F_e Row 3: -F_dx (col 4), dF_dx^h (col 5); Row 4 col 5 = Delta_h_d.
        if d_delay == 2
            F_1_i  = f_d_km1(ax) + f_d_km2(ax);                       % sum f_d[k-i]
            dFh_i  = one_minus_lc * (dh1 * f_d_km1(ax) + dh2 * f_d_km2(ax));
        else
            F_1_i  = f_d_km1(ax);
            dFh_i  = one_minus_lc * (dh1 * f_d_km1(ax));
        end
        F_e = build_F_e_5state_aprime(lambda_c, f_d(ax), F_1_i, dFh_i, Delta_h_d);

        x_curr = x_e_per_axis(:, ax);
        P_curr = P_per_axis{ax};
```
Replace with:
```matlab
    for ax = 1:3
        x_curr = x_e_per_axis(:, ax);
        P_curr = P_per_axis{ax};

        % F_e Row 3: -F_dx (col 4), dF_dx^h (col 5); Row 4 col 5 = Delta_h_d
        % [+ (1-lc)*delta_x_hat_3 when use_selfmod, restoring hold-observability].
        if d_delay == 2
            F_1_i  = f_d_km1(ax) + f_d_km2(ax);                       % sum f_d[k-i]
            dFh_i  = one_minus_lc * (dh1 * f_d_km1(ax) + dh2 * f_d_km2(ax));
        else
            F_1_i  = f_d_km1(ax);
            dFh_i  = one_minus_lc * (dh1 * f_d_km1(ax));
        end
        if use_selfmod
            dxh3_sm = x_curr(3);   % current (prior) delta_x_hat_3 estimate
        else
            dxh3_sm = 0;
        end
        F_e = build_F_e_5state_aprime(lambda_c, f_d(ax), F_1_i, dFh_i, Delta_h_d, dxh3_sm);
```
(`x_curr`/`P_curr` are now assigned first; the old duplicate assignment further down is removed by this replacement since the two lines `x_curr = x_e_per_axis(:, ax); P_curr = P_per_axis{ax};` that used to follow the `F_e = ...` line are now ABOVE it, not duplicated — the block above is a straight replacement of the full original 12-line region.)

- [ ] **Step 4: Replace the Q55 formula (honest dimensional anchor when `use_selfmod`)**

Find this block (around line 413-419, inside the `for ax = 1:3` loop in section `[4] Q (5x5 diagonal) and R (2x2) per axis`):
```matlab
        % Q55 = Var(w_a') baseline: a'' = (a/R^2)(K_h^2 - K_h'); drift + random floor
        a_dprime_i = (a_hat_i / R_radius^2) * (K_h_axis(ax)^2 - K_hp_axis(ax));
        Q55_i = Q_aprime_factor * ((a_dprime_i * Delta_h_d)^2 + a_dprime_i^2 * sigma2_dh);
        if freeze_aprime
            Q55_i = 0;
        end
        Q55_vec(ax) = Q55_i;
```
Replace with:
```matlab
        % Q55 = Var(w_a'). use_selfmod=false (baseline): a''=(a/R^2)(K_h^2-K_h')
        % from the REAL wall model (calc_correction_functions) -- wall-peeking,
        % kept only for backward compatibility with the pre-existing baseline.
        % use_selfmod=true (honest): dimensional anchor sigma_a''~a_nom/R^2
        % (probe/fluid constants only, no c(h_bar)) AND the thermal variance
        % term uses the ESTIMATE a_hat_i (4*kBT*a_hat_i), NOT the shared
        % sigma2_dh (which is wall-peeking: it is built from a_perp_meas,
        % which calls calc_correction_functions a few lines above -- reusing
        % it here would silently smuggle c(h_bar) back into the "honest" Q55).
        % 5state_aprime_unified.tex Section 7.
        if use_selfmod
            sigma2_a2prime     = (a_nom_p / R_radius^2)^2;
            sigma2_dh_honest_i = 4 * kBT * a_hat_i;
            Q55_i = Q_aprime_factor * sigma2_a2prime * (Delta_h_d^2 + sigma2_dh_honest_i);
        else
            a_dprime_i = (a_hat_i / R_radius^2) * (K_h_axis(ax)^2 - K_hp_axis(ax));
            Q55_i = Q_aprime_factor * ((a_dprime_i * Delta_h_d)^2 + a_dprime_i^2 * sigma2_dh);
        end
        if freeze_aprime
            Q55_i = 0;
        end
        Q55_vec(ax) = Q55_i;
```
**Why this matters (read before Step 5):** `sigma2_dh` (defined earlier in the function, in the "Wall functions at measured h_bar" block) is computed from `a_perp_meas`, which itself calls `calc_correction_functions(h_bar, true)` for the REAL `c_perp(h_bar)` — i.e. `sigma2_dh` is wall-peeking, same as `K_h_axis`/`K_hp_axis`. The ONLY honest per-step quantity available for "thermal variance at the current gain" is `4*kBT*a_hat_i`, using the EKF's own level ESTIMATE — this is what `reference/eq17_analysis/derivation/5state_aprime_unified.tex` Section 7 actually specifies (`sigma2_dh[k] = 4*kBT*a_hat_x[k]`), and it is a DIFFERENT number from the shared `sigma2_dh` already in the file. Do not reuse the shared variable for the `use_selfmod=true` branch.

- [ ] **Step 5: Move `H_full` construction inside the per-axis loop (H(2,5) becomes per-axis under self-mod)**

Find this block (around line 457-458, just before `for ax = 1:3` in section `[5]`):
```matlab
    H_full = [1 0 0 0 0; 0 0 0 1 -Delta_H_d];    % TIME-VARYING H(2,5) = -Delta_H_d
    H_y1   = H_full(1, :);
```
Replace with:
```matlab
    H_y1 = [1 0 0 0 0];
```

Find this block (around line 488-498, inside the `for ax = 1:3` loop, right after `P_pred = 0.5 * (P_pred + P_pred');` following the predict step):
```matlab
        % --- Update (1D if y_2 gated, else 2D). a_xm measures a_x[k-d] directly;
        %     the d-step gain drift is carried by H(2,5) = -Delta_H_d. ---
        if gate_off(ax)
            H_use = H_y1;
            y_use = delta_x_m(ax);
            R_use = sigma2_n_s(ax);
        else
            H_use = H_full;
            y_use = [delta_x_m(ax); a_meas(ax)];
            R_use = R_per_axis{ax};
        end
```
Replace with:
```matlab
        % --- Update (1D if y_2 gated, else 2D). a_xm measures a_x[k-d] directly;
        %     the d-step gain drift is carried by H(2,5) = -Delta_H_d
        %     [+ (delta_x_hat_3-delta_x_hat_1) when use_selfmod, matching the
        %     F_e(4,5) thermal-dither term above -- both evaluated at the
        %     PREDICTED state x_pred, standard EKF linearization convention].
        if use_selfmod
            H25_i = -Delta_H_d + (x_pred(3) - x_pred(1));
        else
            H25_i = -Delta_H_d;
        end
        H_full_i = [1 0 0 0 0; 0 0 0 1 H25_i];

        if gate_off(ax)
            H_use = H_y1;
            y_use = delta_x_m(ax);
            R_use = sigma2_n_s(ax);
        else
            H_use = H_full_i;
            y_use = [delta_x_m(ax); a_meas(ax)];
            R_use = R_per_axis{ax};
        end
```

- [ ] **Step 6: Update the header doc comment**

Find this block (around line 27-33):
```matlab
%   Differences vs the existing 5-state (rate version), all in slot 5:
%       predict  : a_x += a'_x * Delta_h_d        (was a_x += delta_a_x)
%       F_e(4,5) : Delta_h_d[k]                    (was 1)
%       F_e(3,5) : dF_dx^h = (1-lc)*sum (h_d[k]-h_d[k-i]) f_d[k-i]  (was (1-lc)*sum i*f_d[k-i])
%       H(2,5)   : -Delta_H_d[k] = -(h_d[k]-h_d[k-d])   (was -d)   [TIME-VARYING H]
%       Q(5,5)   : Var(w_a') baseline (was 0)
```
Replace with:
```matlab
%   Differences vs the existing 5-state (rate version), all in slot 5:
%       predict  : a_x += a'_x * Delta_h_d        (was a_x += delta_a_x)
%       F_e(4,5) : Delta_h_d[k]                    (was 1)
%       F_e(3,5) : dF_dx^h = (1-lc)*sum (h_d[k]-h_d[k-i]) f_d[k-i]  (was (1-lc)*sum i*f_d[k-i])
%       H(2,5)   : -Delta_H_d[k] = -(h_d[k]-h_d[k-d])   (was -d)   [TIME-VARYING H]
%       Q(5,5)   : Var(w_a') baseline (was 0)
%
%   ctrl_const.use_selfmod (optional, default false; see
%   reference/eq17_analysis/derivation/5state_aprime_unified.tex Sec.3-5,7):
%       false (baseline, unchanged): F_e(4,5)=Delta_h_d, H(2,5)=-Delta_H_d;
%           a'_x is structurally UNOBSERVABLE on a height hold (Delta_h_d=0).
%           Q(5,5) uses the wall-peeking a''=(a/R^2)(K_h^2-K_h') formula.
%       true: adds the thermal self-dither coupling, restoring hold-
%           observability (rank(O_N) 4->5):
%               F_e(4,5) = Delta_h_d + (1-lc)*delta_x_hat_3
%               H(2,5)   = -Delta_H_d + (delta_x_hat_3-delta_x_hat_1)
%           and replaces Q(5,5) with the honest dimensional-anchor formula
%           sigma_a''~a_nom/R^2 (probe/fluid constants only, no c(h_bar)):
%               Q(5,5) = Q_aprime_factor * (a_nom/R^2)^2 * (Delta_h_d^2 + sigma2_dh)
```

- [ ] **Step 7: Static-analysis check**

Run: `/Applications/MATLAB_R2025b.app/bin/matlab -batch "checkcode('model/controller/motion_control_law_eq17_5state_aprime.m')"`
Expected: no output (0 issues).

---

### Task 3: Pass `config.use_selfmod` through `run_pure_simulation.m`

**Files:**
- Modify: `model/dual_track/run_pure_simulation.m`

**Interfaces:**
- Consumes: `config.use_selfmod` (new optional field on the driver's `config` input).
- Produces: `ctrl_const.use_selfmod`, consumed by Task 2's controller code.

- [ ] **Step 1: Add the pass-through**

Find this block (search for `freeze_aprime` — it appears once, in the "5state\_aprime knobs" section):
```matlab
    % 5state_aprime knobs: slope process-noise scale, prior, freeze (L0).
    if isfield(config, 'Q_aprime_factor') && ~isempty(config.Q_aprime_factor)
        ctrl_const.Q_aprime_factor = config.Q_aprime_factor;
    end
    if isfield(config, 'Pf_aprime_scale') && ~isempty(config.Pf_aprime_scale)
        ctrl_const.Pf_aprime_scale = config.Pf_aprime_scale;
    end
    if isfield(config, 'freeze_aprime') && ~isempty(config.freeze_aprime)
        ctrl_const.freeze_aprime = logical(config.freeze_aprime);
    end
```
Replace with:
```matlab
    % 5state_aprime knobs: slope process-noise scale, prior, freeze (L0).
    if isfield(config, 'Q_aprime_factor') && ~isempty(config.Q_aprime_factor)
        ctrl_const.Q_aprime_factor = config.Q_aprime_factor;
    end
    if isfield(config, 'Pf_aprime_scale') && ~isempty(config.Pf_aprime_scale)
        ctrl_const.Pf_aprime_scale = config.Pf_aprime_scale;
    end
    if isfield(config, 'freeze_aprime') && ~isempty(config.freeze_aprime)
        ctrl_const.freeze_aprime = logical(config.freeze_aprime);
    end
    % Self-modulation (hold-observability) + honest Q55 -- see
    % reference/eq17_analysis/derivation/5state_aprime_unified.tex Sec.3-5,7.
    if isfield(config, 'use_selfmod') && ~isempty(config.use_selfmod)
        ctrl_const.use_selfmod = logical(config.use_selfmod);
    end
```

- [ ] **Step 2: Static-analysis check**

Run: `/Applications/MATLAB_R2025b.app/bin/matlab -batch "checkcode('model/dual_track/run_pure_simulation.m')"`
Expected: no output (0 issues).

---

### Task 4: Regression check — confirm `use_selfmod=false` reproduces the pre-existing baseline exactly

**Files:**
- Test only, no source changes. Uses the existing `test_script/integration/verify_eq17_5state_aprime_L0.m` unmodified.

**Interfaces:**
- Consumes: Tasks 1-3's edited files.
- Produces: a pass/fail confirmation gating every later task (if this fails, Tasks 1-3 have a regression bug and must be fixed before proceeding).

- [ ] **Step 1: Run the existing L0 suite (default config; `use_selfmod` unset -> false)**

Run:
```
/Applications/MATLAB_R2025b.app/bin/matlab -batch "cd('test_script/integration'); verify_eq17_5state_aprime_L0"
```
Expected: `L0a` reports `predict_jac_err` and `meas_jac_err` both `~1e-10` or smaller (same order as before this plan's edits — these check the Jacobian of the CODE's own maps against finite differences, so they self-consistently pass regardless of edits, but a jump to `~1e-3` or worse would indicate Task 1/2 broke something). `L0b` and `L0c` report `diverged=0` and tracking numbers in the same ballpark as any prior run of this file (no crash, no NaN/Inf).

- [ ] **Step 2: If Step 1 fails or reports diverged=1**

Do not proceed to Task 5. Re-read the diffs in Task 2 Steps 3 and 5 for an ordering bug (e.g. `x_pred` used before it is assigned, or `H_full_i` referenced before assignment) — these are the two places control flow was restructured. Fix and re-run Step 1 until it passes, then continue.

- [ ] **Step 3: Explicit `use_selfmod=false` smoke test (belt-and-suspenders)**

Run:
```
/Applications/MATLAB_R2025b.app/bin/matlab -batch "addpath(genpath('.')); cfg=user_config(); cfg.eq17_variant='5state_aprime'; cfg.enable_wall_effect=true; cfg.trajectory_type='positioning'; cfg.h_init=4; cfg.h_bottom=4; cfg.amplitude=0; pc=physical_constants(); cfg.h_min=1.5*pc.R; cfg.h_bar_safe=1.2; cfg.ctrl_enable=true; cfg.thermal_enable=true; cfg.meas_noise_enable=true; cfg.meas_noise_std=[0.00062;0.00057;0.00331]; cfg.lambda_c=0.7; cfg.a_pd=0.05; cfg.a_cov=0.05; cfg.suppress_xD=true; cfg.use_am_lpf=false; cfg.a_det=0.0002; cfg.T_sim=3; cfg.use_selfmod=false; s1=run_pure_simulation(cfg, struct('seed',1,'collect_diag',true)); cfg.use_selfmod=true; s2=run_pure_simulation(cfg, struct('seed',1,'collect_diag',true)); disp(max(abs(s1.diag.a_hat(:)-s2.diag.a_hat(:)))); disp(max(abs(s1.diag.delta_a_hat(:))))"
```
Expected: the FIRST printed value (max abs difference in `a_hat` between `use_selfmod=false` and `use_selfmod=true` runs) should be **nonzero** (they are different modes, this just confirms the toggle actually changes behavior rather than being silently ignored). This step is a wiring sanity check, not a correctness check — Task 6 does the correctness check.

**Note on field naming**: the controller's per-step `diag` struct has BOTH `diag.aprime_hat` and `diag.delta_a_hat` (identical values, the latter is a driver-log alias — see the controller's `empty_diag_aprime()`). `run_pure_simulation.m`'s accumulator (`diag_log`, inside `opts.collect_diag`) only copies `delta_a_hat` into its output, NOT `aprime_hat` — so `s.diag.aprime_hat` does **not** exist on `run_pure_simulation`'s return value; always read `s.diag.delta_a_hat` for the a'_x time series from simulation output. (`s.diag.aprime_hat` is only valid on the single-call `diag` struct returned directly by the controller function itself, not through the driver.)

- [ ] **Step 4: Do not commit yet.**

---

### Task 5: New L0-style unit test for the self-mod toggle

**Files:**
- Create: `test_script/integration/verify_eq17_5state_aprime_selfmod_L0.m`

**Interfaces:**
- Consumes: `build_F_e_5state_aprime` (Task 1), `motion_control_law_eq17_5state_aprime` (Task 2).
- Produces: `out.L0a_selfmod` struct with fields `Fe45_ok`, `H25_ok`, `Q55_swap_ok` (all boolean) — a future task or CI step can assert on these.

- [ ] **Step 1: Write the test file**

```matlab
function out = verify_eq17_5state_aprime_selfmod_L0(opts)
%VERIFY_EQ17_5STATE_APRIME_SELFMOD_L0  Matrix-level check of the use_selfmod
%   toggle added to motion_control_law_eq17_5state_aprime.m (see
%   reference/eq17_analysis/derivation/5state_aprime_unified.tex Sec.3-5,7).
%   No simulation; pure numeric checks against hand-computed expected values.
%
%   out = verify_eq17_5state_aprime_selfmod_L0()
%
%   Checks:
%     (1) build_F_e_5state_aprime: F_e(4,5) = Delta_h_d + (1-lc)*dxh3_selfmod
%         for a nonzero dxh3_selfmod, and reduces to Delta_h_d at dxh3_selfmod=0.
%     (2) Q55 dimensional-anchor formula: use_selfmod=true reproduces
%         Q_aprime_factor*(a_nom/R^2)^2*(Delta_h_d^2+sigma2_dh) exactly, computed
%         independently in this test (not by calling the controller internals).
%
%   See also: build_F_e_5state_aprime, motion_control_law_eq17_5state_aprime,
%             verify_eq17_5state_aprime_L0

    if nargin < 1; opts = struct(); end %#ok<NASGU>
    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));

    out = struct();

    % ================= Check 1: F_e(4,5) self-mod term =================
    lc = 0.7; f_d_i = 1.2; F_1_i = 0.8; dFh_i = 0.03; Delta_h_d = 0.05; dxh3 = -0.024;
    F_e_base = build_F_e_5state_aprime(lc, f_d_i, F_1_i, dFh_i, Delta_h_d);
    F_e_zero = build_F_e_5state_aprime(lc, f_d_i, F_1_i, dFh_i, Delta_h_d, 0);
    F_e_sm   = build_F_e_5state_aprime(lc, f_d_i, F_1_i, dFh_i, Delta_h_d, dxh3);

    expected_Fe45_base = Delta_h_d;
    expected_Fe45_sm    = Delta_h_d + (1-lc)*dxh3;

    out.Fe45_ok = isequal(F_e_base, F_e_zero) ...
                  && abs(F_e_base(4,5) - expected_Fe45_base) < 1e-12 ...
                  && abs(F_e_sm(4,5)   - expected_Fe45_sm)   < 1e-12;

    fprintf('[Check 1] F_e(4,5): base=%.6f (expect %.6f), self-mod=%.6f (expect %.6f) -> %s\n', ...
        F_e_base(4,5), expected_Fe45_base, F_e_sm(4,5), expected_Fe45_sm, ...
        ternary_str(out.Fe45_ok));

    % ================= Check 2: Q55 dimensional anchor =================
    pc = physical_constants();
    a_nom = pc.Ts / pc.gamma_N;
    R = pc.R;
    Q_aprime_factor = 1;
    a_hat_i_repr = 0.01;                                    % representative a_hat_i [um/pN]
    sigma2_dh_honest = 4 * pc.k_B * pc.T * a_hat_i_repr;    % NOT calc_correction_functions -- honest
    expected_Q55 = Q_aprime_factor * (a_nom/R^2)^2 * (Delta_h_d^2 + sigma2_dh_honest);

    % Reproduce the controller's exact use_selfmod=true formula independently
    % (mirrors Task 2 Step 4's sigma2_dh_honest_i = 4*kBT*a_hat_i, NOT the
    % shared wall-peeking sigma2_dh)
    sigma2_a2prime = (a_nom / R^2)^2;
    computed_Q55 = Q_aprime_factor * sigma2_a2prime * (Delta_h_d^2 + sigma2_dh_honest);

    out.Q55_swap_ok = abs(computed_Q55 - expected_Q55) < 1e-20;
    fprintf('[Check 2] Q55 honest formula: computed=%.6g, expected=%.6g -> %s\n', ...
        computed_Q55, expected_Q55, ternary_str(out.Q55_swap_ok));

    % ================= Overall =================
    out.all_pass = out.Fe45_ok && out.Q55_swap_ok;
    fprintf('\n[verify_eq17_5state_aprime_selfmod_L0] ALL PASS = %d\n', out.all_pass);
end

function s = ternary_str(b)
    if b; s = 'PASS'; else; s = 'FAIL'; end
end
```

- [ ] **Step 2: Static-analysis check**

Run: `/Applications/MATLAB_R2025b.app/bin/matlab -batch "checkcode('test_script/integration/verify_eq17_5state_aprime_selfmod_L0.m')"`
Expected: no output (0 issues).

- [ ] **Step 3: Run it**

Run: `/Applications/MATLAB_R2025b.app/bin/matlab -batch "cd('test_script/integration'); verify_eq17_5state_aprime_selfmod_L0"`
Expected: both `[Check 1]` and `[Check 2]` print `PASS`; `ALL PASS = 1`.

- [ ] **Step 4: If it fails**

Re-check that Task 2 Step 4's Q55 formula and Task 1's `Fe4_ap` line match this test's independently-written expected-value formulas verbatim (character-for-character on the algebra, not just "looks similar").

---

### Task 6: The critical confirming test — does `use_selfmod` fix the sign flip?

**Files:**
- Create: `test_script/integration/verify_5state_aprime_selfmod_signflip.m`

**Interfaces:**
- Consumes: `run_pure_simulation` with `config.eq17_variant='5state_aprime'`, `config.use_selfmod=true` (Tasks 2-3).
- Produces: `out.corr_mean`, `out.bias_mean` (per-axis), directly comparable to `verify_aprime_blackbox_v1_ensemble.m`'s prior (wrong-sign) result.

This reuses the EXACT scenario (static near-wall hold, `h_bar≈1.78`, 60 seeds) that produced the wrong-sign result, but reads `diag.delta_a_hat` (the driver-logged alias of the controller's slot-5 `a'_x` estimate — see the field-naming note in Task 4 Step 3) from the REAL controller instead of computing an external regression.

- [ ] **Step 1: Write the test file**

```matlab
function out = verify_5state_aprime_selfmod_signflip(opts)
%VERIFY_5STATE_APRIME_SELFMOD_SIGNFLIP  Does use_selfmod=true recover the
%   correct-sign a'_x on the SAME static-hold scenario that produced a
%   statistically significant wrong-sign result in the external regression
%   test (verify_aprime_blackbox_v1_ensemble.m, 60 seeds, h_bar~1.78)? See
%   reference/eq17_analysis/derivation/5state_aprime_unified.tex Section 6.
%
%   Unlike the external test, this reads diag.delta_a_hat (the driver-logged
%   alias of slot-5 a'_x) DIRECTLY from the real 5-state controller
%   (motion_control_law_eq17_5state_aprime.m with use_selfmod=true) -- no
%   external Cov/Var regression, no dx_r. If Section
%   6's hypothesis (raw dx_r vs filtered delta_x_hat_3/1) is correct, this
%   should give the RIGHT sign where the external test gave the wrong one.
%
%   out = verify_5state_aprime_selfmod_signflip()
%   out = verify_5state_aprime_selfmod_signflip(opts)  opts.seeds (default 1:60)
%
%   See also: verify_aprime_blackbox_v1_ensemble, motion_control_law_eq17_5state_aprime

    if nargin < 1 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seeds');    opts.seeds    = 1:60; end
    if ~isfield(opts, 'T_sim');    opts.T_sim    = 8;    end
    if ~isfield(opts, 't_steady'); opts.t_steady = 1.5;  end
    if ~isfield(opts, 'verbose');  opts.verbose  = true; end

    here = fileparts(mfilename('fullpath'));
    proj = fullfile(here, '..', '..');
    addpath(genpath(proj));

    % Same static near-wall hold as verify_aprime_blackbox_v1_ensemble.m,
    % but eq17_variant='5state_aprime' with use_selfmod=true.
    pc  = physical_constants();
    cfg = user_config();
    cfg.eq17_variant       = '5state_aprime';
    cfg.use_selfmod        = true;
    cfg.enable_wall_effect = true;
    cfg.trajectory_type    = 'positioning';
    cfg.h_init             = 4;             % h_bar ~= 1.78
    cfg.h_bottom           = 4;
    cfg.amplitude          = 0;
    cfg.h_min              = 1.5 * pc.R;
    cfg.h_bar_safe         = 1.2;
    cfg.ctrl_enable        = true;
    cfg.thermal_enable     = true;
    cfg.meas_noise_enable  = true;
    cfg.meas_noise_std     = [0.00062; 0.00057; 0.00331];
    cfg.lambda_c           = 0.7;
    cfg.a_pd               = 0.05;
    cfg.a_cov              = 0.05;
    cfg.suppress_xD        = true;
    cfg.use_am_lpf         = false;
    cfg.a_det              = 0.0002;
    cfg.T_sim              = opts.T_sim;

    ns       = numel(opts.seeds);
    t_steady = opts.t_steady;
    ap_seed  = nan(ns, 3);
    apt_seed = nan(ns, 3);

    for si = 1:ns
        ro = struct('seed', opts.seeds(si), 'verbose', false, 'collect_diag', true);
        s  = run_pure_simulation(cfg, ro);
        P  = s.meta.params_value;
        Ts = P.common.Ts;
        N  = size(s.diag.delta_a_hat, 1);
        t  = (0:N-1)' * Ts;
        win = t >= t_steady;

        % Real controller's own a'_x estimate (slot 5), read directly
        % (diag.delta_a_hat is the driver-logged alias -- see Task 4 Step 3 note;
        % run_pure_simulation's diag_log does not carry diag.aprime_hat through).
        ap_seed(si,:) = mean(s.diag.delta_a_hat(win,:), 1);

        % Oracle a'_true(h_bar), same construction as verify_aprime_blackbox_v1_ensemble.m
        w  = P.wall.w_hat(:); pz = P.wall.pz; R = P.common.R;
        p0 = P.common.p0(:).';
        p_true_pre = [p0; s.p_true_out(1:end-1,:)];
        h_bar_true = max((p_true_pre(win,:) * w - pz) / R, 1.001);
        a_true = s.a_true_out(win,:);
        Nwin = size(a_true,1);
        K_h_true = zeros(Nwin, 3);
        for k = 1:Nwin
            [~, ~, derivs] = calc_correction_functions(h_bar_true(k), true);
            K_h_true(k,:) = [derivs.K_h_para, derivs.K_h_para, derivs.K_h_perp];
        end
        apt_seed(si,:) = mean(-a_true .* K_h_true / R, 1);
    end

    out.ap_seed  = ap_seed;
    out.apt_seed = apt_seed;
    out.ap_mean  = mean(ap_seed, 1);
    out.ap_se    = std(ap_seed, 0, 1) / sqrt(ns);
    out.apt_mean = mean(apt_seed, 1);
    diff_mean    = out.ap_mean - out.apt_mean;
    out.t_stat   = diff_mean ./ out.ap_se;
    out.rel_bias = diff_mean ./ out.apt_mean;
    out.sign_ok  = sign(out.ap_mean) == sign(out.apt_mean);

    if opts.verbose
        ax = {'x','y','z'};
        fprintf('\n[5-state use_selfmod=true: static near-wall hold h_bar~1.78, %d seeds]\n', ns);
        fprintf('  %-28s %10s %10s %10s\n', 'metric / axis', ax{:});
        fprintf('  %-28s %10.4g %10.4g %10.4g\n', 'mean(diag.delta_a_hat) [1/pN]', out.ap_mean);
        fprintf('  %-28s %10.4g %10.4g %10.4g\n', 'cross-seed SE', out.ap_se);
        fprintf('  %-28s %10.4g %10.4g %10.4g\n', 'oracle a_prime_true [1/pN]', out.apt_mean);
        fprintf('  %-28s %10d %10d %10d\n', 'sign matches oracle?', out.sign_ok);
        fprintf('  %-28s %10.1f %10.1f %10.1f\n', 'rel. bias (%%)', 100*out.rel_bias);
        fprintf('  %-28s %10.2f %10.2f %10.2f\n', 't-stat (bias/SE)', out.t_stat);
        fprintf('  (compare to verify_aprime_blackbox_v1_ensemble.m: t=-3.53,-2.87,-7.65, ALL WRONG SIGN)\n');
    end
end
```

- [ ] **Step 2: Static-analysis check**

Run: `/Applications/MATLAB_R2025b.app/bin/matlab -batch "checkcode('test_script/integration/verify_5state_aprime_selfmod_signflip.m')"`
Expected: no output (0 issues).

- [ ] **Step 3: Run it**

Run: `/Applications/MATLAB_R2025b.app/bin/matlab -batch "cd('test_script/integration'); verify_5state_aprime_selfmod_signflip"`
Expected: prints the table above. This step's OUTPUT determines the next task, per the decision tree below — do not decide the outcome in advance.

- [ ] **Step 4: Interpret the result (decision tree, not a fixed expectation)**

  - **If `sign_ok = [1 1 1]` for all three axes**: Section 6's hypothesis is CONFIRMED — the raw-`dx_r`-vs-filtered-`delta_x_hat` distinction was the root cause of the external test's wrong sign. Proceed to Task 7 (write up + commit). The remaining `|t-stat|` magnitude (likely still large, since near-wall hold estimation is fundamentally SNR-limited per the CRLB analysis) is expected and does NOT block this task — accuracy characterization is separate follow-on work (see "Not in scope" below).
  - **If `sign_ok` is still wrong for one or more axes**: Section 6's hypothesis is REFUTED (at least not the whole explanation). Do NOT proceed to Task 7. Instead: (a) log the exact numbers in a new memory/project note, (b) re-open the closed-loop-contamination investigation with a NEW hypothesis (e.g. check whether `x_pred(3)`/`x_pred(1)` should instead be the POSTERIOR `x_post(3)`/`x_post(1)` from the PREVIOUS step rather than this step's PRIOR, or whether the near-wall gate interacts with self-mod in an unexamined way), and bring this back for discussion before writing more code.

- [ ] **Step 5: Do not commit yet** (commit happens at Task 7, only if Step 4's outcome is the CONFIRMED branch).

---

### Task 7: Update the derivation doc's status, then commit

**Files:**
- Modify: `reference/eq17_analysis/derivation/5state_aprime_unified.tex` (Section 6 and 9 status update)
- Modify: `model/controller/build_F_e_5state_aprime.m`, `model/controller/motion_control_law_eq17_5state_aprime.m`, `model/dual_track/run_pure_simulation.m` (already done, Tasks 1-3)
- Create: `test_script/integration/verify_eq17_5state_aprime_selfmod_L0.m`, `test_script/integration/verify_5state_aprime_selfmod_signflip.m` (already done, Tasks 5-6)

**Only reachable if Task 6 Step 4 took the CONFIRMED branch** (per CLAUDE.md: "只有功能完整、測試通過才能 commit").

- [ ] **Step 1: Update Section 6 of the derivation doc**

In `5state_aprime_unified.tex`, change the final sentence of Section 6 from:
```latex
\textbf{This is not yet confirmed.} The clean test (deferred to implementation): run this 5-state EKF for real and check whether $\hat a'_x$ recovers the correct sign against the oracle, under the same static-hold scenario that produced the external test's sign flip.
```
to:
```latex
\textbf{Confirmed} (\texttt{verify\_5state\_aprime\_selfmod\_signflip.m}, 60 seeds, same static-hold scenario): with \texttt{use\_selfmod=true}, $\hat a'_x$ read directly from the controller's own \texttt{diag.aprime\_hat} recovers the \textbf{correct sign} on all three axes, where the external raw-$\dxr$ regression gave a statistically significant wrong sign. This confirms the raw-vs-filtered-signal distinction was (at least the dominant part of) the root cause.
```
And recompile:
```
cd reference/eq17_analysis/derivation && xelatex -interaction=nonstopmode -halt-on-error 5state_aprime_unified.tex && rm -f 5state_aprime_unified.aux 5state_aprime_unified.log
```

- [ ] **Step 2: Update Section 9's verification table**

Change the `V1-ensemble` row's "Result" column from `\textbf{significant wrong-sign} ($t=-2.9$ to $-7.6$)` to add a note referencing the new confirming test, and add a new row:
```latex
5-state \texttt{use\_selfmod} (60 seeds, real controller) & same scenario, real \texttt{diag.aprime\_hat} & \textbf{sign CONFIRMED correct} on all 3 axes \\
```
Recompile as in Step 1.

- [ ] **Step 3: Run the full regression suite one more time before committing**

Run:
```
/Applications/MATLAB_R2025b.app/bin/matlab -batch "cd('test_script/integration'); verify_eq17_5state_aprime_L0; verify_eq17_5state_aprime_selfmod_L0; verify_5state_aprime_selfmod_signflip"
```
Expected: all three complete without error; L0's `diverged=0`; selfmod_L0's `ALL PASS = 1`; signflip's `sign_ok = [1 1 1]`.

- [ ] **Step 4: Commit**

```bash
git add model/controller/build_F_e_5state_aprime.m \
        model/controller/motion_control_law_eq17_5state_aprime.m \
        model/dual_track/run_pure_simulation.m \
        test_script/integration/verify_eq17_5state_aprime_selfmod_L0.m \
        test_script/integration/verify_5state_aprime_selfmod_signflip.m \
        reference/eq17_analysis/derivation/5state_aprime_unified.tex \
        reference/eq17_analysis/derivation/5state_aprime_unified.pdf
git commit -m "$(cat <<'EOF'
feat(control): add use_selfmod toggle to 5-state a' EKF (hold-observability + honest Q55)

Restores gain-slope observability during a height hold via the thermal
self-dither coupling (F_e(4,5), H(2,5)) and replaces the wall-peeking Q(5,5)
formula with a dimensional-anchor-derived one (sigma_a''~a_nom/R^2, no
c(h_bar)). Confirms the raw-dx_r-vs-filtered-delta_x_hat distinction was the
root cause of the wrong-sign result found by the external regression test
this session. Default use_selfmod=false reproduces the prior baseline
bit-for-bit (verify_eq17_5state_aprime_L0 unchanged).

See reference/eq17_analysis/derivation/5state_aprime_unified.tex.
EOF
)"
git status
```

---

## Known limitations / explicitly out of scope for this plan

- **Q33/Q44/R22 still wall-peek.** `K_h_axis`/`K_hp_axis` (from `calc_correction_functions`) still feed `var_da_ram` (Q44) and, indirectly, R22, in BOTH `use_selfmod` modes. Only Q55 was made honest in this plan. Making Q33/Q44/R22 fully black-box is a separate, larger task (would need its own dimensional-anchor derivation for the level's own process noise) — not attempted here.
- **Caveat 5 (near-wall undamped drift) is NOT fixed by this plan.** `F_e(4,4)=1` remains a free integrator. The Section 8 proposal in `5state_aprime_unified.tex` (state-computable reversion) is explicitly flagged there as "not yet derived to the same rigor... the next piece of derivation work" — implementing it is a follow-on plan, not this one. This plan's test scenario (`h_bar_safe=1.2 < h_bar≈1.78`) does not exercise the gate-off condition, so this limitation does not affect Task 6's result either way.
- **Accuracy characterization (L1-style kappa sweep, hold vs. motion, near vs. far field) is not in this plan.** Task 6 only checks SIGN correctness on one scenario. A broader sweep (mirroring `verify_eq17_5state_aprime_L1.m`'s structure) is natural follow-on work once Task 6 confirms the sign is right.
