# Gain Compare A/B Experiment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the gain-compare A/B experiment per `reference/eq17_analysis/gain_compare_design.md` — compare 6-state controller performance with use_true_gain vs EKF-estimated motion gain, det/ram decomposition, aggressive gate-crossing trajectory, {1,2,5} Hz sweep.

**Architecture:** Two backward-compatible modifications (6-state controller gains an optional `a_ctrl_override` 6th argument + `suppress_xD` support; pure-MATLAB driver gains `opts.use_true_gain` + `a_true_out` logging), plus two new scripts (runner `compare_gain_6state.m`, analyzer `analyze_gain_6state.m`) and one unit test. Runner and analyzer are separate files so results can be re-analyzed without re-running.

**Tech Stack:** MATLAB R2025b, pure-MATLAB dual-track path (`run_pure_simulation`), Bash batch execution (`/Applications/MATLAB_R2025b.app/bin/matlab -batch`). Worktree: `/Users/kevin/Code/MotionControl_Simu-motion-test`, branch `test/motion-test`.

**Conventions:**
- All commands run from the worktree root. `MB="/Applications/MATLAB_R2025b.app/bin/matlab"`.
- Every new/modified `.m` must pass `checkcode` with 0 issues.
- Commit format: `<type>(<scope>): <subject>` + `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>` trailer.
- Do NOT touch the user's interactive MATLAB/MCP session (it points at the main checkout).

---

### Task 0: Pre-flight plumbing verification (no code changes)

Verified during design (recorded here so the executor doesn't re-derive):
- `calc_traj_params.m:23-27` already plumbs `config.t_descend_override` → `traj.t_descend_override` (0 = default 1/f). `trajectory_generator.m:91-95` consumes it.
- `config.h_min` → `calc_wall_params.m:21-22` → `wall.h_min`. Only `test_script/run_simulation.m:110` (Simulink path) enforces it; the pure driver does not. The runner therefore calls `check_trajectory_safety` itself as a pre-flight (Task 4).
- 6-state dispatch: `config.eq17_variant='6state'` (driver line 165); prefill defaults give `warmup_count=0` (no legacy warmup branch in normal runs).
- Driver log convention: `p_d_out[k]` is at t_k, `p_true_out[k]`/`p_m_out[k]` are post-integration (t_k+Ts) → analyzer aligns `e[k] = p_d_out[k+1] − p_true_out[k]`.

- [ ] **Step 0.1: Confirm worktree state**

Run: `git -C /Users/kevin/Code/MotionControl_Simu-motion-test status --porcelain && git -C /Users/kevin/Code/MotionControl_Simu-motion-test log --oneline -1`
Expected: clean tree, HEAD = `8255c70 docs(eq17): gain-compare A/B experiment design ...`

---

### Task 1: Unit test for controller override + suppress_xD (TDD — write failing test first)

**Files:**
- Create: `test_script/unit_tests/verify_eq17_unit_gain_override_6state.m`
- Modify: `reference/eq17_analysis/gain_compare_design.md` (§4.4 filename `test_gain_override_6state.m` → `verify_eq17_unit_gain_override_6state.m`, project naming convention)

- [ ] **Step 1.1: Write the unit test**

```matlab
function verify_eq17_unit_gain_override_6state()
%VERIFY_EQ17_UNIT_GAIN_OVERRIDE_6STATE Unit tests for the optional
%   a_ctrl_override (6th arg) + ctrl_const.suppress_xD support in
%   motion_control_law_eq17_6state (gain-compare A/B experiment, see
%   reference/eq17_analysis/gain_compare_design.md §4.1).
%
%   T1: backward-compat — 5-arg call vs 6-arg call with [] give
%       bit-identical f_d and a_hat over 40 steps.
%   T2: override wiring — first post-init step with constant override
%       a_ovr and pure measurement offset e0 gives
%       f_d = (1-lc)*e0./a_ovr exactly (law uses a_ovr, not a_hat).
%   T3: suppress_xD + shadow reconstruction — 30 steps with override and
%       suppress_xD=true; f_d matches a shadow re-implementation of the
%       law WITHOUT the x_D term at every step.
%   T4: xD timing — suppress_xD=false: f_d matches shadow law using the
%       PREVIOUS step's diag.x_D_hat (posterior[k-1] enters the law).
%   T5: diag.a_ctrl_used — override mode returns a_ovr; normal mode
%       returns previous step's posterior a_hat (one-step lag).

    fprintf('=== verify_eq17_unit_gain_override_6state ===\n');

    this_dir  = fileparts(mfilename('fullpath'));
    repo_root = fullfile(this_dir, '..', '..');
    addpath(fullfile(repo_root, 'model'));
    addpath(fullfile(repo_root, 'model', 'config'));
    addpath(fullfile(repo_root, 'model', 'controller'));
    addpath(fullfile(repo_root, 'model', 'wall_effect'));

    [P, cc] = build_mocks_6state(false);
    lc  = P.ctrl.lambda_c;
    p0  = P.common.p0;

    % Wall-aware init gain (mirrors controller section 0E)
    a_nom = P.ctrl.Ts / P.ctrl.gamma;
    [cpa0, cpe0] = calc_correction_functions(p0(3) / P.common.R);
    a_init = [a_nom / cpa0; a_nom / cpa0; a_nom / cpe0];

    n_pass = 0;

    % ---------------- T1: backward compat (5-arg vs 6-arg []) ----------
    rng(7);
    M = 40;
    p_m_seq = repmat(p0, 1, M) + 1e-3 * randn(3, M);   % small walk around p0

    clear motion_control_law_eq17_6state;
    f_a = zeros(3, M); a_a = zeros(3, M);
    for k = 1:M
        [f, ~, dg] = motion_control_law_eq17_6state(zeros(3,1), p0, p_m_seq(:,k), P, cc);
        f_a(:,k) = f; a_a(:,k) = dg.a_hat;
    end
    clear motion_control_law_eq17_6state;
    f_b = zeros(3, M); a_b = zeros(3, M);
    for k = 1:M
        [f, ~, dg] = motion_control_law_eq17_6state(zeros(3,1), p0, p_m_seq(:,k), P, cc, []);
        f_b(:,k) = f; a_b(:,k) = dg.a_hat;
    end
    assert(max(abs(f_a(:) - f_b(:))) == 0, 'T1: f_d not bit-identical');
    assert(max(abs(a_a(:) - a_b(:))) == 0, 'T1: a_hat not bit-identical');
    n_pass = n_pass + 1; fprintf('T1 PASS backward-compat bit-identical\n');

    % ---------------- T2: override used in the law ---------------------
    a_ovr = 2 * a_init;
    e0 = [0.01; 0.01; 0.01];                            % [um] measurement offset
    clear motion_control_law_eq17_6state;
    motion_control_law_eq17_6state(zeros(3,1), p0, p0, P, cc, a_ovr);   % init call
    [f2, ~, dg2] = motion_control_law_eq17_6state(zeros(3,1), p0, p0 - e0, P, cc, a_ovr);
    f2_expect = (1 - lc) * e0 ./ a_ovr;   % pd const -> traj terms 0; hist 0; xD 0
    assert(max(abs(f2 - f2_expect) ./ abs(f2_expect)) < 1e-12, ...
           'T2: f_d does not match (1-lc)*e0./a_ovr');
    assert(max(abs(dg2.a_ctrl_used - a_ovr)) == 0, 'T2: a_ctrl_used ~= a_ovr');
    n_pass = n_pass + 1; fprintf('T2 PASS override wiring exact\n');

    % ---------------- T3: shadow law, suppress_xD = true ---------------
    cc_sup = cc; cc_sup.suppress_xD = true;
    rng(11);
    M3 = 30;
    p_m_seq3 = repmat(p0, 1, M3) + 2e-3 * randn(3, M3);
    clear motion_control_law_eq17_6state;
    motion_control_law_eq17_6state(zeros(3,1), p0, p0, P, cc_sup, a_ovr);  % init
    f_km1 = zeros(3,1); f_km2 = zeros(3,1);
    for k = 1:M3
        dxm_k = p0 - p_m_seq3(:,k);                     % pd_km2 stays p0 (positioning)
        sum_past = a_ovr .* f_km1 + a_ovr .* f_km2;     % constant override history
        f_exp = (1 ./ a_ovr) .* ((1-lc)*dxm_k - (1-lc)*sum_past);   % NO xD term
        f_act = motion_control_law_eq17_6state(zeros(3,1), p0, p_m_seq3(:,k), P, cc_sup, a_ovr);
        assert(max(abs(f_act - f_exp)) < 1e-10 * max(1, max(abs(f_exp))), ...
               sprintf('T3: shadow mismatch at k=%d', k));
        f_km2 = f_km1; f_km1 = f_act;
    end
    n_pass = n_pass + 1; fprintf('T3 PASS suppress_xD shadow law (30 steps)\n');

    % ---------------- T4: xD timing (suppress off, prior enters law) ---
    rng(13);
    M4 = 30;
    p_m_seq4 = repmat(p0, 1, M4) + 2e-3 * randn(3, M4);
    clear motion_control_law_eq17_6state;
    [~, ~, dg] = motion_control_law_eq17_6state(zeros(3,1), p0, p0, P, cc, a_ovr); % init
    xD_prev = dg.x_D_hat;                               % posterior after init = 0
    f_km1 = zeros(3,1); f_km2 = zeros(3,1);
    for k = 1:M4
        dxm_k = p0 - p_m_seq4(:,k);
        sum_past = a_ovr .* f_km1 + a_ovr .* f_km2;
        f_exp = (1 ./ a_ovr) .* ((1-lc)*dxm_k - (1-lc)*sum_past - xD_prev);
        [f_act, ~, dg] = motion_control_law_eq17_6state(zeros(3,1), p0, p_m_seq4(:,k), P, cc, a_ovr);
        assert(max(abs(f_act - f_exp)) < 1e-10 * max(1, max(abs(f_exp))), ...
               sprintf('T4: xD-timing shadow mismatch at k=%d', k));
        xD_prev = dg.x_D_hat;                           % this step's posterior -> next law
        f_km2 = f_km1; f_km1 = f_act;
    end
    n_pass = n_pass + 1; fprintf('T4 PASS xD prior-timing shadow law (30 steps)\n');

    % ---------------- T5: a_ctrl_used in normal (EKF) mode -------------
    rng(17);
    M5 = 20;
    p_m_seq5 = repmat(p0, 1, M5) + 2e-3 * randn(3, M5);
    clear motion_control_law_eq17_6state;
    [~, ~, dg] = motion_control_law_eq17_6state(zeros(3,1), p0, p0, P, cc);  % init
    a_hat_prev = dg.a_hat;                              % = a_init
    for k = 1:M5
        [~, ~, dg] = motion_control_law_eq17_6state(zeros(3,1), p0, p_m_seq5(:,k), P, cc);
        assert(max(abs(dg.a_ctrl_used - a_hat_prev)) == 0, ...
               sprintf('T5: a_ctrl_used ~= posterior[k-1] at k=%d', k));
        a_hat_prev = dg.a_hat;
    end
    n_pass = n_pass + 1; fprintf('T5 PASS a_ctrl_used = posterior[k-1] (normal mode)\n');

    fprintf('=== ALL %d/5 PASS ===\n', n_pass);
end


function [P, cc] = build_mocks_6state(~)
%BUILD_MOCKS_6STATE Minimal params + ctrl_const (mirrors 3guard test mocks).
    R = 2.25; gamma_N = 0.0425; Ts = 1/1600;
    k_B = 1.3806503e-5; T_K = 310.15; kBT = k_B * T_K;
    sigma2_n_s = [0.00062; 0.00057; 0.00331].^2;

    P.ctrl.enable = 1;       P.ctrl.lambda_c = 0.7;
    P.ctrl.gamma = gamma_N;  P.ctrl.Ts = Ts;
    P.ctrl.k_B = k_B;        P.ctrl.T = T_K;
    P.ctrl.sigma2_noise = sigma2_n_s;
    P.common.R = R; P.common.Ts = Ts; P.common.gamma_N = gamma_N;
    P.common.p0 = [0; 0; 50];
    P.wall.w_hat = [0; 0; 1]; P.wall.pz = 0; P.wall.enable_wall_effect = 1;
    P.thermal = struct('enable', 0);

    opts.lambda_c = 0.7;  opts.sigma2_n_s = sigma2_n_s;  opts.kBT = kBT;
    opts.a_cov = 0.05;    opts.a_pd = 0.05;              opts.d = 2;
    opts.t_warmup_kf = 0; opts.h_bar_safe = 1.5;
    cc = build_eq17_6state_constants(opts);
end
```

- [ ] **Step 1.2: Run the test — expect FAIL**

Run: `MB="/Applications/MATLAB_R2025b.app/bin/matlab"; "$MB" -batch "cd('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/unit_tests'); verify_eq17_unit_gain_override_6state"`
Expected: FAIL — `Too many input arguments` at the first 6-arg call (controller signature has 5 inputs), or missing `diag.a_ctrl_used` field.

- [ ] **Step 1.3: Update spec §4.4 filename** in `reference/eq17_analysis/gain_compare_design.md`: replace `test_script/unit_tests/test_gain_override_6state.m` with `test_script/unit_tests/verify_eq17_unit_gain_override_6state.m` (both in §4.4 heading and §11 acceptance table).

- [ ] **Step 1.4: Commit (test red)**

```bash
git add test_script/unit_tests/verify_eq17_unit_gain_override_6state.m reference/eq17_analysis/gain_compare_design.md
git commit -m "test(eq17): failing unit test for 6-state a_ctrl_override + suppress_xD

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 2: Controller changes (make Task 1 pass)

**Files:**
- Modify: `model/controller/motion_control_law_eq17_6state.m`

All edits below; line numbers refer to the current file (HEAD 8255c70).

- [ ] **Step 2.1: Signature + override normalization** (line 1)

```matlab
function [f_d, ekf_out, diag] = motion_control_law_eq17_6state(del_pd, pd, p_m, params, ctrl_const, a_ctrl_override)
```

Immediately after the help block (before the open-loop bypass section):

```matlab
    % ------------------------------------------------------------------
    % Optional gain override (gain-compare A/B experiment): when non-empty,
    % the CONTROL LAW uses a_ctrl_override (and its history) instead of the
    % EKF a_hat. The EKF itself is untouched (slot 5 still estimated).
    % ------------------------------------------------------------------
    if nargin < 6
        a_ctrl_override = [];
    end
    has_override = ~isempty(a_ctrl_override);
    if has_override
        a_ctrl_override = a_ctrl_override(:);
    end
```

- [ ] **Step 2.2: New persistents** (after line 66 `persistent var_da_ram_km1 var_da_ram_km2`):

```matlab
    persistent a_ctrl_km1 a_ctrl_km2   % control-law gain history (= a_hat history unless override)
    persistent suppress_xD_flag        % ctrl_const.suppress_xD (cached at init)
```

- [ ] **Step 2.3: Init additions**

In section 0B (after the `var_da_inc_factor` block, ~line 105):

```matlab
        suppress_xD_flag = isfield(ctrl_const, 'suppress_xD') && ctrl_const.suppress_xD;
```

In section 0I (after `a_hat_km1 = a_x_init; a_hat_km2 = a_x_init;`, line 187):

```matlab
        a_ctrl_km1 = a_x_init;            a_ctrl_km2 = a_x_init;
```

In section 0L first-call diag (after `diag.sigma2_dxr_hat = sigma2_dxr_hat;`, line 201):

```matlab
            diag.a_ctrl_used   = a_x_init;
```

- [ ] **Step 2.4: Per-step gain/xD selection** — after `xD_comb  = x_e_per_axis(4, :).';` (line 210):

```matlab
    if has_override
        a_ctrl = a_ctrl_override;        % use_true_gain (or externally supplied) gain
    else
        a_ctrl = a_hat;                  % normal mode: EKF posterior[k-1]
    end
    if suppress_xD_flag
        xD_for_ctrl = zeros(3, 1);       % -x_D term removed from the law (EKF still estimates it)
    else
        xD_for_ctrl = xD_comb;
    end
```

- [ ] **Step 2.5: Warmup branch bookkeeping** (legacy mode only) — after `a_hat_km2 = a_hat_km1; a_hat_km1 = a_hat;` (line 270):

```matlab
        a_ctrl_km2 = a_ctrl_km1; a_ctrl_km1 = a_ctrl;
```

and in the warmup diag block (after `diag.f_d = f_d; diag.a_hat = a_hat_post;`, line 278):

```matlab
            diag.a_ctrl_used = a_ctrl;
```

- [ ] **Step 2.6: Control law [3]** — replace lines 294-303 with:

```matlab
    if d_delay == 2
        sum_a_fd_past = a_ctrl_km1 .* f_d_km1 + a_ctrl_km2 .* f_d_km2;
    else
        sum_a_fd_past = a_ctrl_km1 .* f_d_km1;
    end
    inv_a_ctrl = 1 ./ a_ctrl;
    f_d = inv_a_ctrl .* (pd_kp1 - lambda_c * pd - one_minus_lc * pd_km_d ...
                        + one_minus_lc * delta_x_m ...
                        - one_minus_lc * sum_a_fd_past ...
                        - xD_for_ctrl);
```

(Backward compat: normal mode has `a_ctrl == a_hat` and the two history buffer pairs hold identical values, so output is bit-identical to the old `a_hat_km*` form. Q33 in section [4] keeps using `a_hat_km1/km2` — EKF untouched.)

- [ ] **Step 2.7: Bookkeeping [6]** — after `a_hat_km2 = a_hat_km1; a_hat_km1 = a_hat;` (line 459):

```matlab
    a_ctrl_km2 = a_ctrl_km1; a_ctrl_km1 = a_ctrl;
```

- [ ] **Step 2.8: Output diag [7]** — in the `nargout >= 3` block add (next to `diag.a_hat = a_hat_post;`):

```matlab
        diag.a_ctrl_used          = a_ctrl;
```

and in `empty_diag_6state()` add:

```matlab
    d.a_ctrl_used          = zeros(3, 1);
```

- [ ] **Step 2.9: Run unit test — expect PASS**

Run: `"$MB" -batch "cd('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/unit_tests'); verify_eq17_unit_gain_override_6state"`
Expected: `=== ALL 5/5 PASS ===`

- [ ] **Step 2.10: checkcode clean**

Run: `"$MB" -batch "r = checkcode('/Users/kevin/Code/MotionControl_Simu-motion-test/model/controller/motion_control_law_eq17_6state.m'); if isempty(r), disp('CLEAN'), else, disp(r), error('checkcode issues'), end"`
Expected: `CLEAN`

- [ ] **Step 2.11: h50 regression (behavioral backward-compat)**

Run: `"$MB" -batch "cd('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/integration'); r = verify_eq17_6state('h50'); assert(r.pass == true)"`
Expected: `OVERALL: PASS` (tracking std < 40 nm, a_hat bias/rel-std < 5%, |δx_D^d| < 0.01 — same gates as before the change). Runtime ~1-3 min.

- [ ] **Step 2.12: Commit**

```bash
git add model/controller/motion_control_law_eq17_6state.m
git commit -m "feat(eq17): 6-state a_ctrl_override (6th arg) + suppress_xD support

Control law can take an externally supplied gain (use_true_gain A/B experiment)
with its own history buffers; EKF untouched. suppress_xD zeroes the -x_D
term in the law only. Backward-compatible (bit-identical without either).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 3: Driver changes (`opts.use_true_gain` + `a_true_out`)

**Files:**
- Modify: `model/dual_track/run_pure_simulation.m`

- [ ] **Step 3.1: opts default** (after line 58 `collect_diag` default):

```matlab
    % Gain-use_true_gain A/B: feed true time-varying gain to the 6-state control law
    if ~isfield(opts, 'use_true_gain'); opts.use_true_gain = false;              end
```

- [ ] **Step 3.2: Guard after dispatch flag** (after line 165 `is_6state = ...`):

```matlab
    if opts.use_true_gain && ~is_6state
        error('run_pure_simulation:useTrueGainUnsupported', ...
              'opts.use_true_gain=true requires config.eq17_variant=''6state''.');
    end
```

- [ ] **Step 3.3: True-gain constants + log allocation** (in section 7, next to `p_true_out` allocation, line 234):

```matlab
    a_true_out = zeros(N, 3);    % ground-truth gain at controller-call position [um/pN]
```

and before the loop (after section 6 init):

```matlab
    % Gain-use_true_gain support: a_true at the PRE-integration p_curr (the position
    % the controller acts on at step k; p_true_out logs post-integration).
    a_nom_drv   = P.common.Ts / P.common.gamma_N;
    wall_on_drv = isfield(P, 'wall') && P.wall.enable_wall_effect > 0.5;
```

In the `opts.collect_diag` allocation block add:

```matlab
        diag_log.a_ctrl_used       = zeros(N, 3);
```

- [ ] **Step 3.4: Per-step use_true_gain gain + dispatch** — insert between step (c) and (d) in the loop:

```matlab
        % --- (c2) Ground-truth gain at current (pre-integration) position
        if wall_on_drv
            h_bar_true_k = max((dot(p_curr, P.wall.w_hat) - P.wall.pz) / P.common.R, 1.001);
            [c_para_k, c_perp_k] = calc_correction_functions(h_bar_true_k);
            a_true_k = [a_nom_drv / c_para_k; a_nom_drv / c_para_k; a_nom_drv / c_perp_k];
        else
            a_true_k = a_nom_drv * ones(3, 1);
        end
        if opts.use_true_gain
            a_override_k = a_true_k;
        else
            a_override_k = [];
        end
```

Change the two 6-state dispatch calls (lines 283-287) to pass the 6th argument:

```matlab
            if opts.collect_diag
                [f_d_k, ekf_k, diag_k] = motion_control_law_eq17_6state( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            else
                [f_d_k, ekf_k] = motion_control_law_eq17_6state( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            end
```

(7-state branch unchanged — never receives a 6th argument.)

- [ ] **Step 3.5: Logging** — in section (j) add `a_true_out(k, :) = a_true_k.';` and in the collect_diag block add `diag_log.a_ctrl_used(k, :) = diag_k.a_ctrl_used.';`. In section 9 pack add `simOut.a_true_out = a_true_out;`.

- [ ] **Step 3.6: Smoke verify (use_true_gain wiring end-to-end)**

Run:
```bash
"$MB" -batch "cd('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/integration'); \
addpath('../../model','../../model/config','../../model/wall_effect','../../model/thermal_force','../../model/trajectory','../../model/controller','../../model/dual_track'); \
c = user_config(); c.eq17_variant='6state'; c.trajectory_type='positioning'; c.h_init=50; c.h_bottom=50; c.amplitude=0; c.T_sim=0.3; c.ctrl_enable=true; c.thermal_enable=true; c.meas_noise_enable=true; c.lambda_c=0.7; c.a_pd=0.05; c.a_cov=0.05; c.meas_noise_std=[0.00062;0.00057;0.00331]; \
oA = run_pure_simulation(c, struct('seed',1,'collect_diag',true,'use_true_gain',true)); \
assert(max(abs(oA.diag.a_ctrl_used(:) - oA.a_true_out(:))) == 0, 'a=a_true wiring'); \
oB = run_pure_simulation(c, struct('seed',1,'collect_diag',true)); \
assert(max(abs(oB.diag.a_ctrl_used(2:end,:) - oB.diag.a_hat(1:end-1,:))) == 0, 'a=â lag-1'); \
c7 = c; c7 = rmfield(c7,'eq17_variant'); ok = false; \
try, run_pure_simulation(c7, struct('use_true_gain',true)); catch, ok = true; end; assert(ok, '7-state guard'); \
disp('SMOKE PASS')"
```
Expected: `SMOKE PASS` (a=a_true wiring exact, a=â one-step-lag exact, 7-state guard errors).

- [ ] **Step 3.7: checkcode + commit**

Run: `"$MB" -batch "r = checkcode('/Users/kevin/Code/MotionControl_Simu-motion-test/model/dual_track/run_pure_simulation.m'); if isempty(r), disp('CLEAN'), else, disp(r), error('issues'), end"`

```bash
git add model/dual_track/run_pure_simulation.m
git commit -m "feat(eq17): driver use_true_gain mode + a_true_out / a_ctrl_used logging

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 4: Runner `compare_gain_6state.m` (run matrix + Layer 0 + save)

**Files:**
- Create: `test_script/integration/compare_gain_6state.m`

- [ ] **Step 4.1: Write the runner**

```matlab
function results = compare_gain_6state(freqs, opts)
%COMPARE_GAIN_6STATE Run the gain-compare A/B matrix (design doc
%   reference/eq17_analysis/gain_compare_design.md §2-§3, §7 Layer 0).
%
%   results = compare_gain_6state()            % freqs = [1 2 5]
%   results = compare_gain_6state(freqs, opts)
%
%   Per frequency: 2 arms x (1 det run + numel(seeds) noisy runs), all
%   collect_diag. a=a_true = use_true_gain (true time-varying gain in the
%   control law), a=â = EKF gain. Both arms suppress_xD. Layer 0
%   assertions run before saving. Output:
%       test_results/gain_compare/f<f>Hz/runs.mat
%
%   opts: seeds (1:5), T_sim (7.0), verbose (true), out_root, smoke (false;
%         true -> T_sim=2.0 + seeds=1 for fast end-to-end checks).
%
%   See also: analyze_gain_6state, run_pure_simulation

    if nargin < 1 || isempty(freqs); freqs = [1 2 5]; end
    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'seeds');   opts.seeds = 1:5;    end
    if ~isfield(opts, 'T_sim');   opts.T_sim = 7.0;    end
    if ~isfield(opts, 'verbose'); opts.verbose = true; end
    if ~isfield(opts, 'smoke');   opts.smoke = false;  end
    if opts.smoke
        opts.T_sim = 2.0; opts.seeds = 1;
    end

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model'), fullfile(project_root, 'model', 'config'), ...
            fullfile(project_root, 'model', 'wall_effect'), ...
            fullfile(project_root, 'model', 'thermal_force'), ...
            fullfile(project_root, 'model', 'trajectory'), ...
            fullfile(project_root, 'model', 'controller'), ...
            fullfile(project_root, 'model', 'dual_track'), script_dir);
    if ~isfield(opts, 'out_root')
        opts.out_root = fullfile(project_root, 'test_results', 'gain_compare');
    end

    results = struct('freq', {}, 'out_dir', {}, 'layer0', {}, 'n_diverged', {});
    for fi = 1:numel(freqs)
        f = freqs(fi);
        cfg = build_config(f, opts);

        % --- pre-flight: desired trajectory respects h_min override ---
        params = calc_simulation_params(cfg);
        [is_safe, h_min_actual, t_crit] = check_trajectory_safety(params.Value);
        assert(is_safe, 'Trajectory unsafe: h_min_actual=%.3f um at t=%.2f s', ...
               h_min_actual, t_crit);

        if opts.verbose
            fprintf('[compare_gain_6state:%gHz] T_sim=%.1fs seeds=%s\n', ...
                    f, cfg.T_sim, mat2str(opts.seeds));
        end

        % --- run matrix ---
        runs = struct();
        for arm = 'AB'
            use_true_gain = (arm == 'A');
            cfg_det = cfg;
            cfg_det.thermal_enable = false; cfg_det.meas_noise_enable = false;
            runs.(arm).det = run_one(cfg_det, 0, use_true_gain);
            for s = 1:numel(opts.seeds)
                runs.(arm).noisy(s) = run_one(cfg, opts.seeds(s), use_true_gain);
            end
        end

        % --- Layer 0 assertions ---
        layer0 = layer0_checks(runs, opts);

        % --- save ---
        out_dir = fullfile(opts.out_root, sprintf('f%gHz', f));
        if ~exist(out_dir, 'dir'); mkdir(out_dir); end
        save(fullfile(out_dir, 'runs.mat'), 'runs', 'cfg', 'opts', 'layer0', '-v7.3');
        n_div = count_diverged(runs);
        if opts.verbose
            fprintf('  saved %s  (diverged runs: %d)\n', out_dir, n_div);
        end
        results(end+1) = struct('freq', f, 'out_dir', out_dir, ...
                                'layer0', layer0, 'n_diverged', n_div); %#ok<AGROW>
    end
end


function cfg = build_config(f, opts)
%BUILD_CONFIG osc_aggr scenario (design doc §3) at frequency f.
    cfg = user_config();
    cfg.eq17_variant   = '6state';
    cfg.trajectory_type = 'osc';
    cfg.h_init   = 50;            % [um] h_bar ~ 22
    cfg.h_bottom = 2.7;           % [um] h_bar = 1.2 (below gate 1.5)
    cfg.amplitude = 2.5;          % [um] -> h_bar in [1.2, 3.42], gate-crossing
    cfg.frequency = f;
    cfg.n_cycles  = 5 * f;        % osc duration fixed 5.0 s
    cfg.t_hold    = 0.5;
    cfg.t_descend_override = 1.0; % decouple descent from 1/f
    cfg.T_sim     = opts.T_sim;
    cfg.h_min     = 1.05 * 2.25;  % scenario-local override (global default 1.5R blocks h_bar=1.2)
    cfg.ctrl_enable = true;
    cfg.thermal_enable = true;
    cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7;
    cfg.a_pd  = 0.05;
    cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];   % [um]
    cfg.suppress_xD = true;       % both arms (design §2)
end


function rec = run_one(cfg, seed, use_true_gain)
%RUN_ONE Single run wrapped in try/catch (crash = diverged, design §7.3).
    ro = struct('seed', seed, 'verbose', false, 'collect_diag', true, ...
                'use_true_gain', use_true_gain);
    rec = struct('seed', seed, 'use_true_gain', use_true_gain, 'diverged', false, ...
                 'diverge_reason', '', 'simOut', []);
    try
        rec.simOut = run_pure_simulation(cfg, ro);
    catch err
        rec.diverged = true;
        rec.diverge_reason = sprintf('crash: %s', err.message);
        return;
    end
    % post-run divergence scan on aligned physical error (design §6.0/§7.3)
    e = rec.simOut.p_d_out(2:end, :) - rec.simOut.p_true_out(1:end-1, :);
    if max(abs(e(:))) > 0.5
        rec.diverged = true;
        rec.diverge_reason = sprintf('|e|_max = %.3g um > 0.5 um', max(abs(e(:))));
    end
end


function layer0 = layer0_checks(runs, opts)
%LAYER0_CHECKS Design §7 Layer 0: p_d identity + wiring assertions.
    ref = first_ok_run(runs);
    assert(~isempty(ref), 'Layer0: every run crashed — nothing to analyze');
    pd_ref = ref.simOut.p_d_out;
    layer0 = struct('pd_identical', true, 'wiring_A', true, 'wiring_B', true);
    for arm = 'AB'
        recs = [runs.(arm).det, runs.(arm).noisy];
        for r = recs
            if isempty(r.simOut); continue; end   % crashed run: skip checks
            assert(isequal(size(r.simOut.p_d_out), size(pd_ref)) && ...
                   max(abs(r.simOut.p_d_out(:) - pd_ref(:))) == 0, ...
                   'Layer0: p_d_out differs (arm %c seed %d)', arm, r.seed);
            if arm == 'A'
                assert(max(abs(r.simOut.diag.a_ctrl_used(:) - r.simOut.a_true_out(:))) == 0, ...
                       'Layer0: a=a_true a_ctrl_used ~= a_true_out (seed %d)', r.seed);
            else
                d = abs(r.simOut.diag.a_ctrl_used(2:end, :) - r.simOut.diag.a_hat(1:end-1, :));
                assert(max(d(:)) == 0, ...
                       'Layer0: a=â a_ctrl_used ~= a_hat posterior[k-1] (seed %d)', r.seed);
            end
        end
    end
    if opts.verbose; fprintf('  Layer 0: PASS\n'); end
end


function rec = first_ok_run(runs)
    rec = [];
    for arm = 'AB'
        all_r = [runs.(arm).det, runs.(arm).noisy];
        for r = all_r
            if ~isempty(r.simOut); rec = r; return; end
        end
    end
end


function n = count_diverged(runs)
    n = 0;
    for arm = 'AB'
        all_r = [runs.(arm).det, runs.(arm).noisy];
        n = n + sum([all_r.diverged]);
    end
end
```

- [ ] **Step 4.2: checkcode**

Run: `"$MB" -batch "r = checkcode('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/integration/compare_gain_6state.m'); if isempty(r), disp('CLEAN'), else, disp(r), error('issues'), end"`

- [ ] **Step 4.3: Smoke run (1 freq, smoke mode)**

Run: `"$MB" -batch "cd('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/integration'); r = compare_gain_6state(2, struct('smoke', true)); disp(r)"`
Expected: pre-flight safe, `Layer 0: PASS`, `runs.mat` saved under `test_results/gain_compare/f2Hz/`, diverged count reported (a=â may legitimately flag diverged near h̄=1.2 — that is a result, not a failure; only Layer-0 assertion errors fail this step).

Known deviation from design §7.3: a hard CRASH (wall contact inside `run_pure_simulation`) loses the partial waveform — capturing it would require driver-internal try/catch (too invasive). Runs flagged via the |e| > 0.5 μm threshold DO retain their full waveform; true crashes occur at wall contact where the simulation is invalid anyway. Recorded in the run record as `diverge_reason = 'crash: ...'`.

- [ ] **Step 4.4: Commit**

```bash
git add test_script/integration/compare_gain_6state.m
git commit -m "feat(eq17): gain-compare A/B runner (matrix + Layer-0 assertions)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 5: Analyzer `analyze_gain_6state.m` (det/ram + anchors + summary)

**Files:**
- Create: `test_script/integration/analyze_gain_6state.m`

- [ ] **Step 5.1: Write the analyzer** (analysis core + summary; figures in Task 6 extend this file)

```matlab
function analysis = analyze_gain_6state(freqs, opts)
%ANALYZE_GAIN_6STATE det/ram analysis of compare_gain_6state
%   output (design doc §6-§8). Loads runs.mat per frequency, computes det
%   metrics, ram window statistics, paired A/B ratios, p_m cross-check,
%   theory anchor, a_hat decomposition; writes summary.md (+ figures,
%   make_figs). Re-runnable without re-simulating.
%
%   analysis = analyze_gain_6state()             % freqs = [1 2 5]
%   analysis = analyze_gain_6state(freqs, opts)
%
%   opts: out_root (test_results/gain_compare), save_fig (true),
%         verbose (true).
%
%   See also: compare_gain_6state

    if nargin < 1 || isempty(freqs); freqs = [1 2 5]; end
    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'save_fig'); opts.save_fig = true; end
    if ~isfield(opts, 'verbose');  opts.verbose = true;  end

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    if ~isfield(opts, 'out_root')
        opts.out_root = fullfile(project_root, 'test_results', 'gain_compare');
    end

    analysis = struct('freq', {}, 'det', {}, 'ram', {}, 'anchor', {}, ...
                      'ahat', {}, 'flags', {});
    for fi = 1:numel(freqs)
        f = freqs(fi);
        out_dir = fullfile(opts.out_root, sprintf('f%gHz', f));
        S = load(fullfile(out_dir, 'runs.mat'));   % runs, cfg, opts, layer0

        A = per_freq_analysis(S.runs, S.cfg, f);
        write_summary_md(fullfile(out_dir, 'summary.md'), f, S, A);
        save(fullfile(out_dir, 'analysis.mat'), 'A');
        if opts.save_fig
            make_figs(S, A, f, out_dir);           % Task 6
        end
        analysis(end+1) = struct('freq', f, 'det', A.det, 'ram', A.ram, ...
                                 'anchor', A.anchor, 'ahat', A.ahat, ...
                                 'flags', A.flags); %#ok<AGROW>
        if opts.verbose; fprintf('[analyze:%gHz] done -> %s\n', f, out_dir); end
    end
    if numel(freqs) > 1 && opts.save_fig
        make_overview_fig(analysis, fullfile(opts.out_root, 'overview'));
    end
end


% ====================================================================
function A = per_freq_analysis(runs, cfg, f)
    Ts = 1 / 1600;
    sigma2_n = cfg.meas_noise_std(:).^2;            % [um^2]

    % --- aligned error signals (design §6.0: e[k] = p_d[k+1] - p_true[k]) ---
    get_e   = @(so) so.p_d_out(2:end, :) - so.p_true_out(1:end-1, :);
    get_em  = @(so) so.p_d_out(2:end, :) - so.p_m_out(1:end-1, :);
    so_ref  = runs.A.det.simOut;
    t_e     = so_ref.tout(2:end);                   % time of aligned samples
    pd_al   = so_ref.p_d_out(2:end, :);

    % --- windows (design §6.0) ---
    W.desc = (t_e >= 0.5  & t_e < 1.5);
    W.osc  = (t_e >= 2.5  & t_e < 6.5);
    W.tail = (t_e >= 6.5);
    h_bar_d = (pd_al * [0; 0; 1]) / 2.25;           % w_hat=[0;0;1], pz=0 (osc_aggr)
    W.near  = W.osc & (h_bar_d < 1.5);
    W.far = W.osc & (h_bar_d >= 1.5);

    % --- det metrics per arm (design §6.1) ---
    for arm = 'AB'
        e_det = get_e(runs.(arm).det.simOut);
        D.(arm) = det_metrics(e_det, t_e, W, f);
        A.e_det.(arm) = e_det;                      % stored for figures
    end
    A.det = D;

    % --- ram metrics per arm (design §6.2) + p_m cross-check (§6.3) ---
    wins = {'desc', 'osc', 'near', 'far'};
    for arm = 'AB'
        e_det  = A.e_det.(arm);
        em_det = get_em(runs.(arm).det.simOut);     % = e_det (noise-free)
        nz  = runs.(arm).noisy;
        ns  = numel(nz);
        div = [nz.diverged];
        R.(arm).mu    = nan(numel(wins), ns, 3);
        R.(arm).sd    = nan(numel(wins), ns, 3);
        R.(arm).sd_pm = nan(numel(wins), ns, 3);
        for w = 1:numel(wins)
            idx = W.(wins{w});
            for s = 1:ns
                if div(s); continue; end            % diverged: excluded (NaN), reported separately
                ram  = get_e(nz(s).simOut)  - e_det;
                ramm = get_em(nz(s).simOut) - em_det;
                for ax = 1:3
                    R.(arm).mu(w, s, ax)  = mean(ram(idx, ax));
                    R.(arm).sd(w, s, ax)  = std(ram(idx, ax));
                    sd_m = std(ramm(idx, ax));
                    R.(arm).sd_pm(w, s, ax) = sqrt(max(sd_m^2 - sigma2_n(ax), 0));
                end
            end
        end
        R.(arm).diverged = div;
    end
    % paired ratios (same-seed, non-diverged pairs only)
    ok = ~R.A.diverged & ~R.B.diverged;
    for w = 1:numel(wins)
        for ax = 1:3
            r = squeeze(R.B.sd(w, ok, ax)) ./ squeeze(R.A.sd(w, ok, ax));
            R.ratio.mean(w, ax) = mean(r);
            R.ratio.rng(w, ax, 1) = min(r); R.ratio.rng(w, ax, 2) = max(r);
        end
    end
    R.wins = wins;
    A.ram = R;

    % --- stationarity per cycle (design §6.2 / Layer 2) ---
    A.stationarity = cycle_stationarity(runs, A.e_det, get_e, W, f);

    % --- theory anchor on a=a_true (design §7.5) ---
    A.anchor = theory_anchor(runs.A, A.e_det.A, get_e, W, cfg, Ts);

    % --- a_hat decomposition, a=â (design §6.4) ---
    A.ahat = ahat_analysis(runs.B, W);

    % --- flags ---
    A.flags = collect_flags(A);
end


function D = det_metrics(e_det, t_e, W, f)
%DET_METRICS design §6.1 (per axis; z = col 3 is primary).
    for ax = 1:3
        % descent peak
        [pk, ipk] = max(abs(e_det(W.desc, ax)));
        td = t_e(W.desc);
        D.desc_peak(ax) = pk; D.desc_peak_t(ax) = td(ipk);
        % sine fit on W.osc: e ~ c0 + a1 cos + b1 sin
        tw = t_e(W.osc); ew = e_det(W.osc, ax);
        X = [ones(numel(tw), 1), cos(2*pi*f*tw), sin(2*pi*f*tw)];
        c = X \ ew;
        D.c0(ax)  = c(1);
        D.A_e(ax) = hypot(c(2), c(3));
        D.phi_deg(ax) = atan2d(c(3), c(2));
        D.rms_res(ax) = rms(ew - X * c);
        % trough bias: troughs at t = 1.5 + j/f inside W.osc, +-5 samples
        tb = [];
        for tj = (1.5 + 1/f):(1/f):6.49
            if tj < 2.5; continue; end
            [~, k0] = min(abs(t_e - tj));
            tb(end+1) = mean(e_det(max(1, k0-5):min(numel(t_e), k0+5), ax)); %#ok<AGROW>
        end
        D.trough_bias(ax) = mean(tb);
        % cycle-averaged waveform (z only meaningful; stored for figures)
        if ax == 3
            npc = round(1600 / f);
            io = find(W.osc);
            nc = floor(numel(io) / npc);
            D.cycle_wave = mean(reshape(e_det(io(1:nc*npc), 3), npc, nc), 2);
        end
    end
end


function st = cycle_stationarity(runs, e_det_all, get_e, W, f)
%CYCLE_STATIONARITY per-cycle std of z-axis ram, first vs second half.
    for arm = 'AB'
        nz = runs.(arm).noisy;
        npc = round(1600 / f);
        io = find(W.osc);
        nc = floor(numel(io) / npc);
        sd_c = nan(nc, numel(nz));
        for s = 1:numel(nz)
            if nz(s).diverged; continue; end
            ram_z = get_e(nz(s).simOut) - e_det_all.(arm);
            ram_z = ram_z(:, 3);
            for c = 1:nc
                sd_c(c, s) = std(ram_z(io((c-1)*npc+1 : c*npc)));
            end
        end
        m = mean(sd_c, 2, 'omitnan');
        h1 = mean(m(1:floor(nc/2))); h2 = mean(m(floor(nc/2)+1:end));
        st.(arm).per_cycle_sd = m;
        st.(arm).half_rel_diff = abs(h2 - h1) / max(h1, eps);
        st.(arm).stationary = st.(arm).half_rel_diff < 0.20;
    end
end


function anc = theory_anchor(runsA, e_detA, get_e, W, cfg, Ts)
%THEORY_ANCHOR design §7.4-7.5: a=a_true det ~ 0 and normalized ram ~ 1.
    lc = cfg.lambda_c;
    kBT = 1.3806503e-5 * 310.15;
    C_dx = 2 + 1 / (1 - lc^2);                       % 3.9608 at lc=0.7
    sigma2_nz = cfg.meas_noise_std(3)^2;
    % envelope from a=a_true det run's a_true (z axis), aligned to e[k]
    a_true_z = runsA.det.simOut.a_true_out(2:end, 3);
    sigma_th = sqrt(C_dx * 4 * kBT * a_true_z + (1-lc)/(1+lc) * sigma2_nz);
    anc.sigma_th = sigma_th;
    % det anchor (soft gate < 1 nm = 1e-3 um in W.osc)
    anc.det_max_osc_um = max(abs(e_detA(W.osc, 3)));
    anc.det_pass = anc.det_max_osc_um < 1e-3;
    % normalized ram per seed
    nz = runsA.noisy;
    zs = nan(1, numel(nz));
    for s = 1:numel(nz)
        if nz(s).diverged; continue; end
        ram_z = get_e(nz(s).simOut) - e_detA; ram_z = ram_z(:, 3);
        zn = ram_z(W.osc) ./ sigma_th(W.osc);
        zs(s) = std(zn);
    end
    anc.norm_std = zs;
    anc.norm_pass = all(abs(zs(~isnan(zs)) - 1) < 0.15);
end


function ah = ahat_analysis(runsB, W)
%AHAT_ANALYSIS design §6.4: ensemble-mean systematic + per-seed random.
    nz = runsB.noisy;
    ok = find(~[nz.diverged]);
    a_stack = [];
    for s = ok
        a_stack = cat(3, a_stack, nz(s).simOut.diag.a_hat(2:end, :));
    end
    ah.ens_mean = mean(a_stack, 3);                  % [N-1 x 3]
    a_true = runsB.det.simOut.a_true_out(2:end, :);
    ah.rel_err_osc = mean((ah.ens_mean(W.osc, :) - a_true(W.osc, :)) ...
                          ./ a_true(W.osc, :), 1) * 100;   % [%], per axis
    dev = a_stack - ah.ens_mean;
    ah.ram_std_osc = squeeze(std(reshape(dev(W.osc, :, :), [], 3, size(dev,3)), 0, 1));
    ah.a_true = a_true;
    % gate duty cycle in W.osc (per axis, mean over seeds)
    gd = [];
    for s = ok
        gd = cat(3, gd, double(nz(s).simOut.diag.gate_active(2:end, :)));
    end
    ah.gate_duty_osc = squeeze(mean(mean(gd(W.osc, :, :), 1), 3))';
end


function flags = collect_flags(A)
    flags.anchor_det  = A.anchor.det_pass;
    flags.anchor_norm = A.anchor.norm_pass;
    flags.stationary_A = A.stationarity.A.stationary;
    flags.stationary_B = A.stationarity.B.stationary;
end


function write_summary_md(path, f, S, A)
    fid = fopen(path, 'w');
    fprintf(fid, '# gain_compare : %g Hz\n\n', f);
    fprintf(fid, 'osc_aggr (h 50 -> 2.7 um, h_bar_min 1.2, A=2.5 um), %d seeds x %.1fs, suppress_xD both arms.\n\n', ...
            numel(S.opts.seeds), S.cfg.T_sim);
    fprintf(fid, '## det (e_det = p_d - p_true, noise-free run)\n\n');
    fprintf(fid, '| metric | arm | x | y | z |\n|---|---|---|---|---|\n');
    for arm = 'AB'
        D = A.det.(arm);
        fprintf(fid, '| descent peak [nm] | %c | %.1f | %.1f | %.1f |\n', arm, D.desc_peak*1e3);
        fprintf(fid, '| osc A_e [nm] | %c | %.2f | %.2f | %.2f |\n', arm, D.A_e*1e3);
        fprintf(fid, '| osc phase [deg] | %c | %.2f | %.2f | %.2f |\n', arm, D.phi_deg);
        fprintf(fid, '| osc rms_res [nm] | %c | %.2f | %.2f | %.2f |\n', arm, D.rms_res*1e3);
        fprintf(fid, '| trough bias [nm] | %c | %.2f | %.2f | %.2f |\n', arm, D.trough_bias*1e3);
    end
    fprintf(fid, '\n## ram (std over window, 5-seed mean; paired B/A ratio)\n\n');
    fprintf(fid, '| window | A sd_z [nm] | B sd_z [nm] | ratio mean | ratio range |\n|---|---|---|---|---|\n');
    for w = 1:numel(A.ram.wins)
        sdA = mean(squeeze(A.ram.A.sd(w, :, 3)), 'omitnan') * 1e3;
        sdB = mean(squeeze(A.ram.B.sd(w, :, 3)), 'omitnan') * 1e3;
        fprintf(fid, '| %s | %.2f | %.2f | %.2f | [%.2f, %.2f] |\n', A.ram.wins{w}, ...
                sdA, sdB, A.ram.ratio.mean(w, 3), ...
                A.ram.ratio.rng(w, 3, 1), A.ram.ratio.rng(w, 3, 2));
    end
    fprintf(fid, '\n## validation\n\n');
    fprintf(fid, '- a=a_true det anchor: max|e_det| (osc, z) = %.3f nm -> %s (soft gate < 1 nm)\n', ...
            A.anchor.det_max_osc_um*1e3, passstr(A.anchor.det_pass));
    fprintf(fid, '- a=a_true normalized ram std (per seed): %s -> %s (soft gate 1 +- 0.15)\n', ...
            mat2str(round(A.anchor.norm_std, 3)), passstr(A.anchor.norm_pass));
    fprintf(fid, '- stationarity (half-diff): A %.1f%% / B %.1f%%\n', ...
            A.stationarity.A.half_rel_diff*100, A.stationarity.B.half_rel_diff*100);
    fprintf(fid, '- diverged runs: A %s / B %s\n', ...
            mat2str(A.ram.A.diverged), mat2str(A.ram.B.diverged));
    fprintf(fid, '\n## a=â gain estimation\n\n');
    fprintf(fid, '- a_hat ensemble-mean rel-err (osc) [%%]: %s\n', mat2str(round(A.ahat.rel_err_osc, 2)));
    fprintf(fid, '- gate duty cycle (osc): %s\n', mat2str(round(A.ahat.gate_duty_osc, 3)));
    fclose(fid);
end


function s = passstr(b)
    if b; s = 'PASS'; else; s = 'FLAG'; end
end
```

(`make_figs` / `make_overview_fig` are added in Task 6 — until then add temporary stubs at the end of the file so the analyzer runs:)

```matlab
function make_figs(varargin)          %#ok<VANUS> % Task 6
end
function make_overview_fig(varargin)  %#ok<VANUS> % Task 6
end
```

- [ ] **Step 5.2: checkcode**

Run: `"$MB" -batch "r = checkcode('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/integration/analyze_gain_6state.m'); if isempty(r), disp('CLEAN'), else, disp(r), error('issues'), end"`

- [ ] **Step 5.3: Full-length single-seed verification run (analysis needs real windows)**

Run:
```bash
"$MB" -batch "cd('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/integration'); \
compare_gain_6state(2, struct('seeds', 1, 'T_sim', 7.0)); \
a = analyze_gain_6state(2); \
fprintf('anchor det %.4f nm, norm_std %s\n', a(1).anchor.det_max_osc_um*1e3, mat2str(a(1).anchor.norm_std, 3))"
```
Expected: completes without error; summary.md written under `test_results/gain_compare/f2Hz/`; **a=a_true det anchor < 1 nm** (if not, STOP — debug wiring per design §7.4 before continuing); normalized ram std printed (single seed, indicative only).

- [ ] **Step 5.4: Commit**

```bash
git add test_script/integration/analyze_gain_6state.m
git commit -m "feat(eq17): gain-compare A/B analyzer (det/ram, anchors, summary)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 6: Figures (EXP/thesis style + A/B colors)

**Files:**
- Modify: `test_script/integration/analyze_gain_6state.m` (replace the two stubs)

Style constants (design §8): True/theory green `[0 0.6 0]`, a=â red `[0.8 0 0]`, a=a_true blue-purple `[0.45 0.30 0.75]`, grid off, tiledlayout compact, stats-in-title, legend northoutside, FS 18 / LFS 14 / LW 2 (mirror `make_eq17_6state_figures.m`). Sweep palette for overview: `[0.10 0.30 0.85; 0.95 0.55 0.10; 0.55 0.20 0.65]` (1/2/5 Hz).

- [ ] **Step 6.1: Replace `make_figs` stub**

```matlab
function make_figs(S, A, f, out_dir)
%MAKE_FIGS fig1-fig4 per frequency (design §8), EXP/thesis style.
    COL_TRUE = [0 0.6 0]; COL_B = [0.8 0 0]; COL_A = [0.45 0.30 0.75];
    FS = 18; LFS = 14; LW = 2.0;
    so_ref = S.runs.A.det.simOut;
    t_e = so_ref.tout(2:end);

    % ---- fig1: gain tracking (z, x), a_true vs a=â ensemble mean ----
    f1 = figure('Position', [80 80 1100 720], 'Color', 'w', 'NumberTitle', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    cols = [3 1]; lbl = {'a_z', 'a_x'};
    for r = 1:2
        c = cols(r); nexttile; hold on;
        plot(t_e, A.ahat.a_true(:, c),   '-', 'Color', COL_TRUE, 'LineWidth', 3, 'DisplayName', 'True');
        plot(t_e, A.ahat.ens_mean(:, c), '-', 'Color', COL_B,    'LineWidth', LW, 'DisplayName', 'Estimated (ens. mean)');
        title(sprintf('%s:  rel-err (osc) %+.2f%%', lbl{r}, A.ahat.rel_err_osc(c)), ...
              'FontSize', FS, 'FontWeight', 'bold');
        ylabel(sprintf('%s (\\mum/pN)', lbl{r}), 'FontSize', FS, 'FontWeight', 'bold');
        grid off; set(gca, 'FontSize', LFS);
        if r == 1; legend('Location', 'northoutside', 'Orientation', 'horizontal'); end
    end
    xlabel('t (s)', 'FontSize', FS, 'FontWeight', 'bold');
    exportgraphics(f1, fullfile(out_dir, 'fig1_gain_tracking.png'), 'Resolution', 150);

    % ---- fig2: det error overlay (z), descent shading ----
    f2 = figure('Position', [80 80 1100 500], 'Color', 'w', 'NumberTitle', 'off');
    hold on;
    yl = [-1.2, 1.2] * max(1e-3, max(abs(A.e_det.B(:, 3)))) * 1e3;
    patch([0.5 1.5 1.5 0.5], yl([1 1 2 2]), [0.95 0.95 0.80], ...
          'EdgeColor', 'none', 'DisplayName', 'descent');
    plot(t_e, A.e_det.A(:, 3) * 1e3, '-', 'Color', COL_A, 'LineWidth', LW, 'DisplayName', 'a=a_true (use_true_gain)');
    plot(t_e, A.e_det.B(:, 3) * 1e3, '-', 'Color', COL_B, 'LineWidth', LW, 'DisplayName', 'a=â (estimated)');
    title(sprintf('e_{det,z}:  A_e  A %.2f / B %.2f nm,  \\phi  A %.1f / B %.1f deg', ...
          A.det.A.A_e(3)*1e3, A.det.B.A_e(3)*1e3, A.det.A.phi_deg(3), A.det.B.phi_deg(3)), ...
          'FontSize', FS, 'FontWeight', 'bold');
    xlabel('t (s)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('e_{det,z} (nm)', 'FontSize', FS, 'FontWeight', 'bold');
    ylim(yl); grid off; set(gca, 'FontSize', LFS);
    legend('Location', 'northoutside', 'Orientation', 'horizontal');
    exportgraphics(f2, fullfile(out_dir, 'fig2_det_error.png'), 'Resolution', 150);

    % ---- fig3: ram std per window (z), grouped A/B + ratio annotation ----
    f3 = figure('Position', [80 80 900 500], 'Color', 'w', 'NumberTitle', 'off');
    nw = numel(A.ram.wins);
    sdA = arrayfun(@(w) mean(squeeze(A.ram.A.sd(w, :, 3)), 'omitnan'), 1:nw) * 1e3;
    sdB = arrayfun(@(w) mean(squeeze(A.ram.B.sd(w, :, 3)), 'omitnan'), 1:nw) * 1e3;
    hb = bar([sdA(:), sdB(:)], 'grouped');
    hb(1).FaceColor = COL_A; hb(2).FaceColor = COL_B;
    set(gca, 'XTickLabel', A.ram.wins, 'FontSize', LFS);
    ylabel('std(ram_z) (nm)', 'FontSize', FS, 'FontWeight', 'bold');
    for w = 1:nw
        text(w, max(sdA(w), sdB(w)) * 1.05, sprintf('%.2fx', A.ram.ratio.mean(w, 3)), ...
             'HorizontalAlignment', 'center', 'FontSize', LFS, 'FontWeight', 'bold');
    end
    title(sprintf('ram std (z), %g Hz — B/A paired ratio annotated', f), ...
          'FontSize', FS, 'FontWeight', 'bold');
    legend({'a=a_true (use_true_gain)', 'a=â (estimated)'}, 'Location', 'northoutside', ...
           'Orientation', 'horizontal');
    grid off;
    exportgraphics(f3, fullfile(out_dir, 'fig3_ram_std.png'), 'Resolution', 150);

    % ---- fig4: theory anchor — a=a_true normalized ram per cycle ----
    f4 = figure('Position', [80 80 900 420], 'Color', 'w', 'NumberTitle', 'off');
    hold on;
    yline(1.0, '-',  'Color', COL_TRUE, 'LineWidth', 2);
    yline(0.85, '--', 'Color', [0.55 0.55 0.55]); yline(1.15, '--', 'Color', [0.55 0.55 0.55]);
    plot(A.anchor.norm_std, 'o', 'Color', COL_A, 'MarkerFaceColor', COL_A, 'MarkerSize', 8);
    xlabel('seed', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('std(ram_z / \sigma_{th})', 'FontSize', FS, 'FontWeight', 'bold');
    title(sprintf('a=a_true theory anchor (osc window): %s', passstr(A.anchor.norm_pass)), ...
          'FontSize', FS, 'FontWeight', 'bold');
    ylim([0.6 1.4]); grid off; set(gca, 'FontSize', LFS);
    exportgraphics(f4, fullfile(out_dir, 'fig4_theory_anchor.png'), 'Resolution', 150);
    close([f1 f2 f3 f4]);
end
```

- [ ] **Step 6.2: Replace `make_overview_fig` stub**

```matlab
function make_overview_fig(analysis, out_dir)
%MAKE_OVERVIEW_FIG fig5: paired B/A ratio vs frequency (z), per window.
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    sweep = [0.10 0.30 0.85; 0.95 0.55 0.10; 0.55 0.20 0.65];
    FS = 18; LFS = 14;
    f5 = figure('Position', [80 80 900 500], 'Color', 'w', 'NumberTitle', 'off');
    hold on;
    wins = analysis(1).ram.wins;
    freqs = [analysis.freq];
    mk = {'o-', 's-', 'd-', '^-'};
    for w = 1:numel(wins)
        r = arrayfun(@(a) a.ram.ratio.mean(w, 3), analysis);
        plot(freqs, r, mk{w}, 'Color', sweep(min(w, 3), :), 'LineWidth', 2, ...
             'MarkerSize', 9, 'DisplayName', wins{w});
    end
    yline(1.0, '--', 'Color', [0.55 0.55 0.55]);
    set(gca, 'XScale', 'log', 'XTick', freqs, 'FontSize', LFS);
    xlabel('frequency (Hz)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('ram std ratio  B/A  (z)', 'FontSize', FS, 'FontWeight', 'bold');
    title('Cost of estimated gain vs frequency', 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal');
    grid off;
    exportgraphics(f5, fullfile(out_dir, 'fig5_freq_overview.png'), 'Resolution', 150);
    close(f5);
end
```

- [ ] **Step 6.3: checkcode + render check on existing f2Hz data**

Run: `"$MB" -batch "r = checkcode('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/integration/analyze_gain_6state.m'); if isempty(r), disp('CLEAN'), else, disp(r), error('issues'), end"`
Then: `"$MB" -batch "cd('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/integration'); analyze_gain_6state(2);" && ls /Users/kevin/Code/MotionControl_Simu-motion-test/test_results/gain_compare/f2Hz/`
Expected: `fig1_gain_tracking.png fig2_det_error.png fig3_ram_std.png fig4_theory_anchor.png` exist alongside `summary.md`.

- [ ] **Step 6.4: Commit**

```bash
git add test_script/integration/analyze_gain_6state.m
git commit -m "feat(eq17): gain-compare A/B figures (EXP style, A/B colors, freq overview)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 7: Production run + user review checkpoint

- [ ] **Step 7.1: Full matrix run** (3 freqs × 12 runs; expect ~10-30 min total)

Run: `"$MB" -batch "cd('/Users/kevin/Code/MotionControl_Simu-motion-test/test_script/integration'); compare_gain_6state(); analyze_gain_6state();" 2>&1 | tee "$CLAUDE_JOB_DIR/tmp/gain_compare_run.log"` (or run in background and monitor)
Expected: Layer 0 PASS at all 3 frequencies; a=a_true det anchor < 1 nm at all 3; summaries + figures written; a=â divergences (if any) reported per run, not aborting the batch.

- [ ] **Step 7.2: Validation review (executor)**
- a=a_true normalized ram std within 1 ± 0.15 at 1 Hz and 2 Hz (5 Hz may exceed — quasi-static weakest there; document if so).
- p_m cross-check: `ram.B.sd_pm` vs `ram.B.sd` rel diff < 2% (z, osc window) — spot-check from analysis.mat.
- stationarity flags; gate duty cycle sane (near window duty > 0.5).

- [ ] **Step 7.3: Present results to user** — summary tables + figures + flags; **merge back to feat/eq17-6state is a separate user decision** (design §10: requires unit tests + h50 regression + Layer 0/1 + user review). Findings doc (`gain_compare_findings.md`) is written after the user has reviewed results.

---

## Self-review checklist (done at plan-writing time)

- **Spec coverage:** §2 matrix (Task 4 build_config/run matrix) / §3 trajectory (build_config + pre-flight) / §4.1 (Task 2) / §4.2 (Task 3) / §4.3 (Task 4+5+6) / §4.4 (Task 1, renamed) / §4.5 regression (Step 2.11) / §5 signals (driver logging + analyzer) / §6.0-6.4 (Task 5) / §6.5 optional PSD (deferred by design) / §7 Layer 0-2 (Tasks 4, 5, 7) / §8 figures+summary (Tasks 5, 6) / §9 caveats (run_one try/catch, ahat ensemble-mean choice, anchor soft gates) / §10 workflow (worktree, Bash batch) / §11 acceptance (Steps 2.9-2.11, 4.3, 5.3, 7.1-7.2).
- **Placeholder scan:** the two Task-5 figure stubs are explicitly replaced in Task 6 (not left as TODOs); no TBDs elsewhere.
- **Type consistency:** `a_ctrl_override` (3×1|[]), `diag.a_ctrl_used` (3×1), `simOut.a_true_out` [N×3], `runs.(arm).det/.noisy(s)` rec structs with `.simOut/.diverged/.seed` — names match across Tasks 2-6.
