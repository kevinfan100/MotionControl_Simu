# EKF Structural Fix Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the full 23-state EKF converge in closed-loop by fixing 5 identified instability mechanisms.

**Architecture:** Minimal structural changes to the existing EKF: leaky integrator for rate states (rho_f=0.995), Kalman gain decoupling, adaptive g_cov, state-dependent R, and increased theta epsilon. The controller code is rewritten from the current disabled-EKF state back to full 23-state operation.

**Tech Stack:** MATLAB R2025b, Simulink, MATLAB MCP for testing.

**Spec:** `docs/superpowers/specs/2026-03-17-ekf-structural-fix-design.md`

---

## File Structure

| File | Action | Responsibility |
|------|--------|---------------|
| `model/config/user_config.m` | Modify | Add rho_f, change epsilon |
| `model/controller/calc_ctrl_params.m` | Modify | Pass rho_f to ctrl struct |
| `model/calc_simulation_params.m` | Modify | Add rho_f to CtrlBus |
| `model/controller/motion_control_law.m` | Rewrite Steps 0,5,6,8 | Restore full EKF + all structural fixes |

---

## Task 1: Add rho_f Parameter to Pipeline

**Files:**
- Modify: `model/config/user_config.m:62-63`
- Modify: `model/controller/calc_ctrl_params.m:48-49`
- Modify: `model/calc_simulation_params.m:124,155-156`

- [ ] **Step 1.1: Add rho_f to user_config.m**

After the `alpha_f` line (line 63), add:
```matlab
    config.rho_f = 0.995;          % EKF rate state leaky factor (half-life ~86 ms)
```

Change epsilon default (line 62):
```matlab
    config.epsilon = 0.05;          % Anisotropy threshold for theta measurement
```

Update docstring: add `rho_f` and update `epsilon` default in the header comments.

- [ ] **Step 1.2: Pass rho_f through calc_ctrl_params.m**

After the `ctrl.alpha_f` line (line 49), add:
```matlab
    ctrl.rho_f = config.rho_f;
```

Update docstring to list rho_f in inputs/outputs.

- [ ] **Step 1.3: Add rho_f to CtrlBus in calc_simulation_params.m**

Change `elems_ctrl` size from 16 to 17 (line 124):
```matlab
    elems_ctrl = Simulink.BusElement.empty(0, 17);
```

After element 16 (alpha_f), add element 17 (line ~157):
```matlab
    elems_ctrl(17) = Simulink.BusElement; elems_ctrl(17).Name = 'rho_f';
    elems_ctrl(17).Dimensions = [1 1]; elems_ctrl(17).DataType = 'double';
```

- [ ] **Step 1.4: Verify parameter pipeline with MATLAB MCP**

Run:
```matlab
clear; clc;
addpath('model'); addpath('model/config'); addpath('model/controller');
addpath('model/wall_effect'); addpath('model/thermal_force'); addpath('model/trajectory');
config = user_config();
assert(config.rho_f == 0.995, 'rho_f default wrong');
assert(config.epsilon == 0.05, 'epsilon default wrong');
params = calc_simulation_params(config);
assert(params.Value.ctrl.rho_f == 0.995, 'rho_f not in params');
fprintf('Pipeline OK: rho_f=%.3f, epsilon=%.2f, alpha_f=%.3f\n', ...
    params.Value.ctrl.rho_f, params.Value.ctrl.epsilon, params.Value.ctrl.alpha_f);
```

Expected: `Pipeline OK: rho_f=0.995, epsilon=0.05, alpha_f=0.998`

- [ ] **Step 1.5: Commit**

```
feat(control): Add rho_f leaky factor and increase epsilon to 0.05
```

---

## Task 2: Rewrite motion_control_law.m with Full EKF

This is the core task. The entire function is rewritten from the current
disabled-EKF state to full 23-state EKF with all structural fixes.

**Files:**
- Rewrite: `model/controller/motion_control_law.m` (entire function)

- [ ] **Step 2.1: Update docstring**

Replace current "DIRECT-EMA LAMBDA ESTIMATION" header with:
```matlab
%   Full 23-state EKF with structural stability fixes:
%     - Leaky integrator for rate states (rho_f) prevents drift
%     - Kalman gain decoupling prevents position-parameter cross-talk
%     - Adaptive g_cov corrects for EMA variance reduction
%     - State-dependent R uses chi-squared noise model
%     - Theta epsilon=0.05 prevents noise amplification at low anisotropy
```

- [ ] **Step 2.2: Add rho_f to persistent declarations and initialization**

Add to persistent line:
```matlab
    persistent rho_f
```

In Step [0] init, after `alpha_f` extraction:
```matlab
        rho_f = params.ctrl.rho_f;
```

Add warmup persistent and init (same as current pattern):
```matlab
    persistent step_count warmup_steps
```
```matlab
        step_count = 0;
        warmup_steps = ceil(3/a_pd + 3/a_prd);
```

- [ ] **Step 2.3: Restore Step [5] — Full Kalman Gain Computation**

Replace the current `L = zeros(23, 7)` block (lines 176-186) with:
```matlab
    %% Step [5]: Kalman Gain
    idx_obs = [1:3, 16:17, 20:21];
    Pf_HT   = Pf(:, idx_obs);                                    % 23x7
    HPf_HT  = Pf(idx_obs, idx_obs);                              % 7x7
    S       = HPf_HT + R;                                        % 7x7
    S       = (S + S') / 2;                                       % enforce symmetry
    L       = Pf_HT / S;                                         % 23x7 (mldivide)

    % NaN guard
    if any(isnan(L(:)))
        L = zeros(23, 7);
    end

    % EMA warmup gate
    if step_count <= warmup_steps
        L = zeros(23, 7);
    end

    % Decouple: lambda/theta only respond to their own measurements
    L(16:19, [1:3, 6:7]) = 0;     % lambda: keep only cols 4:5
    L(20:23, [1:3, 4:5]) = 0;     % theta: keep only cols 6:7
```

- [ ] **Step 2.4: Restore Step [6] — Full 23-state Update**

Replace the current disabled Step [6] (lines 188-206) with:
```matlab
    %% Step [6]: State Update (all 9 groups)

    % [6a] Wall-frame EKF states
    V_del_p1_hat_kA1  = V_del_p2_hat                    + L(1:3,:)   * err;
    V_del_p2_hat_kA1  = V_del_p3_hat                    + L(4:6,:)   * err;
    V_del_p3_hat_kA1  = lambda_c * V_del_p3_hat         + L(7:9,:)   * err;
    V_d_hat_kA1       = V_d_hat + V_del_d_hat           + L(10:12,:) * err;
    V_del_d_hat_kA1   = V_del_d_hat                     + L(13:15,:) * err;
    lamda_hat_kA1     = lamda_hat + del_lamda_hat        + L(16:17,:) * err;
    del_lamda_hat_kA1 = del_lamda_hat                    + L(18:19,:) * err;
    theta_hat_kA1     = theta_hat + del_theta_hat        + L(20:21,:) * err;
    del_theta_hat_kA1 = del_theta_hat                    + L(22:23,:) * err;

    % [6b] Convert to world frame
    del_p1_hat_kA1 = V * V_del_p1_hat_kA1;
    del_p2_hat_kA1 = V * V_del_p2_hat_kA1;
    del_p3_hat_kA1 = V * V_del_p3_hat_kA1;
    d_hat_kA1      = V * V_d_hat_kA1;
    del_d_hat_kA1  = V * V_del_d_hat_kA1;
```

- [ ] **Step 2.5: Modify Step [8] — Leaky F Matrix for Rate States**

In the F matrix construction (lines 226-241), change three lines:
```matlab
    F(13:15, 13:15) = rho_f * eye(3);                              % Row 5 (leaky)
    F(18:19, 18:19) = rho_f * eye(2);                              % Row 7 (leaky)
    F(22:23, 22:23) = rho_f * eye(2);                              % Row 9 (leaky)
```

All other F entries remain `eye()` (unchanged).

- [ ] **Step 2.6: Add step_count increment**

After the init block (after `end` of the `if isempty(initialized)` block), add:
```matlab
    step_count = step_count + 1;
```

- [ ] **Step 2.7: Run checkcode static analysis**

Run MATLAB MCP `check_matlab_code` on `motion_control_law.m`.
Expected: only benign warnings about unused persistent variables.

- [ ] **Step 2.8: Commit**

```
feat(control): Restore full 23-state EKF with structural stability fixes
```

---

## Task 3: Convergence Tests

**Files:** No files modified. Tests run via MATLAB MCP `evaluate_matlab_code`.

All tests use this shared preamble:
```matlab
clear; close all; clc;
clear motion_control_law trajectory_generator;
project_root = '/Users/kevin/Code/MotionControl_Simu';
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'thermal_force'));
addpath(fullfile(project_root, 'model', 'trajectory'));
addpath(fullfile(project_root, 'model', 'controller'));
```

- [ ] **Step 3.1: Test T1 — h=20, weak wall effect**

```matlab
config = user_config();
config.h_init = 20; config.amplitude = 1.0;
config.frequency = 1; config.n_cycles = 5;
config.thermal_enable = true; config.meas_noise_enable = false;
config.ctrl_enable = true; config.T_sim = 5.3;
```

Success criteria:
- Simulation completes without error
- lambda_hat within 20% of true lambda (last 25%)
- No wall crash (h_bar > 1.5)

- [ ] **Step 3.2: Test T2 — h=7, moderate wall effect**

```matlab
config = user_config();
config.h_init = 7; config.amplitude = 2.0;
config.frequency = 1; config.n_cycles = 5;
config.thermal_enable = true; config.meas_noise_enable = true;
config.meas_noise_std = [0.00062; 0.000057; 0.00331];
config.ctrl_enable = true; config.T_sim = 5.3;
```

Success criteria:
- Simulation completes
- lambda_hat converges (not stuck at clamp)
- h_bar > 1.5

- [ ] **Step 3.3: Test T3 — h=5, standard config (hardest)**

```matlab
config = user_config();
config.h_init = 5; config.amplitude = 2.5;
config.frequency = 1; config.n_cycles = 5;
config.thermal_enable = true; config.meas_noise_enable = true;
config.meas_noise_std = [0.00062; 0.000057; 0.00331];
config.ctrl_enable = true; config.T_sim = 5.3;
```

Success criteria:
- Simulation completes
- h_bar > h_min (1.5)
- Tracking error < 10 um RMS

- [ ] **Step 3.4: Test T4 — Long duration stability**

```matlab
config = user_config();
config.h_init = 7; config.amplitude = 2.0;
config.frequency = 1; config.n_cycles = 20;
config.thermal_enable = true; config.meas_noise_enable = true;
config.meas_noise_std = [0.00062; 0.000057; 0.00331];
config.ctrl_enable = true; config.T_sim = 20.3;
```

Success criteria:
- Simulation completes (20 cycles)
- lambda_hat bounded (not drifting)
- Tracking error not growing over time

- [ ] **Step 3.5: Record results and commit**

Update `agent_docs/ekf-qr-analysis.md` Section 9 with test results.

```
test(control): Verify EKF convergence with structural fixes
```

---

## Task 4: Fallback — Staged Activation (if Task 3 fails)

Only execute if any of T1-T3 fail. Add configurable staged activation
on top of the structural fixes.

**Files:**
- Modify: `model/controller/motion_control_law.m` (Step [5] only)

- [ ] **Step 4.1: Add staged activation logic after warmup gate**

Replace the warmup gate + decoupling block in Step [5] with:
```matlab
    % Staged activation: gradually enable state groups
    if step_count <= warmup_steps
        L = zeros(23, 7);                            % Phase 0: all off
    elseif step_count <= 3 * warmup_steps
        L(10:23, :) = 0;                             % Phase 1: position only
    elseif step_count <= 6 * warmup_steps
        L(16:23, :) = 0;                             % Phase 2: +disturbance
    end
    % Phase 3+: full EKF (with decoupling below)

    % Decouple lambda/theta from position measurements
    L(16:19, [1:3, 6:7]) = 0;
    L(20:23, [1:3, 4:5]) = 0;
```

- [ ] **Step 4.2: Re-run T1-T3 with staged activation**

- [ ] **Step 4.3: Commit if successful**

```
feat(control): Add staged activation fallback for EKF stability
```
