function test_step_dynamics()
%TEST_STEP_DYNAMICS Unit tests for model/pure_matlab/step_dynamics.m
%
%   Verifies:
%       T1: Static input — F_total = 0 → no change in p across one Ts.
%       T2: Linear regime — far-field (wall effect off, isotropic Stokes),
%           constant F → p_next ≈ p + (Gamma_inv * F) * Ts (analytical).
%       T3: Convergence — ode4 with inner step 10 us vs 1 us agrees
%           to better than 1e-6 relative on a typical scenario.
%       T4: Wall-position sensitivity — changing h via pz changes the
%           perpendicular component speed (less motion when closer to
%           wall, since c_perp grows).
%
%   Each test prints "[PASS] ..." on success and asserts on failure.
%
%   Run from MATLAB after addpath of project paths, or directly from
%   this folder.

    this_dir  = fileparts(mfilename('fullpath'));
    repo_root = fullfile(this_dir, '..', '..');
    addpath(fullfile(repo_root, 'model'));
    addpath(fullfile(repo_root, 'model', 'config'));
    addpath(fullfile(repo_root, 'model', 'wall_effect'));
    addpath(fullfile(repo_root, 'model', 'pure_matlab'));

    % --- Common scenario: world frame, w_hat = [0;0;1], no wall offset
    constants = physical_constants();
    P.common.R       = constants.R;          % [um]
    P.common.gamma_N = constants.gamma_N;    % [pN*sec/um]
    P.common.Ts      = constants.Ts;
    P.common.p0      = [0; 0; 50];           % far-field z = 50 um
    P.wall.w_hat     = [0; 0; 1];
    P.wall.pz        = 0;
    P.wall.enable_wall_effect = 0;           % start in isotropic Stokes
    Ts = P.common.Ts;

    % ------------------------------------------------------------------
    % T1: Static input — F_total = 0, position should not drift
    % ------------------------------------------------------------------
    p_in   = P.common.p0;
    F_zero = [0; 0; 0];
    p_out  = step_dynamics(p_in, F_zero, P, Ts);

    drift = norm(p_out - p_in);
    assert(drift < 1e-12, ...
        'T1 failed: drift = %.3e um (expected < 1e-12)', drift);
    fprintf('[PASS] T1 zero-force static (drift = %.3e um)\n', drift);

    % ------------------------------------------------------------------
    % T2: Linear regime — wall OFF, isotropic Stokes
    %     Gamma_inv = (1/gamma_N) * I  →  p_dot = F / gamma_N (constant)
    %     Exact closed form: p_next = p + (F / gamma_N) * Ts
    % ------------------------------------------------------------------
    p_in   = P.common.p0;
    F_test = [1.0; 2.0; 0.5];                % [pN]
    p_out  = step_dynamics(p_in, F_test, P, Ts);

    p_expected = p_in + (F_test / P.common.gamma_N) * Ts;
    err_lin = norm(p_out - p_expected);
    assert(err_lin < 1e-12, ...
        'T2 failed: linear-regime error = %.3e um (expected < 1e-12)', err_lin);
    fprintf('[PASS] T2 linear regime, isotropic Stokes (err = %.3e um)\n', err_lin);

    % ------------------------------------------------------------------
    % T3: Convergence — ode4 step=10us vs ode4 step=1us
    %     Use wall-effect ON with non-trivial h_bar so Gamma_inv(p) varies.
    %     Run a single Ts step using the public step_dynamics (10 us)
    %     and compare to a manually-computed reference using 1 us substeps.
    % ------------------------------------------------------------------
    P_wall = P;
    P_wall.wall.enable_wall_effect = 1;
    p_in   = [0; 0; 5 * P.common.R];          % h_bar = 5 (mid-range, varies)
    F_test = [0.1; 0.0; -0.1];                % small force, mostly perp

    % Public path: 10 us inner step
    p_out_10us = step_dynamics(p_in, F_test, P_wall, Ts);

    % Reference: 1 us inner step, same RK4 algorithm
    p_out_1us = local_rk4_reference(p_in, F_test, P_wall, Ts, 1e-6);

    rel_err = norm(p_out_10us - p_out_1us) / norm(p_out_1us);
    assert(rel_err < 1e-6, ...
        'T3 failed: 10us vs 1us rel-err = %.3e (expected < 1e-6)', rel_err);
    fprintf('[PASS] T3 ode4 convergence (10us vs 1us rel-err = %.3e)\n', rel_err);

    % ------------------------------------------------------------------
    % T4: Wall-position sensitivity — perpendicular motion slower near wall
    %     With wall ON and a perpendicular force, motion should be smaller
    %     when h_bar is small (c_perp larger → 1/c_perp smaller).
    % ------------------------------------------------------------------
    F_perp = [0; 0; 1.0];                     % pure +z (perpendicular)

    % Far-field reference: h_bar = 50 → c_perp ≈ 1
    p_far = [0; 0; 50 * P.common.R];
    p_out_far = step_dynamics(p_far, F_perp, P_wall, Ts);
    delta_far = p_out_far(3) - p_far(3);

    % Near-wall: h_bar = 2 → c_perp > 1, so motion is smaller
    p_near = [0; 0; 2 * P.common.R];
    p_out_near = step_dynamics(p_near, F_perp, P_wall, Ts);
    delta_near = p_out_near(3) - p_near(3);

    assert(delta_far > 0 && delta_near > 0, ...
        'T4 failed: motions should be in +z direction (got far=%g, near=%g)', ...
        delta_far, delta_near);
    assert(delta_near < delta_far, ...
        'T4 failed: near-wall step (%.3e um) should be smaller than far (%.3e um)', ...
        delta_near, delta_far);
    fprintf('[PASS] T4 wall sensitivity (far: %.3e um, near: %.3e um)\n', ...
            delta_far, delta_near);

    fprintf('\nAll step_dynamics tests passed.\n');
end


function p_next = local_rk4_reference(p, F_total, params, Ts, h)
%LOCAL_RK4_REFERENCE  Reference RK4 with explicit inner step h (test-only).
%
%   Mirrors step_dynamics' RK4 substep logic, but with the inner step
%   given as a direct argument rather than computed from the 10us target.

    N_inner = max(1, round(Ts / h));
    h_eff = Ts / N_inner;
    p_curr = p;
    for j = 1:N_inner
        k1 = local_p_dot(p_curr,                F_total, params);
        k2 = local_p_dot(p_curr + (h_eff/2) * k1, F_total, params);
        k3 = local_p_dot(p_curr + (h_eff/2) * k2, F_total, params);
        k4 = local_p_dot(p_curr +  h_eff    * k3, F_total, params);
        p_curr = p_curr + (h_eff/6) * (k1 + 2*k2 + 2*k3 + k4);
    end
    p_next = p_curr;
end


function p_dot = local_p_dot(p, F_total, params)
    Gamma_inv = calc_gamma_inv(p, params);
    p_dot = Gamma_inv * F_total;
end
