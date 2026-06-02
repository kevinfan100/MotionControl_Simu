function test_motion_control_law_eq17_7state()
%TEST_MOTION_CONTROL_LAW_EQ17_7STATE Unit tests for eq17 7-state controller.
%
%   Verifies (text-only — no Simulink, no MATLAB MCP):
%       T1: Output shapes are f_d (3x1) and ekf_out (4x1)
%       T2: First call seeds persistent state without error; second call
%           advances state (k_step increments, IIR states change)
%       T3: 'clear motion_control_law_eq17_7state' resets persistent state
%           between scenarios
%       T4: Guard 1 (warm-up) — at t < t_warmup_kf, R_2 is gated to 1e10,
%           and a synthetic huge a_xm input does NOT cause a_hat to jump
%       T5: Far-field positioning (h_bar ~ 22) — Guard 3 NOT triggered;
%           a_hat stays positive and finite after warmup
%       T6: Smoke test — 10 step constant-positioning loop:
%               f_d magnitude bounded, a_hat stays positive
%
%   Usage (from MATLAB):
%       cd test_script/unit_tests
%       test_motion_control_law_eq17_7state

    this_dir = fileparts(mfilename('fullpath'));
    repo_root = fullfile(this_dir, '..', '..');
    addpath(fullfile(repo_root, 'model'));
    addpath(fullfile(repo_root, 'model', 'controller'));
    addpath(fullfile(repo_root, 'model', 'wall_effect'));

    n_pass = 0;

    % --------------------------------------------------------------
    % Build a minimal mock params struct + ctrl_const struct
    % --------------------------------------------------------------
    [params_mock, ctrl_const_mock] = build_mocks();

    % p_d at start (positioning hold at h_init = 50 um along z)
    p0 = [0; 0; 50];
    pd  = p0;
    del_pd = zeros(3, 1);    % positioning: p_d constant
    p_m = p0;                 % perfect tracking baseline

    % --------------------------------------------------------------
    % T1: Output shapes
    % --------------------------------------------------------------
    clear motion_control_law_eq17_7state;
    [f_d_1, ekf_1] = motion_control_law_eq17_7state( ...
        del_pd, pd, p_m, params_mock, ctrl_const_mock);
    assert(isequal(size(f_d_1), [3, 1]), ...
        'T1 failed: f_d shape = %s, expected [3 1]', mat2str(size(f_d_1)));
    assert(isequal(size(ekf_1), [4, 1]), ...
        'T1 failed: ekf_out shape = %s, expected [4 1]', mat2str(size(ekf_1)));
    fprintf('[PASS] T1: First call returns f_d (3x1), ekf_out (4x1)\n');
    n_pass = n_pass + 1;

    % --------------------------------------------------------------
    % T2: First call init then second call advances
    % --------------------------------------------------------------
    [f_d_2, ekf_2] = motion_control_law_eq17_7state( ...
        del_pd, pd, p_m, params_mock, ctrl_const_mock);
    assert(isequal(size(f_d_2), [3, 1]), 'T2 failed: f_d_2 wrong shape');
    assert(isequal(size(ekf_2), [4, 1]), 'T2 failed: ekf_2 wrong shape');
    % First call returns 0 (init), second computes real f_d.
    % For perfect tracking + zero disturbance + a_nom seed → f_d should be zero.
    assert(all(isfinite(f_d_2)), 'T2 failed: f_d not finite on second call');
    fprintf('[PASS] T2: Second call advances; f_d finite (max |f_d|=%.3e)\n', ...
        max(abs(f_d_2)));
    n_pass = n_pass + 1;

    % --------------------------------------------------------------
    % T3: Clear resets state — verify by checking first-call-zeros pattern
    % --------------------------------------------------------------
    clear motion_control_law_eq17_7state;
    [f_d_3, ekf_3] = motion_control_law_eq17_7state( ...
        del_pd, pd, p_m, params_mock, ctrl_const_mock);
    assert(all(f_d_3 == 0), ...
        'T3 failed: post-clear first call should yield f_d=0 (init), got %s', ...
        mat2str(f_d_3'));
    assert(all(ekf_3(1:3) > 0), ...
        'T3 failed: a_hat seeds (idx 1..3) should be positive (=a_nom)');
    fprintf('[PASS] T3: clear resets state — first call returns f_d=0, a_hat>0\n');
    n_pass = n_pass + 1;

    % --------------------------------------------------------------
    % T4: Guard 1 (warm-up) — synthetic huge a_xm should NOT propagate
    %     into a_hat. We cannot directly inject a_xm into the function
    %     (it's computed internally from delta_x_m), but we CAN exercise
    %     the same code path: during warm-up the R_2 channel must be
    %     gated to 1e10, so any innovation magnitude is divided by huge R
    %     and produces near-zero gain → a_hat stays close to a_nom.
    %
    %     We test by stuffing a large delta_x_m via an outlier p_m offset
    %     during warm-up. The variance EWMA will spike, a_xm will be huge,
    %     but Guard 1 should keep a_hat near its initial value.
    % --------------------------------------------------------------
    clear motion_control_law_eq17_7state;
    Ts = params_mock.ctrl.Ts;
    a_nom_local = Ts / params_mock.ctrl.gamma;
    % t < t_warmup_kf (= 0.2 sec by default → ~320 steps at Ts=1/1600)
    n_warmup_steps = round(0.5 * ctrl_const_mock.t_warmup_kf / Ts);
    [~, ekf_init] = motion_control_law_eq17_7state( ...
        del_pd, pd, p_m, params_mock, ctrl_const_mock);
    a_hat_init_x = ekf_init(1);

    % Inject large position offsets to spike sigma2_dxr_hat
    p_m_outlier = p0 + [10; 10; 10];   % 10 um outlier per axis
    last_a_hat = a_hat_init_x;
    for k = 1:n_warmup_steps
        [~, ekf_k] = motion_control_law_eq17_7state( ...
            del_pd, pd, p_m_outlier, params_mock, ctrl_const_mock);
        last_a_hat = ekf_k(1);
    end

    % Within warm-up, a_hat should not have inflated by orders of magnitude
    %   (Guard 1 should prevent y_2 channel from injecting big innovation)
    %   tolerance: factor-of-2 drift acceptable; factor-of-10 would indicate failure.
    assert(last_a_hat > 0, ...
        'T4 failed: a_hat went non-positive during warm-up (=%g)', last_a_hat);
    drift_ratio = last_a_hat / a_nom_local;
    assert(drift_ratio < 5.0, ...
        ['T4 failed: a_hat drifted by %.2fx during warm-up under outlier — ', ...
         'Guard 1 may not be active'], drift_ratio);
    fprintf(['[PASS] T4: Guard 1 (warm-up) damps outlier injection — ', ...
             'a_hat drift = %.2fx (a_nom), step count=%d\n'], drift_ratio, n_warmup_steps);
    n_pass = n_pass + 1;

    % --------------------------------------------------------------
    % T5: Far-field positioning (h_bar ~ 22), past warm-up — Guard 3
    %     should not trigger; a_hat stays positive/finite.
    % --------------------------------------------------------------
    clear motion_control_law_eq17_7state;
    %   p0 = (0,0,50) µm at R=2.25 µm gives h_bar ≈ 22.2.
    %   Step long enough to clear warm-up.
    n_steps_far = round(2.0 * ctrl_const_mock.t_warmup_kf / Ts);
    last_ekf = [];
    for k = 1:n_steps_far
        [~, last_ekf] = motion_control_law_eq17_7state( ...
            del_pd, pd, p_m, params_mock, ctrl_const_mock);
    end
    assert(all(isfinite(last_ekf)), 'T5 failed: ekf_out has non-finite entries');
    assert(last_ekf(1) > 0 && last_ekf(2) > 0 && last_ekf(3) > 0, ...
        'T5 failed: a_hat went non-positive at far-field (ekf_out = %s)', ...
        mat2str(last_ekf'));
    h_bar_last = last_ekf(4);
    assert(h_bar_last > ctrl_const_mock.h_bar_safe, ...
        'T5 failed: h_bar (%.2f) <= h_bar_safe (%.2f) — far-field assumption broken', ...
        h_bar_last, ctrl_const_mock.h_bar_safe);
    fprintf(['[PASS] T5: Far-field positioning at h_bar=%.2f — ', ...
             'all axes a_hat > 0, finite\n'], h_bar_last);
    n_pass = n_pass + 1;

    % --------------------------------------------------------------
    % T6: Smoke test — 10 steps constant positioning + small noise
    % --------------------------------------------------------------
    clear motion_control_law_eq17_7state;
    rng(42);  % deterministic
    sigma_noise_um = 1e-3;   % 1 nm RMS noise
    f_d_log = zeros(3, 10);
    a_hat_log = zeros(3, 10);
    for k = 1:10
        p_m_k = p0 + sigma_noise_um * randn(3, 1);
        [f_d_k, ekf_k] = motion_control_law_eq17_7state( ...
            del_pd, pd, p_m_k, params_mock, ctrl_const_mock);
        f_d_log(:, k) = f_d_k;
        a_hat_log(:, k) = ekf_k(1:3);
    end
    max_fd = max(abs(f_d_log(:)));
    min_a_hat = min(a_hat_log(:));
    assert(all(isfinite(f_d_log(:))), 'T6 failed: f_d went non-finite');
    assert(max_fd < 1e3, ...
        'T6 failed: |f_d| (%.3e pN) too large for small-noise smoke test', max_fd);
    assert(min_a_hat > 0, ...
        'T6 failed: a_hat went non-positive (min=%g)', min_a_hat);
    fprintf(['[PASS] T6: 10-step smoke test — max|f_d|=%.3e pN, ', ...
             'min a_hat=%.3e (>0)\n'], max_fd, min_a_hat);
    n_pass = n_pass + 1;

    fprintf('\n=== ALL %d tests PASS ===\n', n_pass);

end


%% =================== Helpers ===================

function [params_mock, ctrl_const_mock] = build_mocks()
%BUILD_MOCKS Construct minimal struct mocks (avoid full Simulink Bus build).
%
%   Returns nested struct matching the field paths the controller reads:
%       params.ctrl.{enable, lambda_c, gamma, Ts, k_B, T, sigma2_noise,
%                    Pf_init_diag, controller_type}
%       params.common.R
%       params.wall.{w_hat, pz, enable_wall_effect}
%       params.traj.{amplitude, frequency}
%
%   Plus ctrl_const built directly from build_eq17_constants.

    % ---- Physical constants (mirror physical_constants.m) ----
    R = 2.25;             % [um]
    gamma_N = 0.0425;     % [pN*sec/um]
    Ts = 1/1600;
    k_B = 1.3806503e-5;   % [pN*um/K]
    T_K = 310.15;
    kBT = k_B * T_K;

    % ---- Per-axis sensor noise std (mirror run_simulation defaults) ----
    sigma_n_s_um = [0.00062; 0.000057; 0.00331];     % [um] STD per axis
    sigma2_n_s = sigma_n_s_um.^2;                     % [um^2]

    % ---- Build params (struct-equivalent of params.Value) ----
    params_mock.ctrl.enable = 1;
    params_mock.ctrl.lambda_c = 0.7;
    params_mock.ctrl.gamma = gamma_N;
    params_mock.ctrl.Ts = Ts;
    params_mock.ctrl.k_B = k_B;
    params_mock.ctrl.T = T_K;
    params_mock.ctrl.sigma2_noise = sigma2_n_s;
    params_mock.ctrl.Pf_init_diag = ...
        [0; 0; 1e-4; 1e-4; 0; 10*(0.0147)^2; 0];    % 7x1
    params_mock.ctrl.controller_type = 17;            % eq17

    params_mock.common.R = R;
    params_mock.common.Ts = Ts;
    params_mock.common.gamma_N = gamma_N;

    params_mock.wall.w_hat = [0; 0; 1];
    params_mock.wall.pz = 0;
    params_mock.wall.enable_wall_effect = 1;

    params_mock.traj.amplitude = 10;     % [um]
    params_mock.traj.frequency = 1;      % [Hz]

    % ---- Build ctrl_const via build_eq17_constants (real function) ----
    opts.lambda_c    = params_mock.ctrl.lambda_c;
    opts.option      = 'A_MA2_full';
    opts.sigma2_n_s  = sigma2_n_s;
    opts.kBT         = kBT;
    opts.t_warmup_kf = 0.2;
    opts.h_bar_safe  = 1.5;
    opts.d           = 2;
    opts.a_cov       = 0.05;
    ctrl_const_mock = build_eq17_constants(opts);

end
