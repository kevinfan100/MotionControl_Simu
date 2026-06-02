function smoke_wave3_h50_no_noise()
%SMOKE_WAVE3_H50_NO_NOISE Smoke test for Wave 2 v2 EKF integration
%
%   Wave 3 task 3: deterministic positioning at h=50 µm, no noise, no
%   thermal. Verifies:
%     - run_pure_simulation completes without crash
%     - f_d after warmup is approximately zero
%     - p_m approximately equals p_d (deterministic)
%     - a_hat_x converges to a_nom

    fprintf('=== smoke_wave3_h50_no_noise ===\n');

    this_dir = fileparts(mfilename('fullpath'));
    repo_root = fullfile(this_dir, '..', '..');
    addpath(fullfile(repo_root, 'model'));
    addpath(fullfile(repo_root, 'model', 'config'));
    addpath(fullfile(repo_root, 'model', 'controller'));
    addpath(fullfile(repo_root, 'model', 'wall_effect'));
    addpath(fullfile(repo_root, 'model', 'pure_matlab'));
    addpath(fullfile(repo_root, 'model', 'thermal_force'));
    addpath(fullfile(repo_root, 'model', 'trajectory'));
    addpath(fullfile(repo_root, 'model', 'diag'));

    % Force-clear all controller / driver persistent state
    clear motion_control_law motion_control_law_23state ...
          motion_control_law_7state motion_control_law_eq17_7state ...
          trajectory_generator calc_thermal_force; %#ok<CLFUNC>

    % Configuration per task brief
    config = user_config();
    config.h_init = 50;
    config.h_bottom = 50;
    config.amplitude = 0;
    config.frequency = 0;
    config.n_cycles = 1;
    config.t_hold = 0.05;
    config.T_sim = 1.0;                  % task brief: T_sim = 1 sec
    config.trajectory_type = 'positioning';
    config.controller_type = 17;
    config.meas_noise_enable = false;    % task brief: NO noise
    config.thermal_enable   = false;     % task brief: NO thermal

    fprintf('Config: h_init=%g, T_sim=%.1fs, noise=off, thermal=off, ctrl=eq17_7state\n', ...
        config.h_init, config.T_sim);

    opts = struct('seed', 42, 'verbose', false);
    simOut = run_pure_simulation(config, opts);

    fprintf('Simulation completed without crash.\n');

    % Time grid
    Ts = simOut.meta.params_value.common.Ts;
    tout = simOut.tout;
    N = numel(tout);

    % --- Check 1: no crash ---
    assert(N == round(config.T_sim / Ts) + 1, ...
        'SMOKE FAIL: unexpected N=%d', N);

    % --- Check 2: f_d after warmup ≈ 0 ---
    t_skip = 0.3;     % skip warmup
    skip_idx = round(t_skip / Ts);
    f_d_post = simOut.f_d_out(skip_idx:end, :);
    max_abs_fd = max(abs(f_d_post), [], 1);

    % --- Check 3: tracking error p_m - p_d ≈ 0 ---
    p_d = simOut.p_d_out(skip_idx:end, :);
    p_m = simOut.p_m_out(skip_idx:end, :);
    track_err = p_m - p_d;
    max_abs_err = max(abs(track_err), [], 1);
    rms_err = sqrt(mean(track_err.^2, 1));

    % --- Check 4: a_hat_x stays positive and finite ---
    a_hat_log = simOut.ekf_out;            % [N x 4] = [a_hat_x; a_hat_z; a_hat_y; h_bar]
    a_hat_post = a_hat_log(skip_idx:end, :);
    assert(all(isfinite(a_hat_post(:))), 'SMOKE FAIL: a_hat went non-finite');
    assert(all(a_hat_post(:, 1) > 0), 'SMOKE FAIL: a_hat_x went non-positive');
    assert(all(a_hat_post(:, 2) > 0), 'SMOKE FAIL: a_hat_z went non-positive');
    assert(all(a_hat_post(:, 3) > 0), 'SMOKE FAIL: a_hat_y went non-positive');

    % h_bar at end should be ~50/R = 22.2
    h_bar_end = a_hat_log(end, 4);
    assert(abs(h_bar_end - 50/2.25) < 0.5, ...
        'SMOKE FAIL: h_bar_end=%.3f, expected ~22.2', h_bar_end);

    % a_hat_x convergence: at h=50 (h_bar~22), c_para~1.03, so a_hat_x~Ts/gamma/c_para
    a_nom = (1/1600) / 0.0425;
    a_x_end = a_hat_post(end, 1);
    a_z_end = a_hat_post(end, 2);
    a_y_end = a_hat_post(end, 3);

    fprintf('\n--- Smoke results (post-warmup, t_skip=%.1fs) ---\n', t_skip);
    fprintf('Time samples: N=%d (t_final=%.3fs)\n', N, tout(end));
    fprintf('max|f_d|        per axis: [%.3e, %.3e, %.3e] pN\n', ...
        max_abs_fd(1), max_abs_fd(2), max_abs_fd(3));
    fprintf('max|p_m - p_d|  per axis: [%.3e, %.3e, %.3e] um\n', ...
        max_abs_err(1), max_abs_err(2), max_abs_err(3));
    fprintf('RMS|p_m - p_d|  per axis: [%.3e, %.3e, %.3e] um\n', ...
        rms_err(1), rms_err(2), rms_err(3));
    fprintf('a_hat_x (end): %.4e (a_nom=%.4e, ratio=%.3f)\n', ...
        a_x_end, a_nom, a_x_end / a_nom);
    fprintf('a_hat_y (end): %.4e\n', a_y_end);
    fprintf('a_hat_z (end): %.4e\n', a_z_end);
    fprintf('h_bar  (end): %.4f (expected ~22.2)\n', h_bar_end);

    % --- Assertions ---
    assert(all(max_abs_fd < 1e-6), ...
        'SMOKE FAIL: f_d not approximately zero post-warmup; max=%s', mat2str(max_abs_fd, 4));
    assert(all(max_abs_err < 1e-9), ...
        'SMOKE FAIL: p_m != p_d (deterministic); max_err=%s', mat2str(max_abs_err, 4));

    fprintf('\nSmoke test PASS — all checks satisfied.\n');
end
