function out = temp_test_4state_ar1_1hz(seed)
%TEMP_TEST_4STATE_AR1_1HZ  4-state (AR(1) Q44 fix) on the 1 Hz near-wall osc.
%   Single-seed raw run. Produces two z-axis figures:
%     Fig 1: a_z_hat error  (a_hat_z - a_true_z)            [single signal]
%     Fig 2: a_m (a_xm) vs a_hat overlaid (a_true as light reference)
%
%   Scenario mirrors verify_eq17_4state build_cfg exactly; only difference is
%   use_q44_ar1=true and a single seed (raw, no ensemble averaging).
%
%   TEMP script -- delete after use.

    if nargin < 1; seed = 1; end

    here = fileparts(mfilename('fullpath'));
    addpath(genpath(here));

    f = 1.0; T_sim = 4.0;

    % --- scenario (identical to verify_eq17_4state build_cfg) ---
    cfg = user_config();
    cfg.eq17_variant   = '4state';
    cfg.trajectory_type = 'osc';
    cfg.h_init   = 50;
    cfg.h_bottom = 2.7;                       % h_bar = 1.2 (near wall)
    cfg.amplitude = 2.5;                      % h_bar in [1.2, 3.42]
    cfg.frequency = f;
    cfg.n_cycles  = 2 * f;                    % osc duration = 2 s
    cfg.t_hold    = 0.5;
    cfg.t_descend_override = 1.0;
    cfg.T_sim     = T_sim;
    pc = physical_constants();
    cfg.h_min     = 1.05 * pc.R;
    cfg.ctrl_enable = true;
    cfg.thermal_enable = true;
    cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7;
    cfg.a_pd  = 0.05;
    cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
    cfg.suppress_xD = true;
    cfg.h_bar_safe = 1;
    cfg.use_am_lpf = false;
    cfg.a_det      = 0.005;

    % --- the only change: AR(1) Q44 fix toggle ---
    cfg.use_q44_ar1 = true;

    % --- run, single seed, raw ---
    ro = struct('seed', seed, 'verbose', false, 'collect_diag', true, ...
                'use_true_gain', false);
    s = run_pure_simulation(cfg, ro);

    Ts = s.meta.params_value.common.Ts;

    % aligned (drop sample 1, same convention as verify_eq17_4state)
    a_true = s.a_true_out(2:end, :);          % [N-1 x 3]
    a_hat  = s.diag.a_hat(2:end, :);
    a_xm   = s.diag.a_xm(2:end, :);
    N = size(a_true, 1);
    t = (1:N).' * Ts;

    az = 3;                                    % z axis (wall-normal)
    a_true_z = a_true(:, az);
    a_hat_z  = a_hat(:, az);
    a_xm_z   = a_xm(:, az);
    err_z    = a_hat_z - a_true_z;

    % quick numbers
    relerr_z = median(abs(err_z) ./ a_true_z);
    fprintf('[4-state AR(1) @ %gHz, seed %d]\n', f, seed);
    fprintf('  a_z_hat median |err|/a_true = %.2f%%\n', 100 * relerr_z);
    fprintf('  a_z_hat err: mean = %+.4f nm/pN, std = %.4f nm/pN\n', ...
            1e3 * mean(err_z), 1e3 * std(err_z));

    out_dir = fullfile(here, 'test_results', 'temp_4state_ar1', sprintf('f%gHz_seed%d', f, seed));
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    FS = 18; LFS = 14; AXLW = 2.0;
    COL_T = [0 0 0]; COL_HAT = [0 0.45 0.74]; COL_M = [0.85 0.33 0.10];

    % ============ Fig 1: a_z_hat error (single signal) ============
    f1 = figure('Position', [80 80 1000 460], 'Color', 'w', 'NumberTitle', 'off');
    hold on;
    plot(t, zeros(size(t)), 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    plot(t, err_z * 1e3, '-', 'Color', COL_HAT, 'LineWidth', 1.8);
    ylabel('a_z hat error  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    fig1 = fullfile(out_dir, 'fig1_az_hat_error.png');
    exportgraphics(f1, fig1, 'Resolution', 150);

    % ============ Fig 2: a_m vs a_hat overlaid ============
    f2 = figure('Position', [80 80 1000 460], 'Color', 'w', 'NumberTitle', 'off');
    hold on;
    hM = plot(t, a_xm_z  * 1e3, '-', 'Color', COL_M,   'LineWidth', 1.0, 'DisplayName', 'a_m (a_{xm})');
    hH = plot(t, a_hat_z * 1e3, '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'a_{hat}');
    hT = plot(t, a_true_z * 1e3, '--', 'Color', COL_T, 'LineWidth', 1.6, 'DisplayName', 'a_{true} (ref)');
    ylabel('a_z  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hM hH hT], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    fig2 = fullfile(out_dir, 'fig2_am_vs_ahat.png');
    exportgraphics(f2, fig2, 'Resolution', 150);

    fprintf('  [fig1] %s\n', fig1);
    fprintf('  [fig2] %s\n', fig2);

    out = struct('t', t, 'a_true_z', a_true_z, 'a_hat_z', a_hat_z, ...
                 'a_xm_z', a_xm_z, 'err_z', err_z, 'relerr_z', relerr_z, ...
                 'fig1', fig1, 'fig2', fig2);
end
