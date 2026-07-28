function out = temp_plot_true_vs_pm_z(seed)
%TEMP_PLOT_TRUE_VS_PM_Z  true motion error vs pm error (del_pm), z-axis, 4-state AR(1), 1 Hz.
%   true motion error : delta_x[m] = p_d[m] - x[m]   (clean, current)
%   pm error          : del_pm[k]  = delta_x[k-d] + n_x   (delayed + sensor noise)
%   del_pm is delay-aligned (shift back by d) so it overlays the instant it measures;
%   the residual (del_pm - true) is then just the sensor noise n_x.
%   Project figure style. TEMP -- delete after use.

    if nargin < 1; seed = 1; end
    here = fileparts(mfilename('fullpath'));
    addpath(genpath(here));

    f = 1.0; T_sim = 4.0;
    cfg = user_config();
    cfg.eq17_variant   = '4state';
    cfg.trajectory_type = 'osc';
    cfg.h_init   = 50;  cfg.h_bottom = 2.7;  cfg.amplitude = 2.5;
    cfg.frequency = f;  cfg.n_cycles = 2 * f;  cfg.t_hold = 0.5;
    cfg.t_descend_override = 1.0;  cfg.T_sim = T_sim;
    pc = physical_constants();  cfg.h_min = 1.05 * pc.R;
    cfg.ctrl_enable = true;  cfg.thermal_enable = true;  cfg.meas_noise_enable = true;
    cfg.lambda_c = 0.7;  cfg.a_pd = 0.05;  cfg.a_cov = 0.05;
    cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
    cfg.suppress_xD = true;  cfg.h_bar_safe = 1;  cfg.use_am_lpf = false;  cfg.a_det = 0.005;
    cfg.use_q44_ar1 = true;

    ro = struct('seed', seed, 'verbose', false, 'collect_diag', true, 'use_true_gain', false);
    s = run_pure_simulation(cfg, ro);

    Ts = s.meta.params_value.common.Ts;
    if isfield(s, 'ctrl_const') && isfield(s.ctrl_const, 'd'); d = s.ctrl_const.d; else; d = 2; end
    az = 3;                                       % z axis
    N = size(s.p_d_out, 1);

    true_dx = s.p_d_out(:, az) - [0; s.p_true_out(1:end-1, az)];   % delta_x[m] = p_d[m]-x[m] (m>=2 valid)
    del_pm  = s.diag.delta_x_m(:, az);                              % del_pm[k] = dx[k-d]+n_x

    % delay-align: pair true_dx[m] with del_pm[m+d]
    m   = (2:(N-d)).';
    tru = true_dx(m);
    mea = del_pm(m + d);
    t   = m * Ts;
    resid = mea - tru;                                              % ~ n_x

    fprintf('[4-state AR(1) @ %gHz, seed %d, z-axis]  (d=%d)\n', f, seed, d);
    fprintf('  true dx std       = %.2f nm\n', 1e3*std(tru));
    fprintf('  del_pm (aligned)  = %.2f nm\n', 1e3*std(mea));
    fprintf('  residual std      = %.2f nm  (expect ~ sensor noise %.2f nm)\n', ...
            1e3*std(resid), 1e3*cfg.meas_noise_std(az));
    fprintf('  same-index resid  = %.2f nm  (delay NOT aligned, includes d-step decorrelation)\n', ...
            1e3*std(del_pm(m) - tru));

    out_dir = fullfile(here, 'test_results', 'temp_4state_ar1', sprintf('f%gHz_seed%d', f, seed));
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    FS = 18; LFS = 14; AXLW = 2.0;
    COL_T = [0 0 0]; COL_M = [0.85 0.33 0.10];

    fig = figure('Position', [80 80 1000 460], 'Color', 'w', 'NumberTitle', 'off');
    hold on;
    hM = plot(t, mea*1e3, '-', 'Color', COL_M, 'LineWidth', 1.2, 'DisplayName', '\delta x_m (pm error, delay-aligned)');
    hT = plot(t, tru*1e3, '-', 'Color', COL_T, 'LineWidth', 1.6, 'DisplayName', '\delta x (true motion error)');
    ylabel('\delta x_z  (nm)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hT hM], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    fig_path = fullfile(out_dir, 'fig6_true_vs_pm_z.png');
    exportgraphics(fig, fig_path, 'Resolution', 150);
    fprintf('  [fig] %s\n', fig_path);
    out = struct('t', t, 'true_dx', tru, 'del_pm', mea, 'resid', resid, 'fig', fig_path);
end
