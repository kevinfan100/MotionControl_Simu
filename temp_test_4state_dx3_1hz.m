function out = temp_test_4state_dx3_1hz(seed)
%TEMP_TEST_4STATE_DX3_1HZ  4-state (AR(1)) on 1 Hz near-wall osc, z-axis, 1 seed.
%   Fig 1: estimated dx_hat_3 vs true dx (overlay)
%   Fig 2: estimation error (dx_hat_3 - dx)
%   True dx[k] = p_d[k] - x[k]; driver logs p_true_out[k]=x[k+1], p_d_out[k]=p_d[k]
%   => true dx[k] = p_d_out[k] - p_true_out[k-1].
%   TEMP -- delete after use.

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
    az = 3;                                   % z axis
    N = size(s.p_d_out, 1);
    k = (2:N).';
    t = k * Ts;
    dx_true = s.p_d_out(k, az) - s.p_true_out(k-1, az);   % true dx_z[k]  [um]
    dx_hat3 = s.diag.delta_x_hat_3(k, az);                % estimate      [um]
    err     = dx_hat3 - dx_true;

    relrms = rms(err) / rms(dx_true);
    fprintf('[4-state AR(1) @ %gHz, seed %d, z-axis]\n', f, seed);
    fprintf('  dx3 err: rms = %.2f nm, std = %.2f nm (true dx rms = %.2f nm, rel rms = %.1f%%)\n', ...
            1e3*rms(err), 1e3*std(err), 1e3*rms(dx_true), 100*relrms);

    out_dir = fullfile(here, 'test_results', 'temp_4state_ar1', sprintf('f%gHz_seed%d', f, seed));
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    FS = 18; LFS = 14; AXLW = 2.0;
    COL_T = [0 0 0]; COL_H = [0 0.45 0.74];

    % Fig 1: overlay
    f1 = figure('Position', [80 80 1000 460], 'Color', 'w', 'NumberTitle', 'off');
    hold on;
    hT = plot(t, dx_true*1e3, '-',  'Color', COL_T, 'LineWidth', 1.8, 'DisplayName', '\delta x (true)');
    hH = plot(t, dx_hat3*1e3, '-',  'Color', COL_H, 'LineWidth', 1.6, 'DisplayName', '\delta x_3 hat');
    ylabel('\delta x_z  (nm)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hT hH], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    fig1 = fullfile(out_dir, 'fig3_dx3_vs_true.png');
    exportgraphics(f1, fig1, 'Resolution', 150);

    % Fig 2: error
    f2 = figure('Position', [80 80 1000 460], 'Color', 'w', 'NumberTitle', 'off');
    hold on;
    plot(t, zeros(size(t)), 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    plot(t, err*1e3, '-', 'Color', COL_H, 'LineWidth', 1.6);
    ylabel('\delta x_3 hat error  (nm)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    fig2 = fullfile(out_dir, 'fig4_dx3_error.png');
    exportgraphics(f2, fig2, 'Resolution', 150);

    fprintf('  [fig1] %s\n  [fig2] %s\n', fig1, fig2);
    out = struct('t', t, 'dx_true', dx_true, 'dx_hat3', dx_hat3, 'err', err, ...
                 'fig1', fig1, 'fig2', fig2);
end
