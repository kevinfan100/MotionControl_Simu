function out = temp_plot_ahat_zoom_z(seed, zoom_win)
%TEMP_PLOT_AHAT_ZOOM_Z  a_hat (z) with a local-zoom subplot, 4-state AR(1), 1 Hz.
%   Top   : full a_hat_z (0-T), a_true_z reference, shaded zoom window.
%   Bottom: zoom into zoom_win to see the oscillation/jitter of a_hat.
%   TEMP -- delete after use.   usage: temp_plot_ahat_zoom_z(1, [2.0 2.3])

    if nargin < 1 || isempty(seed);     seed = 1;            end
    if nargin < 2 || isempty(zoom_win); zoom_win = [2.0 2.3]; end
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
    az = 3;
    a_true = s.a_true_out(2:end, az);
    a_hat  = s.diag.a_hat(2:end, az);
    N = numel(a_true);
    t = (1:N).' * Ts;

    iz = t >= zoom_win(1) & t <= zoom_win(2);
    fprintf('[4-state AR(1) @ %gHz, seed %d, z-axis]\n', f, seed);
    fprintf('  a_hat_z: full std = %.3f nm/pN\n', 1e3*std(a_hat));
    fprintf('  zoom [%.2f %.2f]s: a_hat std = %.3f nm/pN about local mean (jitter)\n', ...
            zoom_win, 1e3*std(a_hat(iz) - mean(a_hat(iz))));

    out_dir = fullfile(here, 'test_results', 'temp_4state_ar1', sprintf('f%gHz_seed%d', f, seed));
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    FS = 17; LFS = 13; AXLW = 2.0;
    COL_T = [0 0 0]; COL_H = [0 0.45 0.74]; COL_Z = [0.85 0.85 0.88];

    fig = figure('Position', [80 80 1050 760], 'Color', 'w', 'NumberTitle', 'off');

    % --- top: full ---
    ax1 = subplot(2,1,1); hold on;
    hT = plot(t, a_true*1e3, '-', 'Color', COL_T, 'LineWidth', 1.4, 'DisplayName', 'a_{true}');
    hH = plot(t, a_hat*1e3,  '-', 'Color', COL_H, 'LineWidth', 1.4, 'DisplayName', 'a_{hat}');
    yl = ylim;
    pz = patch([zoom_win(1) zoom_win(2) zoom_win(2) zoom_win(1)], [yl(1) yl(1) yl(2) yl(2)], ...
               COL_Z, 'EdgeColor', 'none', 'FaceAlpha', 0.6, 'HandleVisibility', 'off');
    uistack(pz, 'bottom'); ylim(yl);
    ylabel('a_z  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hT hH], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(ax1, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    % --- bottom: zoom ---
    ax2 = subplot(2,1,2); hold on;
    plot(t(iz), a_true(iz)*1e3, '-', 'Color', COL_T, 'LineWidth', 1.6);
    plot(t(iz), a_hat(iz)*1e3,  '-', 'Color', COL_H, 'LineWidth', 1.6);
    xlim(zoom_win);
    ylabel('a_z  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(sprintf('Time (sec)   [zoom %.2f-%.2f s]', zoom_win(1), zoom_win(2)), ...
           'FontSize', FS, 'FontWeight', 'bold');
    set(ax2, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    fig_path = fullfile(out_dir, 'fig7_ahat_zoom_z.png');
    exportgraphics(fig, fig_path, 'Resolution', 150);
    fprintf('  [fig] %s\n', fig_path);
    out = struct('t', t, 'a_hat', a_hat, 'a_true', a_true, 'zoom_win', zoom_win, 'fig', fig_path);
end
