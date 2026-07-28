function out = temp_plot_aram_zoom3(seed)
%TEMP_PLOT_ARAM_ZOOM3  a_ram = a - a_det(p_d), true vs EKF estimate, 3 zoom windows.
%   a_ram_true[k] = a_true[k] - a_det[k]   (a_det = diag.a_x_det, from p_d)
%   a_ram_hat[k]  = a_hat[k]  - a_det[k]
%   Zoom windows (auto-located from the run): trough (near-wall min a),
%   peak (far-wall max a), hold (post-oscillation static near-wall segment).
%   Scenario mirrors temp_test_4state_ar1_1hz.m (4-state AR(1), 1 Hz osc).
%   TEMP -- delete after use.   usage: temp_plot_aram_zoom3(1)

    if nargin < 1 || isempty(seed); seed = 1; end
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
    a_det  = s.diag.a_x_det(2:end, az);
    N = numel(a_true);
    t = (1:N).' * Ts;

    a_ram_true = a_true - a_det;
    a_ram_hat  = a_hat  - a_det;

    % --- locate the 3 zoom windows from the actual run ---
    osc_lo = cfg.t_hold + cfg.t_descend_override;        % descent ends / osc starts
    osc_hi = osc_lo + cfg.n_cycles / cfg.frequency;       % osc ends / final hold starts
    mid_win = t >= (osc_lo + 0.5) & t <= (osc_hi - 0.5);  % avoid both transition edges

    t_mid = t(mid_win);
    a_true_mid = a_true(mid_win);
    [~, i_trough] = min(a_true_mid);
    [~, i_peak]   = max(a_true_mid);
    t_trough = t_mid(i_trough);
    t_peak   = t_mid(i_peak);
    t_hold_c = osc_hi + 0.25;                             % centre of post-osc hold segment

    half_w = 0.15;
    win_trough = [t_trough - half_w, t_trough + half_w];
    win_peak   = [t_peak   - half_w, t_peak   + half_w];
    win_hold   = [t_hold_c - half_w, t_hold_c + half_w];
    wins = {win_trough, win_peak, win_hold};
    labels = {'trough', 'peak', 'hold'};

    fprintf('[a_ram zoom3] trough @ %.3fs, peak @ %.3fs, hold @ [%.2f %.2f]s\n', ...
            t_trough, t_peak, win_hold(1), win_hold(2));

    out_dir = fullfile(here, 'test_results', 'temp_4state_ar1', sprintf('f%gHz_seed%d', f, seed));
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    FS = 14; LFS = 12; AXLW = 1.6;
    COL_T = [0 0 0]; COL_H = [0 0.45 0.74]; COL_Z = [0.85 0.85 0.88];

    fig = figure('Position', [60 60 1400 720], 'Color', 'w', 'NumberTitle', 'off');

    % --- overview (top, full width) ---
    ax0 = subplot(2, 3, 1:3); hold on;
    hT = plot(t, a_ram_true * 1e3, '-', 'Color', COL_T, 'LineWidth', 1.2, 'DisplayName', 'a_{ram,true}');
    hH = plot(t, a_ram_hat  * 1e3, '-', 'Color', COL_H, 'LineWidth', 1.2, 'DisplayName', 'a_{ram,hat}');
    yl = ylim;
    for i = 1:3
        w = wins{i};
        patch([w(1) w(2) w(2) w(1)], [yl(1) yl(1) yl(2) yl(2)], COL_Z, ...
              'EdgeColor', 'none', 'FaceAlpha', 0.6, 'HandleVisibility', 'off');
    end
    ylim(yl);
    uistack(hT, 'top'); uistack(hH, 'top');
    ylabel('a_{ram,z}  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hT hH], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(ax0, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    % --- 3 zoom panels (bottom row) ---
    for i = 1:3
        w = wins{i};
        axz = subplot(2, 3, 3 + i); hold on;
        iz = t >= w(1) & t <= w(2);
        plot(t(iz), a_ram_true(iz) * 1e3, '-', 'Color', COL_T, 'LineWidth', 1.6);
        plot(t(iz), a_ram_hat(iz)  * 1e3, '-', 'Color', COL_H, 'LineWidth', 1.6);
        xlim(w);
        xlabel(sprintf('Time (sec)  [%s, %.2f-%.2fs]', labels{i}, w(1), w(2)), ...
               'FontSize', FS - 2, 'FontWeight', 'bold');
        if i == 1
            ylabel('a_{ram,z}  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
        end
        set(axz, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    end

    fig_path = fullfile(out_dir, 'fig8_aram_zoom3_z.png');
    exportgraphics(fig, fig_path, 'Resolution', 150);
    fprintf('  [fig] %s\n', fig_path);

    out = struct('t', t, 'a_ram_true', a_ram_true, 'a_ram_hat', a_ram_hat, ...
                 'win_trough', win_trough, 'win_peak', win_peak, 'win_hold', win_hold, ...
                 'fig', fig_path);
end
