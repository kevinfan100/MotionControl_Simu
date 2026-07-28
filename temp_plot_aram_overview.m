function out = temp_plot_aram_overview(seed)
%TEMP_PLOT_ARAM_OVERVIEW  a_ram = a - a_det(p_d), true vs EKF estimate, full timeline only.
%   Same signals as temp_plot_aram_zoom3.m, no zoom-in panels, no shading.
%   Scenario mirrors temp_test_4state_ar1_1hz.m (4-state AR(1), 1 Hz osc).
%   TEMP -- delete after use.   usage: temp_plot_aram_overview(1)

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

    out_dir = fullfile(here, 'test_results', 'temp_4state_ar1', sprintf('f%gHz_seed%d', f, seed));
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    FS = 17; LFS = 13; AXLW = 2.0;
    COL_T = [0 0 0]; COL_H = [0 0.45 0.74];

    fig = figure('Position', [80 80 1100 500], 'Color', 'w', 'NumberTitle', 'off');
    hold on;
    hT = plot(t, a_ram_true * 1e3, '-', 'Color', COL_T, 'LineWidth', 1.2, 'DisplayName', 'a_{ram,true}');
    hH = plot(t, a_ram_hat  * 1e3, '-', 'Color', COL_H, 'LineWidth', 1.4, 'DisplayName', 'a_{ram,hat}');
    ylabel('a_{ram,z}  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hT hH], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    fig_path = fullfile(out_dir, 'fig9_aram_overview_z.png');
    exportgraphics(fig, fig_path, 'Resolution', 150);
    fprintf('  [fig] %s\n', fig_path);

    out = struct('t', t, 'a_ram_true', a_ram_true, 'a_ram_hat', a_ram_hat, 'fig', fig_path);
end
