function out = temp_plot_delpm_z_1hz(seed)
%TEMP_PLOT_DELPM_Z_1HZ  del_pm (delta_x_m) z-axis, 4-state AR(1), 1 Hz, 1 seed.
%   del_pm[k] = p_d[k-d] - p_m[k]  (delayed position-error measurement).
%   Project figure style (no grid, bold axes). TEMP -- delete after use.

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
    az = 3;                                    % z axis
    N = size(s.diag.delta_x_m, 1);
    t = (1:N).' * Ts;
    del_pm_z = s.diag.delta_x_m(:, az);        % [um]

    fprintf('[4-state AR(1) @ %gHz, seed %d, z-axis]\n', f, seed);
    fprintf('  del_pm: std = %.2f nm, mean = %+.2f nm\n', 1e3*std(del_pm_z), 1e3*mean(del_pm_z));

    out_dir = fullfile(here, 'test_results', 'temp_4state_ar1', sprintf('f%gHz_seed%d', f, seed));
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end
    FS = 18; AXLW = 2.0; COL = [0 0.45 0.74];

    fig = figure('Position', [80 80 1000 460], 'Color', 'w', 'NumberTitle', 'off');
    hold on;
    plot(t, zeros(size(t)), 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    plot(t, del_pm_z * 1e3, '-', 'Color', COL, 'LineWidth', 1.4);
    ylabel('\delta x_{m,z}  (nm)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    fig_path = fullfile(out_dir, 'fig5_delpm_z.png');
    exportgraphics(fig, fig_path, 'Resolution', 150);
    fprintf('  [fig] %s\n', fig_path);
    out = struct('t', t, 'del_pm_z', del_pm_z, 'fig', fig_path);
end
