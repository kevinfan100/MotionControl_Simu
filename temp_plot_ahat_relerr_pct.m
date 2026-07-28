function out = temp_plot_ahat_relerr_pct(seed)
%TEMP_PLOT_AHAT_RELERR_PCT  a_hat relative error vs a_true, in percent.
%   pct_err[k] = (a_hat[k] - a_true[k]) / a_true[k] * 100
%   Same scenario as temp_test_4state_ar1_1hz.m (4-state AR(1), 1 Hz osc).
%   TEMP -- delete after use.   usage: temp_plot_ahat_relerr_pct(1)

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
    N = numel(a_true);
    t = (1:N).' * Ts;

    pct_err = (a_hat - a_true) ./ a_true * 100;
    med_abs_pct = median(abs(pct_err));
    fprintf('[a_hat relerr %%] median |pct_err| = %.2f%%, mean = %+.2f%%, std = %.2f%%\n', ...
            med_abs_pct, mean(pct_err), std(pct_err));

    out_dir = fullfile(here, 'test_results', 'temp_4state_ar1', sprintf('f%gHz_seed%d', f, seed));
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    FS = 17; LFS = 13; AXLW = 2.0;
    COL_E = [0.85 0.33 0.10];

    fig = figure('Position', [80 80 1100 500], 'Color', 'w', 'NumberTitle', 'off');
    hold on;
    plot(t, zeros(size(t)), 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    plot(t, pct_err, '-', 'Color', COL_E, 'LineWidth', 1.2);
    ylabel('(a_{hat} - a_{true}) / a_{true} \times 100  (%)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    text(0.02, 0.95, sprintf('median abs err = %.2f%%', med_abs_pct), ...
         'Units', 'normalized', 'FontSize', LFS, 'FontWeight', 'bold', ...
         'VerticalAlignment', 'top');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    fig_path = fullfile(out_dir, 'fig10_ahat_relerr_pct_z.png');
    exportgraphics(fig, fig_path, 'Resolution', 150);
    fprintf('  [fig] %s\n', fig_path);

    out = struct('t', t, 'pct_err', pct_err, 'med_abs_pct', med_abs_pct, 'fig', fig_path);
end
