function temp_diag_kfmeas_layers()
% TEMP diagnostic (chat 2026-07-22): three-layer comparison with the NEW init P55
% default (100x smaller). All init_from_anom=1 (honest). Layers:
%   L1 level-only (a'=0)  ->  L2 + a'-state  ->  L3 + a'-state + C2.
% Panel 1 = full a_hat_z trajectory; Panel 2 = descent/osc rel-err bars. Delete after.

    [sd, ~, ~] = fileparts(mfilename('fullpath')); pr = fileparts(sd);
    addpath(fullfile(pr, 'model'), fullfile(pr, 'model', 'config'), ...
            fullfile(pr, 'model', 'wall_effect'), fullfile(pr, 'model', 'thermal_force'), ...
            fullfile(pr, 'model', 'trajectory'), fullfile(pr, 'model', 'controller'), ...
            fullfile(pr, 'model', 'dual_track'), sd);
    out_dir = fullfile(pr, 'test_results', 'temp_var_centered_figs');
    if ~exist(out_dir, 'dir'); mkdir(out_dir); end

    pc = physical_constants(); R = pc.R;
    base = user_config();
    base.eq17_variant = '4state'; base.trajectory_type = 'osc';
    base.h_init = 50; base.h_bottom = 2.0 * R; base.amplitude = 2.5;
    base.frequency = 1; base.n_cycles = 4; base.t_hold = 0.5; base.t_descend_override = 1.0;
    base.h_min = 1.05 * R;
    base.T_sim = base.t_hold + base.t_descend_override + base.n_cycles / base.frequency + 0.5;
    base.ctrl_enable = true; base.thermal_enable = true; base.meas_noise_enable = true;
    base.lambda_c = 0.7; base.a_pd = 0.05; base.a_cov = 0.05;
    base.meas_noise_std = [0.00062; 0.00057; 0.00331];
    base.suppress_xD = true; base.h_bar_safe = 1;
    base.use_taylor_gain = true; base.init_from_anom = true;
    base.aprime_source = 'kfmeas'; base.aprime_state_kappa = 0;

    Pp = calc_simulation_params(base); Pp = Pp.Value; Ts = Pp.common.Ts;
    t1 = base.t_hold; t2 = t1 + base.t_descend_override; t3 = t2 + base.n_cycles / base.frequency;
    relerr = @(ah, at, m) mean(abs(ah(m) - at(m)) ./ at(m)) * 100;

    cL1 = base; cL1.aprime_learn_t0 = 1e9; cL1.aprime_state_Q55_floor = 0; cL1.use_c2 = false;   % a'=0 level
    cL2 = base; cL2.aprime_learn_t0 = 0;   cL2.aprime_state_Q55_floor = 1e-12; cL2.use_c2 = false;  % + a'-state
    cL3 = base; cL3.aprime_learn_t0 = 0;   cL3.aprime_state_Q55_floor = 1e-12; cL3.use_c2 = true;   % + C2
    s1 = temp_run_pure_sim_kfmeas(cL1, struct('seed', 1, 'collect_diag', true));
    s2 = temp_run_pure_sim_kfmeas(cL2, struct('seed', 1, 'collect_diag', true));
    s3 = temp_run_pure_sim_kfmeas(cL3, struct('seed', 1, 'collect_diag', true));

    N = size(s2.diag.a_hat, 1); t = (0:N - 1)' * Ts;
    tg = s2.a_true_out(:, 3) * 1e3;
    a1 = s1.diag.a_hat(:, 3) * 1e3; a2 = s2.diag.a_hat(:, 3) * 1e3; a3 = s3.diag.a_hat(:, 3) * 1e3;
    ds = t >= t1 & t < t2; os = t >= t2 & t < t3;
    at = s2.a_true_out(:, 3);
    seg = [relerr(s1.diag.a_hat(:, 3), at, ds), relerr(s1.diag.a_hat(:, 3), at, os); ...
           relerr(s2.diag.a_hat(:, 3), at, ds), relerr(s2.diag.a_hat(:, 3), at, os); ...
           relerr(s3.diag.a_hat(:, 3), at, ds), relerr(s3.diag.a_hat(:, 3), at, os)];
    fprintf('\n=== three layers (new P55 default, init_from_anom=1, seed 1) ===\n');
    fprintf('L1 level-only : descent %.1f%%  osc %.1f%%\n', seg(1, 1), seg(1, 2));
    fprintf('L2 +a''-state  : descent %.1f%%  osc %.1f%%\n', seg(2, 1), seg(2, 2));
    fprintf('L3 +C2        : descent %.1f%%  osc %.1f%%\n', seg(3, 1), seg(3, 2));

    C_TRUE = [0.8 0 0]; C1 = [0.6 0.6 0.6]; C2c = [0.93 0.5 0.13]; C3 = [0 0.2 0.9];
    FS = 18; LFS = 12; AXLW = 2.0;
    f = figure('Position', [70 70 1000 780], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile; hold on;
    plot(t, tg, '-', 'Color', C_TRUE, 'LineWidth', 2.6, 'DisplayName', 'a_z true');
    plot(t, a1, '-', 'Color', C1, 'LineWidth', 1.2, 'DisplayName', 'L1 level-only (a''=0)');
    plot(t, a2, '-', 'Color', C2c, 'LineWidth', 1.2, 'DisplayName', 'L2 + a''-state');
    plot(t, a3, '-', 'Color', C3, 'LineWidth', 1.4, 'DisplayName', 'L3 + C2');
    for x = [t1 t2 t3]; xline(x, ':', 'Color', [0.65 0.65 0.65], 'LineWidth', 1.2, 'HandleVisibility', 'off'); end
    xlim([0 t(end)]); ylim([4 17]);
    ylabel('\^a_z  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    nexttile; hold on;
    bh = bar(seg, 'grouped');
    bh(1).FaceColor = [0.35 0.35 0.35]; bh(1).DisplayName = 'descent';
    bh(2).FaceColor = [0.2 0.45 0.85];  bh(2).DisplayName = 'osc';
    set(gca, 'XTick', 1:3, 'XTickLabel', {'L1 level-only', 'L2 +a''-state', 'L3 +C2'});
    ylabel('\^a_z rel err  (%)', 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    for i = 1:3
        text(i - 0.15, seg(i, 1) + 1, sprintf('%.0f', seg(i, 1)), 'FontSize', 11, 'HorizontalAlignment', 'center');
        text(i + 0.15, seg(i, 2) + 1, sprintf('%.0f', seg(i, 2)), 'FontSize', 11, 'HorizontalAlignment', 'center');
    end
    ylim([0 max(seg(:)) * 1.2]);

    fn = fullfile(out_dir, 'kfmeas_three_layers.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('saved %s\n', fn);
end
