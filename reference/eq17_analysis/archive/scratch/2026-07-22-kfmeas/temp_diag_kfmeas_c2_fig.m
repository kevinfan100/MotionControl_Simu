function temp_diag_kfmeas_c2_fig()
% TEMP diagnostic (chat 2026-07-22): C2 on-vs-off figure. Panel 1 = descent zoom
% (true/off/on), panel 2 = per-segment rel-err bars (6-seed means). Project style.
% Delete after use.

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
    base.use_taylor_gain = true;
    base.aprime_source = 'kfmeas'; base.aprime_learn_t0 = 0;
    base.aprime_state_kappa = 0; base.aprime_state_Q55_floor = 1e-12;

    Pp = calc_simulation_params(base); Pp = Pp.Value; Ts = Pp.common.Ts;
    t1 = base.t_hold; t2 = t1 + base.t_descend_override; t3 = t2 + base.n_cycles / base.frequency;

    c0 = base; c0.use_c2 = false;
    c1 = base; c1.use_c2 = true;
    s0 = temp_run_pure_sim_kfmeas(c0, struct('seed', 1, 'collect_diag', true));
    s1 = temp_run_pure_sim_kfmeas(c1, struct('seed', 1, 'collect_diag', true));
    N = size(s0.diag.a_hat, 1); t = (0:N - 1)' * Ts;
    at = s0.a_true_out(:, 3) * 1e3; ah0 = s0.diag.a_hat(:, 3) * 1e3; ah1 = s1.diag.a_hat(:, 3) * 1e3;

    % 6-seed means (from the completed sweep)
    seg_off = [12.19, 8.53]; seg_on = [6.53, 7.52];   % [descent, osc]

    COL_TRUE = [0.8 0 0]; COL_ON = [0 0.2 0.9]; COL_OFF = [0.55 0.55 0.55];
    FS = 18; LFS = 13; AXLW = 2.0;
    f = figure('Position', [70 70 980 780], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    % Panel 1: descent zoom
    nexttile; hold on;
    zi = t >= 0.4 & t <= 1.7;
    plot(t(zi), at(zi),  '-', 'Color', COL_TRUE, 'LineWidth', 2.6, 'DisplayName', 'a_z true');
    plot(t(zi), ah0(zi), '-', 'Color', COL_OFF,  'LineWidth', 1.4, 'DisplayName', 'C_2 off');
    plot(t(zi), ah1(zi), '-', 'Color', COL_ON,   'LineWidth', 1.6, 'DisplayName', 'C_2 on');
    xline(t1, ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.2, 'HandleVisibility', 'off');
    xline(t2, ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.2, 'HandleVisibility', 'off');
    xlim([0.4 1.7]);
    ylabel('a_z  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('descent window  (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    % Panel 2: per-segment rel-err bars (6-seed mean)
    nexttile; hold on;
    bh = bar([seg_off; seg_on]', 'grouped');
    bh(1).FaceColor = COL_OFF; bh(2).FaceColor = COL_ON;
    bh(1).DisplayName = 'C_2 off'; bh(2).DisplayName = 'C_2 on';
    set(gca, 'XTick', [1 2], 'XTickLabel', {'descent', 'osc'});
    ylabel('\^a_z rel err  (%)', 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    for i = 1:2
        text(i - 0.15, seg_off(i) + 0.4, sprintf('%.1f', seg_off(i)), 'FontSize', 12, 'HorizontalAlignment', 'center');
        text(i + 0.15, seg_on(i) + 0.4, sprintf('%.1f', seg_on(i)), 'FontSize', 12, 'HorizontalAlignment', 'center', 'Color', COL_ON);
    end
    ylim([0 15]);

    fn = fullfile(out_dir, 'kfmeas_c2_verify.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('saved %s\n', fn);
end
