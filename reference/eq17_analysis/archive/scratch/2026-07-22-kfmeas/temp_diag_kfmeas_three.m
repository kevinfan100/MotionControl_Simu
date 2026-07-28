function temp_diag_kfmeas_three()
% TEMP diagnostic (chat 2026-07-22): the THREE approaches, wall-aware ("cheat")
% init + P55 another 100x smaller. All in the kfmeas controller (aprime_source
% switch). Layers:
%   S1 a'-state (aprime_source='state', original: a' state, NO direct y2 meas)
%   S2 kfmeas   (aprime_source='kfmeas': a_d remodel + a' measured via H2(5))
%   S3 kfmeas + C2
% Panel 1 = full a_hat_z; Panel 2 = descent/osc rel-err bars. Delete after use.

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
    base.init_from_anom = false;                 % CHEAT: init at a_nom/c(h_init) = true far gain
    base.aprime_state_kappa = 0; base.aprime_learn_t0 = 0;

    Pp = calc_simulation_params(base); Pp = Pp.Value; Ts = Pp.common.Ts;
    a_nom = Pp.common.Ts / Pp.ctrl.gamma;
    base.aprime_state_P0 = (0.001 * a_nom / R)^2;   % 100x smaller than the (0.01)^2 default
    t1 = base.t_hold; t2 = t1 + base.t_descend_override; t3 = t2 + base.n_cycles / base.frequency;
    relerr = @(ah, at, m) mean(abs(ah(m) - at(m)) ./ at(m)) * 100;

    cS1 = base; cS1.aprime_source = 'state';  cS1.aprime_state_Q55_floor = 1e-12; cS1.use_c2 = false;
    cS2 = base; cS2.aprime_source = 'kfmeas'; cS2.aprime_state_Q55_floor = 1e-12; cS2.use_c2 = false;
    cS3 = base; cS3.aprime_source = 'kfmeas'; cS3.aprime_state_Q55_floor = 1e-12; cS3.use_c2 = true;
    s1 = temp_run_pure_sim_kfmeas(cS1, struct('seed', 1, 'collect_diag', true));
    s2 = temp_run_pure_sim_kfmeas(cS2, struct('seed', 1, 'collect_diag', true));
    s3 = temp_run_pure_sim_kfmeas(cS3, struct('seed', 1, 'collect_diag', true));

    N = size(s2.diag.a_hat, 1); t = (0:N - 1)' * Ts;
    tg = s2.a_true_out(:, 3) * 1e3; at = s2.a_true_out(:, 3);
    a1 = s1.diag.a_hat(:, 3) * 1e3; a2 = s2.diag.a_hat(:, 3) * 1e3; a3 = s3.diag.a_hat(:, 3) * 1e3;
    ds = t >= t1 & t < t2; os = t >= t2 & t < t3;
    seg = [relerr(s1.diag.a_hat(:, 3), at, ds), relerr(s1.diag.a_hat(:, 3), at, os); ...
           relerr(s2.diag.a_hat(:, 3), at, ds), relerr(s2.diag.a_hat(:, 3), at, os); ...
           relerr(s3.diag.a_hat(:, 3), at, ds), relerr(s3.diag.a_hat(:, 3), at, os)];
    fprintf('\n=== three approaches (cheat init, P55 x100 smaller, seed 1) ===\n');
    fprintf('S1 a''-state (state) : descent %.1f%%  osc %.1f%%\n', seg(1, :));
    fprintf('S2 kfmeas           : descent %.1f%%  osc %.1f%%\n', seg(2, :));
    fprintf('S3 kfmeas + C2      : descent %.1f%%  osc %.1f%%\n', seg(3, :));

    C_TRUE = [0.8 0 0]; C1 = [0.55 0.55 0.55]; C2c = [0.93 0.5 0.13]; C3 = [0 0.2 0.9];
    FS = 18; LFS = 12; AXLW = 2.0;
    f = figure('Position', [70 70 1000 780], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile; hold on;
    plot(t, tg, '-', 'Color', C_TRUE, 'LineWidth', 2.6, 'DisplayName', 'a_z true');
    plot(t, a1, '-', 'Color', C1, 'LineWidth', 1.2, 'DisplayName', 'S1 a''-state');
    plot(t, a2, '-', 'Color', C2c, 'LineWidth', 1.2, 'DisplayName', 'S2 kfmeas');
    plot(t, a3, '-', 'Color', C3, 'LineWidth', 1.4, 'DisplayName', 'S3 kfmeas + C2');
    for x = [t1 t2 t3]; xline(x, ':', 'Color', [0.65 0.65 0.65], 'LineWidth', 1.2, 'HandleVisibility', 'off'); end
    xlim([0 t(end)]); ylim([4 17]);
    ylabel('\^a_z  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    nexttile; hold on;
    bh = bar(seg, 'grouped');
    bh(1).FaceColor = [0.35 0.35 0.35]; bh(1).DisplayName = 'descent';
    bh(2).FaceColor = [0.2 0.45 0.85];  bh(2).DisplayName = 'osc';
    set(gca, 'XTick', 1:3, 'XTickLabel', {'S1 a''-state', 'S2 kfmeas', 'S3 kfmeas+C2'});
    ylabel('\^a_z rel err  (%)', 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    for i = 1:3
        text(i - 0.15, seg(i, 1) + 0.5, sprintf('%.1f', seg(i, 1)), 'FontSize', 11, 'HorizontalAlignment', 'center');
        text(i + 0.15, seg(i, 2) + 0.5, sprintf('%.1f', seg(i, 2)), 'FontSize', 11, 'HorizontalAlignment', 'center');
    end
    ylim([0 max(seg(:)) * 1.25]);

    fn = fullfile(out_dir, 'kfmeas_three_approaches.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('saved %s\n', fn);
end
