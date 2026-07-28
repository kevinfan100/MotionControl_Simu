function temp_diag_kfmeas_anom()
% TEMP diagnostic (chat 2026-07-22): HONEST charter-legal test. init_from_anom=1
% => a_d starts at a_nom (free-space, NO peek at c). Can the estimator learn the
% true position-dependent gain from a_nom alone? Delete after use.

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
    base.use_c2 = true;
    base.init_from_anom = true;                 % HONEST: start at a_nom, no c peek

    Pp = calc_simulation_params(base); Pp = Pp.Value; Ts = Pp.common.Ts;
    a_nom = Pp.common.Ts / Pp.ctrl.gamma;
    t1 = base.t_hold; t2 = t1 + base.t_descend_override; t3 = t2 + base.n_cycles / base.frequency;

    so = temp_run_pure_sim_kfmeas(base, struct('seed', 1, 'collect_diag', true));
    N = size(so.diag.a_hat, 1); t = (0:N - 1)' * Ts;
    tg = so.a_true_out(:, 3) * 1e3; ah = so.diag.a_hat(:, 3) * 1e3;
    anom_nm = a_nom * 1e3;

    re = @(m) mean(abs(ah(m) - tg(m)) ./ tg(m)) * 100;
    hf = t < t1; ds = t >= t1 & t < t2; os = t >= t2 & t < t3; hn = t >= t3;
    fprintf('\n=== HONEST from-a_nom (init_from_anom=1), 1 Hz, seed 1 ===\n');
    fprintf('a_nom=%.2f, true far=%.2f nm/pN\n', anom_nm, mean(tg(hf)));
    fprintf('rel err: far-hold=%.2f%%  descent=%.2f%%  osc=%.2f%%  near-hold=%.2f%%\n', ...
            re(hf), re(ds), re(os), re(hn));

    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; COL_NOM = [0.5 0.5 0.5];
    FS = 18; LFS = 13; AXLW = 2.0;
    f = figure('Position', [70 70 980 720], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile; hold on;
    yline(anom_nm, '--', 'Color', COL_NOM, 'LineWidth', 2.0, 'DisplayName', 'a_{nom} (only prior, init)');
    plot(t, tg, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_z true');
    plot(t, ah, '-', 'Color', COL_HAT, 'LineWidth', 1.4, 'DisplayName', '\^a_z estimate');
    for x = [t1 t2 t3]; xline(x, ':', 'Color', [0.65 0.65 0.65], 'LineWidth', 1.2, 'HandleVisibility', 'off'); end
    xlim([0 t(end)]); ylim([5 16]);
    ylabel('gain  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    text(t1 / 2, 5.6, 'hold', 'FontSize', 11, 'Color', COL_NOM, 'HorizontalAlignment', 'center');
    text((t1 + t2) / 2, 5.6, 'descent', 'FontSize', 11, 'Color', COL_NOM, 'HorizontalAlignment', 'center');
    text((t2 + t3) / 2, 5.6, 'osc', 'FontSize', 11, 'Color', COL_NOM, 'HorizontalAlignment', 'center');

    % first-hold zoom to show it stays stuck near a_nom
    nexttile; hold on;
    zi = t < 0.6;
    yline(anom_nm, '--', 'Color', COL_NOM, 'LineWidth', 2.0, 'DisplayName', 'a_{nom} init');
    plot(t(zi), tg(zi), '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_z true');
    plot(t(zi), ah(zi), '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', '\^a_z estimate');
    xline(t1, ':', 'Color', [0.65 0.65 0.65], 'LineWidth', 1.2, 'HandleVisibility', 'off');
    xlim([0 0.6]); ylim([13 15.2]);
    ylabel('gain  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('first-hold zoom  (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    fn = fullfile(out_dir, 'kfmeas_from_anom_honest.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('saved %s\n', fn);
end
