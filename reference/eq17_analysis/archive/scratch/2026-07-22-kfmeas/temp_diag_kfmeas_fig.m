function temp_diag_kfmeas_fig()
% TEMP diagnostic (chat 2026-07-22): Option A verification figure for the kfmeas
% z-state, PROJECT STYLE (True=red, Estimate=blue, FS18 bold, no grid, box on,
% legend northoutside). 3 panels: (1) gain a_hat_z vs a_true, (2) a' estimate vs
% oracle, (3) first-hold error a_hat_z - a_z. Delete after use.

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

    Pp = calc_simulation_params(base); Pp = Pp.Value; Ts = Pp.common.Ts;
    t1 = base.t_hold; t2 = t1 + base.t_descend_override; t3 = t2 + base.n_cycles / base.frequency;

    cA = base; cA.aprime_source = 'kfmeas'; cA.aprime_learn_t0 = 0;
    cA.aprime_state_kappa = 0; cA.aprime_state_Q55_floor = 1e-12;
    cO = base; cO.aprime_source = 'known';
    soA = temp_run_pure_sim_kfmeas(cA, struct('seed', 1, 'collect_diag', true));
    soO = run_pure_simulation(cO, struct('seed', 1, 'collect_diag', true));

    N = size(soA.diag.a_hat, 1); t = (0:N - 1)' * Ts;
    ahz = soA.diag.a_hat(:, 3);                % estimate gain [um/pN]
    atz = soA.a_true_out(:, 3);               % true gain     [um/pN]
    apA = soA.diag.a_prime_used(:, 3);        % a' estimate   [1/pN]
    apO = soO.diag.a_prime_used(:, 3);        % a' oracle     [1/pN]

    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; GRY = [0.5 0.5 0.5];
    FS = 18; LFS = 13; AXLW = 2.0;
    f = figure('Position', [70 70 980 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    % ---- Panel 1: gain estimate vs true (full trajectory) ----
    nexttile; hold on;
    plot(t, atz, '-', 'Color', COL_TRUE, 'LineWidth', 2.5, 'DisplayName', 'a_z true');
    plot(t, ahz, '-', 'Color', COL_HAT,  'LineWidth', 1.6, 'DisplayName', '\^a_z estimate');
    for x = [t1 t2 t3]; xline(x, ':', 'Color', GRY, 'LineWidth', 1.2, 'HandleVisibility', 'off'); end
    xlim([0 t(end)]); ylim([0.005 0.020]);
    ylabel('a_z  (\mum/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    ytxt = 0.0062;
    lab = {'hold', t1 / 2; 'descent', (t1 + t2) / 2; 'osc', (t2 + t3) / 2; 'hold', (t3 + t(end)) / 2};
    for i = 1:4; text(lab{i, 2}, ytxt, lab{i, 1}, 'FontSize', 12, 'Color', GRY, 'HorizontalAlignment', 'center'); end

    % ---- Panel 2: a' estimate vs oracle (full trajectory) ----
    nexttile; hold on;
    plot(t, apO, '-', 'Color', COL_TRUE, 'LineWidth', 2.5, 'DisplayName', 'a'' oracle');
    plot(t, apA, '-', 'Color', COL_HAT,  'LineWidth', 1.6, 'DisplayName', 'a'' estimate');
    for x = [t1 t2 t3]; xline(x, ':', 'Color', GRY, 'LineWidth', 1.2, 'HandleVisibility', 'off'); end
    xlim([0 t(end)]);
    ylabel('a''  (1/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    % ---- Panel 3: first-hold error a_hat_z - a_z (nm/pN) ----
    nexttile; hold on;
    zi = t < t1;
    yline(0, ':', 'Color', GRY, 'LineWidth', 1.2, 'HandleVisibility', 'off');
    plot(t(zi), (ahz(zi) - atz(zi)) * 1e3, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', '\^a_z - a_z (first hold)');
    xlim([0 t1]);
    ylabel('error  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    fn = fullfile(out_dir, 'kfmeas_optA_verify.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('saved %s\n', fn);

    hf = t < t1; os = t >= t2 & t < t3;
    fprintf('first-hold: mean err=%+.4f nm/pN, std=%.4f nm/pN | osc rel err=%.2f%%\n', ...
            mean((ahz(hf) - atz(hf))) * 1e3, std((ahz(hf) - atz(hf))) * 1e3, ...
            mean(abs(ahz(os) - atz(os)) ./ atz(os)) * 100);
end
