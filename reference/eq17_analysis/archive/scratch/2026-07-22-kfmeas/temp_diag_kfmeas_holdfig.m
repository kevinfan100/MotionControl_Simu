function temp_diag_kfmeas_holdfig()
% TEMP diagnostic (chat 2026-07-22): first-hold anatomy. init a=a_nom, a'=0.
% P1: a_d tracks true gain via a_xm (gain IS measured in the hold).
% P2: a' -- correct (Option A, frozen ~0) vs wrong (flat Q55, drifts) = the cause.
% P3: a_hat_z -- correct (clean) vs wrong (oscillates) = the symptom. Delete after.

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

    Pp = calc_simulation_params(base); Pp = Pp.Value; Ts = Pp.common.Ts; t1 = base.t_hold;

    cA = base; cA.aprime_state_kappa = 0; cA.aprime_state_Q55_floor = 1e-12;   % Option A (correct)
    cF = base; cF.aprime_state_kappa = 0; cF.aprime_state_Q55_floor = 1e-7;    % flat Q55 (wrong: oscillates)
    cO = base; cO.aprime_source = 'known';
    sA = temp_run_pure_sim_kfmeas(cA, struct('seed', 1, 'collect_diag', true));
    sF = temp_run_pure_sim_kfmeas(cF, struct('seed', 1, 'collect_diag', true));
    sO = run_pure_simulation(cO, struct('seed', 1, 'collect_diag', true));

    N = size(sA.diag.a_hat, 1); t = (0:N - 1)' * Ts; hi = t < t1;
    tg  = sA.a_true_out(:, 3) * 1e3;             % true gain [nm/pN]
    adA = sA.diag.a_hat_d(:) * 1e3;              % a_d posterior [nm/pN]
    ahA = sA.diag.a_hat(:, 3) * 1e3;             % a_hat_z Option A [nm/pN]
    ahF = sF.diag.a_hat(:, 3) * 1e3;             % a_hat_z flat-Q [nm/pN]
    apA = sA.diag.a_prime_used(:, 3) * 1e3;      % a' Option A [1e-3/pN]
    apF = sF.diag.a_prime_used(:, 3) * 1e3;      % a' flat-Q   [1e-3/pN]
    apO = sO.diag.a_prime_used(:, 3) * 1e3;      % a' oracle   [1e-3/pN]

    COL_TRUE = [0.8 0 0]; COL_OK = [0 0.2 0.9]; COL_BAD = [0.5 0.5 0.5];
    FS = 17; LFS = 12; AXLW = 2.0;
    f = figure('Position', [70 70 900 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    % P1: a_d tracks true gain (gain measured by a_xm in the hold)
    nexttile; hold on;
    plot(t(hi), tg(hi),  '-', 'Color', COL_TRUE, 'LineWidth', 2.6, 'DisplayName', 'a_z true');
    plot(t(hi), adA(hi), '-', 'Color', COL_OK,   'LineWidth', 1.8, 'DisplayName', 'a_d (measured by a_{xm})');
    xlim([0 t1]); ylim(mean(tg(hi)) + [-0.6 0.6]);
    ylabel('gain  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    title('(1) gain a_d IS measured in the hold', 'FontSize', FS - 1, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    % P2: a' -- correct frozen vs wrong drifting (the CAUSE)
    nexttile; hold on;
    plot(t(hi), apO(hi), '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a'' oracle (~0)');
    plot(t(hi), apF(hi), '-', 'Color', COL_BAD,  'LineWidth', 1.8, 'DisplayName', 'a'' flat-Q55 (drifts)');
    plot(t(hi), apA(hi), '-', 'Color', COL_OK,   'LineWidth', 1.8, 'DisplayName', 'a'' Option A (frozen)');
    xlim([0 t1]);
    ylabel('a''  (10^{-3}/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    title('(2) CAUSE: flat Q55 drives the unobservable a'' -> drift', 'FontSize', FS - 1, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    % P3: a_hat_z -- clean vs oscillating (the SYMPTOM)
    nexttile; hold on;
    plot(t(hi), tg(hi),  '-', 'Color', COL_TRUE, 'LineWidth', 2.6, 'DisplayName', 'a_z true');
    plot(t(hi), ahF(hi), '-', 'Color', COL_BAD,  'LineWidth', 1.6, 'DisplayName', '\^a_z flat-Q55 (osc)');
    plot(t(hi), ahA(hi), '-', 'Color', COL_OK,   'LineWidth', 1.6, 'DisplayName', '\^a_z Option A (clean)');
    xlim([0 t1]);
    ylabel('\^a_z  (nm/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel('first-hold time  (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    title('(3) SYMPTOM: drifting a'' poisons the gain', 'FontSize', FS - 1, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;

    fn = fullfile(out_dir, 'kfmeas_holdanatomy.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f);
    fprintf('saved %s\n', fn);
    fprintf('hold std(a_hat_z): Option A=%.3f, flat-Q55=%.3f nm/pN | a_d tracks true: mean|a_d-true|=%.3f nm/pN\n', ...
            std(ahA(hi)), std(ahF(hi)), mean(abs(adA(hi) - tg(hi))));
end
