% PURPOSE (2026-09-30, user: "(A) check"): the best-memory arm (q44_mse_opt = 1) shows a far/mid-field offset of
%   about -0.037 with scatter 0.025, while the derivation (min of (v/l)^2 + l R2/2) says lag = 0.71 x noise at the
%   optimum. Split the gap, per window of the T_d = 5 s descent, 10 seeds:
%     measured  : offset = mean over seeds of e_a, scatter = sd over seeds (pooled in the window)
%     formula   : l* = (4 v^2/R2)^(1/3), lag v/l*, noise sqrt(l* R2/2)          (steady state, constant v, y2 only)
%     actual l  : lag v/l42 with the filter's logged l42 (steady state)
%     exact y2  : the y2-only recursion a[k+1] = (1 - l42) a + l42 a_true with the logged l42 (no steady-state assumption)
%     filter    : sqrt(P_a), what the filter claims
%   v = |a'_true(h_true) * Delta p_d| from the log (not from the controller), R2 = 1/w_y2.
%   Runs cached in test_results/temp_5state_noap/q44chk_Td5_opt1_s<seed>.mat.   STATUS: SCRATCH
function check_q44_opt_lag(seeds, kappa)
    if nargin < 1; seeds = 1:10; end
    if nargin < 2; kappa = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    id = fullfile(root, 'test_results', 'temp_5state_noap');
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;  T_d = 5;  T_HOLD0 = 0.5;  T_HOLD1 = 2.0;
    A_COV = 0.05;   % logged K(4,2) acts on y2_fed = a_cov * u; the gain per reading u is a_cov * K(4,2)
    nS = numel(seeds);  run = cell(1, nS);
    for q = 1:nS
        fn = fullfile(id, sprintf('q44chk_Td5_opt%g_s%d.mat', kappa, seeds(q)));
        if exist(fn, 'file'); run{q} = load(fn); continue; end
        cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';
        cfg.trajectory_type = 'ramp_descent';  cfg.h_init = 50;  cfg.h_bottom = 1.10 * R;  cfg.amplitude = 0;
        cfg.t_hold = T_HOLD0;  cfg.t_descend_override = T_d;  cfg.T_sim = T_HOLD0 + T_d + T_HOLD1;  cfg.h_min = 1.10 * R;
        cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0', 'q44_mse_opt', kappa);
        s = run_pure_simulation(cfg, struct('seed', seeds(q), 'collect_diag', true, 'ctrl_const_override', cc));
        r = struct('t', s.tout(:), 'a_true', s.a_true_out(:, 3), 'a_hat', s.diag.a_hat(:, 3), 'P_a', s.diag.P_a(:, 3), ...
                   'w_y2', s.diag.w_y2(:, 3), 'l42', s.diag.K_kf_a_y2(:, 3), 'l41', s.diag.K_kf_a_y1(:, 3), ...
                   'h_true', s.p_true_out(:, 3) / R, 'p_d', s.p_d_out(:, 3));
        save(fn, '-struct', 'r');  run{q} = r;
    end
    t = run{1}.t - T_HOLD0;  n = numel(t);
    E = zeros(n, nS);  LAGX = E;  V = E;  R2 = E;  L42 = E;  L41 = E;  SP = E;
    for q = 1:nS
        r = run{q};
        dh = 1e-4;  h = max(r.h_true, 1.0 + dh);
        cp = arrayfun(@(x) local_cperp(x), h + dh);  cm = arrayfun(@(x) local_cperp(x), h - dh);
        ap = (1 ./ cp - 1 ./ cm) / (2 * dh) * a_nom / R;                    % da/dx [um/pN per um]
        dpd = [diff(r.p_d); 0];
        V(:, q) = abs(ap .* dpd) / a_nom;
        R2(:, q) = (1 ./ r.w_y2) / a_nom^2;  R2(~isfinite(R2(:, q)), q) = NaN;
        L42(:, q) = A_COV * r.l42;  L41(:, q) = r.l41;  SP(:, q) = sqrt(r.P_a) / a_nom;
        E(:, q) = (r.a_true - r.a_hat) / a_nom;
        ax = zeros(n, 1);  ax(1) = r.a_hat(1);                                  % exact y2-only recursion on the truth
        for k = 1:n - 1; lk = A_COV * r.l42(k); ax(k + 1) = (1 - lk) * ax(k) + lk * r.a_true(k); end
        LAGX(:, q) = (r.a_true - ax) / a_nom;
    end
    wins = {[0.5 1.5], [1.5 2.5], [2.5 3.5], [3.5 4.5], [4.5 5.0]};
    fprintf('\nkappa = %g, %d seeds.  lag < 0 = a_hat above a_true (gain units a/a_nom)\n', kappa, nS);
    fprintf('%-9s | %-8s %-8s | %-8s %-8s %-8s | %-8s %-8s | %-8s | %-9s %-9s\n', 'window', 'offset', 'scatter', ...
        'v/l*', 'noise*', 'l*', 'v/l42', 'l42', 'exact y2', 'sqrt(P)', 'lag/noise');
    for w = wins
        m = t >= w{1}(1) & t < w{1}(2);
        off = mean(E(m, :), 'all');  sc = mean(std(E(m, :), 0, 2));
        v = mean(V(m, :), 'all', 'omitnan');  r2 = mean(R2(m, :), 'all', 'omitnan');
        ls = (4 * v^2 / r2)^(1/3);  l42 = mean(L42(m, :), 'all');
        fprintf('%.1f-%.1f   | %+-8.3f %-8.3f | %+-8.3f %-8.3f %-8.1e | %+-8.3f %-8.1e | %+-8.3f | %-9.3f %-9.2f\n', w{1}(1), w{1}(2), ...
            off, sc, -v / ls, sqrt(ls * r2 / 2), ls, -v / l42, l42, mean(LAGX(m, :), 'all'), mean(SP(m, :), 'all'), abs(off) / sc);
    end
    % figure: per window, measured offset / scatter against the y2-only predictions
    tc = cellfun(@mean, wins);  M = zeros(numel(wins), 5);
    for i = 1:numel(wins)
        m = t >= wins{i}(1) & t < wins{i}(2);
        v = mean(V(m, :), 'all', 'omitnan');  r2 = mean(R2(m, :), 'all', 'omitnan');  ls = (4 * v^2 / r2)^(1/3);
        M(i, :) = [abs(mean(E(m, :), 'all')), mean(std(E(m, :), 0, 2)), v / ls, sqrt(ls * r2 / 2), abs(mean(LAGX(m, :), 'all'))];
    end
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');  FS = 18;  LFS = 12;
    f = figure('Position', [40 40 1300 650], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');  ax = axes(f);  hold(ax, 'on');
    h1 = plot(ax, tc, M(:, 1), 'o-', 'Color', [0 0.2 0.9], 'LineWidth', 2.4, 'MarkerFaceColor', [0 0.2 0.9], 'DisplayName', 'measured lag |offset| (10 seeds)');
    h2 = plot(ax, tc, M(:, 2), 's-', 'Color', [0.55 0.75 1.0], 'LineWidth', 2.4, 'MarkerFaceColor', [0.55 0.75 1.0], 'DisplayName', 'measured noise (scatter)');
    h3 = plot(ax, tc, M(:, 3), 'o--', 'Color', [0 0.2 0.9], 'LineWidth', 1.6, 'DisplayName', 'formula lag v/l^* (y_2 only)');
    h4 = plot(ax, tc, M(:, 4), 's--', 'Color', [0.55 0.75 1.0], 'LineWidth', 1.6, 'DisplayName', 'formula noise (l^* R_2/2)^{1/2} (y_2 only)');
    h5 = plot(ax, tc, M(:, 5), 'd:', 'Color', [0.85 0.45 0], 'LineWidth', 2.0, 'DisplayName', 'y_2-only lag with the filter''s own gain');
    xlabel(ax, 'time since the descent start (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax, '[a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');  xlim(ax, [0.5 5]);
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on');
    legend(ax, [h1 h2 h3 h4 h5], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    out = fullfile(fd, 'fig_noap_q44_opt_check.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
    fprintf('mean |l41| in the windows (y1 gain row on a): ');
    for w = wins; m = t >= w{1}(1) & t < w{1}(2); fprintf('%.1e  ', mean(abs(L41(m, :)), 'all')); end
    fprintf('\n');
end

function c = local_cperp(h)
    [~, c] = calc_correction_functions(h);
end
