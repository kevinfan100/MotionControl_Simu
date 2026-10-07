% PURPOSE (2026-10-02, user: "plot L41 and L42 together"): plane, 22.2 -> 1.10 R, T_d = 0.5 | 5 s, 10 seeds,
%   epsilon = Q44/R2 = 0 (grey) vs 1e-5 (blue). First order: the predict leaves a_hat unchanged, so
%     a_hat[k] = a_hat[0] + sum l41 e_y1 + sum l42 e_y2      (exact split of every move of a_hat by channel)
%   Rows: (1) l42 per y2 reading = a_cov K(4,2) | (2) |l41| = |K(4,1)|, a/a_nom per nm of y1 error |
%         (3) cumulative pull of each channel on a_hat (y1 dashed, y2 solid) against a_true - a_true(0) (red).
%   Runs cached in test_results/temp_5state_noap/l4142_plane_<scen>_<arm>_s<seed>.mat.   STATUS: SCRATCH
function plot_5state_noap_l41_l42(TDs, seeds)
    if nargin < 1; TDs = [0.5 5]; end
    if nargin < 2; seeds = 1:10; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  addpath(fullfile(root, 'test_script', 'integration'));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  R = pc.R;  a_nom = pc.Ts / pc.gamma_N;  A_COV = 0.05;
    ARMS = {'q0', '\epsilon = 0', [0.4 0.4 0.4], 0; 'eps', '\epsilon = 10^{-5}', [0 0.2 0.9], 1e-5};
    FS = 16;  LFS = 11;  AXLW = 2.0;
    f = figure('Position', [40 40 1800 1350], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for c = 1:2
        T_d = TDs(c);  stag = strrep(sprintf('ramp%g', T_d), '.', 'p');
        ax = gobjects(1, 3);  for q = 1:3; ax(q) = nexttile((q - 1) * 2 + c); hold(ax(q), 'on'); box(ax(q), 'on'); end
        h0 = plot(ax(1), NaN, NaN, 'w.', 'DisplayName', sprintf('plane, 22.2 -> 1.10 R in T_d = %g s, %d seeds', T_d, numel(seeds)));
        hl1 = h0;  hl3 = gobjects(0);
        for m = 1:2
            L2 = [];  L1 = [];  C1 = [];  C2 = [];  AT = [];
            for q = 1:numel(seeds)
                fn = fullfile(id, sprintf('l4142_plane_%s_%s_s%d.mat', stag, ARMS{m, 1}, seeds(q)));
                if exist(fn, 'file'); r = load(fn);
                else
                    cfg = canonical_scenario(0.05, 1.10, 'deep');  cfg.eq17_variant = '5state_noap';  cfg.amplitude = 0;
                    cfg.trajectory_type = 'ramp_descent';  cfg.h_init = 50;  cfg.h_bottom = 1.10 * R;  cfg.h_min = 1.10 * R;
                    cfg.t_hold = 0.5;  cfg.t_descend_override = T_d;  cfg.T_sim = 0.5 + T_d + 2.0;
                    cc = struct('lf_mode', 'const', 'lambda_f', 1, 'gain_order', 1, 'seed_mode', 'true_h0', 'q44_r2_frac', ARMS{m, 4});
                    s = run_pure_simulation(cfg, struct('seed', seeds(q), 'collect_diag', true, 'ctrl_const_override', cc));
                    r = struct('t', s.tout(:), 'a_true', s.a_true_out(:, 3), 'a_hat', s.diag.a_hat(:, 3), ...
                               'K41', s.diag.K_kf_a_y1(:, 3), 'K42', s.diag.K_kf_a_y2(:, 3), ...
                               'e1', s.diag.innovation_y1(:, 3), 'e2', s.diag.innovation_y2(:, 3));
                    save(fn, '-struct', 'r');
                end
                L2(:, q) = A_COV * r.K42;  L1(:, q) = abs(r.K41) / a_nom * 1e-3;            % per nm of y1 error
                C1(:, q) = cumsum(r.K41 .* r.e1) / a_nom;  C2(:, q) = cumsum(r.K42 .* r.e2) / a_nom;
                AT(:, q) = (r.a_true - r.a_true(1)) / a_nom;
                if q == 1   % exactness check of the split
                    chk = max(abs((r.a_hat - r.a_hat(1)) / a_nom - (C1(:, q) + C2(:, q) - C1(1, q) - C2(1, q))));   % same-step: da[k] = l41 e1[k] + l42 e2[k]
                end
            end
            t = r.t - 0.5;
            fprintf('T_d %-4g %-4s | split check max|a_hat - a_hat0 - pulls| = %.2e (must be ~1e-14)\n', T_d, ARMS{m, 1}, chk);
            for w = {[-0.4 0], [0 T_d], [T_d T_d + 0.5]}
                k = t >= w{1}(1) & t < w{1}(2);  i0 = find(k, 1);  i1 = find(k, 1, 'last');
                fprintf('   t %5.2f-%5.2f | l42 %.2e | |l41| %.2e /nm | pull y1 %+.3f  y2 %+.3f | a_true change %+.3f\n', w{1}(1), w{1}(2), ...
                    mean(L2(k, :), 'all'), mean(L1(k, :), 'all'), mean(C1(i1, :) - C1(i0, :)), mean(C2(i1, :) - C2(i0, :)), mean(AT(i1, :) - AT(i0, :)));
            end
            hl1(end + 1) = plot(ax(1), t, mean(L2, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.8, 'DisplayName', ['l_{42}, ' ARMS{m, 2}]); %#ok<AGROW>
            plot(ax(2), t, mean(L1, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.8);
            hl3(end + 1) = plot(ax(3), t, mean(C2, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.8, 'DisplayName', ['pull by y_2, ' ARMS{m, 2}]); %#ok<AGROW>
            hl3(end + 1) = plot(ax(3), t, mean(C1, 2), '--', 'Color', ARMS{m, 3}, 'LineWidth', 1.8, 'DisplayName', ['pull by y_1, ' ARMS{m, 2}]); %#ok<AGROW>
            if m == 1; hl3(end + 1) = plot(ax(3), t, mean(AT, 2), '-', 'Color', [0.8 0 0], 'LineWidth', 2.2, 'DisplayName', 'a_{true} - a_{true}(0)'); end %#ok<AGROW>
        end
        set(ax(1), 'YScale', 'log');  set(ax(2), 'YScale', 'log');  ylim(ax(1), [3e-6 1e-2]);  ylim(ax(2), [1e-6 3e-3]);  ylim(ax(3), [-1 0.3]);
        ylabel(ax(1), 'l_{42} (share of one y_2 reading)', 'FontSize', FS, 'FontWeight', 'bold');
        ylabel(ax(2), '|l_{41}|  [a/a_{nom} per nm]', 'FontSize', FS, 'FontWeight', 'bold');
        ylabel(ax(3), 'cumulative pull on a_{hat}', 'FontSize', FS, 'FontWeight', 'bold');
        xlabel(ax(3), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for q = 1:3
            xlim(ax(q), [t(1) t(end)]);  yl = ylim(ax(q));
            for x = [0 T_d]; plot(ax(q), [x x], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off'); end
            set(ax(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid(ax(q), 'off');
        end
        yline(ax(3), 0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
        legend(ax(1), hl1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        legend(ax(3), hl3, 'Location', 'southwest', 'NumColumns', 1, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    end
    out = fullfile(fd, 'fig_noap_l41_l42_plane.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
