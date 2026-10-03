% PURPOSE (2026-09-20): paired comparison of the offset-law estimator (delta_state, arm T of run_delta_two_walls.m) against
%   the [1/1] two-constant version A (arm T of run_11_two_walls.m) on the same plants and seeds (canon deep, 10 seeds).
%   Left column plane, right column cell: row 1 e_a = a_true - a_hat (seed mean, both estimators), row 2 sigma_seed of e_a,
%   row 3 the paired difference e_a(delta) - e_a([1/1]) (mean +- sigma_seed). Console: per segment (descent worst, descent,
%   osc, hold) the mean, sd and paired difference with its SEM -- the pre-registered criterion "delta T <= [1/1] A T".
%   Output reference/eq17_analysis/derivation/figures/delta_vs_11_T.png | EXPIRES: with the delta-law line
function plot_delta_vs_11(suffix)
    if nargin < 1; suffix = ''; end                    % '' = [1/1] T with the tight 0.05 prior; '_wide' = [1/1] T with the 0.52 family prior (fair to delta's 0.5)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd_d = fullfile(root, 'test_results', 'temp_delta_two_walls');  rd_11 = fullfile(root, 'test_results', 'temp_11_two_walls');
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    C_D = [0 0.2 0.9]; C_11 = [0.85 0.45 0]; BAND = [0.45 0.55 0.95]; FS = 18; LFS = 13; AXLW = 2.0;   % house style
    f = figure('Position', [80 80 1600 1050], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    PL = {'plane', 'cell'};
    for ip = 1:2
        D = load(fullfile(rd_d, sprintf('%s_T.mat', PL{ip})));  E = load(fullfile(rd_11, sprintf('%s_A_T%s.mat', PL{ip}, suffix)));
        t = D.t;  seg = D.seg;  ns = min(D.ns, E.ns);
        ed = D.a_true(:, 1:ns) - D.a_hat(:, 1:ns);  e11 = E.a_true(:, 1:ns) - E.a_hat(:, 1:ns);  dd = ed - e11;
        desc = t >= seg(1) & t < seg(2);  osc = t >= seg(2) & t < seg(3);  hld = t >= seg(3);
        fprintf('\n=== %s plant, arm T, %d paired seeds: e_a (delta A, prior 0.5) vs e_a ([1/1] A%s) ===\n', upper(PL{ip}), ns, suffix);
        fprintf('  %-8s | %-22s | %-22s | %-24s\n', 'segment', 'delta: mean sd', '[1/1]: mean sd', 'paired diff: mean +- SEM');
        SG = {'descent', desc; 'osc', osc; 'hold', hld};
        for r = 1:size(SG, 1)
            m = SG{r, 2};  a = mean(ed(m, :), 1);  b = mean(e11(m, :), 1);  c = a - b;
            fprintf('  %-8s | %+8.4f %8.4f     | %+8.4f %8.4f     | %+8.4f +- %.4f\n', SG{r, 1}, mean(a), std(a), mean(b), std(b), mean(c), std(c) / sqrt(ns));
        end
        md = mean(ed, 2);  m11 = mean(e11, 2);  [~, i1] = max(abs(md) .* desc);  [~, i2] = max(abs(m11) .* desc);
        fprintf('  descent worst: delta %+.4f at %.2f s (sd %.4f) | [1/1] %+.4f at %.2f s (sd %.4f)\n', md(i1), t(i1), std(ed(i1, :)), m11(i2), t(i2), std(e11(i2, :)));
        nexttile(ip); hold on; box on;
        h1 = plot(t, md, '-', 'Color', C_D, 'LineWidth', 1.8, 'DisplayName', 'e_a,  offset law (\delta state)');
        h2 = plot(t, m11, '-', 'Color', C_11, 'LineWidth', 1.6, 'DisplayName', sprintf('e_a,  [1/1] (c_0, d_1) states%s', strrep(suffix, '_', ', ')));
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s, arm T', upper(PL{ip})));
        xlim([0 t(end)]); ylim([-0.06 0.06]);
        if ip == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        nexttile(2 + ip); hold on; box on;
        g1 = plot(t, std(ed, 0, 2), '-', 'Color', C_D, 'LineWidth', 1.8, 'DisplayName', '\sigma_{seed}(e_a),  offset law');
        g2 = plot(t, std(e11, 0, 2), '-', 'Color', C_11, 'LineWidth', 1.6, 'DisplayName', '\sigma_{seed}(e_a),  [1/1]');
        for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
        xlim([0 t(end)]); ylim([0 0.06]);
        if ip == 1; ylabel('\sigma_{seed}  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([g1 g2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        nexttile(4 + ip); hold on; box on;
        dm = mean(dd, 2); ds = std(dd, 0, 2);
        fill([t; flipud(t)], [dm - ds; flipud(dm + ds)], BAND, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        k1 = plot(t, dm, '-', 'Color', C_D, 'LineWidth', 1.8, 'DisplayName', 'paired  e_a(\delta) - e_a([1/1])  (mean \pm \sigma_{seed})');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        for s = seg; xline(s, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
        xlim([0 t(end)]); ylim([-0.06 0.06]);
        if ip == 1; ylabel('\Delta e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend(k1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    end
    fn = fullfile(od, sprintf('delta_vs_11_T%s.png', suffix));
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
