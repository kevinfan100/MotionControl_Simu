% PURPOSE (2026-10-04, user: "draw the default I[k], what it means, for the document"): ramp_tri, c-only f1p2, plane init,
%   ratio sqrt(P_fix) = 0.15, sqrt(P_0) = 0.1, 10 seeds.  I[k] = 1/P_55[k] - 1/P^f_55[k] (logged as q55a_stat in the ratio mode;
%   checked: q55a_q[k+1] = P_fix^2 q55a_stat[k]).  Columns plane | cell.  Rows: commanded height h_d | I[k] (log scale, seed mean)
%   (cumulative evidence sum_j I[j] printed to console only).  Notation 1004_q55_ratio_from_kf.tex.   STATUS: SCRATCH
function plot_conly_I_rtri()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    WALLS = {'plane', 'cell'};  PFIX = 0.15^2;
    FS = 18;  AXLW = 2.0;  cE = [0 0.2 0.9];
    f = figure('Position', [40 40 1600 800], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, numel(WALLS), 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(2, numel(WALLS));
    for iw = 1:numel(WALLS)
        S = load(fullfile(id, sprintf('f1p2_%s_PI_sig010_rho15_rtri_f1.mat', WALLS{iw})));
        t = S.t;  I = S.q55a_stat;  mI = mean(I, 2);  cI = cumsum(mI);
        ax(1, iw) = nexttile(tl, iw);  hold on; box on;
        hd = S.wd;  hd(hd < 1) = NaN;
        plot(t, hd, '-', 'Color', [0 0 0], 'LineWidth', 2.0);
        ylim([0 23]);  yticks([1.1 5 10 15 22.2]);
        lg = legend(sprintf('h_d  (%s wall, plane init)', WALLS{iw}), 'Location', 'northoutside', 'FontSize', FS - 4, 'FontWeight', 'bold', 'AutoUpdate', 'off');
        ax(2, iw) = nexttile(tl, numel(WALLS) + iw);  hold on; box on;
        plot(t, mI, '-', 'Color', cE, 'LineWidth', 1.6);
        set(gca, 'YScale', 'log');  ylim([1e-4 3]);
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for r = 1:2
            yl = ylim(ax(r, iw));  plot(ax(r, iw), [1 1], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0);  ylim(ax(r, iw), yl);
            xlim(ax(r, iw), [0 3]);  set(ax(r, iw), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(r, iw), 'off');
        end
        % console: where the evidence comes from
        seg = {t < 0.9, t >= 0.9 & t < 1, t >= 1};  nm = {'descent h_d > 3 (0-0.9 s)', 'descent last 0.1 s', 'triangle (1-3 s)'};
        fprintf('\n%s: total evidence sum I = %.0f units = %.1f memory lengths (1/P_fix = %.0f)\n', WALLS{iw}, cI(end), cI(end) * PFIX, 1 / PFIX);
        for s = 1:3; fprintf('  %-28s %6.1f units (%4.1f%%)\n', nm{s}, sum(mI(seg{s})), 100 * sum(mI(seg{s})) / cI(end)); end
        kt = t >= 1;  [~, i1] = min(abs(t - 1.5));  [~, i2] = min(abs(t - 2.0));
        fprintf('  I at top 1.5 s %.3g, at trough 2.0 s %.3g (ratio %.0fx)\n', mI(i1), mI(i2), mI(i2) / mI(i1));
    end
    ylabel(ax(1, 1), 'h_d', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(2, 1), 'I[k]', 'FontSize', FS, 'FontWeight', 'bold');
    linkaxes(ax(2, :), 'y');
    out = fullfile(fd, 'fig_conly_I_rtri.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('\nfigure: %s\n', out);
end
