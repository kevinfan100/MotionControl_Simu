% STATUS: SCRATCH -- 2026-10-20 (user: key quantities only, no |a - a_d| row). Estimation figure of run_t2_3d_reg, columns = seeds
%   (default 1 | 4). Rows: a_N (true red, estimate blue) | a_T (true red, estimate green) | theta_y, theta_x (true red, estimates blue /
%   green, +- sqrt P band). House style (bold, Unicode, no grid / title). -> test_results/temp_3d_wsonly/fig_t2_est_<tag>.png
%   Row 1 also carries the set point a_d (black dashed). | run: plot_t2_est('blk_both_c020', [1 4], {'', ' (best)'})
function plot_t2_est(tag, seeds, lbl)
    if nargin < 1; tag = 'blk_both_c020'; end
    if nargin < 2; seeds = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    S = load(fullfile(od, ['temp_t2_' tag '.mat']));  ty = S.R.opts.theta_y;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0 0.55 0.2];  cB = [0.75 0.85 1];  FS = 20;  LFS = 15;
    AB = char(257);  HT = char(770);  TH = char(952);  DG = char(176);
    sd = cellfun(@(q) q.seed, S.R.runs);  nc = numel(seeds);
    if nargin < 3; lbl = repmat({''}, 1, nc); end
    f = figure('Position', [40 40 950 * nc 1500], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(3, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        r = S.R.runs{find(sd == seeds(ic), 1)};  k = 1:r.kend;  t = r.t(k);
        ax = nexttile(tl, ic);  hold on;  box on;
        h1 = plot(t, r.aN_true(k), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_N']);
        h2 = plot(t, r.aN(k), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HT '_N']);
        h3 = plot(t, S.R.opts.a_d + 0 * t, '--', 'Color', [0 0 0], 'LineWidth', 2, 'DisplayName', sprintf('%s_d = %.1f', AB, S.R.opts.a_d));
        ylim([0 1.1]);  legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        if ic == 1; ylabel([AB '_N'], 'FontSize', FS + 2, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  xlim([0 r.t(end)]);
        ax = nexttile(tl, nc + ic);  hold on;  box on;
        h1 = plot(t, r.aT_true(k), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_T']);
        h2 = plot(t, r.aT(k), '-', 'Color', cGr, 'LineWidth', 1.8, 'DisplayName', [AB HT '_T']);
        ylim([0 1.1]);  legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        if ic == 1; ylabel([AB '_T'], 'FontSize', FS + 2, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  xlim([0 r.t(end)]);
        ax = nexttile(tl, 2 * nc + ic);  hold on;  box on;
        fill([t; flipud(t)], [r.thy(k) + r.sP(k, 6); flipud(r.thy(k) - r.sP(k, 6))], cB, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        h1 = plot(t, ty + 0 * t, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [TH '_y,  ' TH '_x']);
        plot(t, 0 * t, '-', 'Color', cT, 'LineWidth', 4, 'HandleVisibility', 'off');
        h2 = plot(t, r.thy(k), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [TH HT '_y']);
        h3 = plot(t, r.thx(k), '-', 'Color', cGr, 'LineWidth', 1.8, 'DisplayName', [TH HT '_x']);
        ylim([-30 50]);  legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        if ic == 1; ylabel([TH '  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  xlim([0 r.t(end)]);
    end
    out = fullfile(od, sprintf('fig_t2_est_%s.png', tag));
    exportgraphics(f, out, 'Resolution', 120);  close(f);  fprintf('figure: %s\n', out);
end
