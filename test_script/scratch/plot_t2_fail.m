% STATUS: SCRATCH -- 2026-10-21 (user: "無閘 + u、v 控制 1/10 失敗 我想要看一下他怎麼樣失敗的"). Early window of run_t2_3d_reg runs,
%   columns = (tag, seed) pairs, e.g. a run that survives next to the one that fails, same time window. Rows: theta_y, theta_x (true red,
%   estimates blue / green) | a_T, a_N (true red, estimates) | a'_N (true red, estimate blue; |.| on a log axis, negative estimate marked
%   by black dots) | h [R] | regulation step Delta h_d [R/step]. -> test_results/temp_3d_wsonly/fig_t2_fail_<outtag>.png
%   | run: plot_t2_fail({'ng_uvnl_a', 1; 'ng_uvnl_a', 5}, [0 0.35], 'uvnl')
function plot_t2_fail(RUNS, t_win, outtag)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cGr = [0 0.55 0.2];  FS = 18;  LFS = 14;
    AB = char(257);  HT = char(770);  TH = char(952);  DG = char(176);  PR = char(8242);  DL = char(916);
    nc = size(RUNS, 1);
    f = figure('Position', [40 40 900 * nc 1700], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(5, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        S = load(fullfile(od, ['temp_t2_' RUNS{ic, 1} '.mat']));  sd = cellfun(@(q) q.seed, S.R.runs);
        r = S.R.runs{find(sd == RUNS{ic, 2}, 1)};  ty = S.R.opts.theta_y;
        k = find(r.t >= t_win(1) & r.t <= t_win(2));  k = k(k <= r.kend);  t = r.t(k);
        ax = nexttile(tl, ic);  hold on;  box on;
        h1 = plot(t, ty + 0 * t, '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', [TH '_y,  ' TH '_x']);
        plot(t, 0 * t, '-', 'Color', cT, 'LineWidth', 3, 'HandleVisibility', 'off');
        h2 = plot(t, r.thy(k), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', [TH HT '_y']);
        h3 = plot(t, r.thx(k), '-', 'Color', cGr, 'LineWidth', 1.5, 'DisplayName', [TH HT '_x']);
        ylim([-90 90]);  xlim(t_win);  if ic == 1; ylabel([TH '  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        text(0.97, 0.9, sprintf('seed %d', RUNS{ic, 2}), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);
        ax = nexttile(tl, nc + ic);  hold on;  box on;
        h1 = plot(t, r.aT_true(k), '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', [AB '_T,  ' AB '_N']);
        plot(t, r.aN_true(k), '-', 'Color', cT, 'LineWidth', 3, 'HandleVisibility', 'off');
        h2 = plot(t, r.aT(k), '-', 'Color', cGr, 'LineWidth', 1.5, 'DisplayName', [AB HT '_T']);
        h3 = plot(t, r.aN(k), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', [AB HT '_N']);
        ylim([0 1.3]);  xlim(t_win);  if ic == 1; ylabel(AB, 'FontSize', FS + 2, 'FontWeight', 'bold'); end
        legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);
        ax = nexttile(tl, 2 * nc + ic);  hold on;  box on;
        h1 = plot(t, r.apN_true(k), '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', [AB PR '_N']);
        h2 = plot(t, abs(r.apN(k)), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', ['|' AB HT PR '_N|']);
        neg = r.apN(k) < 0;  h3 = plot(t(neg), abs(r.apN(k(neg))), 'k.', 'MarkerSize', 6, 'DisplayName', [AB HT PR '_N < 0']);
        set(ax, 'YScale', 'log');  ylim([1e-4 10]);  xlim(t_win);
        if ic == 1; ylabel([AB PR '_N'], 'FontSize', FS + 2, 'FontWeight', 'bold'); end
        legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);
        ax = nexttile(tl, 3 * nc + ic);  hold on;  box on;
        plot(t, r.h(k), '-', 'Color', cE, 'LineWidth', 1.8);
        ylim([0 25]);  xlim(t_win);  if ic == 1; ylabel('h  [R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);
        ax = nexttile(tl, 4 * nc + ic);  hold on;  box on;
        plot(t, r.dhd(k), '-', 'Color', cE, 'LineWidth', 1.5);
        ylim([-0.6 0.6]);  xlim(t_win);  if ic == 1; ylabel([DL 'h_d  [R/step]'], 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);
    end
    out = fullfile(od, sprintf('fig_t2_fail_%s.png', outtag));
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
