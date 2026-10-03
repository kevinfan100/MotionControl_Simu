% PURPOSE (2026-09-28, user: "I still do not follow the (c, p) constant-velocity block"): closed forms only, no data.
%   row 1  the two signatures in y2 along a constant-velocity descent 22.2 -> 1.1 R (p = 2, c = 0.05):
%          s^c(h) = 1/h - 1/h0 (c wrong by 1), s^p(h) = (1-c)[(1+ln h)/h - (1+ln h0)/h0] (p wrong by 1)
%   row 2  what y2 would read (beyond e_a) for two different error pairs that agree at the wall:
%          A: e_c = 0.10, e_p = 0;   B: e_c = 0, e_p = 0.111  (both give e_c + 0.9 e_p = 0.10)
%   House style. Output reference/eq17_analysis/derivation/figures/cp_alias_walk.png   EXPIRES: with the (c, p) line
function plot_cp_alias_walk()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    h0 = 22.2;  c = 0.05;  h = logspace(log10(1.10), log10(h0), 1000)';
    sc = 1 ./ h - 1 / h0;  sp = (1 - c) * ((1 + log(h)) ./ h - (1 + log(h0)) / h0);
    eA = [0.10 0];  eB = [0 0.111];
    yA = sc * eA(1) + sp * eA(2);  yB = sc * eB(1) + sp * eB(2);
    COL_C = [0 0.2 0.9]; COL_P = [0.8 0 0]; COL_A = [0 0.2 0.9]; COL_B = [0.85 0.4 0]; FS = 15; LFS = 11; AXLW = 2.0; CG = [0.5 0.5 0.5];
    f = figure('Position', [60 60 900 760], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax1 = nexttile; hold on; box on;
    h1 = plot(h, sc, '-', 'Color', COL_C, 'LineWidth', 2.2, 'DisplayName', 's^c(h):  y_2 reading when c is wrong by 1');
    h2 = plot(h, sp, '-', 'Color', COL_P, 'LineWidth', 2.2, 'DisplayName', 's^p(h):  y_2 reading when p is wrong by 1');
    xline(3, '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    set(gca, 'XScale', 'log', 'XDir', 'reverse');  xlim([1.0 25]);  ylim([0 1]);
    legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel('s', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    ax2 = nexttile; hold on; box on;
    h3 = plot(h, yA, '-', 'Color', COL_A, 'LineWidth', 2.2, 'DisplayName', 'A:  e_c = 0.10,  e_p = 0');
    h4 = plot(h, yB, '--', 'Color', COL_B, 'LineWidth', 2.2, 'DisplayName', 'B:  e_c = 0,  e_p = 0.111');
    xline(3, '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    set(gca, 'XScale', 'log', 'XDir', 'reverse');  xlim([1.0 25]);  ylim([0 0.1]);
    legend([h3 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel('y_2 - a_{hat}  (beyond e_a)', 'FontSize', FS, 'FontWeight', 'bold');  xlabel('h_d  (R)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    linkaxes([ax1 ax2], 'x');
    fn = fullfile(od, 'cp_alias_walk.png');  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
    for hq = [10 5 3 2 1.5 1.1]
        [~, j] = min(abs(h - hq));
        fprintf('h %5.1f : s^c %.3f  s^p %.3f  r %.2f | A %.4f  B %.4f  diff %+.4f\n', h(j), sc(j), sp(j), sp(j)/sc(j), yA(j), yB(j), yA(j)-yB(j));
    end
end
