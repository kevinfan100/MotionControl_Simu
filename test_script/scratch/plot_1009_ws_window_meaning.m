% STATUS: SCRATCH -- 2026-10-12, explanation figure (conversation, not tex): what a 3-step window sees vs the whole descent.
%   Cell descent, the estimate at h_d = 3 R: A = (c, w_s) = (0.024, -0.699). B = A + 0.1 v6, C = A + 0.1 v5 (v5, v6 of the 3-step O at
%   3 R: v5 = (0.879, -0.476), v6 = (0.476, 0.879) on (c, w_s)). For each pair the law gives a-bar along the path,
%   a(h) - a(3) = (1 - c) (1/(3 - w_s) - 1/(h - w_s)); plotted: B - A and C - A, the difference the a-bar readings would show
%   (all three pairs pinned to the same a-bar at 3 R). Left: the 3-step window around 3 R; right: the whole descent (h_d = 22.2 - 21.1 t).
function plot_1009_ws_window_meaning()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    cA = 0.024;  wA = -0.699;  v6 = [0.476 0.879];  v5 = [0.879 -0.476];
    P = {[cA wA] + 0.1 * v6, [cA wA] + 0.1 * v5};
    acc = @(cw, h) (1 - cw(1)) * (1 ./ (3 - cw(2)) - 1 ./ (h - cw(2)));
    hOf = @(t) 22.2 - 21.1 * t;  t3 = (22.2 - 3) / 21.1;  Ts = 1 / 1600;
    cE = [0 0.25 0.85];  cW = [0 0.6 0.25];  FS = 20;  NM = {'0.1 v_6', '0.1 v_5'};  CL = {cW, cE};
    f = figure('Position', [40 40 1800 700], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax1 = nexttile(tl);  hold on; box on;  tw = t3 + (-1:0.02:1) * Ts;                 % the 3-step window: k0 - 1 .. k0 + 1 around 3 R
    ax2 = nexttile(tl);  hold on; box on;  tt = linspace(0, 1, 2001);
    for q = 1:2
        d1 = acc(P{q}, hOf(tw)) - acc([cA wA], hOf(tw));  d2 = acc(P{q}, hOf(tt)) - acc([cA wA], hOf(tt));
        plot(ax1, (tw - t3) * 1e3, d1, '-', 'Color', CL{q}, 'LineWidth', 3, 'DisplayName', NM{q});
        plot(ax2, tt, d2, '-', 'Color', CL{q}, 'LineWidth', 3, 'DisplayName', NM{q});
    end
    yline(ax1, 0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');  yline(ax2, 0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
    xline(ax2, t3, '--', 'Color', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    text(ax2, t3 + 0.01, 0.036, 'h_d = 3 R', 'FontSize', FS - 2, 'FontWeight', 'bold');
    xlabel(ax1, 'time from h_d = 3 R  [ms]  (3-step window)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax2, 'Time (sec)  (whole descent)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax1, [char(916) char(257) '  read by y_2'], 'FontSize', FS, 'FontWeight', 'bold');
    ylim(ax1, [-2e-4 2e-4]);  ylim(ax2, [-0.04 0.04]);  xlim(ax2, [0 1]);
    for a = [ax1 ax2]
        set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.8);  grid(a, 'off');
        legend(a, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 2, 'FontWeight', 'bold');
    end
    out = fullfile(root, 'test_results', 'temp_1009_ws', 'fig_1009_window_meaning_cell.png');
    exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
