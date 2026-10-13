% STATUS: SCRATCH -- 2026-10-11, explanation figure (conversation, not tex) for the v5 / v6 rows of fig_1009_ws_svd3_desc_s1.png.
%   One step of the cell descent at h_d = 3 R (c-hat, w_s-hat, v5, v6 from probe_1009_ws_svd3_track.m, test_results/temp_1009_ws/svd3_cell.mat).
%   Left: the (c, w_s) plane, contours of the model slope a' = (1 - c)/(h_d - w_s)^2, arrows v5 and v6 at the estimate (axis equal).
%   Right: a' after moving a distance s along v5 and along v6 from the estimate.
function plot_1009_ws_v56_meaning(h_pick)
    if nargin < 1; h_pick = 3.0; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    r = load(fullfile(root, 'test_results', 'temp_1009_ws', 'svd3_cell.mat'));
    q = find(r.hd <= h_pick, 1);  hd = r.hd(q);  k = r.alias(q);
    S = load(fullfile(root, 'test_results', 'temp_1009_ws', 'desc_ws_cell.mat'));
    i = find(S.t >= r.t(q) - 1e-12, 1);  c0 = S.c_hat(i);  w0 = S.w_hat(i);
    v5 = r.v5(q, 5:6);  v6 = r.v6(q, 5:6);                            % (c, w_s) components
    slope = @(c, w) (1 - c) ./ (hd - w).^2;
    a0 = slope(c0, w0);
    fprintf('cell, h_d %.2f, c-hat %+.3f, w_s-hat %+.3f, slope %.5f | v5 (c, w_s) = (%+.3f, %+.3f), v6 = (%+.3f, %+.3f), k = %.3f\n', hd, c0, w0, a0, v5, v6, k);
    for s = [0.1 0.3]
        a5 = slope(c0 + s * v5(1), w0 + s * v5(2));  a6 = slope(c0 + s * v6(1), w0 + s * v6(2));
        aw = slope(c0, w0 + s);
        fprintf('  move %.1f along v5: c %+.3f w_s %+.3f -> slope %.5f (%+.1f %%) | along v6: c %+.3f w_s %+.3f -> %.5f (%+.2f %%) | w_s alone +%.1f -> %.5f (%+.1f %%)\n', ...
                s, s * v5(1), s * v5(2), a5, 100 * (a5 / a0 - 1), s * v6(1), s * v6(2), a6, 100 * (a6 / a0 - 1), s, aw, 100 * (aw / a0 - 1));
    end
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cW = [0 0.6 0.25];  FS = 20;
    f = figure('Position', [40 40 1700 760], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl);  hold on; box on;
    [C, W] = meshgrid(linspace(c0 - 0.6, c0 + 0.6, 241), linspace(w0 - 0.6, w0 + 0.6, 241));
    contour(ax, C, W, slope(C, W), a0 * (0.6:0.1:1.5), 'Color', [0.6 0.6 0.6], 'LineWidth', 1.2);
    contour(ax, C, W, slope(C, W), [a0 a0], 'Color', 'k', 'LineWidth', 2.5);
    L = 0.45;
    quiver(ax, c0, w0, L * v5(1), L * v5(2), 0, 'Color', cE, 'LineWidth', 3.5, 'MaxHeadSize', 0.6);
    quiver(ax, c0, w0, L * v6(1), L * v6(2), 0, 'Color', cW, 'LineWidth', 3.5, 'MaxHeadSize', 0.6);
    plot(ax, c0, w0, 'o', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
    text(ax, c0 + L * v5(1), w0 + L * v5(2) - 0.06, 'v_5', 'FontSize', FS + 2, 'FontWeight', 'bold', 'Color', cE);
    text(ax, c0 + L * v6(1) + 0.03, w0 + L * v6(2), 'v_6', 'FontSize', FS + 2, 'FontWeight', 'bold', 'Color', cW);
    axis(ax, 'equal');  xlim(ax, c0 + [-0.6 0.6]);  ylim(ax, w0 + [-0.6 0.6]);
    xlabel(ax, 'c', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax, ['w' char(772) '_s  [R]'], 'FontSize', FS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.8);
    ax = nexttile(tl);  hold on; box on;
    s = linspace(-0.3, 0.3, 121);
    plot(ax, s, slope(c0 + s * v5(1), w0 + s * v5(2)) / a0, '-', 'Color', cE, 'LineWidth', 3, 'DisplayName', 'along v_5');
    plot(ax, s, slope(c0 + s * v6(1), w0 + s * v6(2)) / a0, '-', 'Color', cW, 'LineWidth', 3, 'DisplayName', 'along v_6');
    yline(ax, 1, '-', 'Color', 'k', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    xlabel(ax, 'distance moved from the estimate', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax, [char(257) ''' / ' char(257) '''(estimate)'], 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 2, 'FontWeight', 'bold');
    ylim(ax, [0.6 1.5]);  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.8);  grid(ax, 'off');
    out = fullfile(root, 'test_results', 'temp_1009_ws', 'fig_1009_v56_meaning_cell.png');
    exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
