% PURPOSE (2026-09-29, user: "c shows up only through the accumulated walk -- I do not follow"): closed forms only.
%   Both curves start from the same gain at h0 = 22.2 R and integrate their own slope downward:
%     true      a(h) = a(h0) - (1 - c)     (1/h - 1/h0)      c = 0.45
%     estimator a(h) = a(h0) - (1 - c_hat) (1/h - 1/h0)      c_hat = 0.05  (c wrong by 0.40)
%   row 1  the two gain curves vs h (descent 22.2 -> 1.1 R), then a hold at 3 R drawn as the flat continuation
%   row 2  their difference = (c - c_hat) * s^c(h), s^c = 1/h - 1/h0: zero at the top, grows only while walking, frozen in a hold
%   House style. Output reference/eq17_analysis/derivation/figures/c_slope_walk.png   EXPIRES: with the c-only line
function plot_c_slope_walk()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    h0 = 22.2;  c = 0.45;  ch = 0.05;  a0 = 1 - 1.125 / h0;                 % a(h0) shared (plane far field 1 - 9/8/h)
    h = logspace(log10(1.1), log10(h0), 1200)';  s = 1 ./ h - 1 / h0;
    at = a0 - (1 - c) * s;  am = a0 - (1 - ch) * s;  gap = (c - ch) * s;
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; FS = 15; LFS = 11; AXLW = 2.0; CG = [0.5 0.5 0.5];
    f = figure('Position', [60 60 900 780], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax1 = nexttile; hold on; box on;
    h1 = plot(h, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a_{true}(h):  slope (1 - c)/h^2,  c = 0.45');
    h2 = plot(h, am, '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'a_{hat}(h):  slope (1 - c_{hat})/h^2,  c_{hat} = 0.05');
    xline(3, '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    set(gca, 'XScale', 'log', 'XDir', 'reverse');  xlim([1.0 25]);  ylim([0 1]);
    legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    ax2 = nexttile; hold on; box on;
    h3 = plot(h, gap, '-', 'Color', COL_HAT, 'LineWidth', 2.2, 'DisplayName', 'a_{true} - a_{hat} = (c - c_{hat}) (1/h - 1/h_0)  = what y_2 sees of the c error');
    xline(3, '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    for hq = [10 3 1.1]; [~, j] = min(abs(h - hq)); plot(h(j), gap(j), 'o', 'Color', COL_HAT, 'MarkerFaceColor', COL_HAT, 'MarkerSize', 7, 'HandleVisibility', 'off');
        text(h(j), gap(j) + 0.03, sprintf('%.3f', gap(j)), 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center'); end
    set(gca, 'XScale', 'log', 'XDir', 'reverse');  xlim([1.0 25]);  ylim([0 0.45]);
    legend(h3, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');  xlabel('h  (R)   (descent runs left to right; a hold keeps the value where it stops)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    linkaxes([ax1 ax2], 'x');
    fn = fullfile(od, 'c_slope_walk.png');  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
