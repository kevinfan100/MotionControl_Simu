% PURPOSE (2026-09-29, user: "show me the c_eff figure"): c_eff(h) := 1 - h^2 a'_true(h) for the plane and the cell plant
%   (cell = plane shifted 1.05 R below the visible surface), h in [1.1, 22.2] R, log axis reversed (descent runs left to right);
%   horizontal dashed = band fit constant (LS on a', uniform in h: plane 0.053, cell 0.640); the walk Delta c = max - min over the
%   band in the legend (plane 0.20, cell 0.77). Closed forms only. Output reference/eq17_analysis/derivation/figures/ceff_walls.png
%   EXPIRES: with the c-only line
function plot_ceff_walls()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    Dp = @(u) -9/8 + (3/2)*u.^2 - 2.28*u.^3 + u.^4 + 0.385*u.^10 - 0.48*u.^11;
    ap_pl = @(h) -Dp(1 ./ h) ./ h.^2;  ap_ce = @(h) -Dp(1 ./ (h + 1.05)) ./ (h + 1.05).^2;
    ceff_pl = @(h) 1 - h.^2 .* ap_pl(h);  ceff_ce = @(h) 1 - h.^2 .* ap_ce(h);
    h = logspace(log10(1.1), log10(22.2), 2000)';  cp = ceff_pl(h);  cc = ceff_ce(h);
    CFIT = [0.053 0.640];  COL_PL = [0.85 0.45 0]; COL_CE = [0.8 0 0]; FS = 15; LFS = 11; AXLW = 2.0; CG = [0.5 0.5 0.5];
    f = figure('Position', [60 60 900 560], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off'); hold on; box on;
    h1 = plot(h, cc, '-', 'Color', COL_CE, 'LineWidth', 2.4, 'DisplayName', sprintf('c_{eff}(h) = 1 - h^2 a''_{true}(h),  CELL   (walk %+.2f -> %+.2f, \\Delta c = %.2f)', min(cc), max(cc), max(cc) - min(cc)));
    h2 = plot(h, cp, '-', 'Color', COL_PL, 'LineWidth', 2.4, 'DisplayName', sprintf('c_{eff}(h),  PLANE   (walk %+.2f -> %+.2f, \\Delta c = %.2f)', min(cp), max(cp), max(cp) - min(cp)));
    h3 = yline(CFIT(2), '--', 'Color', COL_CE, 'LineWidth', 1.6, 'DisplayName', sprintf('band fit constant, cell  %.3f', CFIT(2)));
    h4 = yline(CFIT(1), '--', 'Color', COL_PL, 'LineWidth', 1.6, 'DisplayName', sprintf('band fit constant, plane  %.3f', CFIT(1)));
    xline(3, ':', 'Color', CG, 'LineWidth', 1.2, 'HandleVisibility', 'off');
    set(gca, 'XScale', 'log', 'XDir', 'reverse');  xlim([1.0 25]);  ylim([-0.2 0.9]);
    set(gca, 'XTick', [1.1 1.5 2 3 5 10 20], 'XTickLabel', {'1.1', '1.5', '2', '3', '5', '10', '20'});
    legend([h1 h2 h3 h4], 'Location', 'northoutside', 'NumColumns', 1, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel('c', 'FontSize', FS, 'FontWeight', 'bold');  xlabel('h  (R)   (descent runs left to right)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    fn = fullfile(od, 'ceff_walls.png');  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
