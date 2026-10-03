% PURPOSE (2026-09-20): the new default test cell plant (cell_cperp_delta = plane slid down 1.05 R) against the Fig 11 data
%   and the old data-fit plant (cell_cperp_fig11), plus the [1/1] band seed refitted to the new plant.
%   Left : a_bar against h = w - w_s (visible-surface coordinate): data, old plant, new plant, [1/1] refit; inset numbers at h = 1.
%   Right: residuals against h: old plant - data, new plant - data (data range), and new - old over the whole band.
%   Console: rms/max of each residual; monotonicity check of both plants on [1, 30]; the refitted (c0, d1) for FIT.cell.
%   Output reference/eq17_analysis/derivation/figures/cell_plant_delta.png.  EXPIRES: with the delta-law line.
function out = plot_cell_plant_delta()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    T = readtable(fullfile(root, 'reference', 'eq17_analysis', 'data', 'meng_11072733_fig11_three_surfaces.csv'));
    m = strcmp(T.surface, 'cell') & T.hd_R <= 4.5;  hd = T.hd_R(m);  yd = T.lambda_norm(m);
    a_old = @(h) 1 ./ cell_cperp_fig11(h);  a_new = @(h) 1 ./ cell_cperp_delta(h);
    % [1/1] band seed for the new plant: least squares of (h + c0)/(h + d1) on 1.1-25 R (same recipe as the old FIT.cell)
    hb = linspace(1.1, 25, 2000)';  yb = a_new(hb);
    cost = @(p) sum(((hb + p(1)) ./ (hb + p(2)) - yb) .^ 2);
    p = fminsearch(cost, [0.056, 1.197], optimset('TolX', 1e-8, 'TolFun', 1e-12, 'MaxFunEvals', 2e4));
    a_11 = @(h) (h + p(1)) ./ (h + p(2));
    r_old = a_old(hd) - yd;  r_new = a_new(hd) - yd;  r_11 = a_11(hb) - yb;
    hm = linspace(1.0, 30, 30000)';
    fprintf('old plant - data : rms %.4f  max %.4f | new plant - data : rms %.4f  max %.4f | [1/1] refit - new plant (band): rms %.4f max %.4f\n', ...
            rms(r_old), max(abs(r_old)), rms(r_new), max(abs(r_new)), rms(r_11), max(abs(r_11)));
    fprintf('monotone on [1, 30]: old plant min diff %+.2e (%s) | new plant min diff %+.2e (%s)\n', ...
            min(diff(a_old(hm))), local_yn(min(diff(a_old(hm))) > 0), min(diff(a_new(hm))), local_yn(min(diff(a_new(hm))) > 0));
    fprintf('new plant: a(1.0) %.4f  a(1.1) %.4f  a(2) %.4f  a(4.5) %.4f  a(22.2) %.4f | far coefficient h(1-a) at 100 R = %.4f\n', ...
            a_new(1), a_new(1.1), a_new(2), a_new(4.5), a_new(22.2), 100 * (1 - a_new(100)));
    fprintf('FIT.cell for the new plant: (c0, d1) = (%.4f, %.4f)  [old plant: (0.056, 1.197)]  a(1.1) = %.4f, kappa = %.4f\n', p, a_11(1.1), p(2) - p(1));
    out = struct('c0', p(1), 'd1', p(2));
    FS = 15; LFS = 11; CD = [0.55 0.75 1]; CO = [0.5 0.5 0.5]; CN = [0 0.2 0.9]; CL = [0 0.55 0.2]; CG = [0.4 0.4 0.4];
    hs = linspace(1.0, 8, 800)';
    f = figure('Position', [40 40 1500 620], 'Color', 'w', 'Visible', 'off');
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile; hold on; box on;
    h1 = plot(hd, yd, 'o', 'Color', CD, 'MarkerFaceColor', CD, 'MarkerSize', 5, 'DisplayName', 'cell data (Meng Fig 11, 1.03-4.5 R)');
    h2 = plot(hs, a_old(hs), '-', 'Color', CO, 'LineWidth', 1.8, 'DisplayName', 'old plant: 12th-order fit in 1/h (cell\_cperp\_fig11)');
    h3 = plot(hs, a_new(hs), '-', 'Color', CN, 'LineWidth', 2.4, 'DisplayName', 'new plant: plane polynomial slid down 1.05 R (cell\_cperp\_delta)');
    h4 = plot(hs, a_11(hs), '--', 'Color', CL, 'LineWidth', 1.8, 'DisplayName', sprintf('[1/1] seed refit on the new plant: (c_0, d_1) = (%.3f, %.3f)', p));
    xline(1, '-', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    text(1.03, 0.03, 'visible surface', 'FontSize', LFS - 1, 'FontWeight', 'bold', 'Color', CG);
    xlim([1 8]); ylim([0 1]);
    xlabel('h = w - w_s   (centre height above the visible surface, R)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2 h3 h4], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on'); grid off;
    nexttile; hold on; box on;
    g1 = plot(hd, r_old, '-', 'Color', CO, 'LineWidth', 1.8, 'DisplayName', 'old plant - data');
    g2 = plot(hd, r_new, '-', 'Color', CN, 'LineWidth', 2.4, 'DisplayName', 'new plant - data');
    g3 = plot(hs, a_new(hs) - a_old(hs), ':', 'Color', [0.85 0.45 0], 'LineWidth', 2.0, 'DisplayName', 'new plant - old plant');
    yline(0, '-', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xlim([1 8]); ylim([-0.05 0.05]);
    xlabel('h = w - w_s   (R)', 'FontSize', FS, 'FontWeight', 'bold'); ylabel('residual  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([g1 g2 g3], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on'); grid off;
    fn = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures', 'cell_plant_delta.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
function s = local_yn(b)
    if b; s = 'monotone'; else; s = 'NOT monotone'; end
end
