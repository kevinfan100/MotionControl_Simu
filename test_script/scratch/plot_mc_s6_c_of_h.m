% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Section 6 figure (Fig. 6.2), h from 1.10 to 22.2 R (log axis).
%   10-13 (user): top = 1 - c(h) = h^2 a'(h) of the two walls overlaid; bottom = ratio of the two, smaller divided by larger,
%   against height. c(h) = 1 - h^2 a'(h) is the value of c that makes the slope law exact at height h.
%   Plane = production polynomial (calc_correction_functions.m), cell = cell_cperp_delta.m (plane slid by 1.05 R).
%   No fitted constants on the figure (user 10-07). Derivative by central difference (step 1e-5 R).
function plot_mc_s6_c_of_h()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(here);  addpath(genpath(fullfile(root, 'model')));
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    h = logspace(log10(1.10), log10(22.2), 600)';  dh = 1e-5;
    a_pl = @(x) 1 ./ local_cperp_plane(x);  a_ce = @(x) 1 ./ cell_cperp_delta(x);
    s_pl = h.^2 .* (a_pl(h + dh) - a_pl(h - dh)) / (2 * dh);          % 1 - c(h), plane
    s_ce = h.^2 .* (a_ce(h + dh) - a_ce(h - dh)) / (2 * dh);          % 1 - c(h), cell
    rt = min(s_pl, s_ce) ./ max(s_pl, s_ce);
    cT = [0.85 0 0];  cP = [0 0.25 0.85];  cG = [0.2 0.2 0.2];  FS = 22;  LFS = 18;  AXLW = 2.2;  XT = [1.1 1.5 2 3 5 10 20];
    f = figure('Position', [40 40 1400 1100], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl);  hold(ax, 'on');  box(ax, 'on');
    hc = plot(ax, h, s_ce, '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', 'cell');    % red first (user 10-07); two walls, equal width
    hp = plot(ax, h, s_pl, '-', 'Color', cP, 'LineWidth', 3.5, 'DisplayName', 'plane');
    set(ax, 'XScale', 'log', 'XTick', XT, 'XTickLabel', [], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);
    xlim(ax, [1.1 22.2]);  ylim(ax, [0 1.2]);  grid(ax, 'off');
    ylabel(ax, '1 - c(h)', 'FontSize', FS + 2, 'FontWeight', 'bold');
    legend(ax, [hp hc], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    ax2 = nexttile(tl);  hold(ax2, 'on');  box(ax2, 'on');
    plot(ax2, h, rt, '-', 'Color', cG, 'LineWidth', 3.5);
    set(ax2, 'XScale', 'log', 'XTick', XT, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);
    xlim(ax2, [1.1 22.2]);  ylim(ax2, [0 1]);  grid(ax2, 'off');
    xlabel(ax2, 'h', 'FontSize', FS + 2, 'FontWeight', 'bold');  ylabel(ax2, 'smaller / larger', 'FontSize', FS + 2, 'FontWeight', 'bold');
    fprintf('1 - c(h)  plane: 22.2 R %.3f, 10 R %.3f, 2 R %.3f, 1.10 R %.3f | cell: %.3f, %.3f, %.3f, %.3f\n', s_pl(end), ...
            interp1(h, s_pl, 10), interp1(h, s_pl, 2), s_pl(1), s_ce(end), interp1(h, s_ce, 10), interp1(h, s_ce, 2), s_ce(1));
    fprintf('ratio smaller/larger: 22.2 R %.3f, 10 R %.3f, 5 R %.3f, 2 R %.3f, 1.10 R %.3f; plane larger at every h: %d\n', rt(end), ...
            interp1(h, rt, 10), interp1(h, rt, 5), interp1(h, rt, 2), rt(1), all(s_pl >= s_ce));
    out = fullfile(fd, 'fig_mc_s6_c_of_h.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end

function cp = local_cperp_plane(h)
    cp = zeros(size(h));
    for i = 1:numel(h); [~, cp(i)] = calc_correction_functions(h(i)); end
end
