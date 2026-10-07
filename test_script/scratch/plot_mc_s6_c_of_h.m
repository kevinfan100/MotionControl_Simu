% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Section 6 figure: c(h) = 1 - h^2 a'(h) of the two walls (the value of c that makes
%   the slope law exact at height h), h from 22.2 down to 1.10 R (log axis, reversed: the descent runs left to right).
%   Plane = production polynomial (calc_correction_functions.m), cell = cell_cperp_delta.m (plane slid by 1.05 R).
%   No fitted constants on the figure (user 10-07). Derivative by central difference (step 1e-5 R).
function plot_mc_s6_c_of_h()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(here);  addpath(genpath(fullfile(root, 'model')));
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    h = logspace(log10(1.10), log10(22.2), 600)';  dh = 1e-5;
    a_pl = @(x) 1 ./ local_cperp_plane(x);  a_ce = @(x) 1 ./ cell_cperp_delta(x);
    c_pl = 1 - h.^2 .* (a_pl(h + dh) - a_pl(h - dh)) / (2 * dh);
    c_ce = 1 - h.^2 .* (a_ce(h + dh) - a_ce(h - dh)) / (2 * dh);
    cT = [0.85 0 0];  cP = [0 0.25 0.85];  FS = 22;  LFS = 18;  AXLW = 2.2;
    f = figure('Position', [40 40 1400 800], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    ax = axes(f);  hold on; box on;
    hc = plot(ax, h, c_ce, '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', 'cell');    % red first (user 10-07); two walls, equal width
    hp = plot(ax, h, c_pl, '-', 'Color', cP, 'LineWidth', 3.5, 'DisplayName', 'plane');
    set(ax, 'XScale', 'log', 'XTick', [1.1 1.5 2 3 5 10 20], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);
    xlim(ax, [1.1 22.2]);  ylim(ax, [-0.2 0.9]);  grid(ax, 'off');
    xlabel(ax, 'h', 'FontSize', FS + 2, 'FontWeight', 'bold');  ylabel(ax, 'c(h)', 'FontSize', FS + 2, 'FontWeight', 'bold');
    legend(ax, [hp hc], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    fprintf('plane c(h): 22.2 R %+.3f, 10 R %+.3f, 1.10 R %+.3f | cell: %+.3f, %+.3f, %+.3f\n', c_pl(end), interp1(h, c_pl, 10), c_pl(1), ...
            c_ce(end), interp1(h, c_ce, 10), c_ce(1));
    out = fullfile(fd, 'fig_mc_s6_c_of_h.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end

function cp = local_cperp_plane(h)
    cp = zeros(size(h));
    for i = 1:numel(h); [~, cp(i)] = calc_correction_functions(h(i)); end
end
