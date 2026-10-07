% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Fig. 6.1: motion gain a-bar(h) of the two walls (plane, cell), h from 22.2 down to
%   1.10 R (log axis reversed, descent left to right); same axes as Fig. 6.2 (c(h), plot_mc_s6_c_of_h.m). Plant laws only.
function plot_mc_s6_a_of_h()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(here);  addpath(genpath(fullfile(root, 'model')));
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    h = logspace(log10(1.10), log10(22.2), 600)';
    a_pl = zeros(size(h));  for i = 1:numel(h); [~, cp] = calc_correction_functions(h(i)); a_pl(i) = 1 / cp; end
    a_ce = 1 ./ cell_cperp_delta(h);
    cT = [0.85 0 0];  cP = [0 0.25 0.85];  FS = 22;  LFS = 18;  AXLW = 2.2;  AB = char(257);
    f = figure('Position', [40 40 1400 800], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    ax = axes(f);  hold on; box on;
    hc = plot(ax, h, a_ce, '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', 'cell');    % red first; two walls, equal width
    hp = plot(ax, h, a_pl, '-', 'Color', cP, 'LineWidth', 3.5, 'DisplayName', 'plane');
    set(ax, 'XScale', 'log', 'XTick', [1.1 1.5 2 3 5 10 20], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);
    xlim(ax, [1.1 22.2]);  ylim(ax, [0 1]);  grid(ax, 'off');
    xlabel(ax, 'h', 'FontSize', FS + 2, 'FontWeight', 'bold');  ylabel(ax, [AB '(h)'], 'FontSize', FS + 2, 'FontWeight', 'bold');
    legend(ax, [hp hc], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    fprintf('a(h): plane 22.2 R %.3f, 2 R %.3f, 1.10 R %.3f | cell %.3f, %.3f, %.3f\n', a_pl(end), interp1(h, a_pl, 2), a_pl(1), a_ce(end), interp1(h, a_ce, 2), a_ce(1));
    out = fullfile(fd, 'fig_mc_s6_a_of_h.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
