% STATUS: SCRATCH -- 2026-10-28 (user): schematic of the hemisphere test (radius 15 um on a plane): start 15 um above the top, hold,
%   regulation to a_d = 0.1, scan half of the bump -- (1) single line top -> edge vs (2) raster over the half x > 0.
%   Output test_results/temp_3d_wsonly/fig_1028_hemisphere_paths.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  hs = fzero(@(h) 1 / local_cp(h) - 0.1, [1.001 30]);  gap = hs * Rr;
fprintf('a_d 0.1 -> h* = %.3f R = %.2f um (centre of the probe to the surface)\n', hs, gap);
rb = 15;  rp = rb + gap;  gC = [0.45 0.45 0.45];  gF = [0.9 0.9 0.9];
fig = figure('Color', 'w', 'Position', [60 60 1800 820]);  tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
a1 = nexttile;  hold on;  box on;  axis equal;
xg = linspace(-30, 30, 400);  fill([xg fliplr(xg)], [0 * xg, -5 + 0 * xg], gF, 'EdgeColor', 'none');
ph = linspace(0, pi, 200);  fill(rb * cos(ph), rb * sin(ph), gF, 'EdgeColor', gC, 'LineWidth', 3);  plot(xg, 0 * xg, 'Color', gC, 'LineWidth', 3);
z0 = rb + 15;  plot(0, z0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 9);  text(1.5, z0 + 1.5, 'start, hold', 'FontSize', 12, 'FontWeight', 'bold');
plot([0 0], [z0 rp], 'r-', 'LineWidth', 2.5);  text(1.2, (z0 + rp) / 2, 'regulation, a_d = 0.1', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
ps = linspace(pi / 2, 0, 100);  plot(rp * cos(ps), rp * sin(ps), 'b-', 'LineWidth', 3);
text(rp * cos(pi / 4) + 1, rp * sin(pi / 4) + 1.5, 'scan', 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');
xlim([-25 30]);  ylim([-3 33]);  xlabel(sprintf('x  [%sm]', char(181)));  ylabel(sprintf('z  [%sm]', char(181)));
title('side view, y = 0');  set(a1, 'FontSize', 13, 'FontWeight', 'bold');
th = linspace(0, 2 * pi, 300);
a2 = nexttile;  hold on;  box on;  axis equal;  fill(rb * cos(th), rb * sin(th), gF, 'EdgeColor', gC, 'LineWidth', 2);
plot([0 rp], [0 0], 'b-', 'LineWidth', 3);  plot(0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 9);  xlim([-20 20]);  ylim([-20 20]);
xlabel(sprintf('x  [%sm]', char(181)));  ylabel(sprintf('y  [%sm]', char(181)));  title('(1) single line: top \rightarrow edge');  set(a2, 'FontSize', 13, 'FontWeight', 'bold');
a3 = nexttile;  hold on;  box on;  axis equal;  fill(rb * cos(th), rb * sin(th), gF, 'EdgeColor', gC, 'LineWidth', 2);
ys = linspace(-rb, rb, 7);  xr = [];  yr = [];
for i = 1:numel(ys)
    xe = sqrt(max(rp^2 - ys(i)^2, 0));  if mod(i, 2); xr = [xr 0 xe]; else; xr = [xr xe 0]; end;  yr = [yr ys(i) ys(i)];
end
plot(xr, yr, 'b-', 'LineWidth', 2);  plot(0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 9);  xlim([-20 20]);  ylim([-20 20]);
xlabel(sprintf('x  [%sm]', char(181)));  ylabel(sprintf('y  [%sm]', char(181)));  title('(2) raster over the half x > 0');  set(a3, 'FontSize', 13, 'FontWeight', 'bold');
exportgraphics(fig, fullfile(fo, 'fig_1028_hemisphere_paths.png'), 'Resolution', 150);  close(fig);
system(['open ' fullfile(fo, 'fig_1028_hemisphere_paths.png')]);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
