% STATUS: SCRATCH -- 2026-10-28 (user): simple 3D sketch of the current scan -- hemisphere (radius 15 um), the meridian path at gap h*
%   (idealized: true angle, step = (e - z) projected on the tangent plane, e = (1, 1, 0)/sqrt 2), the local u, v, w axes at the top and the
%   measurement coordinate x, y, z (axes box in um, triad at the origin).
%   Output test_results/temp_3d_wsonly/fig_1028_scan_path_3d.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
rb = 15;  Rs = rb + 2.37;  ds = 0.02;  e = [1; 1; 0] / sqrt(2);  FS = 16;  hatc = char(770);  L = 6;
p = [0; 0; Rs];  Pk = p';
while p(3) > 0
    n = p / norm(p);  d = (eye(3) - n * n') * (e - [0; 0; 1]);  p = p + ds * d / norm(d);  p = Rs * p / norm(p);  Pk(end + 1, :) = p';   %#ok<AGROW>
end
f = figure('Color', 'w', 'Position', [60 60 1100 950]);  ax = axes(f);  hold on;
[TH, PH] = meshgrid(linspace(0, 2 * pi, 80), linspace(0, pi / 2, 40));
surf(rb * sin(PH) .* cos(TH), rb * sin(PH) .* sin(TH), rb * cos(PH), 'FaceColor', [0.80 0.82 0.86], 'EdgeColor', 'none', 'FaceAlpha', 0.55);  material dull;
plot3(Pk(:, 1), Pk(:, 2), Pk(:, 3), 'Color', [0 0.25 0.85], 'LineWidth', 3.5);
o = [0; 0; Rs];  ax3 = eye(3);  cl = {[0.85 0 0], [0 0.55 0.2], [0 0 0]};  nm = {['u' hatc], ['v' hatc], ['w' hatc]};
for i = 1:3
    quiver3(o(1), o(2), o(3), L * ax3(1, i), L * ax3(2, i), L * ax3(3, i), 0, 'Color', cl{i}, 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
    text(o(1) + 1.2 * L * ax3(1, i), o(2) + 1.2 * L * ax3(2, i), o(3) + 1.2 * L * ax3(3, i), nm{i}, 'FontSize', FS + 4, 'FontWeight', 'bold', 'Color', cl{i});
end
plot3(o(1), o(2), o(3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
Lm = 6;  nmm = {'x', 'y', 'z'};  c0 = [-6; 18; 0];               % measurement coordinate triad, drawn at a corner outside the hemisphere
for i = 1:3
    quiver3(c0(1), c0(2), c0(3), Lm * ax3(1, i), Lm * ax3(2, i), Lm * ax3(3, i), 0, 'Color', [0.3 0.3 0.3], 'LineWidth', 2, 'MaxHeadSize', 0.4);
    text(c0(1) + 1.25 * Lm * ax3(1, i), c0(2) + 1.25 * Lm * ax3(2, i), c0(3) + 1.25 * Lm * ax3(3, i), nmm{i}, 'FontSize', FS + 2, 'FontWeight', 'bold', 'Color', [0.3 0.3 0.3]);
end
text(c0(1) - 2, c0(2) + 1, c0(3) + 3.5, 'measurement', 'FontSize', FS - 1, 'FontWeight', 'bold', 'Color', [0.3 0.3 0.3]);
text(o(1), o(2) - 12, o(3) + 2, 'local (top)', 'FontSize', FS - 1, 'FontWeight', 'bold');
plot3(Pk(end, 1), Pk(end, 2), Pk(end, 3), 'o', 'Color', [0 0.25 0.85], 'MarkerFaceColor', [0 0.25 0.85], 'MarkerSize', 7);
text(Pk(end, 1) + 1, Pk(end, 2) + 1, Pk(end, 3) + 1.5, 'end (equator)', 'FontSize', FS, 'FontWeight', 'bold', 'Color', [0 0.25 0.85]);
axis equal;  box on;  grid off;  view(125, 22);  xlim([-8 20]);  ylim([-8 26]);  zlim([0 26]);
xlabel('x  [\mum]');  ylabel('y  [\mum]');  zlabel('z  [\mum]');  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(ax.Toolbar, 'Visible', 'off');
light('Position', [-20 30 40], 'Style', 'infinite');  lighting gouraud;
exportgraphics(f, fullfile(fo, 'fig_1028_scan_path_3d.png'), 'Resolution', 150);  close(f);  system(['open ' fullfile(fo, 'fig_1028_scan_path_3d.png')]);
