% STATUS: SCRATCH -- 2026-10-29 (user): concept of the blended wall, sphere (radius 5 um) on the bottom wall, drawn in the style of Meng
%   diss. Fig. 52 (probe-centre lines around the sphere, blue). No normal arrows. Geometry only, no simulation.
%   Left: 3D, probe-centre path at h* = 2.37 um along lines y = const (blend l = 1 um), as a raster would follow them.
%   Right top: cross-section y = 0 -- nearest surface only (sharp corner) vs blend l = 1 and 2 um.
%   Right bottom: along the l = 1 um path, the weights w_sphere, w_bottom and h_sphere, h_bottom, h_eff.
%   Blend: w_i = exp(-h_i/l) / sum_j exp(-h_j/l); h_eff = -l log sum_i exp(-h_i/l); n_eff = sum_i w_i n_i (unit).
%   Output test_results/temp_3d_wsonly/fig_1029_blend_meng.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
rb = 5;  hst = 2.37;  FS = 15;  cB = [0 0.25 0.85];  cO = [0.85 0.4 0];  cGr = [0.5 0.5 0.5];
xg = linspace(-14, 14, 841);  zg = linspace(0.01, 14, 421);  [X, Z] = meshgrid(xg, zg);
f = figure('Color', 'w', 'Position', [30 30 2100 1150]);  tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
% ---- 3D, Meng Fig. 52 style ----
ax = nexttile(tl, 1, [2 1]);  hold on;
[TH, PH] = meshgrid(linspace(0, 2 * pi, 60), linspace(0, pi, 40));
surf(rb * sin(PH) .* cos(TH), rb * sin(PH) .* sin(TH), rb + rb * cos(PH), 'FaceColor', [0.85 0.85 0.88], 'EdgeColor', 'none', 'FaceAlpha', 0.35, 'DisplayName', 'sphere');
[GX, GY] = meshgrid([-13 13]);  surf(GX, GY, 0 * GX, 'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', 'bottom wall');
first = true;
for y0 = -9:1.5:9
    H1 = sqrt(X.^2 + y0^2 + (Z - rb).^2) - rb;  He = -1 * log(exp(-H1 / 1) + exp(-Z / 1));
    c = contourc(xg, zg, He, [hst hst]);  i = 1;
    while i < size(c, 2)
        m = c(2, i);  xy = c(:, i + 1:i + m);  hv = 'off';  if first; hv = 'on';  first = false; end
        plot3(xy(1, :), y0 + 0 * xy(1, :), xy(2, :), 'Color', cB, 'LineWidth', 1.6, 'DisplayName', 'probe centre at h* (l = 1 \mum)', 'HandleVisibility', hv);  i = i + m + 1;
    end
end
axis equal;  view(-30, 20);  xlim([-13 13]);  ylim([-13 13]);  zlim([0 14]);  box on;  light('Position', [-20 -30 40], 'Style', 'infinite');  lighting gouraud;
xlabel('X  [\mum]');  ylabel('Y  [\mum]');  zlabel('Z  [\mum]');  legend('Location', 'northoutside', 'Orientation', 'horizontal');
set(ax, 'FontSize', FS, 'FontWeight', 'bold');  set(ax.Toolbar, 'Visible', 'off');
% ---- cross-section y = 0 ----
H1 = sqrt(X.^2 + (Z - rb).^2) - rb;
a2 = nexttile(tl, 2);  hold on;  box on;
tg = linspace(0, 2 * pi, 300);  fill(rb * sin(tg), rb + rb * cos(tg), [0.88 0.88 0.9], 'EdgeColor', cGr, 'LineWidth', 2.5, 'DisplayName', 'sphere');
fill([-14 14 14 -14], [0 0 -1 -1], [0.85 0.85 0.85], 'EdgeColor', 'none', 'HandleVisibility', 'off');  plot([-14 14], [0 0], 'Color', cGr, 'LineWidth', 2.5, 'DisplayName', 'bottom wall');
local_c(contourc(xg, zg, min(H1, Z), [hst hst]), 'k', '--', 2, 'nearest surface only');
local_c(contourc(xg, zg, -1 * log(exp(-H1 / 1) + exp(-Z / 1)), [hst hst]), cB, '-', 2.5, 'blend l = 1 \mum');
local_c(contourc(xg, zg, -2 * log(exp(-H1 / 2) + exp(-Z / 2)), [hst hst]), cO, '-', 2.5, 'blend l = 2 \mum');
axis equal;  xlim([0 14]);  ylim([-1 13]);  xlabel('X  [\mum]');  ylabel('Z  [\mum]');  legend('Location', 'eastoutside');  title(sprintf('cross-section Y = 0, probe centre at h* = %.2f \\mum', hst));
% ---- along the l = 1 path ----
c = contourc(xg, zg, -1 * log(exp(-H1 / 1) + exp(-Z / 1)), [hst hst]);  P = [];  i = 1;
while i < size(c, 2); m = c(2, i);  P = [P; c(:, i + 1:i + m)'];  i = i + m + 1; end   %#ok<AGROW>
P = P(P(:, 1) >= 0, :);  [~, o] = sort(P(:, 1));  P = P(o, :);
hs1 = sqrt(P(:, 1).^2 + (P(:, 2) - rb).^2) - rb;  hs2 = P(:, 2);  e = exp(-[hs1 hs2] / 1);  w = e ./ sum(e, 2);  he = -log(sum(e, 2));
a3 = nexttile(tl, 4);  hold on;  box on;
yyaxis left;  plot(P(:, 1), w(:, 1), '-', 'Color', cGr, 'LineWidth', 2.2);  plot(P(:, 1), w(:, 2), 'k--', 'LineWidth', 2.2);  ylim([0 1.05]);  ylabel('weight');  set(a3, 'YColor', 'k');
yyaxis right;  plot(P(:, 1), hs1, '-', 'Color', [0.6 0.75 1], 'LineWidth', 1.6);  plot(P(:, 1), hs2, '--', 'Color', [0.6 0.75 1], 'LineWidth', 1.6);
plot(P(:, 1), he, '-', 'Color', cB, 'LineWidth', 2.5);  ylim([0 8]);  ylabel('distance  [\mum]');  set(a3, 'YColor', cB);
xlim([0 13]);  xlabel('X  [\mum]');  title('along the l = 1 \mum path (Y = 0)');
legend({'w_{sphere}', 'w_{bottom}', 'h_{sphere}', 'h_{bottom}', 'h_{eff}'}, 'Location', 'eastoutside');
for a = [a2 a3]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
fn = fullfile(fo, 'fig_1029_blend_meng.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open ' fn]);

function local_c(c, col, ls, lw, nm)
    i = 1;  first = true;
    while i < size(c, 2)
        m = c(2, i);  xy = c(:, i + 1:i + m);  hv = 'off';  if first; hv = 'on';  first = false; end
        plot(xy(1, :), xy(2, :), ls, 'Color', col, 'LineWidth', lw, 'DisplayName', nm, 'HandleVisibility', hv);  i = i + m + 1;
    end
end
