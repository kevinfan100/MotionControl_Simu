% STATUS: SCRATCH -- 2026-10-29 (user): concept of the blended wall for the bead (radius 5 um) on the glass, cross-section y = 0.
%   (a) geometry: bead, glass, the probe-centre path at distance h* = 2.37 um -- nearest surface only (sharp crease) vs blended with
%       l = 1 and 2 um (rounded); a few effective normals n_eff on the l = 1 path.
%   (b) along the l = 1 path (vs x): the two distances h_1 (bead), h_2 (glass) and h_eff; (c) the weights w_1, w_2 and the angle of n_eff.
%   Blend: w_i = exp(-h_i/l) / sum_j exp(-h_j/l), h_eff = -l log sum_i exp(-h_i/l), n_eff = sum_i w_i n_i (unit).
%   Output test_results/temp_3d_wsonly/fig_1029_blend_concept.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
rb = 5;  hst = 2.37;  FS = 15;  cB = [0 0.25 0.85];  cO = [0.85 0.4 0];  cGr = [0.5 0.5 0.5];  cK = [0 0 0];
[X, Z] = meshgrid(linspace(-14, 14, 1401), linspace(0.01, 14, 701));
h1 = sqrt(X.^2 + (Z - rb).^2) - rb;  h2 = Z;
heff = @(l) -l * log(exp(-h1 / l) + exp(-h2 / l));
f = figure('Color', 'w', 'Position', [30 30 2100 1150]);  tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl, 1, [2 1]);  hold on;  box on;
tg = linspace(0, 2 * pi, 300);  fill(rb * sin(tg), rb + rb * cos(tg), [0.88 0.88 0.9], 'EdgeColor', cGr, 'LineWidth', 2.5, 'DisplayName', 'bead (radius 5 \mum)');
fill([-14 14 14 -14], [0 0 -1 -1], [0.85 0.85 0.85], 'EdgeColor', 'none', 'HandleVisibility', 'off');  plot([-14 14], [0 0], 'Color', cGr, 'LineWidth', 2.5, 'DisplayName', 'glass');
c0 = contourc(X(1, :), Z(:, 1), min(h1, h2), [hst hst]);  local_cplot(c0, cK, '--', 2, sprintf('path at h* = %.2f \\mum, nearest surface only (crease)', hst));
c1 = contourc(X(1, :), Z(:, 1), heff(1), [hst hst]);  local_cplot(c1, cB, '-', 2.5, 'path with the blend, l = 1 \mum');
c2 = contourc(X(1, :), Z(:, 1), heff(2), [hst hst]);  local_cplot(c2, cO, '-', 2.5, 'path with the blend, l = 2 \mum');
P1 = local_cpts(c1);  P1 = P1(P1(:, 1) >= 0, :);  [~, o] = sort(P1(:, 1));  P1 = P1(o, :);
for xq = [3 5 6 6.5 7 7.5 8 9 11]
    [~, i] = min(abs(P1(:, 1) - xq));  p = P1(i, :);  [w, n] = local_blend(p, rb, 1);
    quiver(p(1), p(2), 1.6 * n(1), 1.6 * n(2), 0, 'Color', cB, 'LineWidth', 1.6, 'MaxHeadSize', 0.6, 'HandleVisibility', 'off');
end
plot(nan, nan, '-', 'Color', cB, 'LineWidth', 1.6, 'DisplayName', 'n_{eff} on the l = 1 path');
axis equal;  xlim([-1 14]);  ylim([-1 13]);  xlabel('x  [\mum]');  ylabel('z  [\mum]');  legend('Location', 'northeast');  title('cross-section y = 0');
set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax.Toolbar, 'Visible', 'off');
% along the l = 1 path
W = zeros(size(P1, 1), 2);  H = W;  ang = zeros(size(P1, 1), 1);
for i = 1:size(P1, 1)
    [W(i, :), n, H(i, :)] = local_blend(P1(i, :), rb, 1);  ang(i) = atan2d(n(1), n(2));
end
he = -1 * log(sum(exp(-H / 1), 2));
a2 = nexttile(tl, 2);  hold on;  box on;
plot(P1(:, 1), H(:, 1), 'Color', cGr, 'LineWidth', 2, 'DisplayName', 'h_1 (to the bead)');  plot(P1(:, 1), H(:, 2), 'k--', 'LineWidth', 2, 'DisplayName', 'h_2 (to the glass)');
plot(P1(:, 1), he, 'Color', cB, 'LineWidth', 2.5, 'DisplayName', 'h_{eff}');  ylim([0 8]);  xlim([0 13]);  ylabel('distance  [\mum]');
legend('Location', 'northwest');  title('along the l = 1 \mum path');
a3 = nexttile(tl, 4);  hold on;  box on;
yyaxis left;  plot(P1(:, 1), W(:, 1), '-', 'Color', cGr, 'LineWidth', 2);  plot(P1(:, 1), W(:, 2), 'k--', 'LineWidth', 2);  ylim([0 1]);  ylabel('weight');
yyaxis right;  plot(P1(:, 1), ang, '-', 'Color', cB, 'LineWidth', 2.5);  ylabel('angle of n_{eff} from z  [\circ]');  set(a3, 'YColor', cB);
xlim([0 13]);  xlabel('x  [\mum]');  legend({'w_1 (bead)', 'w_2 (glass)', 'n_{eff} angle'}, 'Location', 'east');
for a = [a2 a3]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
fn = fullfile(fo, 'fig_1029_blend_concept.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open ' fn]);

function [w, n, h] = local_blend(p, rb, l)
    d = [p(1); p(2) - rb];  h = [norm(d) - rb, p(2)];  ns = [d / norm(d), [0; 1]];
    e = exp(-(h - min(h)) / l);  w = e / sum(e);  n = ns * w';  n = n / norm(n);
end
function P = local_cpts(c)
    P = [];  i = 1;
    while i < size(c, 2); m = c(2, i);  P = [P; c(:, i + 1:i + m)'];  i = i + m + 1; end   %#ok<AGROW>
end
function local_cplot(c, col, ls, lw, nm)
    i = 1;  first = true;
    while i < size(c, 2)
        m = c(2, i);  xy = c(:, i + 1:i + m);  hv = 'off';  if first; hv = 'on'; first = false; end
        plot(xy(1, :), xy(2, :), ls, 'Color', col, 'LineWidth', lw, 'DisplayName', nm, 'HandleVisibility', hv);  i = i + m + 1;
    end
end
