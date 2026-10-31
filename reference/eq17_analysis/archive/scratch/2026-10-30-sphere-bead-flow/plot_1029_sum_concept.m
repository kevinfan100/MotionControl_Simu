% STATUS: SCRATCH -- 2026-10-29 (user): the simplest weighting -- add the extra drag of each wall (plane law, no extra parameter):
%   Gamma / gamma_N = I + (C_sphere - I) + (C_bottom - I), C_i = V_i diag(c_par(h_i), c_par(h_i), c_perp(h_i)) V_i'.
%   The "weight" of a wall = its own extra drag c - 1 (large when near, 0 when far). Cross-section y = 0:
%   left -- where the probe centre sits when the normal gain is regulated to a_d = 0.05 (smallest eigenvalue of inv(Gamma/gamma_N)),
%   nearest wall only (kink) vs drag sum; right -- along the drag-sum path: extra drag of each wall (c_perp - 1) and the tilt of the
%   stiffest direction. Output test_results/temp_3d_wsonly/fig_1029_sum_concept.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
addpath(genpath(fullfile(root, 'model')));
rb = 5;  R = 2.25;  ad = 0.05;  FS = 15;  cB = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];
xg = linspace(0, 14, 561);  zg = linspace(0.2, 14, 553);  [X, Z] = meshgrid(xg, zg);
aN_sum = zeros(size(X));  aN_one = aN_sum;
for i = 1:numel(X)
    [Gs, ~, a1] = local_G([X(i) 0 Z(i)], rb, R);  aN_sum(i) = min(eig(inv(Gs)));  aN_one(i) = a1;
end
f = figure('Color', 'w', 'Position', [30 30 2000 800]);  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl);  hold on;  box on;
tg = linspace(0, pi, 200);  fill([rb * sin(tg) 0], [rb + rb * cos(tg) rb], [0.88 0.88 0.9], 'EdgeColor', cGr, 'LineWidth', 2.5, 'DisplayName', 'sphere');
fill([0 14 14 0], [0 0 -1 -1], [0.85 0.85 0.85], 'EdgeColor', 'none', 'HandleVisibility', 'off');  plot([0 14], [0 0], 'k', 'LineWidth', 2.5, 'DisplayName', 'bottom wall');
local_c(contourc(xg, zg, aN_one, [ad ad]), 'k', '--', 2, 'nearest wall only');
c = contourc(xg, zg, aN_sum, [ad ad]);  local_c(c, cB, '-', 2.5, 'drag of both walls added');
axis equal;  xlim([0 14]);  ylim([-1 13]);  xlabel('X  [\mum]');  ylabel('Z  [\mum]');  legend('Location', 'northeast');
title(sprintf('probe centre where the normal gain = a_d = %.2f (Y = 0)', ad));
P = [];  j = 1;  while j < size(c, 2); m = c(2, j);  P = [P; c(:, j + 1:j + m)'];  j = j + m + 1; end   %#ok<AGROW>
P = P(P(:, 1) > 0.05, :);  [~, o] = sort(P(:, 1));  P = P(o, :);  ex = zeros(size(P, 1), 2);  tilt = zeros(size(P, 1), 1);  hh = ex;
for i = 1:size(P, 1)
    [Gs, ex(i, :), ~, hh(i, :)] = local_G([P(i, 1) 0 P(i, 2)], rb, R);  [V, D] = eig(0.5 * (Gs + Gs'));  [~, im] = max(diag(D));  n = V(:, im);  tilt(i) = abs(atan2d(n(1), n(3)));
    if tilt(i) > 90; tilt(i) = 180 - tilt(i); end
end
a2 = nexttile(tl);  hold on;  box on;
yyaxis left;  plot(P(:, 1), ex(:, 1), '-', 'Color', cGr, 'LineWidth', 2.5);  plot(P(:, 1), ex(:, 2), 'k--', 'LineWidth', 2.5);  ylabel('extra drag  c_\perp - 1');  set(a2, 'YColor', 'k');  ylim([0 22]);
yyaxis right;  plot(P(:, 1), tilt, '-', 'Color', cB, 'LineWidth', 2.5);  ylabel('tilt of the stiffest direction from z  [\circ]');  set(a2, 'YColor', cB);  ylim([0 100]);
xlim([0 13]);  xlabel('X  [\mum]');  title('along the drag-sum path');  legend({'sphere', 'bottom wall', 'tilt'}, 'Location', 'west');
for a = [ax a2]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
fn = fullfile(fo, 'fig_1029_sum_concept.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open ' fn]);
for xq = [0.1 5 7 7.5 8 9 12]
    [~, i] = min(abs(P(:, 1) - xq));  fprintf('X %.1f: Z %.2f | h_sphere %.2f h_bottom %.2f um | extra drag sphere %.2f bottom %.2f | tilt %.0f deg\n', P(i, 1), P(i, 2), hh(i, 1), hh(i, 2), ex(i, 1), ex(i, 2), tilt(i));
end

function [Gn, ex, aone, hs] = local_G(p, rb, R)                 % drag sum; ex = c_perp - 1 of each wall; aone = normal gain of the nearest wall only
    c = [0 0 rb];  d = p - c;  hs = [norm(d) - rb, p(3)];  ns = [d / norm(d); 0 0 1];  Gn = eye(3);  ex = [0 0];  cp = [0 0];
    for i = 1:2
        [cpa, cpe] = calc_correction_functions(max(hs(i) / R, 1.0005));  n = ns(i, :)';
        Gn = Gn + (cpa - 1) * (eye(3) - n * n') + (cpe - 1) * (n * n');  ex(i) = cpe - 1;  cp(i) = cpe;
    end
    [~, im] = min(hs);  aone = 1 / cp(im);
end
function local_c(c, col, ls, lw, nm)
    i = 1;  first = true;
    while i < size(c, 2)
        m = c(2, i);  xy = c(:, i + 1:i + m);  hv = 'off';  if first; hv = 'on';  first = false; end
        plot(xy(1, :), xy(2, :), ls, 'Color', col, 'LineWidth', lw, 'DisplayName', nm, 'HandleVisibility', hv);  i = i + m + 1;
    end
end
