% STATUS: SCRATCH -- 2026-10-28 (user): why the large angle enters the small-angle matrix. V = R_y(theta_y) R_x(theta_x): delta-theta_y turns
%   about the FIXED measurement y axis; in the local coordinate that axis is V' y = (0, cos theta_x, -sin theta_x), i.e. cos theta_x along
%   v-hat (tilts the wall) + (-sin theta_x) along w-hat (spins u-hat, v-hat inside the wall, the wall does not change).
%   Point: 60 deg from the top on the 45 deg meridian of the hemisphere (theta_x = -37.8 deg). Left: 3D with u-hat, v-hat, w-hat, the y axis
%   through the point and its two parts; right: the v-hat / w-hat plane. Output test_results/temp_3d_wsonly/fig_1028_dthy_axis.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
rb = 15;  e = [1; 1; 0] / sqrt(2);  FS = 16;  hatc = char(770);  th = char(952);  L = 7;
ph = 60;  q = rb * [sind(ph) * e(1:2); cosd(ph)];  n = q / norm(q);  thx = -asin(n(2));  thy = atan2(n(1), n(3));
V = [cos(thy) 0 sin(thy); 0 1 0; -sin(thy) 0 cos(thy)] * [1 0 0; 0 cos(thx) -sin(thx); 0 sin(thx) cos(thx)];
yl = V' * [0; 1; 0];  c = cos(thx);  s = sin(thx);
fprintf('theta_x %.1f deg, theta_y %.1f deg | y axis in local = %s = (0, cos, -sin) = (0, %.3f, %.3f)\n', thx * 180 / pi, thy * 180 / pi, mat2str(round(yl', 3)), c, -s);
cU = [0.85 0 0];  cV = [0 0.55 0.2];  cW = [0 0 0];  cY = [0.55 0.2 0.75];
f = figure('Color', 'w', 'Position', [40 40 2100 950]);  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
% ---- 3D ----
ax = nexttile(tl);  hold on;
hp = 9;  [gu, gv] = meshgrid([-1 1] * hp);                          % the wall near the point: tangent patch spanned by u-hat, v-hat
surf(q(1) + gu * V(1, 1) + gv * V(1, 2), q(2) + gu * V(2, 1) + gv * V(2, 2), q(3) + gu * V(3, 1) + gv * V(3, 2), 'FaceColor', [0.80 0.82 0.86], ...
     'EdgeColor', [0.6 0.6 0.6], 'FaceAlpha', 0.55);
ar = @(v, col, lw) quiver3(q(1), q(2), q(3), L * v(1), L * v(2), L * v(3), 0, 'Color', col, 'LineWidth', lw, 'MaxHeadSize', 0.4);
ar(V(:, 1), cU, 2.5);  ar(V(:, 2), cV, 2.5);  ar(V(:, 3), cW, 2.5);  ar([0; 1; 0], cY, 3.5);
pv = c * V(:, 2);  pw = -s * V(:, 3);                              % the two parts of the y axis
plot3(q(1) + L * [0 pv(1)], q(2) + L * [0 pv(2)], q(3) + L * [0 pv(3)], '--', 'Color', cV, 'LineWidth', 2.5);
plot3(q(1) + L * [0 pw(1)], q(2) + L * [0 pw(2)], q(3) + L * [0 pw(3)], '--', 'Color', cW, 'LineWidth', 2.5);
plot3(q(1) + L * [pv(1) pv(1) + pw(1)], q(2) + L * [pv(2) pv(2) + pw(2)], q(3) + L * [pv(3) pv(3) + pw(3)], ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.3);
plot3(q(1) + L * [pw(1) pw(1) + pv(1)], q(2) + L * [pw(2) pw(2) + pv(2)], q(3) + L * [pw(3) pw(3) + pv(3)], ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.3);
tx = @(v, str, col) text(q(1) + 1.18 * L * v(1), q(2) + 1.18 * L * v(2), q(3) + 1.18 * L * v(3), str, 'FontSize', FS + 2, 'FontWeight', 'bold', 'Color', col);
tx(V(:, 1), ['u' hatc], cU);  tx(V(:, 2), ['v' hatc], cV);  tx(V(:, 3), ['w' hatc], cW);  tx([0; 1; 0], 'y  (axis of δθ_y)', cY);
plot3(q(1), q(2), q(3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
dv = -0.8 * V(:, 1) + 0.45 * V(:, 3) + 0.3 * V(:, 2);  dv = dv / norm(dv);   % look across the v-hat / w-hat plane, from outside the wall
axis equal;  box on;  grid off;  view(atan2d(dv(1), -dv(2)), asind(dv(3)));  xlim(q(1) + [-11 11]);  ylim(q(2) + [-11 11]);  zlim(q(3) + [-11 11]);
xlabel('x  [\mum]');  ylabel('y  [\mum]');  zlabel('z  [\mum]');  set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold');  set(ax.Toolbar, 'Visible', 'off');
title(sprintf('wall patch 60° from the top on the 45° meridian:  %s_x = %.1f°', th, thx * 180 / pi), 'FontSize', FS);
% ---- v-hat / w-hat plane ----
ax2 = nexttile(tl);  hold on;  axis equal;  box on;
quiver(0, 0, 1, 0, 0, 'Color', cV, 'LineWidth', 2.5, 'MaxHeadSize', 0.15);  quiver(0, 0, 0, 1, 0, 'Color', cW, 'LineWidth', 2.5, 'MaxHeadSize', 0.15);
quiver(0, 0, c, -s, 0, 'Color', cY, 'LineWidth', 3.5, 'MaxHeadSize', 0.15);
plot([0 c], [0 0], '--', 'Color', cV, 'LineWidth', 3);  plot([0 0], [0 -s], '--', 'Color', cW, 'LineWidth', 3);
plot([c c], [0 -s], ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.3);  plot([0 c], [-s -s], ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.3);
text(1.04, 0, ['v' hatc], 'FontSize', FS + 4, 'FontWeight', 'bold', 'Color', cV);  text(0.02, 1.06, ['w' hatc], 'FontSize', FS + 4, 'FontWeight', 'bold', 'Color', cW);
text(c + 0.03, -s + 0.05, 'y', 'FontSize', FS + 4, 'FontWeight', 'bold', 'Color', cY);
text(c / 2 - 0.25, -0.07, sprintf('cos%s_x = %.2f : tilts the wall', th, c), 'FontSize', FS, 'FontWeight', 'bold', 'Color', cV);
text(-0.6, -s / 2, sprintf('-sin%s_x = %.2f :\nspins inside the wall', th, -s), 'FontSize', FS, 'FontWeight', 'bold', 'Color', cW);
a = linspace(0, atan2(-s, c), 40);  plot(0.25 * cos(a), 0.25 * sin(a), 'k', 'LineWidth', 1.3);
text(0.28 * cos(a(20)) + 0.02, 0.28 * sin(a(20)), sprintf('|%s_x| = %.1f°', th, abs(thx) * 180 / pi), 'FontSize', FS, 'FontWeight', 'bold');
xlim([-0.65 1.2]);  ylim([-0.15 1.2]);  set(ax2, 'XTick', [], 'YTick', [], 'LineWidth', 1.2);
title('the v-hat / w-hat plane at that point (u-hat out of the page)', 'FontSize', FS);
fn = fullfile(fo, 'fig_1028_dthy_axis.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open ' fn]);
