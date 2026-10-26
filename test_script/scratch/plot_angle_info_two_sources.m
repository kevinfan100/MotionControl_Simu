% STATUS: SCRATCH -- 2026-10-24 (user: "從頭來過，具體怎麼得到角度資訊，先不管估測器"): the two physical sources of the wall angle,
%   x-z plane, wall tilted theta_y = 30 deg, a-bar_T = 0.6 along the wall, a-bar_N = 0.3 along the normal (measurement coordinate).
%   (a) random component: cloud of thermal steps, variance per direction = gain -> ellipse whose long axis lies along the wall;
%   (b) motion: push along +x, the probe moves A-bar f = first column of A-bar, bent toward the wall direction (moves in z too).
%   EXPIRES: when the explanation is done | output test_results/figs_unused/fig_angle_info_two_sources.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
aT = 0.6;  aN = 0.3;  th = 30;
u = [cosd(th); -sind(th)];  w = [sind(th); cosd(th)];          % x-z components of u (along the wall) and w (normal)
A = aT * (u * u') + aN * (w * w');                             % = aT I + (aN - aT) w w'
fig = figure('Color', 'w', 'Position', [80 80 1300 560]);
tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
% (a) random component
nexttile;  hold on;  box on;  axis equal;
rng(2);  n = 3000;  P = sqrtm(A) * randn(2, n);
plot(P(1, :), P(2, :), '.', 'Color', [0.55 0.75 1], 'MarkerSize', 4);
ph = linspace(0, 2 * pi, 200);  E = sqrtm(A) * [cos(ph); sin(ph)];
plot(E(1, :), E(2, :), 'r', 'LineWidth', 2.5);
plot([-2.5 2.5] * u(1), [-2.5 2.5] * u(2), 'k--', 'LineWidth', 1.2);
text(2.1 * u(1), 2.1 * u(2) - 0.25, 'wall direction', 'FontSize', 14);
xlim([-2.5 2.5]);  ylim([-2 2]);  xlabel('x');  ylabel('z');  set(gca, 'FontSize', 14);
text(0.02, 0.96, '(a) random component: long axis along the wall', 'Units', 'normalized', 'FontSize', 15, 'FontWeight', 'bold');
% (b) motion
nexttile;  hold on;  box on;  axis equal;
f = [1; 0];  m = A * f;
plot([-1.2 1.2] * u(1), [-1.2 1.2] * u(2), 'k--', 'LineWidth', 1.2);
quiver(0, 0, f(1) * 0.9, f(2), 0, 'Color', [0.4 0.4 0.4], 'LineWidth', 3, 'MaxHeadSize', 0.4);
quiver(0, 0, m(1), m(2), 0, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.4);
text(0.92, 0.08, 'push along x', 'FontSize', 14, 'Color', [0.3 0.3 0.3]);
text(m(1) + 0.03, m(2) - 0.08, sprintf('motion (%.3f, %.3f)', m), 'FontSize', 14, 'Color', 'b');
text(1.0 * u(1) - 0.3, 1.0 * u(2) - 0.15, 'wall direction', 'FontSize', 14);
xlim([-0.4 1.6]);  ylim([-0.8 0.5]);  xlabel('x');  ylabel('z');  set(gca, 'FontSize', 14);
text(0.02, 0.96, '(b) motion: push x, it also moves in z', 'Units', 'normalized', 'FontSize', 15, 'FontWeight', 'bold');
exportgraphics(fig, fullfile(root, 'test_results', 'figs_unused', 'fig_angle_info_two_sources.png'), 'Resolution', 150);
fprintf('A-bar (x-z) = [%.3f %.3f; %.3f %.3f]; push x -> motion (%.3f, %.3f), bent %.1f deg below x\n', A, m, -atan2d(m(2), m(1)));
fprintf('cloud: mean(x^2) %.3f  mean(z^2) %.3f  mean(x z) %.3f\n', mean(P(1,:).^2), mean(P(2,:).^2), mean(P(1,:) .* P(2,:)));
