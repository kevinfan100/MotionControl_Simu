% STATUS: SCRATCH -- 2026-10-28 (user): why the diagonal scan leaves the 45 deg meridian -- idealized walk with the TRUE wall angle
%   (no noise, gap held exactly at h* = 2.37 um, i.e. on the sphere of radius 15 + h*), two step rules from the top:
%   old = (u-hat + v-hat)/sqrt 2 with V = R_y(theta_y) R_x(theta_x); new = (e - z) projected on the tangent plane, e = (1, 1, 0)/sqrt 2.
%   Left: 3D (hemisphere, both paths, arrows u-hat, v-hat, old step, new step at polar angle 60 deg on the meridian);
%   right: top view (azimuth). Output test_results/temp_3d_wsonly/fig_1028_scan_dir_3d.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
rb = 15;  Rs = rb + 2.37;  ds = 0.02;  e = [1; 1; 0] / sqrt(2);  FS = 15;  hatc = char(770);
cO = [0.85 0.4 0];  cN = [0 0.25 0.85];  cU = [0.5 0.5 0.5];
P = cell(1, 2);  nmr = {'old (u+v)/sqrt2', 'new meridian'};
for rule = 1:2
    p = [0; 0; Rs];  s = 0;  Pk = p';
    while s < pi / 2 * Rs && p(3) > 0
        [V, n] = local_frame(p);
        if rule == 1; d = (V(:, 1) + V(:, 2)) / sqrt(2); else; d = (eye(3) - n * n') * (e - [0; 0; 1]);  d = d / norm(d); end
        p = p + ds * d;  p = Rs * p / norm(p);  s = s + ds;  Pk(end + 1, :) = p';   %#ok<AGROW>
    end
    P{rule} = Pk;
    fprintf('%s: end azimuth %.1f deg, end polar angle %.1f deg\n', nmr{rule}, atan2d(p(2), p(1)), atan2d(norm(p(1:2)), p(3)));
end
% arrows at polar 60 deg on the 45 deg meridian
ph = 60;  q = Rs * [sind(ph) * e(1:2); cosd(ph)];  [V, n] = local_frame(q);  L = 6;
dO = (V(:, 1) + V(:, 2)) / sqrt(2);  dN = (eye(3) - n * n') * (e - [0; 0; 1]);  dN = dN / norm(dN);
fprintf('at polar 60 deg: u-hat %s, v-hat %s, old step %s (azimuth %.1f), new step %s (azimuth %.1f)\n', mat2str(round(V(:, 1)', 3)), mat2str(round(V(:, 2)', 3)), ...
        mat2str(round(dO', 3)), atan2d(dO(2), dO(1)), mat2str(round(dN', 3)), atan2d(dN(2), dN(1)));
f = figure('Color', 'w', 'Position', [40 40 2000 900]);  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl);  hold on;  box on;
[TH, PH] = meshgrid(linspace(0, 2 * pi, 80), linspace(0, pi / 2, 40));
surf(rb * sin(PH) .* cos(TH), rb * sin(PH) .* sin(TH), rb * cos(PH), 'FaceColor', [0.85 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.7, 'HandleVisibility', 'off');
tm = linspace(0, pi / 2, 100);  plot3(Rs * sin(tm) * e(1), Rs * sin(tm) * e(2), Rs * cos(tm), 'k--', 'LineWidth', 1.5, 'DisplayName', '45° meridian');
plot3(P{1}(:, 1), P{1}(:, 2), P{1}(:, 3), 'Color', cO, 'LineWidth', 3, 'DisplayName', ['old: (u' hatc ' + v' hatc ')/√2']);
plot3(P{2}(:, 1), P{2}(:, 2), P{2}(:, 3), 'Color', cN, 'LineWidth', 3, 'DisplayName', 'new: (e − z) on the tangent plane');
quiver3(q(1), q(2), q(3), L * V(1, 1), L * V(2, 1), L * V(3, 1), 0, 'Color', cU, 'LineWidth', 2, 'MaxHeadSize', 0.6, 'DisplayName', ['u' hatc ', v' hatc]);
quiver3(q(1), q(2), q(3), L * V(1, 2), L * V(2, 2), L * V(3, 2), 0, 'Color', cU, 'LineWidth', 2, 'MaxHeadSize', 0.6, 'HandleVisibility', 'off');
text(q(1) + 1.15 * L * V(1, 1), q(2) + 1.15 * L * V(2, 1), q(3) + 1.15 * L * V(3, 1), ['u' hatc], 'FontSize', FS + 2, 'FontWeight', 'bold');
text(q(1) + 1.15 * L * V(1, 2), q(2) + 1.15 * L * V(2, 2), q(3) + 1.15 * L * V(3, 2), ['v' hatc], 'FontSize', FS + 2, 'FontWeight', 'bold');
quiver3(q(1), q(2), q(3), L * dO(1), L * dO(2), L * dO(3), 0, 'Color', cO, 'LineWidth', 2.5, 'MaxHeadSize', 0.6, 'HandleVisibility', 'off');
quiver3(q(1), q(2), q(3), L * dN(1), L * dN(2), L * dN(3), 0, 'Color', cN, 'LineWidth', 2.5, 'MaxHeadSize', 0.6, 'HandleVisibility', 'off');
plot3(q(1), q(2), q(3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'HandleVisibility', 'off');
axis equal;  view(125, 22);  xlabel('x  [\mum]');  ylabel('y  [\mum]');  zlabel('z  [\mum]');  xlim([-5 20]);  ylim([-5 22]);  zlim([0 20]);
legend('Location', 'northoutside', 'Orientation', 'horizontal');  set(ax, 'FontSize', FS, 'FontWeight', 'bold');  set(ax.Toolbar, 'Visible', 'off');
ax2 = nexttile(tl);  hold on;  box on;
fill(rb * cos(linspace(0, 2 * pi, 200)), rb * sin(linspace(0, 2 * pi, 200)), [0.9 0.9 0.9], 'EdgeColor', [0.5 0.5 0.5], 'HandleVisibility', 'off');
plot([0 Rs * e(1)], [0 Rs * e(2)], 'k--', 'LineWidth', 1.5, 'DisplayName', '45° meridian');
plot(P{1}(:, 1), P{1}(:, 2), 'Color', cO, 'LineWidth', 3, 'DisplayName', 'old');  plot(P{2}(:, 1), P{2}(:, 2), 'Color', cN, 'LineWidth', 3, 'DisplayName', 'new');
plot(q(1), q(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'arrow point (60° from the top)');
axis equal;  xlim([-3 20]);  ylim([-3 20]);  xlabel('x  [\mum]');  ylabel('y  [\mum]');  legend('Location', 'northoutside', 'Orientation', 'horizontal');
title('top view');  set(ax2, 'FontSize', FS, 'FontWeight', 'bold');  set(ax2.Toolbar, 'Visible', 'off');
fn = fullfile(fo, 'fig_1028_scan_dir_3d.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);  system(['open ' fn]);

function [V, n] = local_frame(p)                                 % true local frame of the sphere at p (same angle convention as the runner)
    n = p / norm(p);  thx = -asin(n(2));  thy = atan2(n(1), n(3));
    V = [cos(thy) 0 sin(thy); 0 1 0; -sin(thy) 0 cos(thy)] * [1 0 0; 0 cos(thx) -sin(thx); 0 sin(thx) cos(thx)];
end
