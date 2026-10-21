% STATUS: SCRATCH -- 2026-10-20 (user: "u 方向的定義 你可以給我畫個圖"). Local coordinate of the wall, V = R_y(theta_y) R_x(theta_x) =
%   [u v w] (agent_docs/shared/math-model.md). Left: 3-D view, wall patch with theta_x = 10, theta_y = 20 deg, measurement axes x y z
%   (grey) and local axes u v w (u red, v green, w blue). Right: x-z side view with theta_x = 0: u along the wall (down-slope toward +x),
%   w = surface normal, theta_y the angle between z and w. -> test_results/temp_3d_wsonly/fig_wall_uvw.png | run: plot_wall_uvw
function plot_wall_uvw()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    Rx = @(a) [1 0 0; 0 cosd(a) -sind(a); 0 sind(a) cosd(a)];
    Ry = @(a) [cosd(a) 0 sind(a); 0 1 0; -sind(a) 0 cosd(a)];
    FS = 20;  cU = [0.85 0 0];  cV = [0 0.55 0.2];  cW = [0 0.25 0.85];  cG = [0.45 0.45 0.45];  TH = char(952);
    f = figure('Position', [40 40 2000 900], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    % ---- 3-D view ----
    ax = nexttile(tl);  hold(ax, 'on');  axis(ax, 'equal');  axis(ax, 'off');  view(ax, [-35 18]);
    V = Ry(20) * Rx(10);  u = V(:, 1);  v = V(:, 2);  w = V(:, 3);
    c = [-1 1 1 -1; -1 -1 1 1];  P = u * c(1, :) * 1.3 + v * c(2, :) * 1.3;
    fill3(ax, P(1, :), P(2, :), P(3, :), [0.88 0.88 0.88], 'EdgeColor', [0.6 0.6 0.6], 'FaceAlpha', 0.7);
    E = eye(3);  lab = {'x', 'y', 'z'};
    for i = 1:3
        quiver3(ax, 0, 0, 0, E(1, i), E(2, i), E(3, i), 0, 'Color', cG, 'LineWidth', 2, 'LineStyle', '--', 'MaxHeadSize', 0.3);
        text(ax, 1.12 * E(1, i), 1.12 * E(2, i), 1.12 * E(3, i), lab{i}, 'FontSize', FS, 'FontWeight', 'bold', 'Color', cG);
    end
    L = {u, v, w};  C = {cU, cV, cW};  N = {'u', 'v', 'w'};
    for i = 1:3
        quiver3(ax, 0, 0, 0, L{i}(1), L{i}(2), L{i}(3), 0, 'Color', C{i}, 'LineWidth', 4, 'MaxHeadSize', 0.35);
        text(ax, 1.18 * L{i}(1), 1.18 * L{i}(2), 1.18 * L{i}(3), N{i}, 'FontSize', FS + 4, 'FontWeight', 'bold', 'Color', C{i});
    end
    text(ax, -1.6, 0, -1.2, [TH '_x = 10' char(176) ',  ' TH '_y = 20' char(176)], 'FontSize', FS, 'FontWeight', 'bold');
    % ---- x-z side view, theta_x = 0 ----
    ax = nexttile(tl);  hold(ax, 'on');  axis(ax, 'equal');  box(ax, 'on');
    ty = 20;  u2 = [cosd(ty); -sind(ty)];  w2 = [sind(ty); cosd(ty)];
    plot(ax, [-1.5 1.5] * u2(1), [-1.5 1.5] * u2(2), '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 6);           % wall
    quiver(ax, 0, 0, 1, 0, 0, 'Color', cG, 'LineWidth', 2, 'LineStyle', '--', 'MaxHeadSize', 0.3);
    quiver(ax, 0, 0, 0, 1, 0, 'Color', cG, 'LineWidth', 2, 'LineStyle', '--', 'MaxHeadSize', 0.3);
    text(ax, 1.05, 0.05, 'x', 'FontSize', FS, 'FontWeight', 'bold', 'Color', cG);  text(ax, 0.05, 1.08, 'z', 'FontSize', FS, 'FontWeight', 'bold', 'Color', cG);
    quiver(ax, 0, 0, u2(1), u2(2), 0, 'Color', cU, 'LineWidth', 4, 'MaxHeadSize', 0.3);
    quiver(ax, 0, 0, w2(1), w2(2), 0, 'Color', cW, 'LineWidth', 4, 'MaxHeadSize', 0.3);
    text(ax, 1.08 * u2(1), 1.08 * u2(2) - 0.08, 'u', 'FontSize', FS + 4, 'FontWeight', 'bold', 'Color', cU);
    text(ax, 1.08 * w2(1) + 0.03, 1.08 * w2(2), 'w', 'FontSize', FS + 4, 'FontWeight', 'bold', 'Color', cW);
    a = linspace(90, 90 - ty, 30);  plot(ax, 0.55 * cosd(a), 0.55 * sind(a), 'k-', 'LineWidth', 1.5);
    text(ax, 0.12, 0.62, [TH '_y'], 'FontSize', FS, 'FontWeight', 'bold');
    text(ax, -1.45, -0.35, 'wall', 'FontSize', FS, 'FontWeight', 'bold', 'Color', [0.4 0.4 0.4]);
    text(ax, -0.2, -0.6, ['side view (' TH '_x = 0):  v = y, out of the page'], 'FontSize', FS - 4, 'FontWeight', 'bold');
    xlim(ax, [-1.6 1.6]);  ylim(ax, [-0.7 1.3]);  set(ax, 'XTick', [], 'YTick', [], 'LineWidth', 1.5);
    out = fullfile(od, 'fig_wall_uvw.png');
    exportgraphics(f, out, 'Resolution', 140);  close(f);  fprintf('figure: %s\n', out);
end
