% STATUS: SCRATCH -- 2026-10-22 (user asked why cos(theta_x-hat) appears): measurement axes x, y, z and the surface normal
%   w = R_y(theta_y) R_x(theta_x) e_z. theta_y turns w about the measurement y axis, so the tip of w runs on a circle of radius
%   cos(theta_x) around y; the same Delta theta_y moves w by cos(theta_x) Delta theta_y. Left theta_x = 0, right theta_x = 60 deg.
%   EXPIRES: when the explanation is done | output test_results/figs_unused/fig_cos_thetax_axes.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
Rx = @(a) [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
Ry = @(a) [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
wv = @(tx, ty) Ry(ty) * Rx(tx) * [0; 0; 1];
d2r = pi / 180;  dty = 40 * d2r;  ty0 = 0;
fig = figure('Color', 'w', 'Position', [80 80 1400 720]);
tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for tx_deg = [0 60]
    nexttile;  hold on;  axis equal off;  view(150, 22);
    tx = tx_deg * d2r;
    L = 1.25;                                                   % measurement axes
    quiver3(0, 0, 0, L, 0, 0, 0, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.3);  text(L * 1.08, 0, 0, 'x', 'FontSize', 18);
    quiver3(0, 0, 0, 0, 0, L, 0, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.3);  text(0, 0, L * 1.08, 'z', 'FontSize', 18);
    plot3([0 0], [-L L], [0 0], 'Color', [0 0.6 0], 'LineWidth', 3);  text(0, L * 1.1, 0, 'y', 'FontSize', 18, 'Color', [0 0.5 0]);
    ph = linspace(0, 2 * pi, 300);  C = zeros(3, numel(ph));    % circle of the w tip over all theta_y
    for i = 1:numel(ph); C(:, i) = wv(tx, ph(i)); end
    plot3(C(1, :), C(2, :), C(3, :), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);
    yc = -sin(tx);                                              % centre of that circle on the y axis, radius cos(theta_x)
    plot3([0 0], [yc yc], [0 0], 'o', 'Color', [0 0.6 0], 'MarkerFaceColor', [0 0.6 0], 'MarkerSize', 6);
    plot3([0 C(1, 1)], [yc yc], [0 C(3, 1)], ':', 'Color', [0 0.6 0], 'LineWidth', 1.5);
    text(0, yc - 0.2, -0.75, sprintf('radius cos\\theta_x = %.2f', cos(tx)), 'FontSize', 15, 'Color', [0 0.5 0], 'HorizontalAlignment', 'center');
    w0 = wv(tx, ty0);  w1 = wv(tx, ty0 + dty);                  % w before and after Delta theta_y
    s = linspace(ty0, ty0 + dty, 60);  A = zeros(3, numel(s));
    for i = 1:numel(s); A(:, i) = wv(tx, s(i)); end
    plot3(A(1, :), A(2, :), A(3, :), 'm', 'LineWidth', 4);       % path of the tip
    quiver3(0, 0, 0, w0(1), w0(2), w0(3), 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.25);
    quiver3(0, 0, 0, w1(1), w1(2), w1(3), 0, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.25);
    text(w0(1), w0(2) + 0.1, w0(3) + 0.12, 'w (\theta_y = 0)', 'FontSize', 16, 'Color', 'r');
    text(w1(1) + 0.35, w1(2) - 0.3, w1(3) + 0.12, sprintf('w (\\theta_y = %d^\\circ)', round(dty / d2r)), 'FontSize', 16, 'Color', 'b');
    ang = acosd(w0' * w1);
    text(0.02, 0.97, sprintf('\\theta_x = %d^\\circ:  \\Delta\\theta_y = %d^\\circ  \\rightarrow  w turns %.1f^\\circ', ...
         tx_deg, round(dty / d2r), ang), 'FontSize', 17, 'FontWeight', 'bold', 'Units', 'normalized');
    [U, Vp] = meshgrid([-0.55 0.55]);                           % small wall patch normal to w at theta_y = 0
    V0 = Ry(ty0) * Rx(tx);  Pp = V0(:, 1) * U(:)' + V0(:, 2) * Vp(:)';
    patch(Pp(1, [1 2 4 3]), Pp(2, [1 2 4 3]), Pp(3, [1 2 4 3]), [1 0.85 0.85], 'FaceAlpha', 0.35, 'EdgeColor', [0.8 0.5 0.5]);
    xlim([-1.3 1.4]);  ylim([-1.5 1.5]);  zlim([-1.1 1.5]);
    fprintf('theta_x %2d deg: w turns %.2f deg for Delta theta_y %d deg (cos theta_x * Delta = %.2f)\n', tx_deg, ang, round(dty / d2r), cos(tx) * dty / d2r);
end
fo = fullfile(root, 'test_results', 'figs_unused', 'fig_cos_thetax_axes.png');
exportgraphics(fig, fo, 'Resolution', 150);
