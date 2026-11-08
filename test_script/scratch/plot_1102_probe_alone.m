% STATUS: SCRATCH -- 2026-11-02 (user): window 8: actual trajectory of the magnetic bead alone, square +-13 um, seed 4 (temp_square_why.mat), start, descent, scan over the plain glass bead and floor. Window 9: side view to scale of the rho 4 wall with the magnetic bead (radius 2.25 um) at three places: centre (blue) to wall point (red) along w = 2.43 um at a_d .07. Sanity: clearance .18 um in every drawing. Outputs fig_1102_probe_alone.png, fig_1102_wall_point.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  rbd = 5;  rho = 4;  ad = 0.07;  MU = [char(956) 'm'];  load(fullfile(fo, 'temp_square_why.mat'), 'rs');  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  L = Rr * hs;
r = rs{4};  ok = find(isfinite(r.h(1:r.kend)) & all(isfinite(r.p(1:r.kend, :)), 2));  k = ok(r.t(ok) <= r.te);  P = r.p(k, :);  t = r.t(k);  t = t(:);  pre = t < r.tsc;  [Xs, Ys, Zs] = sphere(80);
if any(findall(0, 'Type', 'figure') == 8); close(8); end
f = figure(8);  set(f, 'Color', 'w', 'Position', [60 60 1300 880], 'Visible', 'on', 'Name', 'actual trajectory, square, seed 4');  a = axes(f);  hold(a, 'on');
surf(a, rbd * Xs, rbd * Ys, rbd + rbd * Zs, 'FaceColor', [0.85 0.72 0.45], 'EdgeColor', 'none', 'AmbientStrength', 0.6, 'DiffuseStrength', 0.6, 'SpecularStrength', 0);  camlight(a);  lighting(a, 'gouraud');  patch(a, [-16 16 16 -16], [-16 -16 16 16], [-0.02 -0.02 -0.02 -0.02], [0.74 0.76 0.82], 'EdgeColor', 'none', 'FaceLighting', 'none');
h1 = plot3(a, P(pre, 1), P(pre, 2), P(pre, 3), 'Color', [0.85 0 0], 'LineWidth', 2.2);  h2 = plot3(a, P(~pre, 1), P(~pre, 2), P(~pre, 3), 'Color', [0 0 1], 'LineWidth', 0.9);  h3 = plot3(a, P(1, 1), P(1, 2), P(1, 3), 'o', 'MarkerSize', 11, 'MarkerFaceColor', [0 0.6 0.2], 'MarkerEdgeColor', 'k');
axis(a, 'equal');  xlim(a, [-16 16]);  ylim(a, [-16 16]);  zlim(a, [-0.2 16]);  view(a, -37.5, 25);  grid(a, 'off');  box(a, 'off');  xlabel(a, ['X (' MU ')']);  ylabel(a, ['Y (' MU ')']);  zlabel(a, ['Z (' MU ')']);  set(a, 'FontSize', 16, 'LineWidth', 1.1);  legend([h3 h1 h2], {'start', 'hold and descent', 'scan'}, 'Location', 'northoutside', 'Orientation', 'horizontal');  rotate3d(a, 'on');
exportgraphics(f, fullfile(fo, 'fig_1102_probe_alone.png'), 'Resolution', 130);
Rt = 2 * sqrt(rbd * rho);  at = atan2(Rt, rho - rbd);  b = linspace(0, at, 300);  c = linspace(at, 0, 300);  wx = [rbd * sin(b), Rt - rho * sin(c), 18];  wz = [rbd + rbd * cos(b), rho - rho * cos(c), 0];
Q = [rbd * sind(40), rbd + rbd * cosd(40), sind(40), cosd(40);  Rt - rho * sind(45), rho - rho * cosd(45), sind(45), cosd(45);  14, 0, 0, 1];  th = linspace(0, 2 * pi, 200);
if any(findall(0, 'Type', 'figure') == 9); close(9); end
f2 = figure(9);  set(f2, 'Color', 'w', 'Position', [200 100 1300 780], 'Visible', 'on', 'Name', 'magnetic bead and wall point, side view to scale');  a = axes(f2);  hold(a, 'on');  plot(a, wx, wz, 'k', 'LineWidth', 2.2);
for j = 1:3
    w = Q(j, 3:4);  W = Q(j, 1:2);  C = W + L * w;  fill(a, C(1) + Rr * cos(th), C(2) + Rr * sin(th), [0.75 0.82 0.95], 'EdgeColor', [0 0.25 0.85], 'LineWidth', 1.4);  plot(a, [C(1) W(1)], [C(2) W(2)], 'Color', [0 0 0], 'LineWidth', 1.2);  plot(a, C(1), C(2), 'o', 'MarkerSize', 9, 'MarkerFaceColor', [0 0 1], 'MarkerEdgeColor', 'k');  plot(a, W(1), W(2), 'o', 'MarkerSize', 9, 'MarkerFaceColor', [0.9 0 0], 'MarkerEdgeColor', 'k');
end
axis(a, 'equal');  xlim(a, [0 18]);  ylim(a, [-0.5 13]);  box(a, 'on');  xlabel(a, ['radial position  [' MU ']']);  ylabel(a, ['z  [' MU ']']);  set(a, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 1.2);
exportgraphics(f2, fullfile(fo, 'fig_1102_wall_point.png'), 'Resolution', 130);
fprintf('centre to wall %.3f um, magnetic bead radius %.2f um, clearance %.3f um\n', L, Rr, L - Rr);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
