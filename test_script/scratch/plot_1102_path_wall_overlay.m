% STATUS: SCRATCH -- 2026-11-02 (user): probe path (blue) and wall points (red) overlaid, no smoothing, 40 um/s, square +-13 um, 1 um lines, a_d .07, seed 4 of temp_square_why.mat. Red = p - R (1 + h*) w with the normal of the wall from the plant, constant h*, so it is the wall under the probe, not an estimate; do not caption it as estimated or recovered. MATLAB window 13. Sanity: 26 v steps. Output fig_1102_path_wall_overlay.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  ad = 0.07;  MU = [char(956) 'm'];  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  L = Rr * hs;
load(fullfile(fo, 'temp_square_why.mat'), 'rs');  r = rs{4};  ok = find(isfinite(r.aN(1:r.kend)));  ks = ok(r.t(ok) >= r.tsc & r.t(ok) <= r.te);  p = r.p(ks, :);  Wt = local_w(r.th_tr(ks, 1), r.th_tr(ks, 2));  W = p - L * Wt;
isv = movmean([0; diff(p(:, 2))], 21) > 0.012;  [~, nv] = bwlabel(isv);  stepv = conv(double(isv), ones(61, 1), 'same') > 0;  fprintf('v steps %d\n', nv);  P = p;  bad = stepv | abs(p(:, 1)) > 13 | abs(p(:, 2)) > 13;  P(bad, :) = NaN;  W(bad, :) = NaN;
if any(findall(0, 'Type', 'figure') == 13); close(13); end
f = figure(13);  set(f, 'Color', 'w', 'Position', [70 70 1300 900], 'Visible', 'on', 'Name', 'probe path and wall points');  a = axes(f);  hold(a, 'on');  h1 = plot3(a, P(:, 1), P(:, 2), P(:, 3), 'Color', [0 0 1], 'LineWidth', 0.9);  h2 = plot3(a, W(:, 1), W(:, 2), W(:, 3), 'Color', [0.9 0 0], 'LineWidth', 0.9);
axis(a, 'equal');  xlim(a, [-13 13]);  ylim(a, [-13 13]);  zlim(a, [-0.5 13.5]);  view(a, -37.5, 22);  grid(a, 'off');  box(a, 'off');  xlabel(a, ['X (' MU ')']);  ylabel(a, ['Y (' MU ')']);  zlabel(a, ['Z (' MU ')']);  set(a, 'FontSize', 16, 'LineWidth', 1.1);  legend([h1 h2], {'Probe path', 'Wall point'}, 'Location', 'northoutside', 'Orientation', 'horizontal');  rotate3d(a, 'on');
exportgraphics(f, fullfile(fo, 'fig_1102_path_wall_overlay.png'), 'Resolution', 130);
function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
