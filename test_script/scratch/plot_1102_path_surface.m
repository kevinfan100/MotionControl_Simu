% STATUS: SCRATCH -- 2026-11-02 (user): left probe path with start and descent, no bead, no floor; right sample surface, red lines. 40 um/s, square +-13 um, 1 um lines, a_d .07, seed 4 of temp_square_why.mat, no smoothing, same view and limits. Right = p - R (1 + h*) w with the wall normal from the plant, not an estimate. MATLAB window 14. Sanity: 26 v steps. Output fig_1102_path_surface.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  ad = 0.07;  MU = [char(956) 'm'];  VW = [-15 55];  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  L = Rr * hs;
load(fullfile(fo, 'temp_square_why.mat'), 'rs');  r = rs{4};  ok = find(isfinite(r.h(1:r.kend)) & all(isfinite(r.p(1:r.kend, :)), 2));  k = ok(r.t(ok) <= r.te);  P = r.p(k, :);  t = r.t(k);  t = t(:);  pre = t < r.tsc;
ks = k(~pre);  p = r.p(ks, :);  W = p - L * local_w(r.th_tr(ks, 1), r.th_tr(ks, 2));  isv = movmean([0; diff(p(:, 2))], 21) > 0.012;  [~, nv] = bwlabel(isv);  stepv = conv(double(isv), ones(61, 1), 'same') > 0;  fprintf('v steps %d\n', nv);  W(stepv | abs(p(:, 1)) > 13 | abs(p(:, 2)) > 13, :) = NaN;
if any(findall(0, 'Type', 'figure') == 14); close(14); end
f = figure(14);  set(f, 'Color', 'w', 'Position', [40 60 1900 860], 'Visible', 'on', 'Name', 'probe path and sample surface');  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
a1 = nexttile(tl);  hold(a1, 'on');  h1 = plot3(a1, P(pre, 1), P(pre, 2), P(pre, 3), 'Color', [0.85 0 0], 'LineWidth', 2.2);  h2 = plot3(a1, P(~pre, 1), P(~pre, 2), P(~pre, 3), 'Color', [0 0 1], 'LineWidth', 0.9);  h3 = plot3(a1, P(1, 1), P(1, 2), P(1, 3), 'o', 'MarkerSize', 11, 'MarkerFaceColor', [0 0.6 0.2], 'MarkerEdgeColor', 'k');  legend(a1, [h3 h1 h2], {'start', 'hold and descent', 'probe path'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
a2 = nexttile(tl);  hold(a2, 'on');  h4 = plot3(a2, W(:, 1), W(:, 2), W(:, 3), 'Color', [0.9 0 0], 'LineWidth', 0.9);  legend(a2, h4, 'Sample surface', 'Location', 'northoutside', 'Orientation', 'horizontal');
for a = [a1 a2]; axis(a, 'equal');  xlim(a, [-15 15]);  ylim(a, [-15 15]);  zlim(a, [-0.5 16]);  view(a, VW(1), VW(2));  grid(a, 'off');  box(a, 'off');  xlabel(a, ['X (' MU ')']);  ylabel(a, ['Y (' MU ')']);  zlabel(a, ['Z (' MU ')']);  set(a, 'FontSize', 15, 'LineWidth', 1.1);  rotate3d(a, 'on'); end
exportgraphics(f, fullfile(fo, 'fig_1102_path_surface.png'), 'Resolution', 130);
function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
