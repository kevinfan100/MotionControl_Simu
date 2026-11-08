% STATUS: SCRATCH -- 2026-11-02 (user): 40 um/s, recovered wall drawn with the true w beside the estimated w-hat (50 ms mean), same run: square +-13 um, 1 um lines, a_d .07, seed 4 of temp_square_why.mat, constant h*. MATLAB window 12. Reference only: the true w does not exist in an experiment. Sanity: 26 v steps; with the true w the distance to the true wall is the gap error only. Output fig_1102_wall_true_w.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  ad = 0.07;  NW = 81;  MU = [char(956) 'm'];  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  L = Rr * hs;
load(fullfile(fo, 'temp_square_why.mat'), 'rs');  r = rs{4};  ok = find(isfinite(r.aN(1:r.kend)));  ks = ok(r.t(ok) >= r.tsc & r.t(ok) <= r.te);  p = r.p(ks, :);
Ws = movmean(local_w(r.thx(ks), r.thy(ks)), NW, 1);  Ws = Ws ./ vecnorm(Ws, 2, 2);  Wt = local_w(r.th_tr(ks, 1), r.th_tr(ks, 2));  isv = movmean([0; diff(p(:, 2))], 21) > 0.012;  [~, nv] = bwlabel(isv);  stepv = conv(double(isv), ones(61, 1), 'same') > 0;  fprintf('v steps %d, gap minus target: mean %+.3f, sd %.3f um\n', nv, mean(r.h(ks)) * Rr - L, std(r.h(ks)) * Rr);
C = {p - L * Ws, p - L * Wt};  tt = {'estimated w, 50 ms mean', 'true w'};
if any(findall(0, 'Type', 'figure') == 12); close(12); end
f = figure(12);  set(f, 'Color', 'w', 'Position', [50 80 1800 760], 'Visible', 'on', 'Name', 'recovered wall, estimated and true w, 40 um/s');  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for c = 1:2
    W = C{c};  W(stepv | abs(W(:, 1)) > 13 | abs(W(:, 2)) > 13, :) = NaN;  a = nexttile(tl);  hold(a, 'on');  plot3(a, W(:, 1), W(:, 2), W(:, 3), 'Color', [0.9 0 0], 'LineWidth', 1.0);  title(a, tt{c});  axis(a, 'equal');  xlim(a, [-13 13]);  ylim(a, [-13 13]);  zlim(a, [-0.5 11]);  view(a, -37.5, 22);  grid(a, 'off');  box(a, 'off');  xlabel(a, ['X (' MU ')']);  ylabel(a, ['Y (' MU ')']);  zlabel(a, ['Z (' MU ')']);  set(a, 'FontSize', 15, 'LineWidth', 1.1);  rotate3d(a, 'on');
end
exportgraphics(f, fullfile(fo, 'fig_1102_wall_true_w.png'), 'Resolution', 130);
function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
