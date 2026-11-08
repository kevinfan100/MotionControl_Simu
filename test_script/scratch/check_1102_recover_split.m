% STATUS: SCRATCH -- 2026-11-02 (user): the recovered lines of the square scan move far more than the .21 um error. Split the top-view move of the wall point from the probe into geometry (with the true w) and angle error (estimated minus true w). temp_square_why.mat arm D (+-13, v .5 um, seed 1). Figure 6 window: top view, estimated and true w. Sanity: at r > 10 the geometry part is near 0. Output fig_1102_recover_split.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  ad = 0.07;  MU = [char(956) 'm'];  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  L = Rr * hs;
load(fullfile(fo, 'temp_square_why.mat'), 'rs');  r = rs{end};  ok = find(isfinite(r.aN(1:r.kend)));  ks = ok(r.t(ok) >= r.tsc & r.t(ok) <= r.te);  p = r.p(ks, :);
We = local_w(r.thx(ks), r.thy(ks));  Wt = local_w(r.th_tr(ks, 1), r.th_tr(ks, 2));  Pe = movmean(p - L * We, 41, 1);  Pt = movmean(p - L * Wt, 41, 1);  pm = movmean(p, 41, 1);
geo = vecnorm(Pt(:, 1:2) - pm(:, 1:2), 2, 2);  err = vecnorm(Pe(:, 1:2) - Pt(:, 1:2), 2, 2);  err3 = vecnorm(Pe - Pt, 2, 2);  rq = hypot(p(:, 1), p(:, 2));  seg = {rq > 10, rq > 7.2 & rq < 9, rq < 7.2};  nm = {'floor r > 10', 'ring r 7.2-9', 'bead r < 7.2'};
for j = 1:3; m = seg{j}; fprintf('%s: top-view move from geometry median %.3f, max %.3f um | from the angle error median %.3f, 95%% %.3f, max %.3f um (3-D median %.3f, 95%% %.3f)\n', nm{j}, median(geo(m)), max(geo(m)), median(err(m)), prctile(err(m), 95), max(err(m)), median(err3(m)), prctile(err3(m), 95)); end
stepv = conv(double(movmean([0; diff(p(:, 2))], 21) > 0.012), ones(61, 1), 'same') > 0;  bad = stepv | abs(p(:, 1)) > 13 | abs(p(:, 2)) > 13;  Pe(bad, :) = NaN;  Pt(bad, :) = NaN;
if any(findall(0, 'Type', 'figure') == 6); close(6); end
f = figure(6);  set(f, 'Color', 'w', 'Position', [80 80 1550 780], 'Visible', 'on', 'Name', 'recovered lines from above, square, v 0.5 um');  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  C = {Pe, Pt};  tt = {'estimated w', 'true w'};
for j = 1:2; a = nexttile(tl);  plot(a, C{j}(:, 1), C{j}(:, 2), 'Color', [0.9 0 0], 'LineWidth', 0.7);  axis(a, 'equal');  xlim(a, [-13 13]);  ylim(a, [-13 13]);  box(a, 'on');  title(a, tt{j});  xlabel(a, ['X (' MU ')']);  ylabel(a, ['Y (' MU ')']);  set(a, 'FontSize', 14, 'FontWeight', 'bold', 'LineWidth', 1.1); end
exportgraphics(f, fullfile(fo, 'fig_1102_recover_split.png'), 'Resolution', 120);

function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
