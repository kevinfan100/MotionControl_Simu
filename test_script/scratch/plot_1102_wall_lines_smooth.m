% STATUS: SCRATCH -- 2026-11-02 (user): wall recovered along the scan lines, 1 um line spacing, offline two-sided mean of w-hat over 50 ms (81 samples, renormalised), p - R (1 + h*) w-hat, no fitted sphere. Square +-13 um, a_d .07, seed 4 of temp_square_why.mat. MATLAB window 10. Sanity: 26 v steps removed; wall error against the true w printed. Output fig_1102_wall_lines_smooth.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  ad = 0.07;  NW = 81;  MU = [char(956) 'm'];  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  L = Rr * hs;
load(fullfile(fo, 'temp_square_why.mat'), 'rs');  r = rs{4};  ok = find(isfinite(r.aN(1:r.kend)));  ks = ok(r.t(ok) >= r.tsc & r.t(ok) <= r.te);  p = r.p(ks, :);
We = local_w(r.thx(ks), r.thy(ks));  Wt = local_w(r.th_tr(ks, 1), r.th_tr(ks, 2));  Ws = movmean(We, NW, 1);  Ws = Ws ./ vecnorm(Ws, 2, 2);  W = p - L * Ws;  e = L * vecnorm(Ws - Wt, 2, 2);
isv = movmean([0; diff(p(:, 2))], 21) > 0.012;  [~, nv] = bwlabel(isv);  stepv = conv(double(isv), ones(61, 1), 'same') > 0;  fprintf('v steps %d; wall error against the true w: median %.3f, 95%% %.3f um; x %.1f..%.1f, y %.1f..%.1f um\n', nv, median(e), prctile(e, 95), min(p(:, 1)), max(p(:, 1)), min(p(:, 2)), max(p(:, 2)));
W(stepv | abs(W(:, 1)) > 13 | abs(W(:, 2)) > 13, :) = NaN;
if any(findall(0, 'Type', 'figure') == 10); close(10); end
f = figure(10);  set(f, 'Color', 'w', 'Position', [80 80 1250 860], 'Visible', 'on', 'Name', 'recovered wall, 1 um lines, w-hat 50 ms');  a = axes(f);  hold(a, 'on');  plot3(a, W(:, 1), W(:, 2), W(:, 3), 'Color', [0.9 0 0], 'LineWidth', 1.1);
axis(a, 'equal');  xlim(a, [-13 13]);  ylim(a, [-13 13]);  zlim(a, [-0.5 11]);  view(a, -37.5, 22);  grid(a, 'off');  box(a, 'off');  xlabel(a, ['X (' MU ')']);  ylabel(a, ['Y (' MU ')']);  zlabel(a, ['Z (' MU ')']);  set(a, 'FontSize', 16, 'LineWidth', 1.1);  rotate3d(a, 'on');
exportgraphics(f, fullfile(fo, 'fig_1102_wall_lines_smooth.png'), 'Resolution', 130);

function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
