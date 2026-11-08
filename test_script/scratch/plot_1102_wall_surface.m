% STATUS: SCRATCH -- 2026-11-02 (user): can the recovered wall be drawn as a solid. Wall points p - R (1 + h*) w-hat (moving mean 41) of temp_rho4_report.mat (rho 4, a_d .07, seed 1), as a height field z(x, y) on a 0.25 um grid (median per cell, linear between lines), no shape law; the overhang of the rho 4 wall is about 0.05 um. Also the fitted sphere for comparison. Sanity: no true wall used; fraction of cells with data printed. Output fig_1102_wall_surface.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.07;  MU = [char(956) 'm'];  Xw = 13;
load(fullfile(fo, 'temp_rho4_report.mat'), 'r');
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  ok = find(isfinite(r.aN(1:r.kend)));  ks = ok(r.t(ok) >= r.tsc + 0.5 & r.t(ok) <= r.rs_tend);  p = r.p(ks, :);
tx = r.thx(ks) * pi / 180;  ty = r.thy(ks) * pi / 180;  Wh = [sin(ty(:)) .* cos(tx(:)), -sin(tx(:)), cos(ty(:)) .* cos(tx(:))];  W = movmean(p - Rr * hs * Wh, 41, 1);
rq = hypot(W(:, 1), W(:, 2));  sp = W(:, 3) > 5 & rq < 8;  Am = [2 * W(sp, :), ones(sum(sp), 1)];  cf = Am \ sum(W(sp, :).^2, 2);  c0 = cf(1:3)';  rf = sqrt(cf(4) + c0 * c0');  fprintf('fitted sphere centre (%.2f, %.2f, %.2f), radius %.2f um\n', c0, rf);
dgx = 0.25;  [X, Y] = meshgrid(-Xw:dgx:Xw, -Xw:dgx:Xw);  win = abs(W(:, 1)) <= Xw + 1 & abs(W(:, 2)) <= Xw + 1;  ix = round((W(win, 1) + Xw + 1) / dgx) + 1;  iy = round((W(win, 2) + Xw + 1) / dgx) + 1;  nb = round(2 * (Xw + 1) / dgx) + 1;  Zb = accumarray([iy ix], W(win, 3), [nb nb], @median, NaN);  [Xb, Yb] = meshgrid(-(Xw + 1):dgx:Xw + 1, -(Xw + 1):dgx:Xw + 1);  hasd = isfinite(Zb);  F = scatteredInterpolant(Xb(hasd), Yb(hasd), Zb(hasd), 'linear', 'none');  Z = F(X, Y);
fprintf('cells with data %.1f%%, grid NaN %.1f%%\n', 100 * mean(hasd, 'all'), 100 * mean(isnan(Z), 'all'));
[Xs, Ys, Zs] = sphere(80);  kp = c0(3) + rf * Zs >= -0.05;  Zf = c0(3) + rf * Zs;  Zf(~kp) = NaN;
f = figure('Color', 'w', 'Position', [20 20 1800 760]);  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
a = nexttile(tl);  hold on;  surf(X, Y, Z, 'FaceColor', [0.85 0.72 0.45], 'EdgeColor', 'none', 'AmbientStrength', 0.6, 'DiffuseStrength', 0.6, 'SpecularStrength', 0);  camlight;  lighting gouraud;  title('recovered wall');
b = nexttile(tl);  hold on;  surf(c0(1) + rf * Xs, c0(2) + rf * Ys, Zf, 'FaceColor', [0.85 0.72 0.45], 'EdgeColor', 'none', 'AmbientStrength', 0.6, 'DiffuseStrength', 0.6, 'SpecularStrength', 0);  camlight;  lighting gouraud;  title(sprintf('fitted sphere, radius %.2f \\mum', rf));
for q = [a b]; axis(q, 'equal');  xlim(q, [-Xw Xw]);  ylim(q, [-Xw Xw]);  zlim(q, [-0.5 11]);  view(q, -37.5, 22);  grid(q, 'off');  box(q, 'off');  xlabel(q, ['X (' MU ')']);  ylabel(q, ['Y (' MU ')']);  zlabel(q, ['Z (' MU ')']);  set(q, 'FontSize', 15, 'LineWidth', 1.1);  set(q.Toolbar, 'Visible', 'off'); end
exportgraphics(f, fullfile(fo, 'fig_1102_wall_surface.png'), 'Resolution', 130);  close(f);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
