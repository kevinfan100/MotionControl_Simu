% STATUS: SCRATCH -- 2026-10-30 (user): the whole-bead raster at 40 um/s (fillet 10 um, kappa allowed to change, a_d 0.1; arm E of
%   run_1029_fillet_opts.m, temp_fillet_opts_kq40.mat). Top: 3D probe path until the end of each run (seeds 1-3) with the end point,
%   bead (r 5) and glass drawn; bottom: last 2 s of seed 1 and 3 -- a_33 true / estimate and theta true / estimate.
%   Output test_results/temp_3d_wsonly/fig_1030_bead40.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
sfx40 = '_kq40';  if exist('sfx40_in', 'var'); sfx40 = sfx40_in; end;  S = load(fullfile(fo, ['temp_fillet_opts' sfx40 '.mat']));  rs = S.rs;  Rr = S.Rr;  ad = S.ad;  FS = 14;
cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  hatc = char(770);  AB = char(257);  th = char(952);  [Xs, Ys, Zs] = sphere(50);
f = figure('Color', 'w', 'Position', [20 20 2000 1300]);  tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for id = 1:3
    r = rs{1, id};  kk = (1:r.kend)';  ok = kk(r.t(kk) >= r.tsc & all(isfinite(r.p(kk, :)), 2));  ke = ok(end);
    ax = nexttile(tl, id);  hold on;  box on;  grid on;
    surf(5 * Xs, 5 * Ys, 5 * Zs + 5, 'FaceColor', [0.75 0.75 0.75], 'EdgeColor', 'none', 'FaceAlpha', 0.35);
    patch([-16 16 16 -16], [-16 -16 16 16], [0 0 0 0], [0.85 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.35);
    plot3(r.p(ok, 1), r.p(ok, 2), r.p(ok, 3), 'Color', cE, 'LineWidth', 0.9);  plot3(r.p(ke, 1), r.p(ke, 2), r.p(ke, 3), 'rx', 'MarkerSize', 18, 'LineWidth', 3);
    ttx = 'contact';  if ~r.contact; ttx = 'estimator diverged'; end
    axis equal;  xlim([-16 16]);  ylim([-16 16]);  zlim([-1 14]);  view(-35, 30);  xlabel('x  [\mum]');  ylabel('y  [\mum]');  zlabel('z  [\mum]');
    title(sprintf('40 um/s, seed %d: %s at %.1f s', id, ttx, r.t(ke)));  set(ax, 'FontSize', FS, 'FontWeight', 'bold');  set(ax.Toolbar, 'Visible', 'off');
end
for j = 1:2
    id = 2 * j - 1;  r = rs{1, id};  kk = (1:r.kend)';  ke = r.kend;  kz = kk(r.t(kk) >= r.t(ke) - 2);  t = r.t(kz) - r.t(ke);
    a1 = nexttile(tl, 3 + j);  hold on;  box on;
    yyaxis left;  plot(t, r.aN_tr(kz), '-', 'Color', cT, 'LineWidth', 1.8);  plot(t, r.aN(kz), '-', 'Color', cE, 'LineWidth', 1);  yline(ad, 'k--');  ylim([-0.1 0.4]);  ylabel(sprintf('%s_{33}', AB));  set(a1, 'YColor', 'k');
    yyaxis right;  plot(t, r.th_tr(kz, 2), '--', 'Color', cT, 'LineWidth', 1.8);  plot(t, r.thy(kz), '--', 'Color', cE, 'LineWidth', 1);  ylabel(sprintf('%s_y  [\\circ]  (dashed)', th));  set(a1, 'YColor', 'k');
    xlabel('time before the end  (sec)');  title(sprintf('seed %d, last 2 s (red = true, blue = estimate)', id));  set(a1, 'FontSize', FS, 'FontWeight', 'bold');  set(a1.Toolbar, 'Visible', 'off');
end
fn = fullfile(fo, ['fig_1030_bead40' strrep(sfx40, '_kq40', '') '.png']);  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);
