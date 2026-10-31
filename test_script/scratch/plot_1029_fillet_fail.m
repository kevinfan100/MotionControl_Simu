% STATUS: SCRATCH -- 2026-10-29 (user): first whole-bead raster on the 'fillet' wall (run_1029_fillet_raster.m, temp_fillet_raster.mat):
%   all three arms touch the wall in the concave fillet. Figure: top view of the probe path (bead outline r = 5 um, fillet ring r = 10.95 um,
%   contact point x), the line where it touched as a side view (z vs x with the wall cross-section at that y), and h, a_33 (true / estimate),
%   theta (true / estimate) over the last 3 s. Output test_results/temp_3d_wsonly/fig_1029_fillet_fail.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
sfx = '';  if exist('sfx_in', 'var'); sfx = sfx_in; end;  S = load(fullfile(fo, ['temp_fillet_raster' sfx '.mat']));  rs = S.rs;  arm = S.arm;  Rr = S.Rr;  ad = S.ad;
FS = 13;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];  hatc = char(770);  AB = char(257);  th = char(952);
rbd = 5;  rho = 6;  Rt = 2 * sqrt(rbd * rho);  tg = linspace(0, 2 * pi, 300);
f = figure('Color', 'w', 'Position', [20 20 1950 1500]);  tl = tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for q = 1:3
    r = rs{q};  k = (1:r.kend)';  ke = r.kend;  pe = r.p(ke, :);
    a1 = nexttile(tl, q);  hold on;  box on;  plot(rbd * cos(tg), rbd * sin(tg), 'Color', cGr, 'LineWidth', 2.5);  plot(Rt * cos(tg), Rt * sin(tg), '--', 'Color', cGr, 'LineWidth', 1.5);
    ks = k(r.t(k) >= r.tsc);  plot(r.p(ks, 1), r.p(ks, 2), 'Color', cE, 'LineWidth', 1);  plot(pe(1), pe(2), 'rx', 'MarkerSize', 16, 'LineWidth', 3);
    axis equal;  xlim([-15 15]);  ylim([-15 15]);  xlabel('x  [\mum]');  ylabel('y  [\mum]');  title(sprintf('%s: top view, contact at %.1f s', arm{q}, r.t(ke)));
    if q == 1; legend({'bead r = 5 \mum', 'fillet edge on the glass', 'p (scan)', 'contact'}, 'Location', 'southoutside', 'Orientation', 'horizontal'); end
    a2 = nexttile(tl, 3 + q);  hold on;  box on;  yq = pe(2);  xs = linspace(-15, 15, 600);  zw = arrayfun(@(x) local_wall_z(x, yq, rbd, rho, Rt), xs);
    fill([xs fliplr(xs)], [zw zeros(size(xs)) - 1], [0.88 0.88 0.88], 'EdgeColor', 'none');  plot(xs, zw, 'Color', cGr, 'LineWidth', 2.5);
    kl = ks(abs(r.p(ks, 2) - yq) < 0.15);  plot(r.p(kl, 1), r.p(kl, 3), '.', 'Color', cE, 'MarkerSize', 4);  plot(pe(1), pe(3), 'rx', 'MarkerSize', 16, 'LineWidth', 3);
    axis equal;  xlim([-15 15]);  ylim([-1 12]);  xlabel('x  [\mum]');  ylabel('z  [\mum]');  title(sprintf('side view of the line y = %.1f \\mum', yq));
    kz = k(r.t(k) >= r.t(ke) - 3);  t = r.t(kz) - r.t(ke);
    a3 = nexttile(tl, 6 + q);  hold on;  box on;  h1 = plot(t, r.aN_tr(kz), 'Color', cT, 'LineWidth', 2);  h2 = plot(t, r.aN(kz), 'Color', cE, 'LineWidth', 1.2);  h3 = yline(ad, 'k--', 'LineWidth', 1.4);
    ylim([-0.1 0.3]);  ylabel(sprintf('%s_{33}', AB));  if q == 1; legend([h1 h2 h3], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s_d', AB)}, 'Location', 'northwest'); end
    a4 = nexttile(tl, 9 + q);  hold on;  box on;  h1 = plot(t, r.th_tr(kz, 2), 'Color', cT, 'LineWidth', 2);  plot(t, r.th_tr(kz, 1), 'Color', cT, 'LineWidth', 2);
    h2 = plot(t, r.thy(kz), 'Color', cE, 'LineWidth', 1.2);  h3 = plot(t, r.thx(kz), 'Color', cG, 'LineWidth', 1.2);  ylabel(sprintf('%s  [\\circ]', th));  xlabel('time before contact  (sec)');
    if q == 1; legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northwest'); end
    for a = [a3 a4]; xlim(a, [-3 0]); end
    for a = [a1 a2 a3 a4]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, ['fig_1029_fillet_fail' sfx '.png']);  exportgraphics(f, fn, 'Resolution', 115);  close(f);  system(['open ' fn]);

function z = local_wall_z(x, y, rbd, rho, Rt)                 % wall height at (x, y): bead top, fillet, or glass
    r = hypot(x, y);  rs = rho * rbd / (rbd + rho) * 0 + Rt * rbd / (rbd + rho);   % radius of the bead / fillet tangency
    if r <= rs; z = rbd + sqrt(max(rbd^2 - r^2, 0));
    elseif r < Rt; z = rho - sqrt(max(rho^2 - (Rt - r)^2, 0));
    else; z = 0; end
end
