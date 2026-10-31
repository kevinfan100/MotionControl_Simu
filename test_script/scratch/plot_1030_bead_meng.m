% STATUS: SCRATCH -- 2026-10-30 (user): whole-bead scan result in the style of Meng's dissertation Fig. 48 (6.2.1): left = measured scanning
%   motion of the probe in the measurement coordinate (3D); right = probe path (blue) and the bead shape estimated from the scan (red sphere,
%   least-squares fit to the wall points reconstructed from the estimates only, p - h-hat R w-hat, keeping the points above the fillet
%   tangency). Second figure (Meng Fig. 46/47 style): regulation of a-hat_33 to a_d and the estimated gap, true values for reference.
%   Data temp_fillet_opts_kq.mat (run_1029_fillet_opts.m, arm D: fillet 10 um, kappa allowed to change, a_d 0.1, 10 um/s), seed 1.
%   Outputs fig_1030_bead_meng_3d.png, fig_1030_bead_meng_time.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
addpath(genpath(fullfile(root, 'model')));
sd = 1;  if exist('seed_in', 'var'); sd = seed_in; end
msfx = '_kq';  if exist('mat_in', 'var'); msfx = mat_in; end;  S = load(fullfile(fo, ['temp_fillet_opts' msfx '.mat']));  r = S.rs{1, sd};  Rr = S.Rr;  ad = S.ad;  FS = 15;  cE = [0 0.25 0.85];  cT = [0.85 0 0];  hatc = char(770);  AB = char(257);
rbd = 5;  rho = 10;  zs = rho + rho * (rbd - rho) / (rbd + rho);         % height of the bead / fillet tangency
hg = linspace(1.0005, 40, 6000);  ag = arrayfun(@(h) 1 / local_cp(h), hg);
kk = (1:r.kend)';  te = r.t(r.kend);  fprintf('seed %d: contact %d, end %.1f s\n', sd, r.contact, r.t(r.kend));  if ~isnan(r.rs_tend); te = r.rs_tend; end;  ks = kk(r.t(kk) >= r.tsc & r.t(kk) <= te & isfinite(r.aN(kk)));
ah = min(max(r.aN(ks), ag(1)), ag(end));  hh = interp1(ag, hg, ah);  tx = r.thx(ks) * pi / 180;  ty = r.thy(ks) * pi / 180;
wh = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];  wp = r.p(ks, :) - (hh * Rr) .* wh;      % wall points from the estimates
sel = wp(:, 3) > zs;  W = wp(sel, :);  Am = [2 * W, ones(size(W, 1), 1)];  bm = sum(W.^2, 2);  cf = Am \ bm;  c0 = cf(1:3)';  rf = sqrt(cf(4) + c0 * c0');
fprintf('seed %d: raster %.1f-%.1f s, %d wall points on the bead cap | fitted centre (%.2f, %.2f, %.2f) um (true 0, 0, 5) | radius %.2f um (true 5) | glass points z mean %.3f um\n', ...
        sd, r.tsc, te, sum(sel), c0, rf, mean(wp(wp(:, 3) < 0.5 & hypot(wp(:, 1), wp(:, 2)) > 14, 3)));
f = figure('Color', 'w', 'Position', [20 20 2000 850]);  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = nexttile(tl);  hold on;  box on;  grid on;  plot3(r.p(ks, 1), r.p(ks, 2), r.p(ks, 3), 'Color', cE, 'LineWidth', 0.8);
axis equal;  xlim([-15 15]);  ylim([-15 15]);  zlim([0 14]);  view(-35, 30);  xlabel('x  [\mum]');  ylabel('y  [\mum]');  zlabel('z  [\mum]');  title('measured scanning motion of the probe');
ax2 = nexttile(tl);  hold on;  box on;  grid on;  [Xs, Ys, Zs] = sphere(50);  kup = Zs >= (zs - c0(3)) / rf - 0.05;  Xs(~kup) = NaN;
surf(c0(1) + rf * Xs, c0(2) + rf * Ys, c0(3) + rf * Zs, 'FaceColor', cT, 'EdgeColor', 'none', 'FaceAlpha', 0.6);  plot3(r.p(ks, 1), r.p(ks, 2), r.p(ks, 3), 'Color', cE, 'LineWidth', 0.8);
camlight;  lighting gouraud;  axis equal;  xlim([-15 15]);  ylim([-15 15]);  zlim([0 14]);  view(-35, 30);  xlabel('x  [\mum]');  ylabel('y  [\mum]');  zlabel('z  [\mum]');
title(sprintf('probe path (blue) and estimated bead (red): centre z %.2f \\mum, radius %.2f \\mum', c0(3), rf));
for a = [ax1 ax2]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off'); end
fn = fullfile(fo, ['fig_1030_bead_meng_3d' msfx '.png']);  exportgraphics(f, fn, 'Resolution', 130);  close(f);
f = figure('Color', 'w', 'Position', [20 20 1900 800]);  tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');  t = r.t(ks);
a1 = nexttile(tl);  hold on;  box on;  h1 = plot(t, r.aN_tr(ks), 'Color', cT, 'LineWidth', 1.6);  h2 = plot(t, r.aN(ks), 'Color', cE, 'LineWidth', 0.9);  h3 = yline(ad, 'k--', 'LineWidth', 1.5);
ylim([0 0.3]);  ylabel(sprintf('%s_{33}', AB));  legend([h1 h2 h3], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s_d = %g', AB, ad)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
a2 = nexttile(tl);  hold on;  box on;  h1 = plot(t, r.h(ks) * Rr, 'Color', cT, 'LineWidth', 1.6);  h2 = plot(t, hh * Rr, 'Color', cE, 'LineWidth', 0.9);
ylim([2 3.5]);  ylabel('gap  [\mum]');  xlabel('Time (sec)');  legend([h1 h2], {'gap (true)', 'gap from the estimate'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
for a = [a1 a2]; xlim(a, [t(1) t(end)]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off'); end
fn2 = fullfile(fo, ['fig_1030_bead_meng_time' msfx '.png']);  exportgraphics(f, fn2, 'Resolution', 130);  close(f);  system(['open ' fn ' ' fn2]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
