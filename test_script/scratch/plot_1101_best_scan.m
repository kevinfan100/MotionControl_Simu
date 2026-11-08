% STATUS: SCRATCH -- 2026-11-01 (user): best runs so far on the whole bead at 40 um/s: rho 10 and rho 4, seed 1, PI (tau_i .5), tau_a .06, geometric angle Q 0.9 (opts.q_geo). Scan figure in the Meng Fig. 48 style with a transparent background: measured motion | path over the true solid wall | path with the bead fitted from the estimates only. Signal figure: a_33, a_11, slopes, angles, kappa-hat, gap.
%   Sanity: ends as run_1101_bead_tau.m (33.3, 34.7 s), no contact. Outputs fig_1101_best_{scan,signal}_rho{10,4}.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.1;  Ts = 1 / 1612;  Xe = 13;  dy = 1;  nl = 27;  rbd = 5;  FS = 14;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  AB = char(257);  hatc = char(770);  th = char(952);  ka = char(954);  pr = char(8242);  rv = [10 4];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, ...
              'ap_q', true, 'wall', 'fillet', 'bead_r', rbd, 'fillet_rho', 10, 'bead_start', [Xe; -Xe], 'ap_floor_law', true, 'raster_keep_y', true, 'raster', [Xe dy nl], 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], ...
              'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'tag', 'bs', 'du_d', 40 * Ts / Rr, 'T', 12 + 1.4 * nl * (2 * Xe + dy) / 40 + 1, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'seeds', 1);
rs = cell(2, 1);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor j = 1:2
    o = base;  o.fillet_rho = rv(j);  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{j} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'te', r.rs_tend, 'p', r.p, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'aT', r.aT, 'aT_tr', r.aT_tr, 'apN', r.apN, 'apT', r.apT, 'kap', r.kap(:), 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr);
end
hg = linspace(1.0005, 40, 6000);  ag = arrayfun(@(h) 1 / local_cp(h), hg);
for j = 1:2
    r = rs{j};  rho = rv(j);  ok = find(isfinite(r.aN(1:r.kend)));  ks = ok(r.t(ok) >= r.tsc & r.t(ok) <= r.te);
    fprintf('rho %d: contact %d, scan %.1f-%.1f s, run end %.1f s, gap %.2f-%.2f um, mean|a33-a_d| %.3f, est-true %+.4f\n', rho, r.contact, r.tsc, r.te, r.t(ok(end)), min(r.h(ks)) * Rr, max(r.h(ks)) * Rr, mean(abs(r.aN_tr(ks) - ad)), mean(r.aN(ks) - r.aN_tr(ks)));
    ah = min(max(r.aN(ks), ag(1)), ag(end));  hh = interp1(ag, hg, ah);  tx = r.thx(ks) * pi / 180;  ty = r.thy(ks) * pi / 180;  wh = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];  wp = r.p(ks, :) - (hh * Rr) .* wh;
    zs = rho + rho * (rbd - rho) / (rbd + rho);  sel = wp(:, 3) > max(zs, 5);  W = wp(sel, :);  Am = [2 * W, ones(size(W, 1), 1)];  cf = Am \ sum(W.^2, 2);  c0 = cf(1:3)';  rf = sqrt(cf(4) + c0 * c0');
    fprintf('   fitted bead from estimates: centre (%.2f, %.2f, %.2f) um, radius %.2f um, %d points\n', c0, rf, sum(sel));
    [PR, PZ] = local_profile(rbd, rho, 16);  ph = linspace(2 * pi, 0, 180);  XW = PR(:) * cos(ph);  YW = PR(:) * sin(ph);  ZW = repmat(PZ(:), 1, numel(ph));
    f = figure('Color', 'w', 'Position', [20 20 2300 800]);  tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');  pk = ks(1:2:end);
    a1 = nexttile(tl);  hold on;  plot3(r.p(pk, 1), r.p(pk, 2), r.p(pk, 3), 'Color', cE, 'LineWidth', 0.8);  title('measured scanning motion of the probe');
    a2 = nexttile(tl);  hold on;  surf(XW, YW, ZW, 'FaceColor', [0.72 0.72 0.76], 'EdgeColor', 'none', 'AmbientStrength', 0.55, 'DiffuseStrength', 0.5, 'SpecularStrength', 0);  plot3(r.p(pk, 1), r.p(pk, 2), r.p(pk, 3), 'Color', cE, 'LineWidth', 0.8);  camlight;  lighting gouraud;  title(sprintf('path (blue) over the true wall, \\rho = %d \\mum', rho));
    a3 = nexttile(tl);  hold on;  [Xs, Ys, Zs] = sphere(60);  kup = Zs >= (max(zs, 5) - c0(3)) / rf - 0.05;  Xs(~kup) = NaN;  surf(c0(1) + rf * Xs, c0(2) + rf * Ys, c0(3) + rf * Zs, 'FaceColor', cT, 'EdgeColor', 'none', 'FaceAlpha', 0.7);  plot3(r.p(pk, 1), r.p(pk, 2), r.p(pk, 3), 'Color', cE, 'LineWidth', 0.8);  camlight;  lighting gouraud;  title(sprintf('estimated bead (red): centre z %.2f \\mum, radius %.2f \\mum', c0(3), rf));
    for a = [a1 a2 a3]; box(a, 'on');  grid(a, 'on');  axis(a, 'equal');  xlim(a, [-15 15]);  ylim(a, [-15 15]);  zlim(a, [0 14]);  view(a, -35, 30);  xlabel(a, 'x  [\mum]');  ylabel(a, 'y  [\mum]');  zlabel(a, 'z  [\mum]');  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2, 'Color', 'none');  set(a.Toolbar, 'Visible', 'off'); end
    fn = fullfile(fo, sprintf('fig_1101_best_scan_rho%d.png', rho));  exportgraphics(f, fn, 'Resolution', 140, 'BackgroundColor', 'white');  close(f);
    im = imread(fn);  mask = all(im >= 253, 3);  imwrite(im, fn, 'Alpha', uint8(255 * ~mask));
    [~, sN, sT] = deal(0, local_slope(r.h(ok), 2), local_slope(r.h(ok), 1));  k = ok(r.t(ok) >= r.tsc - 0.5);  k = k(1:4:end);  t = r.t(k);  [~, ik] = ismember(k, ok);
    f = figure('Color', 'w', 'Position', [20 20 1950 1250]);  tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'columnmajor');
    b1 = nexttile(tl);  hold on;  h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 1.3);  h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 0.8);  h3 = yline(ad, 'k--', 'LineWidth', 1.4);  ylim([0 0.3]);  ylabel([AB '_{33}']);  legend([h1 h2 h3], {'true', sprintf('estimate, \\rho = %d, seed 1', rho), [AB '_d = 0.1']}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    b2 = nexttile(tl);  hold on;  plot(t, r.aT_tr(k), 'Color', cT, 'LineWidth', 1.3);  plot(t, r.aT(k), 'Color', cE, 'LineWidth', 0.8);  ylim([0.25 0.65]);  ylabel([AB '_{11}']);
    b3 = nexttile(tl);  hold on;  plot(t, sN(ik), 'Color', cT, 'LineWidth', 1.3);  plot(t, r.apN(k), 'Color', cE, 'LineWidth', 0.8);  ylim([-3 4]);  ylabel([AB pr '_{33}']);
    b4 = nexttile(tl);  hold on;  plot(t, sT(ik), 'Color', cT, 'LineWidth', 1.3);  plot(t, r.apT(k), 'Color', cE, 'LineWidth', 0.8);  ylim([-1 2]);  ylabel([AB pr '_{11}']);  xlabel('Time (sec)');
    b5 = nexttile(tl);  hold on;  plot(t, r.th_tr(k, 2), 'Color', cT, 'LineWidth', 1.3);  plot(t, r.thy(k), 'Color', cE, 'LineWidth', 0.8);  ylim([-110 110]);  ylabel([th '_y  [deg]']);
    b6 = nexttile(tl);  hold on;  plot(t, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 1.3);  plot(t, r.thx(k), 'Color', cE, 'LineWidth', 0.8);  ylim([-90 90]);  ylabel([th '_x  [deg]']);
    b7 = nexttile(tl);  hold on;  plot(t, r.kap(k), 'Color', cE, 'LineWidth', 0.8);  yline(0, 'k:');  ylim([-1.5 1.5]);  ylabel([ka hatc '  [1/R]']);
    b8 = nexttile(tl);  hold on;  plot(t, r.h(k) * Rr, 'Color', cT, 'LineWidth', 1.3);  yline(hs * Rr, 'k--', 'LineWidth', 1.2);  yline(Rr, 'k-', 'LineWidth', 1.2);  ylim([2 3.5]);  ylabel('gap  [\mum]');  xlabel('Time (sec)');
    for a = [b1 b2 b3 b4 b5 b6 b7 b8]; box(a, 'on');  xlim(a, [t(1) t(end)]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off'); end
    exportgraphics(f, fullfile(fo, sprintf('fig_1101_best_signal_rho%d.png', rho)), 'Resolution', 130);  close(f);
end

function [PR, PZ] = local_profile(rb, rho, Rm)
    Rt = 2 * sqrt(rb * rho);  a_t = atan2(Rt, rho - rb);  b = linspace(0, a_t, 80);  c = linspace(a_t, 0, 60);
    PR = [rb * sin(b), Rt - rho * sin(c), Rm];  PZ = [rb + rb * cos(b), rho - rho * cos(c), 0];
end
function s = local_slope(hb, w)
    f = @(h) local_c(h, w);  s = arrayfun(@(h) (1 / f(max(h, 1.0003) + 1e-4) - 1 / f(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
end
function c = local_c(h, w)
    [a, b] = calc_correction_functions(h);  c = a;  if w == 2; c = b; end
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
