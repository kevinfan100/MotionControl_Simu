% STATUS: SCRATCH -- 2026-11-01 (user): rho 4 whole-bead scan at 40 um/s, seed 1, PI, tau_a .06, geometric angle Q 0.9, figures matched to Meng dissertation Fig. 45-50 (one file each), from t = 0 including the part before the scan.
%   scan_path Fig. 48 left | scan_truth path with the plain bead and floor (fillet not drawn) | scan_est Fig. 48 right | uvw Fig. 45/49 in the true local coordinate (u, v = distance along the true u, v of each step, w = h R, desired = actual + delta, plot_t2_bend_full.m) | reg Fig. 46/50 | gain_gap Fig. 47 | pre_uvw, pre_est 0-12 s.
%   Scan figures have a transparent background. Sanity: scan ends 34.7 s, no contact. Outputs fig_1101_rho4_*.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.1;  Ts = 1 / 1612;  Xe = 13;  dy = 1;  nl = 27;  rbd = 5;  rho = 4;  FS = 15;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  AB = char(257);  hatc = char(770);  th = char(952);  ka = char(954);  pr = char(8242);  MU = [char(956) 'm'];  DL = char(948);
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
o = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, ...
           'ap_q', true, 'wall', 'fillet', 'bead_r', rbd, 'fillet_rho', rho, 'bead_start', [Xe; -Xe], 'ap_floor_law', true, 'raster_keep_y', true, 'raster', [Xe dy nl], 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], ...
           'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'tag', 'mg', 'du_d', 40 * Ts / Rr, 'T', 12 + 1.4 * nl * (2 * Xe + dy) / 40 + 1, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'seeds', 1);
R = run_t3_xyz_fixed(o);  r = R.runs{1};  ok = find(isfinite(r.aN(1:r.kend)));  ke = ok(end);  ks = ok(r.t(ok) >= r.tsc & r.t(ok) <= r.rs_tend);
fprintf('contact %d, scan %.1f-%.1f s, run end %.1f s\n', r.contact, r.tsc, r.rs_tend, r.t(ke));
hg = linspace(1.0005, 40, 6000);  ag = arrayfun(@(h) 1 / local_cp(h), hg);  ah = min(max(r.aN(1:ke), ag(1)), ag(end));  hh = interp1(ag, hg, ah);
tx = r.thx(1:ke) * pi / 180;  ty = r.thy(1:ke) * pi / 180;  wh = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];  wp = r.p(1:ke, :) - (hh * Rr) .* wh;
zs = max(rho + rho * (rbd - rho) / (rbd + rho), 5);  ws = wp(ks, :);  W = ws(ws(:, 3) > zs, :);  Am = [2 * W, ones(size(W, 1), 1)];  cf = Am \ sum(W.^2, 2);  c0 = cf(1:3)';  rf = sqrt(cf(4) + c0 * c0');
fl = ws(ws(:, 3) < 0.6 & hypot(ws(:, 1), ws(:, 2)) > 11, 3);  fprintf('fitted sphere centre (%.2f, %.2f, %.2f), radius %.2f um; floor z mean %.3f, sd %.3f um\n', c0, rf, mean(fl), std(fl));
pk = ks(1:2:end);  [Xs, Ys, Zs] = sphere(80);
% ---- 3-D, transparent ----
for m = 1:3
    f = figure('Color', 'w', 'Position', [20 20 1100 800]);  a = axes(f);  hold(a, 'on');
    if m == 2; surf(a, rbd * Xs, rbd * Ys, rbd + rbd * Zs, 'FaceColor', [0.85 0.72 0.45], 'EdgeColor', 'none', 'AmbientStrength', 0.6, 'DiffuseStrength', 0.6, 'SpecularStrength', 0);  camlight(a);  lighting(a, 'gouraud');  patch(a, [-15 15 15 -15], [-15 -15 15 15], [-0.02 -0.02 -0.02 -0.02], [0.74 0.76 0.82], 'EdgeColor', 'none', 'FaceLighting', 'none'); end
    if m == 3; surf(a, c0(1) + rf * Xs, c0(2) + rf * Ys, c0(3) + rf * Zs, 'FaceColor', cT, 'EdgeColor', 'none', 'AmbientStrength', 0.6, 'SpecularStrength', 0);  camlight(a);  lighting(a, 'gouraud'); end
    plot3(a, r.p(pk, 1), r.p(pk, 2), r.p(pk, 3), 'Color', [0 0 1], 'LineWidth', 1.0);
    axis(a, 'equal');  xlim(a, [-15 15]);  ylim(a, [-15 15]);  zlim(a, [-0.2 13]);  view(a, -35, 25);  grid(a, 'off');  box(a, 'off');  xlabel(a, ['X (' MU ')']);  ylabel(a, ['Y (' MU ')']);  zlabel(a, ['Z (' MU ')']);  set(a, 'FontSize', 20, 'LineWidth', 1.2, 'Color', 'none');  set(a.Toolbar, 'Visible', 'off');
    nm = {'scan_path', 'scan_truth', 'scan_est'};  fn = fullfile(fo, ['fig_1101_rho4_' nm{m} '.png']);  exportgraphics(f, fn, 'Resolution', 170, 'BackgroundColor', 'white');  close(f);  im = imread(fn);  imwrite(im, fn, 'Alpha', uint8(255 * ~all(im >= 253, 3)));
end
% ---- true local coordinate ----
nk = ke;  E = zeros(nk, 3);  dU = zeros(nk, 2);  D = r.pd(1:nk, :) - r.p(1:nk, :);
for i = 1:nk
    c = r.th_tr(i, 1) * pi / 180;  s = r.th_tr(i, 2) * pi / 180;  V = [cos(s) 0 sin(s); 0 1 0; -sin(s) 0 cos(s)] * [1 0 0; 0 cos(c) -sin(c); 0 sin(c) cos(c)];  E(i, :) = (V' * D(i, :)')';
    if i > 1; dU(i, :) = (V(:, 1:2)' * (r.p(i, :) - r.p(i - 1, :))')'; end
end
Lc = [cumsum(dU), r.h(1:nk) * Rr];  Ld = Lc + E;  t = r.t(1:nk);  NL = {'u', 'v', 'w'};
for w = 1:2
    if w == 1; g = (1:4:nk)'; nmf = 'uvw'; else; g = find(t < 12);  g = g(1:2:end); nmf = 'pre_uvw'; end
    f = figure('Color', 'w', 'Position', [20 20 1900 950]);  tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for q = 1:3
        a = nexttile(tl, 2 * q - 1);  hold on;  box on;  h1 = plot(t(g), Ld(g, q), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t(g), Lc(g, q), 'Color', cE, 'LineWidth', 1.0);  ylabel([NL{q} '  [' MU ']']);  xlim([t(g(1)) t(g(end))]);  if q == 1; legend([h1 h2], {'Reference', 'Measurement'}, 'Location', 'northoutside', 'Orientation', 'horizontal'); end;  if q == 3; xlabel('Time (sec)'); end;  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
        a = nexttile(tl, 2 * q);  hold on;  box on;  plot(t(g), E(g, q), 'Color', cE, 'LineWidth', 0.7);  ylim([-0.3 0.3]);  ylabel([DL NL{q} '  [' MU ']']);  xlim([t(g(1)) t(g(end))]);  if q == 3; xlabel('Time (sec)'); end;  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
    end
    exportgraphics(f, fullfile(fo, ['fig_1101_rho4_' nmf '.png']), 'Resolution', 130);  close(f);
end
fprintf('tracking error rms, scan: du %.1f, dv %.1f, dw %.1f nm\n', 1e3 * sqrt(mean(E(ks, :).^2)));
gg = (1:3:nk)';
f = figure('Color', 'w', 'Position', [20 20 1500 700]);  a = axes(f);  hold(a, 'on');  box(a, 'on');  h1 = plot(a, t(gg), r.aN_tr(gg), 'Color', cT, 'LineWidth', 1.3);  h2 = plot(a, t(gg), r.aN(gg), 'Color', cE, 'LineWidth', 0.8);  h3 = yline(a, ad, 'k--', 'LineWidth', 1.6);  ylim(a, [0 1]);  xlim(a, [0 t(end)]);  ylabel(a, [AB '_{33}']);  xlabel(a, 'Time (sec)');  legend(a, [h1 h2 h3], {'true', 'estimate', [AB '_d = 0.1']}, 'Location', 'northoutside', 'Orientation', 'horizontal');  set(a, 'FontSize', FS + 2, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
exportgraphics(f, fullfile(fo, 'fig_1101_rho4_reg.png'), 'Resolution', 130);  close(f);
f = figure('Color', 'w', 'Position', [20 20 1900 800]);  tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'columnmajor');
a1 = nexttile(tl);  hold on;  box on;  h1 = plot(t(gg), r.aT_tr(gg), 'Color', cT, 'LineWidth', 1.3);  h2 = plot(t(gg), r.aT(gg), 'Color', cE, 'LineWidth', 0.8);  ylim([0.2 1]);  ylabel([AB '_{11}']);  legend([h1 h2], {'true', 'estimate'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
a2 = nexttile(tl);  hold on;  box on;  plot(t(gg), r.aN_tr(gg), 'Color', cT, 'LineWidth', 1.3);  plot(t(gg), r.aN(gg), 'Color', cE, 'LineWidth', 0.8);  ylim([0 1]);  ylabel([AB '_{33}']);  xlabel('Time (sec)');
a3 = nexttile(tl, [2 1]);  hold on;  box on;  h1 = plot(t(gg), r.h(gg) * Rr, 'Color', cT, 'LineWidth', 1.3);  h2 = plot(t(gg), hh(gg) * Rr, 'Color', cE, 'LineWidth', 0.8);  yline(Rr, 'k-', 'LineWidth', 1.2);  set(a3, 'YScale', 'log');  ylim([2 20]);  ylabel(['gap  [' MU ']']);  xlabel('Time (sec)');  legend([h1 h2], {'true', 'from the estimate'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
for a = [a1 a2 a3]; xlim(a, [0 t(end)]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
exportgraphics(f, fullfile(fo, 'fig_1101_rho4_gain_gap.png'), 'Resolution', 130);  close(f);
g = find(t < 12);  g = g(1:2:end);  [~, ~] = deal(0);  sN = arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, r.h(g));  sT = arrayfun(@(h) (1 / local_cq(max(h, 1.0003) + 1e-4) - 1 / local_cq(max(h, 1.0003) - 1e-4)) / 2e-4, r.h(g));
f = figure('Color', 'w', 'Position', [20 20 1900 1150]);  tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'columnmajor');
b1 = nexttile(tl);  hold on;  box on;  h1 = plot(t(g), r.aN_tr(g), 'Color', cT, 'LineWidth', 1.3);  h2 = plot(t(g), r.aN(g), 'Color', cE, 'LineWidth', 0.8);  h3 = yline(ad, 'k--', 'LineWidth', 1.4);  ylim([0 1]);  ylabel([AB '_{33}']);  legend([h1 h2 h3], {'true', 'estimate', [AB '_d = 0.1']}, 'Location', 'northoutside', 'Orientation', 'horizontal');
b2 = nexttile(tl);  hold on;  box on;  plot(t(g), r.aT_tr(g), 'Color', cT, 'LineWidth', 1.3);  plot(t(g), r.aT(g), 'Color', cE, 'LineWidth', 0.8);  ylim([0.3 1]);  ylabel([AB '_{11}']);
b3 = nexttile(tl);  hold on;  box on;  plot(t(g), r.h(g) * Rr, 'Color', cT, 'LineWidth', 1.3);  plot(t(g), hh(g) * Rr, 'Color', cE, 'LineWidth', 0.8);  yline(hs * Rr, 'k--');  ylim([0 16]);  ylabel(['gap  [' MU ']']);  xlabel('Time (sec)');
b4 = nexttile(tl);  hold on;  box on;  plot(t(g), sN, 'Color', cT, 'LineWidth', 1.3);  plot(t(g), r.apN(g), 'Color', cE, 'LineWidth', 0.8);  ylim([-0.5 2]);  ylabel([AB pr '_{33}']);
b5 = nexttile(tl);  hold on;  box on;  plot(t(g), sT, 'Color', cT, 'LineWidth', 1.3);  plot(t(g), r.apT(g), 'Color', cE, 'LineWidth', 0.8);  ylim([-0.5 2]);  ylabel([AB pr '_{11}']);
b6 = nexttile(tl);  hold on;  box on;  plot(t(g), r.th_tr(g, 2), 'Color', cT, 'LineWidth', 1.3);  plot(t(g), r.thy(g), 'Color', cE, 'LineWidth', 0.8);  plot(t(g), r.thx(g), 'Color', [0 0.55 0.2], 'LineWidth', 0.8);  ylim([-15 15]);  ylabel([th '_y (blue), ' th '_x (green)  [deg]']);  xlabel('Time (sec)');
for a = [b1 b2 b3 b4 b5 b6]; xlim(a, [0 12]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
exportgraphics(f, fullfile(fo, 'fig_1101_rho4_pre_est.png'), 'Resolution', 130);  close(f);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
function c = local_cq(h)
    c = calc_correction_functions(h);
end
