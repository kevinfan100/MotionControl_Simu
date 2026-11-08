% 2026-11-04 (user): same figures for the square raster run (run_1104_square_circle_seeds.m, seed 1).
% STATUS: SCRATCH -- 2026-11-04 (user): new flow -- the 10 s hold at 15 um replaced by a horizontal circle 3 um, 8 Hz, 2 s ramp, 5 s (dither in the tangent plane of V-hat, back at the start point at 5 s), then descent 1 s, scan right at the bottom; everything else as plot_1103_flow.m (seed 1, rho 4, a_d .07, 40 um/s, fixed 36 um raster).
%   Same figures as plot_1103_flow.m; 3-D also saved as .fig. Cached in temp_1104_square_circle_s1.mat.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  Ts = 1 / 1612;  nl = 27;  L0 = 36;  ad = 0.07;  rbd = 5;  rho = 4;  FS = 14;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  BL = [0 0 1];  AB = char(257);  MU = [char(956) 'm'];  DL = char(948);  hatc = char(770);  th = char(952);  ka = char(954);  pr = char(8242);
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
o = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 5, 'dither', [3 8 2], 'dither_win', [0 5], 'dither_frame0', true, 'tau_a', 0.06, 'a_d', ad, ...
           'ap_q', true, 'wall', 'fillet', 'bead_r', rbd, 'fillet_rho', rho, 'bead_start', [18; -13], 'ap_floor_law', true, 'raster_local', [L0 1 nl 0.05], 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], ...
           'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 0], 'tag', 'fc', 'du_d', 40 * Ts / Rr, 'T', 6 + 1.2 * nl * L0 / 40 + 1, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'seeds', 1);
cf = fullfile(fo, 'temp_1104_square_circle_s1.mat');
if isfile(cf); load(cf, 'r'); else; R = run_t3_xyz_fixed(o);  r = R.runs{1};  save(cf, 'r', '-v7.3'); end
ok = find(isfinite(r.aN(1:r.kend)));  ke = ok(end);  ks = ok(r.t(ok) >= r.tsc & r.t(ok) <= r.rs_tend);  kp = ok(r.t(ok) < r.tsc);
fprintf('contact %d, scan %.2f-%.2f s, run end %.2f s, min gap %.3f um\n', r.contact, r.tsc, r.rs_tend, r.t(ke), min(r.h(ks)) * Rr);
hg = linspace(1.0005, 40, 6000);  ag = arrayfun(@(h) 1 / local_cp(h), hg);  hh = interp1(ag, hg, min(max(r.aN(1:ke), ag(1)), ag(end)));  hh = hh(:);
tx = r.thx(1:ke) * pi / 180;  ty = r.thy(1:ke) * pi / 180;  tx = tx(:);  ty = ty(:);
Uh = [cos(ty), zeros(ke, 1), -sin(ty)];  Vh = [sin(ty) .* sin(tx), cos(tx), cos(ty) .* sin(tx)];  Wh = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
D = r.pd(1:ke, :) - r.p(1:ke, :);  dp = [zeros(1, 3); diff(r.p(1:ke, :), 1, 1)];  E = [sum(Uh .* D, 2), sum(Vh .* D, 2), sum(Wh .* D, 2)];  dL = [sum(Uh .* dp, 2), sum(Vh .* dp, 2), sum(Wh .* dp, 2)];  dL(~isfinite(dL)) = 0;  Lc = cumsum(dL, 1);  Lc = Lc - Lc(1, :);  Ld = Lc + E;  t = r.t(:);
pp = r.p(ks, :);  sc = movmean(dL(ks, 2), 21);  isv = sc > 0.012;  [~, nv] = bwlabel(isv);  stepv = conv(double(isv), ones(61, 1), 'same') > 0;  fprintf('v steps %d\n', nv);
pb = pp;  pb(~(abs(pp(:, 1)) <= 12 & abs(pp(:, 2)) <= 8 & ~stepv), :) = NaN;
if any(findall(0, 'Type', 'figure') == 30); close(30); end;  f = figure(30);  set(f, 'Color', 'w', 'Position', [20 20 1100 760], 'Visible', 'on', 'Name', 'current flow, scan, seed 1');  a = axes(f);  hold(a, 'on');  plot3(a, pb(:, 1), pb(:, 2), pb(:, 3), 'Color', BL, 'LineWidth', 1.3);  axis(a, 'equal');  xlim(a, [-13 13]);  ylim(a, [-14 14]);  zlim(a, [0 14]);  view(a, -37.5, 22);  grid(a, 'off');  box(a, 'off');  xlabel(a, ['X (' MU ')']);  ylabel(a, ['Y (' MU ')']);  zlabel(a, ['Z (' MU ')']);  set(a, 'FontSize', 20, 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off');
rotate3d(a, 'on');  savefig(f, fullfile(fo, 'fig_1104_square_circle_3d.fig'));  exportgraphics(f, fullfile(fo, 'fig_1104_square_circle_3d.png'), 'Resolution', 150);
NL = {'u', 'v', 'w'};  ka = ok(r.t(ok) <= r.rs_tend);  g = ka(1:4:end);  hR = r.h(1:ke) * Rr;  hR = hR(:);  Sw = Lc(:, 3);  Lc(:, 3) = hR - hR(1);  Ld(:, 3) = Lc(:, 3) + E(:, 3);  wd = hs * Rr - hR(1);  fprintf('w at a_d %.3f um, scan w %.3f..%.3f um, sum w-hat dp range %.2f um\n', wd, min(Lc(ks, 3)), max(Lc(ks, 3)), max(Sw(ks)) - min(Sw(ks)));
f = figure('Visible', 'off', 'Color', 'w', 'Position', [20 20 1800 1000]);  tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'rowmajor');
for q = 1:3
    a = nexttile(tl);  hold on;  box on;  h1 = plot(t(g), Ld(g, q), 'Color', cT, 'LineWidth', 2.0);  h2 = plot(t(g), Lc(g, q), 'Color', cE, 'LineWidth', 0.9);  ylabel([NL{q} '  [' MU ']']);  if q == 1; legend([h1 h2], {'Reference', 'Measurement'}, 'Location', 'northoutside', 'Orientation', 'horizontal'); end;  if q == 3; h3 = yline(wd, 'k--', 'LineWidth', 1.4);  ylim([-15 2]);  legend(h3, sprintf('%s_d = %.2f', AB, ad), 'Location', 'northeast');  xlabel('Time (sec)'); end;  local_style(a, t(g), FS);
    a = nexttile(tl);  hold on;  box on;  plot(t(g), E(g, q), 'Color', cE, 'LineWidth', 0.6);  ylim([-0.3 0.3]);  ylabel([DL NL{q} '  [' MU ']']);  if q == 3; xlabel('Time (sec)'); end;  local_style(a, t(g), FS);
end
exportgraphics(f, fullfile(fo, 'fig_1104_square_circle_motion.png'), 'Resolution', 120);  close(f);
f = figure('Visible', 'off', 'Color', 'w', 'Position', [20 20 1500 700]);  tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
a = nexttile(tl);  hold on;  h1 = plot(t(g), r.aN(g), 'Color', cE, 'LineWidth', 0.8);  h2 = yline(ad, 'k--', 'LineWidth', 1.4);  ylim([0 1]);  ylabel([AB '_{33}']);  legend([h1 h2], {'estimate', sprintf('%s_d = %.2f', AB, ad)}, 'Location', 'northoutside', 'Orientation', 'horizontal');  local_style(a, t(g), FS);
a = nexttile(tl);  hold on;  plot(t(g), r.aT(g), 'Color', cE, 'LineWidth', 0.8);  ylim([0.2 1]);  ylabel([AB '_{11}']);  xlabel('Time (sec)');  local_style(a, t(g), FS);
exportgraphics(f, fullfile(fo, 'fig_1104_square_circle_gain.png'), 'Resolution', 120);  close(f);
g = ks(1:4:end);  f = figure('Visible', 'off', 'Color', 'w', 'Position', [20 20 1500 700]);  tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
a = nexttile(tl);  hold on;  h1 = plot(t(g), r.th_tr(g, 1), 'Color', cT, 'LineWidth', 1.3);  h2 = plot(t(g), r.thx(g), 'Color', cE, 'LineWidth', 0.8);  ylim([-90 90]);  ylabel([th '_x  [deg]']);  legend([h1 h2], {'True', 'Estimate'}, 'Location', 'northoutside', 'Orientation', 'horizontal');  local_style(a, t(g), FS);
a = nexttile(tl);  hold on;  plot(t(g), r.th_tr(g, 2), 'Color', cT, 'LineWidth', 1.3);  plot(t(g), r.thy(g), 'Color', cE, 'LineWidth', 0.8);  ylim([-110 110]);  ylabel([th '_y  [deg]']);  xlabel('Time (sec)');  local_style(a, t(g), FS);
exportgraphics(f, fullfile(fo, 'fig_1104_square_circle_angle.png'), 'Resolution', 120);  close(f);
fprintf('scan: w range %.2f um, delta u v w sd %.3f %.3f %.3f um, mean|a33-a_d| %.3f, est-true %+.3f\n', max(Lc(ks, 3)) - min(Lc(ks, 3)), std(E(ks, 1)), std(E(ks, 2)), std(E(ks, 3)), mean(abs(r.aN_tr(ks) - ad)), mean(r.aN(ks) - r.aN_tr(ks)));

% system(['open -a Preview "' fullfile(fo, 'fig_1104_square_circle_motion.png') '" "' fullfile(fo, 'fig_1104_square_circle_gain.png') '" "' fullfile(fo, 'fig_1104_square_circle_angle.png') '"']);

function local_style(a, tv, fs)
    box(a, 'on');  xlim(a, [tv(1) tv(end)]);  set(a, 'FontSize', fs, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off');
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
