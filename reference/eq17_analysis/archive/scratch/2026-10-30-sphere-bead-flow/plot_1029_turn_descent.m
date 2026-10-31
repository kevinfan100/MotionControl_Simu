% STATUS: SCRATCH -- 2026-10-29 (user): one full turn of the whole void sphere (radius 15 um) at 40 um/s with the curvature state, new
%   approach: hold 0-10 s; 10-15 s constant-speed descent along the estimated normal without regulation (motion control only), speed =
%   (15 - h*) um / 5 s so the probe ends near h*; from 15 s the scan (great circle in the x-z plane) and the regulation (tau_a 2 s to
%   a_d 0.05) start together. Settled estimator otherwise (start 10 deg off the top, theta-hat[0] = 0, prior 10 deg, slope L, angle P_fix
%   2 deg from the scan start). Seeds 1-3, the seed that stays on the sphere longest (then smallest mean gap) drawn.
%   (1) fig_1029_desc_scan: side view (dashed = start along the true w), gains, angles vs time since the scan start.
%   (2) fig_1029_desc_before: gains and angles 0-15 s (dashed = end of the hold).
%   (3) fig_1029_desc_local_p: p in the true local coordinate -- u, v = sum of V_k' dp, w = h R (linear axis, h* dotted), desired = actual +
%       V_k' (p_d - p); dashed = end of the hold, dash-dot = scan start. No numbers on the figures.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  Td = 5;  tS = tH + Td;  ph0 = 10;  Ts = 1 / 1612;  spd = 40;  if exist('spd_in', 'var'); spd = spd_in; end
pf = 2;  if exist('pf_in', 'var'); pf = pf_in; end
FS = 15;  hatc = char(770);  AB = char(257);  th = char(952);  mu = char(181);  DL = char(948);
cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);  vd = (15 - hs * Rr) / Td;
fprintf('descent speed %.3f um/s over %d s (15 um -> %.2f um)\n', vd, Td, hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
              'scan_dir', [1 0], 'hemi_start_deg', ph0, 'scan_plane_n', [0 1 0], 'ap_pfix_law', true, 'kappa_state', true, ...
              'descent_v', vd, 'descent_T', Td, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, 'sP_fix_theta_deg', pf, 'T', tS + Lturn / spd + 0.2, 'tag', 'dc');
ra = cell(1, 3);
if isempty(gcp('nocreate')); parpool('Processes', 3); end
parfor id = 1:3; o = base;  o.seeds = id;  R = run_t3_xyz_fixed(o);  ra{id} = R.runs{1}; end
sc = zeros(1, 3);  Te = tS + Lturn / spd;
for id = 1:3
    r = ra{id};  ks = find(r.t >= tS & r.t <= Te & (1:numel(r.t))' <= r.kend);  gap = r.h(ks) * Rr;
    bad = find(gap > 5 | isnan(r.aN(ks)), 1);  if isempty(bad); kon = numel(ks); else; kon = bad - 1; end;  fr = kon / round((Te - tS) / Ts);
    kS = find(r.t >= tS - 1e-9, 1);  ty = unwrap(r.th_tr(1:r.kend, 2) * pi / 180) * 180 / pi;
    sc(id) = fr - 0.01 * mean(gap(1:max(kon, 1)));
    fprintf('seed %d: at the scan start gap %.2f um, a33 %.3f (true %.3f), angle err (y, x) %.1f %.1f deg | on the sphere %.2f of the turn | gap mean %.2f max %.2f um\n', id, ...
            r.h(kS) * Rr, r.aN(kS), r.aN_tr(kS), ty(kS) - r.thy(kS), r.th_tr(kS, 1) - r.thx(kS), fr, mean(gap(1:max(kon, 1))), max(gap(1:max(kon, 1))));
end
[~, sd] = max(sc);  if exist('seed_in', 'var'); sd = seed_in; end;  r = ra{sd};  fprintf('seed %d drawn\n', sd);
K = find(r.t <= Te & (1:numel(r.t))' <= r.kend);  ty = unwrap(r.th_tr(K, 2) * pi / 180) * 180 / pi;  k = find(r.t(K) >= tS);  t = r.t(K(k)) - tS;
% ---------------- (1) the scan ----------------
f = figure('Color', 'w', 'Position', [30 30 900 1500]);  tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl, 1, [2 1]);  hold on;  box on;
tg = linspace(0, 2 * pi, 400);  fill(rb * sin(tg), rb * cos(tg), [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(rb * sin(tg), rb * cos(tg), 'Color', cGr, 'LineWidth', 3, 'DisplayName', 'wall');
p0 = r.p(1, [1 3]);  pw = rb * p0 / norm(p0);  plot([p0(1) pw(1)], [p0(2) pw(2)], 'k--', 'LineWidth', 1.6, 'DisplayName', 'start along the true w');
plot(r.p(K, 1), r.p(K, 3), 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'p');
plot(p0(1), p0(2), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 'start');
plot(r.p(K(k(1)), 1), r.p(K(k(1)), 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 't_{scan}');
axis equal;  xlim([-24 24]);  ylim([-24 31]);  xlabel('x  [\mum]');  ylabel('z  [\mum]');  legend('Location', 'eastoutside');
title(sprintf('%d um/s, descent + scan, + \\kappa, P_{fix} %g deg, seed %d', spd, pf, sd));
axg = nexttile(tl, 3);  hold on;  box on;
h1 = plot(t, r.aN_tr(K(k)), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.aT_tr(K(k)), 'Color', cT, 'LineWidth', 2.2);
h2 = plot(t, r.aN(K(k)), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(t, r.aT(K(k)), 'Color', cG, 'LineWidth', 1.4);  h4 = yline(ad, 'k--', 'LineWidth', 1.5);
ylim([0 1.1]);  ylabel('gain');
legend([h1 h2 h3 h4], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc), sprintf('%s_d = %g', AB, ad)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
axa = nexttile(tl, 4);  hold on;  box on;
h1 = plot(t, ty(k), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.th_tr(K(k), 1), 'Color', cT, 'LineWidth', 2.2);
h2 = plot(t, r.thy(K(k)), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(t, r.thx(K(k)), 'Color', cG, 'LineWidth', 1.4);
ylabel(sprintf('%s  [\\circ]', th));  xlabel('time since t_{scan}  (sec)');
legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
for a = [ax axg axa]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
set([axg axa], 'XLim', [0 t(end)]);
fn1 = fullfile(fo, 'fig_1029_desc_scan.png');  exportgraphics(f, fn1, 'Resolution', 130);  close(f);
% ---------------- (2) before the scan ----------------
f = figure('Color', 'w', 'Position', [30 30 1100 1300]);  tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
kb = find(r.t(K) < tS);  tb = r.t(K(kb));
axg = nexttile(tl);  hold on;  box on;
h1 = plot(tb, r.aN_tr(K(kb)), 'Color', cT, 'LineWidth', 2.2);  plot(tb, r.aT_tr(K(kb)), 'Color', cT, 'LineWidth', 2.2);
h2 = plot(tb, r.aN(K(kb)), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(tb, r.aT(K(kb)), 'Color', cG, 'LineWidth', 1.4);  h4 = yline(ad, 'k--', 'LineWidth', 1.5);
ylim([0 1.1]);  ylabel('gain');  title(sprintf('before the scan: hold 0-%d s, constant-speed descent %d-%d s (no regulation)', tH, tH, tS));
legend([h1 h2 h3 h4], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc), sprintf('%s_d = %g', AB, ad)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
axa = nexttile(tl);  hold on;  box on;
h1 = plot(tb, ty(kb), 'Color', cT, 'LineWidth', 2.2);  plot(tb, r.th_tr(K(kb), 1), 'Color', cT, 'LineWidth', 2.2);
h2 = plot(tb, r.thy(K(kb)), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(tb, r.thx(K(kb)), 'Color', cG, 'LineWidth', 1.4);
ylabel(sprintf('%s  [\\circ]', th));  xlabel('Time (sec)');
legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
for a = [axg axa]; xline(a, tH, '--', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');  set(a, 'XLim', [0 tS], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
fn2 = fullfile(fo, 'fig_1029_desc_before.png');  exportgraphics(f, fn2, 'Resolution', 130);  close(f);
% ---------------- (3) local motion: p in the true local coordinate ----------------
N3 = numel(K);  uv = zeros(N3, 2);  del = zeros(N3, 3);
for j = 1:N3
    a = r.th_tr(K(j), :) * pi / 180;  Vt = [cos(a(2)) 0 sin(a(2)); 0 1 0; -sin(a(2)) 0 cos(a(2))] * [1 0 0; 0 cos(a(1)) -sin(a(1)); 0 sin(a(1)) cos(a(1))];
    del(j, :) = (Vt' * (r.pd(K(j), :) - r.p(K(j), :))')';
    if j > 1; dpl = Vt' * (r.p(K(j), :) - r.p(K(j - 1), :))';  uv(j, :) = uv(j - 1, :) + dpl(1:2)'; end
end
act = [uv, r.h(K) * Rr];  des = act + del;  nm = {'u', 'v', 'w'};  tt = r.t(K);
f = figure('Color', 'w', 'Position', [30 30 1500 1100]);  tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:3
    a = nexttile(tl);  hold on;  box on;
    h1 = plot(tt, des(:, row), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(tt, act(:, row), 'Color', cE, 'LineWidth', 1.3);
    if row == 3; yline(a, hs * Rr, 'k:', 'LineWidth', 1.5, 'HandleVisibility', 'off');  ylim([0 16]); end
    ylabel(sprintf('%s  [%sm]', nm{row}, mu));  xline(a, tH, '--', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xline(a, tS, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');
    set(a, 'XLim', [0 Te], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
    if row == 1; legend([h1 h2], {sprintf('%s_d', nm{row}), nm{row}}, 'Location', 'northoutside', 'Orientation', 'horizontal');  title(sprintf('p in the true local coordinate, %d um/s, seed %d', spd, sd)); end
    if row == 3; xlabel('Time (sec)'); end
end
fn3 = fullfile(fo, 'fig_1029_desc_local_p.png');  exportgraphics(f, fn3, 'Resolution', 130);  close(f);
system(['open ' fn1 ' ' fn2 ' ' fn3]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
