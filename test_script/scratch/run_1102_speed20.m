% STATUS: SCRATCH -- 2026-11-02 (user): is the w-hat lag in the ring set by the turn rate. Fixed 36 um raster, rho 4, a_d .07, tau_a .06, seeds 1-5, 20 um/s against 40 um/s (temp_square_why.mat). Pre-registered: along mean in the ring -4.9 to about -2.5 deg if the lag scales with the turn rate; rival: unchanged, the lag is not set by the speed. Finished = trajectory ended, no contact and the estimator finite to the end. Window 11: recovered wall, w-hat 50 ms, 40 and 20 um/s, seed 1.
%   Sanity: arm 40 repeats ring 6.00 / 17.94 deg; scan about 49 s at 20 um/s. Cached in temp_speed20.mat. Output fig_1102_speed20.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  rbd = 5;  rho = 4;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  L = Rr * hs;  NS = 5;  MU = [char(956) 'm'];
cf = fullfile(fo, 'temp_speed20.mat');
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, ...
              'ap_q', true, 'wall', 'fillet', 'bead_r', rbd, 'fillet_rho', rho, 'ap_floor_law', true, 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'descent_T', 1, 'post_wait', [0 1], 'tag', 's2', 'du_d', 20 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'raster_local', [36 1 27 0.05], 'bead_start', [18; -13], 'T', 12 + 1.2 * 27 * 36 / 20 + 1);
if isfile(cf)
    load(cf, 'rk');
else
    rk = cell(NS, 1);
    if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
    pctRunOnAll warning('off', 'all');
    parfor q = 1:NS
        o = base;  o.seeds = q;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
        rk{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'te', r.rs_tend, 'p', r.p, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr);
    end
    save(cf, 'rk', '-v7.3');
end
S = load(fullfile(fo, 'temp_square_why.mat'), 'rs', 'A');  r0 = S.rs([S.A.a] == 3);  C = {r0, rk};  nm = {'40 um/s', '20 um/s'};  rb = [0.2 1; 0.1 0.5];
for c = 1:2
    Ang = [];  Al = [];  Rt = [];  RR = [];
    for q = 1:NS
        r = C{c}{q};  ok = find(isfinite(r.aN(1:r.kend)) & isfinite(r.thx(1:r.kend)));  ke = ok(end);  fin = isfinite(r.te) && ~r.contact && r.t(ke) >= r.te;  te = r.t(ke);  if isfinite(r.te); te = min(r.te, te); end;  k = ok(r.t(ok) >= r.tsc + 0.5 & r.t(ok) <= te);  k = k(k > 2);  a = local_w(r.thx(k), r.thy(k));  b = local_w(r.th_tr(k, 1), r.th_tr(k, 2));  b0 = local_w(r.th_tr(k - 1, 1), r.th_tr(k - 1, 2));  db = movmean(b - b0, 11, 1);  n = vecnorm(db, 2, 2);  d = db ./ max(n, 1e-9);  Ang = [Ang; acosd(min(sum(a .* b, 2), 1))];  Al = [Al; asind(max(min(sum((a - b) .* d, 2), 1), -1))];  Rt = [Rt; asind(min(n, 1))];  RR = [RR; hypot(r.p(k, 1), r.p(k, 2))];
        fprintf('%s seed %d: finished %d, end %.1f s, min gap - contact %.1f nm, gap sd %.0f nm, est-true %+.3f\n', nm{c}, q, fin, te, 1e3 * (min(r.h(k)) * Rr - Rr), 1e3 * std(r.h(k)) * Rr, mean(r.aN(k) - r.aN_tr(k)));
    end
    m1 = RR > 7.2 & RR < 9;  m2 = RR < 7.2;  m3 = RR > 10;  mt = m1 & Rt >= rb(c, 1) & Rt < rb(c, 2);
    fprintf('%s: ring angle median %.2f, 95%% %.2f deg, along mean %+.2f sd %.2f deg (turn %.2f-%.2f deg/step) | bead median %.2f, 95%% %.2f | floor median %.2f, 95%% %.2f\n', nm{c}, median(Ang(m1)), prctile(Ang(m1), 95), mean(Al(mt)), std(Al(mt)), rb(c, 1), rb(c, 2), median(Ang(m2)), prctile(Ang(m2), 95), median(Ang(m3)), prctile(Ang(m3), 95));
end
if any(findall(0, 'Type', 'figure') == 11); close(11); end
f = figure(11);  set(f, 'Color', 'w', 'Position', [50 80 1800 760], 'Visible', 'on', 'Name', 'recovered wall, 40 and 20 um/s, seed 1');  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  tt = {'40 \mum/s', '20 \mum/s'};  NW = [81 161];  ax = gobjects(2, 1);
for c = 1:2
    r = C{c}{1};  ok = find(isfinite(r.aN(1:r.kend)));  k = ok(r.t(ok) >= r.tsc & r.t(ok) <= r.te);  p = r.p(k, :);  Ws = movmean(local_w(r.thx(k), r.thy(k)), NW(c), 1);  Ws = Ws ./ vecnorm(Ws, 2, 2);  W = p - L * Ws;  y = p(:, 2);  dv = [0; diff(y)];  stepv = conv(double(movmean(dv, 21) > 0.012), ones(61, 1), 'same') > 0;  W(stepv | abs(W(:, 1)) > 13 | abs(W(:, 2)) > 13, :) = NaN;
    a = nexttile(tl);  hold(a, 'on');  plot3(a, W(:, 1), W(:, 2), W(:, 3), 'Color', [0.9 0 0], 'LineWidth', 1.0);  title(a, tt{c});  axis(a, 'equal');  xlim(a, [-13 13]);  ylim(a, [-13 13]);  zlim(a, [-0.5 11]);  view(a, -37.5, 22);  grid(a, 'off');  box(a, 'off');  xlabel(a, ['X (' MU ')']);  ylabel(a, ['Y (' MU ')']);  zlabel(a, ['Z (' MU ')']);  set(a, 'FontSize', 15, 'LineWidth', 1.1);  rotate3d(a, 'on');
end
exportgraphics(f, fullfile(fo, 'fig_1102_speed20.png'), 'Resolution', 130);
function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
