% STATUS: SCRATCH -- 2026-11-02 (user): why the square range fails where the fixed-length local raster does not. rho 4, a_d .07, tau_a .06, seeds 1-5. Arms: A square +-13 | B square +-18 | C local raster 36 um | D square +-13, v .5 um, seed 1 with all estimates for the result figure.
%   [hypothesis] the floor run after a turn at +-13 is too short before the ring r 7.2-8.7; rival: seed luck; split: B against A, paired seeds. Printed per run: finished, contact point, path since the last turn, a_33 est-true and angle error in the last .5 s. Sanity: A seeds 1-3 end at 25.33, 15.95, 20.91 s. Cached in temp_square_why.mat.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  rbd = 5;  rho = 4;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
cf = fullfile(fo, 'temp_square_why.mat');
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, ...
              'ap_q', true, 'wall', 'fillet', 'bead_r', rbd, 'fillet_rho', rho, 'ap_floor_law', true, 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'descent_T', 1, 'post_wait', [0 1], 'tag', 'wy', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3);
A = struct('a', {}, 'seed', {});  for a = 1:3; for s = 1:5; A(end + 1) = struct('a', a, 'seed', s); end; end;  A(end + 1) = struct('a', 4, 'seed', 1);  N = numel(A);
if isfile(cf)
    load(cf, 'rs');
else
    rs = cell(N, 1);
    if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
    pctRunOnAll warning('off', 'all');
    parfor q = 1:N
        o = base;  o.seeds = A(q).seed;
        switch A(q).a
            case 1; o.raster = [13 1 27];  o.bead_start = [13; -13];  o.T = 12 + 1.4 * 27 * 27 / 40 + 1;
            case 2; o.raster = [18 1 27];  o.bead_start = [18; -13];  o.T = 12 + 1.4 * 27 * 37 / 40 + 1;
            case 3; o.raster_local = [36 1 27 0.05];  o.bead_start = [18; -13];  o.T = 12 + 1.2 * 27 * 36 / 40 + 1;
            case 4; o.raster = [13 0.5 53];  o.bead_start = [13; -13];  o.T = 12 + 1.4 * 53 * 27 / 40 + 1;
        end
        R = run_t3_xyz_fixed(o);  r = R.runs{1};
        rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'te', r.rs_tend, 'p', r.p, 'pd', r.pd, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'aT', r.aT, 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr);
    end
    save(cf, 'rs', 'A', '-v7.3');
end
nm = {'A +-13', 'B +-18', 'C local 36', 'D +-13 v.5'};  fprintf('arm seed fin | end s | min gap | contact x y r | path since turn | est-true | angle err x y [deg] (last .5 s)\n');
for q = 1:N
    r = rs{q};  ok = find(isfinite(r.aN(1:r.kend)) & isfinite(r.h(1:r.kend)) & isfinite(r.thx(1:r.kend)));  fin = isfinite(r.te) && ~r.contact;  te = r.t(ok(end));  if isfinite(r.te); te = r.te; end;  k = ok(r.t(ok) >= r.tsc & r.t(ok) <= te);  [~, im] = min(r.h(k));  ke = k(end);  x = r.p(k, 1);  dx = movmean([0; diff(x)], 101);  ip = find(sign(dx) ~= sign(dx(end)) & dx ~= 0, 1, 'last');  if isempty(ip); ip = 1; end;  L = sum(vecnorm(diff(r.p(k(ip:end), :), 1, 1), 2, 2));
    z = k(r.t(k) >= te - 0.5);  fprintf('%-11s %d %d | %5.1f | %.3f | %6.1f %6.1f %5.1f | %5.1f | %+.3f | %.1f %.1f\n', nm{A(q).a}, A(q).seed, fin, te, min(r.h(k)) * Rr, r.p(ke, 1), r.p(ke, 2), hypot(r.p(ke, 1), r.p(ke, 2)), L, mean(r.aN(z) - r.aN_tr(z)), sqrt(mean((r.thx(z) - r.th_tr(z, 1)).^2)), sqrt(mean((r.thy(z) - r.th_tr(z, 2)).^2)));
end
fprintf('failures: direction of x and z rate over the last .2 s, last z, floor run before the corner\n');
for q = 1:N
    r = rs{q};  if isfinite(r.te) && ~r.contact; continue; end
    ok = find(isfinite(r.aN(1:r.kend)) & isfinite(r.h(1:r.kend)));  k = ok(r.t(ok) >= r.tsc);  ke = k(end);  z = k(max(numel(k) - 322, 1):end);  rq = hypot(r.p(k, 1), r.p(k, 2));  ifl = find(rq > 9.5, 1, 'last');  L = sum(vecnorm(diff(r.p(k(ifl:end), :), 1, 1), 2, 2));  kk = k(max(ifl - 400, 1):ifl);  dxs = r.p(k(ifl), 1) - r.p(kk(1), 1);
    fprintf('%-11s seed %d: dx/dt %+.1f um/s, dz/dt %+.1f um/s, z %.2f um, last on floor at x %.1f y %.1f, heading %+d, %.1f um to contact\n', nm{A(q).a}, A(q).seed, (r.p(ke, 1) - r.p(z(1), 1)) / (r.t(ke) - r.t(z(1))), (r.p(ke, 3) - r.p(z(1), 3)) / (r.t(ke) - r.t(z(1))), r.p(ke, 3), r.p(k(ifl), 1), r.p(k(ifl), 2), sign(dxs), L);
end
ks5 = find([A.a] == 3);  E = [];  RR = [];
for q = ks5
    r = rs{q};  ok = find(isfinite(r.aN(1:r.kend)));  k = ok(r.t(ok) >= r.tsc + 0.5 & r.t(ok) <= r.te);  a1 = local_w(r.thx(k), r.thy(k));  a2 = local_w(r.th_tr(k, 1), r.th_tr(k, 2));  E = [E; acosd(min(sum(a1 .* a2, 2), 1))];  RR = [RR; hypot(r.p(k, 1), r.p(k, 2))];
end
seg = {RR > 10, RR > 7.2 & RR < 9, RR < 7.2, RR >= 0};  sn = {'floor r > 10', 'ring r 7.2-9', 'bead r < 7.2', 'all'};
for j = 1:4; e = E(seg{j}); fprintf('angle between w-hat and w, local raster, 5 seeds, %s: median %.2f, rms %.2f, 95%% %.2f, max %.1f deg | wall shift at 2.43 um median %.3f, 95%% %.3f um\n', sn{j}, median(e), sqrt(mean(e.^2)), prctile(e, 95), max(e), 2.426 * sind(median(e)), 2.426 * sind(prctile(e, 95))); end
r = rs{N};  ok = find(isfinite(r.aN(1:r.kend)));  ks = ok(r.t(ok) >= r.tsc & r.t(ok) <= r.te);  p = r.p(ks, :);  y = p(:, 2);  fprintf('D seed 1: y %.2f..%.2f, scan %.2f s, last 5 lines y: ', min(y), max(y), r.te - r.tsc);  fprintf('%.2f ', y(end - round((4:-1:0) * 0.68 / Ts)));  fprintf('\n');
tx = r.thx(ks) * pi / 180;  ty = r.thy(ks) * pi / 180;  tx = tx(:);  ty = ty(:);  Wh = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];  W = movmean(p - Rr * hs * Wh, 41, 1);
dy = [0; diff(y)];  stepv = conv(double(movmean(dy, 21) > 0.012), ones(61, 1), 'same') > 0;  W(stepv | abs(W(:, 1)) > 13 | abs(W(:, 2)) > 13, :) = NaN;  P = p;  P(stepv, :) = NaN;
if any(findall(0, 'Type', 'figure') == 5); close(5); end
f = figure(5);  set(f, 'Color', 'w', 'Position', [50 80 1750 800], 'Visible', 'on', 'Name', 'square scan result, v 0.5 um, seed 1');  tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  MU = [char(956) 'm'];
a1 = nexttile(tl);  hold(a1, 'on');  plot3(a1, P(:, 1), P(:, 2), P(:, 3), 'Color', [0 0 1], 'LineWidth', 0.7);  title(a1, 'probe path');
a2 = nexttile(tl);  hold(a2, 'on');  plot3(a2, W(:, 1), W(:, 2), W(:, 3), 'Color', [0.9 0 0], 'LineWidth', 0.7);  title(a2, 'recovered wall');
for a = [a1 a2]; axis(a, 'equal');  xlim(a, [-14 14]);  ylim(a, [-14 16]);  zlim(a, [-0.5 13.5]);  view(a, -37.5, 22);  grid(a, 'off');  box(a, 'off');  xlabel(a, ['X (' MU ')']);  ylabel(a, ['Y (' MU ')']);  zlabel(a, ['Z (' MU ')']);  set(a, 'FontSize', 15, 'LineWidth', 1.1);  rotate3d(a, 'on'); end
exportgraphics(f, fullfile(fo, 'fig_1102_square_result.png'), 'Resolution', 130);

function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
