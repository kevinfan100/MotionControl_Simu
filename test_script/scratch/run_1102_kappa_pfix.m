% STATUS: SCRATCH -- 2026-11-02 (user): the w-hat lag in the ring. [hypothesis] kappa-hat cannot follow the ring: sqrt P_fix .3/R is the span of the convex path only (R/7.43 = .30), the ring path is 2.25/1.49 = 1.51/R. Rival: the angle Q alone; split: kappa_pfix 1.5 against .3, fixed 36 um, a_d .07, seeds 1-5, paired (arm .3 from temp_square_why.mat). Pre-registered: along mean in the ring from -5 to near 0 deg, ring 95% below 17.9 deg, no loss on the bead. 1.5 reads the test wall, test only, not a design value.
%   Sanity: arm .3 repeats ring 6.00 / 17.94 deg. Cached in temp_kappa_pfix.mat.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');  Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  rbd = 5;  rho = 4;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  NS = 5;
cf = fullfile(fo, 'temp_kappa_pfix.mat');
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, ...
              'ap_q', true, 'wall', 'fillet', 'bead_r', rbd, 'fillet_rho', rho, 'ap_floor_law', true, 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'descent_T', 1, 'post_wait', [0 1], 'tag', 'kp', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 1.5, 'raster_local', [36 1 27 0.05], 'bead_start', [18; -13], 'T', 12 + 1.2 * 27 * 36 / 40 + 1);
if isfile(cf)
    load(cf, 'rk');
else
    rk = cell(NS, 1);
    if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
    pctRunOnAll warning('off', 'all');
    parfor q = 1:NS
        o = base;  o.seeds = q;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
        rk{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'tsc', r.tsc, 'te', r.rs_tend, 'p', r.p, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr, 'kap', r.kap(:));
    end
    save(cf, 'rk', '-v7.3');
end
S = load(fullfile(fo, 'temp_square_why.mat'), 'rs', 'A');  r0 = S.rs([S.A.a] == 3);  C = {r0, rk};  nm = {'pfix .3', 'pfix 1.5'};
for c = 1:2
    Ang = [];  Al = [];  Rt = [];  RR = [];
    for q = 1:NS
        r = C{c}{q};  ok = find(isfinite(r.aN(1:r.kend)) & isfinite(r.thx(1:r.kend)));  fin = isfinite(r.te) && ~r.contact;  te = r.t(ok(end));  if fin; te = r.te; end;  k = ok(r.t(ok) >= r.tsc + 0.5 & r.t(ok) <= te);  k = k(k > 2);  a = local_w(r.thx(k), r.thy(k));  b = local_w(r.th_tr(k, 1), r.th_tr(k, 2));  b0 = local_w(r.th_tr(k - 1, 1), r.th_tr(k - 1, 2));  db = movmean(b - b0, 11, 1);  n = vecnorm(db, 2, 2);  d = db ./ max(n, 1e-9);  Ang = [Ang; acosd(min(sum(a .* b, 2), 1))];  Al = [Al; asind(max(min(sum((a - b) .* d, 2), 1), -1))];  Rt = [Rt; asind(min(n, 1))];  RR = [RR; hypot(r.p(k, 1), r.p(k, 2))];
        fprintf('%s seed %d: finished %d, end %.1f s, min gap - contact %.1f nm, est-true %+.3f\n', nm{c}, q, fin, te, 1e3 * (min(r.h(k)) * Rr - Rr), mean(r.aN(k) - r.aN_tr(k)));
    end
    m1 = RR > 7.2 & RR < 9;  m2 = RR < 7.2;  m3 = RR > 10;  mt = m1 & Rt >= 0.2 & Rt < 1;
    fprintf('%s: ring angle median %.2f, 95%% %.2f deg, along mean %+.2f deg | bead median %.2f, 95%% %.2f | floor median %.2f, 95%% %.2f\n', nm{c}, median(Ang(m1)), prctile(Ang(m1), 95), mean(Al(mt)), median(Ang(m2)), prctile(Ang(m2), 95), median(Ang(m3)), prctile(Ang(m3), 95));
end
function w = local_w(thx, thy)
    tx = thx(:) * pi / 180;  ty = thy(:) * pi / 180;  w = [sin(ty) .* cos(tx), -sin(tx), cos(ty) .* cos(tx)];
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
