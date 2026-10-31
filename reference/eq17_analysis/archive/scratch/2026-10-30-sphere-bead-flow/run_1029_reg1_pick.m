% STATUS: SCRATCH -- 2026-10-29 (user): flow regulation 1 s after the descent stop, then scan (post_wait [0 1]); pick a seed with good angle
%   convergence and a_33 close to a_d. Seeds 1-10 (threefry). Console per seed: angle error at t_scan, rms angle error over the scan,
%   mean |a_33 - a_d| over the turn, gap range; score = max |angle err at t_scan| / 2 deg + rms scan angle err / 1 deg + mean|a33-a_d| / 0.01.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  spd = 40;  Tr = 0.5;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);  Tsc = Lturn / spd + Tr / 2;  tS = 12;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'kappa_from_scan', true, 'scan_ramp', Tr, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, ...
              'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'T', tS + Tsc + 0.2, 'tag', 'pk');
NS = 10;  rs = cell(1, NS);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor id = 1:NS
    o = base;  o.seeds = id;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{id} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'aN_tr', r.aN_tr, 'h', r.h, 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr);
end
S = zeros(1, NS);
for id = 1:NS
    r = rs{id};  kk = (1:r.kend)';  ty = unwrap(r.th_tr(:, 2) * pi / 180) * 180 / pi;  k0 = kk(abs(r.t(kk) - tS) < Ts / 2);  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tsc);
    e0 = [r.thx(k0) - r.th_tr(k0, 1), r.thy(k0) - ty(k0)];  es = sqrt(mean((r.thx(ks) - r.th_tr(ks, 1)).^2 + (r.thy(ks) - ty(ks)).^2));  ma = mean(abs(r.aN_tr(ks) - ad));
    S(id) = max(abs(e0)) / 2 + es / 1 + ma / 0.01;
    fprintf('seed %2d: contact %d | angle err at t_scan (x, y) %5.1f %5.1f deg | rms angle err in the scan %.2f deg | mean|a33 - a_d| %.3f | gap %.2f-%.2f um | score %.2f\n', ...
            id, r.contact, e0, es, ma, min(r.h(ks)) * Rr, max(r.h(ks)) * Rr, S(id));
end
[~, best] = min(S);  fprintf('best seed %d\n', best);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
