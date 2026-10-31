% STATUS: SCRATCH -- 2026-10-29 (user): regulation right after the descent stop, scan later (time near the wall with regulation motion),
%   vs the current wait. Void sphere radius 15 um, 40 um/s full turn, ramp 0.5 s, kappa-hat from t_scan, tau_a 0.5 s, angle P_fix 2 deg from
%   t_scan, report slope P_fix, twister seeds 1-5. Arms (post_wait = [regulation, scan] after the stop): (A) [1 1] current | (A2) [1 1]
%   without kappa_from_scan | (B) [0 1] | (C) [0 3]. Console at the stop (11 s) and at t_scan: theta-hat error, a-hat - a, a_33;
%   decay of a-hat - a in the second after the stop; over the turn: gap range, mean |a_33 - a_d|, mean |a-hat - a|.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  spd = 40;  Tr = 0.5;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);  Tsc = Lturn / spd + Tr / 2;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'kappa_from_scan', true, 'scan_ramp', Tr, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, ...
              'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'tag', 'rw');
arm = {'(A) wait 1 s, then reg + scan', '(A2) as A, kappa free before scan', '(B) reg at stop, scan after 1 s', '(C) reg at stop, scan after 3 s'};
PW = [1 1; 1 1; 0 1; 0 3];  NA = 4;  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);  o.post_wait = PW(IA(q), :);  o.T = 11 + PW(IA(q), 2) + Tsc + 0.2;
    if IA(q) == 2; o.kappa_from_scan = false; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'aN', r.aN, 'aN_tr', r.aN_tr, 'h', r.h, 'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr);
end
for ia = 1:NA
    tS = 11 + PW(ia, 2);  M = NaN(NS, 12);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ty = unwrap(r.th_tr(:, 2) * pi / 180) * 180 / pi;  e = @(k) [r.thx(k) - r.th_tr(k, 1), r.thy(k) - ty(k)];
        k1 = kk(abs(r.t(kk) - 11) < Ts / 2);  k2 = kk(abs(r.t(kk) - 12) < Ts / 2);  k0 = kk(abs(r.t(kk) - tS) < Ts / 2);  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tsc);
        M(id, :) = [e(k1), e(k0), r.aN(k1) - r.aN_tr(k1), r.aN(k2) - r.aN_tr(k2), r.aN_tr(k0), min(r.h(ks)) * Rr, max(r.h(ks)) * Rr, mean(abs(r.aN_tr(ks) - ad)), mean(abs(r.aN(ks) - r.aN_tr(ks))), r.contact];
    end
    fprintf(['%-36s| theta err (x, y) at 11 s: %s / %s | at t_scan: %s / %s | a-hat - a at 11 s %s, at 12 s %s | a33 at t_scan %s\n' ...
             '%-36s| turn: gap %s - %s um, mean|a33 - a_d| %s (%.3f), mean|a-hat - a| %.3f, contact %d\n'], arm{ia}, ...
            mat2str(round(M(:, 1)', 1)), mat2str(round(M(:, 2)', 1)), mat2str(round(M(:, 3)', 1)), mat2str(round(M(:, 4)', 1)), mat2str(round(M(:, 5)', 3)), mat2str(round(M(:, 6)', 3)), ...
            mat2str(round(M(:, 7)', 3)), '', mat2str(round(M(:, 8)', 2)), mat2str(round(M(:, 9)', 2)), mat2str(round(M(:, 10)', 3)), mean(M(:, 10)), mean(M(:, 11)), sum(M(:, 12)));
end

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
