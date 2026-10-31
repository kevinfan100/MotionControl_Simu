% STATUS: SCRATCH -- 2026-10-29 (user): whole-bead scan, Meng-style. Wall 'fillet': bead radius 5 um on the glass, concave junction filled by a
%   torus of tube radius 6 um (one smooth wall, plane law on the local tangent plane of the nearest point). Flow: start 15 um above the glass
%   at (x, y) = (13, -13) um, hold 10 s, constant-speed descent 1 s, regulation from the stop (tau_a 0.5 s, a_d 0.05), raster from 12 s:
%   lines along -+x between x = -13 and 13 um (direction y x w-hat, so y is kept and the probe follows the surface), 1 um steps in +y,
%   27 lines (y = -13 .. 13 um), 10 um/s (user 10-29), speed ramp 0.5 s, angle P_fix 2 deg, report slope P_fix, threefry.
%   Arms: (1) kappa state on (from the scan) seed 1 | (2) kappa off seed 1 | (3) kappa on seed 2 (override: ad_in, kap_in, seeds_in, sfx_in). Saves test_results/temp_3d_wsonly/
%   temp_fillet_raster.mat (logs needed for the figures).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  if exist('ad_in', 'var'); ad = ad_in; end;  sfx = '';  if exist('sfx_in', 'var'); sfx = sfx_in; end;  kap_arms = [1 0 1];  if exist('kap_in', 'var'); kap_arms = kap_in; end;  Ts = 1 / 1612;  spd = 10;  Xe = 13;  dy = 1;  nl = 27;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Tend = 12 + nl * (2 * Xe / spd + dy / spd) + 1;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'fillet', 'bead_r', 5, 'fillet_rho', 6, 'bead_start', [Xe; -Xe], 'ap_floor_law', true, ...
              'raster', [Xe dy nl], 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', spd * Ts / Rr, 'sP_fix_theta_deg', 2, ...
              'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'T', Tend, 'tag', 'fr');
if exist('keepy_in', 'var'); base.raster_keep_y = keepy_in; end
sds = [1 1 2];  if exist('seeds_in', 'var'); sds = seeds_in; end;  onoff = {'off', 'on'};  arm = arrayfun(@(i) sprintf('kappa %s, seed %d', onoff{kap_arms(i) + 1}, sds(i)), 1:3, 'UniformOutput', false);  rs = cell(1, 3);
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:3
    o = base;  o.seeds = sds(q);  if kap_arms(q); o.kappa_state = true;  o.kappa_from_scan = true; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'p', r.p, 'pd', r.pd, 'h', r.h, 'aN', r.aN, 'aT', r.aT, 'aN_tr', r.aN_tr, 'aT_tr', r.aT_tr, ...
                   'thx', r.thx, 'thy', r.thy, 'th_tr', r.th_tr, 'apN', r.apN, 'tsc', r.tsc, 'rs_tend', r.rs_tend, 'n_floor', r.n_floor);
end
for q = 1:3
    r = rs{q};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= r.tsc);  if ~isnan(r.rs_tend); ks = ks(r.t(ks) <= r.rs_tend); end
    fprintf('%-18s: kend %d/%d contact %d | raster end %.1f s | gap %.2f-%.2f um (target %.2f), mean %.2f | mean|a33 - a_d| %.3f | floor %d\n', arm{q}, r.kend, numel(r.t), r.contact, ...
            r.rs_tend, min(r.h(ks)) * Rr, max(r.h(ks)) * Rr, hs * Rr, mean(r.h(ks)) * Rr, mean(abs(r.aN_tr(ks) - ad)), r.n_floor);
end
save(fullfile(fo, ['temp_fillet_raster' sfx '.mat']), 'rs', 'arm', 'hs', 'Rr', 'ad', 'Xe', 'dy', 'nl', '-v7.3');

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
