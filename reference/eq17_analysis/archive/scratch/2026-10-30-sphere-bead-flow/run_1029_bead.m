% STATUS: SCRATCH -- 2026-10-29 (user): Meng glass bead (diss. Sec. 6.2.1), simplified: bead radius 5 um on the glass, the plant is the
%   two-wall superposition of the plane law (opts wall 'bead2': Gamma / gamma_N = I + sum_i V_i (C_i - I) V_i', drag and thermal from the same
%   Gamma). Probe starts 15 um above the glass at (9, -9) um, theta-hat[0] = 0 (the glass is flat, so the start is right), hold 0-10 s,
%   regulation tau_a 2 s to a_d 0.05 from 10 s, raster from 17 s: lines along x turning at x = +-9 um, steps 1.5 um along y, 13 lines
%   (y = -9 ... 9 um). Document estimator, slope L, angle P_fix from the scan start. Arms: 5 um/s (P_fix 2) | 5 um/s + kappa (1) |
%   40 um/s (4) | 40 um/s + kappa (2). Seed 1. Saves temp_1029_bead.mat for the figures.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  tS = 17;  Ts = 1 / 1612;
arms = struct('name', {'5 um/s', '5 um/s + kappa', '40 um/s', '40 um/s + kappa'}, 'spd', {5, 5, 40, 40}, 'pf', {2, 1, 4, 2}, 'kp', {false, true, false, true});
rs = cell(1, 4);
if isempty(gcp('nocreate')); parpool('Processes', 4); end
parfor ia = 1:4
    a = arms(ia);  Lline = 2 * 9 + 6;  Tsc = (13 * Lline + 12 * 1.5) / a.spd * 1.15;
    o = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
               'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 2, 'a_d', 0.05, ...
               't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'bead2', 'bead_r', 5, 'bead_start', [9 -9], 'ap_floor_law', true, ...
               'scan_dir', [1 0], 'ap_pfix_law', true, 'raster', [9 1.5 13], 'scan_len', 1e9, 'du_d', a.spd * Ts / Rr, ...
               'sP_fix_theta_deg', a.pf, 'T', tS + Tsc, 'seeds', 1, 'tag', 'bd');
    if a.kp; o.kappa_state = true; end
    R = run_t3_xyz_fixed(o);  rs{ia} = R.runs{1};
end
for ia = 1:4
    r = rs{ia};  te = r.rs_tend;  if isnan(te); te = r.t(r.kend); end;  ks = find(r.t >= tS & r.t <= te & (1:numel(r.t))' <= r.kend);
    fprintf('%-16s: contact %d | raster done %d (scan %.1f s) | gap %.2f-%.2f um (mean %.2f) | rms a33 err %.3f\n', arms(ia).name, r.contact, ~isnan(r.rs_tend), te - tS, ...
            min(r.h(ks)) * Rr, max(r.h(ks)) * Rr, mean(r.h(ks)) * Rr, rms(r.aN(ks) - r.aN_tr(ks)));
end
save(fullfile(fo, 'temp_1029_bead.mat'), 'rs', 'arms', 'tS');
