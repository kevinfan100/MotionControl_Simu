% 2026-11-05 (user): as plot_1103_flow.m (hold 10 s, descent 1 s, scan right away) with the line change at 10 um/s (v step .0062 um, 1/4 of the u step).
% STATUS: SCRATCH -- 2026-11-03 (user): current flow, single seed: hold 10 s at 15 um, descent 1 s along w-hat, scan right at the bottom (post_wait 0), Meng local raster (fixed du, 36 um lines, 27 lines, v 1 um), rho 4 bead, a_d .07, 40 um/s, PI tau_a .06, angle Q geometric .9, kappa-hat from the scan start, curvature coupling off before the scan (current code). Seed 1.
%   Four figures as plot_1102_rho4_report.m: 3-D (24 x 16 um window, v steps removed) | motion and motion error | gains (estimates, a_d) | angles. 3-D also in a MATLAB window. Cached in temp_1105_flow_v10.mat.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  Ts = 1 / 1612;  nl = 27;  L0 = 36;  ad = 0.07;  rbd = 5;  rho = 4;  FS = 14;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  BL = [0 0 1];  AB = char(257);  MU = [char(956) 'm'];  DL = char(948);  hatc = char(770);  th = char(952);  ka = char(954);  pr = char(8242);
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
o = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.06, 'a_d', ad, ...
           'ap_q', true, 'wall', 'fillet', 'bead_r', rbd, 'fillet_rho', rho, 'bead_start', [18; -13], 'ap_floor_law', true, 'raster_local', [L0 1 nl 10 * Ts], 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], ...
           'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 0], 'tag', 'nf', 'du_d', 40 * Ts / Rr, 'T', 11 + 1.2 * nl * L0 / 40 + 4, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'seeds', 1);
rs = cell(4, 1);  if isempty(gcp('nocreate')); parpool('Processes', 4); end
parfor q = 2:5
    oo = o;  oo.seeds = q;  R = run_t3_xyz_fixed(oo);  r = R.runs{1};  n = find(isfinite(r.thx(1:r.kend)), 1, 'last');  fin = isfinite(r.rs_tend) && ~r.contact && r.t(n) >= r.rs_tend;
    rs{q - 1} = [q, fin, r.t(n)];  local_save(fullfile(fo, sprintf('temp_1105_flow_v10_s%d.mat', q)), r);
end
for q = 1:4; fprintf('seed %d: finished %d, last finite t %.2f s\n', rs{q}); end
function local_save(fn, r)
    save(fn, 'r', '-v7.3');
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
