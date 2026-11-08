% STATUS: SCRATCH -- 2026-11-04 (user): circle flow (5 s circle in the frozen frame, descent 1 s, scan right away) with the SQUARE raster (turn at x = +-13 um, y step 1 um, 27 lines), rho 4, a_d .07, seeds 1-3, cached temp_1104_square_circle_s<n>.mat.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  Ts = 1 / 1612;  nl = 27;  L0 = 36;  ad = 0.07;  rbd = 5;  rho = 4;  FS = 14;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  BL = [0 0 1];  AB = char(257);  MU = [char(956) 'm'];  DL = char(948);  hatc = char(770);  th = char(952);  ka = char(954);  pr = char(8242);
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
o = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 5, 'dither', [3 8 2], 'dither_win', [0 5], 'dither_frame0', true, 'tau_a', 0.06, 'a_d', ad, ...
           'ap_q', true, 'wall', 'fillet', 'bead_r', rbd, 'fillet_rho', rho, 'bead_start', [13; -13], 'ap_floor_law', true, 'raster', [13 1 nl], 'scan_ramp', 0.5, 'scan_len', 1e6, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], ...
           'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 0], 'tag', 'fc', 'du_d', 40 * Ts / Rr, 'T', 6 + 1.4 * nl * 27 / 40 + 1, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'seeds', 1);
rs = cell(3, 1);  if isempty(gcp('nocreate')); parpool('Processes', 3); end
parfor q = 1:3
    oo = o;  oo.seeds = q;  R = run_t3_xyz_fixed(oo);  r = R.runs{1};  n = find(isfinite(r.thx(1:r.kend)), 1, 'last');  fin = isfinite(r.rs_tend) && ~r.contact && r.t(n) >= r.rs_tend;
    k = find(r.t >= r.tsc + 0.5 & r.t <= r.t(n));  i5 = find(r.t >= 5, 1);  rs{q} = [q, fin, r.t(n), max(abs(r.thx(k))), 1e3 * (min(r.h(k)) * Rr - Rr), r.h(i5) * Rr];
    if q > 0; local_save(fullfile(fo, sprintf('temp_1104_square_circle_s%d.mat', q)), r); end
end
for q = 1:3; fprintf('seed %d: finished %d, last finite t %.2f s, max |theta_x-hat| %.1f deg, min gap - contact %.1f nm, gap at 5 s %.3f um\n', rs{q}); end

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
function local_save(fn, r)
    save(fn, 'r', '-v7.3');
end
