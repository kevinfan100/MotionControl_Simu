% STATUS: SCRATCH -- 2026-10-29 (user): is the different hold-phase angle behaviour vs the morning figure (fig_1029_reg_before.png,
%   plot_1029_turn_reg.m, parfor before the rng fix) a seed / generator effect or a setting effect? Re-run the morning setting (hold 10 s,
%   regulation 10-17 s, slope law L, kappa, seed 1) with each candidate generator, hold phase only (T = 10 s); print theta-hat at 2.4 s and
%   3.7 s (morning figure: theta-hat_y ~ 8 at 2.4 s, theta-hat_x ~ -2.7 at 3.7 s) and the 8-10 s mean error.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
Rr = 2.25;  rb = 15;  ad = 0.05;  Ts = 1 / 1612;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 2, 'a_d', ad, ...
              'ap_q', true, 'q_theta_from', 17, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
              'scan_dir', [1 0], 'hemi_start_deg', 10, 'scan_plane_n', [0 1 0], 'ap_pfix_law', true, 'kappa_state', true, ...
              't_scan', 17, 'scan_len', 0, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'T', 10, 'seeds', 1, 'tag', 'rr');
G = {'twister', 'threefry', 'philox', 'combRecursive', 'multFibonacci'};
for g = 1:numel(G)
    o = base;  o.rng_gen = G{g};  evalc('R = run_t3_xyz_fixed(o);');  r = R.runs{1};
    k1 = round(2.4 / Ts) + 1;  k2 = round(3.7 / Ts) + 1;  k = find(r.t >= 8 & (1:numel(r.t))' <= r.kend);
    fprintf('%-14s: theta-hat_y(2.4 s) %5.1f, theta-hat_x(3.7 s) %5.1f | 8-10 s error x %5.1f, y %5.1f deg\n', G{g}, r.thy(k1), r.thx(k2), mean(r.thx(k) - r.th_tr(k, 1)), mean(r.thy(k) - r.th_tr(k, 2)));
end
