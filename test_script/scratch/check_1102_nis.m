% STATUS: SCRATCH -- 2026-11-02 (user): sqrt P too small only with motion. Compare the innovation of y1 with its model variance S (normalised innovation, ratio 1 = honest) and its lag-1 and lag-2 correlation, no motion and circle 3 um, flat wall true (8,-8), seeds 1-3, 4-10 s, per axis in the frame of V-hat. Pre-registered: ratio above 1 with the circle = the model noise is too small; ratio near 1 but lag correlation large = coloured noise taken as white (information overcounted). No figure.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
Rr = 2.25;  Ts = 1 / 1612;  ad = 0.07;  NS = 3;  hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 8, 'theta_y', -8, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'doc_timing', true, 'theta_out', true, 'gap_gate', 2, 'gate_one', true, 'hold_until', 50, 'tau_a', 0.06, 'a_d', ad, 'descent_v', 15 - hs * Rr, 'descent_T', 1, 'post_wait', [0 1], 'ap_q', true, 'ap_floor_law', true, 'sP_fix_theta_deg', 2, 'q_geo', [0.9 0.9], 'tag', 'ni', 'du_d', 40 * Ts / Rr, 'kappa_state', true, 'kappa_from_scan', true, 'kappa_pfix', 0.3, 'scan_ramp', 0.5, 'scan_len', 1e6, 'T', 10);
[IA, ID] = ndgrid(1:2, 1:NS);  N = numel(IA);  rs = cell(N, 1);  if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end;  pctRunOnAll warning('off', 'all');
parfor q = 1:N
    o = base;  o.seeds = ID(q);  if IA(q) == 2; o.dither = [3 8 2];  o.dither_win = [0 50]; end;  R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t(:), 'nu', r.nu1, 'S', r.Sy3);
end
nm = {'no motion', 'circle 3 um'};  ax = {'u', 'v', 'w'};
for a = 1:2
    for c = 1:3
        v = [];  l1 = [];  l2 = [];  l3 = [];  sg = [];
        for s = find(IA == a)'
            r = rs{s};  n = min(size(r.nu, 1), size(r.S, 1));  w = find(r.t(1:n) >= 4 & isfinite(r.nu(1:n, c)));  y = r.nu(w, c);  z = y ./ sqrt(r.S(w, c));  v = [v; mean(z.^2)];  l1 = [l1; corr(y(1:end - 1), y(2:end))];  l2 = [l2; corr(y(1:end - 2), y(3:end))];  l3 = [l3; corr(y(1:end - 3), y(4:end))];  sg = [sg; sqrt(mean(r.S(w, c)))];
        end
        fprintf('%-12s axis %s: nu^2 / S %.3f (seeds %s) | lag-1 %+.3f lag-2 %+.3f lag-3 %+.3f | model sigma %.5f\n', nm{a}, ax{c}, mean(v), mat2str(round(v', 3)), mean(l1), mean(l2), mean(l3), mean(sg));
    end
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
