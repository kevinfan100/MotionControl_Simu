% STATUS: SCRATCH -- 2026-10-28 (user): angle lag or a' -- which one drives the gap drift during the sphere scan. Reference arms only
%   (not for the conclusion chain): A regulation slope = true d a_33/dh (opts.cheat_ap) | B regulation and scan direction use the true
%   normal (opts.cheat_w) | A+B. Settled flow (whole sphere radius 15 um void, start 10 deg off the top, hold 10 s, regulation tau_a 2 s to
%   a_d 0.05, great-circle scan from 17 s, slope L), one full turn at 5 um/s (angle P_fix 2 deg) and 40 um/s (4 deg), seeds 1-3.
%   Console: fraction of the turn on the sphere (gap < 5 um), gap at 25 / 50 / 75 / 100 % of the turn, mean angle lag (signed).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  tS = 17;  ph0 = 10;  Ts = 1 / 1612;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
              'scan_dir', [1 0], 'hemi_start_deg', ph0, 'scan_plane_n', [0 1 0], 'ap_pfix_law', true, 'scan_len', Lturn, 'tag', 'ch');
arms = {'as is', 'A: true slope in the regulation', 'B: true normal for regulation + scan', 'A + B'};  SP = [5 40];  PF = [2 4];
[IA, IS, ID] = ndgrid(1:4, 1:2, 1:3);  res = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  sp = SP(IS(q));  o.du_d = sp * Ts / Rr;  o.sP_fix_theta_deg = PF(IS(q));  o.T = tS + Lturn / sp + 0.2;  o.seeds = ID(q);
    if IA(q) == 2 || IA(q) == 4; o.cheat_ap = true; end
    if IA(q) >= 3; o.cheat_w = true; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    ks = find(r.t >= tS & r.t <= tS + Lturn / sp & (1:numel(r.t))' <= r.kend);  gap = r.h(ks) * Rr;
    bad = find(gap > 5 | isnan(r.aN(ks)), 1);  if isempty(bad); kon = numel(ks); else; kon = bad - 1; end
    ty = unwrap(r.th_tr(1:r.kend, 2) * pi / 180) * 180 / pi;  lag = ty(ks(1:kon)) - r.thy(ks(1:kon));
    gq = NaN(1, 4);  for j = 1:4; m = round(j / 4 * numel(ks)); if m <= kon; gq(j) = gap(m); end; end
    res{q} = struct('frac', kon / numel(ks), 'gq', gq, 'lag', mean(lag), 'a33', rms(r.aN(ks(1:kon)) - r.aN_tr(ks(1:kon))));
end
for is = 1:2
    for ia = 1:4
        R3 = [res{ia, is, :}];
        fprintf('%2d um/s %-38s: on the sphere %s | gap at 1/4 1/2 3/4 1 turn (seed 1) %s um | mean lag %s deg | rms a33 err %s\n', SP(is), arms{ia}, ...
                mat2str(round([R3.frac], 2)), mat2str(round(R3(1).gq, 2)), mat2str(round([R3.lag], 1)), mat2str(round([R3.a33], 3)));
    end
end

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
