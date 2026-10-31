% STATUS: SCRATCH -- 2026-10-29 (user): (1) why the slow full turn (5 um/s, + kappa) nearly touches the wall: flow A = hold 10 s,
%   constant-speed descent 10-15 s, scan + regulation from 15 s | flow B = hold 10 s, regulation 10-17 s, scan from 17 s;
%   (2) faster integral: P | PI tau_i = 8 s (critical) | 2 s | 0.5 s (tau_a 2 s). Full turn of the void sphere, seeds 1-3.
%   Console per arm: min gap, time of the min, a-hat_33 - a_33 and a'-hat_33 / true slope just before the min, offset after the first quarter.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);  vd = (15 - hs * Rr) / 5;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 2, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'ap_pfix_law', true, 'kappa_state', true, 'scan_len', Lturn, 'tag', 'sp');
% arms: {name, speed, P_fix, flow (A/B), tau_i (0 = P)}
arms = {'5 A P', 5, 1, 'A', 0; '5 B P', 5, 1, 'B', 0; '5 A PI 8', 5, 1, 'A', 8; '5 A PI 2', 5, 1, 'A', 2; '5 A PI 0.5', 5, 1, 'A', 0.5; ...
        '40 A P', 40, 2, 'A', 0; '40 A PI 2', 40, 2, 'A', 2; '40 A PI 0.5', 40, 2, 'A', 0.5};
na = size(arms, 1);  [IA, ID] = ndgrid(1:na, 1:3);  out = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    a = arms(IA(q), :);  o = base;  sp = a{2};  o.du_d = sp * Ts / Rr;  o.sP_fix_theta_deg = a{3};  o.seeds = ID(q);
    if strcmp(a{4}, 'A'); tS = 15;  o.descent_v = vd;  o.descent_T = 5; else; tS = 17;  o.t_scan = tS; end
    o.q_theta_from = tS;  o.T = tS + Lturn / sp + 0.2;  if a{5} > 0; o.reg_pi = true;  o.tau_i = a{5}; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    ks = find(r.t >= tS & r.t <= tS + Lturn / sp & (1:numel(r.t))' <= r.kend);  [hm, im] = min(r.h(ks));  m = ks(im);
    mb = ks(max(1, im - round(0.3 / Ts)):im);   % 0.3 s before the minimum
    sv = zeros(numel(mb), 1);  for j = 1:numel(mb); h = max(r.h(mb(j)), 1.0003); [~, c1] = calc_correction_functions(h + 1e-4); [~, c0] = calc_correction_functions(h - 1e-4); sv(j) = (1 / c1 - 1 / c0) / 2e-4; end
    k2 = ks(ks >= ks(1) + round(0.25 * numel(ks)));
    out{q} = [r.contact, hm * Rr, r.t(m) - tS, mean(r.aN(mb) - r.aN_tr(mb)), mean(r.apN(mb)) / mean(sv), mean(r.aN(k2)) - ad, mean(r.aN_tr(k2)) - ad];
end
for ia = 1:na
    M = cell2mat(out(ia, :)');
    fprintf('%-11s: contact %d | min gap %s um at %s s | before the min: a-hat - a %s, a''-hat/true %s | after 1/4: a-hat - a_d %s, a - a_d %s\n', arms{ia, 1}, sum(M(:, 1)), ...
            mat2str(round(M(:, 2)', 2)), mat2str(round(M(:, 3)', 1)), mat2str(round(M(:, 4)', 3)), mat2str(round(M(:, 5)', 2)), mat2str(round(M(:, 6)', 3)), mat2str(round(M(:, 7)', 3)));
end
save(fullfile(fo, 'temp_1029_slow_pi.mat'), 'arms', 'out');

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
