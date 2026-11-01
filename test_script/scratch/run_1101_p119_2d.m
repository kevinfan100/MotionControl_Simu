% STATUS: SCRATCH -- 2026-11-01 (user): 2-D split of P(a_33, delta w_3)[k+1] (log L.D119): rows of F_e row 11 (1, a' F_w, position, off-diagonal, slope) x columns of F_e row 9 (lc, -F_w, -F_u, -F_v), + Q(11,9).
%   Plane, V-hat true and frozen, kappa off, 5 seeds. Windows: hold (t_scan - 0.9 .. 0), onset (t_scan + 0.15 .. + 0.5, a'-hat not fallen yet), late (+1 .. +3). Arms: P alive 40 | P alive 10 | C 40.
%   Pre-registered: the cell that moves P(11,9) up at onset at P alive 40 and not at C 40, >= half of the change of the sum, carries the flip. Sanity: sum of cells + Q = P(11,9)[k+1], ~ 0.
%   Console: cells x 1e6 (mean per step, mean of seeds), a'-hat, mean F_u, mean P(a_33,a_33), P(a_33,a_uw), P(dw3,dw3), P(a_uw,a_uw).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
Rr = 2.25;  ad = 0.05;  Ts = 1 / 1612;  tS = 12;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', 15, 'tag', 'd2', 'freeze_angle', true);
arm = {'P alive, 40', 'P alive, 10', 'C, 40'};  NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 2; o.du_d = 10 * Ts / Rr; end
    if IA(q) == 3; o.sP_theta_deg = 1e-3;  o.q_theta_off = true; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'D', r.D119, 'Q', r.B119(:, 6), 'P9', r.B119(:, 9), 'S', r.P1112, 'ap', r.apN, 'Fu', r.vF(:, 1));
end
wn = {'hold', 'onset', 'late'};  w0 = [-0.9 0.15 1];  w1 = [0 0.5 3];  ri = {'1', 'a''F_w', 'pos', 'off', 'slope'};  cj = {'lc', '-F_w', '-F_u', '-F_v'};
for ia = 1:NA
    for iw = 1:3
        M = NaN(NS, 20);  q = NaN(NS, 1);  ex = NaN(NS, 7);  cl = 0;
        for id = 1:NS
            r = rs{ia, id};  kk = (1:r.kend - 1)';  g = kk(r.t(kk) >= tS + w0(iw) & r.t(kk) < tS + w1(iw));  M(id, :) = mean(r.D(g, :), 1);  q(id) = mean(r.Q(g));
            ex(id, :) = [mean(r.ap(g)), mean(r.Fu(g)), mean(r.S(g, :), 1), mean(r.P9(g))];  cl = max(cl, max(abs(sum(r.D(g, :), 2) + r.Q(g) - r.P9(g + 1))));
        end
        T = reshape(1e6 * mean(M, 1), 5, 4);
        fprintf('%s, %s | closure %.1e | a''-hat %.2f, F_u %+.4f | P(a33,a33) %.2e, P(a33,a_uw) %+.2e, P(dw3,dw3) %.2e, P(a_uw,a_uw) %.2e | P(11,9) %+.2f | Q %+.2f\n', arm{ia}, wn{iw}, cl, mean(ex(:, 1:6), 1), 1e6 * mean(ex(:, 7)), 1e6 * mean(q));
        fprintf('            %s\n', sprintf('%9s', cj{:}));  for i = 1:5; fprintf('   %-7s %s\n', ri{i}, sprintf('%+9.3f', T(i, :))); end
    end
end

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
