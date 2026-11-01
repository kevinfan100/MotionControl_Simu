% STATUS: SCRATCH -- 2026-11-01 (user): where P(a_33, delta w) flips along delta w_3 -> delta w_2 -> delta w_1. Plane, V-hat true and frozen, kappa off, 5 seeds. Budget (log L.C119, L.B119):
%   Pu(11,9) = Pk(11,9) + sum u9 (7 channels); Pk+(11,6) = sum p6 (groups of row 11: position, a_33, off-diagonal, slope, rest); Pu(11,6) = Pk(11,6) + sum u6; Pk+(11,3) = sum p3.
%   Pre-registered: the term that changes sign or grows the most from hold to scan at P alive 40, and not at C 40, carries the flip. Sanity: closure of the four identities ~ 0.
%   Console: mean per step x 1e6, hold (t_scan - 0.9 .. 0) and scan (t_scan + 0.3 .. + 3), mean over seeds. Arms: P alive 40 | P alive 10 | C 40.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
Rr = 2.25;  ad = 0.05;  Ts = 1 / 1612;  tS = 12;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', 15, 'tag', 'cb', 'freeze_angle', true);
arm = {'P alive, 40', 'P alive, 10', 'C, 40'};  NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 2; o.du_d = 10 * Ts / Rr; end
    if IA(q) == 3; o.sP_theta_deg = 1e-3;  o.q_theta_off = true; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'B', r.B119, 'C', r.C119);
end
wn = {'hold', 'scan'};  ch = {'y1u', 'y1v', 'y1w', 'ym11', 'ym33', 'ymuw', 'ymvw'};  gp = {'pos', 'a33', 'off', 'slope', 'rest'};
for ia = 1:NA
    for iw = 1:2
        A = NaN(NS, 7);  B = A;  P6 = NaN(NS, 5);  P3 = P6;  L = NaN(NS, 3);  cl = 0;
        for id = 1:NS
            r = rs{ia, id};  kk = (1:r.kend - 1)';  if iw == 1; g = kk(r.t(kk) >= tS - 0.9 & r.t(kk) < tS); else; g = kk(r.t(kk) >= tS + 0.3 & r.t(kk) <= tS + 3 - Ts); end
            C = r.C;  A(id, :) = mean(C(g, 1:7), 1);  B(id, :) = mean(C(g, 8:14), 1);  P6(id, :) = mean(C(g, 15:19), 1);  P3(id, :) = mean(C(g, 20:24), 1);  L(id, :) = [mean(r.B(g, 9)), mean(C(g, 25)), mean(r.B(g, 8))];
            cl = max([cl, max(abs(sum(C(g, 15:19), 2) - C(g + 1, 25))), max(abs(sum(C(g, 20:24), 2) - r.B(g + 1, 8)))]);
        end
        fprintf('%s, %s | closure %.1e | Pk(11,9) %+.2f  Pk(11,6) %+.2f  Pk(11,3) %+.2f\n', arm{ia}, wn{iw}, cl, 1e6 * mean(L, 1));
        fprintf('   update of P(11,9): ');  for c = 1:7; fprintf('%s %+.2f  ', ch{c}, 1e6 * mean(A(:, c))); end;  fprintf('\n   predict P(11,6):   ');  for c = 1:5; fprintf('%s %+.2f  ', gp{c}, 1e6 * mean(P6(:, c))); end
        fprintf('\n   update of P(11,6): ');  for c = 1:7; fprintf('%s %+.2f  ', ch{c}, 1e6 * mean(B(:, c))); end;  fprintf('\n   predict P(11,3):   ');  for c = 1:5; fprintf('%s %+.2f  ', gp{c}, 1e6 * mean(P3(:, c))); end;  fprintf('\n');
    end
end

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
