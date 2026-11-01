% STATUS: SCRATCH -- 2026-11-01 (user): step 2b of narrowing open item 1: exact budget of P(a'_33, a_33) per step, from log L.B15. Prediction by column of row 11 of F_e:
%   position (7:9), a_11, a_33 self (F_e(11,11) - 1), off-diagonal (12:13), slope (M-hat_w P_a'), rest; update: y1 channels, gain readouts. P[k+1] - P[k] = sum of these (closure check).
%   Plane theta_y 10, current flow, kappa off, frame true and frozen, 5 seeds. Arms: P alive 40 | P alive 10 | C 40 (no off-diagonal P). Window t_scan .. +3 s.
%   Pre-registered: the source is the term most negative at P alive 40 and not as negative in the two other arms. Sanity: closure error ~ 0; off-diagonal term ~ 0 in C.
%   Console: sum over the window of each term, mean over seeds. Figure fig_1101_p1511_budget.png (seed 5, cumulative terms).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  Ts = 1 / 1612;  tS = 12;  Tw = 3;  FS = 14;  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', tS + Tw, 'tag', 'bg', 'freeze_angle', true);
arm = {'P alive, 40 um/s', 'P alive, 10 um/s', 'C: no P, 40 um/s'};  NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 2; o.du_d = 10 * Ts / Rr; end
    if IA(q) == 3; o.sP_theta_deg = 1e-3;  o.q_theta_off = true; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'B', r.B15);
end
nm = {'position', 'a_{11}', 'a_{33} self', 'off-diagonal', 'slope M_w', 'y_1 update', 'readout update'};  ic = [1:7 9];
for ia = 1:NA
    M = NaN(NS, 10);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tw);  ks = ks(1:end - 1);
        S = sum(r.B(ks, ic), 1);  dP = r.B(ks(end) + 1, 8) - r.B(ks(1), 8);  M(id, :) = [S, dP, dP - sum(S)];
    end
    fprintf('%s (x 1e-3, sum over the window, mean of %d seeds):\n', arm{ia}, NS);
    lb = [nm, {'rest'}];  for c = 1:8; fprintf('   %-16s %+8.3f   seeds %s\n', lb{c}, 1e3 * mean(M(:, c)), mat2str(round(1e3 * M(:, c)', 2))); end
    fprintf('   net change %+.3f, closure error max %.2e\n', 1e3 * mean(M(:, 9)), max(abs(M(:, 10))));
end
f = figure('Color', 'w', 'Position', [20 20 650 * NA 560]);  tl = tiledlayout(1, NA, 'TileSpacing', 'compact', 'Padding', 'compact');  cs = lines(7);
for ia = 1:NA
    r = rs{ia, 5};  kk = (1:r.kend)';  k = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tw);  t = r.t(k) - tS;  a = nexttile(tl, ia);  hold on;  box on;
    for c = 1:7; plot(t, 1e3 * cumsum(r.B(k, c)), 'Color', cs(c, :), 'LineWidth', 1.5, 'DisplayName', nm{c}); end
    plot(t, 1e3 * (r.B(k, 8) - r.B(k(1), 8)), 'k-', 'LineWidth', 2, 'DisplayName', 'net');  yline(0, 'k:', 'HandleVisibility', 'off');
    ylabel(sprintf('cumulative P(a'', a_{33}) x 1e3, %s', arm{ia}));  xlabel('time since t_{scan}  (sec)');  legend('Location', 'northoutside', 'NumColumns', 4);
    set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
end
fn = fullfile(fo, 'fig_1101_p1511_budget.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
