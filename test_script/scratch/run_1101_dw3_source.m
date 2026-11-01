% STATUS: SCRATCH -- 2026-11-01 (user): what biases delta-w-hat_3 (slot 9) at 40 um/s with the off-diagonal P alive. x(9)[k+1] = lc x(9)[k] + Lg(9,:) nu, so
%   (1 - lc) sum x(9) = sum feedback - (x(9)[end] - x(9)[start]) exactly (frame frozen, no rotation). Feedback split: lc K9 nu, -F_w K11 nu, -F_u K12 nu, -F_v K13 nu (log L.fb9).
%   Plane theta_y 10, current flow, kappa off, frame true and frozen, 5 seeds. Arms: P alive 40 | P alive 10 | C 40 | P alive 40, control without a_uw-hat. Window t_scan .. +3 s.
%   Pre-registered: hypothesis holds if at 40 um/s the -F_u term has one sign and >= half of (1 - lc) sum x(9), ~0 in C, clearly smaller at 10 um/s. Sanity: closure error ~ 0.
%   Console per seed. Figure fig_1101_dw3_source.png (seed 5, cumulative).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  Ts = 1 / 1612;  tS = 12;  Tw = 3;  FS = 14;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', tS + Tw, 'tag', 'd3', 'freeze_angle', true);
arm = {'P alive, 40 um/s', 'P alive, 10 um/s', 'C: no P, 40 um/s', 'P alive, 40, control without a_{uw}'};  NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 2; o.du_d = 10 * Ts / Rr; end
    if IA(q) == 3; o.sP_theta_deg = 1e-3;  o.q_theta_off = true; end
    if IA(q) == 4; o.ctrl_no_off_from = tS; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'lc', r.opts.lambda_c, 'fb', r.fb9);
end
nm = {'lc K9 nu', '-F_w K11 nu', '-F_u K12 nu', '-F_v K13 nu'};
for ia = 1:NA
    M = NaN(NS, 7);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tw);  ks = ks(1:end - 1);
        lc = r.lc;  sx = (1 - lc) * sum(r.fb(ks, 6));  dx = r.fb(ks(end) + 1, 6) - r.fb(ks(1), 6);
        M(id, :) = [sum(r.fb(ks, 1:4), 1), sum(r.fb(ks, 5)), sx, sx - (sum(r.fb(ks, 5)) - dx)];
    end
    fprintf('%s\n   (1-lc) sum x(9) %s R, closure max %.1e\n', arm{ia}, mat2str(round(M(:, 6)', 3)), max(abs(M(:, 7))));
    for c = 1:4; fprintf('   %-12s %s\n', nm{c}, mat2str(round(M(:, c)', 3))); end
end
f = figure('Color', 'w', 'Position', [20 20 520 * NA 620]);  tl = tiledlayout(1, NA, 'TileSpacing', 'compact', 'Padding', 'compact');  cs = lines(4);
for ia = 1:NA
    r = rs{ia, 5};  kk = (1:r.kend)';  k = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tw);  t = r.t(k) - tS;  a = nexttile(tl, ia);  hold on;  box on;
    for c = 1:4; plot(t, cumsum(r.fb(k, c)), 'Color', cs(c, :), 'LineWidth', 1.5, 'DisplayName', nm{c}); end
    plot(t, (1 - r.lc) * cumsum(r.fb(k, 6)), 'k-', 'LineWidth', 2, 'DisplayName', '(1 - \lambda_c) sum \delta w_3 estimate');  yline(0, 'k:', 'HandleVisibility', 'off');
    ylabel(sprintf('cumulative [R], %s', arm{ia}));  xlabel('time since t_{scan}  (sec)');  legend('Location', 'northoutside', 'NumColumns', 2);
    set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
end
fn = fullfile(fo, 'fig_1101_dw3_source.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
