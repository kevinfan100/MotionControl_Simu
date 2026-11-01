% STATUS: SCRATCH -- 2026-11-01 (user): is M-hat_w (estimated normal motion, F_e(11,15), fx(11)) a false motion. Sum over t_scan .. +3 s of M-hat_w split into the commanded step (hd_int) and
%   (1 - lambda_c) delta-w-hat_3, vs the true gap change. Pre-registered: sum M-hat_w differs from the true change, and sum a'-hat M-hat_w ~ -(y1-w update of a-hat_33).
%   Plane theta_y 10, current flow, kappa off, frame true and frozen, 5 seeds. Arms: P alive 40 | P alive 10 | C 40 | P alive 40, control without a_uw-hat.
%   Console per seed. Figure fig_1101_false_motion.png (seed 5, cumulative).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  Ts = 1 / 1612;  tS = 12;  Tw = 3;  FS = 14;  cE = [0 0.25 0.85];  cT = [0.85 0 0];  cY = [0.93 0.69 0.13];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', tS + Tw, 'tag', 'fm', 'freeze_angle', true);
arm = {'P alive, 40 um/s', 'P alive, 10 um/s', 'C: no P, 40 um/s', 'P alive, 40, control without a_{uw}'};  NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 2; o.du_d = 10 * Ts / Rr; end
    if IA(q) == 3; o.sP_theta_deg = 1e-3;  o.q_theta_off = true; end
    if IA(q) == 4; o.ctrl_no_off_from = tS; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'h', r.h, 'vMh', r.vMh, 'hd_int', r.hd_int, 'apN', r.apN, 'da33', r.da33, 'off', r.off_post, 'dp', r.dp);
end
for ia = 1:NA
    M = NaN(NS, 7);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tw);  ks = ks(1:end - 1);
        cmd = r.hd_int(ks + 1) - r.hd_int(ks);  Mw = r.vMh(ks, 3);
        M(id, :) = [sum(Mw), sum(cmd), sum(Mw - cmd), r.h(ks(end) + 1) - r.h(ks(1)), sum(r.apN(ks) .* Mw), sum(r.da33(ks, 3)), 1e3 * mean(r.off(ks, 1))];
    end
    fprintf('%s\n   sum M-hat_w %s R\n   commanded %s | (1-lc) dw3-hat %s\n   true gap change %s R\n   sum a''-hat M-hat_w %s\n   y1-w update of a33 %s\n   mean a_uw-hat x 1e3 %s\n', arm{ia}, mat2str(round(M(:, 1)', 3)), ...
            mat2str(round(M(:, 2)', 3)), mat2str(round(M(:, 3)', 3)), mat2str(round(M(:, 4)', 3)), mat2str(round(M(:, 5)', 3)), mat2str(round(M(:, 6)', 3)), mat2str(round(M(:, 7)', 2)));
end
f = figure('Color', 'w', 'Position', [20 20 520 * NA 620]);  tl = tiledlayout(1, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 5};  kk = (1:r.kend)';  k = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tw);  k = k(1:end - 1);  t = r.t(k) - tS;  a = nexttile(tl, ia);  hold on;  box on;
    plot(t, r.h(k) - r.h(k(1)), 'Color', cT, 'LineWidth', 1.2, 'DisplayName', 'true gap change');
    plot(t, cumsum(r.vMh(k, 3)), 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'estimated: sum M_w');
    plot(t, -cumsum(r.da33(k, 3)), 'Color', cY, 'LineWidth', 1.6, 'DisplayName', '-(y_1-w update of a_{33})');  yline(0, 'k:', 'HandleVisibility', 'off');
    ylim([-0.25 0.25]);  ylabel(sprintf('[R], %s', arm{ia}));  xlabel('time since t_{scan}  (sec)');  legend('Location', 'northoutside', 'NumColumns', 1);
    set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
end
fn = fullfile(fo, 'fig_1101_false_motion.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
