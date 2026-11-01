% STATUS: SCRATCH -- 2026-10-31 (user): which covariance feeds P(a'_33, delta w) when the off-diagonal P is alive (mainline-3d open item 1). Plane theta_y 10, current flow,
%   kappa off, frame true and frozen, 5 seeds. P(a', delta w_3) at k+1 = lambda_c P(a', delta w_3) + T11 + T12 + T13 with T11 = -F_w P(a', a_33), T12 = -F_u P(a', a_uw),
%   T13 = -F_v P(a', a_vw) (log L.T15). Arms: 3 at 40 um/s | 3 at 10 um/s | C at 40 um/s (no off-diagonal P, negative control).
%   Sanity before reading: arm C must reproduce run_1031_ap_angle_split (fraction 0, a33 error 0.007) and rms T12 must be ~0 there; rms F_u is 4x between the speeds.
%   Pre-registered: T12 path if rms T12 >= rms T11 at 40 um/s and at least 2x its 10 um/s value; otherwise F_w P(a', a_33) stays the main feed.
%   Console: fraction a'-hat < 0, a'-hat/true, rms T11 T12 T13, rms corr(a', a_33), corr(a', a_uw), summed y1-w update. Figure fig_1031_ap_path.png (seed 5).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  Ts = 1 / 1612;  tS = 12;  Tw = 3;  FS = 14;  cGr = [0.5 0.5 0.5];  cE = [0 0.25 0.85];  cs = [0 0.25 0.85; 0.85 0 0; 0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', tS + Tw, 'tag', 'ph', 'freeze_angle', true);
arm = {'3: 40 um/s, P alive', '3: 10 um/s, P alive', 'C: 40 um/s, no P', '3: 40 um/s, true readout + R noise'};  NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 2; o.du_d = 10 * Ts / Rr; end
    if IA(q) == 3; o.sP_theta_deg = 1e-3;  o.q_theta_off = true; end
    if IA(q) == 4; o.ym_cheat = 'true_noise';  o.ym_cheat_from = tS; end   % reference arm: y1-w vs readout conflict; supported if all |sum y1-w update| <= 0.5 slope and a'/true >= 1
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'apN', r.apN, 'h', r.h, 'aN_tr', r.aN_tr, 'dap', r.dap, 'T15', r.T15, 'c15', r.c15, 'vF', r.vF);
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);  rm = @(x) sqrt(mean(x.^2));  ap0 = slope(hs);
for ia = 1:NA
    M = NaN(NS, 11);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS + 0.5 & r.t(kk) <= tS + Tw);
        M(id, :) = [mean(r.apN(ks) < 0), median(r.apN(ks) ./ slope(r.h(ks))), mean(abs(r.aN_tr(ks) - ad)), rm(r.vF(ks, 1)), rm(r.T15(ks, 1)), rm(r.T15(ks, 2)), rm(r.T15(ks, 3)), rm(r.c15(ks, 3)), rm(r.c15(ks, 4)), rm(r.c15(ks, 5)), sum(r.dap(ks, 3)) / ap0];
    end
    fprintf('%-20s | a''<0 %s | a''/true %.2f | |a33-a_d| %.3f | rms F_u %.4f | rms T11 %.2e T12 %.2e T13 %.2e | rms corr(a'',a33) %.3f corr(a'',a_uw) %.3f corr(a'',a_vw) %.3f | sum y1-w update/slope %s\n', arm{ia}, ...
            mat2str(round(M(:, 1)', 2)), median(M(:, 2)), median(M(:, 3)), mean(M(:, 4)), mean(M(:, 5)), mean(M(:, 6)), mean(M(:, 7)), mean(M(:, 8)), mean(M(:, 9)), mean(M(:, 10)), mat2str(round(M(:, 11)', 2)));
end
f = figure('Color', 'w', 'Position', [20 20 650 * NA 1050]);  tl = tiledlayout(3, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 5};  kk = (1:r.kend)';  k = kk(r.t(kk) >= tS - 1);  t = r.t(k) - tS;  ks0 = k(r.t(k) >= tS);
    a1 = nexttile(tl, ia);  hold on;  box on;  plot(t, r.apN(k) ./ slope(r.h(k)), 'Color', cE, 'LineWidth', 1.3, 'DisplayName', arm{ia});  yline(1, 'r-', 'LineWidth', 1.4, 'HandleVisibility', 'off');
    plot(r.t(ks0) - tS, cumsum(r.dap(ks0, 3)) / ap0, 'Color', [0.93 0.69 0.13], 'LineWidth', 1.5, 'DisplayName', 'cumulative y_1-w update / slope');  ylim([-2 3]);  ylabel('estimate / true slope');
    legend('Location', 'northoutside', 'Orientation', 'horizontal');
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  nm = {'F_w P(a'', a_{33})', 'F_u P(a'', a_{uw})', 'F_v P(a'', a_{vw})'};
    for c = 1:3; plot(t, movmean(r.T15(k, c), 80), 'Color', cs(c, :), 'LineWidth', 1.3, 'DisplayName', nm{c}); end
    ylabel('sources of P(a'', \delta w_3)');  legend('Location', 'northoutside', 'Orientation', 'horizontal');
    a3 = nexttile(tl, 2 * NA + ia);  hold on;  box on;  plot(t, r.c15(k, 3), 'Color', cs(1, :), 'LineWidth', 1.3, 'DisplayName', 'corr(a'', a_{33})');
    plot(t, r.c15(k, 4), 'Color', cs(2, :), 'LineWidth', 1.3, 'DisplayName', 'corr(a'', a_{uw})');  ylim([-1 1]);  ylabel('correlation');  xlabel('time since t_{scan}  (sec)');
    legend('Location', 'northoutside', 'Orientation', 'horizontal');
    for a = [a1 a2 a3]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-1 Tw]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1031_ap_path.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
