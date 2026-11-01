% STATUS: SCRATCH -- 2026-11-01 (user): step 1 of narrowing open item 1: does a_uw-hat act on a'-hat_33 through real motion (control law uses it) or inside the filter only.
%   Plane theta_y 10, current flow, kappa off, frame true and frozen, off-diagonal P alive, 40 um/s, 5 seeds. Arms: base (arm 3 of run_1031_ap_angle_split) | control law without
%   a_uw-hat, a_vw-hat from t_scan (opts.ctrl_no_off_from, diagnostic: plant no longer matches F_e) | C (no off-diagonal P).
%   Pre-registered: real motion if a'/true >= 1 and every |sum y1-w update| <= 0.5 slope; filter bookkeeping if a'/true <= 0.7 and the push keeps its sign.
%   Sanity: base reproduces fractions [0.01 0 0.23 0 0], a'/true 0.52; new arm equals base before t_scan.
%   Console: fraction a' < 0, a'/true, |a33 - a_d|, y1-w update of a' and of a33, rms a_uw-hat, rms F_w, max |base - new| of a-hat_33 before t_scan. Figure fig_1101_ap_ctrl_off.png (seed 5).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  Ts = 1 / 1612;  tS = 12;  Tw = 3;  FS = 14;  cGr = [0.5 0.5 0.5];  cE = [0 0.25 0.85];  cT = [0.85 0 0];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', tS + Tw, 'tag', 'co', 'freeze_angle', true);
arm = {'base: P alive', 'control without a_{uw}, a_{vw}', 'C: no P'};  NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 2; o.ctrl_no_off_from = tS; end
    if IA(q) == 3; o.sP_theta_deg = 1e-3;  o.q_theta_off = true; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'apN', r.apN, 'h', r.h, 'aN', r.aN, 'aN_tr', r.aN_tr, 'da33', r.da33, 'dap', r.dap, 'off', r.off_post, 'vF', r.vF);
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);  rm = @(x) sqrt(mean(x.^2));  ap0 = slope(hs);
for ia = 1:NA
    M = NaN(NS, 7);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS + 0.5 & r.t(kk) <= tS + Tw);  k0 = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tw);
        M(id, :) = [mean(r.apN(ks) < 0), median(r.apN(ks) ./ slope(r.h(ks))), mean(abs(r.aN_tr(ks) - ad)), sum(r.dap(k0, 3)) / ap0, sum(r.da33(k0, 3)), rm(r.off(ks, 1)), rm(r.vF(ks, 3))];
    end
    fprintf('%-30s | a''<0 %s | a''/true %.2f | |a33-a_d| %.3f | y1-w on a''/slope %s | y1-w on a33 %s | rms a_uw-hat %.4f | rms F_w %.4f\n', arm{ia}, mat2str(round(M(:, 1)', 2)), median(M(:, 2)), median(M(:, 3)), ...
            mat2str(round(M(:, 4)', 2)), mat2str(round(M(:, 5)', 3)), mean(M(:, 6)), mean(M(:, 7)));
end
dp = 0;  for id = 1:NS; a = rs{1, id};  b = rs{2, id};  g = a.t(1:min(a.kend, b.kend)) < tS - 2 * Ts;  dp = max(dp, max(abs(a.aN(g) - b.aN(g)))); end
fprintf('max |base - new| a-hat_33 before t_scan: %.3g\n', dp);
f = figure('Color', 'w', 'Position', [20 20 650 * NA 800]);  tl = tiledlayout(2, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    r = rs{ia, 5};  kk = (1:r.kend)';  k = kk(r.t(kk) >= tS - 1);  kd = k(1:5:end);  t = r.t(kd) - tS;  ks0 = k(r.t(k) >= tS);
    a1 = nexttile(tl, ia);  hold on;  box on;  plot(t, r.aN_tr(kd), 'Color', cT, 'LineWidth', 1.2, 'DisplayName', 'a_{33} / a_{nom}');  plot(t, r.aN(kd), 'Color', cE, 'LineWidth', 1.0, 'DisplayName', arm{ia});
    yline(ad, 'k--', 'HandleVisibility', 'off');  ylim([0 0.15]);  ylabel('a_{33} / a_{nom}');  legend('Location', 'northoutside', 'Orientation', 'horizontal');
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  plot(t, r.apN(kd) ./ slope(r.h(kd)), 'Color', cE, 'LineWidth', 1.3, 'DisplayName', 'estimate / true slope');
    plot(r.t(ks0) - tS, cumsum(r.dap(ks0, 3)) / ap0, 'Color', [0.93 0.69 0.13], 'LineWidth', 1.5, 'DisplayName', 'cumulative y_1-w update / slope');  yline(1, 'r-', 'LineWidth', 1.4, 'HandleVisibility', 'off');
    ylim([-2 3]);  ylabel('estimate / true slope');  xlabel('time since t_{scan}  (sec)');  legend('Location', 'northoutside', 'Orientation', 'horizontal');
    for a = [a1 a2]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-1 Tw]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1101_ap_ctrl_off.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
