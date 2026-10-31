% STATUS: SCRATCH -- 2026-10-29 (user, design-panel steps 1-2, log only): (P0) share of the off-diagonal compensation
%   c_k = -a_uw-hat+ M_u / (a_11-hat a_33-hat - a_uw-hat+^2) in the normal force step vF(3); (D/D') the known control-gain term m missing from
%   fx(7:9) (control uses the gains after the feedback, delayed forces use the old gains): m-tilde_w = m_w through z[k+1] = lambda_c z + m,
%   delayed 2 steps; D = sum K(15,w) nu_w, D' = sum K(15,w) (nu_w - m-tilde_w), regression slope of nu_w on m-tilde_w.
%   Current flow (hold 10 s, descent 1 s, regulation 1 s, scan from 12 s, ramp 0.5 s, kappa from t_scan, tau_a 0.5 s, angle P_fix 2 deg,
%   report slope P_fix, threefry), 3 s of scan. Arms: sphere 40 | plane (theta_y 10) 40 | sphere 10 um/s, seeds 1-10.
%   Pre-registered: P0 passes if the share >= 50 % at 40 um/s on both walls (and smaller at 10 um/s); D/D' passes if |D'| <= 0.8 |D| and
%   slope >= 0.5. Figure fig_1029_p0_dd.png (seed 5 per arm).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  Tr = 0.5;  tS = 12;  Tw = 3;  FS = 13;  cE = [0 0.25 0.85];  cT = [0.85 0 0];  cGr = [0.5 0.5 0.5];  lc = 0.4;
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'kappa_from_scan', true, 'scan_ramp', Tr, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, ...
              'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, 'post_wait', [0 1], 'T', tS + Tw, 'tag', 'pd');
arm = {'sphere 40 um/s', 'plane 40 um/s', 'sphere 10 um/s'};  NA = 3;  NS = 10;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 2; o = rmfield(o, {'wall', 'hemi_r', 'hemi_void', 'hemi_start_deg', 'scan_plane_n'});  o.theta_y = 10; end
    if IA(q) == 3; o.du_d = 10 * Ts / Rr; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    lc_q = r.opts.lambda_c;  rs{q} = struct('t', r.t, 'kend', r.kend, 'vF', r.vF, 'vMh', r.vMh, 'mk', r.mk, 'off_post', r.off_post, 'aT', r.aT, 'aN', r.aN, 'k15w', r.k15w(:), 'nuw', r.nuw(:), 'lc', lc_q);
end
lc = rs{1, 1}.lc;  fprintf('lambda_c = %.2f\n', lc);
P = cell(1, NA);
for ia = 1:NA
    M = NaN(NS, 6);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS + 0.5 & r.t(kk) <= tS + Tw);
        au = r.off_post(:, 1);  c = -au .* r.vMh(:, 1) ./ (r.aT .* r.aN - au.^2);
        z = filter(1, [1 -lc], r.mk(:, 3));  mt = [0; 0; z(1:end-2)];
        D = sum(r.k15w(ks) .* r.nuw(ks));  Dp = sum(r.k15w(ks) .* (r.nuw(ks) - mt(ks)));  b = (mt(ks) - mean(mt(ks))) \ (r.nuw(ks) - mean(r.nuw(ks)));
        M(id, :) = [sum(c(ks).^2) / sum(r.vF(ks, 3).^2), sqrt(mean(c(ks).^2)), sqrt(mean(r.vF(ks, 3).^2)), D, Dp, b];
    end
    P{ia} = M;
    fprintf('%-15s| P0: share of the compensation in vF(3)^2 %s (mean %.3f), rms c %.4f vs rms vF(3) %.4f | D %s | D'' %s | |D''|/|D| median %.2f | slope nu_w on m~_w %s\n', arm{ia}, ...
            mat2str(round(M(:, 1)', 3)), mean(M(:, 1)), mean(M(:, 2)), mean(M(:, 3)), mat2str(round(M(:, 4)', 2)), mat2str(round(M(:, 5)', 2)), median(abs(M(:, 5)) ./ abs(M(:, 4))), mat2str(round(M(:, 6)', 2)));
end
f = figure('Color', 'w', 'Position', [20 20 650 * NA 1150]);  tl = tiledlayout(3, NA, 'TileSpacing', 'compact', 'Padding', 'compact');  nmR = 1e3 * Rr;
for ia = 1:NA
    r = rs{ia, 5};  kk = (1:r.kend)';  k = kk(r.t(kk) >= tS - 1);  t = r.t(k) - tS;  ks0 = k(r.t(k) >= tS);
    au = r.off_post(:, 1);  c = -au .* r.vMh(:, 1) ./ (r.aT .* r.aN - au.^2);  z = filter(1, [1 -lc], r.mk(:, 3));  mt = [0; 0; z(1:end-2)];
    a1 = nexttile(tl, ia);  hold on;  box on;  plot(t, r.vF(k, 3), 'Color', [0.7 0.78 1], 'LineWidth', 0.5, 'DisplayName', 'normal force step vF_w');
    plot(t, c(k), 'Color', cT, 'LineWidth', 1.2, 'DisplayName', 'off-diagonal compensation');  ylim([-0.06 0.06]);  ylabel('a_o f_w  [R]');  title(sprintf('%s, seed 5', arm{ia}));
    if ia == 1; legend('Location', 'northwest'); end
    a2 = nexttile(tl, NA + ia);  hold on;  box on;  plot(t, nmR * movmean(r.nuw(k), 80), 'Color', cE, 'LineWidth', 1.4, 'DisplayName', 'normal innovation (80-step mean)');
    plot(t, nmR * movmean(mt(k), 80), 'Color', cT, 'LineWidth', 1.4, 'DisplayName', 'known term m (filtered, delayed)');  yline(0, 'k-', 'HandleVisibility', 'off');
    ylim([-3 3]);  ylabel('[nm]');  if ia == 1; legend('Location', 'northwest'); end
    a3 = nexttile(tl, 2 * NA + ia);  hold on;  box on;  ap0 = (1 / local_cp(hs + 1e-4) - 1 / local_cp(hs - 1e-4)) / 2e-4;
    plot(r.t(ks0) - tS, cumsum(r.k15w(ks0) .* r.nuw(ks0)) / ap0, 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'D: y1-w push on a''');
    plot(r.t(ks0) - tS, cumsum(r.k15w(ks0) .* (r.nuw(ks0) - mt(ks0))) / ap0, '--', 'Color', cT, 'LineWidth', 1.6, 'DisplayName', 'D'': with m removed');
    yline(0, 'k-', 'HandleVisibility', 'off');  ylim([-3 2]);  ylabel('cumulative / true slope');  xlabel('time since t_{scan}  (sec)');  if ia == 1; legend('Location', 'southwest'); end
    for a = [a1 a2 a3]; xline(a, 0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  xlim(a, [-1 Tw]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
end
fn = fullfile(fo, 'fig_1029_p0_dd.png');  exportgraphics(f, fn, 'Resolution', 115);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
