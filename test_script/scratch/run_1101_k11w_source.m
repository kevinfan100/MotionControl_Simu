% STATUS: SCRATCH -- 2026-11-01 (user): does K(11,w) flip sign at 40 um/s with the off-diagonal P alive, and from what. Log L.B119: sources of P(a_33, delta w_3)[k+1] by group of row 11 of F_e
%   (position, a_11, a_33, off-diagonal, slope, rest) + Q(11,9), K(11,w), P(a_33, delta w_1), P(a_33, delta w_3). Also S(0) = sum -F_w K11 nu split by the 7 channels, corr(F_w, nu_w).
%   Pre-registered: hypothesis holds if mean K(11,w) is positive at P alive 40, negative at C and P alive 10, and the off-diagonal group grows with speed and exceeds |Q term|. Sanity: sources sum = P(a_33, dw3)[k+1] (no rotation, frozen arms); off-diagonal ~ 0 in C; Q term negative.
%   5 seeds, window t_scan .. +3 s. Arms: plane V-hat frozen: P alive 40 | P alive 10 | C 40; sphere true normal, V-hat moving. Figure fig_1101_k11w_source.png (seed 5, 80-step mean K(11,w)).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  tS = 12;  Tw = 3;  FS = 14;  cE = [0 0.25 0.85];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);
base = struct('theta_x', 0, 'theta_y', 10, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 10], 'gain0', 'seed', 'sP_theta_deg', 10, 'form', 'local', 'gap_gate', 2, ...
              'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'tau_a', 0.5, 'a_d', ad, 'ap_q', true, 'ap_floor_law', true, 'scan_dir', [1 0], ...
              'kappa_state', false, 'scan_ramp', 0.5, 'scan_len', 1e6, 'du_d', 40 * Ts / Rr, 'sP_fix_theta_deg', 2, 'descent_T', 1, 'descent_v', 15 - hs * Rr, ...
              'post_wait', [0 1], 'T', tS + Tw, 'tag', 'k1', 'freeze_angle', true);
arm = {'P alive, 40', 'P alive, 10', 'C, 40', 'sphere, true normal, 40'};  NA = numel(arm);  NS = 5;  [IA, ID] = ndgrid(1:NA, 1:NS);  rs = cell(size(IA));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IA)
    o = base;  o.seeds = ID(q);
    if IA(q) == 2; o.du_d = 10 * Ts / Rr; end
    if IA(q) == 3; o.sP_theta_deg = 1e-3;  o.q_theta_off = true; end
    if IA(q) == 4
        o = rmfield(o, 'freeze_angle');  o.theta_y = 0;  o.th0_deg = [0 0];  o.wall = 'hemi';  o.hemi_r = rb;  o.hemi_void = true;  o.hemi_start_deg = 10;  o.scan_plane_n = [0 1 0];
        o.kappa_state = true;  o.kappa_from_scan = true;  o.cheat_w = true;
    end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'B', r.B119, 'Fw', r.vF(:, 3), 'da', r.da33, 'nu', r.nuw(:));
end
nm = {'position', 'a_11', 'a_33', 'off-diagonal', 'slope', 'Q(11,9)'};  ch = {'y1 u', 'y1 v', 'y1 w', 'ym 11', 'ym 33', 'ym uw', 'ym vw'};
f = figure('Color', 'w', 'Position', [20 20 620 * NA 460]);  tl = tiledlayout(1, NA, 'TileSpacing', 'compact', 'Padding', 'compact');
for ia = 1:NA
    M = NaN(NS, 6);  Kw = NaN(NS, 2);  S = NaN(NS, 7);  cl = NaN(NS, 1);  cn = NaN(NS, 1);  Pv = NaN(NS, 2);
    for id = 1:NS
        r = rs{ia, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= tS & r.t(kk) <= tS + Tw);  ks = ks(1:end - 1);  B = r.B;
        M(id, :) = mean(B(ks, 1:6), 1);  Kw(id, :) = [mean(B(ks, 7)), mean(B(ks, 7) > 0)];  S(id, :) = sum(-r.Fw(ks) .* r.da(ks, :), 1);
        cl(id) = max(abs(sum(B(ks, [1:6 10]), 2) - B(ks + 1, 9)));  cn(id) = sum(r.Fw(ks) .* r.nu(ks)) / sqrt(sum(r.Fw(ks).^2) * sum(r.nu(ks).^2));  Pv(id, :) = [mean(B(ks, 8)), mean(B(ks, 9))];
    end
    fprintf('%s | closure max %.1e | corr(F_w, nu_w) %s\n   mean K(11,w) %s | fraction > 0 %s | mean P(a33, dw1) %s | P(a33, dw3) %s\n', arm{ia}, max(cl), mat2str(round(cn', 2)), mat2str(round(Kw(:, 1)', 4)), mat2str(round(Kw(:, 2)', 2)), mat2str(Pv(:, 1)' * 1e6, 3), mat2str(Pv(:, 2)' * 1e6, 3));
    fprintf('   sources of P(a33, dw3) x 1e6 (mean per step): ');  for c = 1:6; fprintf('%s %+.2f | ', nm{c}, 1e6 * mean(M(:, c))); end;  fprintf('\n   S(0) by channel: ');  for c = 1:7; fprintf('%s %+.3f | ', ch{c}, mean(S(:, c))); end;  fprintf('\n');
    r = rs{ia, 5};  kk = (1:r.kend)';  k = kk(r.t(kk) >= tS - 1 & r.t(kk) <= tS + Tw);  a = nexttile(tl, ia);  hold on;  box on;  plot(r.t(k) - tS, movmean(r.B(k, 7), 80), 'Color', cE, 'LineWidth', 1.3, 'DisplayName', arm{ia});
    yline(0, 'k:', 'HandleVisibility', 'off');  xline(0, '-.', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  ylabel('K(a_{33}, w), 80-step mean');  xlabel('time since t_{scan}  (sec)');  legend('Location', 'northoutside');  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
end
fn = fullfile(fo, 'fig_1101_k11w_source.png');  exportgraphics(f, fn, 'Resolution', 130);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
