% STATUS: SCRATCH -- 2026-10-30 (user) [10-30 later: tau_i 1 s (zeta 0.71), always vs after-first-crossing integration, windup check; figure fig_1030_pi_nowait_ti1.png] [variant run_1030_pi_nowait: descent 5 s, no wait, P vs PI (tau_a 0.5, tau_i 2 = critical) with two conditional-integration gates: |e| <= 2 sd of a-hat_33 | integrate once e has first crossed 0, seeds 1-10, figure fig_1030_pi_nowait.png] how the flow went from the 5-s descent (fig_1029_desc_scan: a_33 tracked a_d poorly) to the current one
%   (fig_1029_cur_scan_reg1_s5_noy1). Void sphere radius 15 um, 40 um/s full turn, a_d 0.05, angle P_fix 2 deg from the scan start, threefry.
%   Ladder, one change per step: L0 old (descent 5 s, scan + P regulation together at the stop, tau_a 2, slope P_fix law L, kappa from the
%   start, no ramp) | L1 + report slope P_fix | L2 + regulation at the stop, scan 1 s later | L3 + tau_a 0.5 | L4 + kappa from t_scan and speed
%   ramp 0.5 s | L5 descent 1 s (= current) | L6 + slopes not updated by y1 in the scan (= the noy1 figure). Seeds 1-5.
%   Console: mean |a33 - a_d| over the turn, mean (a33 - a_d), min a33, gap range. Figure fig_1030_flow_ladder.png: seed 5 a_33 for L0 and L6,
%   and the per-step mean |a33 - a_d| (seed dots).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  Ts = 1 / 1612;  spd = 40;  FS = 14;  hatc = char(770);  AB = char(257);  cT = [0.85 0 0];  cE = [0 0.25 0.85];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', 10, 'a_d', ad, ...
              'ap_q', true, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, 'scan_dir', [1 0], 'hemi_start_deg', 10, ...
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, 'sP_fix_theta_deg', 2, 'tag', 'pn');
nm = {'P', 'PI tau_i 1 s, always integrate', 'PI tau_i 1 s, after e first crosses 0'};  TDa = [5 5 5];  PWa = [0 0 0];  if exist('nm_in', 'var'); nm = nm_in; end
NL = numel(nm);  NS = 10;  [IL, ID] = ndgrid(1:NL, 1:NS);  rs = cell(size(IL));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IL)
    L = IL(q);  o = base;  o.seeds = ID(q);
    Td = TDa(L);  o.descent_T = Td;  o.descent_v = (15 - hs * Rr) / Td;  o.tau_a = 0.5;  o.post_wait = [0 PWa(L)];
    o.kappa_from_scan = true;  o.scan_ramp = 0.5;  tS = 10 + Td + PWa(L);  o.T = tS + Lturn / spd + 0.5;
    if L >= 2; o.reg_pi = true;  o.tau_i = 2 * o.tau_a; end
    if L == 3; o.reg_pi_cond = 'cross'; end
    R = run_t3_xyz_fixed(o);  r = R.runs{1};  g = [];  eIl = [];  if isfield(r, 'gate_pi'); g = r.gate_pi(:);  eIl = r.eI(:); end
    rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'aN', r.aN, 'aN_tr', r.aN_tr, 'h', r.h, 'tsc', r.tsc, 'apN', r.apN, 'gate', g, 'eI', eIl);
end
M = NaN(NL, NS, 7);
for L = 1:NL
    for id = 1:NS
        r = rs{L, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= r.tsc & r.t(kk) <= r.tsc + Lturn / spd + 0.25);
        gf = NaN;  if ~isempty(r.gate); gg = r.gate(ks(ks <= numel(r.gate)));  gf = mean(gg == 1); end
        M(L, id, :) = [mean(abs(r.aN_tr(ks) - ad)), mean(r.aN_tr(ks) - ad), min(r.aN_tr(ks)), max(r.h(ks)) * Rr, mean(r.apN(ks) < 0), mean(r.aN(ks) - ad), gf];
    end
    W = NaN(NS, 3);
    for id = 1:NS
        r = rs{L, id};  kk = (1:r.kend)';  k1 = kk(r.t(kk) >= r.tsc & r.t(kk) < r.tsc + 1);
        W(id, 1:2) = [min(r.aN_tr(k1)), min(r.aN(k1))];  if ~isempty(r.eI); W(id, 3) = max(abs(r.eI(k1(k1 <= numel(r.eI))))) * Ts / 1; end
    end
    fprintf('   first 1 s after landing: min a33 true %.3f (worst %.3f), min a-hat %.3f | peak |integral| as an equivalent error (Ts/tau_i) sum e: %.4f\n', mean(W(:, 1)), min(W(:, 1)), mean(W(:, 2)), mean(W(:, 3)));
    fprintf('%-52s| contact %d | mean|a33 - a_d| %s (mean %.3f) | mean(a33 - a_d) %+.3f | min a33 %.3f | gap max %.2f um | a''-hat < 0 %.2f | mean(a-hat - a_d) %+.4f | integrating %.2f of the scan\n', nm{L}, sum(cellfun(@(r) r.contact, rs(L, :))), ...
            mat2str(round(squeeze(M(L, :, 1)), 3)), mean(M(L, :, 1)), mean(M(L, :, 2)), min(M(L, :, 3)), max(M(L, :, 4)), mean(M(L, :, 5)), mean(M(L, :, 6)), mean(M(L, :, 7)));
end
f = figure('Color', 'w', 'Position', [20 20 2700 1100]);  tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for j = 1:3
    L = j;  r = rs{L, 5};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= r.tsc - 1 & r.t(kk) <= r.tsc + Lturn / spd + 0.25);  t = r.t(ks) - r.tsc;
    ax = nexttile(tl, j);  hold on;  box on;  ks = kk(r.t(kk) >= r.tsc - 3 & r.t(kk) <= r.tsc + Lturn / spd + 0.25);  t = r.t(ks) - r.tsc;  h1 = plot(t, r.aN_tr(ks), 'Color', cT, 'LineWidth', 1.8);  h2 = plot(t, r.aN(ks), 'Color', cE, 'LineWidth', 1);  h3 = yline(ad, 'k--', 'LineWidth', 1.5);
    xline(0, '-.', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.4, 'HandleVisibility', 'off');  ylim([0 0.3]);  xlim([-3 t(end)]);  ylabel(sprintf('%s_{33}', AB));  xlabel('time since t_{scan}  (sec)');
    title(sprintf('%s, seed 5', nm{L}));  if j == 1; legend([h1 h2 h3], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s_d', AB)}, 'Location', 'northeast'); end
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax.Toolbar, 'Visible', 'off');
end
ax = nexttile(tl, 4, [1 3]);  hold on;  box on;
for L = 1:NL; plot(L + 0.04 * (-4.5:4.5), squeeze(M(L, :, 1)), 'o', 'Color', cE, 'MarkerFaceColor', cE);  plot(L + [-0.25 0.25], mean(M(L, :, 1)) * [1 1], 'k-', 'LineWidth', 3); end
set(ax, 'XTick', 1:NL, 'XTickLabel', nm);
ylabel(sprintf('mean |%s_{33} - %s_d| over the turn', AB, AB));  ylim([0 0.06]);  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax.Toolbar, 'Visible', 'off');
fn = fullfile(fo, 'fig_1030_pi_nowait_ti1.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
