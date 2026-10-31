% STATUS: SCRATCH -- 2026-10-30 (user) [variant run_1030_descent_wait: current flow with descent 1 | 5 s x regulation wait 1 | 0 s, seeds 1-10, figure fig_1030_descent_wait.png] how the flow went from the 5-s descent (fig_1029_desc_scan: a_33 tracked a_d poorly) to the current one
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
              'scan_plane_n', [0 1 0], 'kappa_state', true, 'scan_len', Lturn, 'du_d', spd * Ts / Rr, 'sP_fix_theta_deg', 2, 'tag', 'dw');
nm = {'descent 1 s, wait 1 s (current)', 'descent 5 s, wait 1 s', 'descent 1 s, no wait', 'descent 5 s, no wait'};  TDa = [1 5 1 5];  PWa = [1 1 0 0];
NL = numel(nm);  NS = 10;  [IL, ID] = ndgrid(1:NL, 1:NS);  rs = cell(size(IL));
if isempty(gcp('nocreate')); parpool('Processes', min(12, feature('numcores'))); end
parfor q = 1:numel(IL)
    L = IL(q);  o = base;  o.seeds = ID(q);
    Td = TDa(L);  o.descent_T = Td;  o.descent_v = (15 - hs * Rr) / Td;  o.tau_a = 0.5;  o.post_wait = [0 PWa(L)];
    o.kappa_from_scan = true;  o.scan_ramp = 0.5;  tS = 10 + Td + PWa(L);  o.T = tS + Lturn / spd + 0.5;
    R = run_t3_xyz_fixed(o);  r = R.runs{1};
    rs{q} = struct('t', r.t, 'kend', r.kend, 'contact', r.contact, 'aN', r.aN, 'aN_tr', r.aN_tr, 'h', r.h, 'tsc', r.tsc);
end
M = NaN(NL, NS, 4);
for L = 1:NL
    for id = 1:NS
        r = rs{L, id};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= r.tsc & r.t(kk) <= r.tsc + Lturn / spd + 0.25);
        M(L, id, :) = [mean(abs(r.aN_tr(ks) - ad)), mean(r.aN_tr(ks) - ad), min(r.aN_tr(ks)), max(r.h(ks)) * Rr];
    end
    fprintf('%-52s| contact %d | mean|a33 - a_d| %s (mean %.3f) | mean(a33 - a_d) %+.3f | min a33 %.3f | gap max %.2f um\n', nm{L}, sum(cellfun(@(r) r.contact, rs(L, :))), ...
            mat2str(round(squeeze(M(L, :, 1)), 3)), mean(M(L, :, 1)), mean(M(L, :, 2)), min(M(L, :, 3)), max(M(L, :, 4)));
end
f = figure('Color', 'w', 'Position', [20 20 2000 1500]);  tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for j = 1:4
    L = j;  r = rs{L, 5};  kk = (1:r.kend)';  ks = kk(r.t(kk) >= r.tsc - 1 & r.t(kk) <= r.tsc + Lturn / spd + 0.25);  t = r.t(ks) - r.tsc;
    ax = nexttile(tl, j);  hold on;  box on;  ks = kk(r.t(kk) >= r.tsc - 3 & r.t(kk) <= r.tsc + Lturn / spd + 0.25);  t = r.t(ks) - r.tsc;  h1 = plot(t, r.aN_tr(ks), 'Color', cT, 'LineWidth', 1.8);  h2 = plot(t, r.aN(ks), 'Color', cE, 'LineWidth', 1);  h3 = yline(ad, 'k--', 'LineWidth', 1.5);
    xline(0, '-.', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.4, 'HandleVisibility', 'off');  ylim([0 0.3]);  xlim([-3 t(end)]);  ylabel(sprintf('%s_{33}', AB));  xlabel('time since t_{scan}  (sec)');
    title(sprintf('%s, seed 5', nm{L}));  if j == 1; legend([h1 h2 h3], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s_d', AB)}, 'Location', 'northeast'); end
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax.Toolbar, 'Visible', 'off');
end
ax = nexttile(tl, 5, [1 2]);  hold on;  box on;
for L = 1:NL; plot(L + 0.04 * (-4.5:4.5), squeeze(M(L, :, 1)), 'o', 'Color', cE, 'MarkerFaceColor', cE);  plot(L + [-0.25 0.25], mean(M(L, :, 1)) * [1 1], 'k-', 'LineWidth', 3); end
set(ax, 'XTick', 1:NL, 'XTickLabel', nm);
ylabel(sprintf('mean |%s_{33} - %s_d| over the turn', AB, AB));  ylim([0 0.08]);  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax.Toolbar, 'Visible', 'off');
fn = fullfile(fo, 'fig_1030_descent_wait.png');  exportgraphics(f, fn, 'Resolution', 120);  close(f);  system(['open ' fn]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
