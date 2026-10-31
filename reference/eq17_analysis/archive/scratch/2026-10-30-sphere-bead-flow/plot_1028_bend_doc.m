% STATUS: SCRATCH -- 2026-10-28 (user): bend wall (as the 10-22 run flow5bend) with the document estimator (run_t3_xyz_fixed form 'local',
%   doc_timing + theta_out, one-sided gate 2 sqrt P); hold 0-10 s (no dither), regulation to a_d = 0.1 from 10 s (tau_a 2 s), scan along
%   u-hat 17-20 s at du_d = 0.011 R per step; Q: slopes P_fix^2 I, angle P_fix 2 deg from t_scan; theta-hat[0] = 0, prior 5 deg; seeds 1-3,
%   the seed with the smallest angle error during the scan shown. Figures in the layout of plot_t2_bend_full (10-22):
%   (1) fig_1028_bend_est_s<seed>: a_33 (+ a_d), a_11, angles (true red); dashed = end of hold, dash-dot = t_scan;
%   (2) fig_1028_bend_motion_s<seed>: TRUE local coordinate of each step -- u, v = distance travelled along the true u, v (sum of V_k' dp),
%       w = h R (log axis, h* R dotted), desired = actual + delta; right: delta = V_k' (p_d - p).
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  seeds = 1:3;  n = numel(seeds);  FS = 18;  hatc = char(770);  AB = char(257);  th = char(952);  mu = char(181);  DL = char(948);
tH = 10;  tS = 17;  T = 20;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'T', T, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 5, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', 0.1, ...
              't_scan', tS, 'du_d', 0.011, 'ap_q', true, 'q_theta_from', tS, 'sP_fix_theta_deg', 2, 'wall', 'bend', ...
              'bend_alpha', 20, 'bend_phi', 45, 'bend_Lx', 20, 'bend_xb', 10);
if isempty(gcp('nocreate')); parpool('Processes', min(6, feature('numcores'))); end
rs = cell(1, n);
parfor i = 1:n; o = base;  o.seeds = seeds(i);  o.tag = 'bd';  R = run_t3_xyz_fixed(o);  rs{i} = R.runs{1}; end
sc = cellfun(@(r) mean(abs(r.thy(r.t >= tS) - r.th_tr(r.t >= tS, 2)) + abs(r.thx(r.t >= tS) - r.th_tr(r.t >= tS, 1))), rs);
[~, ib] = min(sc);  r = rs{ib};  k = 1:r.kend;  t = r.t(k);  sb = seeds(ib);
fprintf('mean |angle error| during the scan per seed %s deg -> seed %d\n', mat2str(round(sc, 2)), sb);
hs = fzero(@(h) 1 / local_cp(h) - base.a_d, [1.001 30]);
% ---------------- (1) estimates ----------------
f = figure('Position', [40 40 1300 1800], 'Color', 'w');  tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = nexttile;  hold on;  box on;
h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.5);  h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.6);  h3 = yline(base.a_d, 'k--', 'LineWidth', 2);
ylim([0 1.1]);  ylabel(sprintf('%s_{33}', AB));  legend([h1 h2 h3], {sprintf('%s_{33}', AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s_d = %g', AB, base.a_d)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
ax2 = nexttile;  hold on;  box on;
h1 = plot(t, r.aT_tr(k), 'Color', cT, 'LineWidth', 2.5);  h2 = plot(t, r.aT(k), 'Color', cG, 'LineWidth', 1.6);
ylim([0 1.1]);  ylabel(sprintf('%s_{11}', AB));  legend([h1 h2], {sprintf('%s_{11}', AB), sprintf('%s%s_{11}', AB, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
ax3 = nexttile;  hold on;  box on;
h1 = plot(t, r.th_tr(k, 2), 'Color', cT, 'LineWidth', 2.5);  plot(t, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 2.5);
h2 = plot(t, r.thy(k), 'Color', cE, 'LineWidth', 1.6);  h3 = plot(t, r.thx(k), 'Color', cG, 'LineWidth', 1.6);
ylim([-30 30]);  ylabel(sprintf('%s  [\\circ]', th));  xlabel('Time (sec)');
legend([h1 h2 h3], {sprintf('%s_y,  %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
for ax = [ax1 ax2 ax3]
    xline(ax, tH, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off');  xline(ax, tS, '-.', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off');
    set(ax, 'XLim', [0 T], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.5);  set(ax.Toolbar, 'Visible', 'off');
end
exportgraphics(f, fullfile(fo, sprintf('fig_1028_bend_est_s%d.png', sb)), 'Resolution', 150);  close(f);
% ---------------- (2) motion in the true local coordinate ----------------
N = numel(k);  uv = zeros(N, 2);  del = zeros(N, 3);
for j = 1:N
    Vt = local_Ry(r.th_tr(j, 2) * pi / 180) * local_Rx(r.th_tr(j, 1) * pi / 180);
    del(j, :) = (Vt' * (r.pd(j, :) - r.p(j, :))')';
    if j > 1; dpl = Vt' * (r.p(j, :) - r.p(j - 1, :))';  uv(j, :) = uv(j - 1, :) + dpl(1:2)'; end
end
wv = r.h(k) * Rr;  act = [uv wv];  des = act + del;  nmv = {'u', 'v', 'w'};
f = figure('Position', [40 40 2000 1550], 'Color', 'w');  tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for row = 1:3
    ax = nexttile;  hold on;  box on;
    h1 = plot(t, des(:, row), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(t, act(:, row), 'Color', cE, 'LineWidth', 1.4);
    if row == 3; set(ax, 'YScale', 'log');  yline(hs * Rr, 'k:', 'LineWidth', 1.5);  ylim([2 16]); end
    ylabel(sprintf('%s  [%sm]', nmv{row}, mu));  if row == 1; legend([h1 h2], {'u_d', 'u'}, 'Location', 'northoutside', 'Orientation', 'horizontal'); end
    if row == 3; xlabel('Time (sec)'); end
    xline(ax, tH, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off');  xline(ax, tS, '-.', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off');
    set(ax, 'XLim', [0 T], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.5);  set(ax.Toolbar, 'Visible', 'off');
    ax = nexttile;  hold on;  box on;  plot(t, del(:, row), 'Color', cE, 'LineWidth', 0.6);
    ylabel(sprintf('%s%s  [%sm]', DL, nmv{row}, mu));  ylim([-0.3 0.3]);  if row == 3; xlabel('Time (sec)'); end
    xline(ax, tH, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off');  xline(ax, tS, '-.', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off');
    set(ax, 'XLim', [0 T], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.5);  set(ax.Toolbar, 'Visible', 'off');
end
exportgraphics(f, fullfile(fo, sprintf('fig_1028_bend_motion_s%d.png', sb)), 'Resolution', 150);  close(f);
fprintf('rms delta u v w over the scan [um]: %s | before the scan (10-17 s): %s\n', mat2str(round(sqrt(mean(del(t >= tS, :).^2)), 3)), mat2str(round(sqrt(mean(del(t >= tH & t < tS, :).^2)), 3)));
system(['open ' fullfile(fo, sprintf('fig_1028_bend_est_s%d.png', sb)) ' ' fullfile(fo, sprintf('fig_1028_bend_motion_s%d.png', sb))]);
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
function M = local_Rx(a)
    M = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end
function M = local_Ry(a)
    M = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end
