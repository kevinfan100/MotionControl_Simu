% STATUS: SCRATCH -- 2026-10-28 (user): [variant plot_1028_hemi_lf: columns = angle Q P_fix 2 deg | Meng forgetting factor 0.98 on the
%   off-diagonal slots from t_scan, both 40 um/s] hemisphere (radius 15 um), document estimator, diagonal scan (u-hat + v-hat)/sqrt 2 over the top,
%   drawn split: (1) before the scan (hold 0-10 s, regulation to a_d = 0.1 10-17 s), (2) the scan, columns = scan speed 40 | 10 | 5 um/s
%   (40 = the 10-22 bend run, du_d = 0.011 R per step). Scan stops at the edge arc length; drawn until the probe leaves the hemisphere.
%   Settings as plot_1028_walls_merged.m. Seeds 1-3; the seed with the smallest angle error during the scan shown per column.
%   Rows of (2): side view along the scan direction, gains, angles, delta u, delta v, delta w (true local coordinate).
%   Outputs test_results/temp_3d_wsonly/fig_1028_hemi_before.png, fig_1028_hemi_scan_speeds.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  seeds = 1:3;  n = numel(seeds);  spd = [40 40];  ns = numel(spd);  armlab = {'angle Q: P_fix 2 deg', 'forgetting factor 0.98 (Meng)'};  FS = 14;  hatc = char(770);  AB = char(257);  th = char(952);  mu = char(181);  DL = char(948);
tH = 10;  tS = 17;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];  rb = 15;
hs = fzero(@(h) 1 / local_cp(h) - 0.1, [1.001 30]);  Ts = 1 / 1612;  Lscan = pi / 2 * (rb + hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 5, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', 0.1, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'sP_fix_theta_deg', 2, 'wall', 'hemi', 'hemi_r', rb, 'scan_dir', [1 1], 'scan_len', Lscan);
if isempty(gcp('nocreate')); parpool('Processes', min(9, feature('numcores'))); end
jobs = cell(1, ns * n);
parfor q = 1:ns * n
    is = 1 + floor((q - 1) / n);  i = q - (is - 1) * n;  o = base;  o.du_d = spd(is) * Ts / Rr;  o.T = tS + Lscan / spd(is) + 0.5;
    if is == 2; o = rmfield(o, 'q_theta_from');  o.lf_off = 0.98;  o.lf_from = tS; end
    o.seeds = seeds(i);  o.tag = 'hs';  R = run_t3_xyz_fixed(o);  jobs{q} = R.runs{1};
end
pick = zeros(1, ns);
for is = 1:ns
    rs = jobs((is - 1) * n + (1:n));  sc = NaN(1, n);
    for i = 1:n
        r = rs{i};  kk = local_scan_steps(r, tS);  e = [r.thy(kk) - r.th_tr(kk, 2), r.thx(kk) - r.th_tr(kk, 1)];  dw = local_delta(r, kk);
        sc(i) = mean(sum(abs(e), 2));
        fprintf('%-30s seed %d: contact %d | scan %.2f s | mean |angle err| %.1f deg, max y %.1f x %.1f | rms delta u v w %s um | max gap %.1f um | true angle at exit (y, x) %.0f %.0f\n', ...
                armlab{is}, seeds(i), r.contact, r.t(kk(end)) - tS, sc(i), max(abs(e(:, 1))), max(abs(e(:, 2))), mat2str(round(sqrt(mean(dw.^2)), 3)), ...
                max(r.h(kk)) * Rr, r.th_tr(kk(end), 2), r.th_tr(kk(end), 1));
    end
    [~, pick(is)] = min(sc);
end
% ---------------- (1) before the scan (same for all speeds; slowest column's seed) ----------------
r = jobs{(ns - 1) * n + pick(ns)};  k = find(r.t < tS);  t = r.t(k);  dw = local_delta(r, k);
f = figure('Color', 'w', 'Position', [30 30 2200 1100]);  tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl, 1);  hold on;  box on;  local_side(r, k, tS, tH, rb, cE, cGr);  xlim([-25 30]);  ylim([-3 33]);  set(ax, 'FontSize', FS, 'FontWeight', 'bold');  set(ax.Toolbar, 'Visible', 'off');
axg = nexttile(tl, 3);  local_gains(r, k, cT, cE, cG, AB, hatc);  axa = nexttile(tl, 5);  local_angles(r, k, cT, cE, cG, th, hatc);  xlabel(axa, 'Time (sec)');
axd = gobjects(1, 3);  nm = {'u', 'v', 'w'};
for row = 1:3
    axd(row) = nexttile(tl, 2 * row);  hold on;  box on;  plot(t, dw(:, row), 'Color', cE, 'LineWidth', 0.6);  ylim([-0.3 0.3]);
    ylabel(sprintf('%s%s  [%sm]', DL, nm{row}, mu));  if row == 3; xlabel('Time (sec)'); end
end
for a = [axg axa axd]
    xline(a, tH, '--', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');  set(a, 'XLim', [0 tS], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
end
exportgraphics(f, fullfile(fo, 'fig_1028_hemi_lf_before.png'), 'Resolution', 150);  close(f);
% ---------------- (2) the scan, one column per speed ----------------
f = figure('Color', 'w', 'Position', [20 20 1700 1800]);  tl = tiledlayout(5, ns, 'TileSpacing', 'compact', 'Padding', 'compact');
cU = [0 0.25 0.85];  cV = [0 0.55 0.2];  cW = [0.85 0.4 0];
for is = 1:ns
    r = jobs{(is - 1) * n + pick(is)};  k = local_scan_steps(r, tS);  t = r.t(k) - tS;  dw = local_delta(r, k);
    ax = nexttile(tl, is, [2 1]);  hold on;  box on;  local_side(r, (1:k(end))', tS, tH, rb, cE, cGr);  xlim([-3 22]);  ylim([0 22]);
    title(sprintf('%s, seed %d', armlab{is}, seeds(pick(is))));  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(ax.Toolbar, 'Visible', 'off');
    if is > 1; legend(ax, 'off'); end
    axg = nexttile(tl, 2 * ns + is);  local_gains(r, k, cT, cE, cG, AB, hatc, t);  if is > 1; legend(axg, 'off'); end
    axa = nexttile(tl, 3 * ns + is);  local_angles(r, k, cT, cE, cG, th, hatc, t);  ylim(axa, [-60 80]);  if is > 1; legend(axa, 'off'); end
    axd = nexttile(tl, 4 * ns + is);  hold on;  box on;
    h1 = plot(t, dw(:, 1), 'Color', cU, 'LineWidth', 0.7);  h2 = plot(t, dw(:, 2), 'Color', cV, 'LineWidth', 0.7);  h3 = plot(t, dw(:, 3), 'Color', cW, 'LineWidth', 0.9);
    ylim([-0.2 0.2]);  ylabel(sprintf('[%sm]', mu));  xlabel('time since t_{scan}  (sec)');
    if is == 1; legend([h1 h2 h3], {sprintf('%su', DL), sprintf('%sv', DL), sprintf('%sw', DL)}, 'Location', 'northoutside', 'Orientation', 'horizontal'); end
    set([axg axa axd], 'XLim', [0 t(end)], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);
    set(axg.Toolbar, 'Visible', 'off');  set(axa.Toolbar, 'Visible', 'off');  set(axd.Toolbar, 'Visible', 'off');
end
exportgraphics(f, fullfile(fo, 'fig_1028_hemi_lf_scan.png'), 'Resolution', 130);  close(f);
system(['open ' fullfile(fo, 'fig_1028_hemi_lf_scan.png')]);

function kk = local_scan_steps(r, tS)                            % scan steps until the nearest surface becomes the plane
    kk = find(r.t >= tS & (1:numel(r.t))' <= r.kend);
    e = find(abs(r.th_tr(kk, 1)) < 1e-9 & abs(r.th_tr(kk, 2)) < 1e-9 & (1:numel(kk))' > 10, 1);  if ~isempty(e); kk = kk(1:e - 1); end
end
function dw = local_delta(r, k)
    dw = zeros(numel(k), 3);
    for j = 1:numel(k)
        a = r.th_tr(k(j), :) * pi / 180;  Vt = [cos(a(2)) 0 sin(a(2)); 0 1 0; -sin(a(2)) 0 cos(a(2))] * [1 0 0; 0 cos(a(1)) -sin(a(1)); 0 sin(a(1)) cos(a(1))];
        dw(j, :) = (Vt' * (r.pd(k(j), :) - r.p(k(j), :))')';
    end
end
function local_side(r, k, tS, tH, rb, cE, cGr)
    sdir = [1; 1] / sqrt(2);  sp = r.p(k, 1:2) * sdir;  sg = linspace(-25, 30, 600);  zg = sqrt(max(rb^2 - sg.^2, 0));
    fill([sg fliplr(sg)], [zg, -5 + 0 * sg], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(sg, zg, 'Color', cGr, 'LineWidth', 3, 'DisplayName', 'wall');  plot(sp, r.p(k, 3), 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'p');
    kS = find(r.t(k) >= tS, 1);  if ~isempty(kS); plot(sp(kS), r.p(k(kS), 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 't_{scan}'); end
    axis equal;  xlabel('distance along the scan  [\mum]');  ylabel('z  [\mum]');  legend('Location', 'northeast');
end
function local_gains(r, k, cT, cE, cG, AB, hatc, t)
    if nargin < 8; t = r.t(k); end
    hold on;  box on;  h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.aT_tr(k), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(t, r.aT(k), 'Color', cG, 'LineWidth', 1.4);  yline(0.1, 'k--', 'LineWidth', 1.5);
    ylim([0 1.1]);  ylabel('gain');
    legend([h1 h2 h3], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
end
function local_angles(r, k, cT, cE, cG, th, hatc, t)
    if nargin < 8; t = r.t(k); end
    hold on;  box on;  h1 = plot(t, r.th_tr(k, 2), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.thy(k), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(t, r.thx(k), 'Color', cG, 'LineWidth', 1.4);  ylabel(sprintf('%s  [\\circ]', th));
    legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
