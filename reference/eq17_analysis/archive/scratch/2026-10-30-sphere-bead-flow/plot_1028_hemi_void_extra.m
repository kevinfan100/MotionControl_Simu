% STATUS: SCRATCH -- 2026-10-28 (user): companion of plot_1028_hemi_void.m (same run: void hemisphere, law-derived a' floor, document
%   version, a_d = 0.05, seed 1). Left: before the scan (0 - t_scan) -- a-hat_33, a-hat_11 (truth red, a_d dashed) and theta-hat (truth red),
%   dashed = end of hold; right: motion error over the whole run (hold, regulation, scan; dashed = end of hold, dash-dot = t_scan) in the TRUE local coordinate, delta u, delta v, delta w = V_k' (p_d - p).
%   Speed from spd_in (default 5 um/s), P_fix from pf_in (default 3 deg). Output fig_1028_hemi_void_extra_v<speed>_pf<P_fix>.png;
%   opens it together with fig_1028_hemi_void_v<speed>_pf<P_fix>.png.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  spd = 5;  if exist('spd_in', 'var'); spd = spd_in; end;  pf = 3;  if exist('pf_in', 'var'); pf = pf_in; end;  seed = 1;
FS = 15;  hatc = char(770);  AB = char(257);  th = char(952);  mu = char(181);  DL = char(948);  tH = 10;  tS = 17;  rb = 15;
cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Ts = 1 / 1612;  Lscan = pi / 2 * (rb + hs * Rr);  Tsc = Lscan / spd;
o = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 5, ...
           'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
           't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
           'scan_dir', [1 1], 'scan_len', Lscan, 'du_d', spd * Ts / Rr, 'T', tS + Tsc + 0.5, 'seeds', seed, 'sP_fix_theta_deg', pf, 'tag', 'vx');
if isempty(gcp('nocreate')); parpool('Processes', min(9, feature('numcores'))); end
rc = cell(1, 1);
parfor i = 1:1; R = run_t3_xyz_fixed(o);  rc{i} = R.runs{1}; end      % parallel worker: same noise as plot_1028_hemi_void
r = rc{1};
f = figure('Color', 'w', 'Position', [30 30 2100 1300]);  tl = tiledlayout(6, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
k = find(r.t < tS);  t = r.t(k);
axg = nexttile(tl, 1, [3 1]);  hold on;  box on;
h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.aT_tr(k), 'Color', cT, 'LineWidth', 2.2);
h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(t, r.aT(k), 'Color', cG, 'LineWidth', 1.4);  h4 = yline(ad, 'k--', 'LineWidth', 1.5);
ylim([0 1.1]);  ylabel('gain');
legend([h1 h2 h3 h4], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc), sprintf('%s_d = %g', AB, ad)}, ...
       'Location', 'northoutside', 'Orientation', 'horizontal');
axa = nexttile(tl, 7, [3 1]);  hold on;  box on;
h1 = plot(t, r.th_tr(k, 2), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 2.2);
h2 = plot(t, r.thy(k), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(t, r.thx(k), 'Color', cG, 'LineWidth', 1.4);
ylabel(sprintf('%s  [\\circ]', th));  xlabel('Time (sec)');
legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
for a = [axg axa]; xline(a, tH, '--', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');  set(a, 'XLim', [0 tS]); end
title(axg, sprintf('before the scan: hold 0-%d s, regulation to a_d %d-%d s', tH, tH, tS));
ks = find(r.t <= tS + Tsc & (1:numel(r.t))' <= r.kend);  ts = r.t(ks);  dw = zeros(numel(ks), 3);
for j = 1:numel(ks)
    a = r.th_tr(ks(j), :) * pi / 180;  Vt = [cos(a(2)) 0 sin(a(2)); 0 1 0; -sin(a(2)) 0 cos(a(2))] * [1 0 0; 0 cos(a(1)) -sin(a(1)); 0 sin(a(1)) cos(a(1))];
    dw(j, :) = (Vt' * (r.pd(ks(j), :) - r.p(ks(j), :))')';
end
nm = {'u', 'v', 'w'};  axd = gobjects(1, 3);
for row = 1:3
    axd(row) = nexttile(tl, 2 + 4 * (row - 1), [2 1]);  hold on;  box on;  plot(ts, dw(:, row), 'Color', cE, 'LineWidth', 0.6);  ylim([-0.2 0.2]);
    ylabel(sprintf('%s%s  [%sm]', DL, nm{row}, mu));  set(axd(row), 'XLim', [0 ts(end)]);
    xline(axd(row), tH, '--', 'Color', cGr, 'LineWidth', 1.3);  xline(axd(row), tS, '-.', 'Color', cGr, 'LineWidth', 1.3);
end
xlabel(axd(3), 'Time (sec)');  title(axd(1), sprintf('motion error (true local), %d um/s, P_{fix} %g deg', spd, pf));
for a = [axg axa axd]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
fprintf('before the scan (t = %d s): a-hat_33 %.3f (true %.3f), theta-hat (y, x) %.1f %.1f deg | rms delta u v w hold %s, regulation %s, scan %s um\n', tS, ...
        r.aN(k(end)), r.aN_tr(k(end)), r.thy(k(end)), r.thx(k(end)), mat2str(round(sqrt(mean(dw(ts < tH, :).^2)), 3)), ...
        mat2str(round(sqrt(mean(dw(ts >= tH & ts < tS, :).^2)), 3)), mat2str(round(sqrt(mean(dw(ts >= tS, :).^2)), 3)));
fn = fullfile(fo, sprintf('fig_1028_hemi_void_extra_v%d_pf%g.png', spd, pf));  exportgraphics(f, fn, 'Resolution', 130);  close(f);
system(['open ' fn ' ' fullfile(fo, sprintf('fig_1028_hemi_void_v%d_pf%g.png', spd, pf))]);

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
