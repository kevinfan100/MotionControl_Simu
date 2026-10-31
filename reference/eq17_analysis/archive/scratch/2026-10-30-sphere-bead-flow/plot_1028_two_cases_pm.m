% STATUS: SCRATCH -- 2026-10-28 (user): the two settled cases drawn exactly in the afternoon layout (plot_1028_hemi_mer.m / _extra.m).
%   Whole sphere (radius 15 um, void), start 10 deg off the top 15 um up, theta-hat[0] = 0, angle prior 10 deg; hold 0-10 s; regulation
%   tau_a 2 s to a_d = 0.05; great-circle scan in the x-z plane from 17 s; slope P_fix law (L); document version; seed 1.
%   Case slow: 5 um/s, one full turn, angle P_fix 2 deg.  Case fast: 40 um/s, a quarter turn, angle P_fix 4 deg.
%   (1) fig_1028_pm_<case>_scan: single column -- side view (dashed = start along the true w), gains, angles vs time since t_scan.
%   (2) fig_1028_pm_<case>_extra: left = before the scan (gains, angles; dashed = end of the hold); right = tracking error delta u, v, w in
%       the true local coordinate over the whole run (dashed = end of the hold, dash-dot = t_scan). No numbers on the figures.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  tS = 17;  ph0 = 10;  Ts = 1 / 1612;  seed = 1;
FS = 15;  hatc = char(770);  AB = char(257);  th = char(952);  mu = char(181);  DL = char(948);
cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
              'scan_dir', [1 0], 'hemi_start_deg', ph0, 'scan_plane_n', [0 1 0], 'ap_pfix_law', true, 'seeds', seed, 'tag', 'pm');
cs = struct('name', {'slow', 'fast'}, 'spd', {5, 40}, 'pf', {2, 4}, 'frac', {1, 0.25}, 'lab', {'5 um/s, one full turn', '40 um/s, a quarter turn'}, ...
            'xl', {[-22 22], [-3 22]}, 'yl', {[-22 31], [-5 31]}, 'leg', {'eastoutside', 'eastoutside'});
rs = cell(1, 2);
if isempty(gcp('nocreate')); parpool('Processes', 2); end
parfor ic = 1:2
    c = cs(ic);  o = base;  o.du_d = c.spd * Ts / Rr;  o.sP_fix_theta_deg = c.pf;  o.scan_len = c.frac * Lturn;  o.T = tS + c.frac * Lturn / c.spd + 0.2;
    R = run_t3_xyz_fixed(o);  rs{ic} = R.runs{1};
end
nm = {'u', 'v', 'w'};
for ic = 1:2
    c = cs(ic);  r = rs{ic};  Te = tS + c.frac * Lturn / c.spd;  K = find(r.t <= Te & (1:numel(r.t))' <= r.kend);
    ty = unwrap(r.th_tr(K, 2) * pi / 180) * 180 / pi;  k = find(r.t(K) >= tS);  t = r.t(K(k)) - tS;
    % ---------------- (1) the scan: afternoon layout of plot_1028_hemi_mer.m ----------------
    f = figure('Color', 'w', 'Position', [30 30 900 1500]);  tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl, 1, [2 1]);  hold on;  box on;
    tg = linspace(0, 2 * pi, 400);  fill(rb * sin(tg), rb * cos(tg), [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(rb * sin(tg), rb * cos(tg), 'Color', cGr, 'LineWidth', 3, 'DisplayName', 'wall');
    p0 = r.p(1, [1 3]);  pw = rb * p0 / norm(p0);  plot([p0(1) pw(1)], [p0(2) pw(2)], 'k--', 'LineWidth', 1.6, 'DisplayName', 'start along the true w');
    plot(r.p(K, 1), r.p(K, 3), 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'p');
    plot(p0(1), p0(2), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 'start');
    plot(r.p(K(k(1)), 1), r.p(K(k(1)), 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 't_{scan}');
    axis equal;  xlim(c.xl);  ylim(c.yl);  xlabel('x  [\mum]');  ylabel('z  [\mum]');  legend('Location', c.leg);
    title(sprintf('%s, angle P_{fix} %d deg, seed %d', c.lab, c.pf, seed));
    axg = nexttile(tl, 3);  hold on;  box on;
    h1 = plot(t, r.aN_tr(K(k)), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.aT_tr(K(k)), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.aN(K(k)), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(t, r.aT(K(k)), 'Color', cG, 'LineWidth', 1.4);  h4 = yline(ad, 'k--', 'LineWidth', 1.5);
    ylim([0 1.1]);  ylabel('gain');
    legend([h1 h2 h3 h4], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc), sprintf('%s_d = %g', AB, ad)}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal');
    axa = nexttile(tl, 4);  hold on;  box on;
    h1 = plot(t, ty(k), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.th_tr(K(k), 1), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.thy(K(k)), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(t, r.thx(K(k)), 'Color', cG, 'LineWidth', 1.4);
    ylabel(sprintf('%s  [\\circ]', th));  xlabel('time since t_{scan}  (sec)');
    legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    for a = [ax axg axa]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
    set([axg axa], 'XLim', [0 t(end)]);
    fn1 = fullfile(fo, sprintf('fig_1028_pm_%s_scan.png', c.name));  exportgraphics(f, fn1, 'Resolution', 130);  close(f);
    % ---------------- (2) before the scan + tracking error: afternoon layout of plot_1028_hemi_mer_extra.m ----------------
    f = figure('Color', 'w', 'Position', [30 30 2100 1300]);  tl = tiledlayout(6, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    kb = find(r.t(K) < tS);  tb = r.t(K(kb));
    axg = nexttile(tl, 1, [3 1]);  hold on;  box on;
    h1 = plot(tb, r.aN_tr(K(kb)), 'Color', cT, 'LineWidth', 2.2);  plot(tb, r.aT_tr(K(kb)), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(tb, r.aN(K(kb)), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(tb, r.aT(K(kb)), 'Color', cG, 'LineWidth', 1.4);  h4 = yline(ad, 'k--', 'LineWidth', 1.5);
    ylim([0 1.1]);  ylabel('gain');
    legend([h1 h2 h3 h4], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc), sprintf('%s_d = %g', AB, ad)}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal');
    title(axg, sprintf('before the scan: hold 0-%d s, regulation to a_d %d-%d s', tH, tH, tS));
    axa = nexttile(tl, 7, [3 1]);  hold on;  box on;
    h1 = plot(tb, ty(kb), 'Color', cT, 'LineWidth', 2.2);  plot(tb, r.th_tr(K(kb), 1), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(tb, r.thy(K(kb)), 'Color', cE, 'LineWidth', 1.4);  h3 = plot(tb, r.thx(K(kb)), 'Color', cG, 'LineWidth', 1.4);
    ylabel(sprintf('%s  [\\circ]', th));  xlabel('Time (sec)');
    legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    for a = [axg axa]; xline(a, tH, '--', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');  set(a, 'XLim', [0 tS]); end
    dw = zeros(numel(K), 3);
    for j = 1:numel(K)
        a = r.th_tr(K(j), :) * pi / 180;  Vt = [cos(a(2)) 0 sin(a(2)); 0 1 0; -sin(a(2)) 0 cos(a(2))] * [1 0 0; 0 cos(a(1)) -sin(a(1)); 0 sin(a(1)) cos(a(1))];
        dw(j, :) = (Vt' * (r.pd(K(j), :) - r.p(K(j), :))')';
    end
    axd = gobjects(1, 3);
    for row = 1:3
        axd(row) = nexttile(tl, 2 + 4 * (row - 1), [2 1]);  hold on;  box on;  plot(r.t(K), dw(:, row), 'Color', cE, 'LineWidth', 0.6);  ylim([-0.2 0.2]);
        ylabel(sprintf('%s%s  [%sm]', DL, nm{row}, mu));  set(axd(row), 'XLim', [0 Te]);
        xline(axd(row), tH, '--', 'Color', cGr, 'LineWidth', 1.3);  xline(axd(row), tS, '-.', 'Color', cGr, 'LineWidth', 1.3);
    end
    xlabel(axd(3), 'Time (sec)');  title(axd(1), sprintf('tracking error (true local), %s', c.lab));
    for a = [axg axa axd]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
    fn2 = fullfile(fo, sprintf('fig_1028_pm_%s_extra.png', c.name));  exportgraphics(f, fn2, 'Resolution', 130);  close(f);
    system(['open ' fn1 ' ' fn2]);
end

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
