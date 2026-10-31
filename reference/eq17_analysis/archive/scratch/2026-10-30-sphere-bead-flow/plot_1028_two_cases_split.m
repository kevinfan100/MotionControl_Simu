% STATUS: SCRATCH -- 2026-10-28 (user): the two settled cases, three figures each, afternoon style, no numbers on the figures, no a'.
%   Whole sphere (radius 15 um, void), start 10 deg off the top 15 um up, theta-hat[0] = 0, angle prior 10 deg; hold 0-10 s; regulation
%   tau_a 2 s to a_d = 0.05; great-circle scan in the x-z plane from 17 s; slope P_fix law (L); document version; seed 1.
%   Case slow: 5 um/s, one full turn, angle P_fix 2 deg.  Case fast: 40 um/s, a quarter turn, angle P_fix 4 deg.
%   (1) _before: 0-17 s -- side view of the descent (dashed = start straight down the true normal), gains, angles; dashed = end of the hold.
%   (2) _scan: the scan -- side view, gains, angles vs time since t_scan.
%   (3) _track: tracking error delta u, v, w (true local coordinate); left 0-17 s (dashed = end of the hold), right the scan.
%   Outputs test_results/temp_3d_wsonly/fig_1028_{slow,fast}_{before,scan,track}.png; numbers in the console.
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  tS = 17;  ph0 = 10;  Ts = 1 / 1612;  seed = 1;
FS = 15;  hatc = char(770);  AB = char(257);  th = char(952);  mu = char(181);  DL = char(948);
cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
              'scan_dir', [1 0], 'hemi_start_deg', ph0, 'scan_plane_n', [0 1 0], 'ap_pfix_law', true, 'seeds', seed, 'tag', 'cs');
cs = struct('name', {'slow', 'fast'}, 'spd', {5, 40}, 'pf', {2, 4}, 'frac', {1, 0.25}, 'lab', {'5 um/s, one full turn', '40 um/s, a quarter turn'}, ...
            'xl', {[-22 22], [-2 22]}, 'yl', {[-21 21], [-5 21]});
rs = cell(1, 2);
if isempty(gcp('nocreate')); parpool('Processes', 2); end
parfor ic = 1:2
    c = cs(ic);  o = base;  o.du_d = c.spd * Ts / Rr;  o.sP_fix_theta_deg = c.pf;  o.scan_len = c.frac * Lturn;  o.T = tS + c.frac * Lturn / c.spd + 0.2;
    R = run_t3_xyz_fixed(o);  rs{ic} = R.runs{1};
end
nm = {'u', 'v', 'w'};
for ic = 1:2
    c = cs(ic);  r = rs{ic};  Te = tS + c.frac * Lturn / c.spd;  K = find(r.t <= Te & (1:numel(r.t))' <= r.kend);  t = r.t(K);
    ty = unwrap(r.th_tr(K, 2) * pi / 180) * 180 / pi;  dw = zeros(numel(K), 3);
    for j = 1:numel(K)
        a = r.th_tr(K(j), :) * pi / 180;  Vt = [cos(a(2)) 0 sin(a(2)); 0 1 0; -sin(a(2)) 0 cos(a(2))] * [1 0 0; 0 cos(a(1)) -sin(a(1)); 0 sin(a(1)) cos(a(1))];
        dw(j, :) = (Vt' * (r.pd(K(j), :) - r.p(K(j), :))')';
    end
    ib = t < tS;  is = t >= tS;  kS = K(find(is, 1));
    fprintf('%s: angle err at 10 s %.1f, at 17 s %.1f deg | a33 err at 17 s %+.3f | scan: mean |angle err| %.1f, max %.1f deg, gap %.2f-%.2f um | rms delta u v w: hold %s, regulation %s, scan %s um\n', ...
            c.name, ty(find(t >= tH, 1)) - r.thy(K(find(t >= tH, 1))), ty(find(is, 1)) - r.thy(kS), r.aN(kS) - r.aN_tr(kS), mean(abs(ty(is) - r.thy(K(is)))), ...
            max(abs(ty(is) - r.thy(K(is)))), min(r.h(K(is))) * Rr, max(r.h(K(is))) * Rr, mat2str(round(sqrt(mean(dw(t < tH, :).^2)), 3)), ...
            mat2str(round(sqrt(mean(dw(t >= tH & ib, :).^2)), 3)), mat2str(round(sqrt(mean(dw(is, :).^2)), 3)));
    % ---------------- (1) before the scan ----------------
    f = figure('Color', 'w', 'Position', [40 30 1300 1450]);  tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl, 1, [2 1]);  local_side(ax, r, K(ib), rb, cE, cGr, true, [-3 12], [12 31]);
    title(ax, sprintf('%s, seed %d: before the scan (hold 0-10 s, regulation 10-17 s)', c.lab, seed));
    axg = nexttile(tl, 3);  local_gains(axg, t(ib), r, K(ib), ad, cT, cE, cG, AB, hatc);
    axa = nexttile(tl, 4);  local_ang(axa, t(ib), ty(ib), r, K(ib), cT, cE, cG, th, hatc);  ylim(axa, [-5 20]);  xlabel(axa, 'Time (sec)');
    for a = [axg axa]; xline(a, tH, '--', 'Color', cGr, 'LineWidth', 1.4, 'HandleVisibility', 'off');  set(a, 'XLim', [0 tS]);  local_ax(a, FS); end
    local_ax(ax, FS);  fn1 = fullfile(fo, sprintf('fig_1028_%s_before.png', c.name));  exportgraphics(f, fn1, 'Resolution', 150);  close(f);
    % ---------------- (2) the scan ----------------
    ts = t(is) - tS;
    f = figure('Color', 'w', 'Position', [40 30 1300 1450]);  tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl, 1, [2 1]);  local_side(ax, r, K(is), rb, cE, cGr, false, c.xl, c.yl);
    title(ax, sprintf('%s, seed %d: the scan', c.lab, seed));
    axg = nexttile(tl, 3);  local_gains(axg, ts, r, K(is), ad, cT, cE, cG, AB, hatc);
    axa = nexttile(tl, 4);  local_ang(axa, ts, ty(is), r, K(is), cT, cE, cG, th, hatc);  xlabel(axa, 'time since t_{scan}  (sec)');
    for a = [axg axa]; set(a, 'XLim', [0 ts(end)]);  local_ax(a, FS); end
    local_ax(ax, FS);  fn2 = fullfile(fo, sprintf('fig_1028_%s_scan.png', c.name));  exportgraphics(f, fn2, 'Resolution', 150);  close(f);
    % ---------------- (3) tracking error ----------------
    f = figure('Color', 'w', 'Position', [40 30 1900 1150]);  tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for row = 1:3
        a1 = nexttile(tl);  hold on;  box on;  plot(t(ib), dw(ib, row), 'Color', cE, 'LineWidth', 0.6);  xline(a1, tH, '--', 'Color', cGr, 'LineWidth', 1.4);
        set(a1, 'XLim', [0 tS], 'YLim', [-0.2 0.2]);  ylabel(a1, sprintf('%s%s  [%sm]', DL, nm{row}, mu));  local_ax(a1, FS);
        a2 = nexttile(tl);  hold on;  box on;  plot(ts, dw(is, row), 'Color', cE, 'LineWidth', 0.6);  set(a2, 'XLim', [0 ts(end)], 'YLim', [-0.2 0.2]);  local_ax(a2, FS);
        if row == 1; title(a1, sprintf('%s: before the scan', c.lab));  title(a2, 'the scan'); end
        if row == 3; xlabel(a1, 'Time (sec)');  xlabel(a2, 'time since t_{scan}  (sec)'); end
    end
    fn3 = fullfile(fo, sprintf('fig_1028_%s_track.png', c.name));  exportgraphics(f, fn3, 'Resolution', 150);  close(f);
    system(['open ' fn1 ' ' fn2 ' ' fn3]);
end

function local_side(ax, r, k, rb, cE, cGr, showw, xl, yl)
    hold(ax, 'on');  box(ax, 'on');  tg = linspace(0, 2 * pi, 400);
    fill(ax, rb * sin(tg), rb * cos(tg), [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(ax, rb * sin(tg), rb * cos(tg), 'Color', cGr, 'LineWidth', 3, 'DisplayName', 'wall');
    if showw; p0 = r.p(1, [1 3]);  pw = rb * p0 / norm(p0);  plot(ax, [p0(1) pw(1)], [p0(2) pw(2)], 'k--', 'LineWidth', 1.6, 'DisplayName', 'start along the true w'); end
    plot(ax, r.p(k, 1), r.p(k, 3), 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'p');
    if showw
        plot(ax, r.p(k(1), 1), r.p(k(1), 3), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'DisplayName', 'start');
    end
    plot(ax, r.p(k(end), 1), r.p(k(end), 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'DisplayName', 't_{scan}');
    if ~showw; set(findobj(ax, 'DisplayName', 't_{scan}'), 'XData', r.p(k(1), 1), 'YData', r.p(k(1), 3)); end
    if ~showw; plot(ax, r.p(k(end), 1), r.p(k(end), 3), 'o', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 8, 'DisplayName', 'end of the scan'); end
    axis(ax, 'equal');  xlim(ax, xl);  ylim(ax, yl);  xlabel(ax, 'x  [\mum]');  ylabel(ax, 'z  [\mum]');  legend(ax, 'Location', 'eastoutside');
end
function local_gains(a, x, r, k, ad, cT, cE, cG, AB, hatc)
    hold(a, 'on');  box(a, 'on');
    h1 = plot(a, x, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  plot(a, x, r.aT_tr(k), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(a, x, r.aN(k), 'Color', cE, 'LineWidth', 1.3);  h3 = plot(a, x, r.aT(k), 'Color', cG, 'LineWidth', 1.3);  h4 = yline(a, ad, 'k--', 'LineWidth', 1.3);
    ylim(a, [0 1.05]);  ylabel(a, 'gain');
    legend(a, [h1 h2 h3 h4], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc), sprintf('%s_d', AB)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
end
function local_ang(a, x, ty, r, k, cT, cE, cG, th, hatc)
    hold(a, 'on');  box(a, 'on');
    h1 = plot(a, x, ty, 'Color', cT, 'LineWidth', 2.2);  plot(a, x, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(a, x, r.thy(k), 'Color', cE, 'LineWidth', 1.3);  h3 = plot(a, x, r.thx(k), 'Color', cG, 'LineWidth', 1.3);
    ylabel(a, sprintf('%s  [\\circ]', th));
    legend(a, [h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
end
function local_ax(a, FS)
    set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
