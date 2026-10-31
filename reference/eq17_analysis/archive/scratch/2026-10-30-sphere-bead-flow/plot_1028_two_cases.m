% STATUS: SCRATCH -- 2026-10-28 (user): two settled cases on the whole sphere (radius 15 um, void), two figures each (before the scan | the scan).
%   Settled flow: start 10 deg off the top, 15 um up, theta-hat[0] = 0, angle prior 10 deg; hold 0-10 s; regulation tau_a 2 s to a_d = 0.05
%   (law-derived a' floor); great-circle scan in the x-z plane from 17 s; slope P_fix law (L); document version; seed 1.
%   Case slow: 5 um/s, one full turn, angle P_fix 2 deg.  Case fast: 40 um/s, a quarter turn, angle P_fix 4 deg.
%   Before-scan figure (0-17 s): left gains, a'_33 vs the true slope, angles (truth red); right tracking error delta u, v, w in the TRUE local
%   coordinate (V_k' (p_d - p)); dashed = end of the hold. Scan figure: left side view, gains, a'_33, angles; right delta u, v, w; x axis =
%   position around the circle (deg from the top). Outputs test_results/temp_3d_wsonly/fig_1028_case_{slow,fast}_{before,scan}.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  ad = 0.05;  rb = 15;  tH = 10;  tS = 17;  ph0 = 10;  Ts = 1 / 1612;  seed = 1;
FS = 14;  hatc = char(770);  AB = char(257);  th = char(952);  mu = char(181);  DL = char(948);
cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];
hs = fzero(@(h) 1 / local_cp(h) - ad, [1.001 30]);  Lturn = 2 * pi * (rb + hs * Rr);
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 10, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', ad, ...
              't_scan', tS, 'ap_q', true, 'q_theta_from', tS, 'wall', 'hemi', 'hemi_r', rb, 'hemi_void', true, 'ap_floor_law', true, ...
              'scan_dir', [1 0], 'hemi_start_deg', ph0, 'scan_plane_n', [0 1 0], 'ap_pfix_law', true, 'seeds', seed, 'tag', 'tc');
cs = struct('name', {'slow', 'fast'}, 'spd', {5, 40}, 'pf', {2, 4}, 'frac', {1, 0.25}, 'lab', {'5 \mum/s, one full turn', '40 \mum/s, a quarter turn'});
rs = cell(1, 2);
if isempty(gcp('nocreate')); parpool('Processes', 2); end
parfor ic = 1:2
    c = cs(ic);  o = base;  o.du_d = c.spd * Ts / Rr;  o.sP_fix_theta_deg = c.pf;  o.scan_len = c.frac * Lturn;  o.T = tS + c.frac * Lturn / c.spd + 0.2;
    R = run_t3_xyz_fixed(o);  rs{ic} = R.runs{1};
end
slope = @(hb) arrayfun(@(h) (1 / local_cp(max(h, 1.0003) + 1e-4) - 1 / local_cp(max(h, 1.0003) - 1e-4)) / 2e-4, hb);
nm = {'u', 'v', 'w'};
for ic = 1:2
    c = cs(ic);  r = rs{ic};  K = (1:r.kend)';  dw = zeros(numel(K), 3);
    for j = 1:numel(K)
        a = r.th_tr(K(j), :) * pi / 180;  Vt = [cos(a(2)) 0 sin(a(2)); 0 1 0; -sin(a(2)) 0 cos(a(2))] * [1 0 0; 0 cos(a(1)) -sin(a(1)); 0 sin(a(1)) cos(a(1))];
        dw(j, :) = (Vt' * (r.pd(K(j), :) - r.p(K(j), :))')';
    end
    kb = K(r.t(K) < tS);  ks = K(r.t(K) >= tS & r.t(K) <= tS + c.frac * Lturn / c.spd);
    ty = unwrap(r.th_tr(K, 2) * pi / 180) * 180 / pi;  phi = unwrap(atan2(r.p(ks, 1), r.p(ks, 3))) * 180 / pi;
    fprintf('%s: contact %d | at 17 s: a33 err %+.3f, a''33 err %+.2f, angle err %.1f deg | scan: mean |angle err| %.1f, max %.1f deg, gap %.2f-%.2f um, rms a33 err %.3f, a''33 err %.2f | rms delta u v w before %s, scan %s um\n', ...
            c.name, r.contact, r.aN(kb(end)) - r.aN_tr(kb(end)), r.apN(kb(end)) - slope(r.h(kb(end))), ty(kb(end)) - r.thy(kb(end)), mean(abs(ty(ks) - r.thy(ks))), ...
            max(abs(ty(ks) - r.thy(ks))), min(r.h(ks)) * Rr, max(r.h(ks)) * Rr, rms(r.aN(ks) - r.aN_tr(ks)), rms(r.apN(ks(1:5:end)) - slope(r.h(ks(1:5:end)))), ...
            mat2str(round(sqrt(mean(dw(r.t(K) >= tH & r.t(K) < tS, :).^2)), 3)), mat2str(round(sqrt(mean(dw(ismember(K, ks), :).^2)), 3)));
    % ---------------- before the scan ----------------
    f = figure('Color', 'w', 'Position', [30 30 2000 1250]);  tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  t = r.t(kb);
    ag = nexttile(tl, 1);  local_gains(ag, t, r, kb, cT, cE, cG, AB, hatc);  title(ag, sprintf('%s: before the scan (hold 0-10 s, regulation 10-17 s), seed %d', c.lab, seed));
    ap = nexttile(tl, 3);  local_ap(ap, t, r, kb, slope, cT, cE, AB, hatc);
    aa = nexttile(tl, 5);  local_ang(aa, t, ty(kb), r, kb, cT, cE, cG, th, hatc);  xlabel(aa, 'Time (sec)');
    text(aa, 0.98, 0.15, sprintf('angle error at 17 s: %.1f°', ty(kb(end)) - r.thy(kb(end))), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS - 1, 'FontWeight', 'bold');
    ad3 = gobjects(1, 3);
    for row = 1:3
        ad3(row) = nexttile(tl, 2 * row);  hold on;  box on;  plot(t, dw(ismember(K, kb), row), 'Color', cE, 'LineWidth', 0.6);  ylim([-0.2 0.2]);
        ylabel(sprintf('%s%s  [%sm]', DL, nm{row}, mu));
        text(ad3(row), 0.98, 0.88, sprintf('rms hold %.3f, regulation %.3f %sm', rms(dw(r.t(K) < tH, row)), rms(dw(r.t(K) >= tH & r.t(K) < tS, row)), mu), 'Units', 'normalized', ...
             'HorizontalAlignment', 'right', 'FontSize', FS - 1, 'FontWeight', 'bold');
    end
    title(ad3(1), sprintf('tracking error in the true local coordinate  %s = V''(p_d - p)', DL));  xlabel(ad3(3), 'Time (sec)');
    for a = [ag ap aa ad3]; xline(a, tH, '--', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');  set(a, 'XLim', [0 tS], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
    fn1 = fullfile(fo, sprintf('fig_1028_case_%s_before.png', c.name));  exportgraphics(f, fn1, 'Resolution', 140);  close(f);
    % ---------------- the scan ----------------
    f = figure('Color', 'w', 'Position', [30 30 2000 1900]);  tl = tiledlayout(9, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl, 1, [3 1]);  hold on;  box on;
    tg = linspace(0, 2 * pi, 400);  fill(rb * sin(tg), rb * cos(tg), [0.9 0.9 0.9], 'EdgeColor', cGr, 'LineWidth', 3, 'DisplayName', 'wall (sphere)');
    kp = K(r.t(K) <= tS + c.frac * Lturn / c.spd);  plot(r.p(kp, 1), r.p(kp, 3), 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'p');
    plot(r.p(1, 1), r.p(1, 3), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 'start');
    plot(r.p(ks(1), 1), r.p(ks(1), 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 't_{scan}');
    plot(r.p(ks(end), 1), r.p(ks(end), 3), 'o', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 8, 'DisplayName', 'end');
    axis equal;  xlim([-22 22]);  ylim([-22 31]);  xlabel('x  [\mum]');  ylabel('z  [\mum]');  legend('Location', 'eastoutside');
    title(sprintf('%s: the scan, seed %d', c.lab, seed));  set(ax, 'FontSize', FS, 'FontWeight', 'bold');  set(ax.Toolbar, 'Visible', 'off');
    text(ax, 0.02, 0.04, sprintf('gap %.2f - %.2f %sm (target %.2f)', min(r.h(ks)) * Rr, max(r.h(ks)) * Rr, mu, hs * Rr), 'Units', 'normalized', 'FontSize', FS - 1, 'FontWeight', 'bold');
    ag = nexttile(tl, 7, [2 1]);  local_gains(ag, phi, r, ks, cT, cE, cG, AB, hatc);
    text(ag, 0.98, 0.88, sprintf('rms err %s%s_{33} %.3f', AB, hatc, rms(r.aN(ks) - r.aN_tr(ks))), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS - 1, 'FontWeight', 'bold');
    ap = nexttile(tl, 11, [2 1]);  local_ap(ap, phi, r, ks, slope, cT, cE, AB, hatc);
    aa = nexttile(tl, 15, [2 1]);  local_ang(aa, phi, ty(ismember(K, ks)), r, ks, cT, cE, cG, th, hatc);  xlabel(aa, 'position around the circle  \phi  [\circ from the top]');
    text(aa, 0.98, 0.15, sprintf('mean |error| %.1f°, max %.1f°', mean(abs(ty(ismember(K, ks)) - r.thy(ks))), max(abs(ty(ismember(K, ks)) - r.thy(ks)))), 'Units', 'normalized', ...
         'HorizontalAlignment', 'right', 'FontSize', FS - 1, 'FontWeight', 'bold');
    ad3 = gobjects(1, 3);
    for row = 1:3
        ad3(row) = nexttile(tl, 2 + 6 * (row - 1), [3 1]);  hold on;  box on;  plot(phi, dw(ismember(K, ks), row), 'Color', cE, 'LineWidth', 0.6);  ylim([-0.2 0.2]);
        ylabel(sprintf('%s%s  [%sm]', DL, nm{row}, mu));
        text(ad3(row), 0.98, 0.9, sprintf('rms %.3f %sm', rms(dw(ismember(K, ks), row)), mu), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS - 1, 'FontWeight', 'bold');
    end
    title(ad3(1), sprintf('tracking error in the true local coordinate  %s = V''(p_d - p)', DL));  xlabel(ad3(3), 'position around the circle  \phi  [\circ from the top]');
    for a = [ag ap aa ad3]; set(a, 'XLim', [phi(1) phi(end)], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off'); end
    fn2 = fullfile(fo, sprintf('fig_1028_case_%s_scan.png', c.name));  exportgraphics(f, fn2, 'Resolution', 140);  close(f);
    system(['open ' fn1 ' ' fn2]);
end

function local_gains(a, x, r, k, cT, cE, cG, AB, hatc)
    hold(a, 'on');  box(a, 'on');
    h1 = plot(a, x, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.2);  plot(a, x, r.aT_tr(k), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(a, x, r.aN(k), 'Color', cE, 'LineWidth', 1.2);  h3 = plot(a, x, r.aT(k), 'Color', cG, 'LineWidth', 1.2);  h4 = yline(a, 0.05, 'k--', 'LineWidth', 1.3);
    ylim(a, [0 1.05]);  ylabel(a, 'gain');
    legend(a, [h1 h2 h3 h4], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc), sprintf('%s_d', AB)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
end
function local_ap(a, x, r, k, slope, cT, cE, AB, hatc)
    hold(a, 'on');  box(a, 'on');  kd = 1:10:numel(k);
    h1 = plot(a, x(kd), slope(r.h(k(kd))), 'Color', cT, 'LineWidth', 2.2);  h2 = plot(a, x, r.apN(k), 'Color', cE, 'LineWidth', 1.2);  ylim(a, [0 2]);
    ylabel(a, sprintf('%s''_{33}', AB));  legend(a, [h1 h2], {sprintf('d%s_{33}/dh (true)', AB), sprintf('%s%s''_{33}', AB, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
end
function local_ang(a, x, ty, r, k, cT, cE, cG, th, hatc)
    hold(a, 'on');  box(a, 'on');
    h1 = plot(a, x, ty, 'Color', cT, 'LineWidth', 2.2);  plot(a, x, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(a, x, r.thy(k), 'Color', cE, 'LineWidth', 1.2);  h3 = plot(a, x, r.thx(k), 'Color', cG, 'LineWidth', 1.2);
    ylabel(a, sprintf('%s  [\\circ]', th));
    legend(a, [h1 h2 h3], {sprintf('%s_y, %s_x (true)', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
