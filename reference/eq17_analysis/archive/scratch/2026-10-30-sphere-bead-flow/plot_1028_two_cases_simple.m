% STATUS: SCRATCH -- 2026-10-28 (user): the two settled cases in the afternoon style, no numbers on the figures, no a'.
%   Whole sphere (radius 15 um, void), start 10 deg off the top 15 um up, theta-hat[0] = 0, angle prior 10 deg; hold 0-10 s; regulation
%   tau_a 2 s to a_d = 0.05; great-circle scan in the x-z plane from 17 s; slope P_fix law (L); document version; seed 1.
%   Case slow: 5 um/s, one full turn, angle P_fix 2 deg.  Case fast: 40 um/s, a quarter turn, angle P_fix 4 deg.
%   Figure 1 per case (single column): side view (dashed = start straight down the true normal to the sphere), gains, angles vs time;
%   dashed = end of the hold (10 s), dash-dot = t_scan. Figure 2 per case: tracking error delta u, v, w in the true local coordinate vs time.
%   Outputs test_results/temp_3d_wsonly/fig_1028_{slow,fast}_{est,track}.png; numbers in the console.
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
            'xl', {[-22 22], [-2 24]}, 'yl', {[-22 31], [-6 31]});
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
    ks = t >= tS;  kb = t >= tH & t < tS;
    fprintf('%s: contact %d | angle err at 17 s %.1f deg | scan: mean |angle err| %.1f, max %.1f deg, gap %.2f-%.2f um, rms a33 err %.3f, a11 err %.3f | rms delta u v w: hold %s, regulation %s, scan %s um\n', ...
            c.name, r.contact, ty(find(ks, 1)) - r.thy(K(find(ks, 1))), mean(abs(ty(ks) - r.thy(K(ks)))), max(abs(ty(ks) - r.thy(K(ks)))), min(r.h(K(ks))) * Rr, max(r.h(K(ks))) * Rr, ...
            rms(r.aN(K(ks)) - r.aN_tr(K(ks))), rms(r.aT(K(ks)) - r.aT_tr(K(ks))), mat2str(round(sqrt(mean(dw(t < tH, :).^2)), 3)), mat2str(round(sqrt(mean(dw(kb, :).^2)), 3)), ...
            mat2str(round(sqrt(mean(dw(ks, :).^2)), 3)));
    % ---------------- figure 1: side view, gains, angles ----------------
    f = figure('Color', 'w', 'Position', [40 30 1150 1500]);  tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl, 1, [2 1]);  hold on;  box on;
    tg = linspace(0, 2 * pi, 400);  fill(rb * sin(tg), rb * cos(tg), [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(rb * sin(tg), rb * cos(tg), 'Color', cGr, 'LineWidth', 3, 'DisplayName', 'wall');
    p0 = r.p(1, [1 3]);  pw = rb * p0 / norm(p0);  plot([p0(1) pw(1)], [p0(2) pw(2)], 'k--', 'LineWidth', 1.6, 'DisplayName', 'start along the true w');
    plot(r.p(K, 1), r.p(K, 3), 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'p');
    plot(p0(1), p0(2), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 'start');
    kS = K(find(ks, 1));  plot(r.p(kS, 1), r.p(kS, 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 't_{scan}');
    axis equal;  xlim(c.xl);  ylim(c.yl);  xlabel('x  [\mum]');  ylabel('z  [\mum]');  legend('Location', 'eastoutside');
    title(sprintf('%s, seed %d', c.lab, seed));  set(ax, 'FontSize', FS, 'FontWeight', 'bold');  set(ax.Toolbar, 'Visible', 'off');
    axg = nexttile(tl, 3);  hold on;  box on;
    h1 = plot(t, r.aN_tr(K), 'Color', cT, 'LineWidth', 2.2);  plot(t, r.aT_tr(K), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.aN(K), 'Color', cE, 'LineWidth', 1.3);  h3 = plot(t, r.aT(K), 'Color', cG, 'LineWidth', 1.3);  h4 = yline(ad, 'k--', 'LineWidth', 1.3);
    ylim([0 1.05]);  ylabel('gain');
    legend([h1 h2 h3 h4], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc), sprintf('%s_d', AB)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    axa = nexttile(tl, 4);  hold on;  box on;
    h1 = plot(t, ty, 'Color', cT, 'LineWidth', 2.2);  plot(t, r.th_tr(K, 1), 'Color', cT, 'LineWidth', 2.2);
    h2 = plot(t, r.thy(K), 'Color', cE, 'LineWidth', 1.3);  h3 = plot(t, r.thx(K), 'Color', cG, 'LineWidth', 1.3);
    ylabel(sprintf('%s  [\\circ]', th));  xlabel('Time (sec)');
    legend([h1 h2 h3], {sprintf('%s_y, %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    for a = [axg axa]
        xline(a, tH, '--', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');  xline(a, tS, '-.', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');
        set(a, 'XLim', [0 Te], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
    end
    fn1 = fullfile(fo, sprintf('fig_1028_%s_est.png', c.name));  exportgraphics(f, fn1, 'Resolution', 150);  close(f);
    % ---------------- figure 2: tracking error ----------------
    f = figure('Color', 'w', 'Position', [40 30 1500 1100]);  tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for row = 1:3
        a = nexttile(tl);  hold on;  box on;  plot(t, dw(:, row), 'Color', cE, 'LineWidth', 0.6);  ylim([-0.2 0.2]);
        ylabel(sprintf('%s%s  [%sm]', DL, nm{row}, mu));
        xline(a, tH, '--', 'Color', cGr, 'LineWidth', 1.5);  xline(a, tS, '-.', 'Color', cGr, 'LineWidth', 1.5);
        set(a, 'XLim', [0 Te], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.3);  set(a.Toolbar, 'Visible', 'off');
        if row == 1; title(a, sprintf('tracking error (true local coordinate), %s, seed %d', c.lab, seed)); end
        if row == 3; xlabel(a, 'Time (sec)'); end
    end
    fn2 = fullfile(fo, sprintf('fig_1028_%s_track.png', c.name));  exportgraphics(f, fn2, 'Resolution', 150);  close(f);
    system(['open ' fn1 ' ' fn2]);
end

function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
