% STATUS: SCRATCH -- 2026-10-28 (user): curved walls with the document estimator (run_t3_xyz_fixed form 'local', doc_timing + theta_out,
%   one-sided gate 2 sqrt P). Plant: plane law on the local tangent plane of the nearest surface point (thermal force included).
%   Flow: start 15 um above the surface, hold 0-10 s (no dither), regulation to a_d = 0.1 from 10 s (tau_a 2 s), scan along u-hat from
%   17 s at du_d = 0.011 R per step (as the 10-22 bend run), 20 s total. Q: slopes P_fix^2 I from the start (I ~ 0 without motion), angle
%   P_fix (2 deg) on the off-diagonal slots from t_scan. theta-hat[0] = 0, prior 5 deg, gains at the seeds. Seeds 1-3, one shown.
%   Walls: 'bend' (as run_t2_3d_reg: tilt 0 -> 20 deg over 20 um, downslope azimuth 45 deg, bend 10 um ahead) | 'hemi' (radius 15 um,
%   start above the top, single line toward +x). Figure per wall: side view x-z (wall, p); a-hat_33 (a_d), a-hat_11; theta-hat_y,
%   theta-hat_x vs truth; angle change since t_scan: truth vs sum of delta-theta-hat. Dashed: end of hold, dash-dot: t_scan.
%   Outputs test_results/temp_3d_wsonly/fig_1028_{bend,hemi}_s<seed>.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  seeds = 1:3;  n = numel(seeds);  walls = {'bend', 'hemi'};  FS = 14;  hatc = char(770);  th = ['θ' hatc];  ah = ['a' hatc];  mu = char(181);
tH = 10;  tS = 17;  T = 20;
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'T', T, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 5, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', 0.1, ...
              't_scan', tS, 'du_d', 0.011, 'ap_q', true, 'q_theta_from', tS, 'sP_fix_theta_deg', 2, 'hemi_r', 15, ...
              'bend_alpha', 20, 'bend_phi', 45, 'bend_Lx', 20, 'bend_xb', 10);
if isempty(gcp('nocreate')); parpool('Processes', min(6, feature('numcores'))); end
jobs = cell(1, 2 * n);
parfor q = 1:2 * n
    iw = 1 + floor((q - 1) / n);  i = q - (iw - 1) * n;  o = base;  o.wall = walls{iw};  o.seeds = seeds(i);  o.tag = 'cw';
    if strcmp(o.wall, 'hemi'); o.scan_len = pi / 2 * (o.hemi_r + 1.117 * Rr);  o.T = tS + 1; end   % 10-28 (user): stop at the edge, 1 s after t_scan
    R = run_t3_xyz_fixed(o);  jobs{q} = R.runs{1};
end
for iw = 1:2
    rs = jobs((iw - 1) * n + (1:n));  sc = zeros(1, n);
    for i = 1:n
        r = rs{i};  k = find(r.t >= tS & (1:numel(r.t))' <= r.kend);
        sc(i) = mean(abs(r.thy(k) - r.th_tr(k, 2)) + abs(r.thx(k) - r.th_tr(k, 1)));
        tq = min([tH tS tS + 0.5 tS + 1 T], r.t(end));  kk = arrayfun(@(s) find(r.t >= s - 1e-9, 1), tq);  kk = min(kk, r.kend);
        fprintf('%-4s seed %d: contact %d | angle error (y, x) at %s s: %s / %s deg | gap um %s | mean |err| after t_scan %.2f\n', walls{iw}, seeds(i), r.contact, ...
                mat2str(round(tq, 2)), mat2str(round((r.thy(kk) - r.th_tr(kk, 2))', 1)), mat2str(round((r.thx(kk) - r.th_tr(kk, 1))', 1)), ...
                mat2str(round(r.h(kk)' * Rr, 2)), sc(i));
    end
    [~, ib] = min(sc);  r = rs{ib};  k = 1:r.kend;  t = r.t(k);
    if strcmp(walls{iw}, 'hemi')                                % 10-28 (user): draw only until the probe leaves the hemisphere (nearest surface = plane)
        kS0 = find(t >= tS, 1);  kE = kS0 - 1 + find(abs(r.th_tr(kS0:end, 2)) < 1e-9 & (1:numel(t) - kS0 + 1)' > 10, 1);
        if ~isempty(kE); k = 1:kE - 1;  t = r.t(k);  fprintf('hemi: drawn until t = %.3f s (edge)\n', t(end)); end
    end
    Tw = t(end);  fig = figure('Color', 'w', 'Position', [40 30 1500 1500]);  tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    % side view
    a = nexttile;  hold on;  box on;  axis equal;
    xs = linspace(min(r.p(k, 1)) - 10, max(r.p(k, 1)) + 10, 600);  if strcmp(walls{iw}, 'hemi'); xs = linspace(-25, 30, 600); end;  zs = zeros(size(xs));  ys = median(r.p(k, 2));
    for j = 1:numel(xs)
        [~, wj, pzj] = local_wall_ref(walls{iw}, [xs(j); ys; 200], base);  zs(j) = (pzj - wj(1) * xs(j) - wj(2) * ys) / wj(3);
    end
    fill([xs fliplr(xs)], [zs, min(zs) - 5 + 0 * xs], [0.9 0.9 0.9], 'EdgeColor', 'none');  plot(xs, zs, 'Color', [0.45 0.45 0.45], 'LineWidth', 3);
    plot(r.p(k, 1), r.p(k, 3), 'b-', 'LineWidth', 1.6);
    kH = find(t >= tH, 1);  kS = find(t >= tS, 1);
    plot(r.p(kH, 1), r.p(kH, 3), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 9);  plot(r.p(kS, 1), r.p(kS, 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9);
    if strcmp(walls{iw}, 'hemi'); xlim([-25 30]);  ylim([-3 33]); end
    xlabel(sprintf('x  [%sm]', mu));  ylabel(sprintf('z  [%sm]', mu));  legend({'', 'wall (y of the path)', 'p', 'end of hold', 't_{scan}'}, 'Location', 'northeastoutside');
    set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(a.Toolbar, 'Visible', 'off');
    % gains
    ag = nexttile;  hold on;  box on;
    plot(t, r.aN_tr(k), 'r', 'LineWidth', 2.2);  plot(t, r.aT_tr(k), 'Color', [1 0.55 0.55], 'LineWidth', 2.2);
    h1 = plot(t, r.aN(k), 'Color', [0 0.25 0.85], 'LineWidth', 1.3);  h2 = plot(t, r.aT(k), 'Color', [0.85 0.4 0], 'LineWidth', 1.3);
    yline(0.1, 'k:', 'LineWidth', 1.5);  ylim([0 1.05]);  ylabel('gain');
    legend([h1 h2], {sprintf('%s_{33}  (true red, a_d dotted)', ah), sprintf('%s_{11}  (true light red)', ah)}, 'Location', 'northeastoutside');
    % angles
    a2 = nexttile;  hold on;  box on;
    plot(t, r.th_tr(k, 2), 'r', 'LineWidth', 2.2);  plot(t, r.th_tr(k, 1), 'Color', [1 0.55 0.55], 'LineWidth', 2.2);
    h1 = plot(t, r.thy(k), 'Color', [0 0.25 0.85], 'LineWidth', 1.3);  h2 = plot(t, r.thx(k), 'Color', [0 0.5 0.2], 'LineWidth', 1.3);
    ylabel('[\circ]');  legend([h1 h2], {sprintf('%s_y  (true red)', th), sprintf('%s_x  (true light red)', th)}, 'Location', 'northeastoutside');
    % angle change since t_scan
    a3 = nexttile;  hold on;  box on;  kk = k(t >= tS);  tt = r.t(kk);
    plot(tt, r.th_tr(kk, 2) - r.th_tr(kS, 2), 'r', 'LineWidth', 2.2);  plot(tt, r.th_tr(kk, 1) - r.th_tr(kS, 1), 'Color', [1 0.55 0.55], 'LineWidth', 2.2);
    h1 = plot(tt, cumsum(r.dth(kk, 2)), 'Color', [0 0.25 0.85], 'LineWidth', 1.3);  h2 = plot(tt, cumsum(r.dth(kk, 1)), 'Color', [0 0.5 0.2], 'LineWidth', 1.3);
    ylabel('[\circ]');  xlabel('Time (sec)');
    legend([h1 h2], {sprintf('Σ δ%s_y  (true change red)', th), sprintf('Σ δ%s_x  (true change light red)', th)}, 'Location', 'northeastoutside');
    for ax = [ag a2 a3]
        xline(ax, tH, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.2, 'HandleVisibility', 'off');  xline(ax, tS, '-.', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.2, 'HandleVisibility', 'off');
        set(ax, 'XLim', [0 Tw], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(ax.Toolbar, 'Visible', 'off');
    end
    set(a3, 'XLim', [tS - 0.2, Tw]);
    exportgraphics(fig, fullfile(fo, sprintf('fig_1028_%s_s%d.png', walls{iw}, seeds(ib))), 'Resolution', 150);  close(fig);
    fprintf('%s: figure seed %d\n', walls{iw}, seeds(ib));
    system(['open ' fullfile(fo, sprintf('fig_1028_%s_s%d.png', walls{iw}, seeds(ib)))]);
end

function [W, w, pz] = local_wall_ref(wall, p, o)                % surface under (x, y): nearest-point wall of run_t3_xyz_fixed, p(3) far above
    W = struct();  pz0 = 0;
    switch wall
        case 'hemi'
            rb = o.hemi_r;  rr = norm(p(1:2));
            if rr < rb; n = [p(1:2); sqrt(rb^2 - rr^2)] / rb;  pz = rb; w = n;  pz = n' * (n * rb); else; w = [0; 0; 1];  pz = 0; end
        case 'bend'
            m = [cosd(o.bend_phi); sind(o.bend_phi)];  ae = o.bend_alpha * pi / 180;  Lxi = o.bend_Lx * m(1);
            xi = m' * p(1:2) - o.bend_xb * m(1);
            if xi <= 0; a = 0;  zw = 0; elseif xi <= Lxi; a = ae * xi / Lxi;  zw = (Lxi / ae) * log(cos(a)); else; a = ae;  zw = (Lxi / ae) * log(cos(ae)) - tan(ae) * (xi - Lxi); end
            w = [sin(a) * m(1); sin(a) * m(2); cos(a)];  pz = w' * [p(1); p(2); pz0 + zw];
    end
end
