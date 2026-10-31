% STATUS: SCRATCH -- 2026-10-28 (user): bend wall and hemisphere with the document estimator, one merged figure per wall:
%   left column -- side view (wall cut along the scan direction, p), gains (a_33 with a_d, a_11; true red), angles (true red);
%   right column -- motion error in the TRUE local coordinate, delta u, delta v, delta w = V_k' (p_d - p).
%   Run settings: run_t3_xyz_fixed form 'local' (doc_timing + theta_out, one-sided gate 2 sqrt P), hold 0-10 s, regulation to a_d = 0.1
%   (tau_a 2 s), scan from 17 s at du_d = 0.011 R per step, slopes P_fix^2 I, angle P_fix 2 deg from t_scan, prior 5 deg, seeds 1-3.
%   Hemisphere (radius 15 um): scan along (u-hat + v-hat)/sqrt 2 so that theta_x and theta_y both change; stops at the edge arc length;
%   drawn until the probe leaves the hemisphere. Dashed = end of hold, dash-dot = t_scan.
%   Outputs test_results/temp_3d_wsonly/fig_1028_{bend,hemi}_merged_s<seed>.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  seeds = 1:3;  n = numel(seeds);  walls = {'bend', 'hemi'};  FS = 16;  hatc = char(770);  AB = char(257);  th = char(952);  mu = char(181);  DL = char(948);
tH = 10;  tS = 17;  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  cGr = [0.5 0.5 0.5];
base = struct('theta_x', 0, 'theta_y', 0, 'h0', 15 / Rr, 'T', 20, 'aprime', true, 'th0_deg', [0 0], 'gain0', 'seed', 'sP_theta_deg', 5, ...
              'form', 'local', 'gap_gate', 2, 'gate_one', true, 'doc_timing', true, 'theta_out', true, 'hold_until', tH, 'tau_a', 2, 'a_d', 0.1, ...
              't_scan', tS, 'du_d', 0.011, 'ap_q', true, 'q_theta_from', tS, 'sP_fix_theta_deg', 2, 'hemi_r', 15, ...
              'bend_alpha', 20, 'bend_phi', 45, 'bend_Lx', 20, 'bend_xb', 10);
hs = fzero(@(h) 1 / local_cp(h) - base.a_d, [1.001 30]);
if isempty(gcp('nocreate')); parpool('Processes', min(6, feature('numcores'))); end
jobs = cell(1, 2 * n);
parfor q = 1:2 * n
    iw = 1 + floor((q - 1) / n);  i = q - (iw - 1) * n;  o = base;  o.wall = walls{iw};  o.seeds = seeds(i);  o.tag = 'mg';
    if iw == 2; o.scan_dir = [1 1];  o.scan_len = pi / 2 * (o.hemi_r + hs * Rr);  o.T = tS + 1; end
    R = run_t3_xyz_fixed(o);  jobs{q} = R.runs{1};
end
for iw = 1:2
    rs = jobs((iw - 1) * n + (1:n));  sc = NaN(1, n);
    for i = 1:n
        r = rs{i};  kk = find(r.t >= tS & (1:numel(r.t))' <= r.kend);  kk = local_until_edge(r, kk, walls{iw});
        sc(i) = mean(abs(r.thy(kk) - r.th_tr(kk, 2)) + abs(r.thx(kk) - r.th_tr(kk, 1)));
        fprintf('%-4s seed %d: contact %d | mean |angle error| during the scan %.2f deg | max |error| y %.1f x %.1f | true angle at the end (y, x) %.1f %.1f\n', ...
                walls{iw}, seeds(i), r.contact, sc(i), max(abs(r.thy(kk) - r.th_tr(kk, 2))), max(abs(r.thx(kk) - r.th_tr(kk, 1))), r.th_tr(kk(end), 2), r.th_tr(kk(end), 1));
    end
    [~, ib] = min(sc);  r = rs{ib};  kS0 = find(r.t >= tS, 1);
    kl = local_until_edge(r, (kS0:r.kend)', walls{iw});  k = (1:kl(end))';  t = r.t(k);  Tw = t(end);
    % motion error in the true local coordinate
    del = zeros(numel(k), 3);
    for j = 1:numel(k)
        Vt = local_Ry(r.th_tr(j, 2) * pi / 180) * local_Rx(r.th_tr(j, 1) * pi / 180);  del(j, :) = (Vt' * (r.pd(j, :) - r.p(j, :))')';
    end
    f = figure('Color', 'w', 'Position', [30 30 2300 1500]);  tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    % side view along the scan direction
    ax = nexttile(tl, 1);  hold on;  box on;
    if iw == 1; sdir = [1; 0]; else; sdir = [1; 1] / sqrt(2); end
    sp = r.p(k, 1:2) * sdir;  sg = linspace(min(sp) - 10, max(sp) + 10, 600);  if iw == 2; sg = linspace(-25, 30, 600); end
    zg = zeros(size(sg));
    for j = 1:numel(sg); zg(j) = local_surf_z(walls{iw}, sg(j) * sdir, base); end
    fill([sg fliplr(sg)], [zg, min(zg) - 5 + 0 * sg], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(sg, zg, 'Color', cGr, 'LineWidth', 3, 'DisplayName', 'wall');  plot(sp, r.p(k, 3), 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'p');
    kH = find(t >= tH, 1);  plot(sp(kH), r.p(kH, 3), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 'end of hold');
    plot(sp(kS0), r.p(kS0, 3), 'kd', 'MarkerFaceColor', 'k', 'MarkerSize', 9, 'DisplayName', 't_{scan}');
    axis equal;  if iw == 2; xlim([-25 30]);  ylim([-3 33]); end
    xlabel(sprintf('distance along the scan direction  [%sm]', mu));  ylabel(sprintf('z  [%sm]', mu));
    legend('Location', 'northoutside', 'Orientation', 'horizontal');  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.4);  set(ax.Toolbar, 'Visible', 'off');
    % gains
    axg = nexttile(tl, 3);  hold on;  box on;
    h1 = plot(t, r.aN_tr(k), 'Color', cT, 'LineWidth', 2.4);  plot(t, r.aT_tr(k), 'Color', cT, 'LineWidth', 2.4);
    h2 = plot(t, r.aN(k), 'Color', cE, 'LineWidth', 1.5);  h3 = plot(t, r.aT(k), 'Color', cG, 'LineWidth', 1.5);  h4 = yline(base.a_d, 'k--', 'LineWidth', 1.8);
    ylim([0 1.1]);  ylabel('gain');
    legend([h1 h2 h3 h4], {sprintf('%s_{33}, %s_{11}', AB, AB), sprintf('%s%s_{33}', AB, hatc), sprintf('%s%s_{11}', AB, hatc), sprintf('%s_d = %g', AB, base.a_d)}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal');
    % angles
    axa = nexttile(tl, 5);  hold on;  box on;
    h1 = plot(t, r.th_tr(k, 2), 'Color', cT, 'LineWidth', 2.4);  plot(t, r.th_tr(k, 1), 'Color', cT, 'LineWidth', 2.4);
    h2 = plot(t, r.thy(k), 'Color', cE, 'LineWidth', 1.5);  h3 = plot(t, r.thx(k), 'Color', cG, 'LineWidth', 1.5);
    ylabel(sprintf('%s  [\\circ]', th));  xlabel('Time (sec)');
    yl = [min([r.th_tr(k, :); r.thx(k) r.thy(k)], [], 'all'), max([r.th_tr(k, :); r.thx(k) r.thy(k)], [], 'all')];  ylim(yl + [-5 5]);
    legend([h1 h2 h3], {sprintf('%s_y,  %s_x', th, th), sprintf('%s%s_y', th, hatc), sprintf('%s%s_x', th, hatc)}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    % motion error
    axd = gobjects(1, 3);  nmv = {'u', 'v', 'w'};
    for row = 1:3
        axd(row) = nexttile(tl, 2 * row);  hold on;  box on;  plot(t, del(:, row), 'Color', cE, 'LineWidth', 0.6);
        ylabel(sprintf('%s%s  [%sm]', DL, nmv{row}, mu));  ylim([-0.3 0.3]);  if row == 3; xlabel('Time (sec)'); end
    end
    for ax = [axg axa axd]
        xline(ax, tH, '--', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');  xline(ax, tS, '-.', 'Color', cGr, 'LineWidth', 1.3, 'HandleVisibility', 'off');
        set(ax, 'XLim', [0 Tw], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.4);  set(ax.Toolbar, 'Visible', 'off');
    end
    exportgraphics(f, fullfile(fo, sprintf('fig_1028_%s_merged_s%d.png', walls{iw}, seeds(ib))), 'Resolution', 150);  close(f);
    ks = t >= tS;  fprintf('%s: figure seed %d, drawn to t = %.2f s | rms delta u v w before the scan %s, during %s um\n', walls{iw}, seeds(ib), Tw, ...
                          mat2str(round(sqrt(mean(del(t >= tH & ~ks, :).^2)), 3)), mat2str(round(sqrt(mean(del(ks, :).^2)), 3)));
    system(['open ' fullfile(fo, sprintf('fig_1028_%s_merged_s%d.png', walls{iw}, seeds(ib)))]);
end

function kk = local_until_edge(r, kk, wall)                    % hemisphere: steps before the nearest surface becomes the plane
    if strcmp(wall, 'hemi')
        e = find(abs(r.th_tr(kk, 1)) < 1e-9 & abs(r.th_tr(kk, 2)) < 1e-9 & (1:numel(kk))' > 10, 1);
        if ~isempty(e); kk = kk(1:e - 1); end
    end
end
function z = local_surf_z(wall, xy, o)                          % wall height under (x, y), plane z = 0
    switch wall
        case 'hemi'
            rr = norm(xy);  z = sqrt(max(o.hemi_r^2 - rr^2, 0));
        case 'bend'
            m = [cosd(o.bend_phi); sind(o.bend_phi)];  ae = o.bend_alpha * pi / 180;  Lxi = o.bend_Lx * m(1);  xi = m' * xy - o.bend_xb * m(1);
            if xi <= 0; z = 0; elseif xi <= Lxi; z = (Lxi / ae) * log(cos(ae * xi / Lxi)); else; z = (Lxi / ae) * log(cos(ae)) - tan(ae) * (xi - Lxi); end
    end
end
function c = local_cp(h)
    [~, c] = calc_correction_functions(h);
end
function M = local_Rx(a)
    M = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end
function M = local_Ry(a)
    M = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end
