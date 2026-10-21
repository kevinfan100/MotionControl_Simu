% STATUS: SCRATCH -- 2026-10-20 (user: "先用 2 度的 我要看完整的模擬結果"). Full-run figures of ONE bend-wall run of run_t2_3d_reg
%   (opts.wall = 'bend'), same layouts as the single-plane T2 figures, the truth now changing along the path:
%   (1) fig_t2_bend_xyz_<tag>: measurement coordinate x-z side view [um], wall profile in the x-z cut at the scan line's y (grey), p_d red, p blue dashed, markers;
%       true u, w drawn on the flat part and on the tilted part (w projected on x-z); row 2 = zoom on the bend with the gap h* R (dashed).
%   (2) fig_t2_bend_est_<tag>: a_N (+ a_d) | a_T | theta_y, theta_x (true red, changing; +- sqrt P band), whole run.
%   (3) fig_t2_bend_motion_<tag>: TRUE local coordinate of each step (u, v, w from the logged true angles), Meng diss. Fig. 45 layout:
%       left u, v = distance travelled along u, v, w = h R (log axis, dotted h* R), desired = actual + delta; right delta = V' (p_d - p).
%   Rule (user 10-20): simulation data figures in the local coordinate; the measurement coordinate only in figures that draw the wall.
%   | run: plot_t2_bend_full('bend_pf2')
function plot_t2_bend_full(tag)
    if nargin < 1; tag = 'bend_pf2'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');  addpath(genpath(fullfile(root, 'model')));
    S = load(fullfile(od, ['temp_t2_' tag '.mat']));  o = S.R.opts;  r = S.R.runs{1};  pc = physical_constants();  Rr = pc.R;
    k = 1:r.kend;  t = r.t(k);  p = r.p(k, :);  pd = r.pd(k, :);  ks = find(t >= o.t_scan, 1);
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cW = [0.45 0.45 0.45];  cGr = [0 0.55 0.2];  cB = [0.75 0.85 1];  FS = 20;  LFS = 15;
    MU = char(181);  TH = char(952);  DG = char(176);  AB = char(257);  HT = char(770);  DL = char(948);
    n = [cosd(o.bend_phi) sind(o.bend_phi)];  ae = o.bend_alpha * pi / 180;  Lxi = o.bend_Lx * n(1);
    zw0 = median(p(1:ks, 3) - r.h(1:ks) * Rr);                 % flat-part wall height [um] (from the log, flat => w = z)
    ys = median(p(ks:end, 2));                                  % y of the scan line [um]: the wall is drawn in this x-z cut
    % ---------------- (1) side view ----------------
    f = figure('Position', [40 40 1800 1500], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for row = 1:2
        ax = nexttile(tl, row);  hold on;  box on;
        if row == 1; xl = [min(p(:, 1)) - 10, max(p(:, 1)) + 10]; else; xl = o.bend_xb + [-8, o.bend_Lx + 12]; end
        xw = linspace(xl(1), xl(2), 800);  [zw, al] = local_profile(xw, ys, n, ae, Lxi, o.bend_xb);  zw = zw + zw0;
        fill([xw fliplr(xw)], [zw, min(zw) - 50 + 0 * xw], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
        h1 = plot(xw, zw, '-', 'Color', cW, 'LineWidth', 5, 'DisplayName', sprintf('wall at y = %.1f %sm (scan line)', ys, MU));
        hs = [];
        if row == 2; hs = plot(xw, zw + r.h_star * Rr ./ cos(al), '--', 'Color', cW, 'LineWidth', 2, 'DisplayName', 'h* R'); end
        h2 = plot(pd(:, 1), pd(:, 3), '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', 'p_d');
        h3 = plot(p(:, 1), p(:, 3), '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'p');
        plot(p(ks, 1), p(ks, 3), 's', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 12, 'HandleVisibility', 'off');
        if row == 1
            plot(p(1, 1), p(1, 3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 11, 'HandleVisibility', 'off');
            plot(p(end, 1), p(end, 3), '^', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 12, 'HandleVisibility', 'off');
            text(p(1, 1) + 3, p(1, 3), 'start (t = 0)', 'FontSize', FS - 4, 'FontWeight', 'bold');
            text(p(ks, 1) + 9, p(ks, 3) + 14, sprintf('scan starts (t = %g s)', o.t_scan), 'FontSize', FS - 4, 'FontWeight', 'bold', 'Color', cE);
            text(p(end, 1) - 20, p(end, 3) + 10, sprintf('end (t = %g s)', t(end)), 'FontSize', FS - 4, 'FontWeight', 'bold', 'Color', cE);
            xa = [o.bend_xb + 3, o.bend_xb + o.bend_Lx + 40];  L0 = 12;
        else
            xa = [o.bend_xb - 4, o.bend_xb + o.bend_Lx + 5];  L0 = 3;
        end
        for q = 1:2                                            % true u, w on the flat part and on the tilted part
            [z0, a0] = local_profile(xa(q), ys, n, ae, Lxi, o.bend_xb);  z0 = z0 + zw0;
            thx = -asin(sin(a0) * n(2));  thy = atan2(sin(a0) * n(1), cos(a0));
            V = [cos(thy) 0 sin(thy); 0 1 0; -sin(thy) 0 cos(thy)] * [1 0 0; 0 cos(thx) -sin(thx); 0 sin(thx) cos(thx)];
            uu = V(:, 1);  ww = V(:, 3);
            quiver(xa(q), z0, L0 * uu(1), L0 * uu(3), 0, 'k', 'LineWidth', 3, 'MaxHeadSize', 0.6, 'HandleVisibility', 'off');
            quiver(xa(q), z0, L0 * ww(1), L0 * ww(3), 0, 'k', 'LineWidth', 3, 'MaxHeadSize', 0.6, 'HandleVisibility', 'off');
            text(xa(q) + 1.1 * L0 * uu(1), z0 + 1.1 * L0 * uu(3) - 0.15 * L0, 'u', 'FontSize', FS, 'FontWeight', 'bold');
            text(xa(q) + 1.1 * L0 * ww(1), z0 + 1.1 * L0 * ww(3), 'w', 'FontSize', FS, 'FontWeight', 'bold');
        end
        axis equal;  xlim(xl);
        if row == 1
            ylim([min(zw) - 5, max(p(:, 3)) + 8]);
            legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        else
            ylim([min(zw) - 3, max(zw) + 6]);
            legend([h1 hs h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        end
        xlabel(['x  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(['z  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;
    end
    local_save(f, od, sprintf('fig_t2_bend_xyz_%s.png', tag));
    % ---------------- (2) estimates ----------------
    f = figure('Position', [40 40 1100 1500], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl, 1);  hold on;  box on;
    h1 = plot(t, r.aN_true(k), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_N']);
    h2 = plot(t, r.aN(k), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HT '_N']);
    h3 = plot(t, o.a_d + 0 * t, '--', 'Color', [0 0 0], 'LineWidth', 2, 'DisplayName', sprintf('%s_d = %.1f', AB, o.a_d));
    ylim([0 1.1]);  ylabel([AB '_N'], 'FontSize', FS + 2, 'FontWeight', 'bold');
    legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  xlim([0 t(end)]);
    ax = nexttile(tl, 2);  hold on;  box on;
    h1 = plot(t, r.aT_true(k), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_T']);
    h2 = plot(t, r.aT(k), '-', 'Color', cGr, 'LineWidth', 1.8, 'DisplayName', [AB HT '_T']);
    ylim([0 1.1]);  ylabel([AB '_T'], 'FontSize', FS + 2, 'FontWeight', 'bold');
    legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  xlim([0 t(end)]);
    ax = nexttile(tl, 3);  hold on;  box on;
    fill([t; flipud(t)], [r.thy(k) + r.sP(k, 6); flipud(r.thy(k) - r.sP(k, 6))], cB, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    fill([t; flipud(t)], [r.thx(k) + r.sP(k, 5); flipud(r.thx(k) - r.sP(k, 5))], cB, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    h1 = plot(t, r.thy_true(k), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [TH '_y,  ' TH '_x']);
    plot(t, r.thx_true(k), '-', 'Color', cT, 'LineWidth', 4, 'HandleVisibility', 'off');
    h2 = plot(t, r.thy(k), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [TH HT '_y']);
    h3 = plot(t, r.thx(k), '-', 'Color', cGr, 'LineWidth', 1.8, 'DisplayName', [TH HT '_x']);
    ylim([-30 30]);  ylabel([TH '  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold');  xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  xlim([0 t(end)]);
    local_save(f, od, sprintf('fig_t2_bend_est_%s.png', tag));
    % ---------------- (3) motion (true local coordinate of each step; user 10-20: data figures in local, measurement only with the wall) ----
    % u, v = distance travelled along the true u, v of each step (sum of V_k' dp); w = h R (distance from the wall);
    % desired = actual + delta, delta = V_k' (p_d - p) (same frame as the right column)
    nk = numel(k);  E = zeros(nk, 3);  dU = zeros(nk, 2);  D = pd - p;
    for i = 1:nk
        thx = r.thx_true(i) * pi / 180;  thy = r.thy_true(i) * pi / 180;
        V = [cos(thy) 0 sin(thy); 0 1 0; -sin(thy) 0 cos(thy)] * [1 0 0; 0 cos(thx) -sin(thx); 0 sin(thx) cos(thx)];
        E(i, :) = (V' * D(i, :)')';
        if i > 1; dU(i, :) = (V(:, 1:2)' * (p(i, :) - p(i - 1, :))')'; end
    end
    L = [cumsum(dU), r.h(k) * Rr];  Ld = L + E;
    f = figure('Position', [40 40 2000 1500], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    NL = {'u', 'v', 'w'};
    for q = 1:3
        ax = nexttile(tl, 2 * q - 1);  hold on;  box on;
        h1 = plot(t, Ld(:, q), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [NL{q} '_d']);
        h2 = plot(t, L(:, q), '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', NL{q});
        if q == 3
            plot(t, r.h_star * Rr + 0 * t, ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 2, 'HandleVisibility', 'off');
            set(ax, 'YScale', 'log');
        end
        ylabel([NL{q} '  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        if q == 1; legend(ax, [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold'); end
        if q == 3; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;  xlim([0 t(end)]);
        ax = nexttile(tl, 2 * q);  hold on;  box on;
        plot(t, E(:, q), '-', 'Color', cE, 'LineWidth', 0.8);
        ylim([-0.3 0.3]);  ylabel([DL NL{q} '  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        if q == 3; xlabel('Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;  xlim([0 t(end)]);
    end
    local_save(f, od, sprintf('fig_t2_bend_motion_%s.png', tag));
end

function [z, a] = local_profile(x, y, n, ae, Lxi, xb)         % wall height and tilt at (x, y), as run_t2_3d_reg local_bend_wall
    xi = n(1) * x + n(2) * y - n(1) * xb;  a = ae * min(max(xi / Lxi, 0), 1);
    z = (Lxi / ae) * log(cos(a));
    out = xi > Lxi;  z(out) = (Lxi / ae) * log(cos(ae)) - tan(ae) * (xi(out) - Lxi);
end

function local_save(f, od, name)
    out = fullfile(od, name);  exportgraphics(f, out, 'Resolution', 120);  close(f);  fprintf('figure: %s\n', out);
end
