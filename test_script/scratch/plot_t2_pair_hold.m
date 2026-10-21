% STATUS: SCRATCH -- 2026-10-21 (user: "15 µm 停 15 s 後 regulation 有 u、v 控制 | 沒有 u、v 控制 對比圖"). Two arms side by side, same axes:
%   row 1 = x-z side view (measurement coordinate, it draws the wall) of every seed: wall, true normal through the start (light red
%   dashed), h* R (grey dashed), probe path p (seed 1 bold, others light); row 2 / 3 = u, v of the probe relative to the start in the
%   TRUE local coordinate [um] against time from the end of the hold. 20 deg plane.
%   -> test_results/temp_3d_wsonly/fig_t2_pair_<outtag>.png
%   | run: plot_t2_pair_hold({'h15hold','h15holdnouv'}, {'with u, v control','without u, v control'}, 1:3, 15, 'hold15_uv')
function plot_t2_pair_hold(PRE, LB, SD, t0, outtag)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');  addpath(genpath(fullfile(root, 'model')));  pc = physical_constants();  Rr = pc.R;
    cW = [0.45 0.45 0.45];  cB = {[0 0.25 0.85], [0 0.55 0.2]};  cL = {[0.6 0.7 0.95], [0.6 0.85 0.65]};  FS = 18;  LFS = 13;  MU = char(181);
    nc = numel(PRE);
    f = figure('Position', [40 40 900 * nc 1500], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(3, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        ax = nexttile(tl, ic);  hold on;  box on;
        a2 = nexttile(tl, nc + ic);  hold on;  box on;  a3 = nexttile(tl, 2 * nc + ic);  hold on;  box on;
        for s = SD
            S = load(fullfile(od, sprintf('temp_t2_%s_s%d.mat', PRE{ic}, s)));  r = S.R.runs{1};  k = 1:r.kend;  p = r.p(k, :);
            ty = S.R.opts.theta_y;  uu = [cosd(ty) 0 -sind(ty)];  ww = [sind(ty) 0 cosd(ty)];  V = [uu' [0; 1; 0] ww'];
            if s == SD(1)
                w0 = median(p * ww' - r.h(k) * Rr);  ft = p(1, :) - r.h(1) * Rr * ww;
                xl = ft(1) + [-4 8];  xw = linspace(xl(1), xl(2), 2);  zw = (w0 - xw * ww(1)) / ww(3);
                fill(ax, [xw fliplr(xw)], [zw zw - 50], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
                h1 = plot(ax, xw, zw, '-', 'Color', cW, 'LineWidth', 5, 'DisplayName', ['wall, ' char(952) '_y = ' num2str(ty) char(176)]);
                h2 = plot(ax, xw + r.h_star * Rr * ww(1), zw + r.h_star * Rr * ww(3), '--', 'Color', cW, 'LineWidth', 2, 'DisplayName', 'h* R');
                h3 = plot(ax, [p(1, 1) ft(1)], [p(1, 3) ft(3)], '--', 'Color', [0.95 0.65 0.65], 'LineWidth', 2.5, 'DisplayName', 'normal through the start');
            end
            m = find(r.t(k) >= t0);  t = r.t(k(m)) - t0;  UV = (V(:, 1:2)' * (p(m, :) - p(1, :))')';
            if s == SD(1)
                h4 = plot(ax, p(:, 1), p(:, 3), '-', 'Color', cB{1}, 'LineWidth', 2, 'DisplayName', 'p, seed 1');
                g1 = plot(a2, t, UV(:, 1), '-', 'Color', cB{1}, 'LineWidth', 1.8, 'DisplayName', 'u, seed 1');
                g2 = plot(a3, t, UV(:, 2), '-', 'Color', cB{2}, 'LineWidth', 1.8, 'DisplayName', 'v, seed 1');
            else
                plot(ax, p(:, 1), p(:, 3), '-', 'Color', cL{1}, 'LineWidth', 1, 'HandleVisibility', 'off');
                plot(a2, t, UV(:, 1), '-', 'Color', cL{1}, 'LineWidth', 1, 'HandleVisibility', 'off');
                plot(a3, t, UV(:, 2), '-', 'Color', cL{2}, 'LineWidth', 1, 'HandleVisibility', 'off');
            end
        end
        axis(ax, 'equal');  xlim(ax, xl);  ylim(ax, [min(zw) - 1, max(zw) + 16]);
        text(ax, 0.97, 0.94, LB{ic}, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
        legend(ax, [h1 h2 h3 h4], 'Location', 'northoutside', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold');
        xlabel(ax, ['x  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax, ['z  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        ylim(a2, [-2 2]);  ylim(a3, [-2 2]);  xlim(a2, [0 t(end)]);  xlim(a3, [0 t(end)]);
        ylabel(a2, ['u  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a3, ['v  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        xlabel(a3, sprintf('time from the end of the hold (t - %g s)  [s]', t0), 'FontSize', FS - 2, 'FontWeight', 'bold');
        legend(a2, g1, 'Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold');  legend(a3, g2, 'Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold');
        set([ax a2 a3], 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);
    end
    out = fullfile(od, sprintf('fig_t2_pair_%s.png', outtag));
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
