% STATUS: SCRATCH -- 2026-10-20 (user: "給我一個 measurement 座標中的圖有牆壁和我們的軌跡 我有點不能看懂 u"). Path of run_t2_3d_reg in the
%   MEASUREMENT coordinate (x, z; y = v points into the page since theta_x = 0), columns = seeds. Row 1: whole run, axis equal [um]:
%   wall (grey line, solid below), desired p_d (red), actual p (blue dashed), start / scan start / end markers, and the true local
%   directions u (along the wall, downslope) and w (wall normal) drawn on the wall 35 um into the scan. Row 2: zoom around the scan start so the gap
%   h* R (grey dashed) is visible. The wall offset along w is read from the log (w.p - h R), not assumed.
%   AX (optional, 10-21): common axes for side-by-side comparison, e.g. struct('top_x',[-20 100],'top_z',[-40 60],'top_tick',20,
%   'zoom_half',[8 6],'zoom_tick',2) -- top panel fixed limits and ticks, zoom panel fixed span centred on the landing point.
%   -> test_results/temp_3d_wsonly/fig_t2_xyz_<tag>.png | run: plot_t2_xyz('blk_both_c020', [1 4], {'', ' (best)'})
function plot_t2_xyz(tag, seeds, lbl, AX)
    if nargin < 1; tag = 'blk_both_c020'; end
    if nargin < 2; seeds = 1; end
    nc = numel(seeds);
    if nargin < 3 || isempty(lbl); lbl = repmat({''}, 1, nc); end
    if nargin < 4; AX = struct(); end                         % 10-21 user: common axes across figures (top_x, top_z, zoom_half, ticks)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');  addpath(genpath(fullfile(root, 'model')));
    S = load(fullfile(od, ['temp_t2_' tag '.mat']));  ty = S.R.opts.theta_y;  pc = physical_constants();  Rr = pc.R;
    uu = [cosd(ty) 0 -sind(ty)];  ww = [sind(ty) 0 cosd(ty)];          % true u, w (theta_x = 0)
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cW = [0.45 0.45 0.45];  FS = 20;  LFS = 15;  MU = char(181);  TH = char(952);  DG = char(176);
    sd = cellfun(@(q) q.seed, S.R.runs);
    f = figure('Position', [40 40 1000 * nc 1500], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:nc
        r = S.R.runs{find(sd == seeds(ic), 1)};  k = 1:r.kend;  p = r.p(k, :);  pd = r.pd(k, :);
        w0 = median(p * ww' - r.h(k) * Rr);                              % wall offset along w [um]
        ks = find(r.t(k) >= S.R.opts.t_scan, 1);  if isempty(ks); ks = r.kend; end
        has_scan = S.R.opts.T_scan > 0;               % regulation-only runs: no scan markers, arrows at the end point
        for row = 1:2
            ax = nexttile(tl, (row - 1) * nc + ic);  hold on;  box on;
            if row == 1
                xl = [min([p(:, 1); pd(:, 1)]) - 10, max([p(:, 1); pd(:, 1)]) + 10];  if isfield(AX, 'top_x'); xl = AX.top_x; end
            else
                xl = p(ks, 1) + [-8 8];  if isfield(AX, 'zoom_half'); xl = p(ks, 1) + AX.zoom_half(1) * [-1 1]; end
            end
            xw = linspace(xl(1), xl(2), 2);  zw = (w0 - xw * ww(1)) / ww(3);   % wall: w.p = w0
            zb = min(zw) - 50;
            fill([xw fliplr(xw)], [zw zb zb], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
            h1 = plot(xw, zw, '-', 'Color', cW, 'LineWidth', 5, 'DisplayName', ['wall, ' TH '_y = ' num2str(ty) DG]);
            hs = [];
            if row == 2
                hs = plot(xw + r.h_star * Rr * ww(1), zw + r.h_star * Rr * ww(3), '--', 'Color', cW, 'LineWidth', 2, 'DisplayName', 'h* R');
            end
            ft = p(1, :) - r.h(1) * Rr * ww;                     % 10-21: foot of the TRUE normal through the start (where the probe should land)
            hn = plot([p(1, 1) ft(1)], [p(1, 3) ft(3)], '--', 'Color', [0.95 0.65 0.65], 'LineWidth', 2.5, 'DisplayName', 'normal through the start');
            h2 = plot(pd(:, 1), pd(:, 3), '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', 'p_d');
            h3 = plot(p(:, 1), p(:, 3), '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'p');
            plot(p(ks, 1), p(ks, 3), 's', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 12, 'HandleVisibility', 'off');
            if row == 1
                plot(p(1, 1), p(1, 3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 11, 'HandleVisibility', 'off');
                plot(p(end, 1), p(end, 3), '^', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 12, 'HandleVisibility', 'off');
                text(p(1, 1) + 3, p(1, 3), sprintf('start (t = 0)'), 'FontSize', FS - 4, 'FontWeight', 'bold');
                if has_scan; text(p(ks, 1) + 6, p(ks, 3) + 7, sprintf('scan starts (t = %g s)', S.R.opts.t_scan), 'FontSize', FS - 4, 'FontWeight', 'bold', 'Color', cE); end
                text(p(end, 1) + 2, p(end, 3) + 2, sprintf('end (t = %g s)', r.t(r.kend)), 'FontSize', FS - 4, 'FontWeight', 'bold', 'Color', cE);
                L0 = 15;  q0 = p(ks, :) - r.h(ks) * Rr * ww + 35 * uu;     % arrows from a wall point 35 um down the scan
                if ~has_scan && ~isfield(AX, 'top_x'); L0 = 8;  q0 = p(ks, :) - r.h(ks) * Rr * ww + 10 * uu; end
            else
                L0 = 4;  q0 = p(ks, :) - r.h(ks) * Rr * ww - 4 * uu;
            end
            quiver(q0(1), q0(3), L0 * uu(1), L0 * uu(3), 0, 'k', 'LineWidth', 3, 'MaxHeadSize', 0.6, 'HandleVisibility', 'off');
            quiver(q0(1), q0(3), L0 * ww(1), L0 * ww(3), 0, 'k', 'LineWidth', 3, 'MaxHeadSize', 0.6, 'HandleVisibility', 'off');
            text(q0(1) + 1.1 * L0 * uu(1), q0(3) + 1.1 * L0 * uu(3) - 0.06 * L0, 'u', 'FontSize', FS, 'FontWeight', 'bold');
            text(q0(1) + 1.1 * L0 * ww(1), q0(3) + 1.1 * L0 * ww(3), 'w', 'FontSize', FS, 'FontWeight', 'bold');
            axis equal;  xlim(xl);
            if row == 1
                ylim([min(zw) - 5, max(p(:, 3)) + 8]);  if isfield(AX, 'top_z'); ylim(AX.top_z); end
                if isfield(AX, 'top_tick'); set(ax, 'XTick', AX.top_x(1):AX.top_tick:AX.top_x(2), 'YTick', AX.top_z(1):AX.top_tick:AX.top_z(2)); end
                legend([h1 hn h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            else
                zc = p(ks, 3);  ylim(zc + [-6 6]);  if isfield(AX, 'zoom_half'); ylim(zc + AX.zoom_half(2) * [-1 1]); end
                if isfield(AX, 'zoom_tick'); set(ax, 'XTick', AX.zoom_tick * (ceil(xl(1) / AX.zoom_tick):floor(xl(2) / AX.zoom_tick)), 'YTick', AX.zoom_tick * (ceil((zc - AX.zoom_half(2)) / AX.zoom_tick):floor((zc + AX.zoom_half(2)) / AX.zoom_tick))); end
                legend([h1 hs hn h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            end
            xlabel(['x  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
            if ic == 1; ylabel(['z  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold'); end
            set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;
        end
    end
    out = fullfile(od, sprintf('fig_t2_xyz_%s.png', tag));
    exportgraphics(f, out, 'Resolution', 120);  close(f);  fprintf('figure: %s\n', out);
end
