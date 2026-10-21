% STATUS: SCRATCH -- 2026-10-20 (user: "我想看我們地板和軌跡"). Wall and path of run_t2_3d_reg, seed 1, columns G1 | G2.
%   Row 1: side view x-z [R] (theta_x = 0): wall (grey, through the origin, tilted theta_y), the line at the target gap h* (grey dashed),
%   desired path p_d (red), true path p (blue dashed); markers: start, start of the scan (t_scan), end.
%   Row 2: true gap h [R] against the distance travelled along the TRUE u during the scan (red = h*): a flat line = the scan follows the wall.
%   -> test_results/temp_3d_wsonly/fig_t2_3d_traj_s<seed>.png | run: plot_t2_3d_traj(seed)
function plot_t2_3d_traj(seed)
    if nargin < 1; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    addpath(genpath(fullfile(root, 'model')));  pc = physical_constants();  Rr = pc.R;
    TG = {'G1', 'G2'};  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cW = [0.45 0.45 0.45];  FS = 20;  LFS = 15;  AB = char(257);
    f = figure('Position', [40 40 2000 1300], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        S = load(fullfile(od, ['temp_t2_' TG{ic} '.mat']));  runs = [S.R.runs{:}];  r = runs([runs.seed] == seed);
        k = 1:r.kend;  p = r.p(k, :) / Rr;  pd = r.pd(k, :) / Rr;  ty = S.R.opts.theta_y;
        ax = nexttile(tl, ic);  hold on;  box on;  axis equal;
        xw = [min([p(:, 1); pd(:, 1)]) - 3, max([p(:, 1); pd(:, 1)]) + 3];
        zw = -tand(ty) * xw;
        fill([xw fliplr(xw)], [zw, -40 -40], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
        h1 = plot(xw, zw, '-', 'Color', cW, 'LineWidth', 5, 'DisplayName', ['wall, ' char(952) '_y = ' num2str(ty) char(176)]);
        h2 = plot(xw - r.h_star * sind(ty) * 0 + r.h_star * sind(ty), zw + r.h_star * cosd(ty), '--', 'Color', cW, 'LineWidth', 2, 'DisplayName', 'gap h*');
        h3 = plot(pd(:, 1), pd(:, 3), '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', 'p_d');
        h4 = plot(p(:, 1), p(:, 3), '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'p');
        ks = find(r.t >= S.R.opts.t_scan, 1);
        plot(p(1, 1), p(1, 3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'HandleVisibility', 'off');
        plot(p(ks, 1), p(ks, 3), 's', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 11, 'HandleVisibility', 'off');
        plot(p(end, 1), p(end, 3), '^', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 11, 'HandleVisibility', 'off');
        text(p(1, 1) + 1, p(1, 3), 'start', 'FontSize', FS - 4, 'FontWeight', 'bold');
        text(p(ks, 1) + 1, p(ks, 3) + 2, 'scan starts', 'FontSize', FS - 4, 'FontWeight', 'bold', 'Color', cE);
        text(p(end, 1) - 6, p(end, 3) + 3, 'end', 'FontSize', FS - 4, 'FontWeight', 'bold', 'Color', cE);
        xlim(xw);  ylim([min(zw) - 2, max(p(:, 3)) + 3]);  xlabel('x  [R]', 'FontSize', FS, 'FontWeight', 'bold');
        if ic == 1; ylabel('z  [R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        text(0.97, 0.92, sprintf('%s:  %s_d = %.1f', TG{ic}, AB, S.R.opts.a_d), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h1 h2 h3 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(gca, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;
        ax = nexttile(tl, 2 + ic);  hold on;  box on;
        sc = ks:r.kend;
        g1 = plot(r.u_prog(sc), r.h_star + 0 * sc, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', 'h*');
        g2 = plot(r.u_prog(sc), r.h(sc), '-', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', 'h');
        xlabel('distance along the wall during the scan  [R]', 'FontSize', FS - 2, 'FontWeight', 'bold');
        if ic == 1; ylabel('h  [R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        ylim([1 max(2, r.h_star + 0.6)]);  legend([g1 g2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(gca, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;
    end
    out = fullfile(od, sprintf('fig_t2_3d_traj_s%d.png', seed));
    exportgraphics(f, out, 'Resolution', 120);  close(f);  fprintf('figure: %s\n', out);
end
