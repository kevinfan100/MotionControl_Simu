% STATUS: SCRATCH -- 2026-10-20 (user: two-segment wall, regulate on the flat part, then move along u-hat onto the tilted part; "牆壁的
%   設定先用你的設定方式 也畫圖我看"). Side view (x-z, TRUE scale, unit R) of the wall: flat (theta_y = 0) for x < 0, tilted for x > 0.
%   (a) drops away (theta_y = +ty), (b) rises toward the probe (theta_y = -ty); default ty = 5 deg (small angle first).
%   Procedure (Meng diss. Sec. 6.2.1, non-contact scanning by gain regulation): (1) approach along z from 22.2 R until a-hat_N reaches
%   a_d (here a_d = 0.3, h* ~ 1.5 R on the plane), (2) hold, (3) Delta u_d per control step along u-hat. Dashed = keeps the normal gap h*;
%   dotted = straight along x. -> test_results/temp_3d_wsonly/fig_wall_two_segment.png | run: plot_wall_two_segment_setup(5)
function plot_wall_two_segment_setup(ty)
    if nargin < 1; ty = 5; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    FS = 20;  cW = [0.45 0.45 0.45];  cE = [0 0.25 0.85];  cT = [0.85 0 0];  cG = [0 0.55 0.2];  TH = char(952);  DL = char(916);  HT = char(770);
    hs = 1.5;  x0 = -2;  ztop = 4;  xr = 4;  TY = [ty -ty];  TT = {'(a) wall drops away', '(b) wall rises toward the probe'};
    f = figure('Position', [40 40 2200 800], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        ax = nexttile(tl);  hold(ax, 'on');  axis(ax, 'equal');  box(ax, 'on');
        t = TY(ic);  s = -tand(t);
        xw = [-3.2 0 xr + 0.2];  zw = [0 0 s * (xr + 0.2)];
        fill(ax, [xw xr + 0.2 -3.2], [zw -1 -1], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
        h1 = plot(ax, xw, zw, '-', 'Color', cW, 'LineWidth', 6, 'DisplayName', 'wall');
        h2 = plot(ax, [x0 x0], [ztop hs], '-', 'Color', cE, 'LineWidth', 3, 'DisplayName', '(1) approach from 22.2 R');
        plot(ax, x0, hs, 'o', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 12, 'HandleVisibility', 'off');
        xi = linspace(x0, xr, 300);  zi = hs + (xi > 0) .* (s * xi) + (xi > 0) * hs * (1 / cosd(t) - 1);
        h3 = plot(ax, xi, zi, '--', 'Color', cG, 'LineWidth', 3, 'DisplayName', 'keeps the gap h*');
        h4 = plot(ax, xi, hs + 0 * xi, ':', 'Color', cT, 'LineWidth', 3, 'DisplayName', 'straight along x');
        quiver(ax, x0 + 0.2, hs + 0.45, 1.0, 0, 0, 'Color', 'k', 'LineWidth', 2, 'MaxHeadSize', 0.6, 'HandleVisibility', 'off');
        text(ax, x0 + 0.2, hs + 0.8, ['(3) ' DL 'u_d per step along u' HT], 'FontSize', FS - 4, 'FontWeight', 'bold');
        text(ax, x0 - 1.0, hs + 0.35, '(2) hold', 'FontSize', FS - 4, 'FontWeight', 'bold', 'Color', cE);
        plot(ax, [x0 - 0.2 x0 - 0.2], [0 hs], 'k-', 'LineWidth', 1.2, 'HandleVisibility', 'off');
        text(ax, x0 - 0.65, 0.75, 'h*', 'FontSize', FS - 4, 'FontWeight', 'bold');
        text(ax, -3.0, -0.4, [TH '_y = 0'], 'FontSize', FS - 4, 'FontWeight', 'bold', 'Color', [0.3 0.3 0.3]);
        text(ax, 2.0, s * 2.0 - 0.45, [TH '_y = ' num2str(t) char(176)], 'FontSize', FS - 4, 'FontWeight', 'bold', 'Color', [0.3 0.3 0.3]);
        plot(ax, [0 0], [-0.15 0.15], 'k-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        text(ax, -0.45, -0.4, 'junction', 'FontSize', FS - 6, 'FontWeight', 'bold');
        text(ax, -0.5, ztop - 0.2, TT{ic}, 'FontSize', FS - 2, 'FontWeight', 'bold');
        xlim(ax, [-3.2 xr + 0.2]);  ylim(ax, [-0.8 ztop + 0.2]);  xlabel(ax, 'x  [R]', 'FontSize', FS - 2, 'FontWeight', 'bold');
        if ic == 1; ylabel(ax, 'z  [R]', 'FontSize', FS - 2, 'FontWeight', 'bold'); end
        set(ax, 'FontSize', FS - 4, 'FontWeight', 'bold', 'LineWidth', 1.5);
        if ic == 1
            legend(ax, [h1 h2 h3 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 6, 'FontWeight', 'bold');
        end
    end
    out = fullfile(od, 'fig_wall_two_segment.png');
    exportgraphics(f, out, 'Resolution', 140);  close(f);  fprintf('figure: %s\n', out);
end
