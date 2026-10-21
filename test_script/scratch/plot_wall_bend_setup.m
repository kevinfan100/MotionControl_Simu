% STATUS: SCRATCH -- 2026-10-20 (user: "先針對第一步你先畫牆壁給我看 且都 plane wall"). Stage-1 wall for the changing-angle test:
%   flat (z = 0) -> smooth bend -> tilted plane, the bend line oblique to the scan (azimuth phi of the downslope direction n from x), so
%   theta_x and theta_y both change. Height field z_w(x, y) = -int_0^xi tan alpha(s) ds, xi = n'(x, y), alpha ramps 0 -> alpha_end over
%   the bend; the wall drops away from the probe. Wall type = plane everywhere (plane gain law, local tangent plane).
%   Normal w = (sin a cos phi, sin a sin phi, cos a) -> theta_x = -asin(sin a sin phi), theta_y = atan2(sin a cos phi, cos a)
%   (math-model.md). Panels (user 10-20: top view removed): 3-D view (top row) | side view along the scan line |
%   theta_x, theta_y along the scan line. Unit um. -> test_results/temp_3d_wsonly/fig_wall_bend_setup.png
%   | run: plot_wall_bend_setup(20, 45, 20)
function plot_wall_bend_setup(alpha_end, phi, Lx)
    if nargin < 1; alpha_end = 20; end                         % total tilt [deg]
    if nargin < 2; phi = 45; end                               % azimuth of the downslope direction from +x [deg]
    if nargin < 3; Lx = 20; end                                % bend length measured along the scan line (x) [um]
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    FS = 20;  LFS = 15;  cW = [0.45 0.45 0.45];  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];
    TH = char(952);  DG = char(176);  MU = char(181);
    n = [cosd(phi) sind(phi)];  Lxi = Lx * n(1);               % bend length along n
    x_start = -10;  x_end = 110;  hs = 2.5;                     % approach above x = -10 um; gap ~ h* R [um]
    [X, Y] = meshgrid(linspace(-30, 120, 151), linspace(-40, 40, 81));
    Z = local_zw(X * n(1) + Y * n(2), alpha_end, Lxi);
    xs = linspace(x_start, x_end, 600);  xi_s = xs * n(1);
    zs = local_zw(xi_s, alpha_end, Lxi);  al = local_alpha(xi_s, alpha_end, Lxi);
    thx = -asind(sind(al) * sind(phi));  thy = atan2d(sind(al) * cosd(phi), cosd(al));
    f = figure('Position', [40 40 2200 1500], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    % (1) 3-D view
    ax = nexttile(tl, 1, [1 2]);  hold on;  box on;
    surf(X, Y, Z, 'EdgeColor', 'none', 'FaceColor', [0.85 0.85 0.85], 'FaceLighting', 'gouraud', 'HandleVisibility', 'off');
    light('Position', [-50 -80 150]);
    h1 = plot3(xs, 0 * xs, zs + hs, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', 'scan line (gap h* R above the wall)');
    plot3([x_start x_start], [0 0], [hs 50], '--', 'Color', cT, 'LineWidth', 3, 'HandleVisibility', 'off');
    plot3(x_start, 0, 50, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'HandleVisibility', 'off');
    for xa = [-5 60]                                            % wall normals on the scan line
        a = local_alpha(xa * n(1), alpha_end, Lxi);  w = [sind(a) * cosd(phi), sind(a) * sind(phi), cosd(a)];
        z0 = local_zw(xa * n(1), alpha_end, Lxi);
        quiver3(xa, 0, z0, 15 * w(1), 15 * w(2), 15 * w(3), 0, 'k', 'LineWidth', 3, 'MaxHeadSize', 0.8, 'HandleVisibility', 'off');
        text(xa + 15 * w(1), 15 * w(2), z0 + 15 * w(3) + 3, 'w', 'FontSize', FS, 'FontWeight', 'bold');
    end
    view(-15, 18);  axis equal;  xlim([-30 120]);  ylim([-40 40]);
    xlabel(['x  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(['y  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
    zlabel(['z  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
    legend(h1, 'Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;
    % (2) side view along the scan line (y = 0)
    ax = nexttile(tl, 3);  hold on;  box on;
    xf = linspace(-30, 120, 600);  zf = local_zw(xf * n(1), alpha_end, Lxi);
    fill([xf fliplr(xf)], [zf, -60 + 0 * xf], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(xf, zf, '-', 'Color', cW, 'LineWidth', 5, 'HandleVisibility', 'off');
    h1 = plot(xs, zs, '-', 'Color', cW, 'LineWidth', 5, 'DisplayName', 'wall at y = 0');
    h2 = plot(xs, zs + hs ./ cosd(al), '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', 'gap h* R');
    plot([x_start x_start], [hs 50], '--', 'Color', cT, 'LineWidth', 3, 'HandleVisibility', 'off');
    plot(x_start, 50, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'HandleVisibility', 'off');
    text(x_start + 3, 50, 'start', 'FontSize', FS - 4, 'FontWeight', 'bold');
    xline([0 Lx], ':', 'Color', cW, 'LineWidth', 2, 'HandleVisibility', 'off');
    axis equal;  xlim([-30 120]);  ylim([-35 55]);
    xlabel(['x  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(['z  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;
    % (3) angles along the scan line
    ax = nexttile(tl, 4);  hold on;  box on;
    h1 = plot(xs, thy, '-', 'Color', cE, 'LineWidth', 3, 'DisplayName', [TH '_y']);
    h2 = plot(xs, thx, '-', 'Color', cG, 'LineWidth', 3, 'DisplayName', [TH '_x']);
    xline([0 Lx], ':', 'Color', cW, 'LineWidth', 2, 'HandleVisibility', 'off');
    xlim([-30 120]);  ylim([-20 20]);
    xlabel(['x  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel([TH '  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;
    fprintf('end values: theta_y %.2f deg, theta_x %.2f deg; bend %g um along x (%.1f um along n); start x = %g um\n', ...
            thy(end), thx(end), Lx, Lxi, x_start);
    out = fullfile(od, 'fig_wall_bend_setup.png');
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end

function a = local_alpha(xi, alpha_end, Lxi)                   % tilt [deg]: 0 | linear ramp | alpha_end
    a = alpha_end * min(max(xi / Lxi, 0), 1);
end

function z = local_zw(xi, alpha_end, Lxi)                      % z_w = -int_0^xi tan(alpha(s)) ds (numerical, exact enough)
    s = linspace(0, Lxi, 400);  F = [0 cumsum(diff(s) .* tand(local_alpha((s(1:end-1) + s(2:end)) / 2, alpha_end, Lxi)))];
    z = zeros(size(xi));
    in = xi > 0 & xi <= Lxi;  z(in) = -interp1(s, F, xi(in));
    out = xi > Lxi;  z(out) = -(F(end) + tand(alpha_end) * (xi(out) - Lxi));
end
