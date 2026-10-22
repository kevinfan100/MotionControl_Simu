% STATUS: SCRATCH -- 2026-10-22 (user: "先用 H=3 sigma=7 畫凸起示意圖"). Cover glass (z = 0) with a Gaussian bump z_w = H exp(-r^2 / 2 sigma^2)
%   centred at (xc, 0); the scan line y = y0 (offset so theta_x and theta_y both change); start above the flat glass at x_start.
%   Panels: 3-D view with the scan line at the gap h* R | top view (height contours, scan line, start) | side view along the scan line |
%   wall angles theta_x, theta_y of the local tangent plane along the scan line (V = R_y(theta_y) R_x(theta_x), math-model.md). Unit um.
%   -> test_results/temp_3d_wsonly/fig_wall_bump_setup.png | run: plot_wall_bump_setup(3, 7)
function plot_wall_bump_setup(H, sg)
    if nargin < 1; H = 3; end
    if nargin < 2; sg = 7; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    xc = 25;  y0 = sg / 2;  x_start = 0;  x_end = 50;  gap = 2.5;   % bump centre, scan-line offset, scan range [um], gap ~ h* R
    FS = 18;  LFS = 14;  cW = [0.45 0.45 0.45];  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cG = [0 0.55 0.2];  TH = char(952);  DG = char(176);  MU = char(181);
    zf = @(x, y) H * exp(-((x - xc).^2 + y.^2) / (2 * sg^2));
    [X, Y] = meshgrid(linspace(-5, 55, 241), linspace(-25, 25, 201));  Z = zf(X, Y);
    xs = linspace(x_start, x_end, 600);  ys = y0 + 0 * xs;  zs = zf(xs, ys);
    gx = -(xs - xc) / sg^2 .* zs;  gy = -(ys) / sg^2 .* zs;      % dz/dx, dz/dy
    nrm = [-gx; -gy; ones(size(xs))];  nrm = nrm ./ vecnorm(nrm);
    thx = -asind(nrm(2, :));  thy = atan2d(nrm(1, :), nrm(3, :));
    f = figure('Position', [40 40 2200 1500], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl, 1);  hold on;  box on;
    surf(X, Y, Z, 'EdgeColor', 'none', 'FaceColor', [0.85 0.85 0.85], 'FaceLighting', 'gouraud', 'HandleVisibility', 'off');  light('Position', [-30 -60 80]);
    zt = zs + gap * nrm(3, :);  xt = xs + gap * nrm(1, :);  yt = ys + gap * nrm(2, :);
    h1 = plot3(xt, yt, zt, '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', 'scan line at the gap h* R');
    plot3([x_start x_start], [y0 y0], [gap 15], '--', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
    plot3(x_start, y0, 15, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'HandleVisibility', 'off');
    view(-35, 30);  axis equal;  xlim([-5 55]);  ylim([-25 25]);  zlim([0 16]);
    xlabel(['x  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(['y  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  zlabel(['z  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
    legend(h1, 'Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;
    ax = nexttile(tl, 2);  hold on;  box on;
    contourf(X, Y, Z, 12, 'LineStyle', 'none', 'HandleVisibility', 'off');  colormap(ax, gray(64) * 0.6 + 0.35);
    cb = colorbar;  cb.Label.String = ['z_w  [' MU 'm]'];  cb.Label.FontSize = FS - 2;  cb.Label.FontWeight = 'bold';
    contour(X, Y, Z, [H / 2 H / 2], '--', 'Color', [0 0 0], 'LineWidth', 1.5, 'HandleVisibility', 'off');
    h1 = plot(xs, ys, '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', sprintf('scan line y = %.1f %sm', y0, MU));
    plot(x_start, y0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'HandleVisibility', 'off');
    text(x_start + 1, y0 + 3, 'start', 'FontSize', FS - 4, 'FontWeight', 'bold');
    text(xc + sg * 1.25, -sg * 1.25, 'half height', 'FontSize', FS - 4, 'FontWeight', 'bold');
    axis equal;  xlim([-5 55]);  ylim([-25 25]);
    xlabel(['x  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(['y  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
    legend(h1, 'Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;
    ax = nexttile(tl, 3);  hold on;  box on;
    fill([xs fliplr(xs)], [zs, -3 + 0 * xs], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    h1 = plot(xs, zs, '-', 'Color', cW, 'LineWidth', 4, 'DisplayName', sprintf('wall at y = %.1f %sm', y0, MU));
    h2 = plot(xt, zt, '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', 'gap h* R');
    plot([x_start x_start], [gap 15], '--', 'Color', cT, 'LineWidth', 2.5, 'HandleVisibility', 'off');
    plot(x_start, 15, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'HandleVisibility', 'off');
    text(x_start + 1, 15, sprintf('start (15 %sm gap)', MU), 'FontSize', FS - 4, 'FontWeight', 'bold');
    axis equal;  xlim([-5 55]);  ylim([-3 17]);
    xlabel(['x  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(['z  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;
    ax = nexttile(tl, 4);  hold on;  box on;
    h1 = plot(xs, thy, '-', 'Color', cE, 'LineWidth', 3, 'DisplayName', [TH '_y']);
    h2 = plot(xs, thx, '-', 'Color', cG, 'LineWidth', 3, 'DisplayName', [TH '_x']);
    plot([x_start x_end], [0 0], ':', 'Color', cW, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    xlim([-5 55]);  ylim([-20 20]);
    xlabel(['x  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel([TH '  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;
    tilt = acosd(nrm(3, :));
    fprintf('H %g um, sigma %g um: FWHM %.1f um, max slope on the full profile %.1f deg; on the scan line max tilt %.1f deg, theta_y %.1f..%.1f, theta_x %.1f..%.1f; tip curvature radius %.1f um\n', ...
            H, sg, 2.355 * sg, atand(H / sg * exp(-0.5)), max(tilt), min(thy), max(thy), min(thx), max(thx), sg^2 / H);
    out = fullfile(od, 'fig_wall_bump_setup.png');
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
