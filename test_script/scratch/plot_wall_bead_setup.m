% STATUS: SCRATCH -- 2026-10-22 (user: "玻璃珠先和 meng 一樣 ... 從 15 um 開始下降到平面 ... 再開始掃描"). Meng diss. Sec. 6.2.1 glass bead
%   (diameter 10 um) resting on the cover glass; probe starts 15 um above the glass away from the bead, comes down to the gap h* R on
%   the glass, then scans along +x through the bead centre (theta_x = 0). Wall model under discussion: nearest surface + plane law, so the
%   path at the gap is the offset of the union (glass, bead) by h* R and the normal is the one of the nearest surface.
%   Panels: side view (glass, bead, probe path at the gap, normals at a few points) | tilt of the normal from +z along the path.
%   -> test_results/temp_3d_wsonly/fig_wall_bead_setup.png | run: plot_wall_bead_setup(5, 2.51, 20)
function plot_wall_bead_setup(rb, gap, xc)
    if nargin < 1; rb = 5; end                                  % bead radius [um] (Meng: 10 um diameter)
    if nargin < 2; gap = 2.51; end                              % centre-to-surface gap h* R at a_d = 0.1 [um]
    if nargin < 3; xc = 20; end                                 % bead centre x [um]; start at x = 0
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    FS = 18;  LFS = 14;  cW = [0.45 0.45 0.45];  cT = [0.85 0 0];  cE = [0 0.25 0.85];  MU = char(181);  DG = char(176);  TH = char(952);
    % offset path (parametric): z = gap on the glass up to the corner, then the circle of radius rb + gap around the bead centre (xc, rb)
    % from the corner angle -a0 (normal pointing left and DOWN: undercut) over the top to +a0, then the glass again
    Rr = rb + gap;  a0 = 180 - acosd((gap - rb) / Rr) * 1 + 0;   % angle of the normal from +z where the circle meets z = gap
    a0 = atan2d(sqrt(Rr^2 - (gap - rb)^2), gap - rb);            % = 90 + asind((rb - gap) / Rr)
    xj = xc + [-1 1] * sqrt(Rr^2 - (gap - rb)^2);
    aa = linspace(-a0, a0, 1500);
    xs = [linspace(0, xj(1), 300), xc + Rr * sind(aa), linspace(xj(2), 2 * xc, 300)];
    zs = [gap + 0 * (1:300), rb + Rr * cosd(aa), gap + 0 * (1:300)];
    tilt = [zeros(1, 300), aa, zeros(1, 300)];
    sarc = [linspace(0, xj(1), 300), xj(1) + Rr * deg2rad(aa + a0), xj(1) + Rr * deg2rad(2 * a0) + linspace(0, 2 * xc - xj(2), 300)];   % path length
    f = figure('Position', [40 40 2000 1300], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl);  hold on;  box on;
    fill([-5 2 * xc + 5 2 * xc + 5 -5], [0 0 -4 -4], [0.9 0.9 0.9], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    h1 = plot([-5 2 * xc + 5], [0 0], '-', 'Color', cW, 'LineWidth', 4, 'DisplayName', 'cover glass');
    th = linspace(0, 2 * pi, 400);
    fill(xc + rb * cos(th), rb + rb * sin(th), [0.8 0.88 0.95], 'EdgeColor', cW, 'LineWidth', 3, 'DisplayName', sprintf('glass bead (%g %sm)', 2 * rb, MU));
    h3 = plot(xs, zs, '-', 'Color', cE, 'LineWidth', 2.5, 'DisplayName', 'probe centre at the gap h* R');
    plot([0 0], [gap 15], '--', 'Color', cE, 'LineWidth', 2.5, 'HandleVisibility', 'off');
    plot(0, 15, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'HandleVisibility', 'off');
    text(0.6, 15, sprintf('start, 15 %sm above the glass', MU), 'FontSize', FS - 4, 'FontWeight', 'bold');
    plot(xj, [gap gap], 'kx', 'MarkerSize', 16, 'LineWidth', 3, 'HandleVisibility', 'off');
    text(xj(1) - 7, gap + 1.2, 'corner: normal jumps', 'FontSize', FS - 4, 'FontWeight', 'bold');
    for a = [-100 -60 0 60 100]                                  % normals on the bead at a few tilts (deg from +z)
        p0 = [xc + rb * sind(a), rb + rb * cosd(a)];  n = [sind(a) cosd(a)];
        if p0(2) + gap * n(2) < gap - 1e-9; continue; end
        quiver(p0(1), p0(2), 2.2 * n(1), 2.2 * n(2), 0, 'k', 'LineWidth', 2.5, 'MaxHeadSize', 0.8, 'HandleVisibility', 'off');
    end
    axis equal;  xlim([-3 2 * xc + 3]);  ylim([-2 16]);
    xlabel(['x  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(['z  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;
    ax = nexttile(tl);  hold on;  box on;
    plot(sarc, tilt, '-', 'Color', cE, 'LineWidth', 3);
    yline([-90 90], ':', 'Color', cW, 'LineWidth', 1.5);
    xlim([0 sarc(end)]);  ylim([-120 120]);  yticks(-120:30:120);
    xlabel(['distance along the path  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel([TH '_y  (normal tilt)  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;
    fprintf('bead r %g um, gap %g um: path leaves the glass at x = %.2f um (%.2f um from the bead centre); normal jumps 0 -> %.1f deg from +z;\n  path length on the bead %.1f um (of %.1f um total)\n', ...
            rb, gap, xj(1), xc - xj(1), a0, Rr * deg2rad(2 * a0), sarc(end));
    out = fullfile(od, 'fig_wall_bead_setup.png');
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
