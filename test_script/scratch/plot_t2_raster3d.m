% STATUS: SCRATCH -- 2026-10-22 (user: "完整把掃描出來向 meng 呈現"). Raster scan of the bump (run_t2_3d_reg, opts.raster), Meng diss. Fig. 56 / 59
%   style. Left: probe path during the raster in the measurement coordinate (3-D lines, blue). Right: estimated surface = probe position
%   minus the regulated gap along the estimated normal, p - h* R w-hat (red points), over the true bump (grey mesh). Console: rms and max
%   of the height error of the estimated surface against the true wall height under each point, and the lowest gap during the raster.
%   -> test_results/temp_3d_wsonly/fig_t2_raster3d_<tag>.png | run: plot_t2_raster3d('raster_s2')
function plot_t2_raster3d(tag)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');  addpath(genpath(fullfile(root, 'model')));  pc = physical_constants();  Rr = pc.R;
    S = load(fullfile(od, ['temp_t2_' tag '.mat']));  o = S.R.opts;  r = S.R.runs{1};  k = 1:r.kend;
    H = o.bump(1);  sg = o.bump(2);  xc = o.bump(3);  yc = o.bump(4);
    sc = k(r.t(k) >= o.t_scan);  sc = sc(1:20:end);                 % every 20th step is enough for the picture
    p = r.p(sc, :);
    wh = [sind(r.thy(sc)) .* cosd(r.thx(sc)), -sind(r.thx(sc)), cosd(r.thy(sc)) .* cosd(r.thx(sc))];   % estimated normal
    q = p - r.h_star * Rr * wh;                                         % estimated surface point
    ztrue = H * exp(-((q(:, 1) - xc).^2 + (q(:, 2) - yc).^2) / (2 * sg^2));
    [X, Y] = meshgrid(linspace(-5, 45, 101), linspace(-5, 35, 81));  Z = H * exp(-((X - xc).^2 + (Y - yc).^2) / (2 * sg^2));
    cE = [0 0.25 0.85];  cT = [0.85 0 0];  FS = 18;  LFS = 14;  MU = char(181);
    f = figure('Position', [40 40 2200 900], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = nexttile(tl);  hold on;  box on;
    h1 = plot3(p(:, 1), p(:, 2), p(:, 3), '-', 'Color', cE, 'LineWidth', 1.2, 'DisplayName', 'probe path p');
    view(-35, 30);  axis equal;  xlim([-5 45]);  ylim([-5 35]);  zlim([0 8]);
    xlabel(['x  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(['y  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  zlabel(['z  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
    legend(h1, 'Location', 'northoutside', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;
    ax = nexttile(tl);  hold on;  box on;
    mesh(X, Y, Z, 'EdgeColor', [0.6 0.6 0.6], 'FaceColor', 'none', 'DisplayName', 'true surface');
    h2 = plot3(q(:, 1), q(:, 2), q(:, 3), '.', 'Color', cT, 'MarkerSize', 5, 'DisplayName', ['estimated surface  p - h* R ' 'w' char(770)]);
    view(-35, 30);  axis equal;  xlim([-5 45]);  ylim([-5 35]);  zlim([-1 5]);
    xlabel(['x  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(['y  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  zlabel(['z  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
    legend('Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    set(ax, 'FontSize', FS - 2, 'FontWeight', 'bold', 'LineWidth', 2);  grid off;
    e = q(:, 3) - ztrue;  scf = k(r.t(k) >= o.t_scan);
    fprintf('%s: contact %d (t %.2f), lines done %d of %d | surface height error rms %.3f um, max %.3f um | lowest gap h %.3f R (%.0f nm) | scan h - h* mean %+.3f R\n', ...
            tag, r.contact, r.t_contact, max(r.line(scf)) - 1, o.raster(3), rms(e), max(abs(e)), min(r.h(scf)), (min(r.h(scf)) - 1) * Rr * 1e3, mean(r.h(scf)) - r.h_star);
    out = fullfile(od, sprintf('fig_t2_raster3d_%s.png', tag));
    exportgraphics(f, out, 'Resolution', 110);  close(f);  fprintf('figure: %s\n', out);
end
