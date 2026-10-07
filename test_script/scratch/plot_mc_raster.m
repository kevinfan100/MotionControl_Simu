% STATUS: SCRATCH -- 2026-10-07, raster scan parallel to the wall (user 10-07, from a trajectory sketch): x 4 Hz triangle +-20 um
%   (one pass 40 um in 0.125 s = 320 um/s = 142 R/s), y +1 um at every x turnaround, z held at 1.10 R. Plane wall, Section 6
%   estimator (c-only, q55 = P_fix^2 I, c[0] = 0), lambda_c 0.4, seed 1 (run_mc_s5_conly.m 'raster110', q55 'ratio').
%   Left column: desired x_d, y_d, z_d [um] (x_d, y_d from the generator formula, z_d from the log). Right: measured tracking
%   error delta x_m, delta y_m, delta z_m [um].
function plot_mc_raster(seed, tag)
    if nargin < 1; seed = 1; end
    if nargin < 2; tag = 'raster110'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');  id = fullfile(root, 'test_results', 'temp_plaw_walls');
    f0 = dir(fullfile(id, sprintf('f1p2_plane_PI_mcs5_%s_ratio*.mat', tag)));  S = load(fullfile(id, f0(1).name));
    t = S.t(:);  Ts = t(2) - t(1);  tn = t + Ts;  R_UM = 2.25;
    xd = 20 * (2 / pi) * asin(sin(2 * pi * 4 * tn));  yd = floor(2 * 4 * tn + 0.5);
    wd = S.wd(:);  if wd(1) == 0; wd(1) = wd(2); end;  zd = wd * R_UM;
    D = squeeze(S.dxm_xyz(:, :, seed));
    cE = [0 0.25 0.85];  FS = 22;  AXLW = 2.2;  AXN = {'x', 'y', 'z'};  Y = {xd, yd, zd};  YL = {[-25 25], [-1 18], [0 4]};
    f = figure('Position', [40 40 1800 1200], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ia = 1:3
        ax = nexttile(tl, (ia - 1) * 2 + 1);  hold on; box on;
        plot(ax, t, Y{ia}, '-', 'Color', cE, 'LineWidth', 3.0);
        ylim(ax, YL{ia});  ylabel(ax, [AXN{ia} '_d  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        ax2 = nexttile(tl, (ia - 1) * 2 + 2);  hold on; box on;
        plot(ax2, t, D(:, ia), '-', 'Color', cE, 'LineWidth', 1.0);
        ylim(ax2, [-0.2 0.2]);  yticks(ax2, [-0.1 0 0.1]);
        ylabel(ax2, [char(948) AXN{ia} '_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        for a = [ax ax2]; xlim(a, [0 t(end)]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a, 'off'); end
        if ia == 3; xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');  xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        fprintf('%s: tracking error rms %.4f um, max |.| %.3f um (seed %d) | 5 seeds rms %.4f\n', AXN{ia}, rms(D(:, ia)), max(abs(D(:, ia))), seed, ...
                mean(arrayfun(@(q) rms(S.dxm_xyz(:, ia, q)), 1:size(S.dxm_xyz, 3))));
    end
    fprintf('min h %.3f R | a_true z mean %.3f | a_hat z mean %.3f\n', min(S.h_true(:)), mean(S.a_true(:, seed)), mean(S.a_hat(:, seed)));
    out = fullfile(fd, sprintf('fig_mc_%s_s%d.png', tag, seed));  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
