% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Section 7 (reference result: raster scan parallel to the wall). Plane, z held at
%   1.10 R, x 4 Hz triangle +-20 um, y +1 um at every x turnaround, 2 s; Section 6 estimator on z (c-only, P_fix, c[0] = 0) and
%   par_wall_known on x, y (plane parallel law, wall-normal step only); lambda_c 0.4; seed 1. Run: run_plaw_walls tag
%   _mcs5_raster110_ratio_parwk (see plot_mc_raster_x.m for the before/after comparison of the parallel-axis model).
%   Fig. 7.1 (which = 'traj'): left the scan pattern y vs x, actual (red) and desired (blue dashed); right x_d, y_d, z_d vs t.
%   Fig. 7.2 (which = 'gain'): rows x, y, z; left the gain, true (red) and estimate (blue); right the measured tracking error.
function plot_mc_raster_ref(which, seed)
    if nargin < 1; which = 'traj'; end
    if nargin < 2; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');  id = fullfile(root, 'test_results', 'temp_plaw_walls');
    S = load(fullfile(id, 'f1p2_plane_PI_mcs5_raster110_ratio_parwk_ramp2_lc040.mat'));
    t = S.t(:);  Ts = t(2) - t(1);  tn = t + Ts;  R_UM = 2.25;
    xd = 20 * (2 / pi) * asin(sin(2 * pi * 4 * tn));  yd = floor(2 * 4 * tn + 0.5);
    wd = S.wd(:);  if wd(1) == 0; wd(1) = wd(2); end;  zd = wd * R_UM;
    D = squeeze(S.dxm_xyz(:, :, seed));
    xa = [xd(1); xd(1); xd(1:end-2)] - D(:, 1);  ya = [yd(1); yd(1); yd(1:end-2)] - D(:, 2);
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 20;  LFS = 15;  AXLW = 2.0;  AB = char(257);  HAT = char(770);  AXN = {'x', 'y', 'z'};
    if strcmp(which, 'traj')
        f = figure('Position', [40 40 1800 950], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        ax = nexttile(tl, 1, [3 1]);  hold on; box on;
        ha = plot(ax, xa, ya, '-', 'Color', cT, 'LineWidth', 4.0, 'DisplayName', 'actual');
        hd = plot(ax, xd, yd, '--', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'desired');
        xlim(ax, [-25 25]);  ylim(ax, [-1 17]);  xlabel(ax, ['x  [' char(181) 'm]']);  ylabel(ax, ['y  [' char(181) 'm]']);
        legend(ax, [ha hd], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax, 'off');
        Y = {xd, yd, zd};  YL = {[-25 25], [-1 18], [0 4]};
        for ia = 1:3
            a2 = nexttile(tl, 2 * ia);  hold on; box on;
            plot(a2, t, Y{ia}, '-', 'Color', cE, 'LineWidth', 2.5);
            ylim(a2, YL{ia});  xlim(a2, [0 t(end)]);  ylabel(a2, [AXN{ia} '_d  [' char(181) 'm]']);
            if ia == 3; xlabel(a2, 'Time (sec)'); end
            set(a2, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a2, 'off');
        end
        out = fullfile(fd, sprintf('fig_mc_raster_traj_s%d.png', seed));
    else
        hz = S.h_true(:, seed);  cpa = arrayfun(@(h) local_cpar(h), hz);
        A_true = [1 ./ cpa, 1 ./ cpa, S.a_true(:, seed)];  A_hat = squeeze(S.a_hat_xyz(:, :, seed));
        f = figure('Position', [40 40 1800 1250], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
        tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        for ia = 1:3
            ax = nexttile(tl, 2 * ia - 1);  hold on; box on;
            ht = plot(ax, t, A_true(:, ia), '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', [AB '_' AXN{ia}]);
            hh = plot(ax, t, A_hat(:, ia), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_' AXN{ia}]);
            ylim(ax, [0 0.6]);  xlim(ax, [0 t(end)]);
            legend(ax, [ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            a2 = nexttile(tl, 2 * ia);  hold on; box on;
            plot(a2, t, D(:, ia), '-', 'Color', cE, 'LineWidth', 1.0);
            ylim(a2, [-0.2 0.2]);  yticks(a2, [-0.1 0 0.1]);  xlim(a2, [0 t(end)]);
            ylabel(a2, [char(948) AXN{ia} '_m  [' char(181) 'm]']);
            if ia == 3; xlabel(ax, 'Time (sec)');  xlabel(a2, 'Time (sec)'); end
            for a = [ax a2]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a, 'off'); end
            w = t > 0.5;
            fprintf('%s: gain true %.3f est %.3f (t>0.5) | tracking rms %.4f um, max %.3f um\n', AXN{ia}, mean(A_true(w, ia)), mean(A_hat(w, ia)), rms(D(:, ia)), max(abs(D(:, ia))));
        end
        out = fullfile(fd, sprintf('fig_mc_raster_gain_s%d.png', seed));
    end
    exportgraphics(f, out, 'Resolution', 140);  fprintf('figure: %s\n', out);
end

function c = local_cpar(h)
    [c, ~] = calc_correction_functions(max(h, 1.0));
end
