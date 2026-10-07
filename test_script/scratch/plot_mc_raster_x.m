% STATUS: SCRATCH -- 2026-10-07, raster scan x-axis diagnosis (user: show the full x tracking). Plane, z held at 1.10 R, x 4 Hz
%   triangle +-20 um, y +1 um per x turnaround, lambda_c 0.4, seed 1. Columns: Section 6 estimator in the law (run_mc_s5_conly
%   'raster110', P_fix, c[0] = 0) | true gain in the law (reference). Rows: (1) scan pattern y vs x, actual (red, from
%   x_d[k-2] - delta x_m) and desired (blue dashed); (2) x(t) actual / desired, first 0.3 s; (3) x gain: true a-bar_x = 1/c_par(h)
%   (red) and estimate (blue); (4) delta x_m [um].
function plot_mc_raster_x(seed)
    if nargin < 1; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);  addpath(genpath(fullfile(root, 'model')));
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');  id = fullfile(root, 'test_results', 'temp_plaw_walls');
    ft = fullfile(id, 'f1p2_plane_PI_mcs5_raster110_ctrltrue_ramp2_lc040.mat');
    o = struct('law', 'f1p2', 'seeds', 1:5, 'sig', 0.1, 'plants', {{'plane'}}, 'lambda_c', 0.4, 'q55_adapt', 'ratio', 'q55_adapt_P0', 0.15, ...
               'c_seed', struct('plane', 0, 'cell', 0, 'sphere', 0, 'sphere_pt', 0), 'traj', 'ramp', 'Td', 2, 'a_ctrl_override', 'true', ...
               'tag_suffix', '_mcs5_raster110_ctrltrue');
    o.ov_extra = struct('trajectory_type', 'ramp_descent', 'h_init', 1.1 * 2.25, 'h_bottom', 1.1 * 2.25, 't_hold', 0, 't_descend_override', 2, ...
                        'T_sim', 2, 'amp_u', -20, 'freq_u', 4, 'tan_shape', 1, 'amp_v', -1, 'freq_v', 4, 'tan_v_stair', 1);
    T = load(ft);  if ~isfield(T, 'a_hat_xyz'); evalc('run_plaw_walls(o);'); end
    D = {load(fullfile(id, 'f1p2_plane_PI_mcs5_raster110_ratio_ramp2_lc040.mat')), load(fullfile(id, 'f1p2_plane_PI_mcs5_raster110_ratio_parwk_ramp2_lc040.mat'))};   % before | after par_wall_known
    [cpa, ~] = calc_correction_functions(1.10);  a_par = 1 / cpa;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 20;  LFS = 15;  AXLW = 2.0;  AB = char(257);  HAT = char(770);
    f = figure('Position', [40 40 1700 2100], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    NM = {'before: perpendicular model on x', 'after: parallel law on x, y (par\_wall\_known)'};
    for ic = 1:2
        S = D{ic};  t = S.t(:);  Ts = t(2) - t(1);  tn = t + Ts;
        xd = 20 * (2 / pi) * asin(sin(2 * pi * 4 * tn));  yd = floor(2 * 4 * tn + 0.5);
        dx = S.dxm_xyz(:, 1, seed);  dy = S.dxm_xyz(:, 2, seed);
        xd2 = [xd(1); xd(1); xd(1:end-2)];  yd2 = [yd(1); yd(1); yd(1:end-2)];
        x = xd2 - dx;  y = yd2 - dy;                                  % measured position from delta x_m = x_d[k-2] - x_m[k]
        ax = nexttile(tl, ic);  hold on; box on;
        h1 = plot(ax, x, y, '-', 'Color', cT, 'LineWidth', 3.0, 'DisplayName', 'actual');
        h2 = plot(ax, xd, yd, '--', 'Color', cE, 'LineWidth', 1.5, 'DisplayName', 'desired');
        xlim(ax, [-25 25]);  ylim(ax, [-1 17]);  xlabel(ax, ['x  [' char(181) 'm]']);  ylabel(ax, ['y  [' char(181) 'm]']);
        legend(ax, [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        title(ax, NM{ic}, 'FontSize', FS - 2);
        ax2 = nexttile(tl, 2 + ic);  hold on; box on;  w = t <= 0.3;
        plot(ax2, t(w), x(w), '-', 'Color', cT, 'LineWidth', 3.0);  plot(ax2, t(w), xd(w), '--', 'Color', cE, 'LineWidth', 1.5);
        ylabel(ax2, ['x  [' char(181) 'm]']);  ylim(ax2, [-25 25]);  xlim(ax2, [0 0.3]);
        ax3 = nexttile(tl, 4 + ic);  hold on; box on;
        plot(ax3, t, a_par * ones(size(t)), '-', 'Color', cT, 'LineWidth', 5.0);  plot(ax3, t, S.a_hat_xyz(:, 1, seed), '-', 'Color', cE, 'LineWidth', 1.8);
        ylabel(ax3, [AB '_x,  ' AB HAT '_x']);  ylim(ax3, [0 1.2]);  xlim(ax3, [0 t(end)]);
        ax4 = nexttile(tl, 6 + ic);  hold on; box on;
        plot(ax4, t, dx, '-', 'Color', cE, 'LineWidth', 1.0);  ylabel(ax4, [char(948) 'x_m  [' char(181) 'm]']);  xlim(ax4, [0 t(end)]);
        xlabel(ax4, 'Time (sec)');  xlabel(ax2, 'Time (sec)');  xlabel(ax3, 'Time (sec)');
        for a = [ax ax2 ax3 ax4]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a, 'off'); end
        fprintf('%-26s | dx_m mean (t>0.5) %+.3f, rms %.3f, max %.2f um | a_hat_x mean (t>0.5) %.3f (true %.3f)\n', NM{ic}, mean(dx(t > 0.5)), ...
                rms(dx), max(abs(dx)), mean(S.a_hat_xyz(t > 0.5, 1, seed)), a_par);
    end
    out = fullfile(fd, sprintf('fig_mc_raster110_x_s%d.png', seed));  exportgraphics(f, out, 'Resolution', 130);  fprintf('figure: %s\n', out);
end
