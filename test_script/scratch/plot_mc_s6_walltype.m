% STATUS: SCRATCH -- 2026-10-07, motion_ctrl last figure of Section 6 (user: a ramp whose end value of c-hat tells the wall type).
%   Trajectory: 22.2 -> 1.10 R in T_d = 0.5 s, then hold 0.2 T_d (run_mc_s5_conly.m traj 'rh050'). Estimator of Section 6
%   (q55 = P_fix^2 I, sqrt(P_fix) 0.15) started from the plane on both walls. Columns: plane | cell. Rows: tracking (z red,
%   z_d blue dashed), a-bar_z true (red) / estimate (blue), c(h_d) true (red) / c-hat (blue), delta z_m [um]. Seed 1.
%   T_d choice (10-07, 5 seeds, end-of-hold c-hat, separation cell - plane in seed sd): P_fix 5.4 / 4.1 / 2.9 sigma at
%   T_d 0.5 / 1 / 2 s; q55 = 0 18 / 11 / 8 sigma but biased (cell 0.36 vs c(1.10) = 0.74).
function plot_mc_s6_walltype(seed, tag)
    if nargin < 1; seed = 1; end
    if nargin < 2; tag = 'rh050'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');  id = fullfile(root, 'test_results', 'temp_plaw_walls');
    W = {'plane', 'cell'};  C = cell(1, 2);
    for iw = 1:2
        f0 = dir(fullfile(id, sprintf('f1p2_%s_PI_mcs5_%s_ratio*.mat', W{iw}, tag)));  S = load(fullfile(id, f0(1).name));
        wd = S.wd(:);  if wd(1) == 0; wd(1) = wd(2); end
        c_true = S.c_eff_d(:);  if c_true(1) > c_true(2) + 0.1; c_true(1) = c_true(2); end
        C{iw} = struct('t', S.t(:), 'h_d', wd, 'h', S.h_true(:, seed), 'a_true', S.a_true(:, seed), 'a_hat', S.a_hat(:, seed), ...
                       'c_hat', S.c_hat(:, seed), 'c_true', c_true, 'dxm', S.dxm(:, seed), 'c_all', S.c_hat);
    end
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 22;  LFS = 17;  AXLW = 2.2;  R_UM = 2.25;
    AB = char(257);  HAT = char(770);  az = [AB '_z'];
    f = figure('Position', [40 40 1600 2000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        r = C{ic};  t = r.t;  ax = gobjects(4, 1);
        ax(1) = nexttile(tl, ic);  hold on; box on;
        hz = plot(ax(1), t, r.h * R_UM, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'z');
        hd = plot(ax(1), t, r.h_d * R_UM, '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'z_d');
        ylim(ax(1), [0 55]);
        legend(ax(1), [hz hd], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(2) = nexttile(tl, 2 + ic);  hold on; box on;
        ht = plot(ax(2), t, r.a_true, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', az);
        hh = plot(ax(2), t, r.a_hat, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
        ylim(ax(2), [-0.2 1.6]);
        legend(ax(2), [ht hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(3) = nexttile(tl, 4 + ic);  hold on; box on;
        hct = plot(ax(3), t, r.c_true, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'c');
        hc = plot(ax(3), t, r.c_hat, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', ['c' HAT]);
        ylim(ax(3), [-0.4 1.2]);
        legend(ax(3), [hct hc], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(4) = nexttile(tl, 6 + ic);  hold on; box on;
        plot(ax(4), t, r.dxm, '-', 'Color', cE, 'LineWidth', 1.0);
        ylim(ax(4), [-0.2 0.2]);  yticks(ax(4), [-0.1 0 0.1]);
        xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for q = 1:4; xlim(ax(q), [0 t(end)]);  set(ax(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(q), 'off'); end
        if ic == 1
            ylabel(ax(1), ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
            ylabel(ax(2), az, 'FontSize', FS + 2, 'FontWeight', 'bold');
            ylabel(ax(3), 'c', 'FontSize', FS + 2, 'FontWeight', 'bold');
            ylabel(ax(4), [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        end
        Td = t(end) / 1.2;  mh = t > Td + 0.02;  ce = mean(r.c_all(mh, :), 1);
        fprintf('%-5s | c_hat in hold (seed %d) %.3f | 5 seeds %.3f +- %.3f | true c(1.10) %.3f\n', W{ic}, seed, mean(r.c_hat(mh)), mean(ce), std(ce), r.c_true(end));
    end
    out = fullfile(fd, sprintf('fig_mc_s6_walltype_%s_s%d.png', tag, seed));  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
