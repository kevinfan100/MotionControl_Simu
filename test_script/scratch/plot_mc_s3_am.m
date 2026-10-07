% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Section 3 (measured input gain, paper II-E): z axis, a-bar_z true (red) and the
%   measured input gain a-bar_m (light blue). Data: the Section 2 run with the true motion gain in the control law
%   (run_mc_s2_xyz.m: z 22.2 -> 1.10 R in 5 s, x/y 1 Hz 9 um sines, plane wall, lambda_c 0.4, seed 1).
function plot_mc_s3_am()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    S = load(fullfile(id, 'f1p2_plane_PI_s1_ctrltrue_xyzA_ramp5_lc040.mat'));  t = S.t(:);
    cT = [0.85 0 0];  cM = [0.55 0.72 0.98];  FS = 22;  LFS = 17;  AXLW = 2.2;
    az = [char(257) '_z'];  am = [char(257) '_m'];
    f = figure('Position', [40 40 1600 600], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    ax = axes(f);  hold on; box on;
    h1 = plot(ax, t, S.a_true(:, 1), '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', az);   % red first (user 10-07)
    h2 = plot(ax, t, S.a_m(:, 1), '-', 'Color', cM, 'LineWidth', 1.0, 'DisplayName', am);
    xlim(ax, [0 t(end)]);  ylim(ax, [-0.2 1.6]);
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax, 'off');
    xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax, az, 'FontSize', FS + 2, 'FontWeight', 'bold');
    legend(ax, [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    w = t > 0.5 & t < 1.5;
    fprintf('far field (0.5-1.5 s): mean a_m %.3f, sd %.3f | a_true %.3f\n', mean(S.a_m(w, 1)), std(S.a_m(w, 1)), mean(S.a_true(w, 1)));
    out = fullfile(fd, 'fig_mc_s3_am.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
