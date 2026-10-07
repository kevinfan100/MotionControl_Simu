% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Section 2: desired trajectory of the three-axis run (run_mc_s2_xyz.m),
%   computed from the trajectory definition (trajectory_generator.m: ramp_descent + tangential triangles, tan_shape = 1):
%   x_d = y_d = 1 Hz triangle, amplitude 9 um, z_d = distance from the wall, 50 -> 2.475 um (22.2 -> 1.10 R) at constant velocity over 5 s.
function plot_mc_s2_traj()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    Ts = 1 / 1600;  t = (0:Ts:5)';
    xd = 9 * (2 / pi) * asin(sin(2 * pi * 1 * t));  yd = xd;  zd = 50 - (50 - 1.1 * 2.25) * t / 5;   % x/y triangle (user 10-07), tan_shape = 1
    cE = [0 0.25 0.85];  FS = 22;  AXLW = 2.2;  LW = 3.0;
    f = figure('Position', [40 40 1600 900], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    Y = {xd, yd, zd};  L = {'x_d', 'y_d', 'z_d'};  YL = {[-10 10], [-10 10], [0 55]};
    for i = 1:3
        ax = nexttile(tl);  hold on; box on;
        plot(ax, t, Y{i}, '-', 'Color', cE, 'LineWidth', LW);
        xlim(ax, [0 5]);  ylim(ax, YL{i});
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax, 'off');
        ylabel(ax, [L{i} '  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
        if i == 3; xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
    end
    out = fullfile(fd, 'fig_mc_s2_traj.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
