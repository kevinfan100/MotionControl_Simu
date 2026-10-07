% STATUS: SCRATCH -- 2026-10-07, motion_ctrl Section 2, three-axis figure in the layout of the paper's Fig. 5.
%   Columns: true motion gain in the control law | nominal motion gain. Rows: measured tracking error delta x_m, delta y_m,
%   delta z_m [um], raw only, no legend (the document caption names the arms). All rows: y range +-0.2 um, ticks 0.1 (user 10-07).
%   Data: run_mc_s2_xyz.m (Scenario A shape, 5 s; z 15 -> 2.5 um, x/y 1 Hz 9 um sines; plane wall, lambda_c 0.4, seed 1).
function plot_mc_s2_xyz()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    id = fullfile(root, 'test_results', 'temp_plaw_walls');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    TAG = {'_s1_ctrltrue_xyzA', '_s1_ctrlnom_xyzA'};  NM = {'true', 'nominal'};
    AXN = {'x', 'y', 'z'};  cE = [0 0.25 0.85];  FS = 22;  AXLW = 2.2;
    D = cell(1, 2);
    for ic = 1:2
        S = load(fullfile(id, sprintf('f1p2_plane_PI%s_ramp5_lc040.mat', TAG{ic})));  D{ic} = S;
    end
    f = figure('Position', [40 40 1600 1200], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ia = 1:3
        lim = 0.2;   % 10-07 (user): one y range and one tick spacing for all three rows
        for ic = 1:2
            S = D{ic};  t = S.t(:);  x = S.dxm_xyz(:, ia, 1);
            ax = nexttile(tl, (ia - 1) * 2 + ic);  hold on; box on;
            plot(ax, t, x, '-', 'Color', cE, 'LineWidth', 1.0);
            xlim(ax, [0 t(end)]);  ylim(ax, [-lim lim]);  yticks(ax, [-0.1 0 0.1]);   % user 10-07: three ticks
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax, 'off');
            if ia == 3; xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            if ic == 1; ylabel(ax, [char(948) AXN{ia} '_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold'); end
            last = t > 4;
            fprintf('%-8s %s: rms all %.4f um | rms last 1 s %.4f um | max |.| %.3f um\n', NM{ic}, AXN{ia}, sqrt(mean(x.^2)), sqrt(mean(x(last).^2)), max(abs(x)));
        end
    end
    out = fullfile(fd, 'fig_mc_s2_xyz.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
