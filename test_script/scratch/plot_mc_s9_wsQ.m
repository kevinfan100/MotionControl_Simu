% STATUS: SCRATCH -- 2026-10-15 (user): process noise on w_s, cell wall, true wall moved -1 R (estimator at 0), a_d 0.52, seed 4
%   (the median of the five paired improvements). Columns: Q = 0 | sqrt P_fix = 0.245 (q77_adapt_P0). Rows: true gap h (blue, dashed h*);
%   a-bar_z (red true, blue estimate, dashed a_d); w_s-hat (blue, band +- sqrt P_ws) with the true wall -2.05 R in the estimator frame (red).
%   Files from probe_mc_s9_wsQ (_wq_m1_<000|245>_s4).
function plot_mc_s9_wsQ(sd)
    if nargin < 1; sd = 4; end
    here = fileparts(mfilename('fullpath'));  od = fullfile(fileparts(fileparts(here)), 'test_results', 'temp_1009_ws');
    P = [0 245];  N = {'Q = 0', '\surd P_{fix} = 0.245'};  hs = 1.1741;  wt = -2.05;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 18;  LFS = 14;  AB = char(257);  HAT = char(770);  WB = ['w' char(772)];
    f = figure('Position', [40 40 1800 1450], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');  tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for j = 1:2
        S = load(fullfile(od, sprintf('gainreg_cell_tau100_T6_wq_m1_%03d_s%d.mat', P(j), sd)));  t = S.t;
        a1 = nexttile(tl, j);  hold on; box on;
        plot(a1, t, S.h_true, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'h');  yline(a1, hs, '--', 'Color', 'k', 'LineWidth', 1.8, 'DisplayName', 'h^* (gain = a_d)');
        ylim(a1, [1 2.5]);  text(a1, 0.97, 0.88, N{j}, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', LFS + 3, 'FontWeight', 'bold');
        a2 = nexttile(tl, 2 + j);  hold on; box on;
        plot(a2, t, S.a_true, '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', [AB '_z']);  plot(a2, t, S.a_hat, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [AB HAT '_z']);
        yline(a2, 0.52, '--', 'Color', 'k', 'LineWidth', 2, 'DisplayName', [AB '_d']);  ylim(a2, [0.45 0.8]);
        a3 = nexttile(tl, 4 + j);  hold on; box on;
        fill(a3, [t; flipud(t)], [S.w_hat + S.sPw; flipud(S.w_hat - S.sPw)], cE, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'DisplayName', ['\pm \surd P_{ws}']);
        yline(a3, wt, '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', ['true ' WB '_s']);  plot(a3, t, S.w_hat, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [WB HAT '_s']);
        ylim(a3, [-2.5 0.5]);  xlabel(a3, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if j == 1; ylabel(a1, 'true gap h  [R]', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a2, [AB '_z'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a3, [WB '_s  [R]'], 'FontSize', FS, 'FontWeight', 'bold'); end
        for a = [a1 a2 a3]; xlim(a, [0 t(end)]);  legend(a, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(a, 'off'); end
        fprintf('%s: h - h* %+.3f R, a - a_d %+.4f, w_s-hat end %+.3f (sqrtP %.3f)\n', N{j}, mean(S.h_true(t > 5.5)) - hs, mean(S.a_true(t > 5.5)) - 0.52, S.w_hat(end), S.sPw(end));
    end
    out = fullfile(od, sprintf('fig_mc_s9_wsQ_cell_s%d.png', sd));  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end
