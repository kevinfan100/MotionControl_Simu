% STATUS: SCRATCH -- 2026-10-15 (user): final position with a wrong wall guess (true wall moved by DELTA, estimator at 0), seed 1, from
%   probe_mc_s9_regopt files. Columns: plane DELTA -2 two-stage B1 | cell DELTA -1 single stage (base) | cell DELTA -1 two-stage with dither B2.
%   Rows: true gap h (blue; red = command in the true frame; black dashed h* where the plant gain equals a_d); a-bar_z true (red), estimate (blue), a_d (dashed).
function plot_mc_s9_regopt()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  od = fullfile(root, 'test_results', 'temp_1009_ws');
    C = {'plane', 'B1', -2, 0.07, 1.0781, 'plane, \Delta = -2 R, two stages'; 'cell', 'base', -1, 0.52, 1.1741, 'cell, \Delta = -1 R, single stage'; ...
         'cell', 'B2', -1, 0.52, 1.1741, 'cell, \Delta = -1 R, two stages + dither'};
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 18;  LFS = 14;  AB = char(257);  HAT = char(770);
    f = figure('Position', [40 40 2100 1050], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for j = 1:3
        tg = sprintf('_ro_%s_%s_%+g_s1', C{j, 1}, C{j, 2}, C{j, 3});  tg = strrep(strrep(tg, '+', 'p'), '-', 'm');
        S = load(fullfile(od, sprintf('gainreg_%s_tau100_T6%s.mat', C{j, 1}, tg)));  t = S.t;  wd = S.wd;  wd(1) = wd(2);
        a1 = nexttile(tl, j);  hold on; box on;
        plot(a1, t, wd - C{j, 3}, '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', 'h_d (true frame)');  plot(a1, t, S.h_true, '--', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'h');
        yline(a1, C{j, 5}, '--', 'Color', 'k', 'LineWidth', 1.8, 'DisplayName', 'h^*');  ylim(a1, [0.95 2.5]);
        text(a1, 0.97, 0.88, C{j, 6}, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', LFS + 1, 'FontWeight', 'bold');
        a2 = nexttile(tl, 3 + j);  hold on; box on;
        plot(a2, t, S.a_true, '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', [AB '_z']);  plot(a2, t, S.a_hat, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [AB HAT '_z']);
        yline(a2, C{j, 4}, '--', 'Color', 'k', 'LineWidth', 2, 'DisplayName', [AB '_d']);  ylim(a2, [C{j, 4} - 0.1, C{j, 4} + 0.3]);
        xlabel(a2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if j == 1; ylabel(a1, 'true gap h  [R]', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a2, [AB '_z'], 'FontSize', FS, 'FontWeight', 'bold'); end
        for a = [a1 a2]; xlim(a, [0 t(end)]);  legend(a, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(a, 'off'); end
        fprintf('%s: last 0.5 s h - h* %+.3f R, a - a_d %+.4f, a_hat - a_d %+.4f\n', C{j, 6}, mean(S.h_true(t > 5.5)) - C{j, 5}, mean(S.a_true(t > 5.5)) - C{j, 4}, mean(S.a_hat(t > 5.5)) - C{j, 4});
    end
    out = fullfile(od, 'fig_mc_s9_regopt.png');  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end
