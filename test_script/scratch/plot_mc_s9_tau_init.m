% STATUS: SCRATCH -- 2026-10-14 (user): gain regulation with tau_a 0.1 / 0.05 s and estimator start w_s-hat[0] = -1 / 0 / +1 (plane, seed 1).
%   Columns: w_s-hat[0] = -1 | 0 | +1. Rows: a-bar_z (log) for tau_a 0.1 s, w_s-hat for 0.1 s, a-bar_z for 0.05 s, w_s-hat for 0.05 s.
%   Red true (a-bar_z of the plant; wall position 0), blue estimate, black dashed a_d. Data: run_mc_s9_gainreg.m.
function plot_mc_s9_tau_init()
    here = fileparts(mfilename('fullpath'));  od = fullfile(fileparts(fileparts(here)), 'test_results', 'temp_1009_ws');
    TM = [100 50];  WT = {'_wm1', '', '_wp1'};  W0 = [-1 0 1];
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 18;  LFS = 15;  AB = char(257);  HAT = char(770);  WB = ['w' char(772)];
    f = figure('Position', [40 40 2100 1800], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    fprintf('tau_a | ws0 | min h_true | t63 | last 0.5 s: a - a_d | a_hat - a_d | w_s-hat end\n');
    for it = 1:2
        for iw = 1:3
            S = load(fullfile(od, sprintf('gainreg_plane_tau%03d%s.mat', TM(it), WT{iw})));  t = S.t;  L = t > t(end) - 0.5;
            fprintf('%.2f | %+d | %.4f | ', S.tau_a, W0(iw), min(S.h_true));
            fprintf('%+.4f | %+.4f | %+.3f\n', mean(S.a_true(L) - S.a_d), mean(S.a_hat(L) - S.a_d), S.w_hat(end));
            a1 = nexttile(tl, (2 * it - 2) * 3 + iw);  hold on; box on;
            plot(a1, t, S.a_true, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_z']);
            plot(a1, t, S.a_hat, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [AB HAT '_z']);
            yline(a1, S.a_d, '--', 'Color', 'k', 'LineWidth', 2, 'DisplayName', [AB '_d']);
            set(a1, 'YScale', 'log');  ylim(a1, [0.05 1]);  yticks(a1, [0.05 0.1 0.2 0.5 1]);
            text(a1, 0.97, 0.88, sprintf('%s_a = %.2f s,  %s%s_s[0] = %+d', char(964), S.tau_a, WB, HAT, W0(iw)), 'Units', 'normalized', ...
                 'HorizontalAlignment', 'right', 'FontSize', LFS, 'FontWeight', 'bold');
            if it == 1 && iw == 1; legend(a1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold'); end
            a2 = nexttile(tl, (2 * it - 1) * 3 + iw);  hold on; box on;
            plot(a2, t, zeros(size(t)), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [WB '_s']);
            plot(a2, t, S.w_hat, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [WB HAT '_s']);
            ylim(a2, [-1.5 1.5]);
            if it == 1 && iw == 1; legend(a2, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold'); end
            if it == 2; xlabel(a2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            if iw == 1; ylabel(a1, [AB '_z'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a2, [WB '_s  [R]'], 'FontSize', FS, 'FontWeight', 'bold'); end
            for a = [a1 a2]; xlim(a, [0 t(end)]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(a, 'off'); end
        end
    end
    out = fullfile(od, 'fig_mc_s9_tau_init.png');  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end
