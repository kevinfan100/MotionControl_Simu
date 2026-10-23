% STATUS: SCRATCH -- 2026-10-15 (user): Fig 9.6, wall position alone (p = 2) vs wall position and exponent, cell wall, gain regulation
%   a_d 0.52, 6 s, seed 1, start w_s-hat[0] = 0 (run_mc_s9_gainreg tags '_wsp_s8' / '_wsp_f4', both with fe_cmd_eval). Columns p = 2 | p free.
%   Rows: a-bar_z (red true, blue estimate, black dashed a_d); w_s (red physical wall -1.05, blue estimate); p (left: grey p = 2 fixed;
%   right: blue p-hat, black dashed p_eff(h) = local exponent of the true slope at the true height, physical wall).
function plot_mc_s9_wsp_cell()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));  addpath(here);
    od = fullfile(root, 'test_results', 'temp_1009_ws');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    T = {'_wsp_s8', '_wsp_f4'};  N = {'p = 2', 'p estimated'};
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 20;  LFS = 15;  AB = char(257);  HAT = char(770);  WB = ['w' char(772)];
    f = figure('Position', [40 40 2000 1450], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for j = 1:2
        S = load(fullfile(od, sprintf('gainreg_cell_tau100_T6%s.mat', T{j})));  t = S.t;  q = 1;
        a1 = nexttile(tl, j);  hold on; box on;
        plot(a1, t, S.a_true(:, q), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_z']);
        plot(a1, t, S.a_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [AB HAT '_z']);
        yline(a1, S.a_d, '--', 'Color', 'k', 'LineWidth', 2, 'DisplayName', [AB '_d']);  ylim(a1, [0.45 0.75]);
        text(a1, 0.97, 0.88, N{j}, 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', LFS + 3, 'FontWeight', 'bold');
        a2 = nexttile(tl, 2 + j);  hold on; box on;
        yline(a2, 0, '-', 'Color', [0.55 0.55 0.55], 'LineWidth', 2, 'DisplayName', 'visible surface');   % 10-15 (user): unified wording
        yline(a2, -1.05, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', ['equivalent wall ' WB '_s']);
        plot(a2, t, S.w_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [WB HAT '_s']);  ylim(a2, [-1.5 0.5]);
        a3 = nexttile(tl, 4 + j);  hold on; box on;
        if j == 1
            yline(a3, 2, '-', 'Color', [0.55 0.55 0.55], 'LineWidth', 2.5, 'DisplayName', 'p = 2 (fixed)');
        else
            plot(a3, t, S.p_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', ['p' HAT]);
            plot(a3, t, local_peff(max(S.h_true(:, q), 1.001)), '--', 'Color', 'k', 'LineWidth', 1.8, 'DisplayName', 'p_{eff}(h)');
        end
        ylim(a3, [1.7 2.2]);  xlabel(a3, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if j == 1
            ylabel(a1, [AB '_z'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a2, [WB '_s  [R]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a3, 'p', 'FontSize', FS, 'FontWeight', 'bold');
        end
        for a = [a1 a2 a3]; xlim(a, [0 t(end)]);  legend(a, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(a, 'off'); end
        L = t > 5.5;  fprintf('%-12s | last 0.5 s a - a_d %+.4f | w_s-hat end %+.3f | p-hat end %.3f\n', N{j}, mean(S.a_true(L, q) - S.a_d), S.w_hat(end, q), S.p_hat(end, q));
    end
    out = fullfile(fd, 'fig_mc_s9_wsp_cell_s1.png');  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end

function pe = local_peff(h)
    d = 1e-4;  h1 = h * (1 + d);  h0 = h * (1 - d);
    pe = -(log(local_ap(h1)) - log(local_ap(h0))) ./ (log(h1 + 1.05) - log(h0 + 1.05));
end

function ap = local_ap(h)
    d = 1e-5;  ap = (1 ./ cell_cperp_delta(h + d) - 1 ./ cell_cperp_delta(h - d)) / (2 * d);
end
