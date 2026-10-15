% STATUS: SCRATCH -- 2026-10-15 (user): w_s-hat with the exponent pinned (S8, p = 2) vs free (a' = 1/(h - w_s)^p), both with fe_cmd_eval.
%   Rows: constant-velocity descent 22.2 -> 1.10 R in 1 s + 0.2 s hold (5 seeds); gain regulation (plane a_d 0.07, cell a_d 0.52, 6 s, 3 seeds).
%   Columns plane | cell. Seed-mean w_s-hat (black p = 2, blue p free, light band +- seed sd), red = physical wall (plane 0, cell -1.05).
function plot_mc_wsp_compare()
    od = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'test_results', 'temp_1009_ws');
    WL = {'plane', 'cell'};  WS = [0, -1.05];  cB = [0.15 0.15 0.15];  cF = [0 0.25 0.85];  FS = 18;  LFS = 14;  WB = ['w' char(772)];  HAT = char(770);
    f = figure('Position', [40 40 2000 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for row = 1:2
        for iw = 1:2
            if row == 1
                F = {fullfile(od, sprintf('apcmp_s8fc_%s_desc.mat', WL{iw})), fullfile(od, sprintf('apcmp_f4fc_%s_desc.mat', WL{iw}))};  lab = 'descent';
            else
                F = {fullfile(od, sprintf('gainreg_%s_tau100_T6_wsp_s8.mat', WL{iw})), fullfile(od, sprintf('gainreg_%s_tau100_T6_wsp_f4.mat', WL{iw}))};  lab = 'gain regulation';
            end
            ax = nexttile(tl);  hold on; box on;  C = {cB, cF};  N = {'p = 2', 'p free'};
            for k = 1:2
                S = load(F{k});  t = S.t;  mu = mean(S.w_hat, 2);  sd = std(S.w_hat, 0, 2);
                fill(ax, [t; flipud(t)], [mu + sd; flipud(mu - sd)], C{k}, 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'HandleVisibility', 'off');
                plot(ax, t, mu, '-', 'Color', C{k}, 'LineWidth', 2.4, 'DisplayName', [WB HAT '_s, ' N{k}]);
            end
            yline(ax, WS(iw), '-', 'Color', [0.85 0 0], 'LineWidth', 2.5, 'DisplayName', ['physical ' WB '_s']);
            xlim(ax, [0 t(end)]);  ylim(ax, [-1.6 0.8]);
            text(ax, 0.98, 0.92, sprintf('%s, %s', lab, WL{iw}), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', LFS + 2, 'FontWeight', 'bold');
            if row == 1 && iw == 1; legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold'); end
            if iw == 1; ylabel(ax, [WB '_s  [R]'], 'FontSize', FS, 'FontWeight', 'bold'); end
            if row == 2; xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(ax, 'off');
        end
    end
    out = fullfile(od, 'fig_mc_wsp_compare.png');  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end
