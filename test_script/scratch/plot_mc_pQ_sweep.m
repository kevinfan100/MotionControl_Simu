% STATUS: SCRATCH -- 2026-10-15 (user): Q on the exponent p of a' = 1/(h - w_s)^p (q55_adapt 'ratio', Q_pp = P_fix^2 I_k), constant-velocity
%   descent 22.2 -> 1.10 R in 1 s + 0.2 s hold, 5 seeds (run_mc_apslot_compare arms f4fc, f4fcQNN). Left: w_s-hat at the end (mean +- sd) vs
%   sqrt P_fix with the physical wall (red); right: p-hat at the end vs sqrt P_fix. Black = plane, blue = cell. sqrt P_fix = 0 means Q = 0.
function plot_mc_pQ_sweep()
    od = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'test_results', 'temp_1009_ws');
    A = {'f4fc', 'f4fcQ05', 'f4fcQ10', 'f4fcQ20', 'f4fcQ40'};  X = [0 0.05 0.1 0.2 0.4];  WL = {'plane', 'cell'};  WS = [0 -1.05];
    C = {[0.15 0.15 0.15], [0 0.25 0.85]};  FS = 18;  LFS = 14;  WB = ['w' char(772)];  HAT = char(770);
    f = figure('Position', [40 40 1800 650], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  a1 = nexttile(tl);  hold on; box on;  a2 = nexttile(tl);  hold on; box on;
    for iw = 1:2
        [mw, sw, mp, sp] = deal(zeros(size(X)));
        for ia = 1:numel(A)
            S = load(fullfile(od, sprintf('apcmp_%s_%s_desc.mat', A{ia}, WL{iw})));
            mw(ia) = mean(S.w_hat(end, :));  sw(ia) = std(S.w_hat(end, :));  mp(ia) = mean(S.p_hat(end, :));  sp(ia) = std(S.p_hat(end, :));
        end
        errorbar(a1, X, mw, sw, 'o-', 'Color', C{iw}, 'LineWidth', 2.2, 'MarkerFaceColor', C{iw}, 'DisplayName', [WB HAT '_s  ' WL{iw}]);
        yline(a1, WS(iw), '-', 'Color', [0.85 0 0], 'LineWidth', 2, 'HandleVisibility', 'off');
        errorbar(a2, X, mp, sp, 'o-', 'Color', C{iw}, 'LineWidth', 2.2, 'MarkerFaceColor', C{iw}, 'DisplayName', ['p' HAT '  ' WL{iw}]);
    end
    xlabel(a1, ['sqrt P_{fix} of p'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a1, [WB HAT '_s at the end  [R]'], 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(a2, ['sqrt P_{fix} of p'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a2, ['p' HAT ' at the end'], 'FontSize', FS, 'FontWeight', 'bold');
    for a = [a1 a2]; xlim(a, [-0.02 0.42]);  legend(a, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(a, 'off'); end
    ylim(a1, [-1.3 0.2]);
    out = fullfile(od, 'fig_mc_pQ_sweep.png');  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end
