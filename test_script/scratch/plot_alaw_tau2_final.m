% STATUS: SCRATCH -- 2026-10-19 (user): final values of run_alaw_tau2_long (a law, tau 2 s, no cap, gate off, plane a_d 0.07, 20 s).
%   Figure 1 (fig_alaw_tau2_long_s1.png): seed 1, columns w_s | a' state, a-bar true (red), estimate (blue), a_d (black dashed),
%     zoomed on 10-20 s and [0.05 0.09].
%   Figure 2 (fig_alaw_tau2_final.png): last 5 s means per seed (dots) and mean +- sd (bar) of a_hat - a_d, a_true - a_hat,
%     a_true - a_d, for both estimators; console also prints h - h*.
function plot_alaw_tau2_final(sfx, win)
    if nargin < 1; sfx = ''; end          % '' = 20 s runs, '_T40' = 40 s runs
    if nargin < 2; win = 5; end           % averaging window at the end [s]
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  cT = [0.85 0 0];  cE = [0 0.25 0.85];  cW = [0.95 0.5 0.05];
    FS = 20;  LFS = 15;  AXLW = 2.2;  AB = char(257);  HAT = char(770);  hs = 1.078;
    R = {load(fullfile(od, ['alaw_tau2_long_ws' sfx '.mat'])), load(fullfile(od, ['alaw_tau2_long_ap' sfx '.mat']))};  NM = {['w' char(772) '_s state'], [AB ''' state']};
    f = figure('Position', [40 40 1800 1150], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');   % 10-19 (user): approach | hold, a-bar only
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:2
        S = R{i};  t = S.t;  am = movmean(S.a_true(:, 1), round(1 / (t(2) - t(1))));
        for r = 1:2
            ax = nexttile(tl, (r - 1) * 2 + i);  hold on; box on;
            g1 = plot(ax, t, S.a_true(:, 1), '-', 'Color', cT, 'LineWidth', 2.5, 'DisplayName', [AB '_z']);
            g2 = plot(ax, t, S.a_hat(:, 1), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
            g3 = yline(ax, S.a_d, '--', 'Color', 'k', 'LineWidth', 2, 'DisplayName', [AB '_d']);
            if r == 1                                         % approach: the earliest view (0 - 0.3), first 10 s
                ylim(ax, [0 0.3]);  xlim(ax, [0 10]);
                lg = legend(ax, [g1 g2 g3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
                lg.Title.String = sprintf('plane wall, %s, %s_d = 0.07', NM{i}, AB);
            else                                              % hold: 10-20 s, with the 1 s mean of the true gain to show the drift
                g4 = plot(ax, t, am, '-', 'Color', [0.45 0 0], 'LineWidth', 3, 'DisplayName', [AB '_z, 1 s mean']);
                ylim(ax, [0.04 0.10]);  xlim(ax, [10 20]);  xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
                legend(ax, [g1 g4 g2 g3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            end
            if i == 1; ylabel(ax, [AB '_z'], 'FontSize', FS + 2, 'FontWeight', 'bold'); end
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax, 'off');
        end
    end
    exportgraphics(f, fullfile(od, ['fig_alaw_tau2_long' sfx '_s1.png']), 'Resolution', 150);  close(f);
    Q = {[AB HAT ' - ' AB '_d'], [AB '_{true} - ' AB HAT], [AB '_{true} - ' AB '_d']};
    f = figure('Position', [40 40 1700 650], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    V = cell(2, 3);
    for i = 1:2
        S = R{i};  L = S.t > S.t(end) - win;  ok = ~S.contact;
        ah = mean(S.a_hat(L, ok), 1);  at = mean(S.a_true(L, ok), 1);  hh = mean(S.h_true(L, ok), 1);
        V{i, 1} = ah - S.a_d;  V{i, 2} = at - ah;  V{i, 3} = at - S.a_d;
        fprintf('%-9s (contacts %d/%d) | a_hat - a_d %+.4f (%.4f) | a_true - a_hat %+.4f (%.4f) | a_true - a_d %+.4f (%.4f) | h - h* %+.4f (%.4f) R\n', ...
                NM{i}, nnz(S.contact), numel(S.contact), mean(V{i, 1}), std(V{i, 1}), mean(V{i, 2}), std(V{i, 2}), mean(V{i, 3}), std(V{i, 3}), mean(hh - hs), std(hh - hs));
    end
    for j = 1:3
        ax = nexttile(tl, j);  hold on; box on;
        for i = 1:2
            c = cW;  if i == 2; c = cE; end
            v = V{i, j};  bar(ax, i, mean(v), 0.5, 'FaceColor', c, 'FaceAlpha', 0.35, 'EdgeColor', c, 'LineWidth', 2, 'HandleVisibility', 'off');
            errorbar(ax, i, mean(v), std(v), 'Color', c, 'LineWidth', 2.5, 'CapSize', 14, 'HandleVisibility', 'off');
            plot(ax, i + 0.12 * linspace(-1, 1, numel(v)), v, 'o', 'Color', c, 'MarkerFaceColor', c, 'MarkerSize', 9, 'HandleVisibility', 'off');
        end
        yline(ax, 0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);
        set(ax, 'XTick', [1 2], 'XTickLabel', NM, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  xlim(ax, [0.4 2.6]);  grid(ax, 'off');
        ylim(ax, [-0.03 0.03]);  title(ax, '');  ylabel(ax, Q{j}, 'FontSize', FS, 'FontWeight', 'bold');
    end
    exportgraphics(f, fullfile(od, ['fig_alaw_tau2_final' sfx '.png']), 'Resolution', 150);  close(f);
    fprintf('figures: fig_alaw_tau2_long_s1.png, fig_alaw_tau2_final.png\n');
end
