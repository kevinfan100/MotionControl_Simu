% STATUS: SCRATCH -- 2026-10-13, full validation of arm B (motion_ctrl S8 with numerator 9/8: c pinned at -1/8) against S8 (numerator 1)
%   and c-only (S6), 5 starts x plane / cell x 10 seeds, same seeds (paired). Descent 22.2 -> 1.10 R in 1 s, hold 0.2 s.
%   Per seed: mean |e_a| over the descent, the near-wall window (h_d <= 2, t <= 1 s) and the hold; paired difference B - S8 and B - c-only
%   (mean +- SEM, t). Honesty of w_s-hat at the end: seed spread / mean sqrt(P_w). Figure: per wall, near-wall and descent |e_a| vs start.
function probe_1009_b_validation()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  od = fullfile(root, 'test_results', 'temp_1009_ws');
    IN = [-2 -1 -0.5 0 0.5];  WL = {'plane', 'cell'};  CO = {'wallshift_p0', 'wallshift_m1.05'};
    tg = @(w0) strrep(strrep(strrep(sprintf('%+g', w0), '+', 'p'), '-', 'm'), '.', 'd');
    f = figure('Position', [40 40 2000 800], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  FS = 20;
    for iw = 1:2
        C = load(fullfile(od, [CO{iw} '.mat']));  [cd_, cn_, ch_] = local_win(C);
        fprintf('\n[%s] c-only: descent %.4f  near %.4f  hold %.4f (seed mean of mean |e_a|)\n', WL{iw}, mean(cd_), mean(cn_), mean(ch_));
        fprintf('  start | S8: desc / near / hold    | B: desc / near / hold     | B - S8 near (t)     | B - c-only near (t)  | B - S8 desc (t)     | B end w_s-hat, spread/sqrtP\n');
        Pn = nan(numel(IN), 3);  Pd = nan(numel(IN), 3);
        for ii = 1:numel(IN)
            S = load(fullfile(od, sprintf('wsonly_%s_%s.mat', WL{iw}, tg(IN(ii)))));  B = load(fullfile(od, sprintf('wsonly_%s_%s_n98.mat', WL{iw}, tg(IN(ii)))));
            [sd, sn, sh] = local_win(S);  [bd, bn, bh] = local_win(B);
            hon = std(B.w_hat(end, :)) / mean(B.sP_w(end, :));
            fprintf('  %+4.1f | %.4f / %.4f / %.4f | %.4f / %.4f / %.4f | %s | %s | %s | %+.3f +- %.3f, %.1f\n', IN(ii), mean(sd), mean(sn), mean(sh), ...
                    mean(bd), mean(bn), mean(bh), local_pair(bn, sn), local_pair(bn, cn_), local_pair(bd, sd), mean(B.w_hat(end, :)), std(B.w_hat(end, :)), hon);
            Pn(ii, :) = [mean(sn), mean(bn), std(bn) / sqrt(numel(bn))];  Pd(ii, :) = [mean(sd), mean(bd), std(bd) / sqrt(numel(bd))];
        end
        ax = nexttile(tl);  hold on; box on;
        plot(ax, IN, Pn(:, 1), 'o-', 'Color', [0 0.25 0.85], 'LineWidth', 2.5, 'MarkerSize', 10, 'MarkerFaceColor', [0 0.25 0.85], 'DisplayName', 'S8 (numerator 1), near wall');
        errorbar(ax, IN, Pn(:, 2), Pn(:, 3), 's-', 'Color', [0 0.6 0.25], 'LineWidth', 2.5, 'MarkerSize', 10, 'MarkerFaceColor', [0 0.6 0.25], 'DisplayName', 'B (numerator 9/8), near wall');
        yline(ax, mean(cn_), '-', 'Color', 'k', 'LineWidth', 2.5, 'DisplayName', 'c-only, near wall');
        plot(ax, IN, Pd(:, 1), 'o--', 'Color', [0 0.25 0.85], 'LineWidth', 1.8, 'MarkerSize', 8, 'DisplayName', 'S8, whole descent');
        plot(ax, IN, Pd(:, 2), 's--', 'Color', [0 0.6 0.25], 'LineWidth', 1.8, 'MarkerSize', 8, 'DisplayName', 'B, whole descent');
        yline(ax, mean(cd_), '--', 'Color', 'k', 'LineWidth', 1.8, 'DisplayName', 'c-only, whole descent');
        xlabel(ax, ['start  w' char(772) char(770) '_s[0]  [R]'], 'FontSize', FS, 'FontWeight', 'bold');
        if iw == 1; ylabel(ax, ['mean |e_{' char(257) '}|  (10 seeds)'], 'FontSize', FS, 'FontWeight', 'bold'); end
        text(ax, 0.03, 0.93, WL{iw}, 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');
        ylim(ax, [0 0.06]);  xlim(ax, [-2.3 0.8]);  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.8);  grid(ax, 'off');
        legend(ax, 'Location', 'northoutside', 'NumColumns', 2, 'FontSize', 13, 'FontWeight', 'bold');
    end
    out = fullfile(od, 'fig_1009_b_validation.png');  exportgraphics(f, out, 'Resolution', 130);  fprintf('figure: %s\n', out);
end

function [d, n, h] = local_win(S)
    t = S.t;  wd = S.wd;  wd(1) = wd(2);  e = abs(S.a_true - S.a_hat);
    d = mean(e(t <= 1, :), 1);  n = mean(e(t <= 1 & wd <= 2, :), 1);  h = mean(e(t > 1, :), 1);
end

function s = local_pair(a, b)
    dlt = a - b;  m = mean(dlt);  se = std(dlt) / sqrt(numel(dlt));
    s = sprintf('%+.4f+-%.4f (%+5.1f)', m, se, m / se);
end
