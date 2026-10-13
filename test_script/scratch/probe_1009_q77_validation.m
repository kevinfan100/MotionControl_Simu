% STATUS: SCRATCH -- 2026-10-13, arm D = the same with sqrt(P_fix) 0.245 (the first, mis-scaled q77 run, kept). arm C = B + slot-7 ratio Q (formC_b q77_adapt_P0 = 0.06, sqrt(P_fix) from half the 0.12 R variation of
%   the needed w_s under numerator 9/8) against B, paired (same seeds), 5 starts x plane / cell x 10 seeds. Pre-registered criteria
%   (user 10-13): near-wall |e_a| smaller than B, honesty spread / sqrt(P_w) near 1, hold not worse. Figure: near-wall and hold |e_a| vs start
%   for S8, B, C and c-only, plus the honesty ratio of B and C.
function probe_1009_q77_validation()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  od = fullfile(root, 'test_results', 'temp_1009_ws');
    IN = [-2 -1 -0.5 0 0.5];  WL = {'plane', 'cell'};  CO = {'wallshift_p0', 'wallshift_m1.05'};
    tg = @(w0) strrep(strrep(strrep(sprintf('%+g', w0), '+', 'p'), '-', 'm'), '.', 'd');
    f = figure('Position', [40 40 2000 1300], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  FS = 18;
    cS = [0 0.25 0.85];  cB = [0 0.6 0.25];  cC = [0.85 0.45 0];
    for iw = 1:2
        Cc = load(fullfile(od, [CO{iw} '.mat']));  [~, cn, ch] = local_win(Cc);
        fprintf('\n[%s] c-only near %.4f hold %.4f\n  start | B near / hold / desc  | C near / hold / desc  | C - B near (t)        | C - B hold (t)        | C - B desc (t)        | honesty B, C | end w_s-hat C\n', WL{iw}, mean(cn), mean(ch));
        M = nan(numel(IN), 8);  MD = nan(numel(IN), 3);
        for ii = 1:numel(IN)
            S = load(fullfile(od, sprintf('wsonly_%s_%s.mat', WL{iw}, tg(IN(ii)))));
            B = load(fullfile(od, sprintf('wsonly_%s_%s_n98.mat', WL{iw}, tg(IN(ii)))));
            C = load(fullfile(od, sprintf('wsonly_%s_%s_n98q77.mat', WL{iw}, tg(IN(ii)))));
            D = load(fullfile(od, sprintf('wsonly_%s_%s_n98q245.mat', WL{iw}, tg(IN(ii)))));  [dd, dn, dh] = local_win(D);  hD = std(D.w_hat(end, :)) / mean(D.sP_w(end, :));
            [~, sn, sh] = local_win(S);  [bd, bn, bh] = local_win(B);  [cd, cnn, chh] = local_win(C);
            hB = std(B.w_hat(end, :)) / mean(B.sP_w(end, :));  hC = std(C.w_hat(end, :)) / mean(C.sP_w(end, :));
            fprintf('  %+4.1f | %.4f / %.4f / %.4f | %.4f / %.4f / %.4f | %s | %s | %s | %.1f, %.1f | %+.3f +- %.3f (sqrtP %.3f)\n', IN(ii), ...
                    mean(bn), mean(bh), mean(bd), mean(cnn), mean(chh), mean(cd), local_pair(cnn, bn), local_pair(chh, bh), local_pair(cd, bd), ...
                    hB, hC, mean(C.w_hat(end, :)), std(C.w_hat(end, :)), mean(C.sP_w(end, :)));
            fprintf('        D (sqrtPfix 0.245): near %.4f hold %.4f desc %.4f | D - B near %s | hold %s | honesty %.1f\n', mean(dn), mean(dh), mean(dd), local_pair(dn, bn), local_pair(dh, bh), hD);
            M(ii, :) = [mean(sn), mean(bn), mean(cnn), mean(sh), mean(bh), mean(chh), hB, hC];  MD(ii, :) = [mean(dn), mean(dh), hD];
        end
        ax = nexttile(tl, iw);  hold on; box on;
        plot(ax, IN, M(:, 1), 'o-', 'Color', cS, 'LineWidth', 2.2, 'MarkerSize', 9, 'DisplayName', 'S8 (numerator 1)');
        plot(ax, IN, M(:, 2), 's-', 'Color', cB, 'LineWidth', 2.2, 'MarkerSize', 9, 'DisplayName', 'B (9/8)');
        plot(ax, IN, M(:, 3), 'd-', 'Color', cC, 'LineWidth', 2.2, 'MarkerSize', 9, 'DisplayName', ['C: ' char(8730) 'P_{fix} 0.06']);
        plot(ax, IN, MD(:, 1), '^-', 'Color', [0.6 0.2 0.7], 'LineWidth', 2.2, 'MarkerSize', 9, 'DisplayName', ['D: ' char(8730) 'P_{fix} 0.245']);
        yline(ax, mean(cn), '-', 'Color', 'k', 'LineWidth', 2.2, 'DisplayName', 'c-only');
        ylim(ax, [0 0.06]);  text(ax, 0.03, 0.9, [WL{iw} ': near wall'], 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');
        legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 13, 'FontWeight', 'bold');
        ax2 = nexttile(tl, 2 + iw);  hold on; box on;
        plot(ax2, IN, M(:, 4), 'o-', 'Color', cS, 'LineWidth', 2.2, 'MarkerSize', 9);  plot(ax2, IN, M(:, 5), 's-', 'Color', cB, 'LineWidth', 2.2, 'MarkerSize', 9);
        plot(ax2, IN, M(:, 6), 'd-', 'Color', cC, 'LineWidth', 2.2, 'MarkerSize', 9);  plot(ax2, IN, MD(:, 2), '^-', 'Color', [0.6 0.2 0.7], 'LineWidth', 2.2, 'MarkerSize', 9);  yline(ax2, mean(ch), '-', 'Color', 'k', 'LineWidth', 2.2);
        ylim(ax2, [0 0.05]);  text(ax2, 0.03, 0.9, [WL{iw} ': hold'], 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');
        ax3 = nexttile(tl, 4 + iw);  hold on; box on;
        plot(ax3, IN, M(:, 7), 's-', 'Color', cB, 'LineWidth', 2.2, 'MarkerSize', 9);  plot(ax3, IN, M(:, 8), 'd-', 'Color', cC, 'LineWidth', 2.2, 'MarkerSize', 9);  plot(ax3, IN, MD(:, 3), '^-', 'Color', [0.6 0.2 0.7], 'LineWidth', 2.2, 'MarkerSize', 9);
        yline(ax3, 1, '--', 'Color', 'k', 'LineWidth', 1.5);  set(ax3, 'YScale', 'log');  ylim(ax3, [0.3 30]);
        text(ax3, 0.03, 0.9, [WL{iw} ': honesty  spread / ' char(8730) 'P_w'], 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');
        xlabel(ax3, ['start  w' char(772) char(770) '_s[0]  [R]'], 'FontSize', FS, 'FontWeight', 'bold');
        for a = [ax ax2 ax3]; xlim(a, [-2.3 0.8]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.8);  grid(a, 'off'); end
        if iw == 1; ylabel(ax, ['|e_{' char(257) '}|'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax2, ['|e_{' char(257) '}|'], 'FontSize', FS, 'FontWeight', 'bold'); end
    end
    out = fullfile(od, 'fig_1009_q77_validation.png');  exportgraphics(f, out, 'Resolution', 130);  fprintf('figure: %s\n', out);
end

function [d, n, h] = local_win(S)
    t = S.t;  wd = S.wd;  wd(1) = wd(2);  e = abs(S.a_true - S.a_hat);
    d = mean(e(t <= 1, :), 1);  n = mean(e(t <= 1 & wd <= 2, :), 1);  h = mean(e(t > 1, :), 1);
end

function s = local_pair(a, b)
    dlt = a - b;  m = mean(dlt);  se = std(dlt) / sqrt(numel(dlt));
    s = sprintf('%+.4f+-%.4f (%+5.1f)', m, se, m / se);
end
