% STATUS: SCRATCH -- 2026-10-13, arm G = B + slot-7 information gate (formC_b ws_gate_info, ratio 1) against B, paired, 5 starts x plane / cell
%   x 10 seeds. Pre-registered (user 10-13): mid-section (6..2 R) |e_a| smaller than B (lock reference ~0.002, B ~0.027); near wall and hold
%   not worse; end w_s-hat within 0.05 of B; report when the gate opens (first step at which w_s-hat leaves its start value).
%   Figure: mid / near / hold |e_a| vs start for S8, B, G, the lock reference and c-only, plus the opening height.
function probe_1009_gate_validation()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  od = fullfile(root, 'test_results', 'temp_1009_ws');
    IN = [-2 -1 -0.5 0 0.5];  WL = {'plane', 'cell'};  CO = {'wallshift_p0', 'wallshift_m1.05'};  LK = {'wsonly_plane_m0d1_n98lock', 'wsonly_cell_m1d15_n98lock'};
    tg = @(w0) strrep(strrep(strrep(sprintf('%+g', w0), '+', 'p'), '-', 'm'), '.', 'd');
    f = figure('Position', [40 40 2000 1700], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  FS = 17;
    cS = [0 0.25 0.85];  cB = [0 0.6 0.25];  cG = [0.85 0.1 0.45];
    for iw = 1:2
        Cc = load(fullfile(od, [CO{iw} '.mat']));  [cm, cn, ch] = local_win(Cc);  L = load(fullfile(od, [LK{iw} '.mat']));  [lm, ln, lh] = local_win(L);
        fprintf('\n[%s] c-only mid/near/hold %.4f %.4f %.4f | lock ref %.4f %.4f %.4f\n', WL{iw}, mean(cm), mean(cn), mean(ch), mean(lm), mean(ln), mean(lh));
        fprintf('  start | B mid / near / hold     | G mid / near / hold     | G - B mid (t)          | G - B near (t)         | G - B hold (t)         | end w B, G      | open t (h_d) mean [min max]\n');
        M = nan(numel(IN), 10);
        for ii = 1:numel(IN)
            S = load(fullfile(od, sprintf('wsonly_%s_%s.mat', WL{iw}, tg(IN(ii)))));  B = load(fullfile(od, sprintf('wsonly_%s_%s_n98.mat', WL{iw}, tg(IN(ii)))));
            G = load(fullfile(od, sprintf('wsonly_%s_%s_n98gate.mat', WL{iw}, tg(IN(ii)))));
            [sm, sn, sh] = local_win(S);  [bm, bn, bh] = local_win(B);  [gm, gn, gh] = local_win(G);
            to = nan(1, size(G.w_hat, 2));  ho = to;  wd = G.wd;  wd(1) = wd(2);
            for q = 1:numel(to); i = find(G.w_hat(:, q) ~= G.w_hat(1, q), 1); if ~isempty(i); to(q) = G.t(i); ho(q) = wd(i); end; end
            fprintf('  %+4.1f | %.4f / %.4f / %.4f | %.4f / %.4f / %.4f | %s | %s | %s | %+.3f, %+.3f | %.3f s (%.1f R) [%.3f %.3f]\n', IN(ii), mean(bm), mean(bn), mean(bh), ...
                    mean(gm), mean(gn), mean(gh), local_pair(gm, bm), local_pair(gn, bn), local_pair(gh, bh), mean(B.w_hat(end, :)), mean(G.w_hat(end, :)), mean(to), mean(ho), min(to), max(to));
            M(ii, :) = [mean(sm), mean(bm), mean(gm), mean(sn), mean(bn), mean(gn), mean(sh), mean(bh), mean(gh), mean(ho)];
        end
        T = {'mid (6-2 R)', 'near wall (2-1.1 R)', 'hold'};  RF = [mean(lm), mean(ln), mean(lh); mean(cm), mean(cn), mean(ch)];
        for r = 1:3
            ax = nexttile(tl, (r - 1) * 2 + iw);  hold on; box on;
            plot(ax, IN, M(:, 3 * r - 2), 'o-', 'Color', cS, 'LineWidth', 2.2, 'MarkerSize', 9, 'DisplayName', 'S8 (numerator 1)');
            plot(ax, IN, M(:, 3 * r - 1), 's-', 'Color', cB, 'LineWidth', 2.2, 'MarkerSize', 9, 'DisplayName', 'B (9/8)');
            plot(ax, IN, M(:, 3 * r), 'd-', 'Color', cG, 'LineWidth', 2.2, 'MarkerSize', 9, 'DisplayName', 'G (9/8 + gate)');
            yline(ax, RF(1, r), ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 2, 'DisplayName', 'w_s locked (reference)');
            yline(ax, RF(2, r), '-', 'Color', 'k', 'LineWidth', 2, 'DisplayName', 'c-only');
            ylim(ax, [0 0.05]);  text(ax, 0.03, 0.88, [WL{iw} ': ' T{r}], 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');
            if r == 1; legend(ax, 'Location', 'northoutside', 'NumColumns', 3, 'FontSize', 12, 'FontWeight', 'bold'); end
            if iw == 1; ylabel(ax, ['|e_{' char(257) '}|'], 'FontSize', FS, 'FontWeight', 'bold'); end
            xlim(ax, [-2.3 0.8]);  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.8);  grid(ax, 'off');
        end
        ax = nexttile(tl, 6 + iw);  hold on; box on;
        plot(ax, IN, M(:, 10), 'd-', 'Color', cG, 'LineWidth', 2.2, 'MarkerSize', 9);
        ylim(ax, [0 12]);  xlim(ax, [-2.3 0.8]);  text(ax, 0.03, 0.88, [WL{iw} ': gate opens at h_d [R]'], 'Units', 'normalized', 'FontSize', FS, 'FontWeight', 'bold');
        xlabel(ax, ['start  w' char(772) char(770) '_s[0]  [R]'], 'FontSize', FS, 'FontWeight', 'bold');  set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.8);  grid(ax, 'off');
    end
    out = fullfile(od, 'fig_1009_gate_validation.png');  exportgraphics(f, out, 'Resolution', 130);  fprintf('figure: %s\n', out);
end

function [m, n, h] = local_win(S)
    t = S.t;  wd = S.wd;  wd(1) = wd(2);  e = abs(S.a_true - S.a_hat);
    m = mean(e(t <= 1 & wd <= 6 & wd > 2, :), 1);  n = mean(e(t <= 1 & wd <= 2, :), 1);  h = mean(e(t > 1, :), 1);
end

function s = local_pair(a, b)
    dlt = a - b;  m = mean(dlt);  se = std(dlt) / sqrt(numel(dlt));
    s = sprintf('%+.4f+-%.4f (%+5.1f)', m, se, m / se);
end
