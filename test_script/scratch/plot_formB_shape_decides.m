function res = plot_formB_shape_decides(opts)
%PLOT_FORMB_SHAPE_DECIDES  The one relation that explains the whole comparison.
%
%   STATUS: ACTIVE -- offline (no filter, no simulation). Feeds
%   reference/eq17_analysis/derivation/formB_amp_bonly_probe.tex.
%
%   The apparent paradox -- the writing whose constant moves LESS has the
%   MORE accurate gain -- dissolves once the right pair of quantities is
%   plotted. Substituting the constant the truth demands at the current
%   height returns the truth exactly, in BOTH writings:
%       Form B  b = (c-1)(w-1)    -> a_bar = 1/c
%       Form C  b = w(c-1)/c      -> a_bar = 1/c
%   So the gain error is set by how far the CARRIED constant is from the
%   DEMANDED one. A filter carrying a constant with Q = 0 cannot follow a
%   target that moves within a cycle -- measured: zero-lag correlation
%   +0.12 +- 0.39 (B) and +0.10 +- 0.39 (C), i.e. neither writing tracks.
%
%   What therefore decides the gain is not the estimator at all, but how
%   FLAT the demanded constant is. This plots exactly that:
%       x  swing of b_eff over the envelope       (is the constant constant?)
%       y  sup relative gain error of the BEST single frozen constant
%          (the floor no estimator can beat, anchor-free by construction)
%
%   Boundaries: the published plane (perpendicular and parallel), the exact
%   two-sphere cell at several radii, and the b = 0.13 constructed curve the
%   2026-08-06 identifiability arm used.
%
%   Figure -> derivation/figures/formB_shape_decides.png

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'save'); opts.save = true; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    W = linspace(1.90, 23.222, 2000).';
    BND = { 'plane perp',   @() local_c(W, 'perp'); ...
            'plane para',   @() local_c(W, 'para'); ...
            'cell lam=20', @() local_c(W, 20); ...
            'cell lam=10', @() local_c(W, 10); ...
            'cell lam=5',  @() local_c(W, 5); ...
            'b=0.13 curve',    @() 1 + 0.13 ./ (W - 1) };
    n_b = size(BND, 1);

    res = struct('names', {BND(:,1).'}, 'W', W);
    res.tab = nan(n_b, 6);   % swingB swingC floorB floorC bstarB bstarC

    fprintf('=== 形狀決定一切：b_eff 平坦度 vs 最佳凍結常數的誤差地板 ===\n');
    fprintf('包絡 w_bar in [%.2f, %.2f]，完全不含錨（每個寫法各自取 minimax 常數）\n\n', ...
            W(1), W(end));
    fprintf('%-16s  %-22s  %-22s\n', '邊界', 'Form B  擺幅 / 地板', 'Form C  擺幅 / 地板');
    for k = 1:n_b
        c = BND{k, 2}();  a = 1 ./ c;
        bB = (c - 1) .* (W - 1);
        bC = W .* (c - 1) ./ c;
        fB = @(b) max(abs((1 - b ./ ((W - 1) + b)) ./ a - 1));
        fC = @(b) max(abs((1 - b ./ W) ./ a - 1));
        xB = fminbnd(fB, 0.02, 3);  xC = fminbnd(fC, 0.02, 3);
        res.tab(k, :) = [max(bB) - min(bB), max(bC) - min(bC), ...
                         100 * fB(xB), 100 * fC(xC), xB, xC];
        fprintf('%-16s  %8.4f / %7.3f %%    %8.4f / %7.3f %%\n', BND{k,1}, ...
                res.tab(k,1), res.tab(k,3), res.tab(k,2), res.tab(k,4));
    end

    x = [res.tab(:,1); res.tab(:,2)];
    y = [res.tab(:,3); res.tab(:,4)];
    keep = x > 1e-6;      % drop the degenerate point (truth IS that family:
                          % swing is exactly 0 and log10 is -Inf)
    rho = corr(log10(x(keep)), log10(y(keep)));
    rs  = corr(x(keep), y(keep), 'type', 'Spearman');
    [xs, ii] = sort(x(keep)); ys = y(keep); ys = ys(ii);
    fprintf('\n%d 點（去掉擺幅精確為 0 的退化點），兩個寫法混在一起：\n', sum(keep));
    fprintf('  log-log rho = %.4f   Spearman = %.4f   非單調步數 %d / %d\n', ...
            rho, rs, sum(diff(ys) < 0), numel(ys) - 1);
    fprintf('=> 擺幅是主導因子但不是唯一因子；「哪個寫法」這個標籤本身沒有資訊\n');

    if ~opts.save; return; end

    % ---- figure ---------------------------------------------------------
    FS = 18; CB = [0 0.2 0.9]; CC = [0.45 0.55 0.95];
    f = figure('Position', [60 60 1050 700], 'Color', 'w', 'Visible', 'off');
    ax = axes(f); hold(ax, 'on');
    % trend through all 12 points
    p = polyfit(log10(x(keep)), log10(y(keep)), 1);
    xx = logspace(log10(min(x(keep)) * 0.7), log10(max(x(keep)) * 1.4), 50);
    plot(ax, xx, 10.^polyval(p, log10(xx)), '-', 'Color', [0.75 0.75 0.78], ...
         'LineWidth', 6, 'HandleVisibility', 'off');
    kb = res.tab(:,1) > 1e-6;
    h1 = plot(ax, res.tab(kb,1), res.tab(kb,3), 'o', 'Color', CB, ...
              'MarkerFaceColor', CB, 'MarkerSize', 11);
    h2 = plot(ax, res.tab(:,2), res.tab(:,4), 's', 'Color', CC, ...
              'MarkerFaceColor', CC, 'MarkerSize', 11);
    for k = 1:n_b
        if res.tab(k,1) > 1e-6
            text(ax, res.tab(k,1) * 1.10, res.tab(k,3) * 1.10, BND{k,1}, ...
                 'FontSize', FS - 7, 'Color', CB * 0.8, 'Interpreter', 'none');
        end
        text(ax, res.tab(k,2) * 1.10, res.tab(k,4) * 0.88, BND{k,1}, ...
             'FontSize', FS - 7, 'Color', CC * 0.7, 'Interpreter', 'none');
    end
    set(ax, 'XScale', 'log', 'YScale', 'log');
    xlabel(ax, 'b_{eff} 在包絡上的擺幅   (常數到底有多「常數」)', ...
           'FontSize', FS - 2, 'FontWeight', 'bold');
    ylabel(ax, '最佳凍結常數的 sup 增益誤差  [%]', 'FontSize', FS - 2, 'FontWeight', 'bold');
    legend([h1 h2], {'Form B (length)', 'Form C (amplitude)'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 4);
    xlim(ax, [min(x(keep)) * 0.5, max(x(keep)) * 2.6]);
    ylim(ax, [0.05 6]);
    set(ax, 'FontSize', FS - 4, 'FontWeight', 'bold', 'LineWidth', 1.5, 'Box', 'on');
    grid(ax, 'off');
    out = fullfile(fig_dir, 'formB_shape_decides.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('wrote %s\n', out);
end

% --------------------------------------------------------------------------
function c = local_c(W, which)
    c = zeros(size(W));
    for i = 1:numel(W)
        if ischar(which) && strcmp(which, 'perp')
            [~, c(i)] = calc_correction_functions(W(i), true);
        elseif ischar(which) && strcmp(which, 'para')
            c(i) = calc_correction_functions(W(i), true);
        else
            c(i) = build_truth_two_sphere(W(i), which);
        end
    end
end
