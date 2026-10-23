function res = plot_formB_BC_bonly_complete(opts)
%PLOT_FORMB_BC_BONLY_COMPLETE  The complete Form B vs Form C comparison with
%   ONLY the shape constant free, 12 seeds, plus the init/prior construction.
%
%   STATUS: ACTIVE -- reads test_results/temp_formB_amp_stage4.mat (arms
%   len_bonly and amp). Feeds
%   reference/eq17_analysis/derivation/formB_amp_bonly_probe.tex.
%
%   Both arms: p locked at 1, w_bar_s locked at 1, only the constant free.
%   Both seeded at the SAME far-field anchor 9/8. Each declares the prior its
%   OWN writing honestly costs -- that asymmetry is not a knob, it is the
%   quantity formB_amp_functions.tex page 1 plots.
%
%   G1 formB_BC_init_prior.png     where every init number comes from
%   G2 formB_BC_ensemble.png       12-seed ensemble: gain error and constant
%   G3 formB_BC_metrics.png        per-seed paired metrics
%
%   Style: house rules (no grid, box on, legend northoutside horizontal, no
%   title, statistics to console, exportgraphics 150 dpi).

    if nargin < 1; opts = struct(); end
    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    L = load(fullfile(root, 'test_results', 'temp_formB_amp_stage4.mat'));
    r = L.res;
    iB = find(strcmp(r.arm_names, 'len_bonly'));
    iC = find(strcmp(r.arm_names, 'amp'));
    seeds = r.seeds; n_s = numel(seeds);

    pc = physical_constants();
    a_disp = (pc.Ts / (pc.gamma_N * pc.R)) * pc.R;
    ANCH = 9/8;  ENV = [1.90, 23.222];
    PRI_B = r.priors_len(1);  PRI_C = r.priors_amp(1);

    FS = 18; CB = [0 0.2 0.9]; CC = [0.45 0.55 0.95]; RED = [0.8 0 0];
    GREY = [0.55 0.55 0.55];

    % ================================================================== G1
    % Every init number, and the page-1 quantity it is read off.
    W = logspace(log10(1.05), log10(30), 3000).';
    c = zeros(size(W));
    for i = 1:numel(W); [~, c(i)] = calc_correction_functions(W(i), true); end
    bB = (c - 1) .* (W - 1);        % Form B  page-1 curve
    bC = W .* (c - 1) ./ c;         % Form C  page-1 curve

    f = figure('Position', [60 60 1150 640], 'Color', 'w', 'Visible', 'off');
    ax = axes(f); hold(ax, 'on');
    patch(ax, [ENV(1) ENV(2) ENV(2) ENV(1)], [0.95 0.95 1.25 1.25], ...
          [0.92 0.92 0.94], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    % prior bands around the shared anchor
    hpb = patch(ax, [min(W) max(W) max(W) min(W)], ...
          ANCH + PRI_B * [-1 -1 1 1], CB, 'EdgeColor', 'none', 'FaceAlpha', 0.30);
    hpc = patch(ax, [min(W) max(W) max(W) min(W)], ...
          ANCH + PRI_C * [-1 -1 1 1], CC, 'EdgeColor', 'none', 'FaceAlpha', 0.20);
    h1 = plot(ax, W, bB, '-', 'Color', CB, 'LineWidth', 2.4);
    h2 = plot(ax, W, bC, '-', 'Color', CC, 'LineWidth', 2.4);
    ha = plot(ax, [min(W) max(W)], ANCH * [1 1], '--', 'Color', RED, 'LineWidth', 2.0);
    set(ax, 'XScale', 'log');
    XT = [1 1.5 2 3 5 10 20 30];
    set(ax, 'XTick', XT, 'XTickLabel', arrayfun(@num2str, XT, 'UniformOutput', false), ...
            'XMinorTick', 'off');
    ylim(ax, [0.95 1.25]); xlim(ax, [min(W) max(W)]);
    xlabel(ax, '$\bar{w} = h/R$', 'Interpreter', 'latex', 'FontSize', FS);
    ylabel(ax, 'constant the truth demands', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2 ha hpb hpc], {'Form B  b(w)', 'Form C  b(w)', ...
            'shared init 9/8 (far-field anchor)', ...
            sprintf('Form B prior \\pm%.4f', PRI_B), ...
            sprintf('Form C prior \\pm%.4f', PRI_C)}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', FS - 7, 'NumColumns', 3);
    set(ax, 'FontSize', FS - 4, 'FontWeight', 'bold', 'LineWidth', 1.5, ...
            'Box', 'on', 'Layer', 'top');
    grid(ax, 'off');
    out = fullfile(fig_dir, 'formB_BC_init_prior.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('G1 -> %s\n', out);

    % ================================================================== G2
    t = r.runs{iB, 1}.t;  N = numel(t);
    EB = nan(N, n_s); EC = nan(N, n_s); BB = nan(N, n_s); BC = nan(N, n_s);
    for q = 1:n_s
        a = r.runs{iB, q}; b = r.runs{iC, q};
        EB(:, q) = 100 * (a.a_hat - a.a_true) ./ a.a_true;
        EC(:, q) = 100 * (b.a_hat - b.a_true) ./ b.a_true;
        BB(:, q) = a.b_hat;  BC(:, q) = b.b_hat;
    end
    w = r.runs{iC, 1}.h_bar_d;  ok = w > 1.01;
    beB = nan(N, 1); beC = nan(N, 1);
    for i = 1:N
        if ok(i)
            [~, ci] = calc_correction_functions(w(i), true);
            beB(i) = (ci - 1) * (w(i) - 1);
            beC(i) = w(i) * (ci - 1) / ci;
        end
    end
    tb = [0.5 1.5 3.5];
    f = figure('Position', [40 40 1250 820], 'Color', 'w', 'Visible', 'off');
    ax = subplot(2, 1, 1); hold(ax, 'on');
    hB = local_band(ax, t, EB, CB);  hC = local_band(ax, t, EC, CC);
    plot(ax, t, zeros(size(t)), '-', 'Color', RED, 'LineWidth', 1.6, 'HandleVisibility', 'off');
    plot(ax, t,  2 * ones(size(t)), 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    plot(ax, t, -2 * ones(size(t)), 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    ylim(ax, [-10 6]); ylabel(ax, 'a_z error  [%]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hB hC], {sprintf('Form B (length), %d seeds', n_s), ...
                     sprintf('Form C (amplitude), %d seeds', n_s)}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 5);
    local_ax(ax, FS, tb, t);
    ax = subplot(2, 1, 2); hold(ax, 'on');
    h3 = plot(ax, t, beB, '--', 'Color', CB * 0.6, 'LineWidth', 1.8);
    h4 = plot(ax, t, beC, '--', 'Color', RED, 'LineWidth', 1.8);
    local_band(ax, t, BB, CB);  local_band(ax, t, BC, CC);
    ha = plot(ax, [t(1) t(end)], ANCH * [1 1], 'k:', 'LineWidth', 1.6);
    ylim(ax, [1.00 1.17]);
    ylabel(ax, 'constant  [-]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h3 h4 ha], {'demanded by Form B', 'demanded by Form C', 'shared init 9/8'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 5);
    local_ax(ax, FS, tb, t);
    out = fullfile(fig_dir, 'formB_BC_ensemble.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('G2 -> %s\n', out);

    % ================================================================== G3
    M = {'descent peak |e_a|', 'oscillation RMS e_a', 'final hold mean e_a'};
    f = figure('Position', [40 40 1250 420], 'Color', 'w', 'Visible', 'off');
    for k = 1:3
        ax = subplot(1, 3, k); hold(ax, 'on');
        vB = squeeze(r.tab(iB, :, 4 + k)); vC = squeeze(r.tab(iC, :, 4 + k));
        for q = 1:n_s
            plot(ax, [1 2], [vB(q) vC(q)], '-', 'Color', [0.8 0.8 0.85], 'LineWidth', 1.0);
        end
        plot(ax, ones(1, n_s), vB, 'o', 'Color', CB, 'MarkerFaceColor', CB, 'MarkerSize', 7);
        plot(ax, 2 * ones(1, n_s), vC, 'o', 'Color', CC, 'MarkerFaceColor', CC, 'MarkerSize', 7);
        plot(ax, [0.82 1.18], mean(vB) * [1 1], '-', 'Color', CB, 'LineWidth', 3);
        plot(ax, [1.82 2.18], mean(vC) * [1 1], '-', 'Color', CC, 'LineWidth', 3);
        if k == 3; plot(ax, [0.7 2.3], [0 0], '-', 'Color', RED, 'LineWidth', 1.4); end
        xlim(ax, [0.7 2.3]); set(ax, 'XTick', [1 2], 'XTickLabel', {'B', 'C'});
        ylabel(ax, [M{k} '  [%]'], 'FontSize', FS - 4, 'FontWeight', 'bold');
        text(ax, 0.5, 0.94, sprintf('%.2f -> %.2f', mean(vB), mean(vC)), ...
             'Units', 'normalized', 'HorizontalAlignment', 'center', ...
             'FontSize', FS - 6, 'FontWeight', 'bold');
        set(ax, 'FontSize', FS - 5, 'FontWeight', 'bold', 'LineWidth', 1.3, 'Box', 'on');
        grid(ax, 'off');
    end
    out = fullfile(fig_dir, 'formB_BC_metrics.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('G3 -> %s\n', out);

    % ---- console -------------------------------------------------------
    fprintf('\n=== Form B vs Form C, ONLY the constant free, %d seeds ===\n', n_s);
    fprintf('init (both arms): b0 = 9/8 (far-field anchor), p locked 1, w_s locked 1\n');
    fprintf('prior sqrt(P[0]) : B %.4f   C %.4f   (each its own honest sup on [%.2f, %.2f])\n', ...
            PRI_B, PRI_C, ENV(1), ENV(2));
    fprintf('shape floor      : B %.5f  C %.5f\n', r.priors_len(3), r.priors_amp(2));
    nm = {'desc peak', 'osc RMS', 'hold mean'};
    for k = 1:3
        vB = squeeze(r.tab(iB, :, 4 + k)); vC = squeeze(r.tab(iC, :, 4 + k));
        d = vC - vB;
        fprintf('%-10s  B %6.3f +- %5.3f   C %6.3f +- %5.3f   paired diff %+6.3f  t %+5.2f\n', ...
                nm{k}, mean(vB), std(vB), mean(vC), std(vC), mean(d), ...
                mean(d) / (std(d) / sqrt(n_s)));
    end
    bfB = squeeze(r.tab(iB, :, 2)); bfC = squeeze(r.tab(iC, :, 2));
    fprintf('constant final    B %.4f +- %.4f   C %.4f +- %.4f\n', ...
            mean(bfB), std(bfB), mean(bfC), std(bfC));
    fprintf('P shrink          B %.1f%%   C %.1f%%\n', ...
            100 * (1 - mean(squeeze(r.tab(iB,:,4))) / mean(squeeze(r.tab(iB,:,3)))), ...
            100 * (1 - mean(squeeze(r.tab(iC,:,4))) / mean(squeeze(r.tab(iC,:,3)))));
    fprintf('final absolute sqrt(P)   B %.4f   C %.4f   (B is %.1fx tighter)\n', ...
            mean(squeeze(r.tab(iB,:,4))), mean(squeeze(r.tab(iC,:,4))), ...
            mean(squeeze(r.tab(iC,:,4))) / mean(squeeze(r.tab(iB,:,4))));
    res = struct('priorB', PRI_B, 'priorC', PRI_C);
end

% --------------------------------------------------------------------------
function h = local_band(ax, t, X, col)
    lo = prctile(X, 10, 2); hi = prctile(X, 90, 2); md = median(X, 2);
    patch(ax, [t; flipud(t)], [lo; flipud(hi)], col, 'EdgeColor', 'none', ...
          'FaceAlpha', 0.25, 'HandleVisibility', 'off');
    h = plot(ax, t, md, '-', 'Color', col, 'LineWidth', 2.2);
end

function local_ax(ax, FS, tb, t)
    yl = ylim(ax);
    for x = tb
        plot(ax, [x x], yl, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, ...
             'HandleVisibility', 'off');
    end
    ylim(ax, yl); xlim(ax, [t(1) t(end)]);
    set(ax, 'FontSize', FS - 4, 'FontWeight', 'bold', 'LineWidth', 1.3, 'Box', 'on');
    grid(ax, 'off');
end
