function res = analyze_formB_BC_curvature(opts)
%ANALYZE_FORMB_BC_CURVATURE  Stage O: does the boundary's curvature change
%   which writing is better, when the controller stays blind?
%
%   STATUS: ACTIVE -- offline only (no filter, no simulation). Feeds
%   reference/eq17_analysis/derivation/formB_amp_bonly_probe.tex.
%
%   Everything shown so far was measured on the PLANE, which is the boundary
%   Form B's anchors were derived for. That is circular. This sweeps the
%   boundary away from the plane and asks whether Form B's margin survives.
%
%   BOUNDARY. A sphere of radius Rc, exact two-sphere solution (Jeffrey &
%   Onishi 1984, build_truth_two_sphere), one parameter lam = Rc/R. It is
%   simple (one number), exact, physical, and -- the property that makes the
%   test fair -- it belongs to NEITHER writing's family. A synthetic knob
%   c = 1 + A/(h-1)^q would NOT be fair: at q = 1 that IS Form B exactly.
%
%   CONTROLLER STAYS BLIND: both writings keep the plane anchor 9/8 and the
%   plane-derived prior. That is the honest "c(h) is unknown" case.
%
%   Three quantities per boundary, none of which needs a simulation:
%     (i)   sup|b_eff - 9/8|      the prior each writing would HONESTLY need
%     (ii)  swing of b_eff        how far from constant the target is
%     (iii) sup relative gain error of the ANCHORED law -- the blind cost
%
%   Figure -> derivation/figures/formB_BC_curvature.png

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'lams'); opts.lams = [Inf 200 100 50 30 20 10 5]; end
    if ~isfield(opts, 'save'); opts.save = true; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    ANCH = 9/8;
    W = linspace(1.90, 23.222, 1200).';      % the canonical envelope
    lams = opts.lams(:).';  n_l = numel(lams);

    res = struct('lams', lams, 'W', W);
    res.tab = nan(n_l, 7);   % priorB priorC swingB swingC errB errC dc_max

    fprintf('=== Stage O: curvature sweep, controller BLIND (plane anchor 9/8) ===\n');
    fprintf('envelope w_bar in [%.2f, %.2f]; boundary = sphere of radius lam*R\n\n', W(1), W(end));
    fprintf(['  lam     max|c/c_plane-1|   prior needed        b_eff swing        ' ...
             'blind gain error\n']);
    fprintf(['                              B       C           B       C          ' ...
             ' B        C\n']);

    cpl = local_c(W, Inf);
    for k = 1:n_l
        c = local_c(W, lams(k));
        a_true = 1 ./ c;
        bB = (c - 1) .* (W - 1);
        bC = W .* (c - 1) ./ c;
        aB = 1 - ANCH ./ ((W - 1) + ANCH);       % anchored Form B
        aC = 1 - ANCH ./ W;                      % anchored Form C
        eB = max(abs(aB ./ a_true - 1));
        eC = max(abs(aC ./ a_true - 1));
        res.tab(k, :) = [max(abs(bB - ANCH)), max(abs(bC - ANCH)), ...
                         max(bB) - min(bB),   max(bC) - min(bC), ...
                         100 * eB, 100 * eC, 100 * max(abs(c ./ cpl - 1))];
        if isinf(lams(k)); nm = ' plane'; else; nm = sprintf('%6g', lams(k)); end
        fprintf('%s   %8.2f %%        %.4f  %.4f     %.4f  %.4f     %7.2f %% %7.2f %%\n', ...
                nm, res.tab(k, 7), res.tab(k, 1), res.tab(k, 2), ...
                res.tab(k, 3), res.tab(k, 4), res.tab(k, 5), res.tab(k, 6));
    end

    fprintf('\nB/C ratios (below 1 = Form B still ahead):\n');
    fprintf('  lam        prior   swing   blind gain error\n');
    for k = 1:n_l
        if isinf(lams(k)); nm = ' plane'; else; nm = sprintf('%6g', lams(k)); end
        fprintf('%s      %.3f   %.3f   %.3f\n', nm, ...
                res.tab(k,1)/res.tab(k,2), res.tab(k,3)/res.tab(k,4), ...
                res.tab(k,5)/res.tab(k,6));
    end

    if ~opts.save; return; end

    % ---- figure ---------------------------------------------------------
    FS = 17; CB = [0 0.2 0.9]; CC = [0.45 0.55 0.95]; GREY = [0.5 0.5 0.5];
    x = 1 ./ lams;  x(isinf(lams)) = 0;        % curvature 1/lam, 0 = plane
    f = figure('Position', [40 40 1250 800], 'Color', 'w', 'Visible', 'off');

    ax = subplot(2, 2, 1); hold(ax, 'on');
    h1 = plot(ax, x, res.tab(:,1), '-o', 'Color', CB, 'LineWidth', 2.2, ...
              'MarkerFaceColor', CB, 'MarkerSize', 7);
    h2 = plot(ax, x, res.tab(:,2), '-o', 'Color', CC, 'LineWidth', 2.2, ...
              'MarkerFaceColor', CC, 'MarkerSize', 7);
    set(ax, 'YScale', 'log');
    ylabel(ax, 'prior needed  [-]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2], {'Form B (length)', 'Form C (amplitude)'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 5);
    local_ax(ax, FS);

    ax = subplot(2, 2, 2); hold(ax, 'on');
    plot(ax, x, res.tab(:,5), '-o', 'Color', CB, 'LineWidth', 2.2, ...
         'MarkerFaceColor', CB, 'MarkerSize', 7);
    plot(ax, x, res.tab(:,6), '-o', 'Color', CC, 'LineWidth', 2.2, ...
         'MarkerFaceColor', CC, 'MarkerSize', 7);
    set(ax, 'YScale', 'log');
    ylabel(ax, 'blind gain error  [%]', 'FontSize', FS, 'FontWeight', 'bold');
    local_ax(ax, FS);

    ax = subplot(2, 2, 3); hold(ax, 'on');
    plot(ax, x, res.tab(:,1) ./ res.tab(:,2), '-o', 'Color', [0.2 0.2 0.2], ...
         'LineWidth', 2.2, 'MarkerFaceColor', [0.7 0.7 0.7], 'MarkerSize', 7);
    plot(ax, x, res.tab(:,5) ./ res.tab(:,6), '-s', 'Color', [0.8 0 0], ...
         'LineWidth', 2.2, 'MarkerFaceColor', [0.97 0.8 0.8], 'MarkerSize', 8);
    plot(ax, [min(x) max(x)], [1 1], '--', 'Color', GREY, 'LineWidth', 1.6);
    ylim(ax, [0 1.3]);
    ylabel(ax, 'B / C   (<1 = B ahead)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax, 'curvature  1/\lambda   (0 = plane)', 'FontSize', FS, 'FontWeight', 'bold');
    legend({'prior ratio', 'blind gain error ratio'}, 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', FS - 5);
    local_ax(ax, FS);

    ax = subplot(2, 2, 4); hold(ax, 'on');
    SHOW = [Inf 50 20 10];
    sty = {'-', '--', '-.', ':'};
    hh = gobjects(1, numel(SHOW));
    for j = 1:numel(SHOW)
        c = local_c(W, SHOW(j));
        hh(j) = plot(ax, W, (c - 1) .* (W - 1), sty{j}, 'Color', CB, 'LineWidth', 2.0);
        plot(ax, W, W .* (c - 1) ./ c, sty{j}, 'Color', CC, 'LineWidth', 2.0, ...
             'HandleVisibility', 'off');
    end
    plot(ax, [W(1) W(end)], ANCH * [1 1], '--', 'Color', [0.8 0 0], 'LineWidth', 1.8);
    set(ax, 'XScale', 'log');
    ylabel(ax, 'b_{eff}(w)   B dark / C light', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax, '$\bar{w}$', 'Interpreter', 'latex', 'FontSize', FS);
    legend(hh, {'plane', '\lambda=50', '\lambda=20', '\lambda=10'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 5);
    local_ax(ax, FS);

    out = fullfile(fig_dir, 'formB_BC_curvature.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('\nwrote %s\n', out);
end

% --------------------------------------------------------------------------
function c = local_c(W, lam)
    c = zeros(size(W));
    for i = 1:numel(W)
        if isinf(lam)
            [~, c(i)] = calc_correction_functions(W(i), true);
        else
            c(i) = build_truth_two_sphere(W(i), lam);
        end
    end
end

function local_ax(ax, FS)
    set(ax, 'FontSize', FS - 4, 'FontWeight', 'bold', 'LineWidth', 1.3, 'Box', 'on');
    grid(ax, 'off');
end
