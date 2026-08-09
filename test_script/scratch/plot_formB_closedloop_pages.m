function plot_formB_closedloop_pages(opts)
%PLOT_FORMB_CLOSEDLOOP_PAGES  Closed-loop pages for formB_amp_functions.
%
%   STATUS: ACTIVE -- reads test_results/temp_formB_two_boundaries.mat.
%   Pages 6 and 7 appended to formB_amp_functions (one page per truth
%   function). Single seed, the two writings, constants estimated.
%
%   Two rows, matching the house style of formB_cmp_*.png exactly (latex
%   interpreter so the figures typeset in the document's own Computer Modern,
%   no grid, box on, legend northoutside horizontal, no title, statistics to
%   console, Resolution 200):
%       row 1   the estimated constant, and the b(w_bar) page 1 says the truth
%               demands at the height the particle is at
%       row 2   the gain error that constant produces
%
%   Figures -> derivation/figures/formB_cl_perp.png
%              derivation/figures/formB_cl_para.png

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'ylim_b'); opts.ylim_b = {[1.04 1.16], [0.34 0.60]}; end
    if ~isfield(opts, 'ylim_e'); opts.ylim_e = {[-9 3],      [-14 5]};     end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    L = load(fullfile(root, 'test_results', 'temp_formB_two_boundaries.mat'));
    r = L.res;

    % house palette, identical to plot_formB_form_compare
    C_TRUE = [0.8 0 0];
    C_B    = [0    0.20 0.90];
    C_C    = [0.45 0.55 0.95];
    FS = 20; LFS = 16; AXLW = 2.0; LW = 2.2;
    NAME = {'formB_cl_perp.png', 'formB_cl_para.png'};

    t  = r.trace{1,1,1}.t;
    w  = r.trace{1,1,1}.w;
    ok = w > 1.01;
    cP = nan(size(w)); cA = nan(size(w));
    for i = 1:numel(w)
        if ok(i); [cA(i), cP(i)] = calc_correction_functions(w(i), true); end
    end
    TB = [0.5 1.5 3.5];

    for ib = 1:2
        if ib == 1; c = cP; else; c = cA; end
        beB = (c - 1) .* (w - 1);
        beC = w .* (c - 1) ./ c;
        bB  = r.trace{ib,1,1}.b;   eB = r.trace{ib,1,1}.e;
        bC  = r.trace{ib,2,1}.b;   eC = r.trace{ib,2,1}.e;

        f = figure('Position', [60 60 1000 900], 'Color', 'w', ...
                   'NumberTitle', 'off', 'Visible', 'off');

        % ---- row 1: the constant, estimated and demanded -----------------
        ax = subplot(2, 1, 1); hold(ax, 'on');
        h1 = plot(ax, t, beB, '--', 'Color', C_TRUE, 'LineWidth', LW);
        h2 = plot(ax, t, beC, '-.', 'Color', C_TRUE, 'LineWidth', LW);
        h3 = plot(ax, t, bB,  '-',  'Color', C_B,    'LineWidth', LW);
        h4 = plot(ax, t, bC,  '-',  'Color', C_C,    'LineWidth', LW);
        ylim(ax, opts.ylim_b{ib}); xlim(ax, [t(1) t(end)]);
        ylabel(ax, '$b$', 'Interpreter', 'latex', 'FontSize', FS);
        legend([h1 h2 h3 h4], ...
               {'$b_{\mathrm{eff}}$ B', '$b_{\mathrm{eff}}$ C', ...
                '$\hat{b}$ B', '$\hat{b}$ C'}, ...
               'Interpreter', 'latex', 'Location', 'northoutside', ...
               'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
        local_ax(ax, FS, AXLW, TB);

        % ---- row 2: the gain error it produces ---------------------------
        ax = subplot(2, 1, 2); hold(ax, 'on');
        plot(ax, [t(1) t(end)], [0 0], '-', 'Color', C_TRUE, ...
             'LineWidth', 1.2, 'HandleVisibility', 'off');
        g1 = plot(ax, t, eB, '-', 'Color', C_B, 'LineWidth', LW);
        g2 = plot(ax, t, eC, '-', 'Color', C_C, 'LineWidth', LW);
        ylim(ax, opts.ylim_e{ib}); xlim(ax, [t(1) t(end)]);
        xlabel(ax, '$t$ [s]', 'Interpreter', 'latex', 'FontSize', FS);
        ylabel(ax, '$e_{a}$ [\%]', 'Interpreter', 'latex', 'FontSize', FS);
        legend([g1 g2], {'B', 'C'}, 'Interpreter', 'latex', ...
               'Location', 'northoutside', 'Orientation', 'horizontal', ...
               'FontSize', LFS, 'Box', 'on');
        local_ax(ax, FS, AXLW, TB);

        out = fullfile(fig_dir, NAME{ib});
        exportgraphics(f, out, 'Resolution', 200);
        close(f);
        fprintf('wrote %s\n', out);
    end

    % ---- the numbers that go on the pages, in the document's format -----
    BN = {'perp', 'para'};
    for ib = 1:2
        TB_ = squeeze(r.tab(ib, 1, 1, :, :));   % writing B, free
        TC_ = squeeze(r.tab(ib, 2, 1, :, :));   % writing C, free
        fprintf('\n%s  (12 seeds, free)\n', BN{ib});
        fprintf('  desc / osc / hold   B  %.2f, %.2f, %+.2f     C  %.2f, %.2f, %+.2f\n', ...
                mean(TB_(:,1)), mean(TB_(:,2)), mean(TB_(:,3)), ...
                mean(TC_(:,1)), mean(TC_(:,2)), mean(TC_(:,3)));
        fprintf('  seed-7 single run   B  %.2f, %.2f, %+.2f     C  %.2f, %.2f, %+.2f\n', ...
                squeeze(r.tab(ib,1,1,1,1)), squeeze(r.tab(ib,1,1,1,2)), squeeze(r.tab(ib,1,1,1,3)), ...
                squeeze(r.tab(ib,2,1,1,1)), squeeze(r.tab(ib,2,1,1,2)), squeeze(r.tab(ib,2,1,1,3)));
    end
end

% --------------------------------------------------------------------------
function local_ax(ax, FS, AXLW, TB)
    yl = ylim(ax);
    for x = TB
        plot(ax, [x x], yl, '--', 'Color', [0.62 0.62 0.62], ...
             'LineWidth', 1.0, 'HandleVisibility', 'off');
    end
    ylim(ax, yl);
    set(ax, 'FontSize', FS, 'LineWidth', AXLW, 'Box', 'on', ...
            'TickLabelInterpreter', 'latex');
    grid(ax, 'off');
end
