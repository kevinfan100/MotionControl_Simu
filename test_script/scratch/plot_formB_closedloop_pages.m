function plot_formB_closedloop_pages(opts)
%PLOT_FORMB_CLOSEDLOOP_PAGES  Closed-loop pages for formB_amp_functions.
%
%   STATUS: ACTIVE -- reads test_results/temp_formB_two_boundaries.mat.
%   Pages 7 and 8 appended to formB_amp_functions (one page per truth
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
    if ~isfield(opts, 'ylim_b'); opts.ylim_b = {[1.04 1.16], [0.08 1.28]}; end
    if ~isfield(opts, 'ylim_e'); opts.ylim_e = {[-9 3],      [-17 24]};     end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    L = load(fullfile(root, 'test_results', 'temp_formB_two_boundaries.mat'));
    r = L.res;
    % p.7 is the BLIND test: the controller keeps the plane anchor 9/8 and the
    % plant is c_para. Giving c_para its own 9/16 anchor would hand the filter
    % the one number an unknown boundary does not provide.
    Lb = load(fullfile(root, 'test_results', 'temp_formB_blind_para.mat'));
    rb = Lb.res;

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
        if ib == 1
            bB = r.trace{1,1,1}.b;  eB = r.trace{1,1,1}.e;
            bC = r.trace{1,2,1}.b;  eC = r.trace{1,2,1}.e;
        else   % anchor 9/8 kept; prior widened to cover a non-plane boundary
            bB = rb.trace{3}.b;     eB = rb.trace{3}.e;
            bC = rb.trace{4}.b;     eC = rb.trace{4}.e;
        end

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

    % ---- the numbers behind each page, in the document's format ---------
    fprintf('\np.6  c_perp, anchor 9/8, plane prior  (12 seeds)\n');
    local_row('B', squeeze(r.tab(1,1,1,:,:)), squeeze(r.tab(1,1,1,1,:)));
    local_row('C', squeeze(r.tab(1,2,1,:,:)), squeeze(r.tab(1,2,1,1,:)));

    fprintf('\np.7  c_para, anchor 9/8 KEPT, prior widened on c_para  (12 seeds)\n');
    local_row('B', squeeze(rb.tab(3,:,:)), squeeze(rb.tab(3,1,:)));
    local_row('C', squeeze(rb.tab(4,:,:)), squeeze(rb.tab(4,1,:)));
    fprintf('     prior  B %.4f  C %.4f      b_end  B %.4f  C %.4f\n', ...
            rb.prior(3), rb.prior(4), mean(rb.tab(3,:,4)), mean(rb.tab(4,:,4)));

    fprintf('\n     for reference, the SAME page-7 run on p.6''s prior:\n');
    local_row('B', squeeze(rb.tab(1,:,:)), squeeze(rb.tab(1,1,:)));
    local_row('C', squeeze(rb.tab(2,:,:)), squeeze(rb.tab(2,1,:)));
end

% --------------------------------------------------------------------------
function local_row(nm, T, s1)
    fprintf('  %s  desc/osc/hold  12-seed %6.2f %6.2f %+7.2f   seed 7 %6.2f %6.2f %+7.2f\n', ...
            nm, mean(T(:,1)), mean(T(:,2)), mean(T(:,3)), s1(1), s1(2), s1(3));
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
