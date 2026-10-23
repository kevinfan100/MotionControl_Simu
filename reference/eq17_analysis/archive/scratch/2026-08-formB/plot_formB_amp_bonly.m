function res = plot_formB_amp_bonly(opts)
%PLOT_FORMB_AMP_BONLY  Stage-5 figures for the amplitude-writing probe.
%
%   STATUS: ACTIVE -- reads test_results/temp_formB_amp_stage4.mat, produced by
%   test_script/scratch/run_formB_amp_bonly_arms.m. Feeds
%   reference/eq17_analysis/derivation/formB_amp_bonly_probe.tex.
%
%   F1  formB_bonly_BvsC_seed{7,11}.png   4 rows x 2 cols, left LENGTH (only b
%       free, production prior) / right AMPLITUDE (the tie), same seed, same
%       noise realisation. Rows: gain, gain error %, the free constant with its
%       claimed width, P[0] traversal budget.
%   F2  formB_amp_vs_len_bhat.png         all seeds, both arms, beta(t)
%   F3  formB_amp_jacobian_live.png       the b-channel Jacobian along the REAL
%       trajectory, and the commanded-motion weight it enters with
%   F4  formB_amp_bhat_landing.png        final beta per seed against the four
%       pre-registered landing hypotheses
%   F5  formB_amp_manifold_resid.png      a_hat - (1 - beta/w_d): the residual
%       off the one-parameter law manifold
%
%   Style: house rules (no grid, box on, legend northoutside horizontal, no
%   title, statistics to console, True red / Estimate blue / Measured light
%   blue, exportgraphics 150 dpi).

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'seeds_shown'); opts.seeds_shown = [7 11]; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    L = load(fullfile(root, 'test_results', 'temp_formB_amp_stage4.mat'));
    res = L.res;

    names = res.arm_names;
    iLEN  = find(strcmp(names, 'len_bonly'));
    iAMP  = find(strcmp(names, 'amp'));
    iPRD  = find(strcmp(names, 'len_prod'));
    iW98  = find(strcmp(names, 'len_ws98'));
    seeds = res.seeds;

    B_ANCHOR = 9/8;
    HYP = [1.1250, 1.0833, 1.0651, 1.0588];
    HYP_NM = {'anchor (no move)', 'info-weighted', 'minimax', 'trough-exact'};

    FS = 18; RED = 'r'; BLUE = 'b'; LIGHT_BLUE = [0.45 0.72 0.95];
    BAND = [0.78 0.85 0.95]; GREY = [0.55 0.55 0.55]; ERRB = 2;
    cfg = res.cfg;
    tb  = [cfg.t_hold, cfg.t_hold + cfg.t_descend_override, ...
           cfg.t_hold + cfg.t_descend_override + cfg.n_cycles / cfg.frequency];

    % =================================================================== F1
    for sq = opts.seeds_shown
        q = find(seeds == sq, 1);
        if isempty(q); continue; end
        f = figure('Position', [40 40 1500 1150], 'Color', 'w', 'Visible', 'off');
        cols = [iLEN, iAMP];
        ttl  = {'Form B (length), only b free', 'Form C (amplitude), only \beta free'};
        for cc = 1:2
            a  = res.runs{cols(cc), q};
            pr = res.runs{iPRD, q};
            t  = a.t;  ad = 1;                       % logs already in um/pN
            % --- row 1: gain
            ax = subplot(4, 2, cc); hold(ax, 'on');
            h1 = plot(ax, t, a.a_true, RED, 'LineWidth', 2.0);
            h2 = plot(ax, t, a.a_hat,  BLUE, 'LineWidth', 2.0);
            h3 = plot(ax, t, a.a_xm, 'Color', LIGHT_BLUE, 'LineWidth', 1.0);
            ylabel(ax, 'a_z  [\mum/pN]', 'FontSize', FS, 'FontWeight', 'bold');
            legend([h1 h2 h3], {'a_{true}', 'a_{hat}', 'a_{wm}'}, ...
                   'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', FS - 5);
            text(ax, 0.02, 0.92, ttl{cc}, 'Units', 'normalized', ...
                 'FontSize', FS - 4, 'FontWeight', 'bold');
            xlim(ax, [t(1) t(end)]); style_ax(ax, FS); add_phase(ax, tb);
            % --- row 2: gain error
            e  = 100 * (a.a_hat - a.a_true) ./ a.a_true;
            ep = 100 * (pr.a_hat - pr.a_true) ./ pr.a_true;
            ax = subplot(4, 2, 2 + cc); hold(ax, 'on');
            h0 = plot(ax, t, zeros(size(t)), RED, 'LineWidth', 1.6);
            hp = plot(ax, t, ep, '--', 'Color', GREY, 'LineWidth', 1.4);
            he = plot(ax, t, e, BLUE, 'LineWidth', 2.0);
            plot(ax, t,  ERRB * ones(size(t)), 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
            plot(ax, t, -ERRB * ones(size(t)), 'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
            ylabel(ax, 'a_z error  [%]', 'FontSize', FS, 'FontWeight', 'bold');
            legend([h0 he hp], {'zero', 'this arm', 'production (b,p) free'}, ...
                   'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', FS - 5);
            ylim(ax, [-12 12]); xlim(ax, [t(1) t(end)]); style_ax(ax, FS); add_phase(ax, tb);
            % --- row 3: the free constant and what the truth demands
            ax = subplot(4, 2, 4 + cc); hold(ax, 'on');
            tf = [t; flipud(t)];
            bf = [a.b_hat + a.P_b; flipud(a.b_hat - a.P_b)];
            hb = fill(ax, tf, bf, BAND, 'EdgeColor', 'none', 'FaceAlpha', 0.85);
            if cc == 1
                beff = local_beff_len(a.h_bar_d);
                lbl  = 'b_{eff,B}(w(t)) demanded';
            else
                beff = local_beff_amp(a.h_bar_d);
                lbl  = 'b_{eff,C}(w(t)) demanded';
            end
            ht = plot(ax, t, beff, RED, 'LineStyle', '--', 'LineWidth', 2.0);
            hh = plot(ax, t, a.b_hat, BLUE, 'LineWidth', 2.2);
            ha = plot(ax, t, B_ANCHOR * ones(size(t)), 'k--', 'LineWidth', 1.4);
            ylabel(ax, 'free constant  [-]', 'FontSize', FS, 'FontWeight', 'bold');
            legend([ht hh hb ha], {lbl, 'estimate', ...
                    sprintf('\\pm sqrt(P) = %.4f', a.P_b(1)), 'far-field anchor 9/8'}, ...
                   'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', FS - 9);
            ylim(ax, [0.98 1.20]); xlim(ax, [t(1) t(end)]); style_ax(ax, FS); add_phase(ax, tb);
            % --- row 4: P[0] traversal budget
            ax = subplot(4, 2, 6 + cc); hold(ax, 'on');
            den = a.P_b(1)^2 - a.P_b.^2;
            msk = den > 0.02 * a.P_b(1)^2;              % kill the 0/0 head
            rat = nan(size(t)); rat(msk) = (a.b_hat(msk) - a.b_hat(1)).^2 ./ den(msk);
            hl = plot(ax, t, ones(size(t)), RED, 'LineWidth', 2.0);
            hr = plot(ax, t, rat, 'Color', [0.1 0.5 0.2], 'LineWidth', 2.2);
            set(ax, 'YScale', 'log'); ylim(ax, [1e-3 1e2]);
            ylabel(ax, 'P[0] budget ratio', 'FontSize', FS, 'FontWeight', 'bold');
            xlabel(ax, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
            legend([hl hr], {'limit = 1', '(b-b_0)^2/\DeltaP'}, ...
                   'Location', 'northoutside', 'Orientation', 'horizontal', ...
                   'FontSize', FS - 5);
            xlim(ax, [t(1) t(end)]); style_ax(ax, FS); add_phase(ax, tb);
        end
        out = fullfile(fig_dir, sprintf('formB_bonly_BvsC_seed%d.png', sq));
        exportgraphics(f, out, 'Resolution', 150); close(f);
        fprintf('F1 -> %s\n', out);
    end

    % =================================================================== F2
    f = figure('Position', [60 60 1100 620], 'Color', 'w', 'Visible', 'off');
    ax = axes(f); hold(ax, 'on');
    for q = 1:numel(seeds)
        aL = res.runs{iLEN, q};  aA = res.runs{iAMP, q};
        hL = plot(ax, aL.t, aL.b_hat, '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.2);
        hA = plot(ax, aA.t, aA.b_hat, '-', 'Color', BLUE, 'LineWidth', 1.2);
    end
    ha = plot(ax, aA.t, B_ANCHOR * ones(size(aA.t)), 'k--', 'LineWidth', 1.6);
    hm = plot(ax, aA.t, HYP(3) * ones(size(aA.t)), RED, 'LineStyle', ':', 'LineWidth', 2.0);
    ylabel(ax, 'free constant  [-]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([hL hA ha hm], {sprintf('length, only b (%d seeds)', numel(seeds)), ...
            sprintf('amplitude, only \\beta (%d seeds)', numel(seeds)), ...
            'anchor 9/8', 'envelope minimax 1.0651'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 6);
    bmin = inf; bmax = -inf;
    for q = 1:numel(seeds)
        bmin = min(bmin, min(res.runs{iAMP, q}.b_hat(2:end)));
        bmax = max(bmax, max(res.runs{iAMP, q}.b_hat(2:end)));
    end
    ylim(ax, [bmin - 0.005, max(bmax, 1.14) + 0.005]);
    xlim(ax, [aA.t(1) aA.t(end)]);
    style_ax(ax, FS); add_phase(ax, tb);
    out = fullfile(fig_dir, 'formB_amp_vs_len_bhat.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('F2 -> %s\n', out);

    % =================================================================== F3
    q = find(seeds == opts.seeds_shown(1), 1);
    aL = res.runs{iLEN, q}; aA = res.runs{iAMP, q};
    t = aA.t; wd = aA.h_bar_d;
    bL = [aL.b_hat(1); aL.b_hat(1:end-1)];      % prior, the value the law used
    bA = [aA.b_hat(1); aA.b_hat(1:end-1)];
    bad = ~(wd > 1.01);                         % row 1 is the init call (w_d = 0)
    gL = wd - 1;
    JL = (gL - bL) ./ (gL + bL).^3;             % production J_b
    JA = 1 ./ wd.^2;                            % tied J_beta
    JL(bad) = NaN; JA(bad) = NaN;
    M  = [0; diff(wd)]; M(bad) = NaN;
    f = figure('Position', [60 60 1100 820], 'Color', 'w', 'Visible', 'off');
    ax = subplot(2, 1, 1); hold(ax, 'on');
    h0 = plot(ax, t, zeros(size(t)), 'Color', GREY, 'LineWidth', 1.2);
    h1 = plot(ax, t, JL, '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 2.0);
    h2 = plot(ax, t, JA, '-', 'Color', BLUE, 'LineWidth', 2.0);
    jv = JL(t > tb(1) & ~bad);
    nz = sum(diff(sign(jv)) ~= 0);
    ylim(ax, [-0.05 0.30]);
    ylabel(ax, 'b-channel J  [-]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2 h0], {'length  J_b = (g-b)/(g+b)^3', 'amplitude  J_\beta = 1/w^2', 'zero'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 5);
    xlim(ax, [t(1) t(end)]); style_ax(ax, FS); add_phase(ax, tb);
    ax = subplot(2, 1, 2); hold(ax, 'on');
    h1 = plot(ax, t, abs(M .* JL), '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.6);
    h2 = plot(ax, t, abs(M .* JA), '-', 'Color', BLUE, 'LineWidth', 1.6);
    set(ax, 'YScale', 'log'); ylim(ax, [1e-9 1e-3]);
    ylabel(ax, '|M \cdot J|  channel weight', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2], {'length', 'amplitude'}, 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', FS - 5);
    xlim(ax, [t(1) t(end)]); style_ax(ax, FS); add_phase(ax, tb);
    out = fullfile(fig_dir, 'formB_amp_jacobian_live.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('F3 -> %s   (length J_b sign changes after t=%.1fs: %d)\n', out, tb(1), nz);

    % =================================================================== F4
    bfA = squeeze(res.tab(iAMP, :, 2)); sA = squeeze(res.tab(iAMP, :, 4));
    bfL = squeeze(res.tab(iLEN, :, 2));
    bfW = squeeze(res.tab(iW98, :, 2));
    f = figure('Position', [60 60 1100 620], 'Color', 'w', 'Visible', 'off');
    ax = axes(f); hold(ax, 'on');
    x = 1:numel(seeds);
    hyp_h = gobjects(1, 4);
    hc = {[0.2 0.2 0.2], [0.85 0.4 0.1], [0.8 0 0], [0.5 0 0.6]};
    for j = 1:4
        hyp_h(j) = plot(ax, [0 numel(seeds) + 1], HYP(j) * [1 1], '--', ...
                        'Color', hc{j}, 'LineWidth', 1.6);
    end
    hL = plot(ax, x, bfL, 'o', 'Color', [0.55 0.55 0.55], 'MarkerFaceColor', [0.8 0.8 0.8], ...
              'MarkerSize', 8);
    hW = plot(ax, x, bfW, 's', 'Color', [0.35 0.6 0.35], 'MarkerFaceColor', [0.7 0.88 0.7], ...
              'MarkerSize', 8);
    hA = errorbar(ax, x, bfA, sA, 'o', 'Color', BLUE, 'MarkerFaceColor', BLUE, ...
                  'MarkerSize', 9, 'LineWidth', 1.4, 'LineStyle', 'none', 'CapSize', 6);
    ylabel(ax, '\beta final  [-]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax, 'seed', 'FontSize', FS, 'FontWeight', 'bold');
    set(ax, 'XTick', x, 'XTickLabel', arrayfun(@num2str, seeds, 'UniformOutput', false));
    legend([hA hW hL hyp_h(1) hyp_h(2) hyp_h(3) hyp_h(4)], ...
           [{'amplitude \pm sqrt(P) final', 'length, origin 9/8', 'length, only b'}, HYP_NM], ...
           'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', FS - 8, ...
           'NumColumns', 4);
    xlim(ax, [0.5, numel(seeds) + 0.5]);
ylim(ax, [min(1.0, min([bfA - sA, bfL, bfW])) - 0.005, ...
          max([bfA + sA, bfL, bfW]) + 0.005]);
style_ax(ax, FS);
    out = fullfile(fig_dir, 'formB_amp_bhat_landing.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('F4 -> %s\n', out);

    % =================================================================== F5
    f = figure('Position', [60 60 1100 560], 'Color', 'w', 'Visible', 'off');
    ax = axes(f); hold(ax, 'on');
    % a_hat is logged in um/pN = a_bar * a_disp with a_disp = a_o*R exactly.
    pcv   = physical_constants();
    a_disp = (pcv.Ts / (pcv.gamma_N * pcv.R)) * pcv.R;
    for j = 1:min(3, numel(seeds))
        a = res.runs{iAMP, j};
        ab = a.a_hat / a_disp;                          % back to the normalised gain
        law = 1 - a.b_hat ./ a.h_bar_d;
        plot(ax, a.t, 100 * (ab - law), 'LineWidth', 1.4);
    end
    plot(ax, a.t, zeros(size(a.t)), RED, 'LineWidth', 1.4, 'HandleVisibility', 'off');
    ylabel(ax, 'a_{hat}/a_o - (1-\beta/w)  [%]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax, 'time  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    legend(arrayfun(@(s) sprintf('seed %d', s), seeds(1:min(3, numel(seeds))), ...
           'UniformOutput', false), 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', FS - 5);
    xlim(ax, [a.t(1) a.t(end)]); style_ax(ax, FS); add_phase(ax, tb);
    out = fullfile(fig_dir, 'formB_amp_manifold_resid.png');
    exportgraphics(f, out, 'Resolution', 150); close(f);
    fprintf('F5 -> %s\n', out);
end

% --------------------------------------------------------------------------
function b = local_beff_amp(w)
    b = nan(size(w));
    for i = 1:numel(w)
        if w(i) > 1.001
            [~, c] = calc_correction_functions(w(i), true);
            b(i) = w(i) * (c - 1) / c;
        end
    end
end

function b = local_beff_len(w)
    b = nan(size(w));
    for i = 1:numel(w)
        if w(i) > 1.001
            [~, c] = calc_correction_functions(w(i), true);
            b(i) = (c - 1) * (w(i) - 1);
        end
    end
end

function style_ax(ax, FS)
    set(ax, 'FontSize', FS - 4, 'FontWeight', 'bold', 'LineWidth', 1.3, 'Box', 'on');
    grid(ax, 'off');
end

function add_phase(ax, tb)
    yl = ylim(ax);
    for x = tb
        plot(ax, [x x], yl, '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, ...
             'HandleVisibility', 'off');
    end
    ylim(ax, yl);
end
