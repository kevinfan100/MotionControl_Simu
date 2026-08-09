function res = plot_formB_amp_prior_chain(opts)
%PLOT_FORMB_AMP_PRIOR_CHAIN  Prior six-step chain for the amplitude writing.
%
%   STATUS: ACTIVE -- offline (no filter, no simulation). Feeds
%   reference/eq17_analysis/derivation/formB_amp_bonly_probe.tex figure F6.
%
%   Two gain-law writings, both with the exponent pinned at p = 1 and the law
%   origin pinned at the nominal contact w_bar_s = 1:
%
%       Form B  (length,    production)  a_bar = 1 - b/((w_bar - 1) + b)
%       Form C  (amplitude, this probe)  a_bar = 1 - b/w_bar
%
%   Step 3 of the P[0] chain (reference/shared/param_prior_rules.md) asks for
%   theta_eff(w_bar), the constant the truth demands at each height. Both
%   writings put b at a LEVEL of the deficit 1 - a_bar, so both readings are
%   level readings (formB_ws_ref.tex S10). Inverting each law against the
%   published truth deficit (c-1)/c gives, with no fitting anywhere:
%
%       b_eff,B(w_bar) = (c - 1) * (w_bar - 1)          [production, S10]
%       b_eff,C(w_bar) = w_bar * (c - 1) / c            [amplitude writing]
%
%   Form C's reading is the log-log INTERCEPT: ln(1 - a_bar) = ln b - p ln w,
%   so with p pinned the intercept is read directly and there is no choice of
%   reading to justify.
%
%   Step 6 is then sqrt(P[0]) = sup|b_eff - 9/8| over the sup domain, which
%   the 2026-08-01 revision sets to the planned envelope. The point of this
%   figure is WHERE each sup sits:
%
%       Form B  b_eff is NOT monotone -- sup is interior, boundary-immune
%       Form C  b_eff IS  monotone    -- sup sits ON the envelope floor
%
%   so the amplitude writing's prior width inherits the floor margin's
%   provenance directly, and that must be declared rather than inherited from
%   the production comment ("insensitive by design", true only for Form B).
%
%   Figure  -> derivation/figures/formB_amp_prior_chain.png
%   Truth   -> calc_correction_functions (two-sphere perpendicular series)
%
%   Style: house rules (no grid, box on, legend northoutside horizontal, no
%   title, statistics to console), latex interpreter throughout.
%
%   See also: plot_formB_form_compare, run_formB_ws/local_envelope_priors.

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'save');   opts.save   = true;  end
    if ~isfield(opts, 'n_grid'); opts.n_grid = 40001; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(fullfile(root, 'model', 'wall_effect'));
    addpath(fullfile(root, 'model', 'config'));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    B_ANCHOR  = 9/8;      % far-field method-of-reflections coefficient
    B_CONTACT = 1;        % near-wall (Brenner) coefficient
    ENV_LO    = 1.90;     % canonical envelope floor = trough 2.0 - margin 0.1
    ENV_HI    = 22.3;     % canonical envelope ceiling = start 22.22 + 1
    FLOORS    = [1.90, 1.95, 2.00];   % declared boundary-sensitivity probes

    % ---- truth and the two level readings --------------------------------
    W = logspace(log10(1.001), log10(25), opts.n_grid).';
    c = zeros(size(W));
    for i = 1:numel(W)
        [~, c(i)] = calc_correction_functions(W(i), true);
    end
    bB = (c - 1) .* (W - 1);          % production level reading (S10)
    bC = W .* (c - 1) ./ c;           % amplitude level reading (log-log intercept)

    % ---- the sup on the canonical envelope -------------------------------
    m = W >= ENV_LO & W <= ENV_HI;
    [supB, iB] = max(abs(bB(m) - B_ANCHOR));
    [supC, iC] = max(abs(bC(m) - B_ANCHOR));
    Wm = W(m);

    % ---- boundary sensitivity of each sup --------------------------------
    sensB = zeros(size(FLOORS));
    sensC = zeros(size(FLOORS));
    for k = 1:numel(FLOORS)
        mk = W >= FLOORS(k) & W <= ENV_HI;
        sensB(k) = max(abs(bB(mk) - B_ANCHOR));
        sensC(k) = max(abs(bC(mk) - B_ANCHOR));
    end

    % ---- console ---------------------------------------------------------
    fprintf('\n[amp prior chain]  envelope [%.2f, %.2f], anchors %g (contact) / %g (far)\n', ...
            ENV_LO, ENV_HI, B_CONTACT, B_ANCHOR);
    fprintf('  b_eff range          B [%.4f, %.4f]      C [%.4f, %.4f]\n', ...
            min(bB(m)), max(bB(m)), min(bC(m)), max(bC(m)));
    fprintf('  monotone on envelope B %d                 C %d\n', ...
            all(diff(bB(m)) > 0), all(diff(bC(m)) > 0));
    fprintf('  sup|b_eff - 9/8|     B %.4f at w = %.3f   C %.4f at w = %.3f\n', ...
            supB, Wm(iB), supC, Wm(iC));
    fprintf('  boundary sensitivity (floor 1.90 / 1.95 / 2.00):\n');
    fprintf('      B  %.4f / %.4f / %.4f   (spread %.1e -- interior sup, immune)\n', ...
            sensB, max(sensB) - min(sensB));
    fprintf('      C  %.4f / %.4f / %.4f   (spread %.4f = %.1f%% -- sup ON the floor)\n', ...
            sensC, max(sensC) - min(sensC), 100 * (max(sensC) - min(sensC)) / sensC(1));
    fprintf('  global sup [1.001, 25]  B %.4f   C %.4f   (= the 12.5%% anchor tension)\n', ...
            max(abs(bB - B_ANCHOR)), max(abs(bC - B_ANCHOR)));

    % shape floor: the residual the anchored seed cannot represent
    aB = 1 - B_ANCHOR ./ ((W - 1) + B_ANCHOR);
    aC = 1 - B_ANCHOR ./ W;
    fprintf('  shape floor sup|a_law - 1/c| on envelope   B %.5f   C %.5f  (%.1fx)\n', ...
            max(abs(aB(m) - 1 ./ c(m))), max(abs(aC(m) - 1 ./ c(m))), ...
            max(abs(aC(m) - 1 ./ c(m))) / max(abs(aB(m) - 1 ./ c(m))));
    % identity that self-checks both amplitude formulas (exact, pointwise)
    fprintf('  self-check  floor_a,C == max|b_eff,C - 9/8|/w :  %.3e\n', ...
            abs(max(abs(aC(m) - 1 ./ c(m))) - max(abs(bC(m) - B_ANCHOR) ./ Wm)));

    res = struct('W', W, 'bB', bB, 'bC', bC, 'supB', supB, 'supC', supC, ...
                 'sensB', sensB, 'sensC', sensC);
    if ~opts.save; return; end

    % ---- figure ----------------------------------------------------------
    C_B   = [0    0.20 0.90];      % production (length) writing
    C_C   = [0.45 0.55 0.95];      % amplitude writing
    C_ANC = [0.80 0    0   ];      % anchors (derived truth values)
    C_ENV = [0.90 0.90 0.90];      % envelope shading
    FS = 18; LFS = 14; AXLW = 1.8; LW = 2.2;

    f = figure('Position', [80 80 1000 900], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');

    % --- (a) the two theta_eff readings -----------------------------------
    ax1 = subplot(2, 1, 1); hold(ax1, 'on');
    local_env_patch(ax1, ENV_LO, ENV_HI, [0.95 1.20], C_ENV);
    ha = plot(ax1, [min(W) max(W)], B_ANCHOR * [1 1], '--', ...
              'Color', C_ANC, 'LineWidth', 1.6);
    plot(ax1, [min(W) max(W)], B_CONTACT * [1 1], '--', ...
         'Color', C_ANC, 'LineWidth', 1.6, 'HandleVisibility', 'off');
    hB = plot(ax1, W, bB, '-', 'Color', C_B, 'LineWidth', LW);
    hC = plot(ax1, W, bC, '-', 'Color', C_C, 'LineWidth', LW);
    hsB = plot(ax1, Wm(iB), bB(find(m, 1) + iB - 1), 'o', 'Color', C_B, ...
               'MarkerFaceColor', C_B, 'MarkerSize', 9);
    hsC = plot(ax1, Wm(iC), bC(find(m, 1) + iC - 1), 'o', 'Color', C_C, ...
               'MarkerFaceColor', C_C, 'MarkerSize', 9);
    set(ax1, 'XScale', 'log');
    ylim(ax1, [0.95 1.20]); xlim(ax1, [min(W) max(W)]);
    ylabel(ax1, '$b_{\mathrm{eff}}(\bar{w})$', 'Interpreter', 'latex', 'FontSize', FS);
    legend([hB hC ha hsB hsC], ...
           {'Form B (length)', 'Form C (amplitude)', 'anchors $1$, $9/8$', ...
            'sup B (interior)', 'sup C (on floor)'}, ...
           'Interpreter', 'latex', 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS - 2, 'Box', 'on', ...
           'NumColumns', 3);
    local_style(ax1, FS, AXLW);

    % --- (b) the prior budget |b_eff - anchor| ----------------------------
    ax2 = subplot(2, 1, 2); hold(ax2, 'on');
    local_env_patch(ax2, ENV_LO, ENV_HI, [0 0.14], C_ENV);
    hB2 = plot(ax2, W, abs(bB - B_ANCHOR), '-', 'Color', C_B, 'LineWidth', LW);
    hC2 = plot(ax2, W, abs(bC - B_ANCHOR), '-', 'Color', C_C, 'LineWidth', LW);
    hwB = plot(ax2, [min(W) max(W)], supB * [1 1], ':', 'Color', C_B, 'LineWidth', 1.8);
    hwC = plot(ax2, [min(W) max(W)], supC * [1 1], ':', 'Color', C_C, 'LineWidth', 1.8);
    set(ax2, 'XScale', 'log');
    ylim(ax2, [0 0.14]); xlim(ax2, [min(W) max(W)]);
    xlabel(ax2, '$\bar{w} = h/R$', 'Interpreter', 'latex', 'FontSize', FS);
    ylabel(ax2, '$|b_{\mathrm{eff}} - 9/8|$', 'Interpreter', 'latex', 'FontSize', FS);
    legend([hB2 hC2 hwB hwC], ...
           {'Form B', 'Form C', ...
            sprintf('$\\sqrt{P[0]}_{\\mathrm{B}} = %.4f$', supB), ...
            sprintf('$\\sqrt{P[0]}_{\\mathrm{C}} = %.4f$', supC)}, ...
           'Interpreter', 'latex', 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS - 2, 'Box', 'on', ...
           'NumColumns', 4);
    local_style(ax2, FS, AXLW);

    out = fullfile(fig_dir, 'formB_amp_prior_chain.png');
    exportgraphics(f, out, 'Resolution', 150);
    close(f);
    fprintf('  wrote %s\n', out);
end

% --------------------------------------------------------------------------
function local_env_patch(ax, lo, hi, yl, col)
    patch(ax, [lo hi hi lo], [yl(1) yl(1) yl(2) yl(2)], col, ...
          'EdgeColor', 'none', 'HandleVisibility', 'off');
end

function local_style(ax, FS, AXLW)
    XT = [1 1.5 2 3 5 10 20];
    set(ax, 'FontSize', FS, 'LineWidth', AXLW, 'Box', 'on', ...
            'TickLabelInterpreter', 'latex', 'Layer', 'top', ...
            'XTick', XT, 'XTickLabel', arrayfun(@(v) num2str(v), XT, ...
                                                'UniformOutput', false), ...
            'XMinorTick', 'off');
    grid(ax, 'off');
end
