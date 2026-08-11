function res = plot_formC_state_slope(opts)
%PLOT_FORMC_STATE_SLOPE  Form C's slope read off the STATE, not the height.
%
%   Form C   a_bar = 1 - b/(1 + w-ws)   (p = 1)  =>  1-a_bar = b/(1+w-ws), so
%
%       a_bar' = b/(1+w-ws)^2 = (1-a_bar)^2 / b
%
%   i.e. the slope is a function of the gain VALUE alone -- no height, no wall
%   position.  This script feeds the TRUE gain into that relation with a single
%   frozen b and compares against the true slope:
%
%       a_bar'_law(w) = (1 - a_true(w))^2 / b      vs     a_bar'_true(w)
%
%   opts.b picks the frozen constant that is DRAWN; default 1, i.e. the bare
%   simplification a' = (1-a)^2 with no constant at all.  opts.b = [] draws b*.
%
%   Everything is set by one curve, the b the relation demands at each height:
%
%       b_state(w) := (1 - a_true)^2 / a'_true   =>   D a'/a'_true = b_state/b - 1
%
%   b* is the constant minimizing the RMS of that relative error (closed form,
%   b* = sum(b_state^2)/sum(b_state)); it is a DIAGNOSTIC number and does not
%   enter any controller.  b_state -> 9/8 exactly in the far field (c_perp ~
%   1 + 9/(8 h_bar) gives (1-a)^2/a' -> 9/8), so the existing anchor is also
%   the exact asymptote of this relation -- b* and 9/8 are both printed.
%
%   Range w-ws in [0.1, 25] (user, 2026-08-11).
%
%   Figure -> derivation/figures/formC_state_slope.png  (2 panels, shared x:
%   top = the two slopes, log y because a' spans ~3 decades; bottom = the
%   relative error).
%
%   Style: house rules (no grid, box on, legend northoutside horizontal, no
%   title, stats to console, latex interpreter).
%
%   See also: plot_formB_form_compare.m, formB_amp_functions.tex

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'gap_min'); opts.gap_min = 0.1;  end
    if ~isfield(opts, 'gap_max'); opts.gap_max = 25;   end
    if ~isfield(opts, 'save');    opts.save    = true; end
    % Which constant is DRAWN.  Default 1 = the bare simplification
    % a' = (1-a)^2 (user, 2026-08-11).  opts.b = [] draws b* instead.
    if ~isfield(opts, 'b');       opts.b       = 1;    end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(fullfile(root, 'model', 'wall_effect'));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    G = linspace(opts.gap_min, opts.gap_max, 6000).';
    [a, ap_true] = truth_a(1 + G);

    b_state = (1 - a).^2 ./ ap_true;
    bstar   = sum(b_state.^2) / sum(b_state);       % RMS-relative optimum
    if isempty(opts.b); b_use = bstar; else; b_use = opts.b; end
    ap_law  = (1 - a).^2 / b_use;
    e       = ap_law ./ ap_true - 1;                % == b_state/b_use - 1

    R = @(x) sqrt(mean(x.^2));
    fprintf('\n[Form C, slope from the state]  a'''' = (1-a)^2/b , b frozen\n');
    fprintf('  range w-ws       [%.2f, %.2f]   a_true %.4f -> %.4f\n', ...
            G(1), G(end), a(1), a(end));
    fprintf('  b_state(w)       %.5f -> %.5f   (max %.5f at w-ws = %.2f)\n', ...
            b_state(1), b_state(end), max(b_state), G(find(b_state == max(b_state), 1)));
    fprintf('  far-field anchor b_state(end) - 9/8 = %+.2e\n', b_state(end) - 9/8);
    for bf = unique([b_use, bstar, 9/8], 'stable')
        ee = b_state / bf - 1;
        fprintf('  b = %.5f      RMS %6.3f %%   sup %6.3f %%   range [%+.3f, %+.3f] %%\n', ...
                bf, 100*R(ee), 100*max(abs(ee)), 100*min(ee), 100*max(ee));
    end
    fprintf('  drawn: b = %.5f\n', b_use);
    fprintf('  identity max|a''_law/a''_true - (b_state/b)| = %.1e\n', ...
            max(abs(e - (b_state/b_use - 1))));

    res = struct('G', G, 'a', a, 'ap_true', ap_true, 'ap_law', ap_law, ...
                 'b_state', b_state, 'bstar', bstar, 'b_use', b_use, 'err', e);
    if ~opts.save; return; end

    % ---- house style ----------------------------------------------------
    C_TRUE = [0.8 0 0];  C_LAW = [0 0.2 0.9];
    FS = 18; LFS = 15; AXLW = 2.0; LW = 2.2;
    XL = '$\bar{w}-\bar{w}_s$';

    f = figure('Position', [80 80 1000 860], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    t = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile; hold(ax1, 'on');
    h1 = semilogy(ax1, G, ap_true, '-', 'Color', C_TRUE, 'LineWidth', LW + 0.6, ...
                  'DisplayName', '$\bar{a}^{\prime}_{\mathrm{true}}$');
    if b_use == 1
        LEG_LAW = '$(1-\bar{a})^{2}$';
    else
        LEG_LAW = sprintf('$(1-\\bar{a})^{2}/%.4f$', b_use);
    end
    h2 = semilogy(ax1, G, ap_law, '-', 'Color', C_LAW, 'LineWidth', LW, ...
                  'DisplayName', LEG_LAW);
    set(ax1, 'YScale', 'log');
    xlim(ax1, [G(1) G(end)]);
    ylabel(ax1, '$\bar{a}^{\prime}$', 'Interpreter', 'latex', 'FontSize', FS);
    legend(ax1, [h1 h2], 'Interpreter', 'latex', 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
    set(ax1, 'FontSize', FS, 'LineWidth', AXLW, 'Box', 'on', ...
             'TickLabelInterpreter', 'latex', 'XTickLabel', []);
    grid(ax1, 'off');

    ax2 = nexttile; hold(ax2, 'on');
    yline(ax2, 0, '-', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
    plot(ax2, G, 100*e, '-', 'Color', C_LAW, 'LineWidth', LW);
    xlim(ax2, [G(1) G(end)]);
    xlabel(ax2, XL, 'Interpreter', 'latex', 'FontSize', FS);
    ylabel(ax2, '$\Delta\bar{a}^{\prime}/\bar{a}^{\prime}_{\mathrm{true}}$  [\%]', ...
           'Interpreter', 'latex', 'FontSize', FS);
    set(ax2, 'FontSize', FS, 'LineWidth', AXLW, 'Box', 'on', ...
             'TickLabelInterpreter', 'latex');
    grid(ax2, 'off');

    out = fullfile(fig_dir, 'formC_state_slope.png');
    exportgraphics(t, out, 'Resolution', 200, 'Padding', 'figure');
    close(f);
    fprintf('  wrote %s\n', out);
end

% --------------------------------------------------------------------------
function [a, ap] = truth_a(w)
    c = zeros(size(w)); dc = zeros(size(w));
    for i = 1:numel(w)
        [~, c(i), dv] = calc_correction_functions(w(i), true);
        dc(i) = dv.dc_perp_dh;
    end
    a  = 1 ./ c;
    ap = -dc ./ c.^2;
end
