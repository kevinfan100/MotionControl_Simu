% STATUS: ACTIVE (scratch) | PURPOSE: relative gain error against TRUE h_bar
%   (not against time) for the two formC_state_dist.tex arms on the Meng
%   monotone ramp, seeds pooled. On a ramp the abscissa that carries the
%   physics is height, not time -- this is the figure the h_bar-banded table
%   of run_formC_dist_mengopen.m is a numerical readout of.
%   EXPIRES: baseline / disturbance adjudication.
function plot_formC_dist_err_vs_hbar(oB, oD, opts)
%PLOT_FORMC_DIST_ERR_VS_HBAR  e_a vs h_bar, both arms overlaid, seeds pooled.
%
%   plot_formC_dist_err_vs_hbar(oB, oD)
%   plot_formC_dist_err_vs_hbar(oB, oD, struct('suffix', '_mengopen'))
%
%   Each arm is reduced to the pooled mean of e_a = 100*(a_hat-a_true)/a_true
%   inside narrow h_bar bins, with a +-1 sd band across the pooled samples
%   (seeds x time), so the near-wall behaviour of the two arms can be read
%   against each other at equal height rather than at equal clock time.
%   Only the active run (t >= t_hold) contributes; the initial fixed-height
%   hold would otherwise pile a near-exact seed into the outermost bin.
%
%   Colour: zero error = the truth line, so it is RED per house style; the
%   baseline arm is grey and the arm under test (with delta a) is BLUE.
%   Style: canonical (plot_var_ahat_6state.m) -- 18 pt bold, 2.0 pt axes,
%   no grid, box on, legend northoutside horizontal, no title, Resolution 150.

    if nargin < 3; opts = struct(); end
    if ~isfield(opts, 'suffix');  opts.suffix = '';   end
    if ~isfield(opts, 'n_bins');  opts.n_bins = 110;  end
    if ~isfield(opts, 'bands');   opts.bands = [1.1 1.25 1.5 2 3 4.5 7]; end

    AX = 3;
    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    out = fullfile(fig_dir, sprintf('formC_dist_err_vs_hbar%s.png', opts.suffix));

    arms = {oB, oD};
    t_hold = oB.cfg.t_hold;
    E = cell(2, 1); H = cell(2, 1);
    for c = 1:2
        e_all = []; h_all = [];
        for q = 1:numel(arms{c}.runs)
            r  = arms{c}.runs{q};
            t  = r.tout(:);
            aT = r.a_true_out(:, AX);
            e  = 100 * (r.a_hat_out(:, AX) - aT) ./ max(aT, eps);
            w  = t >= t_hold;
            e_all = [e_all; e(w)];                        %#ok<AGROW>
            h_all = [h_all; r.h_bar_true_out(w)];         %#ok<AGROW>
        end
        E{c} = e_all; H{c} = h_all;
    end

    h_lo = min([min(H{1}), min(H{2})]);
    h_hi = max([max(H{1}), max(H{2})]);
    edges = linspace(h_lo, h_hi, opts.n_bins + 1);
    ctr   = 0.5 * (edges(1:end-1) + edges(2:end));

    mu = nan(2, opts.n_bins); sg = nan(2, opts.n_bins);
    for c = 1:2
        idx = discretize(H{c}, edges);
        for b = 1:opts.n_bins
            v = E{c}(idx == b);
            if numel(v) > 1; mu(c, b) = mean(v); sg(c, b) = std(v); end
        end
    end

    COL_TRUE = [0.8 0 0];
    COL_B    = [0.35 0.35 0.35];    % baseline arm (no delta a)
    COL_D    = [0 0.2 0.9];         % arm under test (with delta a)
    FS = 18; LFS = 13; AXLW = 2.0; LW = 2.0;

    f = figure('Position', [60 60 1250 780], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    ax = axes(f); hold(ax, 'on');

    for b = 2:numel(opts.bands) - 1
        xline(ax, opts.bands(b), ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0, ...
              'HandleVisibility', 'off');
    end

    % +-1 sd drawn as OUTLINES, not fills: the two bands cross each other
    % repeatedly and a translucent fill hides whichever is narrower.
    plot(ax, ctr, mu(1, :) + sg(1, :), '--', 'Color', COL_B, 'LineWidth', 1.2, ...
         'HandleVisibility', 'off');
    hb1 = plot(ax, ctr, mu(1, :) - sg(1, :), '--', 'Color', COL_B, 'LineWidth', 1.2, ...
               'DisplayName', 'no \delta a  \pm\sigma');
    plot(ax, ctr, mu(2, :) + sg(2, :), '--', 'Color', COL_D, 'LineWidth', 1.2, ...
         'HandleVisibility', 'off');
    hb2 = plot(ax, ctr, mu(2, :) - sg(2, :), '--', 'Color', COL_D, 'LineWidth', 1.2, ...
               'DisplayName', 'with \delta a  \pm\sigma');
    h0 = yline(ax, 0, '-', 'Color', COL_TRUE, 'LineWidth', LW, ...
               'DisplayName', 'a_{true}');
    h1 = plot(ax, ctr, mu(1, :), '-', 'Color', COL_B, 'LineWidth', LW + 0.4, ...
              'DisplayName', 'no \delta a');
    h2 = plot(ax, ctr, mu(2, :), '-', 'Color', COL_D, 'LineWidth', LW + 0.4, ...
              'DisplayName', 'with \delta a');

    xlabel(ax, 'true  h / R', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax, '(a_z^{hat} - a_z^{true}) / a_z^{true}   [%]', ...
           'FontSize', FS, 'FontWeight', 'bold');
    legend(ax, [h0 h1 h2 hb1 hb2], 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', ...
           'Box', 'on');
    xlim(ax, [h_lo, h_hi]);
    set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, ...
            'Box', 'on', 'TickLabelInterpreter', 'tex');
    grid(ax, 'off');

    exportgraphics(f, out, 'Resolution', 150);
    close(f);
    fprintf('[err vs h_bar] wrote %s\n', out);
end
