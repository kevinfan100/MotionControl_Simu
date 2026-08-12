function plot_formC_state_panels(o, opts)
%PLOT_FORMC_STATE_PANELS  Simulation-result figures of formC_state_da.
%
%   plot_formC_state_panels(o)                 % four canonical seed pairs
%   plot_formC_state_panels(o, opts)
%
%   One FIGURE = 3 rows x 2 COLUMNS: two seeds side by side, the layout the
%   "Simulation figures" section of formB_ws.tex uses (per panel: gain
%   tracking, relative gain error, free parameters).  Rows:
%       1  gain tracking          a_bar true (red) vs estimate (blue)
%       2  relative gain error    [%]
%       3  free parameter         da_hat with its +-sqrt(P55) band and the
%                                 -1/9 reference line (arm 'B'), or e_a with
%                                 the +-sqrt(P44) honesty band (arm 'A',
%                                 which has no free parameter)
%
%   The two columns of a figure SHARE each row's y limits, so the seeds are
%   read against the same scale.  There are no titles (house rule); the seed
%   is named in the row-1 legend of its own column.
%
%   Default pairing (Config B, 8-seed run) puts the two branches of the
%   descent-peak split on their own pages:
%       p1  27 | 31     p2  7 | 11     p3  23 | 42     p4  101 | 777
%
%   opts fields (defaults first):
%       .arm    'B'     'B' = row 3 is da_hat (the state-writing free
%                       parameter); 'A' = row 3 is the e_a honesty band.
%                       Kept so the committed cfgA figures stay regenerable.
%       .pairs  [27 31; 7 11; 23 42; 101 777]   P x 2 seeds, one row per figure
%       .names  {'p1','p2','p3','p4'}           output basenames, one per pair
%       .Ts     1/1612  [s] control period, for the time axis
%
%   Figures -> derivation/figures/formC_state_cfg<arm>_<name>.png
%
%   Units: P_a_out / P_b_out already store the STANDARD DEVIATION (do not
%   sqrt again); a_disp = a_hat_out(1,3)/a_bar_hat_out(1,3) converts the
%   display-layer gain to a_bar.  Statistics go to the console, never on the
%   figure.
%
%   See also: run_formC_state, plot_var_ahat_6state

    if nargin < 2; opts = struct(); end
    if ~isfield(opts, 'arm');   opts.arm   = 'B'; end
    if ~isfield(opts, 'pairs'); opts.pairs = [27 31; 7 11; 23 42; 101 777]; end
    if ~isfield(opts, 'names'); opts.names = {'p1', 'p2', 'p3', 'p4'};      end
    if ~isfield(opts, 'Ts');    opts.Ts    = 1/1612; end

    assert(~isempty(o) && isstruct(o), 'plot_formC_state_panels:noRun', ...
           'Pass the run struct returned by run_formC_state.');
    assert(size(opts.pairs, 2) == 2, 'plot_formC_state_panels:badPairs', ...
           'opts.pairs must be P x 2 (two seeds per figure).');
    assert(numel(opts.names) == size(opts.pairs, 1), ...
           'plot_formC_state_panels:nameCount', ...
           'opts.names has %d entries for %d pairs.', ...
           numel(opts.names), size(opts.pairs, 1));

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    AX_Z = 3;

    seeds_run = o.seeds(:).';
    for j = 1:size(opts.pairs, 1)
        sq = opts.pairs(j, :);
        rr = cell(1, 2);
        for c = 1:2
            k = find(seeds_run == sq(c), 1);
            assert(~isempty(k), 'plot_formC_state_panels:seedNotInRun', ...
                   'seed %d is not in the run (seeds %s).', sq(c), mat2str(seeds_run));
            rr{c} = o.runs{k};
        end
        out = fullfile(fig_dir, sprintf('formC_state_cfg%s_%s.png', ...
                                        upper(opts.arm), opts.names{j}));
        local_pair_figure(rr, sq, upper(opts.arm), AX_Z, opts.Ts, out);
    end
    fprintf('[panels] wrote %d figures -> %s\n', size(opts.pairs, 1), fig_dir);
end


% --------------------------------------------------------------------------
function local_pair_figure(rr, sq, arm, ax, Ts, out)
%LOCAL_PAIR_FIGURE  One 3x2 page: two seeds side by side, shared row y limits.
    C_TRUE = [0.8 0 0];  C_EST = [0 0.2 0.9];  C_BAND = [0.75 0.80 0.95];
    FS = 15; LFS = 13; AXLW = 1.6; LW = 1.8;
    DA_REF = -1/9;          % far-field anchor, reference line only (never fed)
    PAD    = 0.04;          % y-limit pad, fraction of the row's span

    % ---- pass 1: the curves, and the per-row limits over BOTH columns ----
    d = struct('t', {[], []}, 'aT', {[], []}, 'aH', {[], []}, ...
               'e', {[], []}, 'g', {[], []}, 'sg', {[], []});
    for c = 1:2
        r  = rr{c};
        a_disp = r.a_hat_out(1, ax) / r.a_bar_hat_out(1, ax);
        d(c).aT = r.a_true_out(:, ax) / a_disp;          % a_bar true [-]
        d(c).aH = r.a_bar_hat_out(:, ax);                % a_bar estimate [-]
        d(c).t  = (0:numel(d(c).aH) - 1).' * Ts;         % [s]
        d(c).e  = 100 * (d(c).aH - d(c).aT) ./ d(c).aT;  % [%]
        if arm == 'A'
            d(c).g  = d(c).aH - d(c).aT;                 % e_a [-]
            d(c).sg = r.P_a_out(:, ax) / a_disp;         % sqrt(P44), a_bar units
        else
            d(c).g  = r.b_hat_out(:, ax);                % slot 5 = da [-]
            d(c).sg = r.P_b_out(:, ax);                  % sqrt(P55) [-] (std)
        end
    end
    yl1 = local_lims([d(1).aT; d(1).aH; d(2).aT; d(2).aH], PAD);
    yl2 = local_lims([d(1).e; d(2).e], PAD);
    if arm == 'A'
        m3  = max(abs([d(1).g; d(1).sg; d(2).g; d(2).sg]));
        yl3 = [-1 1] * 1.25 * m3;
    else
        yl3 = local_lims([d(1).g + d(1).sg; d(1).g - d(1).sg; ...
                          d(2).g + d(2).sg; d(2).g - d(2).sg; DA_REF], PAD);
    end

    % ---- pass 2: draw ----------------------------------------------------
    f = figure('Position', [40 40 1500 900], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    for c = 1:2
        t = d(c).t;

        a1 = nexttile(tl, c); hold(a1, 'on');
        h1 = plot(a1, t, d(c).aT, '-', 'Color', C_TRUE, 'LineWidth', LW + 0.6, ...
                  'DisplayName', 'true');
        h2 = plot(a1, t, d(c).aH, '-', 'Color', C_EST, 'LineWidth', LW, ...
                  'DisplayName', sprintf('estimate, seed %d', sq(c)));
        if c == 1
            ylabel(a1, '$\bar{a}$', 'Interpreter', 'latex', 'FontSize', FS);
        end
        legend(a1, [h1 h2], 'Interpreter', 'latex', 'Location', 'northoutside', ...
               'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
        local_ax(a1, t, yl1, FS, AXLW, true);

        a2 = nexttile(tl, 2 + c); hold(a2, 'on');
        yline(a2, 0, '-', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
        plot(a2, t, d(c).e, '-', 'Color', C_EST, 'LineWidth', LW);
        if c == 1
            ylabel(a2, '$\Delta\bar{a}/\bar{a}_{\mathrm{true}}$  [\%]', ...
                   'Interpreter', 'latex', 'FontSize', FS);
        end
        local_ax(a2, t, yl2, FS, AXLW, true);

        a3 = nexttile(tl, 4 + c); hold(a3, 'on');
        if arm == 'A'
            fill(a3, [t; flipud(t)], [d(c).sg; flipud(-d(c).sg)], C_BAND, ...
                 'EdgeColor', 'none', 'FaceAlpha', 0.75, ...
                 'DisplayName', '$\pm\sqrt{P_{44}}$');
            plot(a3, t, d(c).g, '-', 'Color', C_EST, 'LineWidth', LW, ...
                 'DisplayName', '$e_{\bar{a}}$');
            if c == 1
                ylabel(a3, '$e_{\bar{a}}$', 'Interpreter', 'latex', 'FontSize', FS);
            end
        else
            fill(a3, [t; flipud(t)], [d(c).g + d(c).sg; flipud(d(c).g - d(c).sg)], ...
                 C_BAND, 'EdgeColor', 'none', 'FaceAlpha', 0.75, ...
                 'DisplayName', '$\pm\sqrt{P_{55}}$');
            plot(a3, t, d(c).g, '-', 'Color', C_EST, 'LineWidth', LW, ...
                 'DisplayName', '$\widehat{\delta a}$');
            yline(a3, DA_REF, '-', 'Color', C_TRUE, 'LineWidth', LW, ...
                  'DisplayName', '$-1/9$');
            if c == 1
                ylabel(a3, '$\widehat{\delta a}$', 'Interpreter', 'latex', ...
                       'FontSize', FS);
            end
        end
        legend(a3, 'Interpreter', 'latex', 'Location', 'northoutside', ...
               'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
        xlabel(a3, '$t$  [s]', 'Interpreter', 'latex', 'FontSize', FS);
        local_ax(a3, t, yl3, FS, AXLW, false);
    end

    exportgraphics(tl, out, 'Resolution', 150, 'Padding', 'figure');
    close(f);

    % ---- console statistics (never on the figure) ------------------------
    for c = 1:2
        fprintf('cfg%s seed %3d : rel err RMS %6.3f %%  peak %7.3f %%', ...
                arm, sq(c), sqrt(mean(d(c).e.^2)), max(abs(d(c).e)));
        if arm == 'B'
            fprintf('  |  da_end %+.5f (target %+.5f)  sqrt(P55)_end %.5f', ...
                    d(c).g(end), DA_REF, d(c).sg(end));
        end
        fprintf('\n');
    end
    fprintf('  shared y limits: row1 [%.4f %.4f]  row2 [%.3f %.3f]  row3 [%.4f %.4f]  -> %s\n', ...
            yl1(1), yl1(2), yl2(1), yl2(2), yl3(1), yl3(2), out);
end


function yl = local_lims(v, pad)
%LOCAL_LIMS  Padded [min max] over the pooled data of BOTH columns of a row.
    v  = v(isfinite(v));
    lo = min(v); hi = max(v);
    span = hi - lo;
    if span <= 0; span = max(abs(hi), 1) * 0.1; end
    yl = [lo - pad * span, hi + pad * span];
end


function local_ax(a, t, yl, FS, AXLW, hide_x)
    xlim(a, [t(1) t(end)]);
    ylim(a, yl);
    set(a, 'FontSize', FS, 'LineWidth', AXLW, 'Box', 'on', ...
           'TickLabelInterpreter', 'latex');
    if hide_x; set(a, 'XTickLabel', []); end
    grid(a, 'off');
end
