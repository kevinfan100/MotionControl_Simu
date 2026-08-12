function plot_formC_state_panels(oA, oB, opts)
%PLOT_FORMC_STATE_PANELS  The four simulation-result panels of formC_state_da.
%
%   Config A (da locked at -1/9) and Config B (da free from 0), seeds 7 and 11,
%   canonical scenario, z axis.  Three rows per panel:
%       1  gain tracking          a_bar true (red) vs estimate (blue)
%       2  relative gain error    [%]
%       3  A: e_a with the +-sqrt(P44) honesty band
%          B: da_hat with the -1/9 reference line and its +-sqrt(P55) band
%
%   Row 3 differs by arm on purpose: Config A has no free parameter, so the
%   slot that would show it carries the covariance honesty instead.
%
%   Figures -> derivation/figures/formC_state_cfg{A,B}_s{07,11}.png
%
%   Style: house rules (no grid, box on, legend northoutside horizontal, no
%   title, stats to console).
%
%   See also: run_formC_state, plot_var_ahat_6state

    if nargin < 3; opts = struct(); end
    if ~isfield(opts, 'seeds_plot'); opts.seeds_plot = [7 11]; end
    if ~isfield(opts, 'Ts');         opts.Ts         = 1/1612; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    AX_Z = 3;
    seeds_all = [7 11 23 42 101 777];

    for arm = 'AB'
        o = oA; if arm == 'B'; o = oB; end
        for sq = opts.seeds_plot
            k = find(seeds_all == sq, 1);
            out = fullfile(fig_dir, sprintf('formC_state_cfg%c_s%02d.png', arm, sq));
            local_panel(o.runs{k}, arm, sq, AX_Z, opts.Ts, out);
        end
    end
    fprintf('[panels] wrote 4 figures -> %s\n', fig_dir);
end

% --------------------------------------------------------------------------
function local_panel(r, arm, sq, ax, Ts, out)
    C_TRUE = [0.8 0 0];  C_EST = [0 0.2 0.9];  C_BAND = [0.75 0.80 0.95];
    FS = 15; LFS = 13; AXLW = 1.6; LW = 1.8;

    a_disp = r.a_hat_out(1, ax) / r.a_bar_hat_out(1, ax);
    aT = r.a_true_out(:, ax) / a_disp;        % a_bar true
    aH = r.a_bar_hat_out(:, ax);
    sA = r.P_a_out(:, ax) / a_disp;           % sqrt P44 in a_bar units
    N  = numel(aH);  t = (0:N-1).' * Ts;
    e  = 100 * (aH - aT) ./ aT;

    f = figure('Position', [60 60 1000 900], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile; hold(ax1, 'on');
    h1 = plot(ax1, t, aT, '-', 'Color', C_TRUE, 'LineWidth', LW + 0.6, ...
              'DisplayName', 'true');
    h2 = plot(ax1, t, aH, '-', 'Color', C_EST, 'LineWidth', LW, ...
              'DisplayName', 'estimate');
    ylabel(ax1, '$\bar{a}$', 'Interpreter', 'latex', 'FontSize', FS);
    legend(ax1, [h1 h2], 'Interpreter', 'latex', 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
    local_ax(ax1, t, FS, AXLW, true);

    ax2 = nexttile; hold(ax2, 'on');
    yline(ax2, 0, '-', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0);
    plot(ax2, t, e, '-', 'Color', C_EST, 'LineWidth', LW);
    ylabel(ax2, '$\Delta\bar{a}/\bar{a}_{\mathrm{true}}$  [\%]', ...
           'Interpreter', 'latex', 'FontSize', FS);
    local_ax(ax2, t, FS, AXLW, true);

    ax3 = nexttile; hold(ax3, 'on');
    if arm == 'A'
        band = [ sA; flipud(-sA) ];
        fill(ax3, [t; flipud(t)], band, C_BAND, 'EdgeColor', 'none', ...
             'FaceAlpha', 0.75, 'DisplayName', '$\pm\sqrt{P_{44}}$');
        plot(ax3, t, aH - aT, '-', 'Color', C_EST, 'LineWidth', LW, ...
             'DisplayName', '$e_{\bar{a}}$');
        ylabel(ax3, '$e_{\bar{a}}$', 'Interpreter', 'latex', 'FontSize', FS);
        ylim(ax3, [-1 1] * 1.25 * max(abs(aH - aT)));
    else
        da = r.b_hat_out(:, ax);  sD = r.P_b_out(:, ax);
        fill(ax3, [t; flipud(t)], [da + sD; flipud(da - sD)], C_BAND, ...
             'EdgeColor', 'none', 'FaceAlpha', 0.75, ...
             'DisplayName', '$\pm\sqrt{P_{55}}$');
        plot(ax3, t, da, '-', 'Color', C_EST, 'LineWidth', LW, ...
             'DisplayName', '$\widehat{\delta a}$');
        yline(ax3, -1/9, '-', 'Color', C_TRUE, 'LineWidth', LW, ...
              'DisplayName', '$-1/9$');
        ylabel(ax3, '$\widehat{\delta a}$', 'Interpreter', 'latex', 'FontSize', FS);
    end
    legend(ax3, 'Interpreter', 'latex', 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
    xlabel(ax3, '$t$  [s]', 'Interpreter', 'latex', 'FontSize', FS);
    local_ax(ax3, t, FS, AXLW, false);

    exportgraphics(tl, out, 'Resolution', 150, 'Padding', 'figure');
    close(f);

    fprintf('cfg%c seed %3d : rel err RMS %6.3f %%  peak %7.3f %%', ...
            arm, sq, sqrt(mean(e.^2)), max(abs(e)));
    if arm == 'B'
        fprintf('  |  da_end %+.5f (target %+.5f)', r.b_hat_out(end, ax), -1/9);
    end
    fprintf('\n');
end

function local_ax(a, t, FS, AXLW, hide_x)
    xlim(a, [t(1) t(end)]);
    set(a, 'FontSize', FS, 'LineWidth', AXLW, 'Box', 'on', ...
           'TickLabelInterpreter', 'latex');
    if hide_x; set(a, 'XTickLabel', []); end
    grid(a, 'off');
end
