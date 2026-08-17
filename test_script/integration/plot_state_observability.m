function plot_state_observability(outs, slot, fname, arm_names)
%PLOT_STATE_OBSERVABILITY  Companion figure for verify_state_observability.
%   STATUS: ACTIVE | canonical style per `.claude/rules/figure-style.md`
%
%   plot_state_observability(outs, slot, fname, arm_names)
%
%   outs      1 x n cell of verify_state_observability outputs (one per arm).
%             A single struct is accepted for the one-arm case.
%   slot      the state slot to plot (must be listed in each out.free).
%   fname     output png.
%   arm_names 1 x n cell of legend names (defaults to 'arm 1', ...).
%
%   Row 1: CRLB of `slot` per window, one line per channel subset, against the
%          prior width. Log scale, because the interesting range is decades.
%          Reading rule: a marker ABOVE the prior line means the data in that
%          window says less about the slot than the prior already did.
%   Row 2: rank(O) per window with the expected value drawn. A dip is either a
%          real loss of observability or a broken instrument -- check the
%          negative control verdict in the console before believing the former.
%
%   The per-arm panels share the y limits so the arms can be compared without
%   reading two axes. Statistics (verdicts, medians) go to the console, not the
%   figure.

    if ~iscell(outs); outs = {outs}; end
    n_arm = numel(outs);
    if nargin < 4 || isempty(arm_names)
        arm_names = arrayfun(@(i) sprintf('arm %d', i), 1:n_arm, 'uni', 0);
    end
    if nargin < 3 || isempty(fname)
        fname = fullfile(fileparts(mfilename('fullpath')), ...
                         '../../test_results/temp_state_observability.png');
    end

    COL = [0 0.45 0.74; 0.47 0.67 0.19; 0.85 0.33 0.10; 0.49 0.18 0.56];
    STY = {'-o', '--s', ':^', '-.d'};

    % shared limits
    lo = inf; hi = 0;
    for a = 1:n_arm
        j = find(outs{a}.free == slot, 1);
        assert(~isempty(j), 'plot_state_observability:noSlot', ...
               'slot %d is not in arm %d''s free list.', slot, a);
        v = outs{a}.crlb(:, j, :); v = v(isfinite(v) & v > 0);
        lo = min(lo, min(v)); hi = max(hi, max(v));
        lo = min(lo, outs{a}.prior(j)); hi = max(hi, outs{a}.prior(j));
    end

    f = figure('Position', [100 100 480 * n_arm 640], 'Color', 'w');
    for a = 1:n_arm
        o = outs{a};
        j = find(o.free == slot, 1);

        ax = subplot(2, n_arm, a); hold(ax, 'on'); box(ax, 'on');
        for c = 1:numel(o.sub_names)
            y = o.crlb(:, j, c);
            y(~isfinite(y)) = hi * 4;          % Inf pinned above the top
            plot(ax, o.t_win, y, STY{min(c, numel(STY))}, 'LineWidth', 1.6, ...
                 'Color', COL(min(c, size(COL, 1)), :));
        end
        yline(ax, o.prior(j), '-', 'LineWidth', 2, 'Color', [0.8 0 0]);
        set(ax, 'YScale', 'log', 'FontSize', 11);
        ylim(ax, [lo / 3, hi * 6]); xlim(ax, [0, o.N * o.Ts]);
        xlabel(ax, 't [s]');
        ylabel(ax, sprintf('CRLB on %s  [-]', o.labels{j}));
        legend(ax, [o.sub_names, {'prior \surdP[0]'}], 'Orientation', 'horizontal', ...
               'Location', 'northoutside');
        title(ax, '');
        text(ax, 0.02, 0.06, arm_names{a}, 'Units', 'normalized', ...
             'FontSize', 11, 'FontWeight', 'bold');

        ax2 = subplot(2, n_arm, n_arm + a); hold(ax2, 'on'); box(ax2, 'on');
        plot(ax2, o.t_win, o.rank(:, 1), '-o', 'LineWidth', 1.8, 'Color', COL(1, :));
        yline(ax2, numel(o.free), '-', 'LineWidth', 2, 'Color', [0.8 0 0]);
        set(ax2, 'FontSize', 11);
        ylim(ax2, [0, numel(o.free) + 1.5]); xlim(ax2, [0, o.N * o.Ts]);
        xlabel(ax2, 't [s]'); ylabel(ax2, 'rank(O)  [-]');
        legend(ax2, {'measured', 'expected (= n free)'}, 'Orientation', 'horizontal', ...
               'Location', 'northoutside');
    end

    exportgraphics(f, fname, 'Resolution', 150);
    fprintf('saved: %s\n', fname);
end
