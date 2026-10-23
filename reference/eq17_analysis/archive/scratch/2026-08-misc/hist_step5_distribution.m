% STATUS: ACTIVE (scratch figure) | PURPOSE: the raw distribution of step 5's
%   output, sigma-hat^2. Pure measurement: no theoretical curve, nothing fitted,
%   equal-width bars, y = percentage of samples.
%
%   The legend's third number counts DATA POINTS, not area: 58.1 %% of the
%   individual sigma-hat^2 samples are smaller than their own mean. A symmetric
%   distribution would give 50 %%; the excess is the right skew.
%
%   Window: the far-field stationary stretch t = 1..4 s, where a_h moves only
%   6 % across the window, so the histogram is not smeared by the ramp.
%   All seeds are pooled.
function out = hist_step5_distribution(O, ax, tlim)

    if nargin < 2 || isempty(ax);   ax = 3;        end
    if nargin < 3 || isempty(tlim); tlim = [1 4];  end

    s2 = [];  aT = [];
    for q = 1:numel(O.runs)
        o = plot_dhm_to_ahm(O.runs{q}, ax, struct('no_figure', true, 'tlim', tlim));
        s2 = [s2; o.s2(o.i)];   %#ok<AGROW>
        aT = [aT; o.ah(o.i)];   %#ok<AGROW>
    end
    st = struct('mean', mean(s2), 'median', median(s2), 'below', mean(s2 < mean(s2)), ...
                'p05', prctile(s2,5), 'p95', prctile(s2,95));
    fprintf('\npooled %d samples from %d seeds, t = %.1f..%.1f s  (a_h moves %.1f %% here)\n', ...
            numel(s2), numel(O.runs), tlim, 100*(aT(end)/aT(1)-1));
    fprintf('  sigma-hat^2  mean %.4e  median %.4e  %% below mean %.1f\n', st.mean, st.median, 100*st.below);

    out = struct('s2', s2, 'stat', st);
    arm = sprintf('a_{pd} %.4g , a_{cov} %.4g', O.runs{1}.ctrl_const.a_pd, O.runs{1}.ctrl_const.a_cov);
    local_page(out, tlim, numel(O.runs), arm);
end

% ---------------------------------------------------------------------
function local_page(o, tlim, ns, arm)
    FS = 18; LFS = 14; AXLW = 2.0;
    SH = [char(963) char(770)];
    COL = [0.85 0.33 0.10];
    y = o.s2;  NB = 70;
    ed = linspace(0, prctile(y, 99.7), NB);
    bw = ed(2) - ed(1);

    f = figure('Position', [10 10 1500 750], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    a = axes(f); hold(a, 'on');
    h = histogram(a, y, 'BinEdges', ed, 'Normalization', 'probability', ...
                  'FaceColor', COL, 'EdgeColor', 'none', 'FaceAlpha', 0.95);
    yt = get(a, 'YTick');
    set(a, 'YTickLabel', arrayfun(@(v) sprintf('%.0f', 100*v), yt, 'uni', 0));
    legend(a, h, {sprintf('%s^2_{\\delta h_{mr}}      mean %.3e ,  median %.3e ,  %.1f %% of points below the mean', ...
        SH, o.stat.mean, o.stat.median, 100*o.stat.below)}, ...
        'Location','northoutside','Orientation','horizontal', ...
        'FontSize',LFS,'FontWeight','bold','Box','on');
    ylabel(a, 'samples  (%)', 'FontSize', FS, 'FontWeight', 'bold');
    % The last edge is the 99.7th percentile, so 0.30 % of the points sit beyond
    % the last bar and the bars sum to 99.70 %, not 100 %. Reported to the
    % console rather than on the axis, where it collided with the x10^-4 label.
    xlabel(a, [SH sprintf('^2_{\\delta h_{mr}}   (\\mum^2)          bar width %.3e \\mum^2', bw)], ...
           'FontSize', FS, 'FontWeight', 'bold');
    fprintf('  bars sum to %.2f %% ; %.2f %% of points lie beyond the last bar\n', ...
            100*mean(y <= ed(end)), 100*mean(y > ed(end)));
    set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    grid(a, 'off');

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    fn = fullfile(od, sprintf('hist_step5_t%g_%g.png', tlim(1), tlim(2)));
    exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure -> %s\n', fn);
end
