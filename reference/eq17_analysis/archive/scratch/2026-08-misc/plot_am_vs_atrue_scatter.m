% STATUS: ACTIVE (scratch figure) | PURPOSE: the two scatters that move in
%   OPPOSITE directions along the run -- the true gain's own fluctuation
%   sd(a_true), which GROWS toward the wall, and the readout scatter sd(a_hm),
%   which SHRINKS. Companion to plot_dhm_to_ahm.
%
%   sd(a_true) = |da/dh| * sd(h) : near the wall the curve steepens faster than
%   the damping quiets the particle, so the true gain's own jitter grows.
%   sd(a_hm)  = sqrt(K_var*IF) * (a_h + xi) : MULTIPLICATIVE in a_h, so it
%   follows a_h down. The two are different quantities, not a contradiction.
%
%   Both scatters are taken after removing the same 0.5 s moving mean, so the
%   commanded descent is not counted as fluctuation.
function out = plot_am_vs_atrue_scatter(r, ax, W)

    if nargin < 2 || isempty(ax); ax = 3;   end
    if nargin < 3 || isempty(W);  W  = 800; end     % 0.5 s at 1600 Hz

    o = plot_dhm_to_ahm(r, ax, struct('tag', '_tmp'));
    i = o.i;  t = o.t;
    aT = o.ah(i);  aM = o.ahm(i);
    hb = r.h_bar_true_out(i, 1);
    aT_trend = movmean(aT, W);
    sd_aT = movstd(aT - aT_trend, W);
    sd_aM = movstd(aM - movmean(aM, W), W);

    COL_TRUE = [0.8 0 0];  COL_MEAS = [0.45 0.72 0.95];  COL_HAT = [0 0.2 0.9];
    FS = 18; LFS = 13; AXLW = 2.0;
    f = figure('Position', [10 10 1500 1000], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    a = nexttile(tl); hold(a, 'on');
    h1 = plot(a, t, sd_aT, '-', 'Color', COL_TRUE, 'LineWidth', 2.6);
    h2 = plot(a, t, sd_aM, '-', 'Color', COL_MEAS, 'LineWidth', 2.6);
    h3 = plot(a, t, aT_trend, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 2.0);
    set(a, 'YScale', 'log');
    legend(a, [h3 h1 h2], {'a_h', 'sd( a_h )   true fluctuation', 'sd( a_{hm} )   readout scatter'}, ...
        'Location', 'northoutside', 'Orientation', 'horizontal', ...
        'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel(a, '(\mum/pN)', 'FontSize', FS, 'FontWeight', 'bold');
    set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    grid(a, 'off'); xlim(a, [t(1) t(end)]); set(a, 'XTickLabel', []);

    a = nexttile(tl); hold(a, 'on');
    h1 = plot(a, t, 100*sd_aT./aT_trend, '-', 'Color', COL_TRUE, 'LineWidth', 2.6);
    h2 = plot(a, t, 100*sd_aM./aT_trend, '-', 'Color', COL_MEAS, 'LineWidth', 2.6);
    set(a, 'YScale', 'log');
    legend(a, [h1 h2], {'sd( a_h ) / a_h', 'sd( a_{hm} ) / a_h'}, ...
        'Location', 'northoutside', 'Orientation', 'horizontal', ...
        'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel(a, 'relative scatter  (%)', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(a, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
    grid(a, 'off'); xlim(a, [t(1) t(end)]);

    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    fn = fullfile(od, 'am_vs_atrue_scatter.png');
    exportgraphics(f, fn, 'Resolution', 150);  close(f);
    delete(fullfile(od, 'dhm_to_ahm_tmp.png'));
    fprintf('figure -> %s\n', fn);
    out = struct('t', t, 'hb', hb, 'aT', aT_trend, 'sd_aT', sd_aT, 'sd_aM', sd_aM);
end
