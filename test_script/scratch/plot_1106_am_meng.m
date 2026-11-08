function plot_1106_am_meng(seed)
% PURPOSE: single-sample gain readout a-bar_m under the Meng law (run_1106_meng_law_1d meng arm, estimator in the loop)
%   overlaid with the readout formulas evaluated at the TRUE gain: E[a_m] = a, Var(a_m) = 2 (a + xi)^2,
%   C_T, C_n, xi of motion_ctrl/sec3_am.tex (constants of the past-input law). No averaging in the figure; window ratios on the console.
% EXPIRES: with run_1106_meng_law_1d.m.
    if nargin < 1; seed = 1; end
    R = run_1106_meng_law_1d(struct('seeds', seed, 'laws', {{'meng'}}, 'plot', false));
    r = R.meng(1);
    lc = 0.4;  apd = 0.05;  b = 1 - apd;  t0 = 0.05;  IF = 2.72;   % IF: thermal chain of the past-input law (run_1106 local_IF)
    C_T = b^2 * (2 * b * (1 - lc) / (1 - b * lc) + 2 / ((2 - apd) * (1 + lc) * (1 - b * lc)));
    C_n = 2 * b^2 / (2 - apd) * (1 + b^2 * apd * (1 - lc) / (1 - b * lc) + (1 - lc)^2 / ((1 + lc) * (1 - b * lc)));
    xi = C_n / C_T * r.sn2 / r.kap;
    m = r.t >= t0 & r.a > 0;  t = r.t(m);  a = r.a(m);  am = r.am(m);  ah = r.ah(m);
    sd = sqrt(2) * (a + xi);  sq = (am - a).^2;  vth = 2 * (a + xi).^2;
    % ---- console: 0.2 s windows ----
    fprintf('\nseed %d  Meng law  xi = %.4f  C_T = %.4f  C_n = %.4f\n', seed, xi, C_T, C_n);
    fprintf('%-12s %8s %14s %16s\n', 'window [s]', 'mean a', 'mean ratio', 'var ratio');
    edges = [t0, 0.2:0.2:1.0, 1.2];
    for i = 1:numel(edges) - 1
        w = t >= edges(i) & t < edges(i + 1);  N = sum(w);  Ne = N / IF;
        mr = mean(am(w)) / mean(a(w));  mse = std(am(w)) / sqrt(Ne) / mean(a(w));
        vr = mean(sq(w)) / mean(vth(w));  vse = std(sq(w)) / sqrt(Ne) / mean(vth(w));
        fprintf('%4.2f-%4.2f    %8.3f   %5.3f +- %5.3f   %5.3f +- %5.3f\n', edges(i), edges(i + 1), mean(a(w)), mr, mse, vr, vse);
    end
    % ---- figure ----
    COL_TRUE = [0.85 0.10 0.10];  COL_HAT = [0.00 0.30 0.85];  COL_MEAS = [0.45 0.70 0.95];  FS = 18;
    fh = figure('Position', [60 60 1400 950], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile(tl);  hold on;
    yl = [-0.5 3];  ncl = sum(am > yl(2) | am < yl(1));
    h1 = plot(t, min(max(am, yl(1)), yl(2)), '.', 'Color', COL_MEAS, 'MarkerSize', 5);
    h2 = plot(t, a, '-', 'Color', COL_TRUE, 'LineWidth', 2.5);
    h3 = plot(t, a + sd, '--', 'Color', COL_TRUE, 'LineWidth', 1.5);  plot(t, a - sd, '--', 'Color', COL_TRUE, 'LineWidth', 1.5);
    h4 = plot(t, ah, '-', 'Color', COL_HAT, 'LineWidth', 2);
    ylim(yl);  ylabel('$\bar{a}$', 'Interpreter', 'latex');
    legend([h1 h4 h2 h3], {'$\bar{a}_m$', '$\hat{\bar{a}}$', '$\bar{a}$', '$\bar{a} \pm \sqrt{2}\,(\bar{a}+\xi)$'}, 'Interpreter', 'latex', ...
           'Location', 'northoutside', 'Orientation', 'horizontal');
    nexttile(tl);  hold on;
    g1 = semilogy(t, max(sq, 1e-6), '.', 'Color', COL_MEAS, 'MarkerSize', 5);
    g2 = semilogy(t, vth, '-', 'Color', COL_TRUE, 'LineWidth', 2.5);
    set(gca, 'YScale', 'log');  ylim([1e-6 1e2]);
    ylabel('$(\bar{a}_m-\bar{a})^2$', 'Interpreter', 'latex');  xlabel('Time (sec)');
    legend([g1 g2], {'$(\bar{a}_m-\bar{a})^2$', '$2\,(\bar{a}+\xi)^2$'}, 'Interpreter', 'latex', 'Location', 'northoutside', 'Orientation', 'horizontal');
    set(findall(fh, 'Type', 'axes'), 'FontSize', FS, 'FontWeight', 'bold', 'Box', 'on', 'XGrid', 'off', 'YGrid', 'off', 'XLim', [0 1.2]);
    outd = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'test_results', 'temp_1106_meng');  if ~exist(outd, 'dir'); mkdir(outd); end
    fname = fullfile(outd, sprintf('fig_1106_am_meng_s%d.png', seed));
    exportgraphics(fh, fname, 'Resolution', 150);  close(fh);
    fprintf('points outside the row-1 axis [%g %g]: %d of %d\nfigure: %s\n', yl(1), yl(2), ncl, numel(am), fname);
end
