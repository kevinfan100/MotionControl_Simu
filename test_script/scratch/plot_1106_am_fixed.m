function plot_1106_am_fixed(seed)
% PURPOSE: Meng law (run_1106_meng_law_1d meng arm, estimator in the loop), fixed point at h-bar 10 and 1.1, 10 s, statistics 3-10 s:
%   delta-x-bar_r against sigma^2 = C_T kappa_T a + C_n sigma_n^2 (normal pdf) and the single-sample readout a-bar_m against
%   a_m = (a + xi) z^2 - xi, z ~ N(0,1) (mean a, variance 2 (a + xi)^2); C_T, C_n, xi of motion_ctrl/sec3_am.tex, theory at the TRUE gain.
%   Layout as run_1104_force_var.m (columns = heights; per quantity: time series + pdf).
% EXPIRES: with run_1106_meng_law_1d.m.
    if nargin < 1; seed = 1; end
    H = [10 1.1];  t0 = 3;  lc = 0.4;  apd = 0.05;  b = 1 - apd;  IF = 2.72;
    C_T = b^2 * (2 * b * (1 - lc) / (1 - b * lc) + 2 / ((2 - apd) * (1 + lc) * (1 - b * lc)));
    C_n = 2 * b^2 / (2 - apd) * (1 + b^2 * apd * (1 - lc) / (1 - b * lc) + (1 - lc)^2 / ((1 + lc) * (1 - b * lc)));
    cE = [0 0.25 0.85];  cT = [0.85 0 0];  cH = [0.6 0.7 0.9];
    f = figure('Color', 'w', 'Position', [40 40 1700 1500], 'Visible', 'off');
    tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact', 'TileIndexing', 'columnmajor');
    fprintf('\nseed %d  Meng law, estimator in the loop, statistics over %g-10 s\n', seed, t0);
    for q = 1:2
        R = run_1106_meng_law_1d(struct('seeds', seed, 'laws', {{'meng'}}, 'plot', false, 'h0', H(q), 'h1', H(q), 'T_move', 1, 'T_hold', 9));
        r = R.meng(1);  m = r.t >= t0 & r.a > 0;  N = sum(m);  Ne = N / IF;
        nm = r.Rr * 1e3;  xi = C_n / C_T * r.sn2 / r.kap;  a = mean(r.a(m));
        sr = sqrt(C_T * r.kap * a + C_n * r.sn2) * nm;          % sigma of delta-x_r [nm]
        dr = r.dxr(m) * nm;  am = r.am(m);  tt = r.t(m);  sa = sqrt(2) * (a + xi);
        fprintf('h = %4.1f R: a = %.4f  a-hat mean %.4f  xi = %.4f  N = %d\n', H(q), a, mean(r.ah(m)), xi, N);
        fprintf('   delta-x_r : var ratio %.3f +- %.3f   (sigma derivation %.2f nm, simulation %.2f nm)\n', var(dr) / sr^2, sqrt(2 / Ne), sr, std(dr));
        fprintf('   a_m       : mean ratio %.3f +- %.3f   var ratio %.3f +- %.3f\n', mean(am) / a, std(am) / sqrt(Ne) / a, ...
                var(am) / sa^2, std((am - mean(am)).^2) / sqrt(Ne) / sa^2);
        k = 1:3:N;  YL = 5 * sr;
        % row 1: delta-x_r time series
        ax = nexttile(tl);  h1 = plot(ax, tt(k), dr(k), 'Color', cE, 'LineWidth', 0.5);  hold(ax, 'on');
        h2 = yline(ax, sr, '--', 'Color', cT, 'LineWidth', 1.6);  yline(ax, -sr, '--', 'Color', cT, 'LineWidth', 1.6);
        ylim(ax, [-YL YL]);  xlim(ax, [t0 10]);  ylabel(ax, '\delta x_r  [nm]');  xlabel(ax, 'Time (sec)');
        lg = legend(ax, [h1 h2], {'simulation', '\pm\sigma, derivation'}, 'Location', 'northoutside', 'Orientation', 'horizontal');  title(lg, sprintf('h = %.1f R', H(q)));
        % row 2: delta-x_r pdf
        bx = nexttile(tl);  histogram(bx, dr, 80, 'Normalization', 'pdf', 'FaceColor', cH, 'EdgeColor', 'none');  hold(bx, 'on');
        xx = linspace(-YL, YL, 400);  plot(bx, xx, exp(-xx.^2 / (2 * sr^2)) / (sr * sqrt(2 * pi)), 'Color', cT, 'LineWidth', 1.8);
        xlim(bx, [-YL YL]);  xlabel(bx, '\delta x_r  [nm]');  ylabel(bx, 'pdf');
        % row 3: a_m time series
        yhi = a + 6 * sa;  ylo = -1.5 * xi;
        cx = nexttile(tl);  g1 = plot(cx, tt(k), am(k), 'Color', cE, 'LineWidth', 0.5);  hold(cx, 'on');
        g2 = yline(cx, a, '--', 'Color', cT, 'LineWidth', 1.6);  g3 = yline(cx, a + sa, ':', 'Color', cT, 'LineWidth', 2);
        ylim(cx, [ylo yhi]);  xlim(cx, [t0 10]);  ylabel(cx, 'a_m / a_{nom}');  xlabel(cx, 'Time (sec)');
        legend(cx, [g1 g2 g3], {'simulation', 'mean, derivation', 'mean + \sigma, derivation'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
        % row 4: a_m pdf (shifted, scaled chi-square with 1 dof)
        dx_ = nexttile(tl);  histogram(dx_, am, 'BinLimits', [-xi yhi], 'NumBins', 80, 'Normalization', 'pdf', 'FaceColor', cH, 'EdgeColor', 'none');  hold(dx_, 'on');
        yy = linspace(-xi + 1e-4 * (a + xi), yhi, 600);  s = (yy + xi) / (a + xi);
        plot(dx_, yy, exp(-s / 2) ./ sqrt(2 * pi * s) / (a + xi), 'Color', cT, 'LineWidth', 1.8);
        xlim(dx_, [ylo yhi]);  ylim(dx_, [0 1.5 / (a + xi)]);  xlabel(dx_, 'a_m / a_{nom}');  ylabel(dx_, 'pdf');
        for c = [ax bx cx dx_]; box(c, 'on');  set(c, 'FontSize', 14, 'FontWeight', 'bold', 'LineWidth', 1.2); end
        fprintf('   a_m outside [%.3f, %.3f]: %d of %d\n', ylo, yhi, sum(am > yhi | am < ylo), N);
    end
    outd = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'test_results', 'temp_1106_meng');  if ~exist(outd, 'dir'); mkdir(outd); end
    fname = fullfile(outd, sprintf('fig_1106_am_fixed_s%d.png', seed));
    exportgraphics(f, fname, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', fname);
end
