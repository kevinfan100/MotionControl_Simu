function T = plot_1106_am_heights(seeds)
% PURPOSE: Meng law (run_1106_meng_law_1d meng arm, estimator in the loop), fixed points at 10 heights, per height 3 seeds x 3 s
%   (run 3.5 s, first 0.5 s dropped): mean and variance of the single-sample readout a-bar_m against the formulas at the TRUE gain,
%   E[a_m] = a, Var(a_m) = 2 (a + xi)^2 (C_T, C_n, xi of motion_ctrl/sec3_am.tex). Rows: mean | variance (log) | ratio to the formula.
% EXPIRES: with run_1106_meng_law_1d.m.
    if nargin < 1; seeds = 1:3; end
    H = [1.05 1.1 1.2 1.35 1.5 2 3 5 10 20];  t0 = 0.5;  lc = 0.4;  apd = 0.05;  b = 1 - apd;  IF = 2.72;
    C_T = b^2 * (2 * b * (1 - lc) / (1 - b * lc) + 2 / ((2 - apd) * (1 + lc) * (1 - b * lc)));
    C_n = 2 * b^2 / (2 - apd) * (1 + b^2 * apd * (1 - lc) / (1 - b * lc) + (1 - lc)^2 / ((1 + lc) * (1 - b * lc)));
    nH = numel(H);  T = struct('h', H(:), 'a', nan(nH, 1), 'm', nan(nH, 1), 'mse', nan(nH, 1), 'v', nan(nH, 1), 'vse', nan(nH, 1), ...
                               'ahr', nan(nH, 1), 'K3', nan(nH, 1));
    for q = 1:nH
        R = run_1106_meng_law_1d(struct('seeds', seeds, 'laws', {{'meng'}}, 'h0', H(q), 'h1', H(q), 'T_move', 1, 'T_hold', 2.5));
        am = [];  a = [];  ah = [];  k3 = [];
        for is = 1:numel(seeds)
            r = R.meng(is);  m = r.t >= t0 & r.a > 0;
            am = [am; r.am(m)];  a = [a; r.a(m)];  ah = [ah; r.ah(m)];  k3 = [k3; r.K3(m)];   %#ok<AGROW>
        end
        xi = C_n / C_T * r.sn2 / r.kap;  Ne = numel(am) / IF;  vth = mean(2 * (a + xi).^2);
        T.a(q) = mean(a);  T.m(q) = mean(am);  T.mse(q) = std(am) / sqrt(Ne);
        T.v(q) = var(am);  T.vse(q) = std((am - mean(am)).^2) / sqrt(Ne);  T.vth(q, 1) = vth;
        T.ahr(q) = mean(ah) / mean(a);  T.K3(q) = mean(k3);  T.xi = xi;
    end
    fprintf('\nMeng law, estimator in the loop, seeds %s, %g-3.5 s per run, xi = %.4f\n', mat2str(seeds), t0, T.xi);
    fprintf('%6s %7s %16s %16s %8s %6s\n', 'h', 'a', 'mean ratio', 'var ratio', 'a-hat/a', 'K3');
    for q = 1:nH
        sf = 1 + (T.m(q) / T.a(q) - 1) * T.a(q) / (T.a(q) + T.xi);   % delta-x_r variance factor implied by the mean ratio
        fprintf('%6.2f %7.4f   %5.3f +- %5.3f   %5.3f +- %5.3f   %6.3f %6.3f   s %.3f  s^2 %.3f\n', H(q), T.a(q), T.m(q) / T.a(q), T.mse(q) / T.a(q), ...
                T.v(q) / T.vth(q), T.vse(q) / T.vth(q), T.ahr(q), T.K3(q), sf, sf^2);
    end
    % ---- figure ----
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 22;  LFS = 17;  AXLW = 2.2;  AB = char(257);
    hc = logspace(log10(1.02), log10(25), 300)';  ac = zeros(size(hc));
    for i = 1:numel(hc); [~, cp] = calc_correction_functions(hc(i)); ac(i) = 1 / cp; end
    f = figure('Position', [40 40 1500 2200], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(4, 1);
    ax(1) = nexttile(tl);  hold on; box on;
    h1 = plot(ax(1), hc, ac, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'derivation');
    h2 = errorbar(ax(1), H, T.m, T.mse, 'o', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 9, 'LineWidth', 1.8, 'DisplayName', 'simulation');
    lg = legend(ax(1), [h2 h1], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    lg.Title.String = sprintf('plane wall, Meng law, %d seeds x 3 s', numel(seeds));
    ylabel(ax(1), ['mean of ' AB '_m'], 'FontSize', FS, 'FontWeight', 'bold');  ylim(ax(1), [0 1.1]);
    ax(2) = nexttile(tl);  hold on; box on;
    plot(ax(2), hc, 2 * (ac + T.xi).^2, '-', 'Color', cT, 'LineWidth', 5.0);
    errorbar(ax(2), H, T.v, T.vse, 'o', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 9, 'LineWidth', 1.8);
    set(ax(2), 'YScale', 'log');  ylabel(ax(2), ['variance of ' AB '_m'], 'FontSize', FS, 'FontWeight', 'bold');
    ax(3) = nexttile(tl);  hold on; box on;
    yline(ax(3), 1, '-', 'Color', cT, 'LineWidth', 3, 'HandleVisibility', 'off');
    errorbar(ax(3), H, T.m ./ T.a, T.mse ./ T.a, 'o', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 9, 'LineWidth', 1.8);
    ylabel(ax(3), {'mean ratio', 'simulation / derivation'}, 'FontSize', FS, 'FontWeight', 'bold');  ylim(ax(3), [0.95 1.2]);  yticks(ax(3), 0.95:0.05:1.2);
    ax(4) = nexttile(tl);  hold on; box on;
    yline(ax(4), 1, '-', 'Color', cT, 'LineWidth', 3, 'HandleVisibility', 'off');
    errorbar(ax(4), H, T.v ./ T.vth, T.vse ./ T.vth, 's', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 9, 'LineWidth', 1.8);
    ylabel(ax(4), {'variance ratio', 'simulation / derivation'}, 'FontSize', FS, 'FontWeight', 'bold');  ylim(ax(4), [0.95 1.2]);  yticks(ax(4), 0.95:0.05:1.2);
    xlabel(ax(4), ['h' char(772) ' = h / R'], 'FontSize', FS, 'FontWeight', 'bold');
    for q = 1:4; set(ax(q), 'XScale', 'linear', 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  xlim(ax(q), [0 21]);  grid(ax(q), 'off');  xticks(ax(q), [1 5 10 15 20]); end
    od = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'test_results', 'temp_1106_meng');  if ~exist(od, 'dir'); mkdir(od); end
    out = fullfile(od, 'fig_1106_am_heights.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end
