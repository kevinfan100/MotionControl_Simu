% PURPOSE (2026-09-16, user: "present the cell's Pade the same way"; coordinate revised 2026-09-18): the two-point
%   rational form applied to Meng's Fig. 11 cell curve (digitised CSV, reference/eq17_analysis/data). The fit is done in
%   the surface gap g = x - 1 with x = w_bar - w_bar_s the Fig. 11 axis (w_bar_s = the visible cell surface, its zero);
%   the RESULTS are printed and drawn in x (the document's coordinate), constants mapped by g = x - 1:
%       [1/1]  a = (x + c0)/(x + d1)                      c0_x = c0_g - 1, d1_x = d1_g - 1;   kappa = d1 - c0 (both)
%       [2/2]  a = (x^2 + c1 x + c0)/(x^2 + d1 x + d2)    (c0, c1, d1, d2)_x = (1 - c1 + c0, c1 - 2, d1 - 2, 1 - d1 + d2)_g
%   a(inf) = 1 is built in (equal leading coefficients). Coefficients by least squares on the data over 1.1-5.5 R
%   (linearised LS, then fminsearch on the rms), so these are DATA numbers, not derivations. The gain of the [1/1] form
%   vanishes at x = -c0_x (cell: -0.056, one radius below the visible-surface contact x = 1).
%   Output reference/eq17_analysis/derivation/figures/cell_pade.png | EXPIRES: with the s / Pade line | 產線改動不會自動跟上
function out = probe_cell_pade()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    T  = readtable(fullfile(root, 'reference', 'eq17_analysis', 'data', 'meng_11072733_fig11_three_surfaces.csv'));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    m = strcmp(T.surface, 'cell');  x = T.hd_R(m);  y = T.lambda_norm(m);
    k = x >= 1.1 & x <= 5.5;  x = x(k);  y = y(k);  g = x - 1;          % actual wall at w_bar = 1
    band = g >= 0.10 & g <= 2.32;

    % --- [1/1] with free contact: a (g + d1) = g + c0  ->  d1 a - c0 = g - a g -------------
    A1 = [y, -ones(size(g))];  p1 = A1 \ (g - y .* g);                % [d1; c0]
    f1 = @(p, gq) (gq + p(2)) ./ (gq + p(1));
    p1 = fminsearch(@(p) rms(y - f1(p, g)), p1, optimset('TolX', 1e-10, 'TolFun', 1e-12));
    % --- [2/2] with free contact: a (g^2 + d1 g + d2) = g^2 + c1 g + c0 ----------------------
    % the unconstrained 4-coefficient LS puts a pole at g = 4.9 (negative d1, d2): over-free. Denominator is therefore
    % written as (g + r1)(g + r2) with r_i = exp(q_i) > 0, so no pole can sit on g >= 0; numerator stays free.
    q2 = @(p) [exp(p(1)) + exp(p(2)), exp(p(1)) * exp(p(2)), p(3), p(4)];         % -> [d1 d2 c1 c0]
    f2q = @(p, gq) (gq.^2 + p(3) * gq + p(4)) ./ (gq.^2 + (exp(p(1)) + exp(p(2))) * gq + exp(p(1)) * exp(p(2)));
    p0 = [log(1.0), log(1.2), 0.5, 0.6];
    pq = fminsearch(@(p) rms(y - f2q(p, g)), p0, optimset('TolX', 1e-10, 'TolFun', 1e-12, 'MaxFunEvals', 4e4, 'MaxIter', 4e4));
    p2 = q2(pq);                                                                    % [d1; d2; c1; c0]
    f2 = @(p, gq) (gq.^2 + p(3) * gq + p(4)) ./ (gq.^2 + p(1) * gq + p(2));

    d1 = p1(1); c0 = p1(2);
    fprintf('[1/1] free contact : d1 %.4f  c0 %.4f | a(0) %.4f  slope(0) %.4f  kappa %.4f | rms %.4f  max|rel%%| band %.1f\n', ...
            d1, c0, c0/d1, (d1 - c0)/d1^2, d1 - c0, rms(y - f1(p1, g)), 100 * max(abs((f1(p1, g(band)) - y(band)) ./ y(band))));
    D1 = p2(1); D2 = p2(2); C1 = p2(3); C0 = p2(4);
    fprintf('[2/2] free contact : d1 %.4f  d2 %.4f  c1 %.4f  c0 %.4f | a(0) %.4f  slope(0) %.4f  kappa %.4f | rms %.4f  max|rel%%| band %.1f\n', ...
            D1, D2, C1, C0, C0/D2, (C1*D2 - C0*D1)/D2^2, D1 - C1, rms(y - f2(p2, g)), 100 * max(abs((f2(p2, g(band)) - y(band)) ./ y(band))));
    % pole check (denominator roots must be off g >= 0)
    fprintf('denominator roots  : [1/1] %.3f | [2/2] %s\n', -d1, mat2str(roots([1 D1 D2]).', 4));
    fprintf('in x = w - w_s (visible surface at 0): [1/1] (c0, d1) = (%.3f, %.3f), gain zero at x = %.3f, a(1) = %.3f | [2/2] (c0, c1, d1, d2) = (%.3f, %.3f, %.3f, %.3f), a(1) = %.3f\n', ...
            c0 - 1, d1 - 1, -(c0 - 1), (c0) / (d1), 1 - C1 + C0, C1 - 2, D1 - 2, 1 - D1 + D2, C0 / D2);
    % the plane at the same origin, for the figure
    gq = linspace(0.10, 4.5, 500)';  ap = zeros(size(gq));      % drawn on the band only: w_bar >= 1.1, g = 0 is never used
    for i = 1:numel(gq);  [~, cp] = calc_correction_functions(1 + gq(i));  ap(i) = 1 / cp;  end
    out = struct('g', g, 'y', y, 'p11', p1, 'p22', p2, 'f11', f1, 'f22', f2);

    % --- figure: top a_bar, bottom relative error to the data ---------------------------------
    COL_TRUE = [0.8 0 0];  COL_MEAS = [0.45 0.65 0.95];  COL_HAT = [0 0.2 0.9];  COL_ALT = [0.35 0.35 0.35];
    FS = 16;  LFS = 12;  AXLW = 1.8;  hq = 1 + gq;
    f = figure('Position', [40 40 1100 820], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile(1); hold on; box on;
    hd = plot(x, y, '.', 'Color', COL_MEAS, 'MarkerSize', 7, 'DisplayName', '$\bar{a}_{\mathrm{cell}}$, Fig. 11');
    h1 = plot(hq, f1(p1, gq), '--', 'Color', COL_ALT, 'LineWidth', 1.8, 'DisplayName', '$[1/1]$ fit');
    h2 = plot(hq, f2(p2, gq), '-', 'Color', COL_HAT, 'LineWidth', 1.8, 'DisplayName', '$[2/2]$ fit');
    xlim([1 5.5]); ylim([0.4 0.9]); xline(1.10, '-.', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off'); xline(3.32, '-.', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
    ylabel('$\bar{a}_{\mathrm{cell}}$', 'Interpreter', 'latex', 'FontSize', FS + 6);
    legend([hd h1 h2], 'Interpreter', 'latex', 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW); grid off;
    nexttile(2); hold on; box on;
    e1 = plot(x, 100 * (f1(p1, g) - y) ./ y, '.', 'Color', COL_ALT, 'MarkerSize', 6, 'DisplayName', '$[1/1]$');
    e2 = plot(x, 100 * (f2(p2, g) - y) ./ y, '.', 'Color', COL_HAT, 'MarkerSize', 6, 'DisplayName', '$[2/2]$');
    yline(0, ':', 'Color', [0.3 0.3 0.3], 'HandleVisibility', 'off');
    xlim([1 5.5]); ylim([-3 3]); xline(1.10, '-.', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off'); xline(3.32, '-.', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
    ylabel('$(\bar{a}_{\mathrm{Pad\acute{e}}} - \bar{a}_{\mathrm{data}}) / \bar{a}_{\mathrm{data}}$ [\%]', 'Interpreter', 'latex', 'FontSize', FS + 2);
    xlabel('$\bar{w} - \bar{w}_s$', 'Interpreter', 'latex', 'FontSize', FS + 4);
    legend([e1 e2], 'Interpreter', 'latex', 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW); grid off;
    fn = fullfile(od, 'cell_pade.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure: %s\n', fn);
end
