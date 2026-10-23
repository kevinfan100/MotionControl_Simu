% PURPOSE (2026-09-18, explanation figure): what "far-field coefficient" means, walked with numbers.
%   Left : 1 - a_bar against x = w - w_s for the true plane polynomial and for the two [1/1] laws through the contact
%          (c0 = -1; d1 = 0 and d1 = 1/8), log-log so a 1/x law is a straight line of slope -1.
%   Right: x * (1 - a_bar) against x: the number it settles to at large x IS the far-field coefficient
%          (plane 9/8, [1/1] d1 - c0: 1 for d1 = 0, 9/8 for d1 = 1/8).
%   Output reference/eq17_analysis/derivation/figures/far_coefficient_walk.png; prints the table x = 5, 10, 20, 100.
%   EXPIRES: explanation aid for 0916_abar_gap_polynomial.tex; delete when that document is archived.
function plot_far_coefficient_walk()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    Dp = @(u) 1 - 9/8*u + 1/2*u.^3 - 57/100*u.^4 + 1/5*u.^5 + 7/200*u.^11 - 1/25*u.^12;
    x = logspace(log10(1.1), 2, 400);
    one_true = 1 - Dp(1 ./ x);
    one_d0   = 1 ./ x;                        % [1/1] c0 = -1, d1 = 0    : 1 - a = 1/x
    one_d18  = (1/8 + 1) ./ (x + 1/8);        % [1/1] c0 = -1, d1 = 1/8  : 1 - a = (9/8)/(x + 1/8)
    fprintf('%6s | %10s %10s %10s | %12s %12s %12s\n', 'x', '1-a true', '1-a d1=0', '1-a d1=1/8', 'x(1-a) true', 'x(1-a) d1=0', 'x(1-a) d1=1/8');
    for xx = [5 10 20 100]
        t = 1 - Dp(1/xx);  a0 = 1/xx;  a1 = (9/8)/(xx + 1/8);
        fprintf('%6.0f | %10.5f %10.5f %10.5f | %12.4f %12.4f %12.4f\n', xx, t, a0, a1, xx*t, xx*a0, xx*a1);
    end
    FS = 16; LFS = 12; CT = [0.8 0 0]; C0 = [0.35 0.35 0.35]; C1 = [0 0.2 0.9];
    f = figure('Position', [40 40 1500 560], 'Color', 'w', 'Visible', 'off');
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile; hold on; box on;
    h1 = loglog(x, one_true, '-', 'Color', CT, 'LineWidth', 2.4, 'DisplayName', '1 - a_{true}   (plane polynomial)');
    h2 = loglog(x, one_d0, '--', 'Color', C0, 'LineWidth', 1.8, 'DisplayName', '1 - a,  [1/1] c_0 = -1, d_1 = 0   ( = 1/x )');
    h3 = loglog(x, one_d18, ':', 'Color', C1, 'LineWidth', 2.2, 'DisplayName', '1 - a,  [1/1] c_0 = -1, d_1 = 1/8   ( = (9/8)/(x + 1/8) )');
    set(gca, 'XScale', 'log', 'YScale', 'log'); xlim([1.1 100]); ylim([0.008 1]);
    xlabel('x = w - w_s  (= h / R)', 'FontSize', FS, 'FontWeight', 'bold'); ylabel('1 - a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on'); grid off;
    nexttile; hold on; box on;
    g1 = semilogx(x, x .* one_true, '-', 'Color', CT, 'LineWidth', 2.4, 'DisplayName', 'x (1 - a_{true})  ->  9/8');
    g2 = semilogx(x, x .* one_d0, '--', 'Color', C0, 'LineWidth', 1.8, 'DisplayName', 'x (1 - a),  d_1 = 0  ->  1');
    g3 = semilogx(x, x .* one_d18, ':', 'Color', C1, 'LineWidth', 2.2, 'DisplayName', 'x (1 - a),  d_1 = 1/8  ->  9/8');
    yline(9/8, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    yline(1, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    text(60, 9/8 + 0.02, '9/8', 'FontSize', LFS, 'FontWeight', 'bold', 'Color', [0.4 0.4 0.4]);
    text(60, 1 + 0.02, '1', 'FontSize', LFS, 'FontWeight', 'bold', 'Color', [0.4 0.4 0.4]);
    set(gca, 'XScale', 'log'); xlim([1.1 100]); ylim([0 1.3]);
    xlabel('x = w - w_s  (= h / R)', 'FontSize', FS, 'FontWeight', 'bold'); ylabel('x (1 - a / a_{nom})', 'FontSize', FS, 'FontWeight', 'bold');
    legend([g1 g2 g3], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on'); grid off;
    fn = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures', 'far_coefficient_walk.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
