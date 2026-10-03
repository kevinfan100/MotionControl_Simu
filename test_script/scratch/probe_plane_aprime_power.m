% PURPOSE (2026-09-19, advisor's slope form a' = (1 - c)/h^p, plane only, c = 0): which plant boundary conditions pin p.
%   Left  : a' of the plane polynomial against h = w - w_s (log-log) with 1/h^2 and 1/h^p_fit (band fit [1.1, 25]).
%   Middle: local exponent p_eff(h) = -h a''/a' of the plane -- what "p" the plant actually has at each height.
%   Right : a_law - a_plant for p = 2 (both levels exact), p_fit anchored at contact, p_fit anchored far, [1/1] band fit.
%   Console: p_fit, p_eff at 1 / 1.1 / 2 / 5.5 / 22.2, level integral (1 - c)/(p - 1), errors at the four heights.
%   Facts: a'(1) = 1 pins c = 0 (not p); a(1) = 0 and a(inf) = 1 give int a' = 1/(p - 1) = 1 -> p = 2 exactly;
%          far coefficient 9/8 cannot be met (p = 2 gives 1); a' = 1/h^2 IS the [1/1] law c0 = -1, d1 = 0.
%   Output reference/eq17_analysis/derivation/figures/plane_aprime_power.png.  EXPIRES: with the plant-first law decision.
function probe_plane_aprime_power()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    D   = @(u) 1 - 9/8*u + 1/2*u.^3 - 57/100*u.^4 + 1/5*u.^5 + 7/200*u.^11 - 1/25*u.^12;
    Dp  = @(u) -9/8 + 3/2*u.^2 - 228/100*u.^3 + u.^4 + 77/200*u.^10 - 12/25*u.^11;
    Dpp = @(u) 3*u - 684/100*u.^2 + 4*u.^3 + 770/200*u.^9 - 132/25*u.^10;
    a   = @(x) D(1 ./ x);
    ap  = @(x) -Dp(1 ./ x) ./ x.^2;
    app = @(x) Dpp(1 ./ x) ./ x.^4 + 2 * Dp(1 ./ x) ./ x.^3;
    peff = @(x) -x .* app(x) ./ ap(x);
    xb = logspace(log10(1.1), log10(25), 2000);                     % working band
    L = log(xb); Y = log(ap(xb));  p_fit = -(L * Y') / (L * L');     % c = 0: ln a' = -p ln x
    fprintf('band fit [1.1, 25], c = 0: p_fit = %.3f;  level integral (1-c)/(p-1) = %.3f (must be 1)\n', p_fit, 1/(p_fit - 1));
    for xx = [1 1.1 2 5.5 22.2]
        fprintf('  x = %5.1f : a = %.4f  a'' = %.4f  p_eff = %.2f\n', xx, a(xx), ap(xx), peff(xx));
    end
    laws = { ...
        'p = 2, a(1) = 0 and a(\infty) = 1 both exact:  a = 1 - 1/h',          @(x) 1 - 1 ./ x; ...
        sprintf('p = %.2f anchored at contact a(1) = 0', p_fit),                @(x) (1 - x.^(1 - p_fit)) / (p_fit - 1); ...
        sprintf('p = %.2f anchored far a(\\infty) = 1', p_fit),                 @(x) 1 - x.^(1 - p_fit) / (p_fit - 1); ...
        '[1/1] band fit (c_0, d_1) = (-0.984, 0.161)',                          @(x) (x - 0.984) ./ (x + 0.161)};
    fprintf('%-60s %8s %8s %8s %8s %8s\n', 'law', 'e@1.1', 'e@2', 'e@5.5', 'e@22.2', 'max');
    for k = 1:size(laws, 1)
        f = laws{k, 2}; hh = [1.1 2 5.5 22.2]; e = f(hh) - a(hh);
        fprintf('%-60s %+8.4f %+8.4f %+8.4f %+8.4f %8.4f\n', laws{k, 1}, e, max(abs(f(xb) - a(xb))));
    end
    FS = 15; LFS = 11; CT = [0.8 0 0]; C2 = [0 0.2 0.9]; CF = [0 0.55 0.2]; CG = [0.4 0.4 0.4]; CO = [0.85 0.45 0];
    x = logspace(0, log10(30), 600);
    fig = figure('Position', [40 40 1800 600], 'Color', 'w', 'Visible', 'off');
    tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile; hold on; box on;
    h1 = plot(x, ap(x), '-', 'Color', CT, 'LineWidth', 2.4, 'DisplayName', 'a''_{true}  (plane polynomial)');
    h2 = plot(x, 1 ./ x.^2, '--', 'Color', C2, 'LineWidth', 1.8, 'DisplayName', 'a'' = 1/h^2   (c = 0, p = 2)');
    h3 = plot(x, 1 ./ x.^p_fit, ':', 'Color', CF, 'LineWidth', 2.2, 'DisplayName', sprintf('a'' = 1/h^{%.2f}   (c = 0, p fitted on [1.1, 25])', p_fit));
    xline(1.1, '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    set(gca, 'XScale', 'log', 'YScale', 'log'); xlim([1 30]); ylim([1e-3 1.2]);
    xlabel('h = w - w_s  (h / R)', 'FontSize', FS, 'FontWeight', 'bold'); ylabel('a'' = d(a / a_{nom}) / dh', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on'); grid off;
    nexttile; hold on; box on;
    g1 = plot(x, peff(x), '-', 'Color', CT, 'LineWidth', 2.4, 'DisplayName', 'p_{eff}(h) = -h a''''_{true} / a''_{true}   (local exponent of the plane)');
    g2 = yline(2, '--', 'Color', C2, 'LineWidth', 1.8, 'DisplayName', 'p = 2   (from a(1) = 0 and a(\infty) = 1)');
    g3 = yline(p_fit, ':', 'Color', CF, 'LineWidth', 2.2, 'DisplayName', sprintf('p = %.2f   (band fit)', p_fit));
    xline(1.1, '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    set(gca, 'XScale', 'log'); xlim([1 30]); ylim([1.5 3]);
    xlabel('h = w - w_s  (h / R)', 'FontSize', FS, 'FontWeight', 'bold'); ylabel('p_{eff}', 'FontSize', FS, 'FontWeight', 'bold');
    legend([g1 g2 g3], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on'); grid off;
    nexttile; hold on; box on;
    cols = {C2, CF, CO, CG}; sty = {'--', ':', '-.', '-'}; hs = gobjects(1, 4);
    for k = 1:4
        f = laws{k, 2};
        hs(k) = plot(x, f(x) - a(x), sty{k}, 'Color', cols{k}, 'LineWidth', 2.0, 'DisplayName', laws{k, 1});
    end
    yline(0, '-', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xline(1.1, '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    set(gca, 'XScale', 'log'); xlim([1 30]); ylim([-0.06 0.06]);
    xlabel('h = w - w_s  (h / R)', 'FontSize', FS, 'FontWeight', 'bold'); ylabel('a_{law} - a_{true}   [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold');
    legend(hs, 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on'); grid off;
    fn = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures', 'plane_aprime_power.png');
    exportgraphics(fig, fn, 'Resolution', 150); close(fig); fprintf('figure: %s\n', fn);
end
