% PURPOSE (2026-09-21, user: "one constant that serves only a'"): the slope written as a function of the VALUE, a' = Lambda(a),
%   on plane / cell (cell_cperp_delta) / sphere (sphere_cperp_fig11), band h in [1.10, 22.2]. Two slope sources, no fit and one fit:
%     Lambda_plane(a)        the exact plane polynomial in autonomous form (invert a -> h on the plane, read a'_plane there); zero constants
%     s * Lambda_plane(a)    one multiplicative shape scale s per wall (LS on the band)
%     advisor, p = 2         (1 - a)^2 / (1 - c) = b (1 - a)^2, c per wall (LS on the band)   [= production formC_b's b law]
%   Console per wall: sup / at-a(1.1) slope errors for each source, the fitted s and c. Figure: a' against a (three walls, true red;
%   Lambda_plane blue; s Lambda_plane green) and the errors.  Output reference/eq17_analysis/derivation/figures/slope_vs_value_walls.png
%   EXPIRES: with the one-constant-family line
function probe_slope_vs_value_walls()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    addpath(here);
    D_CELL = 1.05;
    D   = @(u) 1 - 9/8*u + 1/2*u.^3 - 57/100*u.^4 + 1/5*u.^5 + 7/200*u.^11 - 1/25*u.^12;
    Dp  = @(u) -9/8 + 3/2*u.^2 - 228/100*u.^3 + u.^4 + 77/200*u.^10 - 12/25*u.^11;
    a_pl  = @(h) D(1 ./ h);  ap_pl = @(h) -Dp(1 ./ h) ./ h.^2;
    a_ce  = @(h) a_pl(h + D_CELL);  ap_ce = @(h) ap_pl(h + D_CELL);
    a_sp  = @(h) 1 ./ sphere_cperp_fig11(h);  ap_sp = @(h) (a_sp(max(h, 1 + 2e-3) + 1e-4) - a_sp(max(h, 1 + 2e-3) - 1e-4)) / 2e-4;   % central difference outside the plant's clamps (h >= 1, a >= 1e-3)
    % Lambda_plane(a): invert the plane polynomial on a fine table (monotone on [1, 60])
    ht = linspace(1, 60, 20000)';  at = a_pl(ht);  apt = ap_pl(ht);
    Lam_pl = @(a) interp1(at, apt, a, 'pchip');
    hU = linspace(1.10, 22.2, 4000)';
    W = {'plane', a_pl, ap_pl;  'cell', a_ce, ap_ce;  'sphere', a_sp, ap_sp};
    fprintf('\nslope as a function of the VALUE, band h in [1.10, 22.2]  (errors = a''_true - model, sup over the band and at h = 1.1)\n');
    S = zeros(1, 3);  C2 = zeros(1, 3);
    for w = 1:3
        af = W{w, 2};  apf = W{w, 3};  a = af(hU);  ap = apf(hU);
        L = Lam_pl(a);                                   % zero-constant plane autonomous form
        s = (L \ ap);  S(w) = s;                         % one scale
        b = ((1 - a).^2 \ ap);  C2(w) = 1 - 1 / b;       % advisor p = 2: a' = b (1-a)^2, b = 1/(1-c)
        e0 = ap - L;  e1 = ap - s * L;  e2 = ap - b * (1 - a).^2;
        a11 = af(1.1);
        fprintf('  %-6s a(1.1) = %.3f, a''(1.1) = %.3f | Lambda_plane: sup %.3f at1.1 %+.3f | s = %.3f: sup %.3f at1.1 %+.3f | advisor p=2 b = %.3f (c %+.3f): sup %.3f at1.1 %+.3f\n', ...
            W{w, 1}, a11, apf(1.1), max(abs(e0)), apf(1.1) - Lam_pl(a11), s, max(abs(e1)), apf(1.1) - s * Lam_pl(a11), b, C2(w), max(abs(e2)), apf(1.1) - b * (1 - a11)^2);
    end
    % the same value on three walls: where is a = 0.505 and what is the slope there
    fprintf('  at the same VALUE a = 0.505: plane h = %.2f a'' = %.3f | cell h = %.2f a'' = %.3f | sphere h = %.2f a'' = %.3f\n', ...
        interp1(at, ht, 0.505), Lam_pl(0.505), interp1(a_ce(ht(ht < 30)), ht(ht < 30), 0.505), ap_ce(interp1(a_ce(ht(ht < 30)), ht(ht < 30), 0.505)), ...
        interp1(a_sp(hU), hU, 0.505), ap_sp(interp1(a_sp(hU), hU, 0.505)));
    % --- figure: a' against a
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; CGRN = [0 0.55 0.2]; FS = 18; LFS = 12; AXLW = 2.0;
    f = figure('Position', [60 60 1800 900], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for w = 1:3
        af = W{w, 2};  apf = W{w, 3};  a = af(hU);  ap = apf(hU);
        nexttile(w); hold on; box on;
        h1 = plot(a, ap, '-', 'Color', COL_TRUE, 'LineWidth', 2.2, 'DisplayName', 'a''_{true}(a)  (plant)');
        h2 = plot(a, Lam_pl(a), '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', '\Lambda_{plane}(a)  (exact polynomial, no constant)');
        h3 = plot(a, S(w) * Lam_pl(a), '--', 'Color', CGRN, 'LineWidth', 1.6, 'DisplayName', sprintf('s \\Lambda_{plane}(a),  s = %.2f', S(w)));
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', upper(W{w, 1}));
        xlim([0 1]); ylim([0 0.9]);
        if w == 1; ylabel('a''  [1/R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'NumColumns', 2);
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        nexttile(3 + w); hold on; box on;
        h4 = plot(a, ap - Lam_pl(a), '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'a''_{true} - \Lambda_{plane}');
        h5 = plot(a, ap - S(w) * Lam_pl(a), '--', 'Color', CGRN, 'LineWidth', 1.6, 'DisplayName', 'a''_{true} - s \Lambda_{plane}');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        xlim([0 1]); ylim([-0.3 0.3]);
        if w == 1; ylabel('e_{a''}  [1/R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('a / a_{nom}  (the value)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h4 h5], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    end
    fn = fullfile(od, 'slope_vs_value_walls.png');
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
