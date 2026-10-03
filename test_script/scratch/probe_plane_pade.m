% PURPOSE (2026-09-16, user: "test the current a(w) with Pade approximation, m = n = 1 and 2, last page of
%   0916_abar_gap_polynomial"): how many rational coefficients does the plane gain law need, and which ends does each
%   low-order form pin? The algebra is done in the surface gap g = h_bar - 1 (contact at g = 0); the RESULTS are reported
%   in the document's coordinate x = w_bar - w_bar_s = h_bar (w_bar_s = wall surface, 2026-09-18): every constant of a
%   form in g maps to the form in x by the substitution g = x - 1 (printed below as "in x").
%     exact      a(g) = D_perp(1/(1+g)),  D_perp(u) = 1 - 9/8 u + 1/2 u^3 - 57/100 u^4 + 1/5 u^5 + 7/200 u^11 - 1/25 u^12
%   (i)  one-point Pade about the contact g = 0 (textbook: match the Taylor series a = g - 327/200 g^2 + 203/40 g^3 - ...)
%          [1/1], [2/2]  -- symbolic pade(); the far-field limit is whatever falls out, it is NOT pinned to 1
%   (ii) two-point Pade: a(0) = 0 and a(inf) = 1 pinned structurally, remaining coefficients from the two ends
%          [1/1]  a = g/(g + b)                      one coefficient: b = 1 (wall slope) OR b = 9/8 (far coefficient), not both
%                 in x: (x - 1)/(x + d1), d1 = b - 1 = 0 or 1/8
%          [2/2]  a = (g^2 + c1 g)/(g^2 + d1 g + d2)  three: wall slope 1, wall curvature -327/200, far coefficient 9/8
%                 in x: (c0, c1, d1, d2) = (1 - c1, c1 - 2, d1 - 2, 1 - d1 + d2)
%   Errors are reported on the canonical deep band g in [0.10, 2.32] and on g in [0, 5].
%   Output reference/eq17_analysis/derivation/figures/plane_pade.png | EXPIRES: with the s / Pade line | 產線改動不會自動跟上
function out = probe_plane_pade()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    syms g u v real
    D  = 1 - sym(9)/8*u + sym(1)/2*u^3 - sym(57)/100*u^4 + sym(1)/5*u^5 + sym(7)/200*u^11 - sym(1)/25*u^12;
    ag = subs(D, u, 1/(1+g));                              % exact a(g)

    % --- expansions at the two ends ------------------------------------------------
    Tw = taylor(ag, g, 'Order', 6);                        % wall:  g - 327/200 g^2 + ...
    Tf = taylor(subs(ag, g, 1/v), v, 'Order', 4);          % far:   1 - 9/8 v + 9/8 v^2 - ... , v = 1/g
    fprintf('wall series : %s\n', char(Tw));
    fprintf('far  series : %s   (v = 1/g)\n', char(Tf));

    % --- (i) one-point Pade about g = 0 -------------------------------------------
    P11 = pade(ag, g, 'Order', [1 1]);
    P22 = pade(ag, g, 'Order', [2 2]);
    fprintf('\none-point [1/1] about g=0 : %s    limit g->inf = %s\n', char(P11), char(limit(P11, g, inf)));
    fprintf('one-point [2/2] about g=0 : %s    limit g->inf = %s\n', char(P22), char(limit(P22, g, inf)));

    % --- (ii) two-point Pade ------------------------------------------------------
    Q11n = g / (g + 1);                                    % wall slope 1
    Q11f = g / (g + sym(9)/8);                             % far coefficient 9/8
    syms c1 d1 d2 real
    Q22 = (g^2 + c1*g) / (g^2 + d1*g + d2);
    wall = taylor(Q22, g, 'Order', 3);                     % c1/d2 g + (1/d2 - c1 d1/d2^2) g^2
    far  = taylor(subs(1 - Q22, g, 1/v), v, 'Order', 2);   % (d1 - c1) v
    cw = coeffs(wall, g, 'All');  tw = coeffs(Tw, g, 'All');        % descending powers
    cf = coeffs(far, v, 'All');   tf = coeffs(1 - Tf, v, 'All');
    eqs = [cw(end-1) == tw(end-1), cw(end-2) == tw(end-2), cf(end-1) == tf(end-1)];   % g^1, g^2, v^1
    sol = solve(eqs, [c1 d1 d2]);
    Q22 = subs(Q22, [c1 d1 d2], [sol.c1 sol.d1 sol.d2]);
    fprintf('\ntwo-point [1/1] wall : %s\n', char(Q11n));
    fprintf('two-point [1/1] far  : %s\n', char(Q11f));
    fprintf('two-point [2/2]      : %s   (c1 %s, d1 %s, d2 %s)\n', char(Q22), char(sol.c1), char(sol.d1), char(sol.d2));
    fprintf('                       decimals c1 %.5f d1 %.5f d2 %.5f\n', double(sol.c1), double(sol.d1), double(sol.d2));
    fprintf('in x = w - w_s = h_bar (surface convention): [1/1] (c0, d1) = (-1, 0) or (-1, 1/8); [2/2] (c0, c1, d1, d2) = (%s, %s, %s, %s) = (%.4f, %.4f, %.4f, %.4f)\n', ...
            char(1 - sol.c1), char(sol.c1 - 2), char(sol.d1 - 2), char(1 - sol.d1 + sol.d2), ...
            double(1 - sol.c1), double(sol.c1 - 2), double(sol.d1 - 2), double(1 - sol.d1 + sol.d2));

    % --- errors ---------------------------------------------------------------------
    F = struct('exact', ag, 'one_11', P11, 'one_22', P22, 'two_11_wall', Q11n, 'two_11_far', Q11f, 'two_22', Q22);
    names = fieldnames(F);  gg = linspace(0, 5, 1001)';  band = gg >= 0.10 & gg <= 2.32;
    vals = struct();
    for k = 1:numel(names);  vals.(names{k}) = double(subs(F.(names{k}), g, gg));  end
    rel = gg > 0;   % relative error (a_Pade - a)/a is undefined at contact
    fprintf('\n%-14s | band [0.10,2.32]: max|err|  rms  max|rel%%|   | [0,5]: max|err|  rms   | rel%% at g = 0.10 / 1.0 / 2.32\n', 'form');
    for k = 2:numel(names)
        e = vals.(names{k}) - vals.exact;  r = 100 * e(rel) ./ vals.exact(rel);  gr = gg(rel);
        q = @(g0) interp1(gr, r, g0);
        fprintf('%-14s | %.4f  %.4f  %6.1f            | %.4f  %.4f   | %+.1f / %+.1f / %+.1f\n', names{k}, ...
                max(abs(e(band))), rms(e(band)), max(abs(r(band(rel)))), max(abs(e)), rms(e), q(0.10), q(1.0), q(2.32));
    end
    out = struct('F', F, 'g', gg, 'vals', vals, 'sol', sol);

    % --- figure: two-point forms only; top a_bar, bottom relative error (what f = dw/a sees) ----------
    COL_TRUE = [0.8 0 0];  COL_HAT = [0 0.2 0.9];  COL_ALT = [0.35 0.35 0.35];  FS = 16;  LFS = 12;  AXLW = 1.8;
    hb = 1 + gg;  hr = hb(rel);  pr = @(x) 100 * (x(rel) - vals.exact(rel)) ./ vals.exact(rel);
    f = figure('Position', [40 40 1100 820], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile(1); hold on; box on;
    h0 = plot(hb, vals.exact, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', '$\bar{a}$ exact');
    h1 = plot(hb, vals.two_11_wall, '--', 'Color', COL_ALT, 'LineWidth', 1.6, 'DisplayName', '$[1/1]$, $c_0 = -1$, $d_1 = 0$');
    h2 = plot(hb, vals.two_11_far, ':', 'Color', COL_ALT, 'LineWidth', 2.0, 'DisplayName', '$[1/1]$, $c_0 = -1$, $d_1 = 1/8$');
    h3 = plot(hb, vals.two_22, '-', 'Color', COL_HAT, 'LineWidth', 1.8, 'DisplayName', '$[2/2]$');
    xlim([1 6]); ylim([0 1]); xline(1.10, '-.', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off'); xline(3.32, '-.', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
    ylabel('$\bar{a}$', 'Interpreter', 'latex', 'FontSize', FS + 6);
    legend([h0 h1 h2 h3], 'Interpreter', 'latex', 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW); grid off;
    nexttile(2); hold on; box on;
    e1 = plot(hr, pr(vals.two_11_wall), '--', 'Color', COL_ALT, 'LineWidth', 1.6, 'DisplayName', '$[1/1]$, $c_0 = -1$, $d_1 = 0$');
    e2 = plot(hr, pr(vals.two_11_far), ':', 'Color', COL_ALT, 'LineWidth', 2.0, 'DisplayName', '$[1/1]$, $c_0 = -1$, $d_1 = 1/8$');
    e3 = plot(hr, pr(vals.two_22), '-', 'Color', COL_HAT, 'LineWidth', 1.8, 'DisplayName', '$[2/2]$');
    yline(0, ':', 'Color', [0.3 0.3 0.3], 'HandleVisibility', 'off');
    xlim([1 6]); ylim([-12 12]); xline(1.10, '-.', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off'); xline(3.32, '-.', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
    ylabel('$(\bar{a}_{\mathrm{Pad\acute{e}}} - \bar{a}) / \bar{a}$ [\%]', 'Interpreter', 'latex', 'FontSize', FS + 4);
    xlabel('$\bar{w} - \bar{w}_s = \bar{h}$', 'Interpreter', 'latex', 'FontSize', FS + 4);
    legend([e1 e2 e3], 'Interpreter', 'latex', 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW); grid off;
    fn = fullfile(od, 'plane_pade.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure: %s\n', fn);
end
