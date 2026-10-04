% PURPOSE (2026-09-21, step 0 of the advisor-law test, wall-test protocol): the advisor's slope form a' = (1 - c)/h^p, integrated
%   from the far field (a(inf) = 1 is the definition of a_nom):  a(h; p, c) = 1 - (1 - c) / ((p - 1) h^(p-1)),  h = w - w_s.
%   (1) PLANE, c = 0: fit p in a (not a') over the band h in [1.10, 22.2] -- uniform in h (primary) and weighted by the canon
%       dwell time (what the estimator actually sees); p_eff(h) = -h a''/a' walk and sup|p_eff - p_fit| (the sqrt(P[0]) a
%       Q_p = 0 estimator would need, CLAUDE.md constraint 4).
%   (2) p := p_plane fixed, fit c on plane / cell (cell_cperp_delta) / sphere (sphere_cperp_fig11): c is linear, closed form;
%       c_eff(h) = 1 - h^p a'_plant(h) walk, sup|c_eff - c_fit|, and a(1) = 1 - (1 - c)/(p - 1) against the plant's visible-contact gain.
%   (3) BOTH free, (p, c) fitted per wall (user 09-21): what the family can do at best on each wall (two constants; online only
%       one is identifiable, so this is an upper bound, not an estimator arm).
%   Figures (house style, linear h): columns plane | cell | sphere; rows a_true (red) vs law (blue), e_a = a_true - a_law,
%   a'_true vs a'_law = (1 - c)/h^p, e_a' = a'_true - a'_law.  advisor_law_fit_walls.png (p fixed) and _pc.png (both free).
%   Output reference/eq17_analysis/derivation/figures/advisor_law_fit_walls{,_pc}.png | EXPIRES: with the advisor-law test
function fit_advisor_law_walls()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    addpath(here);
    H_LO = 1.10;  H_HI = 22.2;  D_CELL = 1.05;
    D   = @(u) 1 - 9/8*u + 1/2*u.^3 - 57/100*u.^4 + 1/5*u.^5 + 7/200*u.^11 - 1/25*u.^12;      % plane polynomial (0916_abar_gap_polynomial.tex)
    Dp  = @(u) -9/8 + 3/2*u.^2 - 228/100*u.^3 + u.^4 + 77/200*u.^10 - 12/25*u.^11;
    Dpp = @(u) 3*u - 684/100*u.^2 + 4*u.^3 + 770/200*u.^9 - 132/25*u.^10;
    a_pl  = @(h) D(1 ./ h);  ap_pl = @(h) -Dp(1 ./ h) ./ h.^2;  app_pl = @(h) Dpp(1 ./ h) ./ h.^4 + 2 * Dp(1 ./ h) ./ h.^3;
    a_ce  = @(h) a_pl(h + D_CELL);  ap_ce = @(h) ap_pl(h + D_CELL);
    a_sp  = @(h) 1 ./ sphere_cperp_fig11(h);  ap_sp = @(h) (a_sp(max(h, 1 + 2e-3) + 1e-4) - a_sp(max(h, 1 + 2e-3) - 1e-4)) / 2e-4;   % central difference outside the plant's clamps (h >= 1, a >= 1e-3)
    law   = @(h, p, c) 1 - (1 - c) ./ ((p - 1) * h.^(p - 1));
    lawp  = @(h, p, c) (1 - c) ./ h.^p;
    % --- canon dwell-time weights (plane run, w_s = 0): h = commanded w_d
    S = load(fullfile(root, 'test_results', 'temp_delta_two_walls', 'plane_MID.mat'), 'wd');
    hT = S.wd(S.wd >= H_LO - 1e-6);  hT = min(hT, H_HI);
    ed = [1.1 1.5 2 3.32 5.5 10 22.2];  cnt = histcounts(hT, ed) / numel(hT);
    fprintf('canon dwell fraction by height bin: ');  for k = 1:numel(cnt); fprintf('[%.2f,%.2f] %.2f  ', ed(k), ed(k+1), cnt(k)); end;  fprintf('\n');
    hU = linspace(H_LO, H_HI, 4000)';                                      % uniform-in-h grid
    % --- (1) plane, c = 0, fit p
    costU = @(p) mean((law(hU, p, 0) - a_pl(hU)).^2);
    costT = @(p) mean((law(hT, p, 0) - a_pl(hT)).^2);
    pU = fminbnd(costU, 1.2, 4);  pT = fminbnd(costT, 1.2, 4);
    peff = @(h) -h .* app_pl(h) ./ ap_pl(h);
    fprintf('\n(1) PLANE, c = 0, band [%.2f, %.1f]:  p_fit uniform-h = %.4f | canon-dwell = %.4f\n', H_LO, H_HI, pU, pT);
    for pp = [pU pT 2]
        e = a_pl(hU) - law(hU, pp, 0);
        fprintf('    p = %.4f: sup|e_a| %.4f  rms %.4f  | e_a at 1.1 %+.4f  2.0 %+.4f  5.5 %+.4f  22.2 %+.4f | a_law(1) = %.4f\n', ...
            pp, max(abs(e)), rms(e), a_pl(1.1) - law(1.1, pp, 0), a_pl(2) - law(2, pp, 0), a_pl(5.5) - law(5.5, pp, 0), a_pl(22.2) - law(22.2, pp, 0), law(1, pp, 0));
    end
    pe = peff(hU);
    fprintf('    p_eff walk on band: min %.3f  max %.3f  | sup|p_eff - p_fit| = %.3f (needed sqrt(P[0]) for Q_p = 0) | p_eff at 1.1 %.2f  2 %.2f  5.5 %.2f  22.2 %.2f\n', ...
        min(pe), max(pe), max(abs(pe - pU)), peff(1.1), peff(2), peff(5.5), peff(22.2));
    p0 = pU;                                                                  % the base: plane, uniform-h
    % --- (2) p fixed, fit c per wall (linear LS: a = 1 - (1 - c) phi  =>  (1 - a) = (1 - c) phi)
    cfit_of_p = @(p, af, hh) 1 - ((1 ./ ((p - 1) * hh.^(p - 1))) \ (1 - af(hh)));
    W = {'plane', a_pl, ap_pl, a_pl(1.0);  'cell', a_ce, ap_ce, a_ce(1.0);  'sphere', a_sp, ap_sp, a_sp(1.0)};
    fprintf('\n(2) p fixed at plane p = %.4f, fit c per wall (uniform-h | canon-dwell):\n', p0);
    RES = struct('name', {}, 'p', {}, 'c', {}, 'cT', {}, 'ceff', {}, 'a', {}, 'ap', {});
    for w = 1:size(W, 1)
        af = W{w, 2};  apf = W{w, 3};
        cU = cfit_of_p(p0, af, hU);  cT = cfit_of_p(p0, af, hT);
        e = af(hU) - law(hU, p0, cU);  ep = apf(hU) - lawp(hU, p0, cU);
        ceff = 1 - hU.^p0 .* apf(hU);
        fprintf('  %-6s c_fit %+.4f | %+.4f  sup|e_a| %.4f rms %.4f | e_a at 1.1 %+.4f 2.0 %+.4f 5.5 %+.4f 22.2 %+.4f | sup|e_a''| %.3f (at 1.1 %+.3f) | a_law(1) = %.3f vs plant a(1) = %.3f | c_eff min %+.3f max %+.3f sup|c_eff - c_fit| %.3f\n', ...
            W{w, 1}, cU, cT, max(abs(e)), rms(e), af(1.1) - law(1.1, p0, cU), af(2) - law(2, p0, cU), af(5.5) - law(5.5, p0, cU), af(22.2) - law(22.2, p0, cU), ...
            max(abs(ep)), apf(1.1) - lawp(1.1, p0, cU), law(1, p0, cU), W{w, 4}, min(ceff), max(ceff), max(abs(ceff - cU)));
        RES(w) = struct('name', W{w, 1}, 'p', p0, 'c', cU, 'cT', cT, 'ceff', ceff, 'a', af, 'ap', apf);
    end
    % --- (3) both free per wall: for each p the best c is linear, so minimise over p only (nested)
    fprintf('\n(3) BOTH free, (p, c) per wall (uniform-h):\n');
    RES2 = RES;
    for w = 1:size(W, 1)
        af = W{w, 2};  apf = W{w, 3};
        cost2 = @(p) mean((law(hU, p, cfit_of_p(p, af, hU)) - af(hU)).^2);
        p2 = fminbnd(cost2, 1.2, 4);  c2 = cfit_of_p(p2, af, hU);
        e = af(hU) - law(hU, p2, c2);  ep = apf(hU) - lawp(hU, p2, c2);
        fprintf('  %-6s p %.4f c %+.4f  K = (1-c)/(p-1) = %.3f | sup|e_a| %.4f rms %.4f | e_a at 1.1 %+.4f 2.0 %+.4f 5.5 %+.4f 22.2 %+.4f | sup|e_a''| %.3f (at 1.1 %+.3f) | a_law(1) = %.3f vs plant a(1) = %.3f\n', ...
            W{w, 1}, p2, c2, (1 - c2) / (p2 - 1), max(abs(e)), rms(e), af(1.1) - law(1.1, p2, c2), af(2) - law(2, p2, c2), af(5.5) - law(5.5, p2, c2), af(22.2) - law(22.2, p2, c2), ...
            max(abs(ep)), apf(1.1) - lawp(1.1, p2, c2), law(1, p2, c2), W{w, 4});
        RES2(w).p = p2;  RES2(w).c = c2;  RES2(w).cT = NaN;  RES2(w).ceff = 1 - hU.^p2 .* apf(hU);
    end
    save(fullfile(root, 'test_results', 'temp_delta_two_walls', 'advisor_law_fit.mat'), 'p0', 'pU', 'pT', 'RES', 'RES2', 'H_LO', 'H_HI');
    local_fig(RES,  law, lawp, H_LO, H_HI, fullfile(od, 'advisor_law_fit_walls.png'),    'p fixed');
    local_fig(RES2, law, lawp, H_LO, H_HI, fullfile(od, 'advisor_law_fit_walls_pc.png'), 'p, c fit');
end

function local_fig(RES, law, lawp, H_LO, H_HI, fn, lab)
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; FS = 18; LFS = 13; AXLW = 2.0; CG = [0.5 0.5 0.5];
    hx = linspace(1, 25, 2400)';
    f = figure('Position', [60 60 1800 1500], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for w = 1:3
        r = RES(w);  al = law(hx, r.p, r.c);  at = r.a(hx);  apl = lawp(hx, r.p, r.c);  apt = r.ap(hx);
        nexttile(w); hold on; box on;
        h1 = plot(hx, at, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a_{true} / a_{nom}  (plant)');
        h2 = plot(hx, al, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', sprintf('law  p %.2f  c %+.3f  (%s)', r.p, r.c, lab));
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', upper(r.name));
        local_band(H_LO, H_HI, CG);  xlim([1 25]); ylim([0 1]);
        if w == 1; ylabel('a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        nexttile(3 + w); hold on; box on;
        h3 = plot(hx, at - al, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'e_a = a_{true} - a_{law}');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        local_band(H_LO, H_HI, CG);  xlim([1 25]); ylim([-0.15 0.15]);
        if w == 1; ylabel('e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(h3, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        nexttile(6 + w); hold on; box on;
        h4 = plot(hx, apt, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'a''_{true} = d(a/a_{nom})/dh  (plant)');
        h5 = plot(hx, apl, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'a''_{law} = (1 - c) / h^p');
        local_band(H_LO, H_HI, CG);  xlim([1 25]); ylim([0 1.1]);
        if w == 1; ylabel('a''  [1/R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h4 h5], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
        nexttile(9 + w); hold on; box on;
        h6 = plot(hx, apt - apl, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', 'e_{a''} = a''_{true} - a''_{law}');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        local_band(H_LO, H_HI, CG);  xlim([1 25]); ylim([-0.5 0.5]);
        if w == 1; ylabel('e_{a''}  [1/R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('h = w - w_s  (h / R)', 'FontSize', FS, 'FontWeight', 'bold');
        legend(h6, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
    end
    exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end

function local_band(lo, hi, c)
    xline(lo, '--', 'Color', c, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xline(hi, '--', 'Color', c, 'LineWidth', 1.0, 'HandleVisibility', 'off');
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
