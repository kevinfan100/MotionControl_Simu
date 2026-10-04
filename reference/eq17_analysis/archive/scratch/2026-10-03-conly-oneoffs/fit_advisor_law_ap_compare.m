% PURPOSE (2026-09-21, user: "you fit a, not a' -- what changes if we fit a' over the whole range vs our band?"): the advisor law
%   a' = (1 - c)/h^p, a = 1 - (1 - c)/((p - 1) h^(p-1)), fitted three ways on plane / cell / sphere, same two-step recipe
%   (plane, c = 0 -> p; then p fixed -> c per wall), and the resulting errors in BOTH a and a':
%     F1  fit a  on the band [1.10, 22.2]   (the figure advisor_law_fit_walls.png)
%     F2  fit a' on the band [1.10, 22.2]   (least squares on the slope; the near wall dominates because a' is 0.77 at 1.1 R and 0.002 at 22 R)
%     F3  fit a' on the whole range [1.0, 25]
%   Console table per wall: p, c, sup|e_a|, e_a at 1.1 / 2.0, sup|e_a'|, e_a' at 1.1 / 2.0.  Figure: rows e_a, e_a'; columns walls;
%   the three fits overlaid.  Output reference/eq17_analysis/derivation/figures/advisor_law_fit_ap_compare.png
%   EXPIRES: with the advisor-law test
function fit_advisor_law_ap_compare()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    addpath(here);
    D_CELL = 1.05;
    D   = @(u) 1 - 9/8*u + 1/2*u.^3 - 57/100*u.^4 + 1/5*u.^5 + 7/200*u.^11 - 1/25*u.^12;
    Dp  = @(u) -9/8 + 3/2*u.^2 - 228/100*u.^3 + u.^4 + 77/200*u.^10 - 12/25*u.^11;
    a_pl  = @(h) D(1 ./ h);  ap_pl = @(h) -Dp(1 ./ h) ./ h.^2;
    a_ce  = @(h) a_pl(h + D_CELL);  ap_ce = @(h) ap_pl(h + D_CELL);
    a_sp  = @(h) 1 ./ sphere_cperp_fig11(h);  ap_sp = @(h) (a_sp(max(h, 1 + 2e-3) + 1e-4) - a_sp(max(h, 1 + 2e-3) - 1e-4)) / 2e-4;   % central difference outside the plant's clamps (h >= 1, a >= 1e-3)
    law   = @(h, p, c) 1 - (1 - c) ./ ((p - 1) * h.^(p - 1));
    lawp  = @(h, p, c) (1 - c) ./ h.^p;
    W = {'plane', a_pl, ap_pl;  'cell', a_ce, ap_ce;  'sphere', a_sp, ap_sp};
    FITS = {'F1: fit a,  [1.10, 22.2]',  'a',  1.10, 22.2; ...
            'F2: fit a'', [1.10, 22.2]', 'ap', 1.10, 22.2; ...
            'F3: fit a'', [1.0, 25]',    'ap', 1.00, 25.0};
    hE = linspace(1.10, 22.2, 4000)';                                        % errors are always reported on the band
    RES = cell(size(FITS, 1), 3);  P = zeros(size(FITS, 1), 1);
    for f = 1:size(FITS, 1)
        kind = FITS{f, 2};  hF = linspace(FITS{f, 3}, FITS{f, 4}, 4000)';
        if strcmp(kind, 'a'); cost = @(p) mean((law(hF, p, 0) - a_pl(hF)).^2); else; cost = @(p) mean((lawp(hF, p, 0) - ap_pl(hF)).^2); end
        p = fminbnd(cost, 1.2, 4);  P(f) = p;
        fprintf('\n%s :  plane, c = 0  ->  p = %.4f\n', FITS{f, 1}, p);
        for w = 1:3
            af = W{w, 2};  apf = W{w, 3};
            if strcmp(kind, 'a'); c = 1 - ((1 ./ ((p - 1) * hF.^(p - 1))) \ (1 - af(hF)));    % a = 1 - (1-c) phi
            else;                 c = 1 - ((1 ./ hF.^p) \ apf(hF)); end                       % a' = (1-c)/h^p
            e = af(hE) - law(hE, p, c);  ep = apf(hE) - lawp(hE, p, c);
            fprintf('  %-6s c %+.4f | e_a : sup %.4f  at 1.1 %+.4f  2.0 %+.4f  5.5 %+.4f | e_a'': sup %.3f  at 1.1 %+.3f  2.0 %+.3f\n', ...
                W{w, 1}, c, max(abs(e)), af(1.1) - law(1.1, p, c), af(2) - law(2, p, c), af(5.5) - law(5.5, p, c), ...
                max(abs(ep)), apf(1.1) - lawp(1.1, p, c), apf(2) - lawp(2, p, c));
            RES{f, w} = struct('p', p, 'c', c);
        end
    end
    % --- figure: e_a and e_a' on [1, 25] for the three fits
    FS = 18; LFS = 12; AXLW = 2.0; CG = [0.5 0.5 0.5];
    COLS = {[0 0.2 0.9], [0.85 0.45 0], [0 0.55 0.2]};  STY = {'-', '--', ':'};
    hx = linspace(1, 25, 2400)';
    fig = figure('Position', [60 60 1800 900], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for w = 1:3
        af = W{w, 2};  apf = W{w, 3};
        nexttile(w); hold on; box on;  hs = gobjects(1, 3);
        for f = 1:3
            r = RES{f, w};
            hs(f) = plot(hx, af(hx) - law(hx, r.p, r.c), STY{f}, 'Color', COLS{f}, 'LineWidth', 2.0, 'DisplayName', sprintf('%s  p %.2f c %+.2f', FITS{f, 1}, r.p, r.c));
        end
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        xline(1.10, '--', 'Color', CG, 'HandleVisibility', 'off');  xline(22.2, '--', 'Color', CG, 'HandleVisibility', 'off');
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', upper(W{w, 1}));
        xlim([1 25]); ylim([-0.2 0.2]);
        if w == 1; ylabel('e_a = a_{true} - a_{law}', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 hs], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'NumColumns', 2);
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        nexttile(3 + w); hold on; box on;  hs = gobjects(1, 3);
        for f = 1:3
            r = RES{f, w};
            hs(f) = plot(hx, apf(hx) - lawp(hx, r.p, r.c), STY{f}, 'Color', COLS{f}, 'LineWidth', 2.0, 'DisplayName', FITS{f, 1});
        end
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
        xline(1.10, '--', 'Color', CG, 'HandleVisibility', 'off');  xline(22.2, '--', 'Color', CG, 'HandleVisibility', 'off');
        xlim([1 25]); ylim([-0.5 0.5]);
        if w == 1; ylabel('e_{a''} = a''_{true} - a''_{law}', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('h = w - w_s  (h / R)', 'FontSize', FS, 'FontWeight', 'bold');
        legend(hs, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    end
    fn = fullfile(od, 'advisor_law_fit_ap_compare.png');
    exportgraphics(fig, fn, 'Resolution', 150); close(fig); fprintf('figure: %s\n', fn);
end
