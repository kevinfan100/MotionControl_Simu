% PURPOSE (2026-09-21, user: "two layers, separate figures"): the advisor slope form a' = (1 - c)/h^p fitted ON THE SLOPE in two layers.
%   LAYER 1 (figure advisor_layer1_plane.png): plane only, c = 0, fit p.  Fit ranges: our band [1.10, 22.2] and the "whole"
%           range of the original polynomial 1 -> inf (taken as [1.0, 1000], uniform grid 0.01 R; a' ~ 1/h^2 so nothing beyond
%           25 R weighs anything); plus the near / mid / far sub-segments of the band to show how p depends on height.
%           Panels: a' true vs law | e_a'  (p_eff(h) stays on the console).
%   LAYER 2 (figure advisor_layer2_cell_sphere.png): p fixed at the layer-1 band value, fit c on a' for cell and sphere on the band
%           and on the whole range.  Rows: a' true vs law, e_a'.  The band [1.10, 22.2] is marked on every panel.
%   Console: p per range, c per wall per range, sup / pointwise e_a' on the band.  EXPIRES: with the advisor-law test
function fit_advisor_two_layers()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    addpath(here);
    D   = @(u) 1 - 9/8*u + 1/2*u.^3 - 57/100*u.^4 + 1/5*u.^5 + 7/200*u.^11 - 1/25*u.^12;
    Dp  = @(u) -9/8 + 3/2*u.^2 - 228/100*u.^3 + u.^4 + 77/200*u.^10 - 12/25*u.^11;
    Dpp = @(u) 3*u - 684/100*u.^2 + 4*u.^3 + 770/200*u.^9 - 132/25*u.^10;
    ap_pl  = @(h) -Dp(1 ./ h) ./ h.^2;  app_pl = @(h) Dpp(1 ./ h) ./ h.^4 + 2 * Dp(1 ./ h) ./ h.^3;
    ap_ce  = @(h) ap_pl(h + 1.05);
    a_sp   = @(h) 1 ./ sphere_cperp_fig11(h);  ap_sp = @(h) (a_sp(max(h, 1 + 2e-3) + 1e-4) - a_sp(max(h, 1 + 2e-3) - 1e-4)) / 2e-4;   % central difference outside the plant's clamps (h >= 1, a >= 1e-3)
    lawp   = @(h, p, c) (1 - c) ./ h.^p;
    peff   = @(h) -h .* app_pl(h) ./ ap_pl(h);
    H_LO = 1.10;  H_HI = 22.2;  hB = linspace(H_LO, H_HI, 4000)';
    RNG = {'band [1.10, 22.2]', linspace(H_LO, H_HI, 4000)'; 'whole [1, inf)', (1.0:0.01:1000)'; ...
           'near [1.10, 2.0]', linspace(1.10, 2.0, 2000)'; 'mid [2.0, 5.5]', linspace(2.0, 5.5, 2000)'; 'far [5.5, 22.2]', linspace(5.5, 22.2, 2000)'};
    % ---- layer 1
    fprintf('\nLAYER 1: plane, c = 0, fit p on a''\n');
    P = zeros(size(RNG, 1), 1);
    for r = 1:size(RNG, 1)
        hF = RNG{r, 2};  P(r) = fminbnd(@(p) mean((lawp(hF, p, 0) - ap_pl(hF)).^2), 1.2, 4);
        e = ap_pl(hB) - lawp(hB, P(r), 0);
        fprintf('  %-20s p = %.4f | e_a'' on band: sup %.4f  at 1.1 %+.4f  2.0 %+.4f  5.5 %+.4f  22.2 %+.5f\n', RNG{r, 1}, P(r), max(abs(e)), ...
                ap_pl(1.1) - lawp(1.1, P(r), 0), ap_pl(2) - lawp(2, P(r), 0), ap_pl(5.5) - lawp(5.5, P(r), 0), ap_pl(22.2) - lawp(22.2, P(r), 0));
    end
    fprintf('  p_eff(h) = -h a''''/a'' at 1.1 / 1.5 / 2 / 5.5 / 22.2 : %.3f %.3f %.3f %.3f %.3f\n', peff(1.1), peff(1.5), peff(2), peff(5.5), peff(22.2));
    p0 = P(1);
    % ---- layer 2
    fprintf('\nLAYER 2: p = %.4f fixed (band), fit c on a''\n', p0);
    W = {'cell', ap_ce; 'sphere', ap_sp};  C = zeros(2, 2);
    for w = 1:2
        for r = 1:2
            hF = RNG{r, 2};  c = 1 - ((1 ./ hF.^p0) \ W{w, 2}(hF));  C(w, r) = c;
            e = W{w, 2}(hB) - lawp(hB, p0, c);
            fprintf('  %-6s %-20s c = %+.4f | e_a'' on band: sup %.4f  at 1.1 %+.4f  2.0 %+.4f  5.5 %+.4f\n', W{w, 1}, RNG{r, 1}, c, max(abs(e)), ...
                    W{w, 2}(1.1) - lawp(1.1, p0, c), W{w, 2}(2) - lawp(2, p0, c), W{w, 2}(5.5) - lawp(5.5, p0, c));
        end
    end
    % ---- figure 1: plane
    COL_TRUE = [0.8 0 0]; CB = [0 0.2 0.9]; CO = [0.85 0.45 0]; CS = {[0 0.55 0.2], [0.5 0 0.6], [0.4 0.4 0.4]}; FS = 18; LFS = 12; AXLW = 2; CG = [0.5 0.5 0.5];
    hx = linspace(1, 5, 2400)';                                                % zoom h = 1-5 (user 09-21); the fits themselves are unchanged
    f = figure('Position', [60 60 1300 600], 'Color', 'w', 'Visible', 'off');  tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');   % p_eff panel dropped (user 09-21)
    nexttile; hold on; box on;
    h1 = plot(hx, ap_pl(hx), '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a''_{true}  (plane polynomial)');
    h2 = plot(hx, lawp(hx, P(1), 0), '-', 'Color', CB, 'LineWidth', 1.8, 'DisplayName', sprintf('1/h^p,  p %.3f  (fit on band)', P(1)));
    h3 = plot(hx, lawp(hx, P(2), 0), '--', 'Color', CO, 'LineWidth', 1.8, 'DisplayName', sprintf('1/h^p,  p %.3f  (fit on 1 to inf)', P(2)));
    local_band(H_LO, H_HI, CG);  xlim([1 5]); ylim([0 1.05]);
    xlabel('h = w - w_s  (h / R)', 'FontSize', FS, 'FontWeight', 'bold');  ylabel('a''  [1/R]', 'FontSize', FS, 'FontWeight', 'bold');
    legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    local_ax(FS, AXLW);
    nexttile; hold on; box on;  hs = gobjects(1, 2);                          % two cases only: band vs whole (user 09-21)
    for r = 1:2
        col = {CB, CO}; col = col{r}; sty = '-'; if r == 2; sty = '--'; end
        hs(r) = plot(hx, ap_pl(hx) - lawp(hx, P(r), 0), sty, 'Color', col, 'LineWidth', 2.0, 'DisplayName', sprintf('%s  p %.3f', RNG{r, 1}, P(r)));
    end
    yline(0, '-', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');  local_band(H_LO, H_HI, CG);  xlim([1 5]); ylim([-0.12 0.12]);
    xlabel('h = w - w_s  (h / R)', 'FontSize', FS, 'FontWeight', 'bold');  ylabel('e_{a''} = a''_{true} - 1/h^p', 'FontSize', FS, 'FontWeight', 'bold');
    legend(hs, 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    local_ax(FS, AXLW);
    fn = fullfile(od, 'advisor_layer1_plane.png');  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
    % ---- figure 2: cell | sphere
    f = figure('Position', [60 60 1300 950], 'Color', 'w', 'Visible', 'off');  tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for w = 1:2
        apf = W{w, 2};
        nexttile(w); hold on; box on;
        h1 = plot(hx, apf(hx), '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'a''_{true}  (plant)');
        h2 = plot(hx, lawp(hx, p0, C(w, 1)), '-', 'Color', CB, 'LineWidth', 1.8, 'DisplayName', sprintf('(1-c)/h^p,  c %+.3f  (fit on band)', C(w, 1)));
        h3 = plot(hx, lawp(hx, p0, C(w, 2)), '--', 'Color', CO, 'LineWidth', 1.8, 'DisplayName', sprintf('(1-c)/h^p,  c %+.3f  (fit on 1 to inf)', C(w, 2)));
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s,  p = %.3f fixed', upper(W{w, 1}), p0));
        local_band(H_LO, H_HI, CG);  xlim([1 5]); ylim([0 1.05]);
        if w == 1; ylabel('a''  [1/R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend([h0 h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on', 'NumColumns', 2);
        local_ax(FS, AXLW);
        nexttile(2 + w); hold on; box on;
        h4 = plot(hx, apf(hx) - lawp(hx, p0, C(w, 1)), '-', 'Color', CB, 'LineWidth', 2.0, 'DisplayName', 'e_{a''}  (c fit on band)');
        h5 = plot(hx, apf(hx) - lawp(hx, p0, C(w, 2)), '--', 'Color', CO, 'LineWidth', 2.0, 'DisplayName', 'e_{a''}  (c fit on 1 to inf)');
        yline(0, '-', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');  local_band(H_LO, H_HI, CG);  xlim([1 5]); ylim([-0.3 0.3]);
        if w == 1; ylabel('e_{a''} = a''_{true} - (1-c)/h^p', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('h = w - w_s  (h / R)', 'FontSize', FS, 'FontWeight', 'bold');
        legend([h4 h5], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        local_ax(FS, AXLW);
    end
    fn = fullfile(od, 'advisor_layer2_cell_sphere.png');  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end

function local_band(lo, hi, c)
    xline(lo, '--', 'Color', c, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xline(hi, '--', 'Color', c, 'LineWidth', 1.0, 'HandleVisibility', 'off');
end

function local_ax(FS, AXLW)
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
end
