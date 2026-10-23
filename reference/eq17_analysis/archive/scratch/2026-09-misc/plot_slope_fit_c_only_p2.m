% PURPOSE (2026-09-24, user: "the a' fit figure for (1-c)/h^p with p fixed at 2, in the style of the earlier slope-fit figures,
%   fit + error + c only"): the c-only slope law's fit to the plant slope, plane | cell (cell_cperp_delta = plane shifted 1.05 R
%   below the visible surface), h = commanded gap to the visible surface (w_s = 0). Same layout as slope_two_param_height_F4.png:
%     row 1  a'_true (plant, red) vs (1-c)/h^2 with the band-fit c (blue), h in [1, 5] R linear
%     row 2  e_a' = a'_true - (1-c)/h^2                                          h in [1, 5] R linear
%     row 3  c fitted per segment (near [1.1, 2] / mid [2, 5.5] / far [5.5, 22.2]) at the segment centre, band fit as a line (log h)
%   Fits: least squares on a' with uniform h over the band [1.10, 22.2] (fit_two_param_height_walls.m convention).
%   House style. Output reference/eq17_analysis/derivation/figures/slope_fit_c_only_p2.png   EXPIRES: with the c-only line
function plot_slope_fit_c_only_p2()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    Dp = @(u) -9/8 + (3/2)*u.^2 - 2.28*u.^3 + u.^4 + 0.385*u.^10 - 0.48*u.^11;   % plane D'(u), u = 1/h
    ap_pl = @(h) -Dp(1 ./ h) ./ h.^2;
    ap_ce = @(h) ap_pl(h + 1.05);
    W = {'PLANE', ap_pl; 'CELL', ap_ce};
    SEG = {[1.1 2], [2 5.5], [5.5 22.2]};  H_BAND = [1.10 22.2];
    hu = linspace(H_BAND(1), H_BAND(2), 20000)';  hp = linspace(1.0, 5.0, 2000)';
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; FS = 15; LFS = 11; AXLW = 2.0; CG = [0.5 0.5 0.5];
    fitc = @(hh, yy) 1 - ((hh.^-2)' * yy) / ((hh.^-2)' * (hh.^-2));
    f = figure('Position', [60 60 1250 1050], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(3, 2);
    for w = 1:2
        lab = W{w, 1};  apf = W{w, 2};
        cfit = fitc(hu, apf(hu));
        cs = zeros(1, 3);  hc = zeros(1, 3);
        for s = 1:3; m = hu >= SEG{s}(1) & hu <= SEG{s}(2);  cs(s) = fitc(hu(m), apf(hu(m)));  hc(s) = sqrt(prod(SEG{s})); end
        tru = apf(hp);  law = (1 - cfit) ./ hp.^2;
        % ---- row 1
        ax(1, w) = nexttile(w); hold on; box on;
        h1 = plot(hp, tru, '-', 'Color', COL_TRUE, 'LineWidth', 2.2, 'DisplayName', 'a''_{true}   (plant)');
        h2 = plot(hp, law, '-', 'Color', COL_HAT, 'LineWidth', 1.8, 'DisplayName', sprintf('(1-c)/h^2,   c %+.3f   (band fit)', cfit));
        h0 = plot(NaN, NaN, 'w.', 'DisplayName', lab);
        xline(H_BAND(1), '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
        xlim([1 5]);  ylim([0 1.05]);
        legend([h0 h1 h2], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        if w == 1; ylabel('a''  [1/R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        % ---- row 2
        ax(2, w) = nexttile(2 + w); hold on; box on;
        h3 = plot(hp, tru - law, '-', 'Color', COL_HAT, 'LineWidth', 1.8, 'DisplayName', 'e_{a''} = a''_{true} - (1-c)/h^2');
        yline(0, '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8, 'HandleVisibility', 'off');
        xline(H_BAND(1), '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
        xlim([1 5]);  ylim([-0.15 0.15]);
        legend(h3, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        if w == 1; ylabel('e_{a''}  [1/R]', 'FontSize', FS, 'FontWeight', 'bold'); end
        % ---- row 3
        ax(3, w) = nexttile(4 + w); hold on; box on;
        h4 = plot(hc, cs, '-o', 'Color', COL_TRUE, 'LineWidth', 2.0, 'MarkerFaceColor', COL_TRUE, 'MarkerSize', 9, 'DisplayName', 'c per segment (near / mid / far)');
        h5 = yline(cfit, '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', sprintf('c band fit %+.3f', cfit));
        xline(H_BAND(1), '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
        set(gca, 'XScale', 'log');  xlim([1 25]);  ylim([-0.4 1.0]);
        legend([h4 h5], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        if w == 1; ylabel('c', 'FontSize', FS, 'FontWeight', 'bold'); end
        xlabel('h = w - w_s  (h / R),  segment centre (log axis)', 'FontSize', FS, 'FontWeight', 'bold');
        for q = 1:3; set(ax(q, w), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid(ax(q, w), 'off'); end
        e_band = apf(hu) - (1 - cfit) ./ hu.^2;
        fprintf('%-5s  band c = %+.4f | segments near/mid/far c = %+.3f %+.3f %+.3f (spread %.3f) | sup|e_a''| = %.4f | max|int e dh| = %.3f\n', ...
                lab, cfit, cs, max(cs) - min(cs), max(abs(e_band)), max(abs(cumtrapz(hu, e_band))));
    end
    for q = 1:3; linkaxes(ax(q, :), 'y'); end
    fn = fullfile(od, 'slope_fit_c_only_p2.png');  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
