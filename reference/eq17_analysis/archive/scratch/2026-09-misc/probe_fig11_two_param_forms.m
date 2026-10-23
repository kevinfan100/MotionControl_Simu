% PURPOSE (2026-09-16, user: "can we estimate with the [1/1] form -- are its parameters constants per wall type?"):
%   two 2-parameter forms, actual wall fixed at w_bar_s = 1 (g = w_bar - 1), least squares on each Fig. 11 curve over
%   1.1-5.5 R; the question is whether the curve is IN the family (residual at the digitisation floor, no trend), which is
%   what "the parameters are constants" means for data.
%     F1  [1/1] free contact   a = (g + c0)/(g + d1)          <=> 1/(1-a) = (g + d1)/(d1 - c0): the b law with a free origin
%     F2  plane, shift + gap scale   a = D_perp(1/(1 + s (g + delta)))   plane = (0, 1) exactly
%   Floor: the plane curve itself digitised against the exact polynomial, rms 0.0015 / max 0.0045 (09-16).
%   Output reference/eq17_analysis/derivation/figures/fig11_two_param_forms.png | EXPIRES: with the s / Pade line
function out = probe_fig11_two_param_forms()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    T  = readtable(fullfile(root, 'reference', 'eq17_analysis', 'data', 'meng_11072733_fig11_three_surfaces.csv'));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    D  = @(u) 1 - 9/8*u + 1/2*u.^3 - 57/100*u.^4 + 1/5*u.^5 + 7/200*u.^11 - 1/25*u.^12;
    F1 = @(p, g) (g + p(1)) ./ (g + p(2));                          % p = [c0 d1]
    F2 = @(p, g) D(1 ./ (1 + p(2) * max(g + p(1), 1e-6)));          % p = [delta s]
    S = {'plane', 'sphere', 'cell'};  P10 = [0 1.1; 0 0.9; 1.0 2.2];  P20 = [0 1; 0 1.3; 1.05 1];
    opt = optimset('TolX', 1e-10, 'TolFun', 1e-12, 'MaxFunEvals', 2e4, 'MaxIter', 2e4);
    out = struct();
    fprintf('%-7s | F1 [1/1] (c0, d1)   -> a(0)  kappa | rms    max|rel%%| band | F2 shift+scale (delta, s) | rms    max|rel%%| band\n', 'surface');
    for i = 1:3
        m = strcmp(T.surface, S{i});  x = T.hd_R(m);  y = T.lambda_norm(m);
        k = x >= 1.1 & x <= 5.5;  x = x(k);  y = y(k);  g = x - 1;  band = g >= 0.10 & g <= 2.32;
        p1 = fminsearch(@(p) rms(y - F1(p, g)), P10(i, :), opt);
        p2 = fminsearch(@(p) rms(y - F2(p, g)), P20(i, :), opt);
        r1 = 100 * (F1(p1, g) - y) ./ y;  r2 = 100 * (F2(p2, g) - y) ./ y;
        fprintf('%-7s | (%+.3f, %.3f) -> %.3f  %.3f | %.4f  %5.1f          | (%+.3f, %.3f)             | %.4f  %5.1f\n', S{i}, ...
                p1(1), p1(2), p1(1)/p1(2), p1(2) - p1(1), rms(y - F1(p1, g)), max(abs(r1(band))), ...
                p2(1), p2(2), rms(y - F2(p2, g)), max(abs(r2(band))));
        out.(S{i}) = struct('x', x, 'y', y, 'p1', p1, 'p2', p2, 'r1', r1, 'r2', r2);
    end
    % figure: relative error of both forms, one column per surface
    COL_HAT = [0 0.2 0.9];  COL_ALT = [0.35 0.35 0.35];  FS = 14;  LFS = 11;  AXLW = 1.8;
    f = figure('Position', [40 40 1500 420], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        o = out.(S{i});  nexttile(i); hold on; box on;
        h1 = plot(o.x, o.r1, '.', 'Color', COL_ALT, 'MarkerSize', 6, 'DisplayName', sprintf('%s: $[1/1]$', S{i}));
        h2 = plot(o.x, o.r2, '.', 'Color', COL_HAT, 'MarkerSize', 6, 'DisplayName', 'plane, $\delta$ + $s$');
        yline(0, ':', 'Color', [0.3 0.3 0.3], 'HandleVisibility', 'off');
        xline(1.10, '-.', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off'); xline(3.32, '-.', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
        xlim([1 5.5]); ylim([-8 8]);
        if i == 1; ylabel('$(\bar{a}_{\mathrm{fit}} - \bar{a}_{\mathrm{data}}) / \bar{a}_{\mathrm{data}}$ [\%]', 'Interpreter', 'latex', 'FontSize', FS + 2); end
        xlabel('$\bar{w}$', 'Interpreter', 'latex', 'FontSize', FS + 4);
        legend([h1 h2], 'Interpreter', 'latex', 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
        set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW); grid off;
    end
    fn = fullfile(od, 'fig11_two_param_forms.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure: %s\n', fn);
end
