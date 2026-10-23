function plot_abar_vs_gap()
%PLOT_ABAR_VS_GAP  a_z/a_nom of the plane c_perp polynomial against the gap (w - w_s)/R.
%
%   Figure page of reference/eq17_analysis/derivation/0916_abar_gap_polynomial.tex.
%   The curve is 1/c_perp from model/wall_effect/calc_correction_functions.m evaluated
%   at h_bar = w_bar - w_bar_s, w_bar_s being the WALL SURFACE in the centre coordinate
%   (plane: 0; coordinate revised 2026-09-18, was the contact height 1 with the gap
%   h_bar - 1 as the variable). No fit, no asymptote: the polynomial itself, gain
%   normalised by the far-field a_nom = Ts/gamma_N.
%
%   Also prints (Symbolic Math Toolbox) the exact coefficients of
%       (w-w_s)^12 * a_bar   as ascending powers of x = w - w_s
%   and checks the closed form against calc_correction_functions pointwise.
%
%   Style: figure-style.md (no grid, no title, box on, legend northoutside
%   horizontal, True = red, FontSize 18 bold), copied from plot_var_ahat_6state.m.

    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(fileparts(script_dir));
    addpath(fullfile(project_root, 'model', 'wall_effect'));
    out_png = fullfile(project_root, 'reference', 'eq17_analysis', 'derivation', ...
                       'figures', 'abar_vs_gap.png');

    % --- the curve: 1/c_perp against h_bar = w_bar - w_bar_s, contact at 1 ---
    h_bar = linspace(1, 10, 901);                % h/R = (w - w_s)/R
    abar  = zeros(size(h_bar));
    for k = 1:numel(h_bar)
        [~, c_perp] = calc_correction_functions(h_bar(k));
        abar(k) = 1 / c_perp;
    end

    % --- exact coefficients of the single-fraction form (printed, not plotted) ---
    syms x real                                  % x = w_bar - w_bar_s = h_bar
    D = 1 - sym(9)/8/x + sym(1)/2/x^3 - sym(57)/100/x^4 ...
          + sym(1)/5/x^5 + sym(7)/200/x^11 - sym(1)/25/x^12;
    N = expand(simplifyFraction(D * x^12));
    cN = fliplr(coeffs(N, x, 'All'));            % ascending powers
    fprintf('[plot_abar_vs_gap] x^12 * a_bar, ascending powers of x = w-w_s = h_bar:\n');
    for n = 0:numel(cN)-1
        fprintf('   x^%-2d : %-12s = %.6f\n', n, char(cN(n+1)), double(cN(n+1)));
    end
    fprintf('   a_bar(1) = %s,  d a_bar/dx (1) = %s,  a_bar(inf) = %s\n', ...
        char(subs(D, x, 1)), char(subs(diff(D, x), x, 1)), char(limit(D, x, inf)));
    chk = max(abs(abar - double(subs(N, x, h_bar)) ./ h_bar.^12));
    fprintf('   max |1/c_perp - N/x^12| over the plotted grid = %.2e\n', chk);

    % --- figure (house style) --------------------------------------------------
    COL_TRUE = [0.8 0 0]; FS = 18; LFS = 13; AXLW = 2.0;
    f = figure('Position', [80 80 1100 560], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    ht = plot(h_bar, abar, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, ...
              'DisplayName', 'a_z / a_{nom}  (true, 1 / c_\perp)');
    xlim([1 10]); ylim([0 1]); xticks(1:10);
    xlabel('(w - w_s) / R  = h / R', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel('a_z / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ht, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
           'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    exportgraphics(f, out_png, 'Resolution', 150);
    close(f);
    fprintf('[plot_abar_vs_gap] -> %s\n', out_png);
end
