function plot_formB_amp_functions(opts)
%PLOT_FORMB_AMP_FUNCTIONS  Figures for the Form B amplitude writing
%   a_bar = 1 - b/(1 + (w_bar - w_bar_s)),  p = 1,  w_bar_s KNOWN,
%   with b solved pointwise from the truth so that b VARIES with height.
%
%   Feeds reference/eq17_analysis/derivation/formB_amp_functions.tex.
%   Figures are written to derivation/figures/ (committed, per the rule that
%   permanent artifacts may not depend on gitignored outputs).
%
%   Definitions (u is the CODE variable for 1 + (w_bar - w_bar_s); the document
%   writes it out in full, no shorthand):
%       b       = u*(1 - a_true)                 pointwise inversion, closed form
%       db/dw   = 1 - 1/c + u*c'/c^2             analytic
%       a       = 1 - b/u          ( == a_true by construction )
%       a'      = b/u^2 - (db/dw)/u              frozen term + drift term
%       da/db   = -1/u                           level sensitivity  (b-free)
%       J_b     = 1/u^2                          slope Jacobian     (b-free)
%
%   Truth: a_true = 1/c_perp, c_perp and dc_perp/dh from the SSOT
%   calc_correction_functions (two-sphere perpendicular series). Note the
%   published curve's stated validity floor is h_bar = 1.1; the grid here
%   starts at w_bar - w_bar_s = 1e-3 so the contact limit b -> 1 is visible.
%
%   opts.gap_max  (default 10)   x-axis upper end
%   opts.save     (default true)
%
%   See also: analyze_formB_amplitude_writing, plot_var_ahat_6state (style).

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'gap_max'); opts.gap_max = 10;   end
    if ~isfield(opts, 'save');    opts.save    = true; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(fullfile(root, 'model', 'wall_effect'));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    WS = 1;                       % w_bar_s, known
    B_FAR = 9/8;                  % far-field reflection coefficient
    B_NEAR = 1;                   % near-wall lubrication

    gap = linspace(1e-3, opts.gap_max, 4000).';
    u   = 1 + gap;                % = w_bar when w_bar_s = 1
    w   = u + (WS - 1);

    c = zeros(size(u)); dc = zeros(size(u));
    for i = 1:numel(u)
        [~, c(i), dv] = calc_correction_functions(w(i), true);
        dc(i) = dv.dc_perp_dh;
    end
    a_true  = 1 ./ c;
    ap_true = -dc ./ c.^2;

    b     = u .* (1 - a_true);
    dbdw  = 1 - 1./c + u .* dc ./ c.^2;
    a     = 1 - b ./ u;
    ap_fr = b ./ u.^2;            % frozen-b term
    ap_dr = -dbdw ./ u;           % b-drift term
    dA_db = -1 ./ u;
    J_b   = 1 ./ u.^2;

    fprintf('\n[formB amp functions]  w_s = %g, p = 1, gap in [%.3f, %g]\n', ...
            WS, gap(1), opts.gap_max);
    fprintf('  b      : %.4f -> %.4f   (anchors %.4f / %.4f)\n', b(1), b(end), B_NEAR, B_FAR);
    fprintf('  db/dw  : %.4f -> %.4f   peak %.4f at gap %.3f\n', ...
            dbdw(1), dbdw(end), max(dbdw), gap(dbdw == max(dbdw)));
    fprintf('  a''     : sum vs truth, max |diff| = %.3e\n', max(abs(ap_fr + ap_dr - ap_true)));
    fprintf('  drift term share |ap_dr/ap_fr| : max %.3f%% at gap %.3f\n', ...
            100*max(abs(ap_dr./ap_fr)), gap(abs(ap_dr./ap_fr) == max(abs(ap_dr./ap_fr))));

    if ~opts.save; return; end

    % --- house style (plot_var_ahat_6state) ---
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; COL_M = [0.45 0.55 0.95];
    COL_REF  = [0.45 0.45 0.45];
    FS = 20; LFS = 16; AXLW = 2.0; LW = 2.2;
    XL = '$\bar{w}-\bar{w}_s$';

    % ===== FIG 1: b =====
    f = new_fig();
    h1 = plot(gap, b, '-', 'Color', COL_TRUE, 'LineWidth', LW, 'DisplayName', '$b$');
    h2 = yline(B_NEAR, '--', 'Color', COL_REF, 'LineWidth', 1.5, 'DisplayName', '$1$');
    h3 = yline(B_FAR,  ':',  'Color', COL_REF, 'LineWidth', 1.5, 'DisplayName', '$9/8$');
    ylim([B_NEAR - 0.006, B_FAR + 0.006]);
    finish(f, gap, XL, '$b$', [h1 h2 h3], FS, LFS, AXLW, ...
           fullfile(fig_dir, 'formB_amp_b.png'));

    % ===== FIG 2: db/dw =====
    f = new_fig();
    h1 = plot(gap, dbdw, '-', 'Color', COL_TRUE, 'LineWidth', LW, ...
              'DisplayName', '$\mathrm{d}b/\mathrm{d}\bar{w}$');
    yline(0, '-', 'Color', COL_REF, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    finish(f, gap, XL, '$\mathrm{d}b/\mathrm{d}\bar{w}$', h1, FS, LFS, AXLW, ...
           fullfile(fig_dir, 'formB_amp_dbdw.png'));

    % ===== FIG 3: a =====
    f = new_fig();
    h1 = plot(gap, a, '-', 'Color', COL_TRUE, 'LineWidth', LW, 'DisplayName', '$\bar{a}$');
    finish(f, gap, XL, '$\bar{a}$', h1, FS, LFS, AXLW, ...
           fullfile(fig_dir, 'formB_amp_a.png'));

    % ===== FIG 4a: a' (final) =====
    f = new_fig();
    h1 = plot(gap, ap_fr + ap_dr, '-', 'Color', COL_TRUE, 'LineWidth', LW, ...
              'DisplayName', '$\bar{a}^{\prime}$');
    finish(f, gap, XL, '$\bar{a}^{\prime}$', h1, FS, LFS, AXLW, ...
           fullfile(fig_dir, 'formB_amp_aprime_final.png'));

    % ===== FIG 4b: a' decomposition =====
    f = new_fig();
    h1 = plot(gap, ap_fr, '-',  'Color', COL_HAT, 'LineWidth', LW, ...
              'DisplayName', '$b/(1+\bar{w}-\bar{w}_s)^{2}$');
    h2 = plot(gap, ap_dr, '-',  'Color', COL_M,   'LineWidth', LW, ...
              'DisplayName', '$-(\mathrm{d}b/\mathrm{d}\bar{w})/(1+\bar{w}-\bar{w}_s)$');
    h3 = plot(gap, ap_fr + ap_dr, '--', 'Color', COL_TRUE, 'LineWidth', LW, ...
              'DisplayName', '$\bar{a}^{\prime}$');
    yline(0, '-', 'Color', COL_REF, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    finish(f, gap, XL, '$\bar{a}^{\prime}$', [h1 h2 h3], FS, LFS, AXLW, ...
           fullfile(fig_dir, 'formB_amp_aprime.png'));

    % ===== FIG 5: level sensitivity =====
    f = new_fig();
    h1 = plot(gap, dA_db, '-', 'Color', COL_HAT, 'LineWidth', LW, ...
              'DisplayName', '$\partial\bar{a}/\partial b$');
    yline(0, '-', 'Color', COL_REF, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    finish(f, gap, XL, '$\partial\bar{a}/\partial b$', h1, FS, LFS, AXLW, ...
           fullfile(fig_dir, 'formB_amp_dadb.png'));

    % ===== FIG 6: slope Jacobian =====
    f = new_fig();
    h1 = plot(gap, J_b, '-', 'Color', COL_HAT, 'LineWidth', LW, ...
              'DisplayName', '$J_b=\partial\bar{a}^{\prime}/\partial b$');
    finish(f, gap, XL, '$J_b$', h1, FS, LFS, AXLW, ...
           fullfile(fig_dir, 'formB_amp_Jb.png'));

    fprintf('  wrote 7 figures -> %s\n', fig_dir);
end

% --------------------------------------------------------------------------
function f = new_fig()
    f = figure('Position', [80 80 1000 680], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    hold on;
end

function finish(f, x, xl, yl, hleg, FS, LFS, AXLW, out)
%FINISH  House style, with ALL text on the latex interpreter so the figures
%   typeset in the same Computer Modern as the .tex they are embedded in
%   (the canonical template's Helvetica bold is dropped for that reason).
    xlim([min(x) max(x)]);
    xlabel(xl, 'Interpreter', 'latex', 'FontSize', FS);
    if ~isempty(yl)
        ylabel(yl, 'Interpreter', 'latex', 'FontSize', FS);
    end
    legend(hleg, 'Interpreter', 'latex', 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
    set(gca, 'FontSize', FS, 'LineWidth', AXLW, 'Box', 'on', ...
             'TickLabelInterpreter', 'latex');
    grid off;
    exportgraphics(f, out, 'Resolution', 200, 'Padding', 'figure');
    close(f);
end
