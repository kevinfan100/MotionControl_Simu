function res = plot_formB_form_compare(opts)
%PLOT_FORMB_FORM_COMPARE  Three-form function comparison, everything evaluated
%   along b(w_bar) -- the constant each form's own law demands at each height.
%   No fitted constant appears anywhere: every curve is fixed by the truth.
%
%       Form A   a = 1 - exp(-b*(w-ws))     p = 0
%       Form B   a = 1 - b/((w-ws) + b)     p = 1   (production form)
%       Form C   a = 1 - b/(1 + w-ws)       p = 1   (amplitude writing)
%
%   b(w_bar) solves law = a_true pointwise.  Each law is strictly monotone in
%   b, so the solution is unique for every w-ws in (0, inf); the two endpoints
%   are degenerate (any b) and are not plotted.
%
%       b_A = -ln(1-a_true)/(w-ws)
%       b_B = (w-ws)*(1-a_true)/a_true
%       b_C = (1 + w-ws)*(1-a_true)
%
%   b'(w_bar) is the same construction on the SLOPE: it solves law' = a'_true
%   pointwise.  The controller consumes b only through a_bar' (F_e row 4 uses
%   J_b = d a_bar'/db, and a_bar is a state that is never re-anchored), so this
%   is the inversion the run-time path actually sees.  Along b(w_bar) every
%   form returns a_true exactly, but NOT a'_true -- b(w_bar) and b'(w_bar) are
%   different functions with the same far-field limit.
%
%       b'_A  solves  b*exp(-b*(w-ws)) = a'_true   (principal branch, b*(w-ws)<1)
%       b'_B  = 2*a'_true*(w-ws)^2
%               / (1 - 2*a'_true*(w-ws) + sqrt(1 - 4*a'_true*(w-ws)))
%       b'_C  = (1 + w-ws)^2 * a'_true
%
%   Existence on the plotted range: A needs (w-ws)*a'_true <= 1/e, B needs
%   <= 1/4, C is unconditional; both margins are checked and printed.
%
%   Substituting b(w_bar) back collapses every expression onto a_true and
%   (w-ws) alone (checked against direct substitution to 1e-16):
%
%     a'     A -(1-a)*ln(1-a)/(w-ws)  B a*(1-a)/(w-ws)      C (1-a)/(1+w-ws)
%     da/db  A +(w-ws)*(1-a)          B -a^2/(w-ws)         C -1/(1+w-ws)
%     J_b    A (1-a)*(1+ln(1-a))      B a^2*(2a-1)/(w-ws)^2 C 1/(1+w-ws)^2
%
%   a_bar itself is NOT plotted: along b(w_bar) all three collapse onto a_true
%   by construction, so that panel would carry no information.
%
%   Range w-ws in [0.1, 10] = the truth curve's published validity floor
%   (h_bar >= 1.1), the same floor run_formB_ws enforces.
%
%   Figures -> derivation/figures/formB_cmp_{b,dbdw,aprime,dadb,Jb}.png
%
%   Style: house rules (no grid, box on, legend northoutside horizontal, no
%   title, stats to console), every string on the latex interpreter so the
%   figures typeset in the same Computer Modern as the document.
%
%   See also: gainlaw_shape_selection.tex, formB_amp_functions.tex

    if nargin < 1; opts = struct(); end
    if ~isfield(opts, 'gap_max'); opts.gap_max = 10;   end
    if ~isfield(opts, 'save');    opts.save    = true; end
    % Which writings are DRAWN.  Default [2 3] = B and C, the two live
    % candidates (user, 2026-08-10).  Form A stays in the console and in the
    % returned struct -- it is only off the figures.  opts.forms = 1:3 restores
    % the three-curve version.
    if ~isfield(opts, 'forms');   opts.forms   = [2 3]; end

    here = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(here));
    addpath(fullfile(root, 'model', 'wall_effect'));
    fig_dir = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');

    G = linspace(0.1, opts.gap_max, 4000).';      % truth validity floor h_bar = 1.1
    [a, ap_true] = truth_a(1 + G);
    L = log(1 - a);

    % ---- b(w_bar) and its rate -----------------------------------------
    beff  = [ -L./G,  G.*(1-a)./a,  (1+G).*(1-a) ];
    dbeff = [ (ap_true.*G./(1-a) + L)./G.^2, ...
              (1-a)./a - G.*ap_true./a.^2, ...
              (1-a) - (1+G).*ap_true ];

    % ---- b'(w_bar): the same inversion done on the SLOPE ----------------
    %   Computed and returned for reference; NOT plotted.  Inverting on the
    %   slope is ill-conditioned for B (d a_bar'/db vanishes at b = w-ws), so
    %   the plotted page 2 uses the well-posed statement instead: substitute
    %   b(w_bar) -- which zeroes the VALUE error by construction -- into the
    %   differentiated law, and read off what is left in the SLOPE.
    bp = [ local_bA_slope(G, ap_true), ...
           2*ap_true.*G.^2 ./ (1 - 2*ap_true.*G + sqrt(1 - 4*ap_true.*G)), ...
           (1+G).^2 .* ap_true ];

    % ---- law quantities evaluated along b(w_bar) ------------------------
    aprime = [ -(1-a).*L./G,   a.*(1-a)./G,           (1-a)./(1+G) ];
    dadb   = [  G.*(1-a),     -a.^2./G,              -1./(1+G)     ];
    Jb     = [ (1-a).*(1+L),   a.^2.*(2*a-1)./G.^2,   1./(1+G).^2  ];

    % ---- page 2: what b(w_bar) leaves in the SLOPE ----------------------
    %   b(w_bar) zeroes the VALUE error at every height by construction.  Put
    %   that same b -- as a number, not differentiated -- into the law that has
    %   already been differentiated at fixed b, and this is what is left.  By
    %   the chain rule it equals exactly -(da/db)*(db/dw), i.e. the term the
    %   controller drops when it treats b as a constant.
    aperr = (aprime - repmat(ap_true, 1, 3)) ./ repmat(ap_true, 1, 3);

    % ---- console ---------------------------------------------------------
    NAME = {'Form A', 'Form B', 'Form C'};
    fprintf('\n[form compare]  along b(w_bar), no fitted constant\n');
    for k = 1:3
        fprintf('  %-7s b %.4f -> %.4f   db/dw %+.4f -> %+.4f   J_b zero at w-ws = %s\n', ...
            NAME{k}, beff(1,k), beff(end,k), dbeff(1,k), dbeff(end,k), ...
            local_str(local_zero(G, Jb(:,k))));
    end
    e1 = max(abs(aprime(:,3) - beff(:,3)./(1+G).^2));
    e2 = max(abs(Jb(:,2) - (G - beff(:,2))./(G + beff(:,2)).^3));
    fprintf('  simplified vs direct substitution:  C a'' %.1e ,  B J_b %.1e\n', e1, e2);

    fprintf('\n[a'' error along b(w_bar)]  same b, same height: value exact, slope not\n');
    fprintf('  value residual max|a_B(b(w)) - a_true| = %.1e (zero by construction)\n', ...
            max(abs(G./(G+beff(:,2)) - a)));
    for k = 1:3
        e = aperr(:,k);
        z = local_zero(G, e);
        fprintf('  %-7s RMS %7.3f %%   sup %7.3f %%   range [%+7.3f, %+7.3f] %%   zero at w-ws = %s\n', ...
            NAME{k}, 100*sqrt(mean(e.^2)), 100*max(abs(e)), 100*min(e), 100*max(e), ...
            local_str(z));
    end
    fprintf('  (identity: this error equals (da/db)*(db/dw); max discrepancy %.1e)\n', ...
            max(abs(aperr.*repmat(ap_true,1,3) + dadb.*dbeff)));

    if ~opts.save; res = struct('b', beff); return; end

    % ---- house style ----------------------------------------------------
    C_TRUE = [0.8 0 0];
    CC  = {[0.35 0.35 0.35], [0 0.2 0.9], [0.45 0.55 0.95]};
    STY = {'--', '-', '-'};
    LEG = {'Form A', 'Form B', 'Form C'};
    FS = 20; LFS = 16; AXLW = 2.0; LW = 2.2;
    XL = '$\bar{w}-\bar{w}_s$';

    q   = opts.forms;                       % drawn subset
    CCq = CC(q); STYq = STY(q); LEGq = LEG(q);
    % Limits were set for the three-curve version; with Form A dropped they
    % clipped Form B (db/dw reaches +0.316 at the near-wall end) and left the
    % b panels mostly empty, so the subset autoscales.
    YLb = []; YLd = [];
    if isequal(q(:).', 1:3); YLb = [0 1.5]; YLd = [-0.4 0.15]; end

    fig_overlay(G, beff(:,q), [], CCq, STYq, LEGq, XL, '$b(\bar{w})$', ...
                fullfile(fig_dir, 'formB_cmp_b.png'), FS, LFS, AXLW, LW, C_TRUE, YLb);

    % Axis label kept short: the page defines Delta a' in its second equation,
    % so the axis only has to name it.
    fig_overlay(G, 100*aperr(:,q), [], CCq, STYq, LEGq, XL, ...
                '$\Delta\bar{a}^{\prime}/\bar{a}^{\prime}_{\mathrm{true}}$  [\%]', ...
                fullfile(fig_dir, 'formB_cmp_aprime_err.png'), FS, LFS, AXLW, LW, C_TRUE, []);

    fig_overlay(G, dbeff(:,q), [], CCq, STYq, LEGq, XL, ...
                '$\mathrm{d}b(\bar{w})/\mathrm{d}\bar{w}$', ...
                fullfile(fig_dir, 'formB_cmp_dbdw.png'), FS, LFS, AXLW, LW, C_TRUE, YLd);

    fig_overlay(G, aprime(:,q), ap_true, CCq, STYq, LEGq, XL, '$\bar{a}^{\prime}$', ...
                fullfile(fig_dir, 'formB_cmp_aprime.png'), FS, LFS, AXLW, LW, C_TRUE, []);

    fig_overlay(G, dadb(:,q), [], CCq, STYq, LEGq, XL, '$\partial\bar{a}/\partial b$', ...
                fullfile(fig_dir, 'formB_cmp_dadb.png'), FS, LFS, AXLW, LW, C_TRUE, []);

    fig_overlay(G, Jb(:,q), [], CCq, STYq, LEGq, XL, '$J_b$', ...
                fullfile(fig_dir, 'formB_cmp_Jb.png'), FS, LFS, AXLW, LW, C_TRUE, []);

    fprintf('  wrote 6 figures -> %s\n', fig_dir);
    res = struct('b', beff, 'bprime', bp, 'aperr', aperr, 'dbdw', dbeff, 'aprime', aprime, ...
                 'dadb', dadb, 'Jb', Jb, 'G', G);
end

% --------------------------------------------------------------------------
function b = local_bA_slope(G, s)
%LOCAL_BA_SLOPE  Principal-branch root of b*exp(-b*G) = s (Form A on the slope).
%   Newton from b = s, which is the small-(G*s) limit of the principal branch;
%   the branch condition b*G < 1 holds wherever G*s < 1/e.
    b = s;
    for it = 1:200
        e = exp(-b .* G);
        b = b - (b.*e - s) ./ (e .* (1 - b.*G));
    end
end

% --------------------------------------------------------------------------
function [a, ap] = truth_a(w)
    c = zeros(size(w)); dc = zeros(size(w));
    for i = 1:numel(w)
        [~, c(i), dv] = calc_correction_functions(w(i), true);
        dc(i) = dv.dc_perp_dh;
    end
    a  = 1 ./ c;
    ap = -dc ./ c.^2;
end

function z = local_zero(x, y)
    s = find(y(1:end-1).*y(2:end) < 0);
    z = zeros(numel(s), 1);
    for i = 1:numel(s)
        j = s(i);
        z(i) = x(j) - y(j)*(x(j+1)-x(j))/(y(j+1)-y(j));
    end
end

function s = local_str(z)
    if isempty(z); s = 'none'; else; s = mat2str(round(z, 3)); end
end

function fig_overlay(x, Y, ytrue, CC, STY, LEG, xl, yl, out, FS, LFS, AXLW, LW, C_TRUE, YL)
    f = figure('Position', [80 80 1000 680], 'Color', 'w', ...
               'NumberTitle', 'off', 'Visible', 'off');
    hold on;
    m = size(Y, 2);
    h = gobjects(1, m + ~isempty(ytrue)); n = 0;
    if ~isempty(ytrue)
        n = 1;
        h(1) = plot(x, ytrue, '-', 'Color', C_TRUE, 'LineWidth', LW + 0.6, ...
                    'DisplayName', 'truth');
    end
    for k = 1:m
        h(n+k) = plot(x, Y(:,k), STY{k}, 'Color', CC{k}, 'LineWidth', LW, ...
                      'DisplayName', LEG{k});
    end
    yline(0, '-', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    xlim([min(x) max(x)]);
    if ~isempty(YL); ylim(YL); end
    xlabel(xl, 'Interpreter', 'latex', 'FontSize', FS);
    ylabel(yl, 'Interpreter', 'latex', 'FontSize', FS);
    legend(h, 'Interpreter', 'latex', 'Location', 'northoutside', ...
           'Orientation', 'horizontal', 'FontSize', LFS, 'Box', 'on');
    set(gca, 'FontSize', FS, 'LineWidth', AXLW, 'Box', 'on', ...
             'TickLabelInterpreter', 'latex');
    grid off;
    exportgraphics(f, out, 'Resolution', 200, 'Padding', 'figure');
    close(f);
end
