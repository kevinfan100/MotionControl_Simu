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

    % ---- law quantities evaluated along b(w_bar) ------------------------
    aprime = [ -(1-a).*L./G,   a.*(1-a)./G,           (1-a)./(1+G) ];
    dadb   = [  G.*(1-a),     -a.^2./G,              -1./(1+G)     ];
    Jb     = [ (1-a).*(1+L),   a.^2.*(2*a-1)./G.^2,   1./(1+G).^2  ];

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

    if ~opts.save; res = struct('b', beff); return; end

    % ---- house style ----------------------------------------------------
    C_TRUE = [0.8 0 0];
    CC  = {[0.35 0.35 0.35], [0 0.2 0.9], [0.45 0.55 0.95]};
    STY = {'--', '-', '-'};
    LEG = {'Form A', 'Form B', 'Form C'};
    FS = 20; LFS = 16; AXLW = 2.0; LW = 2.2;
    XL = '$\bar{w}-\bar{w}_s$';

    fig_overlay(G, beff, [], CC, STY, LEG, XL, '$b(\bar{w})$', ...
                fullfile(fig_dir, 'formB_cmp_b.png'), FS, LFS, AXLW, LW, C_TRUE, [0 1.5]);

    fig_overlay(G, dbeff, [], CC, STY, LEG, XL, ...
                '$\mathrm{d}b(\bar{w})/\mathrm{d}\bar{w}$', ...
                fullfile(fig_dir, 'formB_cmp_dbdw.png'), FS, LFS, AXLW, LW, C_TRUE, [-0.4 0.15]);

    fig_overlay(G, aprime, ap_true, CC, STY, LEG, XL, '$\bar{a}^{\prime}$', ...
                fullfile(fig_dir, 'formB_cmp_aprime.png'), FS, LFS, AXLW, LW, C_TRUE, []);

    fig_overlay(G, dadb, [], CC, STY, LEG, XL, '$\partial\bar{a}/\partial b$', ...
                fullfile(fig_dir, 'formB_cmp_dadb.png'), FS, LFS, AXLW, LW, C_TRUE, []);

    fig_overlay(G, Jb, [], CC, STY, LEG, XL, '$J_b$', ...
                fullfile(fig_dir, 'formB_cmp_Jb.png'), FS, LFS, AXLW, LW, C_TRUE, []);

    fprintf('  wrote 5 figures -> %s\n', fig_dir);
    res = struct('b', beff, 'dbdw', dbeff, 'aprime', aprime, 'dadb', dadb, ...
                 'Jb', Jb, 'G', G);
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
    h = gobjects(1, 3 + ~isempty(ytrue)); n = 0;
    if ~isempty(ytrue)
        n = 1;
        h(1) = plot(x, ytrue, '-', 'Color', C_TRUE, 'LineWidth', LW + 0.6, ...
                    'DisplayName', 'truth');
    end
    for k = 1:3
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
