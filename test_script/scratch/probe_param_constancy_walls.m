% PURPOSE (2026-09-21, user: "with one parameter, how do we describe a' so that the parameter is as constant as possible?"):
%   the EFFECTIVE parameter along the band h in [1.10, 22.2] -- the value the one constant would have to take at each height for the
%   slope law to be exact there -- for three writings of the slope, on plane / cell (cell_cperp_delta) / sphere (sphere_cperp_fig11):
%     (1) advisor, height writing   a' = (1 - c)/h^p, p = 2.042        c_eff(h) = 1 - h^p a'_true(h)
%     (2) advisor, value writing    a' = (1 - a)^2/(1 - c)  (p = 2)    c_eff(h) = 1 - (1 - a_true)^2 / a'_true     [= production's b law, b = 1/(1-c)]
%     (3) plane base, value writing a' = s * Lambda_plane(a)            s_eff(h) = a'_true / Lambda_plane(a_true)
%   Constancy = the spread of the effective parameter over the band (max - min) and the slope residual left by its best constant.
%   Figure: rows (1) (2) (3), columns walls, effective parameter against h with the band marked; console table.
%   Output reference/eq17_analysis/derivation/figures/param_constancy_walls.png | EXPIRES: with the one-constant-family line
function probe_param_constancy_walls()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    addpath(here);
    D_CELL = 1.05;  P0 = 2.042;
    D   = @(u) 1 - 9/8*u + 1/2*u.^3 - 57/100*u.^4 + 1/5*u.^5 + 7/200*u.^11 - 1/25*u.^12;
    Dp  = @(u) -9/8 + 3/2*u.^2 - 228/100*u.^3 + u.^4 + 77/200*u.^10 - 12/25*u.^11;
    a_pl  = @(h) D(1 ./ h);  ap_pl = @(h) -Dp(1 ./ h) ./ h.^2;
    a_ce  = @(h) a_pl(h + D_CELL);  ap_ce = @(h) ap_pl(h + D_CELL);
    a_sp  = @(h) 1 ./ sphere_cperp_fig11(h);  ap_sp = @(h) (a_sp(max(h, 1 + 2e-3) + 1e-4) - a_sp(max(h, 1 + 2e-3) - 1e-4)) / 2e-4;
    ht = linspace(1, 60, 20000)';  Lam_pl = @(a) interp1(a_pl(ht), ap_pl(ht), a, 'pchip');
    hU = linspace(1.10, 22.2, 4000)';
    W = {'plane', a_pl, ap_pl;  'cell', a_ce, ap_ce;  'sphere', a_sp, ap_sp};
    F = {'(1) advisor, height: c_{eff} = 1 - h^p a''',      @(a, ap, h) 1 - h.^P0 .* ap,        @(a, ap, h, th) (1 - th) ./ h.^P0; ...
         '(2) advisor, value (p = 2): c_{eff} = 1 - (1-a)^2/a''', @(a, ap, h) 1 - (1 - a).^2 ./ ap, @(a, ap, h, th) (1 - a).^2 ./ (1 - th); ...
         '(3) plane base, value: s_{eff} = a''/\Lambda_{plane}(a)', @(a, ap, h) ap ./ Lam_pl(a),  @(a, ap, h, th) th .* Lam_pl(a)};
    fprintf('\neffective parameter along the band [1.10, 22.2]: value at 1.1 / 2.0 / 5.5 / 22.2, spread (max - min), best constant (LS on a'') and its slope residual sup|e_a''|\n');
    TH = zeros(3, 3);
    for f = 1:3
        fprintf('%s\n', regexprep(F{f, 1}, '[{}\\]', ''));
        for w = 1:3
            af = W{w, 2};  apf = W{w, 3};  a = af(hU);  ap = apf(hU);  th = F{f, 2}(a, ap, hU);
            % best constant: LS of a' against the law's parameter-linear form
            switch f
                case 1; th0 = 1 - ((1 ./ hU.^P0) \ ap);
                case 2; th0 = 1 - 1 / (((1 - a).^2) \ ap);
                case 3; th0 = (Lam_pl(a) \ ap);
            end
            TH(f, w) = th0;  e = ap - F{f, 3}(a, ap, hU, th0);
            q = @(x) F{f, 2}(af(x), apf(x), x);
            fprintf('  %-6s  at 1.1 %+.3f  2.0 %+.3f  5.5 %+.3f  22.2 %+.3f | spread %.3f | best constant %+.3f -> sup|e_a''| %.3f (at 1.1 %+.3f)\n', ...
                W{w, 1}, q(1.1), q(2.0), q(5.5), q(22.2), max(th) - min(th), th0, max(abs(e)), apf(1.1) - F{f, 3}(af(1.1), apf(1.1), 1.1, th0));
        end
    end
    % --- figure
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; FS = 18; LFS = 12; AXLW = 2.0; CG = [0.5 0.5 0.5];
    hx = linspace(1.10, 22.2, 2000)';
    fig = figure('Position', [60 60 1800 1250], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    YL = {[-0.4 1.0], [-0.4 1.0], [0.6 1.6]};  YLAB = {'c_{eff}', 'c_{eff}', 's_{eff}'};
    for f = 1:3
        for w = 1:3
            af = W{w, 2};  apf = W{w, 3};
            nexttile((f - 1) * 3 + w); hold on; box on;
            h1 = plot(hx, F{f, 2}(af(hx), apf(hx), hx), '-', 'Color', COL_TRUE, 'LineWidth', 2.2, 'DisplayName', 'effective parameter the plant asks for at h');
            h2 = yline(TH(f, w), '-', 'Color', COL_HAT, 'LineWidth', 1.6, 'DisplayName', sprintf('best constant %+.3f', TH(f, w)));
            h0 = plot(NaN, NaN, 'w.', 'DisplayName', sprintf('%s   %s', upper(W{w, 1}), F{f, 1}));
            xlim([1 23]); ylim(YL{f});  xline(1.10, '--', 'Color', CG, 'HandleVisibility', 'off');  xline(22.2, '--', 'Color', CG, 'HandleVisibility', 'off');
            if w == 1; ylabel(YLAB{f}, 'FontSize', FS, 'FontWeight', 'bold'); end
            if f == 3; xlabel('h = w - w_s  (h / R)', 'FontSize', FS, 'FontWeight', 'bold'); end
            legend([h0 h1 h2], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
        end
    end
    fn = fullfile(od, 'param_constancy_walls.png');
    exportgraphics(fig, fn, 'Resolution', 150); close(fig); fprintf('figure: %s\n', fn);
end
