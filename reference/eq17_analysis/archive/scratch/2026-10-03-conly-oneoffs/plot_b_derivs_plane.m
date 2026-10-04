% PURPOSE (2026-09-14, user: "plot b, b' and b'' of the wall, only the w_s case"): the constant the law would need at
%   each height when the wall parameter is held at the house value ws0_perp = 1, i.e. w_s = ws0_perp - 1 = 0:
%       b(w) = 1 / [ (1 - a_true(w)) * (w - w_s) ]        from  a = 1 - 1/(b (w - w_s))
%   and its first two derivatives in w (central differences on the same curve).
%   plot_b_derivs_plane('plane')  a_true(w) = 1/c_perp(w)            the Brenner plane
%   plot_b_derivs_plane('cell')   a_true(w) = 1/c_perp(w + 1.03)     the same curve with its no-slip surface 1.03 R
%     deeper (run_three_walls 'cellb'). The 1.03 R is NOT an independent physical number: it is the difference of the
%     two law-family origins fitted to Meng Fig 11 on 09-07 (plane -0.168, cell -1.197), i.e. another way of writing
%     the same fit.
%   Marker: the canonical band. The chord seed is NOT drawn -- it belongs to a different origin (w_s = -0.131).
%   Output reference/eq17_analysis/derivation/figures/b_derivs_<wall>.png | EXPIRES: with the b line | 產線改動不會自動跟上
function out = plot_b_derivs_plane(wall)
    if nargin < 1 || isempty(wall); wall = 'plane'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();
    ws = 0;                                        % ws0_perp = 1  =>  w_s = 0
    sh = 0;  if strcmp(wall, 'cell'); sh = 1.03; end
    w = (1.02:0.002:6)';  h = 1e-4;
    b   = local_b(w, sh, ws);
    bp  = (local_b(w + h, sh, ws) - local_b(w - h, sh, ws)) / (2*h);
    bpp = (local_b(w + h, sh, ws) - 2*b + local_b(w - h, sh, ws)) / h^2;
    cfg = canonical_scenario(0.05, 1.1, 'deep');
    w_lo = cfg.h_bottom / pc.R;  w_hi = (cfg.h_bottom + 2*cfg.amplitude) / pc.R;
    w0 = cfg.h_init / pc.R;  [~, cp0] = calc_correction_functions(w0, true);  a0 = 1/cp0;
    b_chord = (1/(1 - a0) - 1) / (w0 - 1.0);
    m = w >= w_lo & w <= w_hi;
    fprintf(['[%s, w_s = %.2f] band [%.2f %.2f] R:  b %.4f .. %.4f  |  db/dw %.4f .. %.4f  |  ', ...
             'd2b/dw2 %.3f .. %.3f  |  chord seed %.4f\n'], wall, ws, w_lo, w_hi, min(b(m)), max(b(m)), ...
            min(bp(m)), max(bp(m)), min(bpp(m)), max(bpp(m)), b_chord);
    FS = 15;  LFS = 13;  AXLW = 1.8;  C = [0.85 0 0];
    Y  = {b, bp, bpp};
    YL = {'$b$', '$\mathrm{d}b\,/\,\mathrm{d}\bar{w}$', '$\mathrm{d}^{2}b\,/\,\mathrm{d}\bar{w}^{2}$'};
    f = figure('Units','inches','Position',[0 0 7.5 8.5], 'Color','w', 'Visible','off');
    tiledlayout(3, 1, 'TileSpacing','compact', 'Padding','compact');
    for r = 1:3
        nexttile; hold on; box on;
        hh = plot(w, Y{r}, '-', 'Color', C, 'LineWidth', 2.2);
        yl = local_lim(Y{r}(w >= 1.05));
        plot([w_lo w_lo], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility','off');
        plot([w_hi w_hi], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility','off');
        legend(hh, YL(r), 'Interpreter','latex', 'Location','northoutside', 'Orientation','horizontal', ...
               'FontSize', LFS, 'Box','on');
        ylabel(YL{r}, 'Interpreter','latex', 'FontSize', FS+4);
        set(gca, 'FontSize', FS, 'FontWeight','bold', 'LineWidth', AXLW);
        xlim([w(1) w(end)]);  ylim(yl);
        if r == 3; xlabel('$\bar{w}$', 'Interpreter','latex', 'FontSize', FS+4); end
    end
    fn = fullfile(od, sprintf('b_derivs_%s.png', wall));  exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure: %s\n', fn);
    out = struct('w', w, 'b', b, 'bp', bp, 'bpp', bpp, 'b_chord', b_chord, 'band', [w_lo w_hi], 'ws', ws, 'shift', sh);
end

function b = local_b(w, shift, ws)
    b = zeros(size(w));
    for i = 1:numel(w)
        [~, cp] = calc_correction_functions(max(w(i) + shift, 1.001), true);
        a = 1/cp;
        b(i) = 1 / ((1 - a) * (w(i) - ws));
    end
end

function yl = local_lim(y)
    y = y(isfinite(y));  lo = min(y);  hi = max(y);  pad = 0.08*(hi - lo) + eps;
    yl = [lo - pad, hi + pad];
end
