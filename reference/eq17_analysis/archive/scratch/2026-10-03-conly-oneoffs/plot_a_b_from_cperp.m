% PURPOSE (2026-09-14, user: "we have c(h), so solve the b curve directly -- give me that and its derivatives, and the
%   a curve of the c(h) you used for the cell"): everything is read straight out of the known correction function.
%     a_true(w)  = 1 / c_perp(w + shift)
%     a'_true(w) = -(dc_perp/dh) / c_perp^2          (the analytic derivative the model already returns)
%     b(w)       = a'_true(w) / (1 - a_true(w))^2     <- the law a = 1 - 1/(b (w - w_s)) solved for b through its
%                                                        DERIVATIVE form a' = b (1-a)^2, with ws0_perp = 1 (w_s = 0)
%     db/dw, d2b/dw2 by central differences on that curve.
%   shift = 0 plane (Brenner), shift = 1.03 cell (the no-slip surface 1.03 R deeper; that 1.03 is the difference of the
%   two law-family origins fitted to Meng Fig 11 on 09-07, not an independent physical number).
%   Output reference/eq17_analysis/derivation/figures/a_b_from_cperp.png | EXPIRES: with the b line | 產線改動不會自動跟上
function out = plot_a_b_from_cperp()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  cfg = canonical_scenario(0.05, 1.1, 'deep');
    w_lo = cfg.h_bottom / pc.R;  w_hi = (cfg.h_bottom + 2*cfg.amplitude) / pc.R;
    WALL = {'plane', 'cell'};  SH = [0, 1.03];
    w = (1.02:0.002:6)';  hh = 1e-4;  FS = 15;  LFS = 13;  AXLW = 1.8;  C = [0.85 0 0];
    YL = {'$\bar{a}$', '$b$', '$\mathrm{d}b\,/\,\mathrm{d}\bar{w}$', '$\mathrm{d}^{2}b\,/\,\mathrm{d}\bar{w}^{2}$'};
    f = figure('Units','inches','Position',[0 0 12 11], 'Color','w', 'Visible','off');
    tiledlayout(4, 2, 'TileSpacing','compact', 'Padding','compact');  out = struct();
    for c = 1:2
        a   = local_a(w, SH(c));
        b   = local_b(w, SH(c));
        bp  = (local_b(w + hh, SH(c)) - local_b(w - hh, SH(c))) / (2*hh);
        bpp = (local_b(w + hh, SH(c)) - 2*b + local_b(w - hh, SH(c))) / hh^2;
        m = w >= w_lo & w <= w_hi;  [bmin, im] = min(b(m));  wm = w(find(m, 1) + im - 1);
        fprintf(['[%s] band [%.2f %.2f] R:  a %.3f .. %.3f  |  b %.4f .. %.4f (minimum %.4f at w = %.2f)  |  ', ...
                 'db/dw %.4f .. %.4f  |  d2b/dw2 %.3f .. %.3f\n'], WALL{c}, w_lo, w_hi, min(a(m)), max(a(m)), ...
                min(b(m)), max(b(m)), bmin, wm, min(bp(m)), max(bp(m)), min(bpp(m)), max(bpp(m)));
        Y = {a, b, bp, bpp};
        for r = 1:4
            nexttile(2*(r-1) + c); hold on; box on;
            hcur = plot(w, Y{r}, '-', 'Color', C, 'LineWidth', 2.2);
            yl = local_lim(Y{r}(w >= 1.05));
            plot([w_lo w_lo], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility','off');
            plot([w_hi w_hi], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility','off');
            legend(hcur, {sprintf('%s', WALL{c})}, 'Location','northoutside', 'Orientation','horizontal', ...
                   'FontSize', LFS, 'FontWeight','bold', 'Box','on');
            if c == 1; ylabel(YL{r}, 'Interpreter','latex', 'FontSize', FS+4); end
            set(gca, 'FontSize', FS, 'FontWeight','bold', 'LineWidth', AXLW);  xlim([w(1) w(end)]);  ylim(yl);
            if r == 4; xlabel('$\bar{w}$', 'Interpreter','latex', 'FontSize', FS+4); end
        end
        out.(WALL{c}) = struct('w', w, 'a', a, 'b', b, 'bp', bp, 'bpp', bpp, 'b_min', bmin, 'w_min', wm);
    end
    fn = fullfile(od, 'a_b_from_cperp.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure: %s\n', fn);
end

function a = local_a(w, shift)
    a = zeros(size(w));
    for i = 1:numel(w)
        [~, cp] = calc_correction_functions(max(w(i) + shift, 1.001), true);
        a(i) = 1/cp;
    end
end

function b = local_b(w, shift)
    b = zeros(size(w));
    for i = 1:numel(w)
        [~, cp, dd] = calc_correction_functions(max(w(i) + shift, 1.001), true);
        a = 1/cp;  ap = -dd.dc_perp_dh / cp^2;
        b(i) = ap / (1 - a)^2;
    end
end

function yl = local_lim(y)
    y = y(isfinite(y));  lo = min(y);  hi = max(y);  pad = 0.08*(hi - lo) + eps;
    yl = [lo - pad, hi + pad];
end
