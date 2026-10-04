% PURPOSE (2026-09-14, user: "overlay the a curves of those b values"): what a CONSTANT b can and cannot do. For each
%   wall the true gain curve is drawn together with the law a = 1 - 1/(b (w - w_s)), w_s = ws0_perp - 1 = 0, for the
%   three constants that match the truth exactly at the valley, the mid band and the far end.
%     plane   a_true(w) = 1/c_perp(w)
%     cell    a_true(w) = 1/c_perp(w + 1.03)     (run_three_walls 'cellb'; the 1.03 R is the difference of the two
%             law-family origins fitted to Meng Fig 11 on 09-07, not an independent physical number)
%   Output reference/eq17_analysis/derivation/figures/a_curves_const_b.png | EXPIRES: with the b line | 產線改動不會自動跟上
function out = plot_a_curves_const_b()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    if ~exist(od, 'dir'); mkdir(od); end
    pc = physical_constants();  cfg = canonical_scenario(0.05, 1.1, 'deep');
    w_lo = cfg.h_bottom / pc.R;  w_hi = (cfg.h_bottom + 2*cfg.amplitude) / pc.R;
    ws = 0;  WALL = {'plane', 'cell'};  SH = [0, 1.03];  W_FIT = [w_lo, 2.2, 6.0];
    COL = {[0.85 0 0], [0.9 0.5 0], [0 0.2 0.9]};
    FS = 15;  LFS = 12;  AXLW = 1.8;  w = (1.02:0.002:6)';  out = struct();
    f = figure('Units','inches','Position',[0 0 12 5], 'Color','w', 'Visible','off');
    tiledlayout(1, 2, 'TileSpacing','compact', 'Padding','compact');
    for c = 1:2
        at = local_a(w, SH(c));
        nexttile; hold on; box on;  H = gobjects(1, 4);
        H(1) = plot(w, at, '-', 'Color', 'k', 'LineWidth', 2.6);
        bs = zeros(1, 3);
        for j = 1:3
            aj = local_a(W_FIT(j), SH(c));
            bs(j) = 1 / ((1 - aj) * (W_FIT(j) - ws));
            H(1+j) = plot(w, 1 - 1./(bs(j)*(w - ws)), '-', 'Color', COL{j}, 'LineWidth', 1.8);
        end
        plot([w_lo w_lo], [0 1], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility','off');
        plot([w_hi w_hi], [0 1], '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility','off');
        legend(H, {sprintf('%s: true', WALL{c}), sprintf('$b = %.3f$', bs(1)), sprintf('$b = %.3f$', bs(2)), ...
                   sprintf('$b = %.3f$', bs(3))}, 'Interpreter','latex', 'Location','southeast', 'FontSize', LFS, 'Box','on');
        if c == 1; ylabel('$\bar{a}$', 'Interpreter','latex', 'FontSize', FS+4); end
        xlabel('$\bar{w}$', 'Interpreter','latex', 'FontSize', FS+4);
        set(gca, 'FontSize', FS, 'FontWeight','bold', 'LineWidth', AXLW);  xlim([w(1) 6]);  ylim([0 1]);
        out.(WALL{c}) = struct('b_fit', bs, 'w_fit', W_FIT);
        fprintf('[%s] constants that match at w = %.2f / %.2f / %.2f : b = %.3f / %.3f / %.3f\n', ...
                WALL{c}, W_FIT, bs);
    end
    fn = fullfile(od, 'a_curves_const_b.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure: %s\n', fn);
end

function a = local_a(w, shift)
    a = zeros(size(w));
    for i = 1:numel(w)
        [~, cp] = calc_correction_functions(max(w(i) + shift, 1.001), true);
        a(i) = 1/cp;
    end
end
