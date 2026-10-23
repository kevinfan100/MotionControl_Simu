% PURPOSE (2026-09-28, user: "what does a lower bottom 1.10 -> 1.05 -> 1.01 R do to c_hat; motion end only, zoom in"): c-only law
%   (f1p2), plane init, prior 0.10, lambda = 1, T_d = 2 s, 10 seeds; runs f1p2_{plane,cell}_PI_sig010_ramp2{,_hb1.05,_hb1.01}.mat.
%   Columns plane | cell; rows: c_hat vs h_d (full, log reversed), c_hat zoom 2.0 -> 1.0 R, e_a zoom 2.0 -> 1.0 R; three bottoms
%   overlaid, descent-end points marked. Console: end c_hat, end e_a, s^c(h_b), lowest true gap, floor hits (health first).
%   House style. Output reference/eq17_analysis/derivation/figures/c_bottom_zoom.png   EXPIRES: with the c-only line
function plot_c_bottom_zoom()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    HB = {'', '_hb105', '_hb101'};  HBV = [1.10 1.05 1.01];  PL = {'plane', 'cell'};
    COLS = [0 0.2 0.9; 0.85 0.45 0; 0 0.6 0.2];  COL_TRUE = [0.8 0 0]; FS = 15; LFS = 11; AXLW = 2.0; CG = [0.5 0.5 0.5];
    Dp = @(u) -9/8 + (3/2)*u.^2 - 2.28*u.^3 + u.^4 + 0.385*u.^10 - 0.48*u.^11;
    ceff = struct('plane', @(h) 1 - h.^2 .* (-Dp(1 ./ h) ./ h.^2), 'cell', @(h) 1 - h.^2 .* (-Dp(1 ./ (h + 1.05)) ./ (h + 1.05).^2));
    hh = logspace(log10(1.0), log10(22.2), 1500)';
    f = figure('Position', [60 60 1250 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(3, 2);
    fprintf('\n%-6s %-5s | %9s %8s %8s | %10s %6s\n', 'plant', 'h_b', 'c_hat_end', 'e_a_end', 's^c(h_b)', 'min h_true', 'floor');
    for ip = 1:2
        pl = PL{ip};  hl = gobjects(1, 3);  ok = false(1, 3);
        for rr = 1:3; ax(rr, ip) = nexttile(2 * (rr - 1) + ip); hold(ax(rr, ip), 'on'); box(ax(rr, ip), 'on'); end
        for ib = 1:3
            fn = fullfile(rd, sprintf('f1p2_%s_PI_sig010_ramp2%s.mat', pl, HB{ib}));
            if ~exist(fn, 'file'); fprintf('%-6s %-5.2f | missing %s\n', pl, HBV(ib), fn); continue; end
            r = load(fn);  t = r.t;  desc = t >= r.seg(1) & t < r.seg(2);  ie = find(desc, 1, 'last');  id = find(desc);
            cm = mean(r.c_hat, 2);  em = mean(r.a_true - r.a_hat, 2);  h0 = r.wd(id(1));  sc = 1 / r.wd(ie) - 1 / h0;
            hmin = min(r.h_true(:));  nfloor = nnz(r.h_true(:) <= 1.001 + 1e-9);
            fprintf('%-6s %-5.2f | %9.3f %+8.3f %8.3f | %10.4f %6d\n', pl, HBV(ib), cm(ie), em(ie), sc, hmin, nfloor);
            ok(ib) = true;
            for rr = 1:3
                if rr == 3; yv = em; else; yv = cm; end
                hl(ib) = plot(ax(rr, ip), r.wd(id), yv(id), '-', 'Color', COLS(ib, :), 'LineWidth', 1.8, 'DisplayName', sprintf('h_b = %.2f R', HBV(ib)));
                plot(ax(rr, ip), r.wd(ie), yv(ie), 'o', 'Color', COLS(ib, :), 'MarkerFaceColor', COLS(ib, :), 'MarkerSize', 7, 'HandleVisibility', 'off');
            end
        end
        for rr = 1:3
            a = ax(rr, ip);
            hc = gobjects(0);
            if rr < 3; hc = plot(a, hh, ceff.(pl)(hh), '-', 'Color', COL_TRUE, 'LineWidth', 1.6, 'DisplayName', 'c_{eff}(h) of this wall'); end
            set(a, 'XScale', 'log', 'XDir', 'reverse');
            if rr == 1; xlim(a, [1.0 25]); ylim(a, [-0.2 0.9]); else; xlim(a, [1.0 2.0]); set(a, 'XTick', [1.0 1.1 1.2 1.5 2.0]); end
            if rr == 2; ylim(a, [0 0.9]); end
            if rr == 3; ylim(a, [-0.05 0.2]); yline(a, 0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off'); end
            xline(a, 1.1, '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
            if ip == 1
                if rr == 3; ylabel(a, 'e_a  [a / a_{nom}]', 'FontSize', FS, 'FontWeight', 'bold'); else; ylabel(a, 'c', 'FontSize', FS, 'FontWeight', 'bold'); end
            end
            if rr == 3; xlabel(a, 'h_d  (R)', 'FontSize', FS, 'FontWeight', 'bold'); end
            lab = sprintf('%s wall, plane init, c_{hat} (10-seed mean)', upper(pl));
            if rr == 3; lab = sprintf('%s wall, plane init, e_a = a_{true} - a_{hat}', upper(pl)); end
            h0l = plot(a, NaN, NaN, 'w.', 'DisplayName', lab);
            L = [h0l hl(ok) hc];
            legend(a, L, 'Location', 'northoutside', 'NumColumns', 3, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
            set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid(a, 'off');
        end
    end
    fn = fullfile(od, 'c_bottom_zoom.png');  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
