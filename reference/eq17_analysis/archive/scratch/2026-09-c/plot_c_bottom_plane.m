% PURPOSE (2026-09-28, user: "show me the plane case"): plane wall only, c-only law (f1p2), plane init, prior 0.10, lambda = 1,
%   T_d = 2 s, 10 seeds, bottoms 1.10 / 1.05 / 1.01 R (runs f1p2_plane_PI_sig010_ramp2{,_hb105,_hb101}.mat). Zoom 2.0 -> 1.0 R:
%   row 1 a_true (red) and a_hat per bottom, with the estimator's gain floor 0.05; row 2 e_a; row 3 c_hat with c_eff(h).
%   Descent-end points marked. House style. Output reference/eq17_analysis/derivation/figures/c_bottom_plane.png
%   EXPIRES: with the c-only line
function plot_c_bottom_plane()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    HB = {'', '_hb105', '_hb101'};  HBV = [1.10 1.05 1.01];
    COLS = [0 0.2 0.9; 0.85 0.45 0; 0 0.6 0.2];  COL_TRUE = [0.8 0 0]; FS = 15; LFS = 11; AXLW = 2.0; CG = [0.5 0.5 0.5];
    Dp = @(u) -9/8 + (3/2)*u.^2 - 2.28*u.^3 + u.^4 + 0.385*u.^10 - 0.48*u.^11;
    ceff = @(h) 1 - h.^2 .* (-Dp(1 ./ h) ./ h.^2);  hh = logspace(log10(1.0), log10(2.0), 800)';
    f = figure('Position', [60 60 800 1050], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(3, 1);  for rr = 1:3; ax(rr) = nexttile(rr); hold(ax(rr), 'on'); box(ax(rr), 'on'); end
    hl = gobjects(3, 3);  ok = false(1, 3);  htrue = gobjects(0);
    fprintf('\n%-5s | %9s %9s %8s | %10s %10s\n', 'h_b', 'a_true_end', 'a_hat_end', 'e_a_end', 'min h_true', 'a_hat<=0.05');
    for ib = 1:3
        fn = fullfile(rd, sprintf('f1p2_plane_PI_sig010_ramp2%s.mat', HB{ib}));
        if ~exist(fn, 'file'); fprintf('%-5.2f | missing\n', HBV(ib)); continue; end
        r = load(fn);  t = r.t;  desc = t >= r.seg(1) & t < r.seg(2);  id = find(desc);  ie = id(end);
        am = mean(r.a_hat, 2);  at = mean(r.a_true, 2);  em = at - am;  cm = mean(r.c_hat, 2);
        fprintf('%-5.2f | %9.3f %9.3f %+8.3f | %10.4f %10d\n', HBV(ib), at(ie), am(ie), em(ie), min(r.h_true(:)), nnz(r.a_hat(desc, :) <= 0.05 + 1e-9));
        ok(ib) = true;
        if ib == 1; htrue = plot(ax(1), r.wd(id), at(id), '-', 'Color', COL_TRUE, 'LineWidth', 2.2, 'DisplayName', 'a_{true} / a_{nom}'); end
        Y = {am, em, cm};
        for rr = 1:3
            hl(rr, ib) = plot(ax(rr), r.wd(id), Y{rr}(id), '-', 'Color', COLS(ib, :), 'LineWidth', 1.8, 'DisplayName', sprintf('h_b = %.2f R', HBV(ib)));
            plot(ax(rr), r.wd(ie), Y{rr}(ie), 'o', 'Color', COLS(ib, :), 'MarkerFaceColor', COLS(ib, :), 'MarkerSize', 7, 'HandleVisibility', 'off');
        end
    end
    hfl = yline(ax(1), 0.05, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.6, 'DisplayName', 'estimator gain floor 0.05');
    hce = plot(ax(3), hh, ceff(hh), '-', 'Color', COL_TRUE, 'LineWidth', 1.6, 'DisplayName', 'c_{eff}(h) of the plane');
    yline(ax(2), 0, '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    YL = {[0 0.5], [-0.05 0.05], [-0.1 0.2]};  LAB = {'a / a_{nom}', 'e_a  [a / a_{nom}]', 'c'};
    TIT = {'PLANE wall, plane init, a_{hat} (10-seed mean)', 'PLANE wall, plane init, e_a = a_{true} - a_{hat}', 'PLANE wall, plane init, c_{hat} (10-seed mean)'};
    for rr = 1:3
        a = ax(rr);  set(a, 'XScale', 'log', 'XDir', 'reverse');  xlim(a, [1.0 2.0]);  set(a, 'XTick', [1.0 1.1 1.2 1.5 2.0]);  ylim(a, YL{rr});
        xline(a, 1.1, '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
        ylabel(a, LAB{rr}, 'FontSize', FS, 'FontWeight', 'bold');  if rr == 3; xlabel(a, 'h_d  (R)', 'FontSize', FS, 'FontWeight', 'bold'); end
        h0 = plot(a, NaN, NaN, 'w.', 'DisplayName', TIT{rr});
        L = [h0 hl(rr, ok)];  if rr == 1; L = [L htrue hfl]; end;  if rr == 3; L = [L hce]; end
        legend(a, L, 'Location', 'northoutside', 'NumColumns', 3, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid(a, 'off');
    end
    fn = fullfile(od, 'c_bottom_plane.png');  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
end
