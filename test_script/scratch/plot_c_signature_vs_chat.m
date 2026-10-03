% PURPOSE (2026-09-28, user: "I do not see the point of this analysis"): two panels on the same height axis, cell wall, c-only,
%   lambda = 1, T_d = 2 s (f1p2_cell_PI_sig010_ramp2.mat):
%     row 1  what the matrix says: the c signature in y2, s(h) = 1/h - 1/h0 (closed form, no data), flat in the two holds
%     row 2  what the estimator did: c_hat (10-seed mean) along the same heights, with c_eff(h) of the wall for reference
%   House style. Output reference/eq17_analysis/derivation/figures/c_signature_vs_chat.png   EXPIRES: with the c-only line
function plot_c_signature_vs_chat()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    A = load(fullfile(rd, 'f1p2_cell_PI_sig010_ramp2.mat'));
    t = A.t;  desc = t >= A.seg(1) & t < A.seg(2);  h0 = A.wd(find(desc, 1, 'first'));
    Dp = @(u) -9/8 + (3/2)*u.^2 - 2.28*u.^3 + u.^4 + 0.385*u.^10 - 0.48*u.^11;
    ap_ce = @(h) -Dp(1 ./ (h + 1.05)) ./ (h + 1.05).^2;  ceff = @(h) 1 - h.^2 .* ap_ce(h);
    s = @(h) 1 ./ h - 1 / h0;
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; FS = 15; LFS = 11; AXLW = 2.0; CG = [0.5 0.5 0.5];
    f = figure('Position', [60 60 900 760], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    hh = logspace(log10(1.10), log10(h0), 1000)';
    ax1 = nexttile; hold on; box on;
    h1 = plot(hh, s(hh), '-', 'Color', COL_HAT, 'LineWidth', 2.2, 'DisplayName', 's(h) = 1/h - 1/h_0   (y_2 entry of the c column, no data)');
    xline(3, '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    set(gca, 'XScale', 'log', 'XDir', 'reverse');  xlim([1.0 25]);  ylim([0 1]);
    legend(h1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel('s', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    ax2 = nexttile; hold on; box on;
    h2 = plot(hh, ceff(hh), '-', 'Color', COL_TRUE, 'LineWidth', 2.2, 'DisplayName', 'c_{eff}(h) of the cell wall');
    h3 = plot(A.wd(desc), mean(A.c_hat(desc, :), 2), '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'c_{hat}  (10-seed mean, \lambda = 1, T_d = 2 s)');
    xline(3, '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    set(gca, 'XScale', 'log', 'XDir', 'reverse');  xlim([1.0 25]);  ylim([-0.1 0.8]);
    legend([h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel('c', 'FontSize', FS, 'FontWeight', 'bold');  xlabel('h_d  (R)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    linkaxes([ax1 ax2], 'x');
    fn = fullfile(od, 'c_signature_vs_chat.png');  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
    id = find(desc);
    for hq = [22.2 10 5 3 2 1.5 1.1]
        [~, j] = min(abs(A.wd(id) - hq));  k = id(j);
        fprintf('h_d %5.1f : s %5.3f | c_eff %+.3f | c_hat %+.3f\n', A.wd(k), s(A.wd(k)), ceff(A.wd(k)), mean(A.c_hat(k, :)));
    end
end
