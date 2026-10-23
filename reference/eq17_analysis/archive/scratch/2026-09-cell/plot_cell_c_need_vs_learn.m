% PURPOSE (2026-09-24, user: "c must change where c can be learned -- I don't follow"): two panels, cell wall, x = commanded
%   gap h_d along the T_d = 2 s descent (log, reversed):
%     row 1  the c the wall NEEDS at each height, c_eff(h) = 1 - h^2 a'_true(h) (red), against what the estimator DID:
%            c_hat (10-seed mean) for lambda = 1 (blue dashed) and for the information-driven beta = 3 (blue solid)
%     row 2  how much about c the data had taught by that height: sigma_c accumulated from the descent start (obs cache)
%   House style. Output reference/eq17_analysis/derivation/figures/cell_c_need_vs_learn.png   EXPIRES: with the c-only line
function plot_cell_c_need_vs_learn()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    rd = fullfile(root, 'test_results', 'temp_plaw_walls');  od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    Dp = @(u) -9/8 + (3/2)*u.^2 - 2.28*u.^3 + u.^4 + 0.385*u.^10 - 0.48*u.^11;
    ap_ce = @(h) -Dp(1 ./ (h + 1.05)) ./ (h + 1.05).^2;           % cell_cperp_delta: plane shifted 1.05 R below the visible surface
    h = logspace(log10(1.10), log10(22.2), 2000)';  ceff = 1 - h.^2 .* ap_ce(h);
    A = load(fullfile(rd, 'f1p2_cell_PI_sig010_ramp2.mat'));      % lambda = 1
    B = load(fullfile(rd, 'f1p2_cell_PI_sig010_ib3_ramp2.mat'));  % information-driven, beta = 3
    O = load(fullfile(rd, 'obs_ramp_f1p2_Td2.mat'));              % sigma_c accumulated from the descent start vs h_d (cell)
    desc = A.t >= A.seg(1) & A.t < A.seg(2);
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; FS = 15; LFS = 11; AXLW = 2.0; CG = [0.5 0.5 0.5];
    f = figure('Position', [60 60 900 820], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    % ---- row 1
    ax1 = nexttile; hold on; box on;
    h1 = plot(h, ceff, '-', 'Color', COL_TRUE, 'LineWidth', 2.4, 'DisplayName', 'c the wall needs at this height:  c_{eff}(h) = 1 - h^2 a''_{true}(h)');
    h2 = plot(A.wd(desc), mean(A.c_hat(desc, :), 2), '--', 'Color', COL_HAT, 'LineWidth', 1.8, 'DisplayName', 'c_{hat},  \lambda = 1');
    h3 = plot(B.wd(desc), mean(B.c_hat(desc, :), 2), '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'c_{hat},  information-driven \beta = 3');
    xline(3, '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    set(gca, 'XScale', 'log', 'XDir', 'reverse');  xlim([1.0 25]);  ylim([-0.1 0.8]);
    h0 = plot(NaN, NaN, 'w.', 'DisplayName', 'CELL wall, plane init, T_d = 2 s');
    legend([h0 h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'vertical', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel('c', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    % ---- row 2
    ax2 = nexttile; hold on; box on;
    h4 = plot(O.cell.HKd, O.cell.MGd(2, :), '-', 'Color', COL_HAT, 'LineWidth', 2.2, 'DisplayName', 'what the data had taught about c by this height:  \sigma_c accumulated from the descent start');
    xline(3, '--', 'Color', CG, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    set(gca, 'XScale', 'log', 'XDir', 'reverse');  xlim([1.0 25]);
    legend(h4, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    ylabel('\sigma_c', 'FontSize', FS, 'FontWeight', 'bold');  xlabel('commanded gap  h_d  (R)', 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on'); grid off;
    linkaxes([ax1 ax2], 'x');
    fn = fullfile(od, 'cell_c_need_vs_learn.png');  exportgraphics(f, fn, 'Resolution', 150); close(f); fprintf('figure: %s\n', fn);
    for hh = [22.2 10 5 3 2 1.1]
        fprintf('h_d = %5.1f R : c_eff %+.3f | c_hat lambda=1 %+.3f | c_hat beta=3 %+.3f | sigma_c %6.1f\n', hh, interp1(h, ceff, hh), ...
                interp1(A.wd(desc), mean(A.c_hat(desc, :), 2), hh), interp1(B.wd(desc), mean(B.c_hat(desc, :), 2), hh), interp1(O.cell.HKd, O.cell.MGd(2, :), hh));
    end
end
