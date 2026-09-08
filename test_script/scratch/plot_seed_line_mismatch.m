% PURPOSE (2026-09-08): the seed-line picture. The estimator is seeded with (a_hat[0], b_hat[0]); the exact law step
%   u+ = u + b_hat M (u = 1/(1-a_hat)) then carries u along the descent, so the carried level at the bottom is the straight LINE
%   through (w_0, u_0) with slope b_hat[0]. Yesterday's seed-at-truth (b_hat[0] = 8/9, a_hat[0] = truth) puts that line's
%   zero-gain height (u = 1) at 1.106 R, ABOVE the canon bottom 1.10 R; today's chord seed (b_hat[0] from start gain + contact)
%   puts it at 1.00 R. Panels: (a) u vs w over the whole descent, (b) zoom near the wall, (c) a_hat near the first bottom
%   (10-seed mean, both arms, truth red), (d) b_hat over the run (both arms) with b_true(w).
%   Data: three_walls_canon_plane_rep_bhp0-...mat (arm rep_bhp0 = yesterday's bhp0 rebuilt) and ..._bseed0-btseed0.mat.
%   Output seed_line_mismatch_canon.png | EXPIRES: with the unknown-wall line | 產線改動不會自動跟上
function plot_seed_line_mismatch()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model'))); od = fullfile(root, 'test_results', 'apd_acov_meng');
    A = load(fullfile(od, 'three_walls_canon_plane_rep_bhp0-rep_bhp0_c15-bseed0_c105-mix_b89_wc-mix_bch_w99.mat'), 'plane_rep_bhp0', 'phases');
    B = load(fullfile(od, 'three_walls_canon_plane-sphere-cell_bseed0-btseed0.mat'), 'plane_bseed0', 'plane_btseed0');
    Y = A.plane_rep_bhp0; T = B.plane_bseed0; BT = B.plane_btseed0;
    w0bar = 22.222; [~, cp0] = calc_correction_functions(w0bar, true); a0 = 1/cp0; u0 = 1/(1 - a0);
    bY = 8/9; bT = (u0 - 1)/(w0bar - 1.0);
    w = linspace(1.0, 22.5, 2000); ut = zeros(size(w)); for i = 1:numel(w); [~, c] = calc_correction_functions(w(i), true); ut(i) = 1/(1 - 1/c); end
    uY = u0 - bY*(w0bar - w); uT = u0 - bT*(w0bar - w);
    COL_TRUE = [0.8 0 0]; COL_Y = [0 0.2 0.9]; COL_T = [0 0.55 0.2]; FS = 14; LFS = 11; AXLW = 1.6;
    f = figure('Units','inches','Position',[0 0 14 5.6], 'Color','w', 'Visible','off'); tiledlayout(1, 2, 'TileSpacing','compact', 'Padding','compact');
    % (1) gain vs height near the wall: truth and the two seed lines (the line = what the law carries down from the start)
    nexttile; hold on; wz = w(w <= 3.0);
    aT = 1 - 1./ut(w <= 3.0); aY = max(1 - 1./uY(w <= 3.0), 0); aG = 1 - 1./uT(w <= 3.0);
    h1 = plot(wz, aT, '-', 'Color', COL_TRUE, 'LineWidth', 2.4);
    h2 = plot(wz, aY, '--', 'Color', COL_Y, 'LineWidth', 2.0);
    h3 = plot(wz, aG, '-', 'Color', COL_T, 'LineWidth', 2.0);
    h5 = xline(1.10, '-.', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.4);
    legend([h1 h2 h3 h5], {'a_z/a_{nom}  true', 'seed line, b[0] = 8/9   (zero at 1.106 R)', sprintf('seed line, b[0] = %.3f   (zero at 1.0 R)', bT), 'trajectory bottom 1.10 R'}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on','NumColumns',2);
    xlim([1.0 3.0]); ylim([0 0.7]); xlabel('w/R', 'FontSize', FS, 'FontWeight','bold'); ylabel('a_z / a_{nom}', 'FontSize', FS, 'FontWeight','bold');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off;
    % (2) what the estimator did on the first descent (10-seed mean)
    nexttile; hold on; t = Y.t(:);
    h1 = plot(t, mean(Y.AT,2), '-', 'Color', COL_TRUE, 'LineWidth', 2.4);
    h2 = plot(t, mean(Y.AH,2), '--', 'Color', COL_Y, 'LineWidth', 2.0);
    h3 = plot(t, mean(T.AH,2), '-', 'Color', COL_T, 'LineWidth', 2.0);
    legend([h1 h2 h3], {'a_z/a_{nom}  true', '\^a_z/a_{nom}   seeded b[0] = 8/9', sprintf('\\^a_z/a_{nom}   seeded b[0] = %.3f', bT)}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on','NumColumns',2);
    xlim([1.2 1.7]); ylim([0 0.75]); xlabel('time [s]   (first descent, 10-seed mean)', 'FontSize', FS, 'FontWeight','bold'); ylabel('a_z / a_{nom}', 'FontSize', FS, 'FontWeight','bold');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off;
    png = fullfile(od, 'seed_line_mismatch_canon.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
    fprintf('u0 %.3f at w0 %.3f | line 8/9: u(1.10) = %.3f -> a = %.3f | line %.4f: u(1.10) = %.3f -> a = %.3f | true a(1.10) = %.3f\n', u0, w0bar, u0 - bY*(w0bar-1.10), 1 - 1/(u0 - bY*(w0bar-1.10)), bT, u0 - bT*(w0bar-1.10), 1 - 1/(u0 - bT*(w0bar-1.10)), 1 - 1/ut(find(w >= 1.10, 1)));
end
