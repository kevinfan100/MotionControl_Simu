% STATUS: SCRATCH -- 2026-10-14 (user-approved test of formC_b fe_cmd_eval): base vs P-predict Jacobian at the commanded point, plane.
%   Row 1: fixed hold at 1.105 R (no regulation, w_s-hat free, prior 0.3), 3-seed mean a_true - a_hat (probe_mc_s9_holdbias 'base' / 'fc').
%   Row 2: gain regulation a_d = 0.1 (run_mc_s9_gainreg _ad100_base / _ad100_fc), 3-seed mean a-bar_z, black dashed a_d.
%   Black = base, blue = fe_cmd_eval. Output test_results/temp_1009_ws/fig_mc_fecmd_compare.png.
function plot_mc_fecmd_compare()
    here = fileparts(mfilename('fullpath'));  od = fullfile(fileparts(fileparts(here)), 'test_results', 'temp_1009_ws');
    cB = [0.15 0.15 0.15];  cF = [0 0.25 0.85];  FS = 20;  LFS = 16;  AB = char(257);  HAT = char(770);
    f = figure('Position', [40 40 1500 1100], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    a1 = nexttile(tl);  hold on; box on;
    B = load(fullfile(od, 'holdbias_base.mat'));  F = load(fullfile(od, 'holdbias_fc.mat'));
    plot(a1, B.t, movmean(mean(B.e, 2), 400), '-', 'Color', cB, 'LineWidth', 2.5, 'DisplayName', 'base');
    plot(a1, F.t, movmean(mean(F.e, 2), 400), '-', 'Color', cF, 'LineWidth', 2.5, 'DisplayName', 'F_e at commanded point');
    yline(a1, 0, '-', 'Color', [0.85 0 0], 'LineWidth', 1.5, 'HandleVisibility', 'off');
    xlim(a1, [1.5 20]);  ylim(a1, [-0.035 0.02]);  ylabel(a1, [AB '_z - ' AB HAT '_z'], 'FontSize', FS, 'FontWeight', 'bold');
    text(a1, 0.02, 0.1, 'fixed hold at 1.105 R, no regulation', 'Units', 'normalized', 'FontSize', LFS, 'FontWeight', 'bold');
    legend(a1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    a2 = nexttile(tl);  hold on; box on;
    B = load(fullfile(od, 'gainreg_plane_tau100_T6_ad100_base.mat'));  F = load(fullfile(od, 'gainreg_plane_tau100_T6_ad100_fc.mat'));
    plot(a2, B.t, movmean(mean(B.a_true, 2), 200), '-', 'Color', cB, 'LineWidth', 2.5, 'DisplayName', [AB '_z base']);
    plot(a2, F.t, movmean(mean(F.a_true, 2), 200), '-', 'Color', cF, 'LineWidth', 2.5, 'DisplayName', [AB '_z  F_e at commanded point']);
    yline(a2, B.a_d, '--', 'Color', 'k', 'LineWidth', 2, 'DisplayName', [AB '_d']);
    xlim(a2, [0.5 6]);  ylim(a2, [0.085 0.115]);  ylabel(a2, [AB '_z'], 'FontSize', FS, 'FontWeight', 'bold');  xlabel(a2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    text(a2, 0.02, 0.1, ['gain regulation ' AB '_d = 0.1'], 'Units', 'normalized', 'FontSize', LFS, 'FontWeight', 'bold');
    legend(a2, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    for a = [a1 a2]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(a, 'off'); end
    out = fullfile(od, 'fig_mc_fecmd_compare.png');  exportgraphics(f, out, 'Resolution', 130);  fprintf('figure: %s\n', out);
end
