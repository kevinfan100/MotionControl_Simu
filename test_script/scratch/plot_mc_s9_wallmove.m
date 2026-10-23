% STATUS: SCRATCH -- 2026-10-15 (user): Fig 9.7, the estimator starts at w_s-hat[0] = 0 while the true wall is moved by DELTA R (plant side
%   only, run_formC_b ws_inject; DELTA > 0 = closer than assumed). Plane, a_d 0.07, 6 s, seed 1 (probe_mc_s9_wallmove files _mv<DELTA>).
%   Columns DELTA = -2 | -1 | +2. Rows: true gap h (blue; red = commanded, shifted to the true wall; black dashed h* = 1.078 R where the
%   plant gain equals a_d); a-bar_z true (red), estimate (blue), a_d (black dashed); w_s-hat (blue) with the true wall position DELTA (red).
function plot_mc_s9_wallmove(deltas, plant)
    if nargin < 1 || isempty(deltas); deltas = [-2 -1 2]; end
    if nargin < 2; plant = 'plane'; end   % 10-15: 'cell' (a_d 0.52, h* 1.174 R, true wall at DELTA - 1.05 in the estimator frame; files _mv<DELTA>_ad520)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 18;  LFS = 14;  AB = char(257);  HAT = char(770);  WB = ['w' char(772)];  hs = 1.0781;  w0 = 0;  sfx = '';  yl = [0 0.3];
    if strcmp(plant, 'cell'); hs = 1.1741;  w0 = -1.05;  sfx = '_ad520';  yl = [0.42 0.82]; end
    nc = numel(deltas);  f = figure('Position', [40 40 700 * nc 1450], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
    for j = 1:nc
        DE = deltas(j);  tg = strrep(strrep(sprintf('_mv%+g', DE), '+', 'p'), '-', 'm');
        S = load(fullfile(od, sprintf('gainreg_%s_tau100_T6%s%s.mat', plant, tg, sfx)));  t = S.t;  q = 1;  wd = S.wd(:, q);  wd(1) = wd(2);
        a1 = nexttile(tl, j);  hold on; box on;
        plot(a1, t, wd - DE, '-', 'Color', cT, 'LineWidth', 3, 'DisplayName', 'h_d (true frame)');
        plot(a1, t, S.h_true(:, q), '--', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', 'h');
        yline(a1, hs, '--', 'Color', 'k', 'LineWidth', 1.8, 'DisplayName', 'h^* (gain = a_d)');  ylim(a1, [0.95 2.5]);
        text(a1, 0.97, 0.88, sprintf('\\Delta = %+g R', DE), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', LFS + 4, 'FontWeight', 'bold');
        a2 = nexttile(tl, nc + j);  hold on; box on;
        plot(a2, t, S.a_true(:, q), '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', [AB '_z']);  plot(a2, t, S.a_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [AB HAT '_z']);
        yline(a2, S.a_d, '--', 'Color', 'k', 'LineWidth', 2, 'DisplayName', [AB '_d']);  ylim(a2, yl);
        a3 = nexttile(tl, 2 * nc + j);  hold on; box on;
        if w0 ~= 0; yline(a3, DE, '-', 'Color', [0.55 0.55 0.55], 'LineWidth', 2, 'DisplayName', 'visible surface'); end   % 10-15 (user): unified wording
        yline(a3, DE + w0, '-', 'Color', cT, 'LineWidth', 3.5, 'DisplayName', ['equivalent wall ' WB '_s']);  plot(a3, t, S.w_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [WB HAT '_s']);
        ylim(a3, [-3.5 5.5]);  xlabel(a3, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if j == 1; ylabel(a1, 'true gap h  [R]', 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a2, [AB '_z'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a3, [WB '_s  [R]'], 'FontSize', FS, 'FontWeight', 'bold'); end
        for a = [a1 a2 a3]; xlim(a, [0 t(end)]);  legend(a, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
            set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(a, 'off'); end
    end
    out = fullfile(fd, sprintf('fig_mc_s9_wallmove%s_s1.png', strrep(['_' plant], '_plane', '')));  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end
