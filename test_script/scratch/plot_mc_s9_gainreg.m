% STATUS: SCRATCH -- 2026-10-13, motion_ctrl Section 9 figure: gain regulation, one wall, seed 1, the simplest rows (user 10-13).
%   Rows: z_d (red) and z (blue dashed) [um]; a-bar_z true (red), estimate (blue), a_d (black dashed); w_s-hat (blue), wall position 0 (red);
%   delta z_m [um]. Data: run_mc_s9_gainreg.m (test_results/temp_1009_ws/gainreg_<plant><sfx>.mat).
%   10-14: sfx selects the run (e.g. '_tau100_T6'); w_s row adds black dashed = w_s that makes 1/(h - w_s)^2 equal the plant slope at the
%   true height (as in Fig 8.1), so a wall that is not a plane shows what w_s-hat should read.
function plot_mc_s9_gainreg(plant, sfx)
    if nargin < 1; plant = 'plane'; end
    if nargin < 2; sfx = ''; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    addpath(genpath(fullfile(root, 'model')));  S = load(fullfile(od, sprintf('gainreg_%s%s.mat', plant, sfx)));  t = S.t;  q = 1;  wd = S.wd(:, q);  wd(1) = wd(2);
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 22;  LFS = 17;  R_UM = 2.25;  AB = char(257);  HAT = char(770);  WB = ['w' char(772)];
    f = figure('Position', [40 40 1100 2000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    hasp = isfield(S, 'p_hat') && any(S.p_hat(:, q) > 0);  nr = 4 + hasp;   % 10-15: p-hat row when the exponent is estimated
    if hasp; set(f, 'Position', [40 40 1100 2400]); end
    tl = tiledlayout(nr, 1, 'TileSpacing', 'compact', 'Padding', 'compact');  ax = gobjects(nr, 1);
    ax(1) = nexttile(tl);  hold on; box on;
    h1 = plot(ax(1), t, wd * R_UM, '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', 'z_d');
    h2 = plot(ax(1), t, S.h_true(:, q) * R_UM, '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'z');
    set(ax(1), 'YScale', 'log');  ylim(ax(1), [1 60]);  yticks(ax(1), [1 2.5 10 50]);  legend(ax(1), [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    ax(2) = nexttile(tl);  hold on; box on;
    h3 = plot(ax(2), t, S.a_true(:, q), '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', [AB '_z']);
    h4 = plot(ax(2), t, S.a_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [AB HAT '_z']);
    h5 = yline(ax(2), S.a_d, '--', 'Color', 'k', 'LineWidth', 2, 'DisplayName', [AB '_d']);
    ylim(ax(2), [-0.2 1.2]);  legend(ax(2), [h3 h4 h5], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    ax(3) = nexttile(tl);  hold on; box on;
    h6 = plot(ax(3), t, zeros(size(t)), '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', [WB '_s']);
    h7 = plot(ax(3), t, S.w_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [WB HAT '_s']);
    hb = max(S.h_true(:, q), 1.001);  wsq = hb - 1 ./ sqrt(local_ap(hb, plant));
    h8 = plot(ax(3), t, wsq, '--', 'Color', 'k', 'LineWidth', 1.8, 'DisplayName', [WB '_s that fits the slope']);
    ylim(ax(3), [-1.5 1]);  legend(ax(3), [h6 h7 h8], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    if hasp
        ax(5) = nexttile(tl, 4);  hold on; box on;
        plot(ax(5), t, S.p_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', ['p' HAT]);
        ylim(ax(5), [1.5 2.5]);  ylabel(ax(5), ['p' HAT], 'FontSize', FS, 'FontWeight', 'bold');
        legend(ax(5), 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(4) = nexttile(tl, 5);  hold on; box on;
    else
        ax(4) = nexttile(tl);  hold on; box on;
    end
    plot(ax(4), t, S.dxm(:, q), '-', 'Color', cE, 'LineWidth', 1.0);
    ylim(ax(4), [-0.2 0.2]);  yticks(ax(4), [-0.1 0 0.1]);  xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax(1), ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(2), [AB '_z'], 'FontSize', FS + 2, 'FontWeight', 'bold');
    ylabel(ax(3), [WB '_s  [R]'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax(4), [char(948) 'z_m  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
    for k = 1:nr; xlim(ax(k), [0 t(end)]);  set(ax(k), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2.2);  grid(ax(k), 'off'); end
    out = fullfile(fd, sprintf('fig_mc_s9_%s%s_s1.png', plant, sfx));  exportgraphics(f, out, 'Resolution', 130);  fprintf('figure: %s\n', out);
end

function ap = local_ap(h, plant)
    d = 1e-4;  ap = (local_a(h + d, plant) - local_a(h - d, plant)) / (2 * d);
end

function a = local_a(h, plant)
    if strcmp(plant, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
