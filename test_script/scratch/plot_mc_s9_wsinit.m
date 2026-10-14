% STATUS: SCRATCH -- 2026-10-14 (user): gain regulation, tau_a 0.1 s, large wrong estimator start w_s-hat[0] = -3 | 0 | +3 (plane, seed 1).
%   Rows: a-bar_z (log; red true, blue estimate, black dashed a_d), w_s-hat (red plant wall 0, blue estimate). Data: run_mc_s9_gainreg.m.
function plot_mc_s9_wsinit(sfx, plant)
    if nargin < 1; sfx = ''; end   % e.g. '_T6' for the 6 s runs
    if nargin < 2; plant = 'plane'; end   % 10-14: 'cell' too; w_s row adds black dashed = w_s that fits the plant slope at the true height
    addpath(genpath(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'model')));
    here = fileparts(mfilename('fullpath'));  od = fullfile(fileparts(fileparts(here)), 'test_results', 'temp_1009_ws');
    WT = {'_wm3', '', '_wp3'};  W0 = [-3 0 3];
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 18;  LFS = 15;  AB = char(257);  HAT = char(770);  WB = ['w' char(772)];
    f = figure('Position', [40 40 2100 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    fprintf('ws0 | min h_true | last 0.5 s: a - a_d | a_hat - a_d | w_s-hat at 0.1 / 0.5 / 1.5 / 3 / end\n');
    for iw = 1:3
        S = load(fullfile(od, sprintf('gainreg_%s_tau100%s%s.mat', plant, WT{iw}, sfx)));  t = S.t;  L = t > t(end) - 0.5;
        wq = arrayfun(@(tq) S.w_hat(find(t >= min(tq, t(end)), 1)), [0.1 0.5 1.5 3]);
        fprintf('%+d | %.4f | %+.4f | %+.4f | %+.3f %+.3f %+.3f %+.3f %+.3f\n', W0(iw), min(S.h_true), mean(S.a_true(L) - S.a_d), mean(S.a_hat(L) - S.a_d), wq, S.w_hat(end));
        a1 = nexttile(tl, iw);  hold on; box on;
        plot(a1, t, S.a_true, '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_z']);
        plot(a1, t, S.a_hat, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [AB HAT '_z']);
        yline(a1, S.a_d, '--', 'Color', 'k', 'LineWidth', 2, 'DisplayName', [AB '_d']);
        set(a1, 'YScale', 'log');  if strcmp(plant, 'cell'); ylim(a1, [0.3 1]);  yticks(a1, [0.3 0.4 0.5 0.7 1]); else; ylim(a1, [0.01 1]);  yticks(a1, [0.01 0.02 0.05 0.1 0.2 0.5 1]); end
        text(a1, 0.97, 0.88, sprintf('%s%s_s[0] = %+d', WB, HAT, W0(iw)), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', LFS + 2, 'FontWeight', 'bold');
        legend(a1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        a2 = nexttile(tl, 3 + iw);  hold on; box on;
        plot(a2, t, zeros(size(t)), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [WB '_s']);
        plot(a2, t, S.w_hat, '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [WB HAT '_s']);
        hb = max(S.h_true, 1.001);  plot(a2, t, hb - 1 ./ sqrt(local_ap(hb, plant)), '--', 'Color', 'k', 'LineWidth', 1.6, 'DisplayName', [WB '_s that fits the slope']);
        ylim(a2, [-3.5 3.5]);  xlabel(a2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        legend(a2, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        if iw == 1; ylabel(a1, [AB '_z'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(a2, [WB '_s  [R]'], 'FontSize', FS, 'FontWeight', 'bold'); end
        for a = [a1 a2]; xlim(a, [0 t(end)]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(a, 'off'); end
    end
    out = fullfile(fileparts(fileparts(here)), 'reference', 'eq17_analysis', 'derivation', 'figures', sprintf('fig_mc_s9_wsinit%s%s_s1.png', strrep(['_' plant], '_plane', ''), sfx));  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end

function ap = local_ap(h, plant)
    d = 1e-4;  ap = (local_a(h + d, plant) - local_a(h - d, plant)) / (2 * d);
end

function a = local_a(h, plant)
    if strcmp(plant, 'cell'); a = 1 ./ cell_cperp_delta(h); return; end
    a = zeros(size(h));  for i = 1:numel(h); [~, c] = calc_correction_functions(h(i)); a(i) = 1 / c; end
end
