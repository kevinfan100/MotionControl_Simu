% PURPOSE: 報告用的精簡形狀圖 -- 只留剛性平壁真值 + 頁 6/8 實驗換用的那條曲線
%          (b, p, w_s) = (0.1286, 1, 1)，即 gainlaw_fit_cell.m 第 (3b) 節
%          short-reach 族的 p = 1 成員
% SCOPE:   perpendicular axis；真值 = calc_correction_functions (SSOT)
% 構造:    要求 a_bar(G_TARGET) = A_TARGET 且原點留在表面，固定 p = 1
%              r = (1 - A)^(1/p),   b = r*G/(1 - r)
%          A = 0.70, G = 0.3 R, p = 1  ->  b = 0.1286
% 產物:    reference/eq17_analysis/derivation/figures/formB_report_two_curves.png
%          (被 formB_report_prior_belief.tex 引用，故放 derivation/figures 而非 temp_)
% STYLE:   抄自 gainlaw_fit_cell.m 第 (3b) 節與 plot_var_ahat_6state.m (canonical)
%
% STATUS: ACTIVE -- figure generator for formB_report_prior_belief.tex (two-curve shape
%   page). PERMANENT ARTIFACT DEPENDENCY: do not delete while that tex lives.

clear; clc;

root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root, 'model', 'wall_effect'));

% ---- 構造那條換用的曲線（與 gainlaw_fit_cell.m 完全同式）----------------
A_TARGET = 0.70;      % 這個 gap 上增益已經到 0.70
G_TARGET = 0.3;       % [R]
P_SUB    = 1;         % 遠場衰減指數，維持平壁值
r_sub    = (1 - A_TARGET)^(1 / P_SUB);
B_SUB    = r_sub * G_TARGET / (1 - r_sub);

B_PLANE  = 9/8;       % 平壁錨

% ---- 網格與真值（剛性平壁）---------------------------------------------
g      = linspace(1e-3, 9, 4000).';         % gap = w_bar - w_s，w_s = 1
a_true = zeros(size(g));
for i = 1:numel(g)
    [~, c_perp_i] = calc_correction_functions(1 + g(i));
    a_true(i) = 1 / c_perp_i;
end
a_sub = 1 - (B_SUB ./ (g + B_SUB)).^P_SUB;

% ---- console（數字不上圖）----------------------------------------------
fprintf('\n=== 報告用兩條曲線 ===\n');
fprintf('  剛性平壁真值   Brenner 級數，等效 b 錨 = %.4f (9/8)\n', B_PLANE);
fprintf('  換用的曲線     b = %.4f, p = %.1f, w_s = 1\n', B_SUB, P_SUB);
fprintf('  兩者 b 相差    %.2f 倍   (距離 |9/8 - b| = %.4f)\n', ...
        B_PLANE / B_SUB, abs(B_PLANE - B_SUB));
fprintf('  以推導出的 P initial 寬度 0.0157 量：%.1f 個寬度\n', ...
        abs(B_PLANE - B_SUB) / 0.0157);
fprintf('  兩條曲線在 gap %.1f R 的增益：平壁 %.3f / 換用 %.3f\n', ...
        G_TARGET, interp1(g, a_true, G_TARGET), A_TARGET);

% ---- 圖（專案風格；顏色沿用原第 2 頁，方便交叉對照）--------------------
COL_TRUE = [0.8 0 0];
COL_SUB  = [0.10 0.45 0.10];
FS = 18; LFS = 14; AXLW = 2.0;

fig = figure('Position', [80 80 1050 720], 'Color', 'w', ...
             'NumberTitle', 'off', 'Visible', 'off');
hold on;
h1 = plot(g, a_true, '-', 'Color', COL_TRUE, 'LineWidth', 3.0, ...
          'DisplayName', 'rigid plane (anchored b = 9/8)');
h2 = plot(g, a_sub, '-.', 'Color', COL_SUB, 'LineWidth', 2.8, ...
          'DisplayName', sprintf('substituted truth (b = %.2f, p = 1)', B_SUB));
xlim([0 9]); ylim([0 1]);
xlabel('$\bar{w}-\bar{w}_s$   [radii]', 'Interpreter', 'latex', ...
       'FontSize', FS, 'FontWeight', 'bold');
ylabel('motion gain   a / a_o', 'FontSize', FS, 'FontWeight', 'bold');
legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');
grid off;

out = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures', ...
               'formB_report_two_curves.png');
exportgraphics(fig, out, 'Resolution', 150);
close(fig);
fprintf('\n圖 -> %s\n', out);
