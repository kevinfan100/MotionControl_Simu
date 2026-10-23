% PURPOSE: 單參數 gain-law 族的逐點參數曲線 theta_eff(w_bar - w_s)
%          固定 w_s = 1，在每個 gap 上反解「要讓模型等於真值，該參數必須是多少」
%          平坦 = 該族的參數真的是常數；擺幅 = 必須誠實承擔的 prior 寬度
% SCOPE:   perpendicular axis only (c_perp)。範圍 w_bar in [1.1, 22]
% TRUTH:   model/wall_effect/calc_correction_functions.m  (SSOT)
% STYLE:   抄自 test_script/integration/plot_var_ahat_6state.m (canonical)
%
% STATUS: ACTIVE -- pointwise theta_eff(gap) for the one-parameter families; the swing IS
%   the prior width that must be carried honestly. First of a three-script chain.

clear; clc;

repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(repo_root, 'model', 'wall_effect'));

% ------------------------------------------------------------------
% 真值：a_bar_true = 1 / c_perp
% ------------------------------------------------------------------
w_bar  = logspace(log10(1.1), log10(22), 6000).';
a_true = zeros(size(w_bar));
for i = 1:numel(w_bar)
    [~, c_perp_i] = calc_correction_functions(w_bar(i));
    a_true(i) = 1 / c_perp_i;
end

w_s = 1;
u   = w_bar - w_s;          % 橫軸 = w_bar - w_s (gap)
A   = a_true;

b_far  = 9/8;               % 遠場 reflections 錨
b_near = 1;                 % 近牆 lubrication 錨

% ---- 候選 1  a = (w-ws) / [ (w-ws) + b ] ----------------------------
b_eff = u .* (1 - A) ./ A;

% ---- 候選 2  a = 1 - [ 1 + (w-ws)/(9/8) ]^(-p) ---------------------
p_eff = -log(1 - A) ./ log(1 + u ./ b_far);

% ---- 候選 3  b(w) = 1 + (1/8)*(w-ws)/[ (w-ws) + s ] ----------------
u_cross = 8 * (b_eff - 1);
s_eff   = u .* (1 - u_cross) ./ u_cross;
s_valid = (u_cross > 0) & (u_cross < 1);

% ---- 候選 4  a = 1 - exp[ -(w-ws)/b ]  (陰性對照) -------------------
b_eff_exp = -u ./ log(1 - A);

% ------------------------------------------------------------------
% Console 統計（數字不上圖）
% ------------------------------------------------------------------
fprintf('\n=== theta_eff 逐點參數  (w-ws) in [%.2f, %.2f], w_s = %.1f ===\n', ...
        u(1), u(end), w_s);
fprintf('%-30s %9s %9s %9s %12s\n', 'candidate (parameter)', 'min', 'max', 'swing%', 'sup|d|/th0%');
rep = @(n, th, th0) fprintf('%-30s %9.4f %9.4f %8.2f%% %11.2f%%\n', n, ...
        min(th), max(th), 100*(max(th)-min(th))/mean(th), 100*max(abs(th-th0))/th0);
rep('1 Mobius p=1      (b)', b_eff,     b_far);
rep('2 FormB b=9/8     (p)', p_eff,     1);
rep('4 exponential     (b)', b_eff_exp, b_near);
fprintf('%-30s %9.4f %9.4f %8.2f%% %11s\n', '3 crossover       (s)', ...
        min(s_eff(s_valid)), max(s_eff(s_valid)), ...
        100*(max(s_eff(s_valid))-min(s_eff(s_valid)))/mean(s_eff(s_valid)), 'no anchor');
fprintf('   候選 3 有效域僅 %.1f%% 的範圍 (需 1 < b_eff < 9/8)\n', 100*mean(s_valid));

% ------------------------------------------------------------------
% 圖（專案風格：no grid / no title / box on / legend northoutside）
% ------------------------------------------------------------------
COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; COL_ALT = [0.55 0.55 0.55];
FS = 18; LFS = 12; AXLW = 2.0;

fg = figure('Position', [60 60 1400 1000], 'Color', 'w', ...
            'NumberTitle', 'off', 'Visible', 'off');
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% --- 1 ---
nexttile; hold on;
h1 = plot(u, b_eff, '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'b_{eff}');
h2 = yline(b_far,  '-',  'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'far 9/8');
h3 = yline(b_near, '--', 'Color', COL_ALT,  'LineWidth', 1.6, 'DisplayName', 'near 1');
set(gca, 'XScale', 'log', 'FontSize', FS, 'FontWeight', 'bold', ...
         'LineWidth', AXLW, 'Box', 'on'); grid off;
xlim([u(1) u(end)]);
ylabel('b_{eff}', 'FontSize', FS, 'FontWeight', 'bold');
legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');

% --- 2 ---
nexttile; hold on;
h1 = plot(u, p_eff, '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'p_{eff}');
h2 = yline(1,     '-',  'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'far 1');
h3 = yline(b_far, '--', 'Color', COL_ALT,  'LineWidth', 1.6, 'DisplayName', 'near 9/8');
set(gca, 'XScale', 'log', 'FontSize', FS, 'FontWeight', 'bold', ...
         'LineWidth', AXLW, 'Box', 'on'); grid off;
xlim([u(1) u(end)]);
ylabel('p_{eff}', 'FontSize', FS, 'FontWeight', 'bold');
legend([h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');

% --- 3 ---
nexttile; hold on;
h1 = plot(u(s_valid), s_eff(s_valid), '-', 'Color', COL_HAT, 'LineWidth', 2.0, ...
          'DisplayName', 's_{eff}');
set(gca, 'XScale', 'log', 'FontSize', FS, 'FontWeight', 'bold', ...
         'LineWidth', AXLW, 'Box', 'on'); grid off;
xlim([u(1) u(end)]);
xlabel('$\bar{w}-\bar{w}_s$', 'Interpreter', 'latex', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('s_{eff}', 'FontSize', FS, 'FontWeight', 'bold');
legend(h1, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');

% --- 4 ---
nexttile; hold on;
h1 = plot(u, b_eff_exp, '-', 'Color', COL_HAT, 'LineWidth', 2.0, 'DisplayName', 'b_{eff} (exp)');
h2 = yline(b_near, '-', 'Color', COL_TRUE, 'LineWidth', 2.0, 'DisplayName', 'near 1');
set(gca, 'XScale', 'log', 'FontSize', FS, 'FontWeight', 'bold', ...
         'LineWidth', AXLW, 'Box', 'on'); grid off;
xlim([u(1) u(end)]);
xlabel('$\bar{w}-\bar{w}_s$', 'Interpreter', 'latex', 'FontSize', FS, 'FontWeight', 'bold');
ylabel('b_{eff}', 'FontSize', FS, 'FontWeight', 'bold');
legend([h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', ...
       'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');

out_png = fullfile(repo_root, 'test_results', 'temp_theta_eff_1param.png');
if ~exist(fileparts(out_png), 'dir'); mkdir(fileparts(out_png)); end
exportgraphics(fg, out_png, 'Resolution', 150);
close(fg);
fprintf('\n圖 -> %s\n', out_png);
