% PURPOSE: 把兩個判準放同一張表 -- 形狀能力 vs 可分辨性，同一組 gap 範圍
%          形狀 = sup|theta_eff - theta0|/theta0（族裝不裝得下真值）
%          可分辨 = 兩條靈敏度向量夾角（KF 分不分得出 theta 與 w_s）
%          兩者由同一件事決定（走過的 gap 範圍）⇒ 必然對著幹
% SCOPE:   perpendicular axis；靈敏度在推導錨 (b=9/8, p=1) 上求值
% 承接:    analyze_theta_eff_1param.m（第一關）+ analyze_ws_theta_separability.m（第二關）
%
% STATUS: ACTIVE -- shape capability vs (theta, w_s) separability on one gap range; the two
%   criteria are set by the same thing and necessarily pull against each other.
%   Follows analyze_theta_eff_1param.m and analyze_ws_theta_separability.m.

clear; clc;

repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(repo_root, 'model', 'wall_effect'));

b0 = 9/8;  p0 = 1;  w_s = 1;

ranges = [2.00  2.10;    % 準 hold
          2.00  2.50;
          2.00  3.00;
          2.00  5.00;
          2.00 10.00;
          2.00 22.00;
          1.50 22.00;
          1.10 22.00];

fprintf('\n=== 形狀能力 vs 可分辨性（同一組 gap 範圍） ===\n');
fprintf('錨 b0 = %.4f, p0 = %.1f, w_s = %.1f。形狀 = sup|th_eff-th0|/th0；放大 = 1/sqrt(1-cos^2)\n\n', ...
        b0, p0, w_s);
fprintf('%-18s | %-19s | %-19s\n', 'w_bar range', ...
        '候選1 Mobius (b)', '候選2 FormB (p)');
fprintf('%-18s | %9s %9s | %9s %9s\n', '(gap span)', '形狀%', '放大x', '形狀%', '放大x');

for r = 1:size(ranges, 1)
    w  = linspace(ranges(r,1), ranges(r,2), 6000).';
    g  = w - w_s;
    A  = zeros(size(w));
    for i = 1:numel(w)
        [~, cp] = calc_correction_functions(w(i));
        A(i) = 1 / cp;
    end

    % ---- 形狀：逐點反解參數，對推導錨的最大偏離 ----
    b_eff = g .* (1 - A) ./ A;
    p_eff = -log(1 - A) ./ log(1 + g ./ b0);
    sh1 = 100 * max(abs(b_eff - b0)) / b0;
    sh2 = 100 * max(abs(p_eff - p0)) / p0;

    % ---- 可分辨：兩條靈敏度向量夾角 ----
    d_b  = -g  ./ (g + b0).^2;                    % da/db      (候選 1)
    d_w1 = -b0 ./ (g + b0).^2;                    % da/dws     (候選 1)
    rt   = b0 ./ (g + b0);
    d_p  =  rt.^p0 .* log(1 ./ rt);               % da/dp      (候選 2)
    d_w2 = -p0 * b0^p0 .* (g + b0).^(-p0 - 1);    % da/dws     (候選 2)
    in1 = infl(d_b, d_w1);
    in2 = infl(d_p, d_w2);

    fprintf('[%5.2f %5.2f] g=%4.1f | %8.2f%% %8.2f | %8.2f%% %8.2f\n', ...
            ranges(r,1), ranges(r,2), ranges(r,2)-ranges(r,1), sh1, in1, sh2, in2);
end

fprintf(['\n讀法：兩欄都是「越小越好」。跨度變大 ⇒ 放大倍數改善、形狀誤差惡化。\n', ...
         '      這不是可調的旋鈕，兩者由同一份資訊（走過的 gap 範圍）決定。\n']);

% ---- 單點（純 hold）的 (a, a'') 二方程檢查：det 恆不為零但條件極差 ----
fprintf('\n=== 單點 hold：(a_bar, a_bar'') 兩方程的 2x2 行列式 ===\n');
for w_hold = [2 3 5 10]
    g = w_hold - w_s;
    ap = p0 * b0^p0 * (g + b0)^(-p0 - 1);
    M1 = [ -g*(g+b0)^-2                    , -b0*(g+b0)^-2 ; ...
            ap*(p0/b0 - (p0+1)/(g+b0))     ,  ap*(p0+1)/(g+b0) ];
    fprintf('  w_bar = %5.1f : det = %+.4e  (非零 ⇒ 幾何上單點可分)\n', w_hold, det(M1));
end
fprintf(['  但 a_bar'' 的資訊要靠運動才進得來（H2 第 7 欄 ∝ 命令速度，hold 段恆 0）\n', ...
         '  ⇒ 幾何可分 ≠ 濾波器拿得到。\n']);

% ------------------------------------------------------------------
function v = infl(s1, s2)
    s1 = s1(:); s2 = s2(:);
    c  = abs(dot(s1, s2) / (norm(s1) * norm(s2)));
    v  = 1 / sqrt(max(1 - c^2, eps));
end
