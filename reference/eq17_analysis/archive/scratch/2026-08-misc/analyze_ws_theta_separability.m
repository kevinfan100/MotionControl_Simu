% PURPOSE: 單參數族的 (theta, w_s) 可分辨性 -- 兩條靈敏度向量的夾角
%          cos(phi) -> 1 = 共線 = KF 分不開；變異數放大 = 1/sqrt(1-cos^2)
% SCOPE:   perpendicular axis；靈敏度在推導錨 (b=9/8, p=1) 上求值
% NOTE:    均勻取樣 = 「軌跡在每個高度停留時間相同」的代理。真軌跡要換成實際 w_bar 序列
% 承接:    analyze_theta_eff_1param.m（第一關：形狀能力）；本檔是第二關
%
% STATUS: ACTIVE -- (theta, w_s) separability: angle between the two sensitivity vectors.
%   Uniform sampling is a proxy for dwell time; the real trajectory version is
%   analyze_formB_fisher_2param.m. Second of the three-script chain.

clear; clc;

b0 = 9/8;   p0 = 1;   w_s = 1;

% 掃描的高度區間 (w_bar 下界, 上界)
ranges = { 'full  [1.1 , 22  ]', 1.10, 22.00; ...
           'envel [1.90, 23.2]', 1.90, 23.22; ...
           'desc  [2.0 , 5.0 ]', 2.00,  5.00; ...
           'near  [1.1 , 3.0 ]', 1.10,  3.00; ...
           'hold  [4.0 , 4.5 ]', 4.00,  4.50; ...
           'hold  [4.0 , 4.05]', 4.00,  4.05 };

fprintf('\n=== (theta, w_s) 可分辨性：兩條靈敏度向量的夾角 ===\n');
fprintf('錨值 b0 = %.4f, p0 = %.1f, w_s = %.1f\n\n', b0, p0, w_s);
fprintf('%-20s | %-26s | %-26s\n', 'range (w_bar)', ...
        '候選1  Mobius p=1  (b, ws)', '候選2  FormB b=9/8 (p, ws)');
fprintf('%-20s | %8s %8s %8s | %8s %8s %8s\n', '', ...
        '|cos|', 'infl', 'cond', '|cos|', 'infl', 'cond');

for r = 1:size(ranges, 1)
    w = linspace(ranges{r,2}, ranges{r,3}, 4000).';
    g = w - w_s;

    % --- 候選 1: a = g/(g+b),  theta = b ---
    d_b  = -g      ./ (g + b0).^2;        % da/db
    d_w1 = -b0     ./ (g + b0).^2;        % da/dws
    [c1, i1, k1] = sep_metrics(d_b, d_w1);

    % --- 候選 2: a = 1 - (b0/(g+b0))^p,  theta = p ---
    ratio = b0 ./ (g + b0);
    d_p  =  ratio.^p0 .* log(1 ./ ratio);            % da/dp
    d_w2 = -p0 * b0^p0 .* (g + b0).^(-p0 - 1);       % da/dws
    [c2, i2, k2] = sep_metrics(d_p, d_w2);

    fprintf('%-20s | %8.5f %8.2f %8.1f | %8.5f %8.2f %8.1f\n', ...
            ranges{r,1}, c1, i1, k1, c2, i2, k2);
end

fprintf(['\n讀法：|cos|->1 共線(分不開)；infl = 1/sqrt(1-cos^2) = 不獨立的變異數放大倍數；\n', ...
         '      cond = 2x2 Fisher 條件數。均勻取樣 = 每個高度停留時間相同的代理。\n']);

% ------------------------------------------------------------------
function [abscos, infl, kappa] = sep_metrics(s1, s2)
    s1 = s1(:); s2 = s2(:);
    abscos = abs(dot(s1, s2) / (norm(s1) * norm(s2)));
    infl   = 1 / sqrt(max(1 - abscos^2, eps));
    M      = [s1 s2].' * [s1 s2];
    kappa  = cond(M);
end
