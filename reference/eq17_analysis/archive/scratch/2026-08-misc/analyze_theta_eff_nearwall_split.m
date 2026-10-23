% PURPOSE: 承 analyze_theta_eff_1param.m -- 擺幅是不是全部住在近牆段？
%          掃描下界 w_min，看 theta_eff 在 [w_min, 22] 上的擺幅怎麼縮
% SCOPE:   perpendicular axis only
%
% STATUS: ACTIVE -- does the theta_eff swing live entirely in the near-wall segment?
%   Sweeps the lower bound w_min. Follows analyze_theta_eff_1param.m.

clear; clc;

repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(fullfile(repo_root, 'model')));

w_bar = linspace(1.1, 22, 8000).';
a_true = zeros(size(w_bar));
for i = 1:numel(w_bar)
    [~, c_perp_i] = calc_correction_functions(w_bar(i));
    a_true(i) = 1 / c_perp_i;
end

g = w_bar - 1;
b_eff = g .* (1 - a_true) ./ a_true;
p_eff = -log(1 - a_true) ./ log(1 + g ./ (9/8));

w_min_list = [1.1 1.25 1.5 1.75 2.0 2.5 3.0 4.0 5.0];

fprintf('\n=== theta_eff 擺幅 vs 下界 w_min (上界固定 22) ===\n');
fprintf('%8s | %-28s | %-28s\n', 'w_min', 'Mobius p=1  (b)', 'FormB b=9/8  (p)');
fprintf('%8s | %8s %8s %9s | %8s %8s %9s\n', ...
        '', 'min', 'max', 'swing%', 'min', 'max', 'swing%');
for wm = w_min_list
    m = w_bar >= wm;
    bb = b_eff(m); pp = p_eff(m);
    fprintf('%8.2f | %8.4f %8.4f %8.2f%% | %8.4f %8.4f %8.2f%%\n', wm, ...
            min(bb), max(bb), 100*(max(bb)-min(bb))/mean(bb), ...
            min(pp), max(pp), 100*(max(pp)-min(pp))/mean(pp));
end

% b_eff 的峰值位置
[bmax, imax] = max(b_eff);
fprintf('\nb_eff 峰值 %.4f @ w_bar = %.3f  (超過遠場錨 9/8 = %.4f，差 +%.2f%%)\n', ...
        bmax, w_bar(imax), 9/8, 100*(bmax - 9/8)/(9/8));
i_cross = find(b_eff >= 9/8, 1, 'first');
fprintf('b_eff 穿越 9/8 於 w_bar = %.3f  ==> 上方單調族 (b in [1,9/8]) 只在 w_bar < %.2f 有效\n', ...
        w_bar(i_cross), w_bar(i_cross));
