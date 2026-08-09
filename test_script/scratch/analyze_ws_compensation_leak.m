% PURPOSE: 釘死一個形狀參數後，被丟掉的形狀自由度會不會由 w_s 代償？
%          做法：在不同的高度視窗上擬合，看 ws_hat 會不會跟著視窗跑。
%          牆沒動，ws_hat 卻移動 = w_s 在補形狀誤差，不是在報牆位
%          ⇒ 「w_s 近乎常數」這個先驗在該架構下不成立
% SCOPE:   perpendicular axis, 真值 = Brenner (calc_correction_functions)
% 判準:    spread(ws_hat) 跨視窗 = 可偵測的真實牆漂移下限
%
% STATUS: ACTIVE -- does w_s absorb the shape error once a shape constant is pinned?
%   Window-dependent ws_hat = compensation, not a wall reading. Open question,
%   same direction as defect 3; see the 08-09 to-do in MEMORY.md.

clear; clc;

repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(repo_root, 'model', 'wall_effect'));

R_um   = 2.25;          % 粒子半徑 [um] (b0 = 9/8 -> 2.53 um)
b0     = 9/8;  p0 = 1;  ws_true = 1;

windows = [2.00 22.00; 2.00 10.00; 2.00  5.00; 3.00 22.00; ...
           4.00 22.00; 1.50 22.00; 2.00  3.00; 2.50  8.00];

fprintf('\n=== 釘死一個參數後，w_s 有沒有在代償形狀誤差？ ===\n');
fprintf('真值 w_s = %.1f。ws_hat 偏離 0 = 代償；跨視窗移動 = w_s 不是常數\n\n', ws_true);
fprintf('%-14s | %-22s | %-22s | %-22s\n', 'window', ...
        'A: p=1 fix, est(b,ws)', 'B: b=9/8 fix, est(p,ws)', 'C: est(b,p,ws)  [3-par]');
fprintf('%-14s | %8s %6s %6s | %8s %6s %6s | %8s %6s %6s\n', '', ...
        'ws-1[R]', 'um', 'sup%', 'ws-1[R]', 'um', 'sup%', 'ws-1[R]', 'um', 'sup%');

wsA = []; wsB = []; wsC = [];
for r = 1:size(windows, 1)
    w = linspace(windows(r,1), windows(r,2), 1500).';
    A = zeros(size(w));
    for i = 1:numel(w)
        [~, cp] = calc_correction_functions(w(i));
        A(i) = 1 / cp;
    end

    mdl = @(g, b, p) 1 - (1 + g ./ b).^(-p);
    cost = @(b, p, ws) max(abs(mdl(w - ws, b, p) ./ A - 1));

    % --- A: p 釘 1，估 (b, ws) ---
    fA = @(x) cost(exp(x(1)), p0, x(2));
    xA = multistart(fA, [log(b0) 1.0; log(1.0) 0.9; log(1.3) 1.05]);
    bA = exp(xA(1)); sA = xA(2); eA = 100 * fA(xA);

    % --- B: b 釘 9/8，估 (p, ws) ---
    fB = @(x) cost(b0, exp(x(1)), x(2));
    xB = multistart(fB, [log(p0) 1.0; log(0.8) 0.9; log(1.2) 1.05]);
    pB = exp(xB(1)); sB = xB(2); eB = 100 * fB(xB);

    % --- C: 三參數（參照組）---
    fC = @(x) cost(exp(x(1)), exp(x(2)), x(3));
    xC = multistart(fC, [log(b0) log(p0) 1.0; log(1.0) log(0.95) 0.99]);
    sC = xC(3); eC = 100 * fC(xC);

    wsA(end+1) = sA;  wsB(end+1) = sB;  wsC(end+1) = sC;   %#ok<SAGROW>

    fprintf('[%5.2f %5.2f] | %+8.4f %+6.3f %6.2f | %+8.4f %+6.3f %6.2f | %+8.4f %+6.3f %6.2f\n', ...
            windows(r,1), windows(r,2), ...
            sA-ws_true, (sA-ws_true)*R_um, eA, ...
            sB-ws_true, (sB-ws_true)*R_um, eB, ...
            sC-ws_true, (sC-ws_true)*R_um, eC);
end

fprintf('\n--- 跨視窗散布（牆完全沒動，這就是 ws_hat 自己跑掉的量）---\n');
sm = @(n, v) fprintf('  %-24s  範圍 [%+.4f, %+.4f] R = [%+.3f, %+.3f] um   跨度 %.4f R = %.3f um\n', ...
        n, min(v)-1, max(v)-1, (min(v)-1)*R_um, (max(v)-1)*R_um, max(v)-min(v), (max(v)-min(v))*R_um);
sm('A  p=1 fix,  est(b,ws)', wsA);
sm('B  b=9/8 fix, est(p,ws)', wsB);
sm('C  3-par     est(b,p,ws)', wsC);

fprintf(['\n判讀：跨度就是「可偵測的真實牆漂移下限」。真牆漂移小於這個量，\n', ...
         '      與換視窗造成的代償無法區分 ⇒ 「w_s 近乎常數」的先驗在該架構下失效。\n']);

% ------------------------------------------------------------------
function xb = multistart(f, X0)
    opt = optimset('TolX', 1e-10, 'TolFun', 1e-12, 'MaxFunEvals', 2e4, 'MaxIter', 2e4);
    best = inf; xb = X0(1,:);
    for k = 1:size(X0, 1)
        x = fminsearch(f, X0(k,:), opt);
        x = fminsearch(f, x, opt);          % 再收一次（minimax 非光滑）
        v = f(x);
        if v < best; best = v; xb = x; end
    end
end
