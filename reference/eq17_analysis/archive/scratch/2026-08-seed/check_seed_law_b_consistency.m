% The seed and the law use DIFFERENT b, and this checks what that costs.
%
% controller:163  seed  a_bar_hat[0] = 1 - 1/(w_bar[0] - w0_hat)
%   That formula has NO b in it. It is the law's closed form with b = 1
%   exactly (b = 1 is the contact anchor: a_bar(1) = 0).
% controller:1057 law   a_bar' = b_hat * (1 - a_bar)^2,  b seeded at 8/9
%   8/9 is the FAR-FIELD anchor.
% So the filter starts on the b = 1 curve and then integrates along the
% b = 8/9 one. Open loop, closed form 1/(1-a_bar) = b*w_bar + C:
%
%   seed b=1, integrate b=8/9 :  a_bar(1.10) = 0.7097   truth 0.0873   8.1x
%   seed b=1, integrate b=1   :  a_bar(1.10) = 0.0909   truth 0.0873   +4%
%
% So the far-field anchor is the wrong constant for a trajectory that ends at
% the wall, and the contact anchor is nearly exact there. The driver already
% carries both as arms ('b1' = locked at 1, 'bmid' = locked at 8/9), so this
% needs no code change.
%
% PRE-REGISTERED: if the seed/law mismatch is load-bearing, the b1 arm's
% trough bias must be MUCH smaller than bmid's. If b1 is no better, the open
% loop argument does not survive contact with the closed loop and I drop it.
cd('/Users/kevin/Code/MotionControl_Simu-motion-test');
addpath(genpath('test_script')); addpath(genpath('model'));

SEEDS = 1:30;  AX = 3;
arms = {'b1', 'bmid', 'best'};   % locked at 1 | locked at 8/9 | free from 8/9
R = struct();
for i = 1:numel(arms)
    clear run_formC_b motion_control_law_formC_b;
    fprintf('\n=== arm %s ===\n', arms{i});
    D = run_formC_b(struct('seeds', SEEDS, 'arm', arms{i}));
    t = D.runs{1}.tout(2:end);  ih = t >= 3.75;  im = t >= 0.5 & t < 3.5;
    ns = numel(D.runs);  bh = zeros(1, ns);  bm = zeros(1, ns);
    for s = 1:ns
        r  = D.runs{s};
        ah = r.a_bar_hat_out(2:end, AX);
        at = r.a_true_out(2:end, AX) / r.a_nom;
        bh(s) = mean((ah(ih) - at(ih)) ./ at(ih));      % trough hold
        bm(s) = mean((ah(im) - at(im)) ./ at(im));      % descent + oscillation
    end
    R.(arms{i}) = struct('hold', bh, 'move', bm);
end

fprintf('\n============ trough-hold bias on a_hat, 30 seeds ============\n');
fprintf('  arm    b used |   bias      sd     SEM   |  moving-window bias\n');
bval = struct('b1', 1, 'bmid', 8/9, 'best', 8/9);
for i = 1:numel(arms)
    a = arms{i};  x = R.(a).hold;  y = R.(a).move;
    fprintf('  %-6s %6.4f | %+7.3f%% %6.3f%% %6.3f%% | %+7.3f%% %6.3f%%\n', ...
            a, bval.(a), 100*mean(x), 100*std(x), 100*std(x)/sqrt(numel(x)), ...
            100*mean(y), 100*std(y));
end
d = R.b1.hold - R.bmid.hold;
fprintf('\n  paired b1 - bmid : %+8.4f%%   t = %+.2f   (n = %d)\n', ...
        100*mean(d), mean(d)/(std(d)/sqrt(numel(d))), numel(d));
save('test_results/am_r22_deep/seed_law_b_consistency.mat', 'R', 'arms', 'SEEDS');
