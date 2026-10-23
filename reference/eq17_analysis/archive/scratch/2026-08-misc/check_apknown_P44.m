% Follow-up to Stage A: is the ap_known arm's sign flip driven by the DELETED
% A_a*M term rather than by the true slope?
%
% The controller's own comment (motion_control_law_formC_b.m ~1050) records
% this as a defect found 2026-08-13: "A_a*M is what grows P44 along the
% descent, so removing it made the filter over-confident". ap_known sets
% A_a = 0 (controller:1082), so it re-creates exactly that condition.
%
% Prediction, written before the run: if over-confidence is the driver, the
% ap_known arm's P44 must be MUCH SMALLER than the baseline's, and the gap
% must OPEN during the descent (that is where A_a*M does its growing).
% If P44 is comparable between arms, the sign flip is NOT over-confidence and
% this hypothesis is dead.
%
% 10 seeds is enough: P44 is a covariance, not a noisy realization -- it is
% near-deterministic across seeds. No production file is modified.
cd('/Users/kevin/Code/MotionControl_Simu-motion-test');
addpath(genpath('test_script')); addpath(genpath('model'));

SEEDS = 1:10;  AX = 3;
out = struct();
for a = 1:2
    clear run_formC_b motion_control_law_formC_b;
    if a == 1
        o = struct('seeds', SEEDS, 'arm', 'bmid');                      nm = 'base';
    else
        o = struct('seeds', SEEDS, 'arm', 'bmid', 'ap_known', true);    nm = 'apknown';
    end
    D = run_formC_b(o);
    N = numel(D.runs{1}.tout);
    Pa = zeros(N, numel(SEEDS));  Ka = zeros(N, numel(SEEDS));
    for s = 1:numel(SEEDS)
        Pa(:, s) = D.runs{s}.P_a_out(:, AX);
        Ka(:, s) = D.runs{s}.K_a_y1_out(:, AX);
    end
    out.(nm).P = Pa(2:end, :);  out.(nm).K = Ka(2:end, :);
    out.t = D.runs{1}.tout(2:end);
end

t = out.t;
wins = {{'far hold', t < 0.5}, {'descend', t >= 0.5 & t < 1.5}, ...
        {'oscillate', t >= 1.5 & t < 3.5}, {'trough hold', t >= 3.75}};
fprintf('\n  window        sqrt(P44) base   sqrt(P44) apknown   ratio    |K1| base  |K1| apk   ratio\n');
for w = 1:numel(wins)
    m = wins{w}{2};
    pB = mean(mean(sqrt(max(out.base.P(m, :), 0))));
    pA = mean(mean(sqrt(max(out.apknown.P(m, :), 0))));
    kB = mean(mean(abs(out.base.K(m, :))));
    kA = mean(mean(abs(out.apknown.K(m, :))));
    fprintf('  %-12s  %11.3e     %11.3e     %6.3f   %9.3e %9.3e   %6.3f\n', ...
            wins{w}{1}, pB, pA, pA/pB, kB, kA, kA/kB);
end
fprintf('\n');
