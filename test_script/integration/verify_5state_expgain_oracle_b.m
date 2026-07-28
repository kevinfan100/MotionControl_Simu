%VERIFY_5STATE_EXPGAIN_ORACLE_B  Oracle ablation: is the +2.2% steady bias
%   caused by the gain MODEL, or by something the gain model cannot touch?
%
%   The near-wall bias a_hat/a_true ~ +2.2% survives every structural change
%   made so far (7b 1.0238, 7a 1.0220, power-law sibling 1.0178), so it is not
%   caused by HOW the family is used. This test asks whether it is caused by
%   the family at all.
%
%   Method (the project's oracle-ablation pattern: pin the suspect parameter at
%   its known-best value, cheat deliberately, and see whether the symptom
%   survives).  b is frozen at three reference values:
%
%       0.9232  best constant b over the oscillation band h_bar in [2, 4.3]
%               -> model level error there is 0.272% RMS / 0.435% max
%       0.9386  best constant b over the whole traversed range [2, 22.2]
%       1.0000  the value both asymptotic anchors give
%
%   Reading:
%     bias falls to ~0.3-0.4%  -> the family's single constant b IS the cause;
%                                 the estimator is placing b at a compromise
%                                 between far field and near wall
%     bias stays near 2%       -> the gain model is innocent. Look at the
%                                 readout chain instead (C_dpmr / IF_var /
%                                 the closed-loop variance formula), none of
%                                 which the 7a/7b reformulation touches.
%
%   The decisive column is a_hm/a_true: a_hm is the RAW variance readout and
%   never passes through the gain model. If IT is biased +2%, the gain model
%   cannot be the cause of anything.
%
%   Two windows are reported separately because they are different regimes and
%   the earlier "osc window t>1.6" silently merged them:
%       osc       1.6 - 3.5 s   the actual oscillation
%       nearwall  3.5 - 4.8 s   parked at h_bar = 2.0

clear; clc;
here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', '..')));
pc = physical_constants();
az = 3;
seeds = [7 11 23 42 101 777];

cfg = user_config();
cfg.trajectory_type = 'osc';
cfg.h_init = 50; cfg.h_bottom = 4.5; cfg.amplitude = 2.5;
cfg.frequency = 1; cfg.n_cycles = 2; cfg.t_hold = 0.5;
cfg.t_descend_override = 1.0; cfg.T_sim = 4.8;
cfg.h_min = 1.05 * pc.R;
cfg.ctrl_enable = true; cfg.thermal_enable = true; cfg.meas_noise_enable = true;
cfg.lambda_c = 0.7; cfg.a_pd = 0.05; cfg.a_cov = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
cfg.h_bar_safe = 1.5;

FROZEN = struct('Pf_b_std', 1e-6);

arms = { 'diff  production (b est)',   'diff', struct(); ...
         'alg   production (b est)',   'alg',  struct(); ...
         'alg   b frozen 0.9232 (band opt)', 'alg', setfield(FROZEN,'b_init',0.9232); ...
         'alg   b frozen 0.9386 (range opt)','alg', setfield(FROZEN,'b_init',0.9386); ...
         'alg   b frozen 1.0000 (anchors)',  'alg', setfield(FROZEN,'b_init',1.0) };

lastwarn('');
fprintf('=== oracle-b ablation, z axis, 6 seeds ===\n\n');
fprintf('%-34s | %18s | %18s | %8s\n', '', 'OSC 1.6-3.5 s', 'NEARWALL 3.5-4.8 s', '');
fprintf('%-34s | %8s %9s | %8s %9s | %8s\n', 'arm', ...
        'a_hat/aT', 'a_hm/aT', 'a_hat/aT', 'a_hm/aT', 'b_hat');
for k = 1:size(arms, 1)
    v = arm_stats(cfg, arms{k,3}, seeds, az, arms{k,2});
    fprintf('%-34s | %8.4f %9.4f | %8.4f %9.4f | %8.4f\n', arms{k,1}, ...
            v(1), v(2), v(3), v(4), v(5));
end

fprintf('\nmodel level error with b frozen (a_o = a_nom, from the exact curve):\n');
fprintf('  b = 0.9232 over [2, 4.3]  : 0.272%% RMS / 0.435%% max\n');
fprintf('  b = 0.9386 over [2, 22.2] : 0.477%% RMS / 1.621%% max\n');
fprintf('  b = 1.0000 over [2, 22.2] : 2.044%% RMS / 6.240%% max\n');

[wmsg, wid] = lastwarn();
if isempty(wmsg)
    fprintf('\nWARNINGS: none\n');
else
    fprintf('\nWARNINGS: [%s] %s\n', wid, wmsg);
end


function v = arm_stats(cfg, ov, seeds, az, variant)
%ARM_STATS  [a_hat/aT osc, a_hm/aT osc, a_hat/aT nearwall, a_hm/aT nearwall, b_hat]
    n = numel(seeds);
    P = zeros(n, 5);
    for q = 1:n
        s = run_5state_expgain(cfg, struct('seed', seeds(q), 'verbose', false, ...
                                           'ctrl_const_override', ov, 'variant', variant));
        t  = s.tout(:);
        wo = t > 1.6 & t <= 3.5;
        wn = t > 3.5;
        aT = s.a_true_out(:, az);
        P(q, :) = [mean(s.a_hat_out(wo,az)./aT(wo)), mean(s.a_xm_out(wo,az)./aT(wo)), ...
                   mean(s.a_hat_out(wn,az)./aT(wn)), mean(s.a_xm_out(wn,az)./aT(wn)), ...
                   mean(s.b_hat_out(wn,az))];
    end
    v = mean(P, 1);
end
