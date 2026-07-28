%VERIFY_5STATE_EXPGAIN_INIT  Acceptance V5 + V6: can the filter FIND the gain?
%
%   V5  IDENTIFIABILITY UNDER AN HONEST-BUT-WRONG SEED. With the asymptotic
%       seed the gain starts at the right value (0.0007% off here), so
%       a_hat/a_true = 1.000 only proves the estimate did not WALK AWAY -- it
%       proves nothing about whether the filter could have found it. Replacing
%       the seed with flat a_nom (+5.3% at h_bar_0 = 22.2) turns the same run
%       into a real question. This is the check that made the power-law
%       sibling's y1 channel confess: there, y2 alone cleared the error within
%       a second while y1 contributed bias and no usable information.
%
%   V6  CHANNEL 2x2. Which measurement carries the gain's level?
%       y1+y2 / y1 only / y2 only / neither (prior + predict only).
%
%   V-prior  The production Pf_a_frac is matched to the asymptotic seed's own
%       residual and is therefore very tight. A tight prior also throttles how
%       fast ANY systematic error can accumulate, so the drift test's verdict
%       is re-checked here at a deliberately loose prior.
%
%   Focus axis z, pure positioning at h_bar = 22.2, 6 seeds.

clear; clc;
here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', '..')));
pc = physical_constants();
az = 3;
seeds = [7 11 23 42 101 777];

base = user_config();
base.h_min = 1.05 * pc.R;
base.ctrl_enable = true; base.thermal_enable = true; base.meas_noise_enable = true;
base.lambda_c = 0.7; base.a_pd = 0.05; base.a_cov = 0.05;
base.meas_noise_std = [0.00062; 0.00057; 0.00331];
base.h_bar_safe = 1.5;
pos = base; pos.trajectory_type = 'positioning'; pos.h_init = 50; pos.T_sim = 12;

lastwarn('');

fprintf('=== V5  seed = flat a_nom (+5.3%%): can the filter find a_h? ===\n');
fprintf('%-34s | %8s %8s %8s %8s\n', 'arm', 't=0', 't=1', 't=6', 't=12');
arms = { ...
  'asym seed  (production)',        struct(); ...
  'flat seed  y1+y2',               struct('a_init_mode','flat'); ...
  'flat seed  y1 only (y2 off)',    struct('a_init_mode','flat','y2_off',true); ...
  'flat seed  y2 only (y1 gain off)',struct('a_init_mode','flat','y1_gain_off',true); ...
  'flat seed  neither (open loop)', struct('a_init_mode','flat','y2_off',true,'y1_gain_off',true) };
for r = 1:size(arms, 1)
    v = run_arm(pos, arms{r,2}, seeds, az);
    fprintf('%-34s | %8.4f %8.4f %8.4f %8.4f\n', arms{r,1}, v(1), v(2), v(3), v(4));
end
fprintf('\nThe flat seed starts at 1.053. An arm that stays there learned nothing;\n');
fprintf('an arm that reaches 1.00 found the gain from the data.\n');

fprintf('\n=== V-prior  does the tight production prior hide the fdet effect? ===\n');
fprintf('%-34s | %8s %8s %8s %8s\n', 'arm', 't=0', 't=1', 't=6', 't=12');
for pf = [0.002 0.0101 0.05]
    for fd = [true false]
        ov = struct('Pf_a_frac', pf, 'use_fdet', fd);
        v = run_arm(pos, ov, seeds, az);
        if fd; nm = 'fdet ON '; else; nm = 'fdet OFF'; end
        fprintf('Pf_a_frac %-6.4g  %-8s          | %8.4f %8.4f %8.4f %8.4f\n', ...
                pf, nm, v(1), v(2), v(3), v(4));
    end
end
fprintf('\n0.002  = production default (matched to the asymptotic seed residual)\n');
fprintf('0.0101 = the power-law sibling''s prior, for a like-for-like drift scale\n');

[wmsg, wid] = lastwarn();
if isempty(wmsg)
    fprintf('\nWARNINGS: none\n');
else
    fprintf('\nWARNINGS: [%s] %s\n', wid, wmsg);
end


function r = run_arm(cfg, ov, seeds, az)
%RUN_ARM  Mean a_hat/a_true at t = 0 / 1 / 6 / 12 s over the seed set.
    V = zeros(numel(seeds), 4);
    tq = [0 1 6 12];
    for q = 1:numel(seeds)
        s = run_5state_expgain(cfg, struct('seed', seeds(q), 'verbose', false, ...
                                           'ctrl_const_override', ov));
        t = s.tout(:);
        rr = s.a_hat_out(:, az) ./ s.a_true_out(:, az);
        for k = 1:4
            [~, i1] = min(abs(t - min(tq(k), cfg.T_sim)));
            V(q, k) = rr(i1);
        end
    end
    r = mean(V, 1);
end
