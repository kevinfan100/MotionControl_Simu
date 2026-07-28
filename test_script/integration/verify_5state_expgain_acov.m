%VERIFY_5STATE_EXPGAIN_ACOV  Acceptance V2 + V3 for the exponential-gain
%   controller.
%
%   V2  a_cov INVARIANCE. a_cov is the IIR variance estimator's own EWMA
%       weight -- a number internal to the readout, not to the physics. It must
%       not appear in any conclusion. Sweeping it 10x and watching the results
%       move is the sharpest available fingerprint of a modelling error (it is
%       what exposed the un-whitened y2 on the power-law sibling: feeding the
%       AR(1) output as if it were white over-states its information by
%       (2-a_cov)/a_cov = 39x). Both y2 forms are swept here so the acceptance
%       is established ON THIS CONTROLLER, not inherited.
%
%   V3  P[0] IS A BUDGET. b has Q55 = 0 and F_e row 5 = [0 0 0 0 1], so its
%       covariance can only shrink; the total distance the estimate is allowed
%       to travel is fixed by the prior:
%           E[(b_inf - b_0)^2] = P55[0] - P55[inf]
%       Breaking this upper bound means the filter is not self-consistent (it
%       independently caught an illegal "fix the prior but not R2" configuration
%       on the sibling).
%
%   Focus axis z, canonical oscillation scenario, 6 seeds.

clear; clc;
here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', '..')));
pc = physical_constants();
az = 3;
seeds = [7 11 23 42 101 777];

osc = user_config();
osc.h_min = 1.05 * pc.R;
osc.ctrl_enable = true; osc.thermal_enable = true; osc.meas_noise_enable = true;
osc.lambda_c = 0.7; osc.a_pd = 0.05;
osc.meas_noise_std = [0.00062; 0.00057; 0.00331];
osc.h_bar_safe = 1.5;
osc.trajectory_type = 'osc';
osc.h_init = 50; osc.h_bottom = 4.5; osc.amplitude = 2.5;
osc.frequency = 1; osc.n_cycles = 2; osc.t_hold = 0.5;
osc.t_descend_override = 1.0; osc.T_sim = 4.8;

lastwarn('');

for wh = [true false]
    if wh
        fprintf('=== V2  a_cov invariance -- y2 WHITENED (default) ===\n');
    else
        fprintf('\n=== V2  a_cov invariance -- y2 RAW (spec body form) ===\n');
    end
    fprintf('%8s | %9s %8s | %9s %8s | %8s %8s %5s\n', 'a_cov', ...
            'a_hat/aT', 'sc', 'b_osc', 'sc', 'trk z', 'b budget', 'NaN');
    M = [];
    for acv = [0.02 0.05 0.10 0.20]
        c = osc; c.a_cov = acv;
        ov = struct('y2_whiten', wh);
        ar = zeros(6,1); bo = zeros(6,1); tk = zeros(6,1);
        bud = zeros(6,1); nn = 0;
        for q = 1:6
            s = run_5state_expgain(c, struct('seed', seeds(q), 'verbose', false, ...
                                             'ctrl_const_override', ov));
            t = s.tout(:); mo = t > 1.6;
            ar(q) = mean(s.a_hat_out(mo, az) ./ max(s.a_true_out(mo, az), eps));
            bo(q) = mean(s.b_hat_out(mo, az));
            e = s.p_d_out(2:end, :) - s.p_true_out(1:end-1, :); te = t(2:end);
            tk(q) = 1e3 * std(e(te > 1.6, az));
            % V3: travelled^2 vs the prior budget P55[0]-P55[end]
            trav2 = (s.b_hat_out(end, az) - s.b_hat_out(1, az))^2;
            budget = s.P_b_out(1, az)^2 - s.P_b_out(end, az)^2;
            bud(q) = trav2 / max(budget, eps);
            nn = nn + (any(~isfinite(s.a_hat_out(:))) || any(~isfinite(s.p_true_out(:))));
        end
        fprintf('%8.2f | %9.4f %8.4f | %9.4f %8.4f | %8.2f %8.3f %5d\n', acv, ...
                mean(ar), std(ar), mean(bo), std(bo), mean(tk), mean(bud), nn);
        M = [M; mean(ar) mean(bo) mean(tk)]; %#ok<AGROW>
    end
    sp = (max(M,[],1) - min(M,[],1)) ./ max(abs(mean(M,1)), eps);
    fprintf('a_cov invariance:  a_hat %.2f%%   b %.2f%%   trk %.2f%%   (PASS < 1.1%%)\n', ...
            100*sp(1), 100*sp(2), 100*sp(3));
end

fprintf('\nV3 "b budget" column = (b_hat[end]-b_hat[0])^2 / (P55[0]-P55[end]).\n');
fprintf('   Q55 = 0 and F_e row 5 = e_5, so P55 only shrinks and this ratio\n');
fprintf('   must stay <= 1; > 1 means the filter moved further than its own\n');
fprintf('   prior allowed, i.e. it is not self-consistent.\n');

[wmsg, wid] = lastwarn();
if isempty(wmsg)
    fprintf('\nWARNINGS: none\n');
else
    fprintf('\nWARNINGS: [%s] %s\n', wid, wmsg);
end
