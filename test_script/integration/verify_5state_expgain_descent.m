%VERIFY_5STATE_EXPGAIN_DESCENT  Where does the descent-phase gain error come from?
%
%   The canonical run shows the largest a_hat error of the whole trajectory
%   during the descent (t ~ 1.0-1.5 s, h_bar sweeping 10 -> 2), not near the
%   wall and not at steady state. Four candidate mechanisms, each with a
%   distinguishing experiment:
%
%   H1  READOUT LAG / RAMP CONTAMINATION.  a_hm is built from the residual
%       dh_r = dh_m - EWMA_mean(dh_m). On a ramp the EWMA mean lags the actual
%       mean, so dh_r keeps a DETERMINISTIC component, inflating the variance
%       and hence a_hm. Signature: the error scales with the descent RATE and
%       with a_pd; a_hm itself (not just a_hat) is biased during the descent.
%
%   H2  a_cov LEAK.  The invariance test used the window t > 1.6 s, which
%       EXCLUDES the descent. If a_cov leaks anywhere, this is where.
%
%   H3  SINGLE-b TRUNCATION.  b_slope sweeps 0.998 -> 0.918 across the descent,
%       and the model carries one constant b. Signature: the error survives
%       when b is frozen at the correct local value, or shrinks when it is.
%
%   H4  ROW-4 COUPLING SIZE.  Delta_hbar_d during the descent is ~0.0125/step,
%       3x its peak during the oscillation, so the gain-recursion terms are at
%       their largest here.  Signature: sensitivity to fe_row4_full.
%
%   All arms report the same three numbers over the descent window: the peak
%   |a_hat error|, its mean, and the mean |a_hm error| (the raw readout).

clear; clc;
here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', '..')));
pc = physical_constants();
az = 3;
seeds = [7 11 23 42 101 777];

base = user_config();
base.trajectory_type = 'osc';
base.h_init = 50; base.h_bottom = 4.5; base.amplitude = 2.5;
base.frequency = 1; base.n_cycles = 2; base.t_hold = 0.5;
base.t_descend_override = 1.0; base.T_sim = 4.8;
base.h_min = 1.05 * pc.R;
base.ctrl_enable = true; base.thermal_enable = true; base.meas_noise_enable = true;
base.lambda_c = 0.7; base.a_pd = 0.05; base.a_cov = 0.05;
base.meas_noise_std = [0.00062; 0.00057; 0.00331];
base.h_bar_safe = 1.5;

lastwarn('');

fprintf('=== descent-window diagnostics (z), 6 seeds ===\n');
fprintf('window = the descent ramp itself; peak/mean over it\n\n');

fprintf('--- H2: does a_cov leak into the DESCENT? (invariance was only\n');
fprintf('        ever checked on t > 1.6 s, which excludes it) ---\n');
fprintf('%10s | %9s %9s | %9s\n', 'a_cov', 'peak|e| %', 'mean e %', 'a_hm e %');
M = [];
for acv = [0.02 0.05 0.10 0.20]
    c = base; c.a_cov = acv;
    v = descent_stats(c, struct(), seeds, az);
    fprintf('%10.2f | %9.2f %9.2f | %9.2f\n', acv, v(1), v(2), v(3));
    M = [M; v]; %#ok<AGROW>
end
sp = (max(M,[],1) - min(M,[],1)) ./ max(abs(mean(M,1)), eps);
fprintf('a_cov spread over the descent: peak %.1f%%  mean %.1f%%  a_hm %.1f%%\n\n', ...
        100*sp(1), 100*sp(2), 100*sp(3));

fprintf('--- H1: does the error scale with the descent RATE? ---\n');
fprintf('%10s | %9s %9s | %9s\n', 't_desc[s]', 'peak|e| %', 'mean e %', 'a_hm e %');
for td = [0.5 1.0 2.0 4.0]
    c = base; c.t_descend_override = td; c.T_sim = 4.8 + (td - 1.0);
    v = descent_stats(c, struct(), seeds, az);
    fprintf('%10.1f | %9.2f %9.2f | %9.2f\n', td, v(1), v(2), v(3));
end
fprintf('\n');

fprintf('--- H1b: does a_pd (the LP mean that dh_r is taken against) matter? ---\n');
fprintf('%10s | %9s %9s | %9s\n', 'a_pd', 'peak|e| %', 'mean e %', 'a_hm e %');
for apd = [0.02 0.05 0.20]
    c = base; c.a_pd = apd;
    v = descent_stats(c, struct(), seeds, az);
    fprintf('%10.2f | %9.2f %9.2f | %9.2f\n', apd, v(1), v(2), v(3));
end
fprintf('\n');

fprintf('--- H3 / H4: model structure ---\n');
fprintf('%-28s | %9s %9s | %9s\n', 'arm', 'peak|e| %', 'mean e %', 'a_hm e %');
arms = { 'production',                 struct(); ...
         'b frozen at 1 (Pf_b=0)',     struct('Pf_b_std', 1e-6); ...
         'b frozen at 0.93 (best fit)',struct('Pf_b_std', 1e-6, 'b_init', 0.93); ...
         'fe_row4_full = false',       struct('fe_row4_full', false); ...
         'y2 off (predict only)',      struct('y2_off', true) };
for k = 1:size(arms, 1)
    v = descent_stats(base, arms{k,2}, seeds, az);
    fprintf('%-28s | %9.2f %9.2f | %9.2f\n', arms{k,1}, v(1), v(2), v(3));
end

[wmsg, wid] = lastwarn();
if isempty(wmsg)
    fprintf('\nWARNINGS: none\n');
else
    fprintf('\nWARNINGS: [%s] %s\n', wid, wmsg);
end


function v = descent_stats(cfg, ov, seeds, az)
%DESCENT_STATS  [peak |a_hat err| %, mean a_hat err %, mean a_hm err %] over
%   the descent ramp, averaged across seeds.
    t0 = cfg.t_hold;
    t1 = cfg.t_hold + cfg.t_descend_override;
    P = zeros(numel(seeds), 3);
    for q = 1:numel(seeds)
        s = run_5state_expgain(cfg, struct('seed', seeds(q), 'verbose', false, ...
                                           'ctrl_const_override', ov));
        t = s.tout(:);
        w = t >= t0 & t <= t1;
        aT = s.a_true_out(w, az);
        e  = 100 * (s.a_hat_out(w, az) - aT) ./ aT;
        em = 100 * (s.a_xm_out(w, az)  - aT) ./ aT;
        P(q, :) = [max(abs(e)), mean(e), mean(em)];
    end
    v = mean(P, 1);
end
