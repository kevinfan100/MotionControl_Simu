%VERIFY_5STATE_EXPGAIN_READOUT_FREQ  Is the readout bias a RATE effect?
%
%   Established so far: during the oscillation the measured Var(dh_r) exceeds
%   what the readout equation
%       sigma^2_{dh_r} = C_dpmr*4kBT*a_h + C_n*sigma^2_n
%   predicts from the true gain, by ~4%, and a_hm inherits it (+5%). When
%   parked (either far or near the wall) the same equation is exact. Already
%   ruled out: deterministic trajectory contamination (the across-seed
%   deterministic fraction sits at the 1/6 sampling floor in every window) and
%   the a_hat self-consistency loop (pinning a_ctrl = a_true leaves it).
%
%   Remaining candidate: the gain a_h is not constant over the window the
%   readout averages. The EWMA time constant is 1/a_cov = 20 steps = 12.5 ms;
%   at 1 Hz and 2.5 um amplitude the height moves 0.087 h_bar in that time, and
%   a_h moves ~3.5%. If that is the mechanism, the bias must scale with how
%   fast a_h changes, i.e. with FREQUENCY at fixed amplitude.
%
%   Two sweeps, both at fixed amplitude so the height RANGE is unchanged and
%   only the RATE varies:
%     A  frequency 0.25 / 0.5 / 1 / 2 Hz
%     B  a_cov 0.02 / 0.05 / 0.20, which changes the averaging window length
%        by 10x at fixed trajectory -- the same mechanism seen from the other
%        side, and a_cov must not appear in the answer for any other reason.
%
%   Flat in both  -> not a rate effect; look elsewhere.
%   Scales in both -> the readout equation needs a term for a_h varying inside
%                     the averaging window.

clear; clc;
here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', '..')));
pc = physical_constants();
az = 3;
seeds = [7 11 23 42 101 777];

base = user_config();
base.trajectory_type = 'osc';
base.h_init = 50; base.h_bottom = 4.5; base.amplitude = 2.5;
base.n_cycles = 2; base.t_hold = 0.5; base.t_descend_override = 1.0;
base.h_min = 1.05 * pc.R;
base.ctrl_enable = true; base.thermal_enable = true; base.meas_noise_enable = true;
base.lambda_c = 0.7; base.a_pd = 0.05; base.a_cov = 0.05;
base.meas_noise_std = [0.00062; 0.00057; 0.00331];
base.h_bar_safe = 1.5;

o = struct('lambda_c', base.lambda_c, 'option', 'A_MA2_full', ...
           'sigma2_n_s', base.meas_noise_std(:).^2, 'kBT', pc.k_B*pc.T, 'd', 2, ...
           'a_cov', base.a_cov, 'a_pd', base.a_pd, 't_warmup_kf', 0, ...
           'h_bar_safe', 1.5, 'iir_warmup_mode', 'prefill');
kBT = pc.k_B*pc.T;  s2n = base.meas_noise_std(az)^2;

lastwarn('');

fprintf('=== A  frequency sweep (amplitude fixed, so the height RANGE is fixed) ===\n');
fprintf('%8s | %8s | %9s %9s %7s | %8s\n', 'f [Hz]', 'd a/dt', ...
        'Var meas', 'Var pred', 'ratio', 'a_hm/aT');
for f = [0.25 0.5 1 2]
    c = base; c.frequency = f;
    c.T_sim = c.t_hold + c.t_descend_override + c.n_cycles/f + 0.3;
    t0 = c.t_hold + c.t_descend_override + 0.1;
    t1 = c.t_hold + c.t_descend_override + c.n_cycles/f;
    v = readout_stats(c, o, seeds, az, kBT, s2n, t0, t1);
    fprintf('%8.2f | %8.2f | %9.1f %9.1f %7.3f | %8.4f\n', f, v(4), ...
            1e6*v(1), 1e6*v(2), v(1)/v(2), v(3));
end

fprintf('\n=== B  a_cov sweep at 1 Hz (changes the averaging window 10x) ===\n');
fprintf('%8s | %8s | %9s %9s %7s | %8s\n', 'a_cov', 'tau [ms]', ...
        'Var meas', 'Var pred', 'ratio', 'a_hm/aT');
for acv = [0.02 0.05 0.20]
    c = base; c.frequency = 1; c.T_sim = 4.8; c.a_cov = acv;
    oo = o; oo.a_cov = acv;
    v = readout_stats(c, oo, seeds, az, kBT, s2n, 1.6, 3.5);
    fprintf('%8.2f | %8.1f | %9.1f %9.1f %7.3f | %8.4f\n', acv, ...
            1000*(1/acv)*(1/1600), 1e6*v(1), 1e6*v(2), v(1)/v(2), v(3));
end

fprintf('\n=== C  static holds at fixed heights (control: no motion at all) ===\n');
fprintf('%8s | %9s %9s %7s | %8s\n', 'h_bar', 'Var meas', 'Var pred', 'ratio', 'a_hm/aT');
for hb = [2.0 3.2 4.3 10 22.2]
    c = base; c.trajectory_type = 'positioning'; c.h_init = hb*pc.R; c.T_sim = 3.0;
    v = readout_stats(c, o, seeds, az, kBT, s2n, 0.5, 3.0);
    fprintf('%8.2f | %9.1f %9.1f %7.3f | %8.4f\n', hb, ...
            1e6*v(1), 1e6*v(2), v(1)/v(2), v(3));
end

[wmsg, wid] = lastwarn();
if isempty(wmsg)
    fprintf('\nWARNINGS: none\n');
else
    fprintf('\nWARNINGS: [%s] %s\n', wid, wmsg);
end


function v = readout_stats(cfg, o, seeds, az, kBT, s2n, t0, t1)
%READOUT_STATS  [Var(dh_r) measured, predicted, mean a_hm/a_true, mean |da/dt|/a]
    K = build_eq17_6state_constants(o);
    nS = numel(seeds);
    for q = 1:nS
        s = run_5state_expgain(cfg, struct('seed', seeds(q), 'verbose', false, ...
                                           'variant', 'alg'));
        if q == 1
            t = s.tout(:); m = t > t0 & t <= t1;
            DR = zeros(sum(m), nS); AT = zeros(sum(m), nS); AM = zeros(sum(m), nS);
            dadt = mean(abs(diff(s.a_true_out(m, az)))) / (t(2)-t(1)) ...
                   / mean(s.a_true_out(m, az));
        end
        DR(:, q) = s.dx_r_out(m, az);
        AT(:, q) = s.a_true_out(m, az);
        AM(:, q) = s.a_xm_out(m, az);
    end
    v_meas = mean(DR(:).^2);
    v_pred = K.C_dpmr * 4 * kBT * mean(AT(:)) + K.C_n * s2n;
    v = [v_meas, v_pred, mean(mean(AM./max(AT, eps), 2)), dadt];
end
