%VERIFY_5STATE_EXPGAIN_READOUT_WHERE  Motion, or the gain changing?
%
%   Established: the readout equation
%       sigma^2_{dh_r} = C_dpmr*4kBT*a_h + C_n*sigma^2_n
%   is accurate to a flat +1.0% at EVERY static height (h_bar 2 .. 22.2), but
%   during a 1 Hz oscillation it under-predicts by 4%, and a_hm reads +5% high.
%   The excess scales with oscillation frequency and is completely insensitive
%   to a_cov (the averaging window), and the across-seed decomposition shows it
%   is not deterministic trajectory content.
%
%   Two candidates remain, and one experiment separates them:
%
%     (i)  MOTION itself -- the delayed error dh_m = h_d[k-d] - h[k] carries the
%          command's own d-step displacement, which is large during motion.
%     (ii) THE GAIN CHANGING inside the loop's/estimator's memory -- a_h swings
%          +-22% over an oscillation near the wall, and every variance relation
%          behind C_dpmr assumes it constant.
%
%   Oscillate with the SAME amplitude and the SAME frequency, but centred far
%   from the wall where a' ~ 0 so a_h is essentially constant:
%
%     near band  h_bar 2.0 - 4.3   a_h swings 0.47 - 0.74 a_o   (+-22%)
%     far band   h_bar 20  - 22.2  a_h swings 0.944 - 0.949 a_o (+-0.3%)
%
%   bias vanishes in the far band -> (ii): the readout needs a_h-varying terms
%   bias survives in the far band -> (i): it is the motion, independent of gain
%
%   The a_pd sweep is included because a_pd sets the mean-tracking EWMA that
%   defines dh_r, and it is the window that candidate (i) would act through
%   (the a_cov sweep already ruled out the variance EWMA).

clear; clc;
here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', '..')));
pc = physical_constants();
az = 3;
seeds = [7 11 23 42 101 777];

base = user_config();
base.trajectory_type = 'osc';
base.amplitude = 2.5; base.frequency = 1; base.n_cycles = 2;
base.t_hold = 0.5; base.t_descend_override = 1.0; base.T_sim = 4.8;
base.h_min = 1.05 * pc.R;
base.ctrl_enable = true; base.thermal_enable = true; base.meas_noise_enable = true;
base.lambda_c = 0.7; base.a_pd = 0.05; base.a_cov = 0.05;
base.meas_noise_std = [0.00062; 0.00057; 0.00331];
base.h_bar_safe = 1.5;

kBT = pc.k_B*pc.T;  s2n = base.meas_noise_std(az)^2;
mko = @(c) struct('lambda_c', c.lambda_c, 'option', 'A_MA2_full', ...
                  'sigma2_n_s', c.meas_noise_std(:).^2, 'kBT', kBT, 'd', 2, ...
                  'a_cov', c.a_cov, 'a_pd', c.a_pd, 't_warmup_kf', 0, ...
                  'h_bar_safe', 1.5, 'iir_warmup_mode', 'prefill');

lastwarn('');

fprintf('=== same oscillation, near the wall vs far from it ===\n');
fprintf('%-34s | %7s %7s | %9s %9s %7s | %8s\n', 'band', 'h_bar', 'a swing', ...
        'Var meas', 'Var pred', 'ratio', 'a_hm/aT');
bands = { 'near  h_bar 2.0-4.3  (a +-22%)',  50, 4.5; ...
          'mid   h_bar 8.9-11.1 (a +-3%)',   50, 20;  ...
          'far   h_bar 20-22.2  (a +-0.3%)', 50, 45 };
for k = 1:size(bands,1)
    c = base; c.h_init = bands{k,2}; c.h_bottom = bands{k,3};
    v = readout_stats(c, mko(c), seeds, az, kBT, s2n, 1.6, 3.5);
    fprintf('%-34s | %7.2f %6.1f%% | %9.1f %9.1f %7.3f | %8.4f\n', bands{k,1}, ...
            v(5), 100*v(6), 1e6*v(1), 1e6*v(2), v(1)/v(2), v(3));
end

fprintf('\n=== a_pd sweep, near band 1 Hz (a_pd sets the EWMA that defines dh_r) ===\n');
fprintf('%8s | %8s | %9s %9s %7s | %8s\n', 'a_pd', 'tau [ms]', ...
        'Var meas', 'Var pred', 'ratio', 'a_hm/aT');
for apd = [0.01 0.05 0.20]
    c = base; c.h_init = 50; c.h_bottom = 4.5; c.a_pd = apd;
    v = readout_stats(c, mko(c), seeds, az, kBT, s2n, 1.6, 3.5);
    fprintf('%8.2f | %8.1f | %9.1f %9.1f %7.3f | %8.4f\n', apd, ...
            1000*(1/apd)/1600, 1e6*v(1), 1e6*v(2), v(1)/v(2), v(3));
end

[wmsg, wid] = lastwarn();
if isempty(wmsg)
    fprintf('\nWARNINGS: none\n');
else
    fprintf('\nWARNINGS: [%s] %s\n', wid, wmsg);
end


function v = readout_stats(cfg, o, seeds, az, kBT, s2n, t0, t1)
%READOUT_STATS  [Var meas, Var pred, a_hm/a_true, --, mean h_bar, a swing frac]
    K = build_eq17_6state_constants(o);
    nS = numel(seeds);
    for q = 1:nS
        s = run_5state_expgain(cfg, struct('seed', seeds(q), 'verbose', false, ...
                                           'variant', 'alg'));
        if q == 1
            t = s.tout(:); m = t > t0 & t <= t1;
            DR = zeros(sum(m), nS); AT = zeros(sum(m), nS);
            AM = zeros(sum(m), nS); HB = zeros(sum(m), nS);
        end
        DR(:, q) = s.dx_r_out(m, az);
        AT(:, q) = s.a_true_out(m, az);
        AM(:, q) = s.a_xm_out(m, az);
        HB(:, q) = s.h_bar_true_out(m);
    end
    v_meas = mean(DR(:).^2);
    v_pred = K.C_dpmr * 4 * kBT * mean(AT(:)) + K.C_n * s2n;
    swing  = (max(AT(:,1)) - min(AT(:,1))) / (2*mean(AT(:)));
    v = [v_meas, v_pred, mean(mean(AM./max(AT, eps), 2)), 0, mean(HB(:)), swing];
end
