%VERIFY_5STATE_EXPGAIN_BCHANNEL  What drives b_hat during the descent?
%
%   In pure positioning y1 carries ZERO gain information: the deterministic
%   control effort F_dh_det -> 0, so F_e(3,4) = -F_dh_det -> 0 and the gain
%   states are decoupled from the position chain (measured: a_hat frozen at its
%   seed to four decimals).
%
%   During a descent that is no longer true. F_dh_det is large, so b reaches y1
%   through two steps of dynamic coupling,
%
%       b --F_e(4,5)--> a_h --F_e(3,4)--> dh_3 --> dh_1 --> y1
%
%   and b_hat can be driven by the POSITION residual even though its nominal
%   measurement y2 contributes almost nothing (turning y2 off changes the
%   descent error by 0.05%).
%
%   Prediction: switching the gain rows of y1's Kalman gain off should remove
%   the b_hat excursion, and with it about half the descent error -- the same
%   half that freezing b removes.
%
%   Reported over the descent ramp: peak |a_hat error|, peak |b_hat - 1|
%   (the excursion away from the seed), and b_hat at the end of the ramp.

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

lastwarn('');
fprintf('=== which channel moves b_hat during the descent? (z, 6 seeds) ===\n');
fprintf('%-30s | %10s %10s %10s\n', 'arm', 'peak|e_a|%', 'peak|b-1|', 'b @ end');

arms = { 'production (y1 + y2)',        struct(); ...
         'y1 gain rows off',            struct('y1_gain_off', true); ...
         'y2 off',                      struct('y2_off', true); ...
         'both off (predict only)',     struct('y1_gain_off', true, 'y2_off', true); ...
         'b frozen (Pf_b = 0)',         struct('Pf_b_std', 1e-6) };

for k = 1:size(arms, 1)
    ov = arms{k, 2};
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
        bw = s.b_hat_out(w, az);
        P(q, :) = [max(abs(e)), max(abs(bw - 1)), bw(end)];
    end
    v = mean(P, 1);
    fprintf('%-30s | %10.2f %10.4f %10.4f\n', arms{k,1}, v(1), v(2), v(3));
end

fprintf('\nb_slope(h_bar) at the two ends of the ramp: %.4f (h=22.2) -> %.4f (h=2.0)\n', ...
        0.9983, 0.9184);
fprintf('PASS for the hypothesis if "y1 gain rows off" removes both the b_hat\n');
fprintf('excursion and about half the peak a error, while "y2 off" removes neither.\n');

[wmsg, wid] = lastwarn();
if isempty(wmsg)
    fprintf('\nWARNINGS: none\n');
else
    fprintf('\nWARNINGS: [%s] %s\n', wid, wmsg);
end
