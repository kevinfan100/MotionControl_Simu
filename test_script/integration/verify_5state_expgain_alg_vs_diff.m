%VERIFY_5STATE_EXPGAIN_ALG_VS_DIFF  7a vs 7b on the same trajectory and seeds.
%
%   Same two-parameter family a_h = a_o*[1 - h_bar^(-b)], two ways of using it:
%
%     7b 'diff'  a_h is a STATE swept by a' ;  states [a_h, b]
%     7a 'alg'   a_h is EVALUATED from the model ; states [a_o, b], both constant
%
%   Claims to test, in the order they were derived:
%     C1  the descent peak error drops, because the level is no longer obtained
%         by integrating a slope that carries a 1.65% RMS model error over a
%         10x height traverse (7b's predict-only floor was 5.4%)
%     C2  b actually converges, because its y2 sensitivity is O(1) instead of
%         O(height change over d steps) -- predicted 159x to 7886x more
%         information per sample
%     C3  b converges DURING THE INITIAL HOLD, which 7b cannot do at all
%         (its b channel is proportional to motion)
%     C4  adding measurements HELPS rather than hurts, i.e. the trade that
%         signalled the modelling error is gone
%     C5  a_cov invariance survives
%
%   Focus axis z. Reference for b is b_level (the exponent that reproduces the
%   LEVEL at that height) for 7a and b_slope (the exponent that reproduces the
%   SLOPE) for 7b -- each formulation must be judged against the reading it
%   actually uses.

clear; clc;
here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', '..')));
pc = physical_constants();
az = 3;
seeds = [7 11 23 42 101 777];

osc = user_config();
osc.trajectory_type = 'osc';
osc.h_init = 50; osc.h_bottom = 4.5; osc.amplitude = 2.5;
osc.frequency = 1; osc.n_cycles = 2; osc.t_hold = 0.5;
osc.t_descend_override = 1.0; osc.T_sim = 4.8;
osc.h_min = 1.05 * pc.R;
osc.ctrl_enable = true; osc.thermal_enable = true; osc.meas_noise_enable = true;
osc.lambda_c = 0.7; osc.a_pd = 0.05; osc.a_cov = 0.05;
osc.meas_noise_std = [0.00062; 0.00057; 0.00331];
osc.h_bar_safe = 1.5;

pos = osc; pos.trajectory_type = 'positioning'; pos.h_init = 50; pos.T_sim = 12;

lastwarn('');

% ---------------- C1 / C4: descent peak + measurement ablation ----------------
fprintf('=== C1/C4  descent window, z, 6 seeds ===\n');
fprintf('%-8s %-26s | %10s %10s | %10s\n', 'variant', 'arm', 'peak|e_a|%', 'mean e %', 'trk nm');
arms = { 'full (y1+y2)',       struct(); ...
         'predict only',       struct('y1_gain_off', true, 'y2_off', true); ...
         'y2 only',            struct('y1_gain_off', true); ...
         'y1 only',            struct('y2_off', true) };
for vv = {'diff', 'alg'}
    for k = 1:size(arms, 1)
        s = arm_stats(osc, arms{k,2}, seeds, az, vv{1}, 'descent');
        fprintf('%-8s %-26s | %10.2f %10.2f | %10.2f\n', vv{1}, arms{k,1}, s(1), s(2), s(3));
    end
end

% ---------------- C2 / C3: does b converge, and when? ----------------
fprintf('\n=== C2/C3  b convergence (z, seed 7, single run) ===\n');
for vv = {'diff', 'alg'}
    s = run_5state_expgain(osc, struct('seed', 7, 'verbose', false, 'variant', vv{1}));
    t = s.tout(:);
    a_o = s.a_nom;
    bl = -log(max(1 - s.a_true_out(:,az)/a_o, eps)) ./ log(max(s.h_bar_true_out, 1+1e-9));
    bs = s.h_bar_true_out .* s.a_prime_true_out(:,az) ./ max(a_o - s.a_true_out(:,az), eps);
    if strcmp(vv{1}, 'alg'); bref = bl; nmref = 'b_level'; else; bref = bs; nmref = 'b_slope'; end
    idx = @(tt) find(t >= tt, 1, 'first');
    fprintf('%-5s  b_hat @ t = 0 / 0.45(end of hold) / 1.5 / 4.8 s : %.4f %.4f %.4f %.4f\n', ...
            vv{1}, s.b_hat_out(1,az), s.b_hat_out(idx(0.45),az), ...
            s.b_hat_out(idx(1.5),az), s.b_hat_out(end,az));
    fprintf('       reference %-8s at the same times          : %.4f %.4f %.4f %.4f\n', ...
            nmref, bref(1), bref(idx(0.45)), bref(idx(1.5)), bref(end));
    if strcmp(vv{1}, 'alg')
        fprintf('       a_o_hat/a_nom @ 0 / 1.5 / 4.8 s               : %.4f %.4f %.4f\n', ...
                s.ao_hat_out(1,az)/a_o, s.ao_hat_out(idx(1.5),az)/a_o, s.ao_hat_out(end,az)/a_o);
    end
end

% ---------------- steady state + positioning ----------------
fprintf('\n=== steady state (osc window t>1.6) and pure positioning ===\n');
fprintf('%-8s | %10s %8s | %10s | %10s\n', 'variant', 'a/aT osc', 'sc', 'trk nm', 'a/aT pos12');
for vv = {'diff', 'alg'}
    so = arm_stats(osc, struct(), seeds, az, vv{1}, 'osc');
    sp = arm_stats(pos, struct(), seeds, az, vv{1}, 'end');
    fprintf('%-8s | %10.4f %8.4f | %10.2f | %10.4f\n', vv{1}, so(4), so(5), so(3), sp(4));
end

% ---------------- C5: a_cov invariance ----------------
fprintf('\n=== C5  a_cov invariance (osc, z) ===\n');
for vv = {'diff', 'alg'}
    M = [];
    for acv = [0.02 0.05 0.10 0.20]
        c = osc; c.a_cov = acv;
        s = arm_stats(c, struct(), seeds, az, vv{1}, 'osc');
        M = [M; s(4) s(3)]; %#ok<AGROW>
    end
    sp = (max(M,[],1) - min(M,[],1)) ./ max(abs(mean(M,1)), eps);
    fprintf('%-8s  a_hat %.2f%%   trk %.2f%%   (PASS < 1.1%%)\n', vv{1}, 100*sp(1), 100*sp(2));
end

[wmsg, wid] = lastwarn();
if isempty(wmsg)
    fprintf('\nWARNINGS: none\n');
else
    fprintf('\nWARNINGS: [%s] %s\n', wid, wmsg);
end


function v = arm_stats(cfg, ov, seeds, az, variant, window)
%ARM_STATS  [peak|e|%, mean e%, tracking std nm, mean a/aT, seed scatter]
    n = numel(seeds);
    P = zeros(n, 4);
    for q = 1:n
        s = run_5state_expgain(cfg, struct('seed', seeds(q), 'verbose', false, ...
                                           'ctrl_const_override', ov, 'variant', variant));
        t = s.tout(:);
        switch window
            case 'descent'
                w = t >= cfg.t_hold & t <= cfg.t_hold + cfg.t_descend_override;
                wt = w;
            case 'osc'
                w = t > 1.6; wt = w;
            case 'end'
                w = t > max(1.0, 0.9*cfg.T_sim); wt = t > 1.0;
        end
        aT = s.a_true_out(w, az);
        e  = 100 * (s.a_hat_out(w, az) - aT) ./ aT;
        er = s.p_d_out(2:end, :) - s.p_true_out(1:end-1, :);
        te = t(2:end);
        P(q, :) = [max(abs(e)), mean(e), 1e3*std(er(wt(2:end), az)), ...
                   mean(s.a_hat_out(w, az) ./ max(aT, eps))];
    end
    v = [mean(P, 1), std(P(:, 4))];
end
