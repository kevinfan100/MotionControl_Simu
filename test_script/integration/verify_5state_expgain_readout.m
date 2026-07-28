%VERIFY_5STATE_EXPGAIN_READOUT  Does the gain-readout ledger balance?
%
%   The gain readout inverts one equation:
%
%       sigma^2_{dh_r}  =  C_dpmr * 4kBT * a_h   +   C_n * sigma^2_n          (*)
%                          \___thermal___/           \__sensor__/
%       a_hm = ( sigma_hat^2_{dh_r} - C_n sigma^2_n ) / ( C_dpmr * 4kBT )
%
%   So a_hm is biased by exactly the amount by which (*) fails. Nothing in the
%   gain model, the state vector, or the choice between the differential and
%   algebraic formulations can change that -- which is why this is measured
%   first and directly, instead of being inferred from a_hat.
%
%   Three things are measured per regime:
%
%   1. LEDGER   measured Var(dh_r) versus the right-hand side of (*), evaluated
%      with the TRUE a_h. The ratio is the readout bias, with no estimator in
%      the path.
%
%   2. DETERMINISTIC vs RANDOM  dh_r is split using the fact that every seed
%      sees the SAME commanded trajectory: the across-seed mean at each time
%      index is the deterministic (trajectory-driven) component, the rest is
%      random. Equation (*) accounts only for the random part, so any
%      deterministic content is unmodelled variance that inflates a_hm.
%
%   3. SELF-CONSISTENCY  the same ledger with a_ctrl := a_true, which pins the
%      closed loop at the ideal one C_dpmr was derived for. If the bias
%      disappears the cause is a_hat feeding back into its own measurement; if
%      it survives, the constants themselves are wrong off the stationary point.
%
%   Windows: far hold (h_bar 22.2, stationary) / descent / oscillation /
%   near-wall hold (h_bar 2.0, stationary). Stationary versus moving is the
%   distinction (*) is blind to.

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

o = struct('lambda_c', cfg.lambda_c, 'option', 'A_MA2_full', ...
           'sigma2_n_s', cfg.meas_noise_std(:).^2, ...
           'kBT', pc.k_B*pc.T, 'd', 2, 'a_cov', cfg.a_cov, 'a_pd', cfg.a_pd, ...
           't_warmup_kf', 0, 'h_bar_safe', 1.5, 'iir_warmup_mode', 'prefill');
K   = build_eq17_6state_constants(o);
kBT = pc.k_B * pc.T;
s2n = cfg.meas_noise_std(az)^2;

lastwarn('');

for mode = {'a_ctrl = a_hat (production)', 'a_ctrl = a_true (loop pinned)'}
    if startsWith(mode{1}, 'a_ctrl = a_true'); aov = 'true'; else; aov = []; end

    nS = numel(seeds);
    for q = 1:nS
        s = run_5state_expgain(cfg, struct('seed', seeds(q), 'verbose', false, ...
                                           'variant', 'alg', 'a_ctrl_override', aov));
        if q == 1
            t = s.tout(:); N = numel(t);
            DR = zeros(N, nS); AT = zeros(N, nS); AM = zeros(N, nS);
            DHM = zeros(N, nS); HB = zeros(N, nS);
        end
        DR(:, q)  = s.dx_r_out(:, az);
        AT(:, q)  = s.a_true_out(:, az);
        AM(:, q)  = s.a_xm_out(:, az);
        DHM(:, q) = s.dh_m_out(:, az);
        HB(:, q)  = s.h_bar_true_out(:);
    end

    wins = { 'far hold  (stationary)', t > 0.10 & t <= 0.50; ...
             'descent   (moving)',     t > 0.60 & t <= 1.45; ...
             'oscillate (moving)',     t > 1.60 & t <= 3.50; ...
             'nearwall  (stationary)', t > 3.60 & t <= 4.80 };

    fprintf('\n================ %s ================\n', mode{1});
    fprintf('%-24s | %8s | %9s %9s %7s | %8s %8s\n', 'window', 'h_bar', ...
            'Var meas', 'Var pred', 'ratio', 'det frac', 'a_hm/aT');
    fprintf('%-24s | %8s | %9s %9s %7s | %8s %8s\n', '', '', ...
            '[nm^2]', '[nm^2]', '', '', '');
    for w = 1:size(wins, 1)
        m   = wins{w, 2};
        dr  = DR(m, :);                       % [um]
        det = mean(dr, 2);                    % across-seed mean = deterministic
        v_tot = mean(dr(:).^2);               % E[dh_r^2] (dh_r is already zero-mean-ish)
        v_det = mean(det.^2);
        a_bar = mean(mean(AT(m, :), 2));
        v_pred = K.C_dpmr * 4 * kBT * a_bar + K.C_n * s2n;
        fprintf('%-24s | %8.2f | %9.1f %9.1f %7.3f | %7.1f%% %8.4f\n', ...
                wins{w,1}, mean(mean(HB(m,:),2)), 1e6*v_tot, 1e6*v_pred, ...
                v_tot/v_pred, 100*v_det/v_tot, ...
                mean(mean(AM(m,:)./max(AT(m,:), eps), 2)));
    end
end

fprintf('\n---------------------------------------------------------------\n');
fprintf('ratio  = measured Var(dh_r) / the value equation (*) predicts from\n');
fprintf('         the TRUE gain. 1.000 means the readout equation is exact\n');
fprintf('         in that regime; it is what a_hm inverts, so a_hm inherits\n');
fprintf('         this ratio directly.\n');
fprintf('det frac = share of Var(dh_r) that is identical across seeds, i.e.\n');
fprintf('         driven by the commanded trajectory rather than by noise.\n');
fprintf('         Equation (*) has no term for it.\n');

[wmsg, wid] = lastwarn();
if isempty(wmsg)
    fprintf('\nWARNINGS: none\n');
else
    fprintf('\nWARNINGS: [%s] %s\n', wid, wmsg);
end
