%TEMP_DIAG_6PCT_POSITIONING  Root-cause probe for the ~6% a_hat deficit (z).
%   Test bed: PURE POSITIONING at the far field (Delta_h_d = 0), so no descent,
%   no power-law sweep, p_hat frozen -- only the F_dh / y1 channel is alive.
%
%   Three competing hypotheses, each with a falsifiable prediction:
%     H1 endogeneity via n_h : F_dh contains (1-lc)*delta_h_m/a_hat, and the same
%        n_h sits in eps_h -> regressor correlated with residual (errors-in-vars,
%        attenuating).  PREDICTS: bias grows with sigma_n.
%     H2 endogeneity via e_ah: F_dh ~ 1/a_hat = 1/(a_h - e_ah), regressor
%        correlated with the parameter being estimated.
%        PREDICTS: bias grows with (1-lambda_c) (both paths carry that factor).
%     H3 P44 collapse       : far field a' ~ 0 -> Q44 ~ 0 -> a_h has almost no
%        process noise -> P44 -> 0 and a_hat freezes wherever it happens to be.
%        PREDICTS: bias is set early and does NOT improve with run length;
%                  sqrt(P44) at the end is tiny.
%   If none of the sweeps move the bias, all three are wrong.
%   Scratch; delete with the rest of this round's temp files.

clear; clc;
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..');
addpath(genpath(proj));

base = user_config();
base.trajectory_type = 'positioning';      % hold at h_init, Delta_h_d = 0
base.h_init = 50;                          % h_bar = 22.2, far field
base.T_sim  = 5.0;
pc = physical_constants();
base.h_min = 1.05 * pc.R;
base.ctrl_enable = true;  base.thermal_enable = true;  base.meas_noise_enable = true;
base.lambda_c = 0.7;  base.a_pd = 0.05;  base.a_cov = 0.05;
base.h_bar_safe = 1.5;  base.Pf_a_frac = 0.03;
sn_nom = [0.00062; 0.00057; 0.00331];
base.meas_noise_std = sn_nom;
az = 3;  seeds = [7 11 23 42 101 777];

    function [ar, ax, sp, e1, e5] = run_arm(c, seeds, az)
        ar=zeros(6,1); ax=zeros(6,1); sp=zeros(6,1); e1=zeros(6,1); e5=zeros(6,1);
        for q = 1:6
            s = temp_run_powerlaw_diag(c, struct('seed', seeds(q), 'verbose', false));
            tt = s.tout(:);  ss = tt > 1.0;              % steady window
            ar(q) = mean(s.a_hat_out(ss,az)./max(s.a_true_out(ss,az),eps));
            ax(q) = mean(s.a_xm_out(ss,az))/mean(s.a_true_out(ss,az));
            sp(q) = s.P_a_out(end,az)/mean(s.a_true_out(ss,az));   % sqrt(P44)/a
            [~,i1] = min(abs(tt-1.0));  [~,i5] = min(abs(tt-5.0));
            e1(q) = s.a_hat_out(i1,az)/s.a_true_out(i1,az);
            e5(q) = s.a_hat_out(i5,az)/s.a_true_out(i5,az);
        end
    end

fprintf('PURE POSITIONING at h_bar = %.1f, T = %.1f s, 6 seeds, z axis\n\n', ...
        base.h_init/pc.R, base.T_sim);

fprintf('=== H1: sigma_n sweep (nominal z = %.2f nm) ===\n', 1e3*sn_nom(3));
fprintf('%10s | %10s %8s | %10s | %10s | %s\n','sigma_n x','a_hat/aT','sc', ...
        'a_xm/aT','sqrtP44/a','a_hat@1s -> @5s');
for k = [0.05 0.25 0.5 1 2 4]
    c = base;  c.meas_noise_std = k * sn_nom;
    [ar, ax, sp, e1, e5] = run_arm(c, seeds, az);
    fprintf('%10.2f | %10.4f %8.4f | %10.4f | %10.2e | %.4f -> %.4f\n', ...
            k, mean(ar), std(ar), mean(ax), mean(sp), mean(e1), mean(e5));
end

fprintf('\n=== H2: lambda_c sweep (sigma_n nominal) ===\n');
fprintf('%10s | %10s %8s | %10s | %10s | %s\n','lambda_c','a_hat/aT','sc', ...
        'a_xm/aT','sqrtP44/a','a_hat@1s -> @5s');
for lc = [0.5 0.7 0.9]
    c = base;  c.lambda_c = lc;
    [ar, ax, sp, e1, e5] = run_arm(c, seeds, az);
    fprintf('%10.2f | %10.4f %8.4f | %10.4f | %10.2e | %.4f -> %.4f\n', ...
            lc, mean(ar), std(ar), mean(ax), mean(sp), mean(e1), mean(e5));
end

fprintf('\n=== H3: does a longer run help? (freeze test) ===\n');
fprintf('%10s | %10s %8s | %10s\n','T_sim [s]','a_hat/aT','sc','sqrtP44/a');
for T = [2 5 12]
    c = base;  c.T_sim = T;
    [ar, ~, sp] = run_arm(c, seeds, az);
    fprintf('%10.1f | %10.4f %8.4f | %10.2e\n', T, mean(ar), std(ar), mean(sp));
end

fprintf('\n--- 判讀 ---\n');
fprintf('H1 真 -> a_hat/aT 隨 sigma_n 單調變差\n');
fprintf('H2 真 -> a_hat/aT 隨 lambda_c 上升而改善 (1-lc 變小)\n');
fprintf('H3 真 -> 拉長 T_sim 不改善, 且 @1s 與 @5s 幾乎相同, sqrtP44/a 極小\n');
fprintf('全部不動 -> 三個假設都錯\n');
