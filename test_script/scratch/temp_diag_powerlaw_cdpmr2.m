% SCRATCH | PURPOSE: 缺陷2 證據 — C_dpmr_true 反算 vs code/tex | EXPIRES: Cdpmr_Cn_derivation.tex 重推完成後歸檔
%TEMP_DIAG_POWERLAW_CDPMR2  Back out C_dpmr from the data.
%   a_xm = (sig2_dxr - C_n sig2_n)/(C_dpmr_code * 4kBT), and in the thermal-
%   dominated regime E[a_xm] = a_h * C_dpmr_true/C_dpmr_code.
%   So  mean(a_xm)/mean(a_true)  IS the ratio C_dpmr_true/C_dpmr_code.
%   Measured in the IDEAL-loop arm (a_ctrl = a_true), where the closed loop is
%   exactly the one C_dpmr was derived for -> a clean test of the constant.
%   Scratch; delete after the diagnosis.

clear; clc;
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(proj));

cfg = user_config();
cfg.trajectory_type = 'osc';
cfg.h_init = 50;  cfg.h_bottom = 4.5;  cfg.amplitude = 2.5;
cfg.frequency = 1;  cfg.n_cycles = 2;
cfg.t_hold = 0.5;  cfg.t_descend_override = 1.0;  cfg.T_sim = 4.8;
pc = physical_constants();
cfg.h_min = 1.05 * pc.R;
cfg.ctrl_enable = true;  cfg.thermal_enable = true;  cfg.meas_noise_enable = true;
cfg.lambda_c = 0.7;  cfg.a_pd = 0.05;  cfg.a_cov = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
cfg.h_bar_safe = 1.5;  cfg.Pf_a_frac = 0.03;
seeds = [7 11 23 42 101 777];
lc = cfg.lambda_c;

fprintf('C_dpmr_code = 3.1610 (full a_pd) ; C_dpmr_tex = %.4f (a_pd->0)\n', 2+1/(1-lc^2));
fprintf('mean(a_xm)/mean(a_true) = C_dpmr_true / C_dpmr_code\n\n');
fprintf('%-28s | %-6s | %8s %8s %8s\n','arm','axis','ratio','sc','implied C_dpmr_true');
for r = 1:2
    c = cfg;  c.y2_whiten = 1;  c.Pf_p_std = 0.03;  c.a_ctrl_true = (r == 2);
    nm = {'baseline (a_ctrl=a_hat)','ideal    (a_ctrl=a_true)'};
    Rt = zeros(6,3);
    for q = 1:6
        s = temp_run_powerlaw_diag(c, struct('seed', seeds(q), 'verbose', false));
        tt = s.tout(:);  mo = tt > 1.6;
        for ax = 1:3
            Rt(q,ax) = mean(s.a_xm_out(mo,ax)) / mean(s.a_true_out(mo,ax));
        end
    end
    an = {'x','y','z'};
    for ax = 1:3
        fprintf('%-28s | %-6s | %8.4f %8.4f %8.4f\n', ...
                ternary(ax==1, nm{r}, ''), an{ax}, mean(Rt(:,ax)), std(Rt(:,ax)), ...
                3.1610*mean(Rt(:,ax)));
    end
end
fprintf('\nreference: eq17 epsilon_h = a_h f_T[k] + (1-lc) sum_{i=1..d} a_h f_T[k-i] + (1-lc) n_h\n');
fprintf('   Var(eps_h)/(a_h^2 sig2_fT) = 1 + d(1-lc)^2 = %.4f  (d=2, lc=%.2f)\n', ...
        1 + 2*(1-lc)^2, lc);

function o = ternary(c,a,b); if c; o=a; else; o=b; end; end
