% SCRATCH | PURPOSE: 缺陷2 證據 — doc-vs-code C_dpmr constants audit | EXPIRES: Cdpmr_Cn_derivation.tex 重推完成後歸檔
%TEMP_DIAG_POWERLAW_CDPMR  (1) doc-vs-code constants audit, (2) attribute the
%   residual ~6% a_hat deficit that the whitened R2 exposed.
%   Test: a_ctrl_override = a_true makes the closed loop IDEAL, which is exactly
%   the assumption under which C_dpmr / C_n were derived. If a_hat/a_true -> 1,
%   the 6% is the self-referential a_xm inversion, not a Q or init problem.
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
az = 3;  seeds = [7 11 23 42 101 777];
lc = cfg.lambda_c;

% ---------- doc vs code constants ----------
o.lambda_c = lc;  o.option = 'A_MA2_full';
o.sigma2_n_s = (cfg.meas_noise_std(:)).^2;
pv = calc_simulation_params(cfg);  Pv = pv.Value;
o.kBT = Pv.ctrl.k_B * Pv.ctrl.T;  o.d = 2;
o.a_cov = cfg.a_cov;  o.a_pd = cfg.a_pd;
o.t_warmup_kf = 0;  o.h_bar_safe = 1.5;  o.iir_warmup_mode = 'prefill';
cc = build_eq17_6state_constants(o);

fprintf('=========== constants: 5state_powerlaw_hd.tex  vs  code ===========\n');
Cd_tex = 2 + 1/(1-lc^2);  Cn_tex = 2/(1+lc);
fprintf('%-14s %12s %12s %10s\n','quantity','tex (a_pd->0)','code (full a_pd)','ratio');
fprintf('%-14s %12.4f %12.4f %10.4f\n','C_dpmr', Cd_tex, cc.C_dpmr, Cd_tex/cc.C_dpmr);
fprintf('%-14s %12.4f %12.4f %10.4f\n','C_n',    Cn_tex, cc.C_n,    Cn_tex/cc.C_n);
xi_tex  = (Cn_tex/Cd_tex) * o.sigma2_n_s / (4*o.kBT);
fprintf('%-14s %12.4e %12.4e %10.4f   (z axis)\n','xi', xi_tex(3), cc.xi_per_axis(3), ...
        xi_tex(3)/cc.xi_per_axis(3));
fprintf('K_var (code) = %.6f  = 2*a_cov/(2-a_cov) = %.6f\n', cc.K_var, 2*o.a_cov/(2-o.a_cov));
fprintf('-> a_xm = (sig2 - C_n sig2_n)/(C_dpmr 4kBT): the tex constant would scale\n');
fprintf('   a_xm by %.4f (i.e. %+.1f %%). The CODE value is the defensible one\n', ...
        cc.C_dpmr/Cd_tex, 100*(cc.C_dpmr/Cd_tex - 1));
fprintf('   (full a_pd closed form); the .tex text is the stale one.\n');
a_nom = Pv.common.Ts / Pv.common.gamma_N;
fprintf('xi(z) = %.4f nm/pN = %.2f %% of a_h at h_bar=2 (%.2f nm/pN) -> thermal-dominated\n', ...
        1e3*cc.xi_per_axis(3), 100*cc.xi_per_axis(3)/(a_nom/2.1248), 1e3*a_nom/2.1248);

% ---------- attribution of the residual a_hat deficit ----------
fprintf('\n=========== a_hat deficit attribution (whitened R2, honest Pf_p_std) ===========\n');
fprintf('%-30s | %9s %9s | %9s %9s | %7s\n', 'arm', 'a_hat/aT','sc','p_osc','sc','trk z');
arms = { 'baseline  (a_ctrl = a_hat)', false; ...
         'ideal loop(a_ctrl = a_true)', true };
for r = 1:2
    c = cfg;  c.y2_whiten = 1;  c.Pf_p_std = 0.03;  c.a_ctrl_true = arms{r,2};
    ar = zeros(6,1); po = zeros(6,1); tk = zeros(6,1);
    for q = 1:6
        s = temp_run_powerlaw_diag(c, struct('seed', seeds(q), 'verbose', false));
        tt = s.tout(:);  mo = tt > 1.6;
        e = s.p_d_out(2:end,:) - s.p_true_out(1:end-1,:);  te = tt(2:end);
        ar(q) = mean(s.a_hat_out(mo,az) ./ max(s.a_true_out(mo,az), eps));
        po(q) = mean(s.p_hat_out(mo,az));
        tk(q) = 1e3*std(e(te>1.6,az));
    end
    fprintf('%-30s | %9.4f %9.4f | %9.4f %9.4f | %7.2f\n', arms{r,1}, ...
            mean(ar), std(ar), mean(po), std(po), mean(tk));
end
fprintf('\nif the ideal-loop arm goes to ~1.00, the residual deficit is the\n');
fprintf('self-referential a_xm inversion (C_dpmr assumes a_hat = a_true), NOT Q/init.\n');
