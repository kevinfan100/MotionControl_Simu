% SCRATCH | PURPOSE: 缺陷2 證據 — V1-V7 否證測試 (V5 = sigma2_dxr ledger) | EXPIRES: Cdpmr_Cn_derivation.tex 重推完成後歸檔
%TEMP_VERIFY_POWERLAW_CLAIMS  Falsification tests for the four A-level claims.
%   Each test is stated so that a FAIL kills the claim.
%
%   V1  a_xm[k] = (1-a_cov)a_xm[k-1] + a_cov u[k] to machine precision?
%   V2  ACF of (a_xm - a_true) geometric with pole 1-a_cov = 0.95?
%   V3  Var(a_xm - a_true) ~= R2 the filter actually uses?
%       (claim: R2 gets the MARGINAL VARIANCE right, the INDEPENDENCE wrong.
%        if V3 fails, the story is different -- e.g. R2 is just too small)
%   V4  whitened increment actually white (short ACF) and its variance equal to
%       the derived R2' = a_cov(2-a_cov)*R2_intrinsic?
%   V5  empirical sigma2_dxr vs C_dpmr(code) vs C_dpmr(tex) -- which constant?
%   V6  power-law with p=1 vs the TRUE da/dh_bar: max rel error = the honest
%       prior width for p.
%   V7  honest a_h[0] (no c peek) with Pf_a_frac = 0.03: is it visibly slow?
%
%   Stationary test bed = the HOLD window (t < 0.5): a_true constant, no sweep.
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
az = 3;  lc = cfg.lambda_c;  acv = cfg.a_cov;
seeds = [7 11 23 42 101 777];

s = temp_run_powerlaw_diag(cfg, struct('seed', 7, 'verbose', false));
Pv = s.meta.params_value;
kBT = Pv.ctrl.k_B * Pv.ctrl.T;  a_nom = Pv.common.Ts / Pv.common.gamma_N;
t = s.tout(:);  hold_m = t > 0.05 & t < 0.5;      % skip the very first steps
axm = s.a_xm_out(:,az);  aT = s.a_true_out(:,az);
cc = s.ctrl_const;

fprintf('=============== V1: is a_xm exactly AR(1) with pole 1-a_cov? ===============\n');
% k>=4: the first call returns an all-zero diag (no a_xm / dx_r yet)
kk = 4:numel(axm);
lhs = axm(kk);
rhs = (1-acv)*axm(kk-1) + acv * ...
      ((s.DG.dx_r(kk,az).^2 - cc.C_n*cfg.meas_noise_std(az)^2) / (cc.C_dpmr*4*kBT));
resid = lhs - rhs;
fprintf('max |a_xm[k] - [(1-a_cov)a_xm[k-1] + a_cov*u[k]]| = %.3e\n', max(abs(resid)));
fprintf('   relative to std(a_xm) = %.3e   -> %s\n', max(abs(resid))/std(axm), ...
        ternary(max(abs(resid))/std(axm) < 1e-9, 'IDENTITY HOLDS (V1 PASS)', 'V1 FAIL'));

fprintf('\n=============== V2: ACF of (a_xm - a_true), hold window ===============\n');
err = axm(hold_m) - aT(hold_m);  err = err - mean(err);
L = 120;  [ac, lg] = xcorr(err, L, 'coeff');
lg = lg(:);  ac = ac(:);  ac = ac(lg >= 0);  lg = lg(lg >= 0);
fprintf('%6s %10s %12s %10s\n','lag','measured','0.95^lag','ratio');
for Lq = [1 3 5 10 15 20 30]
    fprintf('%6d %10.4f %12.4f %10.3f\n', Lq, ac(lg==Lq), (1-acv)^Lq, ...
            ac(lg==Lq)/(1-acv)^Lq);
end
% fit the pole over lags 5..25 (short lags carry the MA(2) colour of dx_r^2;
% lags > ~30 are killed by the finite 0.45 s hold window, ACF -> negative)
sel = lg >= 5 & lg <= 25 & ac > 0;
cf = polyfit(lg(sel), log(ac(sel)), 1);
pfit = exp(cf(1));
fprintf('fitted pole over lags 5-25 = %.4f   (predicted 1-a_cov = %.4f)  -> %s\n', ...
        pfit, 1-acv, ternary(abs(pfit-(1-acv)) < 0.02, 'V2 PASS', 'V2 FAIL'));
tau_geom = (1+pfit)/(1-pfit);
fprintf('geometric ACF with that pole gives 1+2*sum(rho) = (1+phi)/(1-phi) = %.1f\n', tau_geom);
fprintf('   predicted over-count (2-a_cov)/a_cov = %.1f  -> ratio %.3f\n', ...
        (2-acv)/acv, tau_geom/((2-acv)/acv));

fprintf('\n=============== V3: does R2 match the MARGINAL variance of a_xm err? ===============\n');
R2_used = mean(s.DG.dg_R2(hold_m,az));
fprintf('empirical Var(a_xm - a_true) = %.4e\n', var(err));
fprintf('R2 the filter used (mean)     = %.4e\n', R2_used);
fprintf('ratio = %.3f  -> %s\n', var(err)/R2_used, ...
        ternary(abs(log(var(err)/R2_used)) < log(1.6), ...
        'MARGINAL VARIANCE IS RIGHT (V3 PASS: the bug is independence, not magnitude)', ...
        'V3 FAIL: R2 magnitude itself is off, claim needs revising'));

fprintf('\n=============== V4: whitened increment -- white? correct variance? ===============\n');
y2w = axm(2:end) - (1-acv)*axm(1:end-1);
tw = t(2:end);  hw = tw > 0.05 & tw < 0.5;
ew = y2w(hw) - mean(y2w(hw));
[aw, lw] = xcorr(ew, 20, 'coeff');
lw = lw(:);  aw = aw(:);  aw = aw(lw>=0);  lw = lw(lw>=0);
fprintf('ACF of whitened increment: ');
for Lq = 1:6; fprintf('rho(%d)=%+.3f  ', Lq, aw(lw==Lq)); end
fprintf('\n1+2*sum(rho, lag1-20) = %.2f  (before whitening the pole implied %.1f)\n', ...
        1+2*sum(aw(lw>=1)), tau_geom);
IF_e = if_eff_local(cc.IF_abc, cc.C_dpmr, cc.C_n, kBT, mean(aT(hold_m)), cfg.meas_noise_std(az)^2);
R2_intr = cc.K_var * IF_e * (mean(aT(hold_m)) + cc.xi_per_axis(az))^2;
R2w_pred = acv*(2-acv) * R2_intr;
fprintf('Var(y2'') empirical = %.4e ; derived R2'' = a_cov(2-a_cov)*R2_intr = %.4e ; ratio %.3f\n', ...
        var(ew), R2w_pred, var(ew)/R2w_pred);
fprintf('-> %s\n', ternary(1+2*sum(aw(lw>=1)) < 3 && abs(log(var(ew)/R2w_pred)) < log(1.6), ...
        'V4 PASS (whitening works and R2'' formula is right)', 'V4 PARTIAL/FAIL - see numbers'));

fprintf('\n=============== V5: which C_dpmr does the data support? ===============\n');
sig2_emp = mean(s.DG.sigma2_dxr_hat(hold_m,az));
a_h_true = mean(aT(hold_m));  s2n = cfg.meas_noise_std(az)^2;
Cd_code = cc.C_dpmr;  Cn_code = cc.C_n;
Cd_tex = 2 + 1/(1-lc^2);  Cn_tex = 2/(1+lc);
fprintf('empirical sigma2_dxr (EWMA mean, hold) = %.4e um^2\n', sig2_emp);
fprintf('  code form: %.4f*4kBT*a + %.4f*sig2_n = %.4e   ratio %.4f\n', Cd_code, Cn_code, ...
        Cd_code*4*kBT*a_h_true + Cn_code*s2n, sig2_emp/(Cd_code*4*kBT*a_h_true + Cn_code*s2n));
fprintf('  tex  form: %.4f*4kBT*a + %.4f*sig2_n = %.4e   ratio %.4f\n', Cd_tex, Cn_tex, ...
        Cd_tex*4*kBT*a_h_true + Cn_tex*s2n, sig2_emp/(Cd_tex*4*kBT*a_h_true + Cn_tex*s2n));
r_code = sig2_emp/(Cd_code*4*kBT*a_h_true + Cn_code*s2n);
r_tex  = sig2_emp/(Cd_tex*4*kBT*a_h_true + Cn_tex*s2n);
fprintf('-> %s\n', ternary(abs(r_code-1) < abs(r_tex-1), ...
        'CODE form closer (V5 PASS: .tex is the stale one)', 'V5 FAIL: tex form closer'));

fprintf('\n=============== V6: power-law with p=1 vs the true slope ===============\n');
hbg = linspace(2.0, 22.22, 500);
relerr_perp = zeros(size(hbg));  relerr_para = zeros(size(hbg));
for i = 1:numel(hbg)
    [cpa, cpe, dv] = calc_correction_functions(hbg(i), true);
    a_pe = a_nom/cpe;  a_pa = a_nom/cpa;
    sh_true_pe = -a_nom*dv.dc_perp_dh/cpe^2;
    sh_true_pa = -a_nom*dv.dc_para_dh/cpa^2;
    sh_mod_pe = (1/(hbg(i)-1)) * a_pe*(a_nom-a_pe)/a_nom;   % p = 1
    sh_mod_pa = (1/(hbg(i)-1)) * a_pa*(a_nom-a_pa)/a_nom;
    relerr_perp(i) = sh_mod_pe/sh_true_pe - 1;
    relerr_para(i) = sh_mod_pa/sh_true_pa - 1;
end
fprintf('slope error of the p=1 power law over h_bar in [2, 22.2]:\n');
fprintf('   z   (perp): max %+.4f %%  rms %.4f %%\n', 100*max(abs(relerr_perp)), ...
        100*rms(relerr_perp));
fprintf('   x/y (para): max %+.4f %%  rms %.4f %%\n', 100*max(abs(relerr_para)), ...
        100*rms(relerr_para));
fprintf('-> honest prior on p (z) = %.3f ; current Pf_p_std = 0.300 -> %.0fx too loose : %s\n', ...
        max(abs(relerr_perp)), 0.3/max(abs(relerr_perp)), ...
        ternary(0.3/max(abs(relerr_perp)) > 5, 'V6 PASS', 'V6 FAIL'));

fprintf('\n=============== V7: honest a_h[0] (no c peek) x Pf_a_frac ===============\n');
fprintf('%-26s | %10s %10s | %9s %8s\n','arm','a_hat err@t=0.1','settle t[s]','a_hat/aT','trk z');
for r = 1:3
    c = cfg;  c.y2_whiten = 1;  c.Pf_p_std = 0.03;
    switch r
        case 1, c.a_init_honest = 0; c.Pf_a_frac = 0.03; nm = 'peek     + Pf_a 0.03';
        case 2, c.a_init_honest = 1; c.Pf_a_frac = 0.03; nm = 'HONEST   + Pf_a 0.03';
        case 3, c.a_init_honest = 1; c.Pf_a_frac = 0.06; nm = 'HONEST   + Pf_a 0.06';
    end
    e0 = zeros(6,1); st = zeros(6,1); ar = zeros(6,1); tk = zeros(6,1);
    for q = 1:6
        ss = temp_run_powerlaw_diag(c, struct('seed', seeds(q), 'verbose', false));
        tt = ss.tout(:);  rel = (ss.a_hat_out(:,az)-ss.a_true_out(:,az))./ss.a_true_out(:,az);
        [~,i1] = min(abs(tt-0.1));  e0(q) = 100*rel(i1);
        k1 = find(abs(rel) < 0.02 & tt < 0.5, 1, 'first');
        if isempty(k1); st(q) = NaN; else; st(q) = tt(k1); end
        mo = tt > 1.6;  ar(q) = mean(ss.a_hat_out(mo,az)./max(ss.a_true_out(mo,az),eps));
        e = ss.p_d_out(2:end,:) - ss.p_true_out(1:end-1,:);  te = tt(2:end);
        tk(q) = 1e3*std(e(te>1.6,az));
    end
    fprintf('%-26s | %10.2f%% %10.3f | %9.4f %8.2f\n', nm, mean(e0), ...
            mean(st,'omitnan'), mean(ar), mean(tk));
end
fprintf('(settle t = first time |a_hat err| < 2%% during the hold)\n');

function IF = if_eff_local(IF_abc, C_dpmr, C_n, kBT, a, s2n)
    sxT = 4*kBT*a;
    num = sxT^2*IF_abc(1) + 2*sxT*s2n*IF_abc(2) + s2n^2*IF_abc(3);
    den = (C_dpmr*sxT + C_n*s2n)^2;
    IF = 1 + 2*num/den;
end
function o = ternary(c,a,b); if c; o=a; else; o=b; end; end
