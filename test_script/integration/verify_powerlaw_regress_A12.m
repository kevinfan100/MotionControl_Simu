%VERIFY_POWERLAW_REGRESS_A12  Regression for A-level fixes 1 (whitened y2 / R2)
%   and 2 (Pf_p_std prior) on the PRODUCTION controller
%   (motion_control_law_5state_powerlaw, via run_5state_powerlaw).
%
%   Acceptance:
%     1. a_cov-invariance: every reported metric must be flat across a 10x change
%        in a_cov (a_cov is an internal IIR parameter; it must not leak).
%     2. p_hat: descent scatter << the pre-fix 0.29, no clamp hits, p_osc near the
%        true p_eff ~ 1.003 (z).
%     3. No regression in tracking or run health vs the pre-fix baseline
%        (pre-fix: trk z 24.25 nm, p_desc_sd 0.2921, p_osc 0.9765+-0.1587,
%         a_hat/aT 1.0164, honesty 7.9).
%   Scratch; delete after the fix is accepted.

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
cfg.lambda_c = 0.7;  cfg.a_pd = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
cfg.h_bar_safe = 1.5;  cfg.Pf_a_frac = 0.03;
% Pf_p_std intentionally NOT set -> exercise the new 0.025 default
az = 3;  seeds = [7 11 23 42 101 777];

fprintf('PRODUCTION path: motion_control_law_5state_powerlaw (default Pf_p_std)\n\n');
fprintf('%8s | %9s | %8s %8s %9s %7s | %8s %8s | %7s %6s %5s\n', 'a_cov', ...
        'p_desc_sd','p_osc','sc','sig_p_end','honest','a_hat/aT','sc','trk z', ...
        'clamp','NaN');
M = [];
lastwarn('');
for acv = [0.02 0.05 0.10 0.20]
    c = cfg;  c.a_cov = acv;
    pd_sd=zeros(6,1); po=zeros(6,1); sg=zeros(6,1); ar=zeros(6,1); tk=zeros(6,1);
    cl = 0;  nn = 0;
    for q = 1:6
        s = run_5state_powerlaw(c, struct('seed', seeds(q), 'verbose', false));
        tt = s.tout(:);  ph = s.p_hat_out(:,az);
        md = tt>0.5 & tt<1.5;  mo = tt>1.6;
        e = s.p_d_out(2:end,:) - s.p_true_out(1:end-1,:);  te = tt(2:end);
        pd_sd(q)=std(ph(md)); po(q)=mean(ph(mo)); sg(q)=s.P_p_out(end,az);
        ar(q)=mean(s.a_hat_out(mo,az)./max(s.a_true_out(mo,az),eps));
        tk(q)=1e3*std(e(te>1.6,az));
        cl = cl + any(ph <= 1e-9 | ph >= 5-1e-9);
        nn = nn + (any(~isfinite(s.a_hat_out(:))) || any(~isfinite(s.p_true_out(:))));
    end
    fprintf('%8.2f | %9.4f | %8.4f %8.4f %9.4f %7.1f | %8.4f %8.4f | %7.2f %6d %5d\n', ...
        acv, mean(pd_sd), mean(po), std(po), mean(sg), std(po)/max(mean(sg),eps), ...
        mean(ar), std(ar), mean(tk), cl, nn);
    M = [M; mean(pd_sd) mean(po) std(po) mean(ar) mean(tk)]; %#ok<AGROW>
end

fprintf('\n--- acceptance ---\n');
spread = (max(M,[],1) - min(M,[],1)) ./ max(abs(mean(M,1)), eps);
nmv = {'p_desc_sd','p_osc','p_osc scatter','a_hat/aT','trk z'};
for j = 1:5
    fprintf('%-16s : spread across a_cov = %6.2f %%  -> %s\n', nmv{j}, 100*spread(j), ...
            ternary(spread(j) < 0.05, 'INVARIANT', 'LEAKS'));
end
[wm, wid] = lastwarn();
if isempty(wm)
    fprintf('warnings: none\n');
else
    fprintf('warnings: [%s] %s\n', wid, wm);
end
fprintf('\npre-fix baseline for comparison (a_cov=0.05, Pf_p_std=0.3):\n');
fprintf('   p_desc_sd 0.2921 | p_osc 0.9765 +- 0.1587 | honesty 7.9 | a_hat/aT 1.0164 | trk 24.25\n');
fprintf('true p_eff(z) over the trajectory range: 0.976 .. 1.006 (mean 1.003)\n');

function o = ternary(c,a,b); if c; o=a; else; o=b; end; end
