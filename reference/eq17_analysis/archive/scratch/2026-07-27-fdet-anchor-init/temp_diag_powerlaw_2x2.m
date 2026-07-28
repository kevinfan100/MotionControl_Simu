%TEMP_DIAG_POWERLAW_2X2  Separate the contributions of the two A-level fixes.
%   Fix 1 = whitened y2 + matching R2      (y2_whiten)
%   Fix 2 = honest p prior 0.3 -> 0.025    (Pf_p_std)
%   Full 2x2 so the attribution is measured, not inferred. 6 seeds per arm.
%   Scratch; delete with the rest of this round's temp files.

clear; clc;
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..');
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

fprintf('%-30s | %9s | %8s %8s %9s %7s | %8s | %7s %6s\n', 'arm', 'p_desc_sd', ...
        'p_osc','sc','sig_p_end','honest','a_hat/aT','trk z','clamp');
fprintf('%s\n', repmat('-', 1, 108));
arms = { 'neither (as shipped)',      0, 0.30; ...
         'fix 2 only (prior 0.025)',  0, 0.025; ...
         'fix 1 only (whitened y2)',  1, 0.30; ...
         'BOTH  = production',        1, 0.025 };
for r = 1:4
    c = cfg;  c.y2_whiten = arms{r,2};  c.Pf_p_std = arms{r,3};
    pd_sd=zeros(6,1); po=zeros(6,1); sg=zeros(6,1); ar=zeros(6,1); tk=zeros(6,1); cl=0;
    for q = 1:6
        s = temp_run_powerlaw_diag(c, struct('seed', seeds(q), 'verbose', false));
        tt = s.tout(:);  ph = s.p_hat_out(:,az);
        md = tt>0.5 & tt<1.5;  mo = tt>1.6;
        e = s.p_d_out(2:end,:) - s.p_true_out(1:end-1,:);  te = tt(2:end);
        pd_sd(q)=std(ph(md)); po(q)=mean(ph(mo)); sg(q)=s.P_p_out(end,az);
        ar(q)=mean(s.a_hat_out(mo,az)./max(s.a_true_out(mo,az),eps));
        tk(q)=1e3*std(e(te>1.6,az));
        cl = cl + any(ph <= 1e-9 | ph >= 5-1e-9);
    end
    fprintf('%-30s | %9.4f | %8.4f %8.4f %9.4f %7.1f | %8.4f | %7.2f %6d\n', ...
        arms{r,1}, mean(pd_sd), mean(po), std(po), mean(sg), ...
        std(po)/max(mean(sg),eps), mean(ar), mean(tk), cl);
end
fprintf('%s\n', repmat('-', 1, 108));
fprintf('true p_eff(z) over the trajectory range: mean 1.003 ; honest = scatter/claimed (1 = honest)\n');
fprintf('NOTE: fix 2 without fix 1 still leaks a_cov into every result -- the number\n');
fprintf('      may look fine but it is a tightened prior hiding a wrong R2.\n');
