%TEMP_DIAG_6PCT_DRIFT  Is the (1-lc)*a'*delta_h3_hat term the drift source?
%   Pure positioning, far field. drift_off = 1 removes that single term from the
%   a_h prediction. Also reports mean(delta_h3_hat) -- the term can only drift
%   a_hat if that mean is non-zero.
%   Scratch; delete with the rest of this round's temp files.
clear; clc;
here = fileparts(mfilename('fullpath'));  addpath(genpath(fullfile(here,'..')));
base = user_config();
base.trajectory_type = 'positioning';  base.h_init = 50;
pc = physical_constants();  base.h_min = 1.05*pc.R;
base.ctrl_enable = true; base.thermal_enable = true; base.meas_noise_enable = true;
base.lambda_c = 0.7; base.a_pd = 0.05; base.a_cov = 0.05;
base.meas_noise_std = [0.00062;0.00057;0.00331];
base.h_bar_safe = 1.5; base.Pf_a_frac = 0.03;
az = 3; seeds = [7 11 23 42 101 777];
fprintf('PURE POSITIONING h_bar=22.2, 6 seeds, z axis\n\n');
fprintf('%-14s %6s | %10s %8s | %12s %12s | %10s\n','arm','T[s]', ...
        'a_hat/aT','sc','mean dh3_hat','mean dh3 true','sqrtP44/a');
for dof = [0 1]
  for T = [2 5 12]
    c = base; c.T_sim = T; c.drift_off = dof;
    ar=zeros(6,1); d3=zeros(6,1); dt=zeros(6,1); sp=zeros(6,1);
    for q=1:6
        s = temp_run_powerlaw_diag(c, struct('seed',seeds(q),'verbose',false));
        tt=s.tout(:); ss=tt>1.0;
        ar(q)=mean(s.a_hat_out(ss,az)./max(s.a_true_out(ss,az),eps));
        d3(q)=1e3*mean(s.DG.delta_x_hat_3(ss,az));
        e = s.p_d_out(2:end,:)-s.p_true_out(1:end-1,:); te=tt(2:end);
        dt(q)=1e3*mean(e(te>1.0,az));
        sp(q)=s.P_a_out(end,az)/mean(s.a_true_out(ss,az));
    end
    fprintf('%-14s %6.1f | %10.4f %8.4f | %9.3f nm %9.3f nm | %10.2e\n', ...
        ternary(dof==0,'drift ON','drift OFF'), T, mean(ar), std(ar), ...
        mean(d3), mean(dt), mean(sp));
  end
end
fprintf('\ndrift OFF 若不再隨 T 惡化 -> 那一項就是漂移源\n');
function o=ternary(c,a,b); if c; o=a; else; o=b; end; end
