%TEMP_DIAG_INIT_VS_PRIOR  Green vs blue in the convergence figure changed TWO
%   things at once (init value AND prior width). 2x2 to separate them.
clear; clc;
here = fileparts(mfilename('fullpath'));  addpath(genpath(fullfile(here,'..')));
pc = physical_constants();  az=3; seeds=[7 11 23 42 101 777];
base = user_config();
base.trajectory_type='positioning'; base.h_init=50; base.T_sim=12;
base.h_min=1.05*pc.R; base.ctrl_enable=true; base.thermal_enable=true;
base.meas_noise_enable=true; base.lambda_c=0.7; base.a_pd=0.05; base.a_cov=0.05;
base.meas_noise_std=[0.00062;0.00057;0.00331]; base.h_bar_safe=1.5;
fprintf('純定位 12 s, z, 6 seed.  a_hat/a_true 隨時間\n\n');
for fd=[0 1]
 fprintf('=== fdet %s ===\n', ternary(fd==0,'OFF','ON'));
 fprintf('%-12s %-9s | %7s %7s %7s %7s %7s\n','init','Pf_a', ...
         't=0','t=1','t=3','t=6','t=12');
 for asym=[0 1]
  for pfa=[0.01 0.06]
    c=base; c.a_init_asym=asym; c.a_init_honest=~asym;
    c.Pf_a_frac=pfa; c.fe_use_fdet=fd;
    V=zeros(6,5);
    for q=1:6
      s=temp_run_powerlaw_diag(c,struct('seed',seeds(q),'verbose',false));
      t=s.tout(:); r=s.a_hat_out(:,az)./s.a_true_out(:,az);
      tq=[0 1 3 6 12];
      for k=1:5; [~,i1]=min(abs(t-tq(k))); V(q,k)=r(i1); end
    end
    fprintf('%-12s %-9.2f | %7.4f %7.4f %7.4f %7.4f %7.4f\n', ...
      ternary(asym==0,'flat a_nom','asym'), pfa, ...
      mean(V(:,1)),mean(V(:,2)),mean(V(:,3)),mean(V(:,4)),mean(V(:,5)));
  end
 end
 fprintf('\n');
end
fprintf('比較橫列 = prior 的效果 ; 比較縱列 = init 值的效果\n');
function o=ternary(c,a,b); if c; o=a; else; o=b; end; end
