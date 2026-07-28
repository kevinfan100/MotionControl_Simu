%TEMP_DIAG_FDET_HONESTINIT  The test that actually matters:
%   with fdet ON the y1->a_h channel is dead in positioning, so ONLY y2 can
%   remove an init error. Under the PEEK init a_h[0] is already exact, so the
%   earlier "0.998" proved nothing about identification -- it only proved the
%   drift stopped. Re-run with realizable inits and watch CONVERGENCE.
%     peek : a_nom/c(hbar_0)          exact          (cheat)
%     flat : a_nom                    +5.33% error   (honest, coarse)
%     asym : a_nom/(1+(9/8)/hbar_0)   +0.25% error   (honest, far-field asymptote)
clear; clc;
here = fileparts(mfilename('fullpath'));  addpath(genpath(fullfile(here,'..')));
pc = physical_constants();  az=3; seeds=[7 11 23 42 101 777];
base = user_config();
base.h_min=1.05*pc.R; base.ctrl_enable=true; base.thermal_enable=true;
base.meas_noise_enable=true; base.lambda_c=0.7; base.a_pd=0.05; base.a_cov=0.05;
base.meas_noise_std=[0.00062;0.00057;0.00331]; base.h_bar_safe=1.5;

inits = { 'peek (作弊)', 0,0,0.03; 'flat a_nom', 1,0,0.06; 'asym (9/8)/hbar', 0,1,0.01 };

fprintf('=== A. 純定位 12 s : 能不能把 init 誤差拉回來 ===\n');
pos=base; pos.trajectory_type='positioning'; pos.h_init=50; pos.T_sim=12;
fprintf('%-17s %-9s | %7s %7s %7s %7s %7s | %8s %8s\n','init','fdet', ...
        't=0','t=1','t=3','t=6','t=12','y1 %%','y2 %%');
aTn = 13.962;
for ii=1:3
  for fd=[0 1]
    c=pos; c.a_init_honest=inits{ii,2}; c.a_init_asym=inits{ii,3};
    c.Pf_a_frac=inits{ii,4}; c.fe_use_fdet=fd;
    V=zeros(6,5); y1=zeros(6,1); y2=zeros(6,1);
    for q=1:6
      s=temp_run_powerlaw_diag(c,struct('seed',seeds(q),'verbose',false));
      t=s.tout(:); r=s.a_hat_out(:,az)./s.a_true_out(:,az);
      for k=1:5
        tq=[0 1 3 6 12]; [~,i1]=min(abs(t-tq(k))); V(q,k)=r(i1);
      end
      y1(q)=1e3*sum(s.DG.dg_dA_y1(:,az)); y2(q)=1e3*sum(s.DG.dg_dA_y2(:,az));
    end
    fprintf('%-17s %-9s | %7.4f %7.4f %7.4f %7.4f %7.4f | %+7.1f %+7.1f\n', ...
      ternary(fd==0,inits{ii,1},''), ternary(fd==0,'OFF','ON'), ...
      mean(V(:,1)),mean(V(:,2)),mean(V(:,3)),mean(V(:,4)),mean(V(:,5)), ...
      100*mean(y1)/aTn, 100*mean(y2)/aTn);
  end
end

fprintf('\n=== B. 振盪場景 4.8 s (有確定性激勵) ===\n');
osc=base; osc.trajectory_type='osc'; osc.h_init=50; osc.h_bottom=4.5;
osc.amplitude=2.5; osc.frequency=1; osc.n_cycles=2; osc.t_hold=0.5;
osc.t_descend_override=1.0; osc.T_sim=4.8;
fprintf('%-17s %-9s | %10s %8s | %8s\n','init','fdet','a_hat/aT z','sc','trk z nm');
for ii=1:3
  for fd=[0 1]
    c=osc; c.a_init_honest=inits{ii,2}; c.a_init_asym=inits{ii,3};
    c.Pf_a_frac=inits{ii,4}; c.fe_use_fdet=fd;
    ar=zeros(6,1); tk=zeros(6,1);
    for q=1:6
      s=temp_run_powerlaw_diag(c,struct('seed',seeds(q),'verbose',false));
      t=s.tout(:); mo=t>1.6;
      ar(q)=mean(s.a_hat_out(mo,az)./max(s.a_true_out(mo,az),eps));
      e=s.p_d_out(2:end,:)-s.p_true_out(1:end-1,:); te=t(2:end);
      tk(q)=1e3*std(e(te>1.6,az));
    end
    fprintf('%-17s %-9s | %10.4f %8.4f | %8.2f\n', ...
      ternary(fd==0,inits{ii,1},''), ternary(fd==0,'OFF','ON'), ...
      mean(ar), std(ar), mean(tk));
  end
end
fprintf('\n判讀: fdet ON + 誠實 init, 若 a_hat 停在 init 附近不動\n');
fprintf('      -> y1 被刪掉真的毀了辨識能力, 先前的 0.998 是假象\n');
function o=ternary(c,a,b); if c; o=a; else; o=b; end; end
