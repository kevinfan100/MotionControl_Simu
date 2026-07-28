%VERIFY_POWERLAW_REGRESS_FINAL  PRODUCTION regression after fdet + asym init.
%   Runs motion_control_law_5state_powerlaw via run_5state_powerlaw with
%   NO overrides -- i.e. exercising every new default.
clear; clc;
here = fileparts(mfilename('fullpath'));  addpath(genpath(fullfile(here,'..','..')));
pc = physical_constants(); az=3; seeds=[7 11 23 42 101 777];
base = user_config();
base.h_min=1.05*pc.R; base.ctrl_enable=true; base.thermal_enable=true;
base.meas_noise_enable=true; base.lambda_c=0.7; base.a_pd=0.05; base.a_cov=0.05;
base.meas_noise_std=[0.00062;0.00057;0.00331]; base.h_bar_safe=1.5;
lastwarn('');

fprintf('=== 純定位 h_bar=22.2, 全部用產線預設 (無 override) ===\n');
fprintf('%6s | %8s %8s %8s %8s | %8s %8s\n','T[s]','t=0','t=1','t=6','t=12','a_xm/aT','trk nm');
pos=base; pos.trajectory_type='positioning'; pos.h_init=50;
for T=[2 5 12]
  pos.T_sim=T; V=zeros(6,4); axm=zeros(6,1); tk=zeros(6,1);
  for q=1:6
    s=run_5state_powerlaw(pos,struct('seed',seeds(q),'verbose',false));
    t=s.tout(:); r=s.a_hat_out(:,az)./s.a_true_out(:,az);
    tq=[0 1 6 12];
    for k=1:4; [~,i1]=min(abs(t-min(tq(k),T))); V(q,k)=r(i1); end
    ss=t>1.0; axm(q)=mean(s.a_xm_out(ss,az))/mean(s.a_true_out(ss,az));
    e=s.p_d_out(2:end,:)-s.p_true_out(1:end-1,:); te=t(2:end);
    tk(q)=1e3*std(e(te>1.0,az));
  end
  fprintf('%6.0f | %8.4f %8.4f %8.4f %8.4f | %8.4f %8.2f\n', T, ...
      mean(V(:,1)),mean(V(:,2)),mean(V(:,3)),mean(V(:,4)),mean(axm),mean(tk));
end

fprintf('\n=== 振盪場景 (a_cov 不變性驗收) ===\n');
osc=base; osc.trajectory_type='osc'; osc.h_init=50; osc.h_bottom=4.5;
osc.amplitude=2.5; osc.frequency=1; osc.n_cycles=2; osc.t_hold=0.5;
osc.t_descend_override=1.0; osc.T_sim=4.8;
fprintf('%8s | %9s %8s | %9s %8s | %8s %6s %5s\n','a_cov','a_hat/aT','sc', ...
        'p_osc','sc','trk z','clamp','NaN');
M=[];
for acv=[0.02 0.05 0.10 0.20]
  c=osc; c.a_cov=acv; ar=zeros(6,1); po=zeros(6,1); tk=zeros(6,1); cl=0; nn=0;
  for q=1:6
    s=run_5state_powerlaw(c,struct('seed',seeds(q),'verbose',false));
    t=s.tout(:); mo=t>1.6;
    ar(q)=mean(s.a_hat_out(mo,az)./max(s.a_true_out(mo,az),eps));
    po(q)=mean(s.p_hat_out(mo,az));
    e=s.p_d_out(2:end,:)-s.p_true_out(1:end-1,:); te=t(2:end);
    tk(q)=1e3*std(e(te>1.6,az));
    ph=s.p_hat_out(:,az); cl=cl+any(ph<=1e-9|ph>=5-1e-9);
    nn=nn+(any(~isfinite(s.a_hat_out(:)))||any(~isfinite(s.p_true_out(:))));
  end
  fprintf('%8.2f | %9.4f %8.4f | %9.4f %8.4f | %8.2f %6d %5d\n', acv, ...
      mean(ar),std(ar),mean(po),std(po),mean(tk),cl,nn);
  M=[M; mean(ar) mean(po) mean(tk)]; %#ok<AGROW>
end
sp=(max(M,[],1)-min(M,[],1))./max(abs(mean(M,1)),eps);
fprintf('\na_cov 不變性: a_hat %.2f%%  p_osc %.2f%%  trk %.2f%%\n',100*sp(1),100*sp(2),100*sp(3));
[wm,wid]=lastwarn(); if isempty(wm); fprintf('warnings: none\n'); else; fprintf('warnings: [%s] %s\n',wid,wm); end
fprintf('\n對照: 修改前 (peek init + realised f_d) 定位 12 s = 0.8375, 振盪 z = 0.9427\n');
