% FORK OF run_meng_ch4_sA.m @ 455d954 | PURPOSE: ledger 29 pre-registered acceptance for y2_whiten (P1-P6) | EXPIRES: ledger 29 verdict recorded (KEPT for re-run)
% ledger 29 pre-registered acceptance for y2_whiten (predictions BEFORE data):
%  P1 x honesty Var(e)/P66 in [0.5,2]   (from 6-16x)
%  P2 x actual band ~ claimed +-30%, expected 1.2-2.2%
%  P3 z honesty in [0.5,2]              (from ~7-16x)
%  P4 late-window (8.5-10 s) x |mean e| <= y2-off arm's (y2 info retained)
%  P5 a_cov 0.05->0.025 with whiten on: x band shift < 0.5 pp (invariance)
%  P6 flag off: seed-7 band 2.93% / bias +2.48% reproduced (bit-identity)
here = fileparts(mfilename('fullpath')); cd(fileparts(fileparts(here)));
warning('off','all'); addpath(genpath('model')); addpath('test_script/scratch');
pc = physical_constants(); Ts=pc.Ts; kBT=pc.k_B*pc.T; gN=pc.gamma_N;
SIGMA_N=[0.0007;0.0007;0.0023];
cd44 = calc_cdpmr_ch4(0.4, 0.9995, 0.35);
cc = build_eq17_constants(struct('lambda_c',0.4,'sigma2_n_s',SIGMA_N.^2,'kBT',kBT,'t_warmup_kf',0));
cc.iir_warmup_mode='prefill'; cc.force_Q77_zero=true; cc.h_bar_safe=1;
cc.control_law='ch4'; cc.lambda_f=[0.9995;0.9995;0.999];
cc.ch4_fdet=true; cc.ch4_stale_ff=true;
cc.C_dpmr_eff=cd44.C_dpmr(1:3); cc.C_np_eff=cd44.C_n(1:3);
cc.xi_per_axis=(cd44.C_n(1:3)./cd44.C_dpmr(1:3)).*SIGMA_N.^2/(4*kBT);
params = struct('ctrl',struct('enable',1,'Ts',Ts,'k_B',pc.k_B,'T',pc.T,'gamma',gN,'sigma2_noise',SIGMA_N.^2), ...
    'common',struct('R',pc.R,'p0',[0;0;15],'gamma_N',gN), ...
    'wall',struct('w_hat',[0;0;1],'pz',0,'enable_wall_effect',1), ...
    'traj',struct('amplitude',9,'frequency',1));
T=10; N=round(T/Ts); t=(0:N)'*Ts;
hz = max(15-12.5*t/T, 2.5); pd_arr=[9*sin(2*pi*t), 9*sin(2*pi*t), hz];
w = t(1:N)>3 & t(1:N)<8; late = t(1:N)>8.5;
arms = struct('nm',{'BASE','WHIT','ACOV'},'wh',{false,true,true},'ac',{[],[],0.025});
for ia=1:3
  a=arms(ia);
  for sd = [7 11 23 42]
    if ia==3 && sd>11; continue; end
    ccr=cc; ccr.y2_whiten=a.wh; if ~isempty(a.ac); ccr.a_cov=a.ac; end
    rng(sd); clear motion_control_law_eq17_core
    p=[0;0;15]; pe1=p; pe2=p;
    ea=zeros(N,2); cb=zeros(N,2);   % cols: x, z
    for k=1:N
        pd=pd_arr(k,:)'; del_pd=pd_arr(k+1,:)'-pd; pe=p; p_m=pe2+SIGMA_N.*randn(3,1);
        [f_d,~,dg]=motion_control_law_eq17_core(del_pd,pd,p_m,params,ccr);
        h_bar=max(p(3)/pc.R,1.001); [cpar,cper]=calc_correction_functions(h_bar);
        gam=gN*[cpar;cpar;cper]; at3=Ts./gam;
        p=p+at3.*(f_d+sqrt(4*kBT*gam/Ts).*randn(3,1));
        ea(k,:)=[dg.a_hat(1)/at3(1)-1, dg.a_hat(3)/at3(3)-1];
        cb(k,:)=[sqrt(dg.P_a(1))/at3(1), sqrt(dg.P_a(3))/at3(3)];
        pe2=pe1; pe1=pe;
    end
    hx=var(ea(w,1))/mean(cb(w,1))^2; hzn=var(ea(w,2))/mean(cb(w,2))^2;
    fprintf('%s seed %2d: x band %.2f%% claimed %.2f%% hon %.1f bias %+.2f%% late %+.2f%% | z band %.2f%% claimed %.2f%% hon %.1f bias %+.2f%%\n', ...
      a.nm, sd, 100*std(ea(w,1)), 100*mean(cb(w,1)), hx, 100*mean(ea(w,1)), 100*mean(ea(late,1)), ...
      100*std(ea(w,2)), 100*mean(cb(w,2)), hzn, 100*mean(ea(w,2)));
  end
end
fprintf('WHITEN DONE\n');
