% FORK OF run_meng_ch4_sA.m @ 455d954 | PURPOSE: ledger 27/28 arms S+O -- innovation coloring vs oracle-a nonlinearity discriminators for the gain-band excess | EXPIRES: ledger 28 verdict recorded (KEPT for re-run)
% Arm S: innovation low-freq coloring vs S1-white booking (STALEFF config).
% Arm O: oracle-a law (f_d and f_det use true a per step; estimator untouched).
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
base = struct('ctrl',struct('enable',1,'Ts',Ts,'k_B',pc.k_B,'T',pc.T,'gamma',gN,'sigma2_noise',SIGMA_N.^2), ...
    'common',struct('R',pc.R,'p0',[0;0;15],'gamma_N',gN), ...
    'wall',struct('w_hat',[0;0;1],'pz',0,'enable_wall_effect',1), ...
    'traj',struct('amplitude',9,'frequency',1));
T=10; N=round(T/Ts); t=(0:N)'*Ts;
hz = max(15-12.5*t/T, 2.5); pd_arr=[9*sin(2*pi*t), 9*sin(2*pi*t), hz];
w = t(1:N)>3 & t(1:N)<8;
for arm = {'S','O'}
  for sd = [7 11 23 42]
    rng(sd); clear motion_control_law_eq17_core
    p=[0;0;15]; pe1=p; pe2=p;
    ea=zeros(N,1); cb=zeros(N,1); iv=zeros(N,1); s1=zeros(N,1);
    for k=1:N
        pd=pd_arr(k,:)'; del_pd=pd_arr(k+1,:)'-pd; pe=p; p_m=pe2+SIGMA_N.*randn(3,1);
        params=base;
        h_bar=max(p(3)/pc.R,1.001); [cpar,cper]=calc_correction_functions(h_bar);
        gam=gN*[cpar;cpar;cper]; at=Ts./gam;
        if strcmp(arm{1},'O'); params.ctrl.a_true_oracle = at; end
        [f_d,~,dg]=motion_control_law_eq17_core(del_pd,pd,p_m,params,cc);
        p=p+at.*(f_d+sqrt(4*kBT*gam/Ts).*randn(3,1));
        ea(k)=dg.a_hat(1)/at(1)-1; cb(k)=sqrt(dg.P_a(1))/at(1);
        if isfield(dg,'innovation_y1'); iv(k)=dg.innovation_y1(1); s1(k)=dg.S1_pred(1); end
        pe2=pe1; pe1=pe;
    end
    % band + honesty
    tot=std(ea(w)); claimed=mean(cb(w));
    % fast/slow split
    win=round(0.5/Ts); ker=ones(win,1)/win; sl=conv(ea,ker,'same'); fast=std(ea(w)-sl(w));
    % innovation coloring: block-mean variance factor at 0.5 s and 1.25 s
    vi=iv(w); vs1=mean(s1(w)); nrm=var(vi);
    kap=zeros(1,2); Ms=[round(0.5/Ts) round(1.25/Ts)];
    for j=1:2
        M=Ms(j); nb=floor(numel(vi)/M); bm=mean(reshape(vi(1:nb*M),M,nb),1);
        kap(j)=var(bm)*M/nrm;   % =1 for white
    end
    fprintf('%s seed %2d: band %.2f%% (claimed %.2f%%, fast %.2f%%) | Var(innov)/S1 %.2f | color kappa 0.5s/1.25s = %.2f / %.2f\n', ...
        arm{1}, sd, 100*tot, 100*claimed, 100*fast, nrm/vs1, kap(1), kap(2));
  end
end
fprintf('ARMS DONE\n');
