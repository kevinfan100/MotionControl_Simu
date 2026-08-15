% FORK OF run_meng_ch4_sA.m @ 455d954 | PURPOSE: ledger 28 arm Y -- kill x-axis y2 via IF_override; convicts the variance channel as the slow-wander driver | EXPIRES: y2 coloring model lands
% Arm Y: kill y2 on x axis (R22 -> huge). If the slow wander collapses toward
% the (recomputed) claimed band -> y2 colored-noise convicted as band driver.
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
cc.IF_eff_per_axis = [1e12; cc.IF_eff; cc.IF_eff];   % kill y2 on x only
params = struct('ctrl',struct('enable',1,'Ts',Ts,'k_B',pc.k_B,'T',pc.T,'gamma',gN,'sigma2_noise',SIGMA_N.^2), ...
    'common',struct('R',pc.R,'p0',[0;0;15],'gamma_N',gN), ...
    'wall',struct('w_hat',[0;0;1],'pz',0,'enable_wall_effect',1), ...
    'traj',struct('amplitude',9,'frequency',1));
T=10; N=round(T/Ts); t=(0:N)'*Ts;
hz = max(15-12.5*t/T, 2.5); pd_arr=[9*sin(2*pi*t), 9*sin(2*pi*t), hz];
w = t(1:N)>3 & t(1:N)<8;
for sd = [7 11 23 42]
    rng(sd); clear motion_control_law_eq17_core
    p=[0;0;15]; pe1=p; pe2=p; ea=zeros(N,1); cb=zeros(N,1);
    for k=1:N
        pd=pd_arr(k,:)'; del_pd=pd_arr(k+1,:)'-pd; pe=p; p_m=pe2+SIGMA_N.*randn(3,1);
        [f_d,~,dg]=motion_control_law_eq17_core(del_pd,pd,p_m,params,cc);
        h_bar=max(p(3)/pc.R,1.001); [cpar,cper]=calc_correction_functions(h_bar);
        gam=gN*[cpar;cpar;cper]; at=Ts./gam;
        p=p+at.*(f_d+sqrt(4*kBT*gam/Ts).*randn(3,1));
        ea(k)=dg.a_hat(1)/at(1)-1; cb(k)=sqrt(dg.P_a(1))/at(1);
        pe2=pe1; pe1=pe;
    end
    win=round(0.5/Ts); ker=ones(win,1)/win; sl=conv(ea,ker,'same');
    fprintf('Y seed %2d: band %.2f%% (claimed %.2f%%, fast %.2f%%) | bias %+.2f%%\n', ...
        sd, 100*std(ea(w)), 100*mean(cb(w)), 100*std(ea(w)-sl(w)), 100*mean(ea(w)));
end
fprintf('Y2OFF DONE\n');
