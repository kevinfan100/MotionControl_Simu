% ledger 40 pre-registered acceptance for y2_ma2 (BEFORE data):
%  M1 innov2 (1+2*sum rho) 2.6-2.9 -> ~1 (0.8-1.3)
%  M2 per-step honesty Var(innov2)/S2 0.35 -> 0.8-1.2
%  M3 z band shrinks >= 1.2x vs champion (3.0-4.4 -> <= 2.5-3.7), bias unchanged (<= +-2%)
%  M4 a_cov 0.05 -> 0.025 invariance holds (band shift < 0.5 pp)
%  M5 flag off bit-identical (champion numbers reproduced)
cd('/Users/kevin/Code/MotionControl_Simu-meng-ch4');
warning('off','all'); addpath(genpath('model')); addpath('test_script/scratch');
pc = physical_constants(); Ts=pc.Ts; kBT=pc.k_B*pc.T; gN=pc.gamma_N;
SIGMA_N=[0.0007;0.0007;0.0023];
cd44 = calc_cdpmr_ch4(0.4, 0.9995, 0.35);
cc = build_eq17_constants(struct('lambda_c',0.4,'sigma2_n_s',SIGMA_N.^2,'kBT',kBT,'t_warmup_kf',0));
cc.iir_warmup_mode='prefill'; cc.force_Q77_zero=true; cc.h_bar_safe=1;
cc.control_law='ch4'; cc.lambda_f=[0.9995;0.9995;0.999];
cc.ch4_fdet=false; cc.ch4_stale_ff=true; cc.y2_whiten=true;
cc.lf_schedule=true; cc.lf_sched_scale=[2;2;1];
cc.C_dpmr_eff=cd44.C_dpmr(1:3); cc.C_np_eff=cd44.C_n(1:3);
cc.xi_per_axis=(cd44.C_n(1:3)./cd44.C_dpmr(1:3)).*SIGMA_N.^2/(4*kBT);
params = struct('ctrl',struct('enable',1,'Ts',Ts,'k_B',pc.k_B,'T',pc.T,'gamma',gN,'sigma2_noise',SIGMA_N.^2), ...
    'common',struct('R',pc.R,'p0',[0;0;15],'gamma_N',gN), ...
    'wall',struct('w_hat',[0;0;1],'pz',0,'enable_wall_effect',1), ...
    'traj',struct('amplitude',9,'frequency',1));
T=10; N=round(T/Ts); t=(0:N-1)'*Ts;
tt=(0:N)'*Ts; hz = max(15-12.5*tt/T, 2.5);
pd_arr=[9*sin(2*pi*tt), 9*sin(2*pi*tt), hz];
wmid=t>2&t<6; wb=t>3&t<8; wend=t>9;
seeds=[7 11 23 42];
arms = struct('nm',{'MA  ','ACOV'}, 'ac',{[],0.025}, 'ns',{4,1});
DM=cell(1,4);
for ia=1:2
  a=arms(ia);
  ccr=cc; ccr.y2_ma2=true;
  if ~isempty(a.ac); ccr.a_cov=a.ac; end
  for is=1:a.ns
    rng(seeds(is)); clear motion_control_law_eq17_core
    p=[0;0;15]; pe1=p; pe2=p;
    AH=zeros(N,3); AT=zeros(N,3); PA=zeros(N,3); IV2=zeros(N,3); AM=zeros(N,3);
    for k=1:N
        pd=pd_arr(k,:)'; del_pd=pd_arr(k+1,:)'-pd; pe=p; p_m=pe2+SIGMA_N.*randn(3,1);
        [f_d,~,dg]=motion_control_law_eq17_core(del_pd,pd,p_m,params,ccr);
        h_bar=max(p(3)/pc.R,1.001); [cpar,cper]=calc_correction_functions(h_bar);
        gam=gN*[cpar;cpar;cper]; at3=Ts./gam;
        p=p+at3.*(f_d+sqrt(4*kBT*gam/Ts).*randn(3,1));
        AH(k,:)=dg.a_hat'; AT(k,:)=at3'; PA(k,:)=dg.P_a'; AM(k,:)=dg.a_xm';
        if isfield(dg,'innovation_y2'); IV2(k,:)=dg.innovation_y2'; end
        pe2=pe1; pe1=pe;
    end
    if ia==1; DM{is}=struct('AH',AH,'AT',AT,'PA',PA,'AM',AM); end
    for ax=[1 3]
      e=100*(AH(:,ax)-AT(:,ax))./AT(:,ax);
      v2=IV2(wb,ax); v2=v2-mean(v2); nrm2=mean(v2.^2);
      rs=0; for lg=1:12; rs=rs+mean(v2(1+lg:end).*v2(1:end-lg))/nrm2; end
      fprintf('%s s%02d ax%d: band %.2f%% bias %+5.2f%% fin1s %+6.2f%% | acf(1+2S) %.2f\n', ...
        a.nm, seeds(is), ax, std(e(wb)), mean(e(wb)), mean(e(wend)), 1+2*rs);
    end
  end
end
save('/Users/kevin/.claude/jobs/8581427c/tmp/ma2_data.mat','DM','t','seeds');
fprintf('MA2 DONE\n');
