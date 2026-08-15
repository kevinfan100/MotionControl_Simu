% FORK OF run_meng_ch4_y2whiten_accept.m @ 8899dc2 | PURPOSE: ledger 31 pre-registered acceptance for lf_schedule (P1-P7 incl curvature arms) | EXPIRES: ledger 31 verdict recorded (KEPT for re-run)
% ledger 31 pre-registered acceptance for lf_schedule (predictions BEFORE data):
%  P1 z mid slow-std 3.8-4.6% -> ~2.5-3%
%  P2 z final-1s mean (s11) +28% -> <= 10%
%  P3 x mid ~unchanged (1.3-1.6%)
%  P4 x final-1s +7-12% -> <= 5%
%  P5 honesty stays in [0.5,2] both axes
%  P6 curvature: T* x2 and x0.5 both degrade (flat valley, mild)
%  P7 flag off = bit-identical (CONST arm reproduces WHIT numbers)
here0 = fileparts(mfilename('fullpath')); cd(fileparts(fileparts(here0)));
warning('off','all'); addpath(genpath('model')); addpath('test_script/scratch');
pc = physical_constants(); Ts=pc.Ts; kBT=pc.k_B*pc.T; gN=pc.gamma_N;
SIGMA_N=[0.0007;0.0007;0.0023];
cd44 = calc_cdpmr_ch4(0.4, 0.9995, 0.35);
cc = build_eq17_constants(struct('lambda_c',0.4,'sigma2_n_s',SIGMA_N.^2,'kBT',kBT,'t_warmup_kf',0));
cc.iir_warmup_mode='prefill'; cc.force_Q77_zero=true; cc.h_bar_safe=1;
cc.control_law='ch4'; cc.lambda_f=[0.9995;0.9995;0.999];
cc.ch4_fdet=true; cc.ch4_stale_ff=true; cc.y2_whiten=true;
cc.C_dpmr_eff=cd44.C_dpmr(1:3); cc.C_np_eff=cd44.C_n(1:3);
cc.xi_per_axis=(cd44.C_n(1:3)./cd44.C_dpmr(1:3)).*SIGMA_N.^2/(4*kBT);
params = struct('ctrl',struct('enable',1,'Ts',Ts,'k_B',pc.k_B,'T',pc.T,'gamma',gN,'sigma2_noise',SIGMA_N.^2), ...
    'common',struct('R',pc.R,'p0',[0;0;15],'gamma_N',gN), ...
    'wall',struct('w_hat',[0;0;1],'pz',0,'enable_wall_effect',1), ...
    'traj',struct('amplitude',9,'frequency',1));
T=10; N=round(T/Ts); t=(0:N-1)'*Ts;
tt=(0:N)'*Ts; hz = max(15-12.5*tt/T, 2.5);
pd_arr=[9*sin(2*pi*tt), 9*sin(2*pi*tt), hz];
wmid = t>2 & t<6; wlate = t>8.5; wend = t>9; wb = t>3 & t<8;
arms = struct('nm',{'CONST','SCHED','SC2.0','SC0.5'}, ...
              'sc',{[],1,2.0,0.5}, 'ns',{4,4,2,2});
seeds=[7 11 23 42];
D=cell(4,4);
for ia=1:4
  a=arms(ia);
  for is=1:a.ns
    ccr=cc;
    if ~isempty(a.sc); ccr.lf_schedule=true; ccr.lf_sched_scale=a.sc; end
    rng(seeds(is)); clear motion_control_law_eq17_core
    p=[0;0;15]; pe1=p; pe2=p;
    AH=zeros(N,3); AT=zeros(N,3); AM=zeros(N,3); PA=zeros(N,3); LF=zeros(N,3);
    for k=1:N
        pd=pd_arr(k,:)'; del_pd=pd_arr(k+1,:)'-pd; pe=p; p_m=pe2+SIGMA_N.*randn(3,1);
        [f_d,~,dg]=motion_control_law_eq17_core(del_pd,pd,p_m,params,ccr);
        h_bar=max(p(3)/pc.R,1.001); [cpar,cper]=calc_correction_functions(h_bar);
        gam=gN*[cpar;cpar;cper]; at3=Ts./gam;
        p=p+at3.*(f_d+sqrt(4*kBT*gam/Ts).*randn(3,1));
        AH(k,:)=dg.a_hat'; AT(k,:)=at3'; AM(k,:)=dg.a_xm'; PA(k,:)=dg.P_a';
        if isfield(dg,'lambda_f_used'); LF(k,:)=dg.lambda_f_used'; end
        pe2=pe1; pe1=pe;
    end
    D{ia,is}=struct('AH',AH,'AT',AT,'AM',AM,'PA',PA,'LF',LF);
    for ax=[1 3]
      e=100*(AH(:,ax)-AT(:,ax))./AT(:,ax);
      win=round(0.5/Ts); ker=ones(win,1)/win; sl=conv(e,ker,'same');
      hon=var(e(wb)/100.*AT(wb,ax))/mean(PA(wb,ax));
      fprintf('%-5s s%02d ax%d: mid %.2f%% | late %+.2f%% | fin1s %+.2f%% | band %.2f%% hon %.2f\n', ...
        a.nm, seeds(is), ax, std(sl(wmid)), mean(e(wlate)), mean(e(wend)), std(e(wb)), hon);
    end
  end
end
save('/Users/kevin/.claude/jobs/8581427c/tmp/lfsched_data.mat','D','t','seeds');
fprintf('LFSCHED DONE\n');
