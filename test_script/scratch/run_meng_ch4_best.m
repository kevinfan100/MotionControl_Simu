% STATUS: ACTIVE (scratch) | SSOT for the Scenario-A champion configuration
% (ledger 35/36 convergence): ch4 + stale_ff + y2_whiten + lf_schedule[2;2;1],
% fdet OFF. Every retained flag holds current-stack mechanism evidence
% (ledger 36 LOO); fdet retired with the interaction account (ledger 35).
% Scorecard (4 seeds): x band 0.77-1.49% fin1s +3.2-4.3%; z band 2.06-3.22%
% (paper figure level) fin1s +2.3-8.3%; criteria 1-3 PASS.
% EXPIRES: Scenario-B adjudication supersedes.
here0 = fileparts(mfilename('fullpath')); cd(fileparts(fileparts(here0)));
warning('off','all'); addpath(genpath('model')); addpath('test_script/scratch');
pc = physical_constants(); Ts=pc.Ts; kBT=pc.k_B*pc.T; gN=pc.gamma_N;
SIGMA_N=[0.0007;0.0007;0.0023];
cd44 = calc_cdpmr_ch4(0.4, 0.9995, 0.35);
cc = build_eq17_constants(struct('lambda_c',0.4,'sigma2_n_s',SIGMA_N.^2,'kBT',kBT,'t_warmup_kf',0));
cc.iir_warmup_mode='prefill'; cc.force_Q77_zero=true; cc.h_bar_safe=1;
cc.control_law='ch4'; cc.lambda_f=[0.9995;0.9995;0.999];
cc.ch4_fdet=false; cc.ch4_stale_ff=true; cc.y2_whiten=true;   % fdet OFF ablation
cc.lf_schedule=true; cc.lf_sched_scale=[2;2;1];
cc.C_dpmr_eff=cd44.C_dpmr(1:3); cc.C_np_eff=cd44.C_n(1:3);
cc.C_dpmr_eff = cc.C_dpmr_eff .* [1.232;1.194;0.938];   % ledger 44/45: readout-chain C_emp
cc.xi_per_axis=(cd44.C_n(1:3)./cc.C_dpmr_eff).*SIGMA_N.^2/(4*kBT);
params = struct('ctrl',struct('enable',1,'Ts',Ts,'k_B',pc.k_B,'T',pc.T,'gamma',gN,'sigma2_noise',SIGMA_N.^2), ...
    'common',struct('R',pc.R,'p0',[0;0;15],'gamma_N',gN), ...
    'wall',struct('w_hat',[0;0;1],'pz',0,'enable_wall_effect',1), ...
    'traj',struct('amplitude',9,'frequency',1));
T=10; N=round(T/Ts); t=(0:N-1)'*Ts;
tt=(0:N)'*Ts; hz = max(15-12.5*tt/T, 2.5);
pd_arr=[9*sin(2*pi*tt), 9*sin(2*pi*tt), hz];
wmid=t>2&t<6; wb=t>3&t<8; wend=t>9; wl=t>8.5;
DB=cell(1,4); seeds=[7 11 23 42];
for is=1:4
    rng(seeds(is)); clear motion_control_law_eq17_core
    p=[0;0;15]; pe1=p; pe2=p;
    AH=zeros(N,3); AT=zeros(N,3); AM=zeros(N,3); PA=zeros(N,3); P3=zeros(N,3);
    for k=1:N
        pd=pd_arr(k,:)'; del_pd=pd_arr(k+1,:)'-pd; pe=p; p_m=pe2+SIGMA_N.*randn(3,1);
        [f_d,~,dg]=motion_control_law_eq17_core(del_pd,pd,p_m,params,cc);
        h_bar=max(p(3)/pc.R,1.001); [cpar,cper]=calc_correction_functions(h_bar);
        gam=gN*[cpar;cpar;cper]; at3=Ts./gam;
        p=p+at3.*(f_d+sqrt(4*kBT*gam/Ts).*randn(3,1));
        AH(k,:)=dg.a_hat'; AT(k,:)=at3'; AM(k,:)=dg.a_xm'; PA(k,:)=dg.P_a'; P3(k,:)=p';
        pe2=pe1; pe1=pe;
    end
    dx = pd_arr(2:N+1,:) - P3;   % aligned pairing
    DB{is}=struct('AH',AH,'AT',AT,'AM',AM,'PA',PA,'dx',dx);
    s1=sin(2*pi*t); c1=cos(2*pi*t); wl5=t>5;
    L1 = 2*mean(dx(wl5,1).*(s1(wl5)+1i*c1(wl5)));
    for ax=[1 3]
      e=100*(AH(:,ax)-AT(:,ax))./AT(:,ax);
      win=round(0.5/Ts); ker=ones(win,1)/win; sl=conv(e,ker,'same');
      fprintf('F0 s%02d ax%d: mid %.2f%% | late %+.2f%% | fin1s %+.2f%% | band %.2f%%\n', ...
        seeds(is), ax, std(sl(wmid)), mean(e(wl)), mean(e(wend)), std(e(wb)));
    end
    fprintf('F0 s%02d dx: std x %.1f nm mean x %+.1f nm | std z %.1f nm mean z %+.1f nm | 1Hz line %.1f nm\n', ...
      seeds(is), 1e3*std(dx(wl5,1)), 1e3*mean(dx(wl5,1)), 1e3*std(dx(wl5,3)), 1e3*mean(dx(wl5,3)), 1e3*abs(L1));
end
save(fullfile('test_results','meng_ch4_s0','meng_best_run.mat'),'DB','t','seeds');
fprintf('F0 DONE\n');
