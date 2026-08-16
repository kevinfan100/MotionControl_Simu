% STATUS: ACTIVE (scratch) | PURPOSE: Scenario B (journal IV-B) driver -- champion stack, arms N/E98/E9995/ESCH; five pre-registrations adjudicated in ledger 38 (all hit) | EXPIRES: replication line closed, kept for re-run
% Scenario B (journal IV-B first/second experiments; ledger 33).
% p_d = [9 sin(2pi 5t), 9 sin(2pi 5t), 3.5 + 1 sin(2pi 3t)], T = 3 s,
% wall z=0 convention (paper: wall 15, start 11.5 -> h center 3.5, amp 1).
% Pre-registered (BEFORE data):
%  B1 arm N (a_N frozen + suppress_xD): bounded delta, deterministic 5 Hz
%     line in x/y, NO divergence (g = a/a_N < 1 overdamped side).
%  B2 arm E98 (champion stack, lf=0.98 thesis const): delta zero-mean,
%     |delta| scale ~ tens of nm; criteria 1 and 3 (aligned pairing) pass.
%  B3 E98 a_z tracks the 3 Hz a_z oscillation (true 0.0023-0.0058) with
%     attenuation > 0.5; E9995 (lf=0.9995) CANNOT track (flat near mean).
%  B4 ESCH (lf_schedule v2): z tracking comparable-or-better than E98 with
%     tail sigma within the 10% validity cap.
%  B5 gate duty reported (G2 near-wall SNR) per 08-12 rule.
here0 = fileparts(mfilename('fullpath')); cd(fileparts(fileparts(here0)));
warning('off','all'); addpath(genpath('model')); addpath('test_script/scratch');
pc = physical_constants(); Ts=pc.Ts; kBT=pc.k_B*pc.T; gN=pc.gamma_N; a_N=Ts/gN;
SIGMA_N=[0.0007;0.0007;0.0023];
T=3; N=round(T/Ts); t=(0:N)'*Ts;
pd_arr=[9*sin(2*pi*5*t), 9*sin(2*pi*5*t), 3.5+1*sin(2*pi*3*t)];
params = struct('ctrl',struct('enable',1,'Ts',Ts,'k_B',pc.k_B,'T',pc.T,'gamma',gN,'sigma2_noise',SIGMA_N.^2), ...
    'common',struct('R',pc.R,'p0',[0;0;3.5],'gamma_N',gN), ...
    'wall',struct('w_hat',[0;0;1],'pz',0,'enable_wall_effect',1), ...
    'traj',struct('amplitude',9,'frequency',5));
base = build_eq17_constants(struct('lambda_c',0.4,'sigma2_n_s',SIGMA_N.^2,'kBT',kBT,'t_warmup_kf',0));
base.iir_warmup_mode='prefill'; base.force_Q77_zero=true; base.h_bar_safe=1;
mkE = @(lf,sched) deal(0); %#ok<NASGU>
seeds=[7 11 23 42];
% arm table
AR = {};
% N arm
ccN = base; ccN.a_hat_freeze = a_N*[1;1;1]; ccN.suppress_xD = true;
AR{end+1} = struct('nm','N    ','cc',ccN,'ns',1);
% E arms share the ch4 stack
for v = {{'E98  ',0.98,false},{'E9995',0.9995,false},{'ESCH ',0.9995,true}}
    nm=v{1}{1}; lf=v{1}{2}; sched=v{1}{3};
    cd44 = calc_cdpmr_ch4(0.4, lf, 0.35);
    cc = base;
    cc.control_law='ch4'; cc.lambda_f=lf*[1;1;1];
    pfl=lf; if pfl>=1; pfl=0.98; end; cc.Pf_init_lambda_f=pfl*[1;1;1];
    cc.ch4_fdet=false; cc.ch4_stale_ff=true; cc.y2_whiten=true;   % champion stack (ledger 35-37): fdet retired
    cc.lf_schedule=sched;
    if sched; cc.lf_sched_scale=[2;2;1]; end
    cc.C_dpmr_eff=cd44.C_dpmr(1:3); cc.C_np_eff=cd44.C_n(1:3);
    cc.xi_per_axis=(cd44.C_n(1:3)./cd44.C_dpmr(1:3)).*SIGMA_N.^2/(4*kBT);
    ns = 2; if strcmp(nm,'E9995'); ns=1; end
    AR{end+1} = struct('nm',nm,'cc',cc,'ns',ns);
end
w = t(1:N)>0.5;  % post-transient window
D=struct();
for ia=1:numel(AR)
  A=AR{ia};
  for is=1:A.ns
    rng(seeds(is)); clear motion_control_law_eq17_core
    p=[0;0;3.5]; pe1=p; pe2=p;
    P3=zeros(N,3); AH=zeros(N,3); AT=zeros(N,3); PA=zeros(N,3); G=zeros(N,3);
    for k=1:N
        pd=pd_arr(k,:)'; del_pd=pd_arr(k+1,:)'-pd; pe=p; p_m=pe2+SIGMA_N.*randn(3,1);
        [f_d,~,dg]=motion_control_law_eq17_core(del_pd,pd,p_m,params,A.cc);
        h_bar=max(p(3)/pc.R,1.001); [cpar,cper]=calc_correction_functions(h_bar);
        gam=gN*[cpar;cpar;cper]; at3=Ts./gam;
        p=p+at3.*(f_d+sqrt(4*kBT*gam/Ts).*randn(3,1));
        P3(k,:)=p'; AH(k,:)=dg.a_hat'; AT(k,:)=at3'; PA(k,:)=dg.P_a';
        G(k,:)=dg.guards_individual(:,3)';
        pe2=pe1; pe1=pe;
        if ~isfinite(p(3)) || abs(p(3))>1e6; fprintf('%s s%d DIVERGED k=%d\n',A.nm,seeds(is),k); break; end
    end
    % aligned delta: pd[k+1] vs p[k+1]
    dx = pd_arr(2:N+1,:) - P3;
    fld = sprintf('%s_s%d', strtrim(A.nm), seeds(is));
    D.(fld) = struct('P3',P3,'AH',AH,'AT',AT,'PA',PA,'dx',dx);
    % metrics: per axis mean/std of delta; 5 Hz (x) and 3 Hz (z) line on delta;
    % a_z oscillation tracking: correlate AH vs AT at 3 Hz
    s5=sin(2*pi*5*t(1:N)); c5=cos(2*pi*5*t(1:N));
    s3b=sin(2*pi*3*t(1:N)); c3b=cos(2*pi*3*t(1:N));
    L5 = 2*mean(dx(w,1).*(s5(w)+1i*c5(w)));
    L3 = 2*mean(dx(w,3).*(s3b(w)+1i*c3b(w)));
    % gain-tracking attenuation: project AH,AT (z) on 3 Hz
    PH = 2*mean((AH(w,3)-mean(AH(w,3))).*(s3b(w)+1i*c3b(w)));
    PT = 2*mean((AT(w,3)-mean(AT(w,3))).*(s3b(w)+1i*c3b(w)));
    fprintf('%s s%02d: dx std %5.1f/%5.1f/%5.1f nm  mean %+5.1f/%+5.1f/%+5.1f nm | line5(x) %4.1f nm line3(z) %4.1f nm | az-track %.2f ang %+.0f deg | az bias %+.1f%% | G2 duty %.2f\n', ...
      A.nm, seeds(is), 1e3*std(dx(w,1)),1e3*std(dx(w,2)),1e3*std(dx(w,3)), ...
      1e3*mean(dx(w,1)),1e3*mean(dx(w,2)),1e3*mean(dx(w,3)), ...
      1e3*abs(L5), 1e3*abs(L3), abs(PH/PT), rad2deg(angle(PH/PT)), ...
      100*(mean(AH(w,3))/mean(AT(w,3))-1), mean(G(w,2)));
  end
end
save(fullfile('test_results','meng_ch4_s0','meng_sB_run.mat'),'D','t','pd_arr','seeds');
fprintf('SB DONE\n');
