% FORK OF run_meng_ch4_sA.m @ 81e4047 | PURPOSE: Fig-6-layout A/B (RAW / y2_whiten / lf=0.98) vs journal Fig 6 texture | EXPIRES: Fig-6 texture adjudication recorded in ledger
% Fig-6-style A/B: three arms rendered in Meng's 3-row layout at his scales.
% Arms: RAW (current mainline, per-axis lf), WHIT (y2_whiten), LF98 (lf=0.98 raw).
% Console: per-axis fast (<0.1 s) and slow components of e_a/a in 3-8 s window.
here = fileparts(mfilename('fullpath')); cd(fileparts(fileparts(here)));
warning('off','all'); addpath(genpath('model')); addpath('test_script/scratch');
pc = physical_constants(); Ts=pc.Ts; kBT=pc.k_B*pc.T; gN=pc.gamma_N;
SIGMA_N=[0.0007;0.0007;0.0023];
params = struct('ctrl',struct('enable',1,'Ts',Ts,'k_B',pc.k_B,'T',pc.T,'gamma',gN,'sigma2_noise',SIGMA_N.^2), ...
    'common',struct('R',pc.R,'p0',[0;0;15],'gamma_N',gN), ...
    'wall',struct('w_hat',[0;0;1],'pz',0,'enable_wall_effect',1), ...
    'traj',struct('amplitude',9,'frequency',1));
T=10; N=round(T/Ts); t=(0:N)'*Ts;
hz = max(15-12.5*t/T, 2.5); pd_arr=[9*sin(2*pi*t), 9*sin(2*pi*t), hz];
w = t(1:N)>3 & t(1:N)<8;
arms = struct('nm',{'RAW','WHIT','LF98'}, 'lf',{[0.9995;0.9995;0.999],[0.9995;0.9995;0.999],[0.98;0.98;0.98]}, ...
              'wh',{false,true,false});
for ia=1:3
    a=arms(ia);
    lfu = unique(a.lf)';
    cc = build_eq17_constants(struct('lambda_c',0.4,'sigma2_n_s',SIGMA_N.^2,'kBT',kBT,'t_warmup_kf',0));
    cc.iir_warmup_mode='prefill'; cc.force_Q77_zero=true; cc.h_bar_safe=1;
    cc.control_law='ch4'; cc.lambda_f=a.lf;
    cc.ch4_fdet=true; cc.ch4_stale_ff=true; cc.y2_whiten=a.wh;
    pfl=a.lf; pfl(pfl>=1)=0.98; cc.Pf_init_lambda_f=pfl;
    Cd=zeros(3,1); Cn=zeros(3,1);
    for u=lfu
        cdu=calc_cdpmr_ch4(0.4,u,0.35); m=(a.lf==u); Cd(m)=cdu.C_dpmr(m); Cn(m)=cdu.C_n(m);
    end
    cc.C_dpmr_eff=Cd; cc.C_np_eff=Cn;
    cc.xi_per_axis=(Cn./Cd).*SIGMA_N.^2/(4*kBT);
    rng(7); clear motion_control_law_eq17_core
    p=[0;0;15]; pe1=p; pe2=p; AH=zeros(N,3); AT=zeros(N,3);
    for k=1:N
        pd=pd_arr(k,:)'; del_pd=pd_arr(k+1,:)'-pd; pe=p; p_m=pe2+SIGMA_N.*randn(3,1);
        [f_d,~,dg]=motion_control_law_eq17_core(del_pd,pd,p_m,params,cc);
        h_bar=max(p(3)/pc.R,1.001); [cpar,cper]=calc_correction_functions(h_bar);
        gam=gN*[cpar;cpar;cper]; at3=Ts./gam;
        p=p+at3.*(f_d+sqrt(4*kBT*gam/Ts).*randn(3,1));
        AH(k,:)=dg.a_hat'; AT(k,:)=at3';
        pe2=pe1; pe1=pe;
    end
    % metrics
    for ax=1:3
        ea=AH(:,ax)./AT(:,ax)-1;
        win=round(0.1/Ts); ker=ones(win,1)/win; sl=conv(ea,ker,'same');
        fprintf('%s ax%d: fast(<0.1s) %.2f%%  slow %.2f%%  total %.2f%%\n', a.nm, ax, ...
            100*std(ea(w)-sl(w)), 100*std(sl(w)), 100*std(ea(w)));
    end
    % Fig-6 style page
    f=figure('Position',[40 40 900 700],'Color','w','Visible','off');
    tl=tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');
    yls={[0.0063 0.0142],[0.0063 0.0142],[0.0012 0.0128]};
    labs={'a_x','a_y','a_z'};
    for ax=1:3
        aa=nexttile(tl,ax); hold(aa,'on');
        hE=plot(aa,t(1:N),AH(:,ax),'Color',[0 0.2 0.9],'LineWidth',0.7);
        hR=plot(aa,t(1:N),AT(:,ax),'Color',[0.8 0 0],'LineWidth',2.0);
        ylim(aa,yls{ax}); xlim(aa,[0 10]); box(aa,'on');
        ylabel(aa,labs{ax}); set(aa,'FontSize',15,'FontWeight','bold');
        if ax==1
            legend(aa,[hE hR],{'Estimated','Real'},'Orientation','horizontal','Location','northoutside');
        end
        if ax==3; xlabel(aa,'Time(s)'); end
    end
    od='test_results/meng_ch4_s0';
    exportgraphics(f,fullfile(od,sprintf('meng_fig6_style_%s.png',a.nm)),'Resolution',150);
end
fprintf('FIG6AB DONE\n');
