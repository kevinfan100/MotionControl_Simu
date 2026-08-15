% FORK OF plot_meng_ch4_sApanels.m @ 455d954 | PURPOSE: ledger 28 comparison figure -- e_a with claimed band, y2 on vs off, 2 seeds | EXPIRES: with run_meng_ch4_y2off_arm.m
here = fileparts(mfilename('fullpath')); cd(fileparts(fileparts(here)));
warning('off','all'); addpath(genpath('model')); addpath('test_script/scratch');
pc = physical_constants(); Ts=pc.Ts; kBT=pc.k_B*pc.T; gN=pc.gamma_N;
SIGMA_N=[0.0007;0.0007;0.0023];
cd44 = calc_cdpmr_ch4(0.4, 0.9995, 0.35);
cc0 = build_eq17_constants(struct('lambda_c',0.4,'sigma2_n_s',SIGMA_N.^2,'kBT',kBT,'t_warmup_kf',0));
cc0.iir_warmup_mode='prefill'; cc0.force_Q77_zero=true; cc0.h_bar_safe=1;
cc0.control_law='ch4'; cc0.lambda_f=[0.9995;0.9995;0.999];
cc0.ch4_fdet=true; cc0.ch4_stale_ff=true;
cc0.C_dpmr_eff=cd44.C_dpmr(1:3); cc0.C_np_eff=cd44.C_n(1:3);
cc0.xi_per_axis=(cd44.C_n(1:3)./cd44.C_dpmr(1:3)).*SIGMA_N.^2/(4*kBT);
ccY = cc0; ccY.IF_eff_per_axis = [1e12; cc0.IF_eff; cc0.IF_eff];
params = struct('ctrl',struct('enable',1,'Ts',Ts,'k_B',pc.k_B,'T',pc.T,'gamma',gN,'sigma2_noise',SIGMA_N.^2), ...
    'common',struct('R',pc.R,'p0',[0;0;15],'gamma_N',gN), ...
    'wall',struct('w_hat',[0;0;1],'pz',0,'enable_wall_effect',1), ...
    'traj',struct('amplitude',9,'frequency',1));
T=10; N=round(T/Ts); t=(0:N)'*Ts;
hz = max(15-12.5*t/T, 2.5); pd_arr=[9*sin(2*pi*t), 9*sin(2*pi*t), hz];
seeds=[7 11]; ccs={cc0,ccY};
EA=zeros(N,2,2); CB=zeros(N,2,2);
for ia=1:2
  for is=1:2
    rng(seeds(is)); clear motion_control_law_eq17_core
    p=[0;0;15]; pe1=p; pe2=p;
    for k=1:N
        pd=pd_arr(k,:)'; del_pd=pd_arr(k+1,:)'-pd; pe=p; p_m=pe2+SIGMA_N.*randn(3,1);
        [f_d,~,dg]=motion_control_law_eq17_core(del_pd,pd,p_m,params,ccs{ia});
        h_bar=max(p(3)/pc.R,1.001); [cpar,cper]=calc_correction_functions(h_bar);
        gam=gN*[cpar;cpar;cper]; at=Ts./gam;
        p=p+at.*(f_d+sqrt(4*kBT*gam/Ts).*randn(3,1));
        EA(k,is,ia)=100*(dg.a_hat(1)/at(1)-1); CB(k,is,ia)=100*sqrt(dg.P_a(1))/at(1);
        pe2=pe1; pe1=pe;
    end
  end
end
C_TRUE=[0.8 0 0]; C_EST=[0 0.2 0.9]; C_BAND=[0.75 0.80 0.95]; FS=15; LW=1.2;
tv=t(1:N); f=figure('Position',[40 40 1500 700],'Color','w','Visible','off');
tl=tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact');
lab={'y_2 on (production)','y_2 off (x axis)'};
yl=[-8 8];
for ia=1:2
  for is=1:2
    a=nexttile(tl,2*(ia-1)+is); hold(a,'on');
    fill(a,[tv;flipud(tv)],[CB(:,is,ia);-flipud(CB(:,is,ia))],C_BAND,'EdgeColor','none','FaceAlpha',0.8);
    plot(a,tv,EA(:,is,ia),'Color',C_EST,'LineWidth',LW);
    yline(a,0,'Color',C_TRUE,'LineWidth',1);
    xlim(a,[0 10]); ylim(a,yl); box(a,'on');
    set(a,'FontSize',FS,'FontWeight','bold');
    if ia==2; xlabel(a,'t [s]'); end
    if is==1; ylabel(a,sprintf('%s  e_a/a [%%]',lab{ia})); end
    if ia==1; title(a,sprintf('seed %d',seeds(is)),'FontWeight','bold'); end
  end
end
od='test_results/meng_ch4_s0'; if ~exist(od,'dir'); mkdir(od); end
exportgraphics(f,fullfile(od,'meng_y2off_band_cmp.png'),'Resolution',150);
fprintf('FIG DONE\n');
