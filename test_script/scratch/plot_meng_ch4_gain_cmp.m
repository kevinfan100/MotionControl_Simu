% FORK OF plot_formC_dist_compare.m (test/motion-test) @ style port | PURPOSE: Meng gain-tracking RAW|WHIT pages in the formC house style + three-window split (mid/late/final-1s) | EXPIRES: Scenario-B figure set supersedes
% Meng gain-tracking comparison pages in the formC_dist_compare style:
% one page per (seed, axis); LEFT = y2 raw (Meng design), RIGHT = y2_whiten.
% Rows: 1 gain tracking (a_xm readout / true / hat), 2 rel err %, 3 e_a +- sqrt(P66).
% Console: three-window split per arm -- mid (2-6 s) slow wander, late (8.5-10 s)
% mean, final second (9-10 s) mean -- the user's two reading points separated.
here0 = fileparts(mfilename('fullpath')); cd(fileparts(fileparts(here0)));
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
T=10; N=round(T/Ts); t=(0:N-1)'*Ts;
tt=(0:N)'*Ts; hz = max(15-12.5*tt/T, 2.5);
pd_arr=[9*sin(2*pi*tt), 9*sin(2*pi*tt), hz];
seeds=[7 11];
D=cell(2,2);   % {arm, seed}: struct arrays with AH/AT/AM/PA (N x 3)
for ia=1:2
  ccr=cc; ccr.y2_whiten=(ia==2);
  for is=1:2
    rng(seeds(is)); clear motion_control_law_eq17_core
    p=[0;0;15]; pe1=p; pe2=p;
    AH=zeros(N,3); AT=zeros(N,3); AM=zeros(N,3); PA=zeros(N,3);
    for k=1:N
        pd=pd_arr(k,:)'; del_pd=pd_arr(k+1,:)'-pd; pe=p; p_m=pe2+SIGMA_N.*randn(3,1);
        [f_d,~,dg]=motion_control_law_eq17_core(del_pd,pd,p_m,params,ccr);
        h_bar=max(p(3)/pc.R,1.001); [cpar,cper]=calc_correction_functions(h_bar);
        gam=gN*[cpar;cpar;cper]; at3=Ts./gam;
        p=p+at3.*(f_d+sqrt(4*kBT*gam/Ts).*randn(3,1));
        AH(k,:)=dg.a_hat'; AT(k,:)=at3'; AM(k,:)=dg.a_xm'; PA(k,:)=dg.P_a';
        pe2=pe1; pe1=pe;
    end
    D{ia,is}=struct('AH',AH,'AT',AT,'AM',AM,'PA',PA);
  end
end
% console: three-window split
wmid = t>2 & t<6; wlate = t>8.5; wend = t>9;
AN={'RAW ','WHIT'};
for ia=1:2
  for is=1:2
    for ax=[1 3]
      e=100*(D{ia,is}.AH(:,ax)-D{ia,is}.AT(:,ax))./D{ia,is}.AT(:,ax);
      win=round(0.5/Ts); ker=ones(win,1)/win; sl=conv(e,ker,'same');
      fprintf('%s s%02d ax%d: mid slow-std %.2f%% | late mean %+.2f%% | final-1s mean %+.2f%%\n', ...
        AN{ia}, seeds(is), ax, std(sl(wmid)), mean(e(wlate)), mean(e(wend)));
    end
  end
end
% pages
COL_TRUE=[0.8 0 0]; COL_HAT=[0 0.2 0.9]; COL_MEAS=[0.45 0.72 0.95]; BANDC=[0.45 0.55 0.95];
FS=18; LFS=13; AXLW=2.0; LW=2.0;
NAME={'y_2 raw','y_2 whitened'};
axlab={'a_x','','a_z'};
od='test_results/meng_ch4_s0';
for is=1:2
  for ax=[1 3]
    d=cell(1,2);
    for ia=1:2
      s=struct();
      s.aT=D{ia,is}.AT(:,ax); s.aH=D{ia,is}.AH(:,ax); s.aM=D{ia,is}.AM(:,ax);
      s.e=100*(s.aH-s.aT)./s.aT;
      s.ea=s.aH-s.aT; s.sd=sqrt(D{ia,is}.PA(:,ax));
      s.t=t; d{ia}=s;
    end
    YL=cell(3,1);
    YL{1}=local_lim([d{1}.aT; d{1}.aH; d{1}.aM; d{2}.aT; d{2}.aH; d{2}.aM],0.05);
    YL{2}=local_lim([d{1}.e; d{2}.e],0.08);
    YL{3}=local_lim([d{1}.ea+d{1}.sd; d{1}.ea-d{1}.sd; d{2}.ea+d{2}.sd; d{2}.ea-d{2}.sd],0.10);
    f=figure('Position',[40 40 1500 1050],'Color','w','NumberTitle','off','Visible','off');
    tl=tiledlayout(f,3,2,'TileSpacing','compact','Padding','compact');
    for row=1:3
      for c=1:2
        s=d{c}; a=nexttile(tl,(row-1)*2+c); hold(a,'on');
        switch row
          case 1
            h0=plot(a,s.t,s.aM,'-','Color',COL_MEAS,'LineWidth',1.0,'DisplayName','a_{m} readout');
            h1=plot(a,s.t,s.aT,'-','Color',COL_TRUE,'LineWidth',LW+0.6,'DisplayName','a_{true}');
            h2=plot(a,s.t,s.aH,'-','Color',COL_HAT,'LineWidth',LW, ...
                    'DisplayName',sprintf('a_{hat}  %s  seed %d',NAME{c},seeds(is)));
            legend(a,[h1 h2 h0],'Location','northoutside','Orientation','horizontal', ...
                   'FontSize',LFS,'FontWeight','bold','Box','on');
            if c==1; ylabel(a,[axlab{ax} '  [\mum/pN]'],'FontSize',FS,'FontWeight','bold'); end
          case 2
            yline(a,0,'-','Color',[0.4 0.4 0.4],'LineWidth',1.0,'HandleVisibility','off');
            plot(a,s.t,s.e,'-','Color',COL_HAT,'LineWidth',LW);
            if c==1; ylabel(a,[axlab{ax} '  error  [%]'],'FontSize',FS,'FontWeight','bold'); end
          case 3
            fill(a,[s.t; flipud(s.t)],[s.ea+s.sd; flipud(s.ea-s.sd)],BANDC, ...
                 'EdgeColor','none','FaceAlpha',0.30,'DisplayName','\pm sqrt(P_{66})');
            plot(a,s.t,s.ea,'-','Color',COL_HAT,'LineWidth',LW,'DisplayName','e_a');
            yline(a,0,'-','Color',COL_TRUE,'LineWidth',LW,'DisplayName','zero');
            legend(a,'Location','northoutside','Orientation','horizontal', ...
                   'FontSize',LFS,'FontWeight','bold','Box','on');
            if c==1; ylabel(a,'e_a  [\mum/pN]','FontSize',FS,'FontWeight','bold'); end
            xlabel(a,'time  [s]','FontSize',FS,'FontWeight','bold');
        end
        xlim(a,[s.t(1) s.t(end)]); ylim(a,YL{row});
        set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on', ...
              'TickLabelInterpreter','tex');
        if row<3; set(a,'XTickLabel',[]); end
        if c==2; set(a,'YTickLabel',[]); end
        grid(a,'off');
      end
    end
    exportgraphics(f,fullfile(od,sprintf('meng_gain_cmp_ax%d_s%02d.png',ax,seeds(is))),'Resolution',150);
    close(f);
  end
end
fprintf('GAINCMP DONE\n');

function L = local_lim(v, pad)
    lo=min(v); hi=max(v); r=hi-lo; if r<=0; r=1; end
    L=[lo-pad*r, hi+pad*r];
end
