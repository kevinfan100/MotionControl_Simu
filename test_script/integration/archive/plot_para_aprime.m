function plot_para_aprime()
%PLOT_PARA_APRIME  Para-model gain-slope (a') estimation figures, current
%   canonical style (mirrors plot_var_ahat_6state): NO title (stats -> console),
%   True=red [0.8 0 0], Estimated=blue [0 0.2 0.9], Measured=light-blue,
%   FS=18 bold, legend northoutside box on, Box on / grid off, Visible off,
%   units (\mum/pN), exportgraphics 150 dpi.
%
%   plot_para_aprime()
%
%   EXPLORATORY plotter for the unknown-wall a' study. The para estimator
%   c = 1 + p1/(h_bar-1) (z, pole basis) / c = 1 + b1/h_bar (x,y) is run inline
%   via the 4-state controller's a_ctrl_override hook (a_ctrl = a(h_bar; theta_hat)
%   closes the loop -- Stage-2). NOT production.
%
%   Writes to test_results/para_stage1/ (gitignored):
%     fig_s2_a_vs_hbar.png       gain a   vs h/R   (Measured / True / Estimated)
%     fig_s2_aprime_vs_hbar.png  slope a' vs h/R   (True / Estimated)
%     fig_s2_a_time.png          a_z(t)   descent(0-1s) -> oscillation(1-4s)
%     fig_s2_aprime_time.png     a'_z(t)  descent -> oscillation
%     fig_1param_vs_2param.png   open-loop a' identifiability: 1-param (tight) vs
%                                2-param (collinear) over 5 seeds. Requires
%                                result_5seed_corrected.mat + result_1param.mat
%                                (skipped with a notice if absent).
%
%   Trajectory: NO positioning hold (t_hold=0). See also: plot_var_ahat_6state,
%   motion_control_law_eq17_4state.

    [sd,~,~]=fileparts(mfilename('fullpath')); proj=fileparts(fileparts(sd));
    addpath(genpath(fullfile(proj,'model')));
    od=fullfile(proj,'test_results','para_stage1');
    if ~exist(od,'dir'); mkdir(od); end

    % ---- config: descent(0-1) -> oscillation(1-4), no holds either end ----
    config = user_config();
    config.eq17_variant='4state'; config.trajectory_type='osc';
    config.h_init=20; config.h_bottom=2.5; config.amplitude=2.5; config.frequency=1; config.n_cycles=3;
    config.t_hold=0; config.T_sim=4.0;
    config.ctrl_enable=true; config.meas_noise_enable=true; config.thermal_enable=true;
    config.lambda_c=0.7; config.a_pd=0.05; config.a_cov=0.05;
    config.meas_noise_std=[0.00062;0.00057;0.00331];

    rng(1); P=calc_simulation_params(config).Value;
    Ts=P.common.Ts; gN=P.common.gamma_N; R=P.common.R; w=P.wall.w_hat; pz=P.wall.pz; a_nom=Ts/gN;
    eq=struct('lambda_c',config.lambda_c,'option','A_MA2_full','sigma2_n_s',(config.meas_noise_std(:)).^2,...
      'kBT',P.ctrl.k_B*P.ctrl.T,'t_warmup_kf',0,'h_bar_safe',1.5,'d',2,'a_cov',config.a_cov,'a_pd',config.a_pd,'iir_warmup_mode','prefill');
    ctrl_const=build_eq17_6state_constants(eq); d=ctrl_const.d;
    clear motion_control_law_eq17_4state trajectory_generator calc_thermal_force;
    N=round(config.T_sim/Ts)+1; tau=20; relstd=0.42;
    p_curr=P.common.p0; p_m_buf=repmat(p_curr,1,d+1); pd_for_ctrl=p_curr;
    thx=0; thy=0; thz=0; Px=9; Py=9; Pz=9;
    tL=zeros(N,1); hbmL=zeros(N,1); aTz=zeros(N,1); aEz=zeros(N,1); apEz=zeros(N,1); apTz=zeros(N,1); axmzL=zeros(N,1);
    for k=1:N
      t=(k-1)*Ts; [pd_kp1,del_pd]=trajectory_generator(t,P); pd_k=pd_for_ctrl; p_m_del=p_m_buf(:,1);
      hbar=(dot(p_m_del,w)-pz)/R; hh=max(hbar,1.001);
      a_ov=[a_nom/(1+thx/hh); a_nom/(1+thy/hh); a_nom/(1+thz/max(hh-1,0.05))];
      [f_d_k,~,diag_k]=motion_control_law_eq17_4state(del_pd,pd_k,p_m_del,P,ctrl_const,a_ov);
      axm=diag_k.a_xm; gt=diag_k.gate_active_per_axis; hbm=diag_k.h_bar; hm=max(hbm,1.001);
      if hbm>1 && all(isfinite(axm))
        phi=1/hm;     c=1+thx*phi; ap=a_nom/c; if ~gt(1)&&axm(1)>0; H=-(ap/c)*phi; RR=(relstd*ap)^2*tau; S=H*Px*H+RR; K=Px*H/S; thx=thx+K*(axm(1)-ap); Px=(1-K*H)*Px; end
        phi=1/hm;     c=1+thy*phi; ap=a_nom/c; if ~gt(2)&&axm(2)>0; H=-(ap/c)*phi; RR=(relstd*ap)^2*tau; S=H*Py*H+RR; K=Py*H/S; thy=thy+K*(axm(2)-ap); Py=(1-K*H)*Py; end
        phi=1/(hm-1); c=1+thz*phi; ap=a_nom/c; if ~gt(3)&&axm(3)>0; H=-(ap/c)*phi; RR=(relstd*ap)^2*tau; S=H*Pz*H+RR; K=Pz*H/S; thz=thz+K*(axm(3)-ap); Pz=(1-K*H)*Pz; end
      end
      hbt=max((dot(p_curr,w)-pz)/R,1.001); [~,cpz,dz]=calc_correction_functions(hbt,true);
      aTz(k)=a_nom/cpz; apTz(k)=(1/R)*(-a_nom/cpz^2)*dz.dc_perp_dh;
      hq=max(hbar,1.05); ce=1+thz/(hq-1); dce=-thz/(hq-1)^2; aEz(k)=a_nom/ce; apEz(k)=(1/R)*(-a_nom/ce^2)*dce;
      tL(k)=t; hbmL(k)=hbar; axmzL(k)=axm(3);
      if P.thermal.enable>0.5; f_th=calc_thermal_force(p_curr,P); else f_th=zeros(3,1); end
      p_curr=step_dynamics(p_curr,f_d_k+f_th,P,Ts);
      if config.meas_noise_enable; nm=config.meas_noise_std(:).*randn(3,1); else nm=zeros(3,1); end
      p_m_buf=[p_m_buf(:,2:end),p_curr+nm]; pd_for_ctrl=pd_kp1;
    end

    % h/R-grid curves from final theta_z
    g=linspace(1.1,3.5,300).'; cpg=zeros(size(g)); dcg=zeros(size(g));
    for i=1:numel(g); [~,cpg(i),dq]=calc_correction_functions(g(i),true); dcg(i)=dq.dc_perp_dh; end
    aTg=a_nom./cpg; apTg=(1/R)*(-a_nom./cpg.^2).*dcg;
    cg=1+thz./(g-1); aEg=a_nom./cg; apEg=(1/R)*(-a_nom./cg.^2).*(-thz./(g-1).^2);

    % ===== current canonical style =====
    COL_TRUE=[0.8 0 0]; COL_EST=[0 0.2 0.9]; COL_MEAS=[0.45 0.55 0.95]; COL_ALT=[0.6 0.6 0.6];
    FS=18; LFS=13; AXLW=2.0; LW=2.0;
    osc=tL>=1; mv=hbmL>1;
    biasA =100*mean((aEz(osc)-aTz(osc))./aTz(osc));   sdA =100*std((aEz(osc)-aTz(osc))./aTz(osc));
    biasAp=100*mean((apEz(osc)-apTz(osc))./apTz(osc)); sdAp=100*std((apEz(osc)-apTz(osc))./apTz(osc));

    % ---- Fig1: a vs h/R ----
    f=figure('Position',[80 80 820 600],'Color','w','NumberTitle','off','Visible','off');
    s=scatter(hbmL(mv),axmzL(mv),9,COL_MEAS,'filled','DisplayName','Measured a_{xm}'); s.MarkerFaceAlpha=0.22; hold on;
    plot(g,aTg,'-','Color',COL_TRUE,'LineWidth',LW,'DisplayName','True');
    plot(g,aEg,'-','Color',COL_EST,'LineWidth',LW,'DisplayName','Estimated');
    xlabel('h / R','FontSize',FS,'FontWeight','bold'); ylabel('a  (\mum/pN)','FontSize',FS,'FontWeight','bold');
    legend('Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    ylim([0 0.025]); set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off;
    set(findall(f,'Type','Axes'),{'Toolbar'},{[]}); exportgraphics(f,fullfile(od,'fig_s2_a_vs_hbar.png'),'Resolution',150); close(f);

    % ---- Fig2: a' vs h/R ----
    f=figure('Position',[80 80 820 600],'Color','w','NumberTitle','off','Visible','off');
    plot(g,apTg,'-','Color',COL_TRUE,'LineWidth',LW,'DisplayName','True'); hold on;
    plot(g,apEg,'-','Color',COL_EST,'LineWidth',LW,'DisplayName','Estimated');
    xlabel('h / R','FontSize',FS,'FontWeight','bold'); ylabel('a'' = da/dh  ((\mum/pN)/\mum)','FontSize',FS,'FontWeight','bold');
    legend('Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off;
    set(findall(f,'Type','Axes'),{'Toolbar'},{[]}); exportgraphics(f,fullfile(od,'fig_s2_aprime_vs_hbar.png'),'Resolution',150); close(f);

    % ---- Fig3: a_z(t) ----
    f=figure('Position',[80 80 1100 560],'Color','w','NumberTitle','off','Visible','off');
    plot(tL,axmzL,'-','Color',COL_MEAS,'LineWidth',0.5,'DisplayName','Measured a_{xm}'); hold on;
    plot(tL,aTz,'-','Color',COL_TRUE,'LineWidth',LW,'DisplayName','True');
    plot(tL,aEz,'-','Color',COL_EST,'LineWidth',LW,'DisplayName','Estimated');
    xlabel('Time (sec)','FontSize',FS,'FontWeight','bold'); ylabel('a_z  (\mum/pN)','FontSize',FS,'FontWeight','bold');
    legend('Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    xlim([0 4]); ylim([0 0.025]); set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off;
    set(findall(f,'Type','Axes'),{'Toolbar'},{[]}); exportgraphics(f,fullfile(od,'fig_s2_a_time.png'),'Resolution',150); close(f);

    % ---- Fig4: a'_z(t) ----
    f=figure('Position',[80 80 1100 560],'Color','w','NumberTitle','off','Visible','off');
    plot(tL,apTz,'-','Color',COL_TRUE,'LineWidth',LW,'DisplayName','True'); hold on;
    plot(tL,apEz,'-','Color',COL_EST,'LineWidth',LW,'DisplayName','Estimated');
    xlabel('Time (sec)','FontSize',FS,'FontWeight','bold'); ylabel('a''_z = da/dh  ((\mum/pN)/\mum)','FontSize',FS,'FontWeight','bold');
    legend('Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    xlim([0 4]); set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off;
    set(findall(f,'Type','Axes'),{'Toolbar'},{[]}); exportgraphics(f,fullfile(od,'fig_s2_aprime_time.png'),'Resolution',150); close(f);

    % ---- Fig5: 1-param vs 2-param a' identifiability (open-loop, from saved .mat) ----
    cf1=fullfile(od,'result_5seed_corrected.mat'); cf2=fullfile(od,'result_1param.mat');
    if exist(cf1,'file') && exist(cf2,'file')
        A=load(cf1); B=load(cf2); TH=A.TH; P1=B.P1;
        f=figure('Position',[80 80 900 620],'Color','w','NumberTitle','off','Visible','off'); hold on;
        for sx=1:size(TH,1)
            ce=1+TH(sx,1)./(g-1)+TH(sx,2)./g; ap=(1/R)*(-a_nom./ce.^2).*(-TH(sx,1)./(g-1).^2-TH(sx,2)./g.^2);
            h2=plot(g,ap,'-','Color',COL_ALT,'LineWidth',1.0);
            if sx==1; set(h2,'DisplayName','2-param (5 seeds)'); else; set(h2,'HandleVisibility','off'); end
        end
        for sx=1:numel(P1)
            ce=1+P1(sx)./(g-1); ap=(1/R)*(-a_nom./ce.^2).*(-P1(sx)./(g-1).^2);
            h1=plot(g,ap,'-','Color',COL_EST,'LineWidth',1.3);
            if sx==1; set(h1,'DisplayName','1-param (5 seeds)'); else; set(h1,'HandleVisibility','off'); end
        end
        plot(g,apTg,'-','Color',COL_TRUE,'LineWidth',3.0,'DisplayName','True');
        xlabel('h / R','FontSize',FS,'FontWeight','bold'); ylabel('a'' = da/dh  ((\mum/pN)/\mum)','FontSize',FS,'FontWeight','bold');
        legend('Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        ylim([0 0.012]); set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off;
        set(findall(f,'Type','Axes'),{'Toolbar'},{[]}); exportgraphics(f,fullfile(od,'fig_1param_vs_2param.png'),'Resolution',150); close(f);
        fprintf('fig_1param_vs_2param regenerated (new style).\n');
    else
        fprintf('SKIP fig_1param_vs_2param: missing %s / %s (run the Stage-1 5-seed fits first).\n', cf1, cf2);
    end

    fprintf('NO-title style (true=red, est=blue). console stats: a bias %+.1f%% std %.1f%% | a'' bias %+.1f%% std %.1f%%\n',biasA,sdA,biasAp,sdAp);
    fprintf('saved figs to %s\n',od);
end
