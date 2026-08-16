% FORK OF plot_formC_dist_compare.m style | PURPOSE: ledger 38 Scenario-B pages (Fig 8/9 counterparts) | EXPIRES: with run_meng_ch4_sB.m
% Scenario B pages (formC style):
%  page 1 delta: rows x/y/z [um], LEFT = arm N (nominal), RIGHT = E98 -- Fig 8 counterpart
%  page 2 az:    rows track/err%/e_a band, LEFT = E98, RIGHT = ESCH -- Fig 9 counterpart
here0 = fileparts(mfilename('fullpath')); cd(fileparts(fileparts(here0)));
L=load(fullfile('test_results','meng_ch4_s0','meng_sB_run.mat'));
D=L.D; t=L.t(1:end-1); pd=L.pd_arr;
COL_TRUE=[0.8 0 0]; COL_HAT=[0 0.2 0.9]; BANDC=[0.45 0.55 0.95];
FS=18; LFS=13; AXLW=2.0; LW=1.2;
od='test_results/meng_ch4_s0';
% ---- page 1: delta, N | E98, seed 7 ----
S={D.N_s7, D.E98_s7}; NM={'nominal a_N (arm N)','estimated (E98)'};
axn='xyz';
f=figure('Position',[40 40 1500 1050],'Color','w','Visible','off');
tl=tiledlayout(f,3,2,'TileSpacing','compact','Padding','compact');
for ax=1:3
  yl=[0;0];
  vv=[S{1}.dx(:,ax); S{2}.dx(:,ax)]; r=max(vv)-min(vv);
  yl=[min(vv)-0.05*r, max(vv)+0.05*r];
  for c=1:2
    a=nexttile(tl,(ax-1)*2+c); hold(a,'on');
    plot(a,t,S{c}.dx(:,ax),'-','Color',COL_HAT,'LineWidth',LW, ...
         'DisplayName',sprintf('\\delta%c  %s  seed 7',axn(ax),NM{c}));
    yline(a,0,'-','Color',[0.4 0.4 0.4],'LineWidth',1.0,'HandleVisibility','off');
    if ax==1
      legend(a,'Location','northoutside','Orientation','horizontal', ...
             'FontSize',LFS,'FontWeight','bold','Box','on');
    end
    xlim(a,[0 3]); ylim(a,yl); box(a,'on');
    if c==1; ylabel(a,sprintf('\\delta%c  [\\mum]',axn(ax)),'FontSize',FS,'FontWeight','bold'); end
    if ax==3; xlabel(a,'time  [s]','FontSize',FS,'FontWeight','bold'); end
    set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on');
    if ax<3; set(a,'XTickLabel',[]); end
    if c==2; set(a,'YTickLabel',[]); end
    grid(a,'off');
  end
end
exportgraphics(f,fullfile(od,'meng_sB_delta_NvsE.png'),'Resolution',150); close(f);
% ---- page 2: az tracking, E98 | ESCH, seed 7 ----
S={D.E98_s7, D.ESCH_s7}; NM={'\lambda_f = 0.98 const','\lambda_f scheduled'};
f=figure('Position',[40 40 1500 1050],'Color','w','Visible','off');
tl=tiledlayout(f,3,2,'TileSpacing','compact','Padding','compact');
YL1=[0 0.008]; YL2=[-40 40]; YL3=[-2e-3 2e-3];
for row=1:3
  for c=1:2
    Sx=S{c}; a=nexttile(tl,(row-1)*2+c); hold(a,'on');
    e=100*(Sx.AH(:,3)-Sx.AT(:,3))./Sx.AT(:,3);
    switch row
      case 1
        h1=plot(a,t,Sx.AT(:,3),'-','Color',COL_TRUE,'LineWidth',2.6,'DisplayName','a_{true}');
        h2=plot(a,t,Sx.AH(:,3),'-','Color',COL_HAT,'LineWidth',1.4, ...
                'DisplayName',sprintf('a_{hat}  %s  seed 7',NM{c}));
        legend(a,[h1 h2],'Location','northoutside','Orientation','horizontal', ...
               'FontSize',LFS,'FontWeight','bold','Box','on');
        ylim(a,YL1);
        if c==1; ylabel(a,'a_z  [\mum/pN]','FontSize',FS,'FontWeight','bold'); end
      case 2
        yline(a,0,'-','Color',[0.4 0.4 0.4],'LineWidth',1.0,'HandleVisibility','off');
        plot(a,t,e,'-','Color',COL_HAT,'LineWidth',1.2);
        ylim(a,YL2);
        if c==1; ylabel(a,'a_z  error  [%]','FontSize',FS,'FontWeight','bold'); end
      case 3
        sd=sqrt(Sx.PA(:,3)); ea=Sx.AH(:,3)-Sx.AT(:,3);
        fill(a,[t; flipud(t)],[ea+sd; flipud(ea-sd)],BANDC, ...
             'EdgeColor','none','FaceAlpha',0.30,'DisplayName','\pm sqrt(P_{66})');
        plot(a,t,ea,'-','Color',COL_HAT,'LineWidth',1.2,'DisplayName','e_a');
        yline(a,0,'-','Color',COL_TRUE,'LineWidth',2.0,'DisplayName','zero');
        legend(a,'Location','northoutside','Orientation','horizontal', ...
               'FontSize',LFS,'FontWeight','bold','Box','on');
        ylim(a,YL3);
        if c==1; ylabel(a,'e_a  [\mum/pN]','FontSize',FS,'FontWeight','bold'); end
        xlabel(a,'time  [s]','FontSize',FS,'FontWeight','bold');
    end
    xlim(a,[0 3]); box(a,'on');
    set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on');
    if row<3; set(a,'XTickLabel',[]); end
    if c==2; set(a,'YTickLabel',[]); end
    grid(a,'off');
  end
end
exportgraphics(f,fullfile(od,'meng_sB_az_track.png'),'Resolution',150); close(f);
fprintf('SBPAGES DONE\n');
