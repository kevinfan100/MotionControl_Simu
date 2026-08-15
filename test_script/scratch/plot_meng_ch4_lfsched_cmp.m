% FORK OF plot_meng_ch4_gain_cmp.m @ 8899dc2 | PURPOSE: ledger 31 pages -- const vs scheduled lambda_f + lambda_f(t) trace | EXPIRES: with lf_schedule flag
% formC-style pages: LEFT = const lambda_f (whiten), RIGHT = lf_schedule.
% Plus a lambda_f(t) trace page. Loads lfsched_data.mat (arms 1=CONST 2=SCHED).
here0 = fileparts(mfilename('fullpath')); cd(fileparts(fileparts(here0)));
L=load('/Users/kevin/.claude/jobs/8581427c/tmp/lfsched_data.mat');
D=L.D; t=L.t; seeds=L.seeds;
COL_TRUE=[0.8 0 0]; COL_HAT=[0 0.2 0.9]; COL_MEAS=[0.45 0.72 0.95]; BANDC=[0.45 0.55 0.95];
FS=18; LFS=13; AXLW=2.0; LW=2.0;
NAME={'\lambda_f const','\lambda_f scheduled'};
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
    exportgraphics(f,fullfile(od,sprintf('meng_lfsched_cmp_ax%d_s%02d.png',ax,seeds(is))),'Resolution',150);
    close(f);
  end
end
% lambda_f(t) trace page: rows = ax1, ax3; SCHED seed 7 and 11
f=figure('Position',[40 40 1500 700],'Color','w','NumberTitle','off','Visible','off');
tl=tiledlayout(f,2,1,'TileSpacing','compact','Padding','compact');
CONST_LF=[0.9995 0.9995 0.999];
for r=1:2
  ax=[1 3]; ax=ax(r);
  a=nexttile(tl,r); hold(a,'on');
  h1=plot(a,t,D{2,1}.LF(:,ax),'-','Color',COL_HAT,'LineWidth',1.5,'DisplayName','scheduled, seed 7');
  h2=plot(a,t,D{2,2}.LF(:,ax),'-','Color',COL_MEAS,'LineWidth',1.5,'DisplayName','scheduled, seed 11');
  h3=yline(a,CONST_LF(ax),'--','Color',COL_TRUE,'LineWidth',2.0,'DisplayName','const');
  legend(a,[h1 h2 h3],'Location','northoutside','Orientation','horizontal', ...
         'FontSize',LFS,'FontWeight','bold','Box','on');
  ylabel(a,sprintf('\\lambda_f  (ax %d)',ax),'FontSize',FS,'FontWeight','bold');
  if r==2; xlabel(a,'time  [s]','FontSize',FS,'FontWeight','bold'); end
  xlim(a,[t(1) t(end)]); ylim(a,[0.9945 1.0002]);
  set(a,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on');
  grid(a,'off');
end
exportgraphics(f,fullfile(od,'meng_lfsched_trace.png'),'Resolution',150);
close(f);
fprintf('PAGES DONE\n');

function Lm = local_lim(v, pad)
    lo=min(v); hi=max(v); r=hi-lo; if r<=0; r=1; end
    Lm=[lo-pad*r, hi+pad*r];
end
