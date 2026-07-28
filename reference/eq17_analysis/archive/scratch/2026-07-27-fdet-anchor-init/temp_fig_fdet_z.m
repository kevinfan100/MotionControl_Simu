%TEMP_FIG_FDET_Z  z-axis only: what fdet does, in both scenarios.
%   1  positioning 12 s : a_hat/a_true(t), fdet OFF vs ON, 6 seeds
%   2  positioning 12 s : cumulative y1 contribution to a_hat
%   3  osc 4.8 s        : a_z  true / hat(OFF) / hat(ON), seed 7
%   4  osc 4.8 s        : a_hat relative error, OFF vs ON, 6 seeds
clear; clc;
here = fileparts(mfilename('fullpath'));  addpath(genpath(fullfile(here,'..')));
pc = physical_constants();  az=3; seeds=[7 11 23 42 101 777];
base = user_config();
base.h_min=1.05*pc.R; base.ctrl_enable=true; base.thermal_enable=true;
base.meas_noise_enable=true; base.lambda_c=0.7; base.a_pd=0.05; base.a_cov=0.05;
base.meas_noise_std=[0.00062;0.00057;0.00331]; base.h_bar_safe=1.5; base.Pf_a_frac=0.03;

pos = base; pos.trajectory_type='positioning'; pos.h_init=50; pos.T_sim=12;
osc = base; osc.trajectory_type='osc'; osc.h_init=50; osc.h_bottom=4.5;
osc.amplitude=2.5; osc.frequency=1; osc.n_cycles=2; osc.t_hold=0.5;
osc.t_descend_override=1.0; osc.T_sim=4.8;

P={}; C={}; O={}; OE={};
for fd=[0 1]
  c=pos; c.fe_use_fdet=fd; Rr=[]; Cc=[];
  for q=1:6
    s=temp_run_powerlaw_diag(c,struct('seed',seeds(q),'verbose',false));
    Rr(:,q)=s.a_hat_out(:,az)./s.a_true_out(:,az); Cc(:,q)=1e3*cumsum(s.DG.dg_dA_y1(:,az));
    tp=s.tout(:);
  end
  P{fd+1}=Rr; C{fd+1}=Cc;
  c=osc; c.fe_use_fdet=fd; Aa=[]; Ee=[];
  for q=1:6
    s=temp_run_powerlaw_diag(c,struct('seed',seeds(q),'verbose',false));
    Aa(:,q)=s.a_hat_out(:,az); Ee(:,q)=100*(s.a_hat_out(:,az)-s.a_true_out(:,az))./s.a_true_out(:,az);
    to=s.tout(:); aT=s.a_true_out(:,az);
  end
  O{fd+1}=Aa; OE{fd+1}=Ee;
end

FS=18; gy=[0.55 0.55 0.55]; gyl=[0.80 0.80 0.80]; bl=[0 0 1]; bll=[0.62 0.72 0.98];
fig=figure('Color','w','Position',[60 40 960 1200]);

ax1=subplot(4,1,1); hold(ax1,'on');
for q=1:6; plot(ax1,tp,P{1}(:,q),'Color',gyl,'LineWidth',0.8,'HandleVisibility','off'); end
for q=1:6; plot(ax1,tp,P{2}(:,q),'Color',bll,'LineWidth',0.8,'HandleVisibility','off'); end
h1=plot(ax1,tp,P{1}(:,1),'Color',gy,'LineWidth',2.2);
h2=plot(ax1,tp,P{2}(:,1),'Color',bl,'LineWidth',2.2);
h3=plot(ax1,tp,ones(size(tp)),'r--','LineWidth',1.6);
ylabel(ax1,'a_{hat}/a_{true}','FontSize',FS,'FontWeight','bold');
legend([h3 h1 h2],{'truth','fdet OFF','fdet ON'},'Location','northoutside', ...
       'Orientation','horizontal','FontSize',FS-5);
ylim(ax1,[0.75 1.10]); title(ax1,''); sty(ax1,FS,[0 12]);
text(ax1,6,0.80,'定位 12 s','FontSize',FS-6,'FontWeight','bold','Color',gy,'HorizontalAlignment','center');

ax2=subplot(4,1,2); hold(ax2,'on');
g1=plot(ax2,tp,mean(C{1},2),'Color',gy,'LineWidth',2.4);
g2=plot(ax2,tp,mean(C{2},2),'Color',bl,'LineWidth',2.4);
plot(ax2,tp,zeros(size(tp)),'r--','LineWidth',1.4,'HandleVisibility','off');
ylabel(ax2,{'y_1 累積貢獻','[nm/pN]'},'FontSize',FS-2,'FontWeight','bold');
legend([g1 g2],{'fdet OFF','fdet ON'},'Location','northoutside', ...
       'Orientation','horizontal','FontSize',FS-5);
ylim(ax2,[-4 1]); sty(ax2,FS,[0 12]); xlabel(ax2,'time [s]','FontSize',FS-2,'FontWeight','bold');

ax3=subplot(4,1,3); hold(ax3,'on');
hr=plot(ax3,to,aT*1e3,'r','LineWidth',2.6);
ho=plot(ax3,to,O{1}(:,1)*1e3,'Color',gy,'LineWidth',1.9);
hn=plot(ax3,to,O{2}(:,1)*1e3,'Color',bl,'LineWidth',1.9);
ylabel(ax3,'a_z  [nm/pN]','FontSize',FS,'FontWeight','bold');
legend([hr ho hn],{'a_{true}','fdet OFF','fdet ON'},'Location','northoutside', ...
       'Orientation','horizontal','FontSize',FS-5);
ylim(ax3,[4 17]); sty(ax3,FS,[0 4.8]);
text(ax3,2.4,5.2,'振盪場景 (seed 7)','FontSize',FS-6,'FontWeight','bold','Color',gy,'HorizontalAlignment','center');

ax4=subplot(4,1,4); hold(ax4,'on');
for q=1:6; plot(ax4,to,OE{1}(:,q),'Color',gyl,'LineWidth',0.8,'HandleVisibility','off'); end
for q=1:6; plot(ax4,to,OE{2}(:,q),'Color',bll,'LineWidth',0.8,'HandleVisibility','off'); end
e1=plot(ax4,to,mean(OE{1},2),'Color',gy,'LineWidth',2.4);
e2=plot(ax4,to,mean(OE{2},2),'Color',bl,'LineWidth',2.4);
plot(ax4,to,zeros(size(to)),'r--','LineWidth',1.4,'HandleVisibility','off');
ylabel(ax4,'a_{hat} error [%]','FontSize',FS,'FontWeight','bold');
xlabel(ax4,'time  [s]','FontSize',FS,'FontWeight','bold');
legend([e1 e2],{'fdet OFF (6 seed 平均)','fdet ON'},'Location','northoutside', ...
       'Orientation','horizontal','FontSize',FS-5);
ylim(ax4,[-20 12]); sty(ax4,FS,[0 4.8]);

outp=fullfile(here,'temp_fdet_z.png');
exportgraphics(fig,outp,'Resolution',140); close(fig);
fprintf('FIGURE saved: %s\n',outp);
function sty(ax,FS,xl)
    set(ax,'FontSize',FS-5,'FontWeight','bold','LineWidth',1.3,'Box','on');
    grid(ax,'off'); xlim(ax,xl);
end
