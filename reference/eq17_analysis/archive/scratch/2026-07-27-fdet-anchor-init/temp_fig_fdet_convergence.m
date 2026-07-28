%TEMP_FIG_FDET_CONVERGENCE  Does the estimator FIND a_h, or does it just sit on
%   a cheating init?  Three realizable/cheating inits, fdet OFF vs ON.
%   1  positioning 12 s, fdet OFF : all three inits drift away
%   2  positioning 12 s, fdet ON  : all three converge to truth (same y scale)
%   3  osc 4.8 s                  : three inits collapse onto one curve per arm
clear; clc;
here = fileparts(mfilename('fullpath'));  addpath(genpath(fullfile(here,'..')));
pc = physical_constants();  az=3; seeds=[7 11 23 42 101 777];
base = user_config();
base.h_min=1.05*pc.R; base.ctrl_enable=true; base.thermal_enable=true;
base.meas_noise_enable=true; base.lambda_c=0.7; base.a_pd=0.05; base.a_cov=0.05;
base.meas_noise_std=[0.00062;0.00057;0.00331]; base.h_bar_safe=1.5;

% name, a_init_honest, a_init_asym, Pf_a_frac, colour
IN = { 'peek  a_nom/c   (作弊)', 0,0,0.03, [0.50 0.50 0.50]; ...
       'flat  a_nom     (誠實)', 1,0,0.06, [0.00 0.60 0.00]; ...
       'asym  1+(9/8)/h (誠實)', 0,1,0.01, [0.00 0.00 1.00] };

pos=base; pos.trajectory_type='positioning'; pos.h_init=50; pos.T_sim=12;
osc=base; osc.trajectory_type='osc'; osc.h_init=50; osc.h_bottom=4.5;
osc.amplitude=2.5; osc.frequency=1; osc.n_cycles=2; osc.t_hold=0.5;
osc.t_descend_override=1.0; osc.T_sim=4.8;

Ppos=cell(3,2); Posc=cell(3,2);
for ii=1:3
  for fd=[0 1]
    c=pos; c.a_init_honest=IN{ii,2}; c.a_init_asym=IN{ii,3};
    c.Pf_a_frac=IN{ii,4}; c.fe_use_fdet=fd;
    M=[];
    for q=1:6
      s=temp_run_powerlaw_diag(c,struct('seed',seeds(q),'verbose',false));
      M(:,q)=s.a_hat_out(:,az)./s.a_true_out(:,az); tp=s.tout(:);
    end
    Ppos{ii,fd+1}=mean(M,2);
    c=osc; c.a_init_honest=IN{ii,2}; c.a_init_asym=IN{ii,3};
    c.Pf_a_frac=IN{ii,4}; c.fe_use_fdet=fd;
    M=[];
    for q=1:6
      s=temp_run_powerlaw_diag(c,struct('seed',seeds(q),'verbose',false));
      M(:,q)=s.a_hat_out(:,az)./s.a_true_out(:,az); to=s.tout(:);
    end
    Posc{ii,fd+1}=mean(M,2);
  end
end

FS=18; yl=[0.75 1.10];
fig=figure('Color','w','Position',[60 40 960 1150]);

ax1=subplot(3,1,1); hold(ax1,'on');
hh=gobjects(3,1);
for ii=1:3; hh(ii)=plot(ax1,tp,Ppos{ii,1},'Color',IN{ii,5},'LineWidth',2.4); end
plot(ax1,tp,ones(size(tp)),'r--','LineWidth',1.6);
ylabel(ax1,'a_{hat}/a_{true}','FontSize',FS,'FontWeight','bold');
legend(hh,IN(:,1),'Location','northoutside','Orientation','horizontal','FontSize',FS-7);
ylim(ax1,yl); sty(ax1,FS,[0 12]);
text(ax1,6,0.79,'定位 12 s   fdet OFF (現況)','FontSize',FS-5,'FontWeight','bold', ...
     'HorizontalAlignment','center','Color',[0.35 0.35 0.35]);

ax2=subplot(3,1,2); hold(ax2,'on');
for ii=1:3; plot(ax2,tp,Ppos{ii,2},'Color',IN{ii,5},'LineWidth',2.4); end
plot(ax2,tp,ones(size(tp)),'r--','LineWidth',1.6);
ylabel(ax2,'a_{hat}/a_{true}','FontSize',FS,'FontWeight','bold');
ylim(ax2,yl); sty(ax2,FS,[0 12]);
text(ax2,6,0.79,'定位 12 s   fdet ON   (同一個 y 尺度)','FontSize',FS-5, ...
     'FontWeight','bold','HorizontalAlignment','center','Color',[0.35 0.35 0.35]);
xlabel(ax2,'time [s]','FontSize',FS-2,'FontWeight','bold');

ax3=subplot(3,1,3); hold(ax3,'on');
for ii=1:3
  plot(ax3,to,Posc{ii,1},'Color',IN{ii,5},'LineWidth',2.0,'LineStyle','--');
  plot(ax3,to,Posc{ii,2},'Color',IN{ii,5},'LineWidth',2.4);
end
plot(ax3,to,ones(size(to)),'r--','LineWidth',1.6);
ylabel(ax3,'a_{hat}/a_{true}','FontSize',FS,'FontWeight','bold');
xlabel(ax3,'time  [s]','FontSize',FS,'FontWeight','bold');
ylim(ax3,[0.80 1.10]); sty(ax3,FS,[0 4.8]);
text(ax3,2.4,0.83,'振盪場景   虛線 = fdet OFF, 實線 = fdet ON', ...
     'FontSize',FS-5,'FontWeight','bold','HorizontalAlignment','center','Color',[0.35 0.35 0.35]);

outp=fullfile(here,'temp_fdet_convergence.png');
exportgraphics(fig,outp,'Resolution',140); close(fig);
fprintf('FIGURE saved: %s\n',outp);
function sty(ax,FS,xl)
    set(ax,'FontSize',FS-5,'FontWeight','bold','LineWidth',1.3,'Box','on');
    grid(ax,'off'); xlim(ax,xl);
end
