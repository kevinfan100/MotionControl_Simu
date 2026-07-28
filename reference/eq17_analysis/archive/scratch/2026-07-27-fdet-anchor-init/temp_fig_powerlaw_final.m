%TEMP_FIG_POWERLAW_FINAL  Production before/after the fdet + asymptotic-init change.
%   BEFORE = commit ae41395 behaviour (peek init a_nom/c, realised f_d in F_e),
%            reproduced with the scratch fork.
%   AFTER  = the production controller as it now stands, no overrides.
%   1  positioning 12 s : a_hat/a_true, 6 seeds
%   2  osc 4.8 s        : a_z  true / before / after, seed 7
%   3  osc 4.8 s        : a_hat relative error, 6-seed mean + spread
clear; clc;
here = fileparts(mfilename('fullpath'));  addpath(genpath(fullfile(here,'..')));
pc = physical_constants(); az=3; seeds=[7 11 23 42 101 777];
base = user_config();
base.h_min=1.05*pc.R; base.ctrl_enable=true; base.thermal_enable=true;
base.meas_noise_enable=true; base.lambda_c=0.7; base.a_pd=0.05; base.a_cov=0.05;
base.meas_noise_std=[0.00062;0.00057;0.00331]; base.h_bar_safe=1.5;

pos=base; pos.trajectory_type='positioning'; pos.h_init=50; pos.T_sim=12;
osc=base; osc.trajectory_type='osc'; osc.h_init=50; osc.h_bottom=4.5;
osc.amplitude=2.5; osc.frequency=1; osc.n_cycles=2; osc.t_hold=0.5;
osc.t_descend_override=1.0; osc.T_sim=4.8;

% BEFORE (ae41395): peek init, Pf_a 0.03, realised f_d
bp=pos; bp.fe_use_fdet=0; bp.a_init_honest=0; bp.a_init_asym=0; bp.Pf_a_frac=0.03;
bo=osc; bo.fe_use_fdet=0; bo.a_init_honest=0; bo.a_init_asym=0; bo.Pf_a_frac=0.03;

Pb=[];Pa=[];Ob=[];Oa=[];Eb=[];Ea=[];
for q=1:6
    s=temp_run_powerlaw_diag(bp,struct('seed',seeds(q),'verbose',false));
    Pb(:,q)=s.a_hat_out(:,az)./s.a_true_out(:,az); tp=s.tout(:); %#ok<*AGROW>
    s=run_5state_powerlaw(pos,struct('seed',seeds(q),'verbose',false));
    Pa(:,q)=s.a_hat_out(:,az)./s.a_true_out(:,az);
    s=temp_run_powerlaw_diag(bo,struct('seed',seeds(q),'verbose',false));
    Ob(:,q)=s.a_hat_out(:,az); Eb(:,q)=100*(s.a_hat_out(:,az)-s.a_true_out(:,az))./s.a_true_out(:,az);
    to=s.tout(:); aT=s.a_true_out(:,az);
    s=run_5state_powerlaw(osc,struct('seed',seeds(q),'verbose',false));
    Oa(:,q)=s.a_hat_out(:,az); Ea(:,q)=100*(s.a_hat_out(:,az)-s.a_true_out(:,az))./s.a_true_out(:,az);
end

FS=18; gy=[0.50 0.50 0.50]; gyl=[0.80 0.80 0.80]; bl=[0 0 1]; bll=[0.62 0.72 0.98];
fig=figure('Color','w','Position',[60 40 960 1050]);

ax1=subplot(3,1,1); hold(ax1,'on');
for q=1:6; plot(ax1,tp,Pb(:,q),'Color',gyl,'LineWidth',0.8,'HandleVisibility','off'); end
for q=1:6; plot(ax1,tp,Pa(:,q),'Color',bll,'LineWidth',0.8,'HandleVisibility','off'); end
h1=plot(ax1,tp,mean(Pb,2),'Color',gy,'LineWidth',2.4);
h2=plot(ax1,tp,mean(Pa,2),'Color',bl,'LineWidth',2.4);
h3=plot(ax1,tp,ones(size(tp)),'r--','LineWidth',1.6);
ylabel(ax1,'a_{hat}/a_{true}','FontSize',FS,'FontWeight','bold');
legend([h3 h1 h2],{'truth','before (ae41395)','after (fdet + asym init)'}, ...
       'Location','northoutside','Orientation','horizontal','FontSize',FS-6);
ylim(ax1,[0.75 1.10]); sty(ax1,FS,[0 12]);
text(ax1,6,0.79,'純定位 12 s','FontSize',FS-6,'FontWeight','bold', ...
     'HorizontalAlignment','center','Color',gy);
xlabel(ax1,'time [s]','FontSize',FS-3,'FontWeight','bold');

ax2=subplot(3,1,2); hold(ax2,'on');
hr=plot(ax2,to,aT*1e3,'r','LineWidth',2.6);
hb=plot(ax2,to,Ob(:,1)*1e3,'Color',gy,'LineWidth',1.9);
ha=plot(ax2,to,Oa(:,1)*1e3,'Color',bl,'LineWidth',1.9);
ylabel(ax2,'a_z  [nm/pN]','FontSize',FS,'FontWeight','bold');
legend([hr hb ha],{'a_{true}','before','after'},'Location','northoutside', ...
       'Orientation','horizontal','FontSize',FS-6);
ylim(ax2,[4 17]); sty(ax2,FS,[0 4.8]);
text(ax2,2.4,5.0,'振盪場景 (seed 7)','FontSize',FS-6,'FontWeight','bold', ...
     'HorizontalAlignment','center','Color',gy);

ax3=subplot(3,1,3); hold(ax3,'on');
for q=1:6; plot(ax3,to,Eb(:,q),'Color',gyl,'LineWidth',0.8,'HandleVisibility','off'); end
for q=1:6; plot(ax3,to,Ea(:,q),'Color',bll,'LineWidth',0.8,'HandleVisibility','off'); end
e1=plot(ax3,to,mean(Eb,2),'Color',gy,'LineWidth',2.4);
e2=plot(ax3,to,mean(Ea,2),'Color',bl,'LineWidth',2.4);
plot(ax3,to,zeros(size(to)),'r--','LineWidth',1.4,'HandleVisibility','off');
ylabel(ax3,'a_{hat} error [%]','FontSize',FS,'FontWeight','bold');
xlabel(ax3,'time  [s]','FontSize',FS,'FontWeight','bold');
legend([e1 e2],{'before (6 seed)','after (6 seed)'},'Location','northoutside', ...
       'Orientation','horizontal','FontSize',FS-6);
ylim(ax3,[-18 12]); sty(ax3,FS,[0 4.8]);

outp=fullfile(here,'temp_powerlaw_final.png');
exportgraphics(fig,outp,'Resolution',140); close(fig);
fprintf('FIGURE saved: %s\n',outp);
fprintf('定位 12 s : before %.4f -> after %.4f\n', mean(Pb(end,:)), mean(Pa(end,:)));
fprintf('振盪 osc  : before %.4f -> after %.4f\n', ...
        mean(mean(Ob(to>1.6,:)./aT(to>1.6),1)), mean(mean(Oa(to>1.6,:)./aT(to>1.6),1)));
function sty(ax,FS,xl)
    set(ax,'FontSize',FS-5,'FontWeight','bold','LineWidth',1.3,'Box','on');
    grid(ax,'off'); xlim(ax,xl);
end
