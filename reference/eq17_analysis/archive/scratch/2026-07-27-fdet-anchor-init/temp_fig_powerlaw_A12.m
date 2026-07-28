%TEMP_FIG_POWERLAW_A12  Before / after the A-level fixes 1 (whitened y2 + R2)
%   and 2 (p prior = power-law slope residual).
%   BEFORE = scratch fork with y2_whiten=0, Pf_p_std=0.3 (the shipped behaviour
%            before this change).  AFTER = the PRODUCTION controller.
%   Panels: a_z (after) / p_hat raw BEFORE / p_hat raw AFTER (same y scale) /
%           claimed vs realised sigma_p.
%   Scratch; delete with the rest of this round's temp files.

clear; clc;
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..');
addpath(genpath(proj));

cfg = user_config();
cfg.trajectory_type = 'osc';
cfg.h_init = 50;  cfg.h_bottom = 4.5;  cfg.amplitude = 2.5;
cfg.frequency = 1;  cfg.n_cycles = 2;
cfg.t_hold = 0.5;  cfg.t_descend_override = 1.0;  cfg.T_sim = 4.8;
pc = physical_constants();
cfg.h_min = 1.05 * pc.R;
cfg.ctrl_enable = true;  cfg.thermal_enable = true;  cfg.meas_noise_enable = true;
cfg.lambda_c = 0.7;  cfg.a_pd = 0.05;  cfg.a_cov = 0.05;
cfg.meas_noise_std = [0.00062; 0.00057; 0.00331];
cfg.h_bar_safe = 1.5;  cfg.Pf_a_frac = 0.03;
az = 3;  seeds = [7 11 23 42 101 777];  ns = numel(seeds);

cB = cfg;  cB.y2_whiten = 0;  cB.Pf_p_std = 0.3;      % BEFORE
PB = [];  SB = [];
for q = 1:ns
    s = temp_run_powerlaw_diag(cB, struct('seed', seeds(q), 'verbose', false));
    PB(:,q) = s.p_hat_out(:,az);  SB(:,q) = s.P_p_out(:,az); %#ok<AGROW>
end
PA = [];  SA = [];  sA = [];
for q = 1:ns
    s = run_5state_powerlaw(cfg, struct('seed', seeds(q), 'verbose', false)); % PRODUCTION
    PA(:,q) = s.p_hat_out(:,az);  SA(:,q) = s.P_p_out(:,az); %#ok<AGROW>
    if q == 1; sA = s; end
end

t = sA.tout(:);  P = sA.meta.params_value;
w = P.wall.w_hat(:);  pz = P.wall.pz;  R = P.common.R;
hb_d = (sA.p_d_out * w - pz) / R;
p_eff = nan(numel(t),1);
for i = 1:numel(t)
    hbv = max(hb_d(i), 1.001);
    [~, cpe, dv] = calc_correction_functions(hbv, true);
    p_eff(i) = -dv.dc_perp_dh * (hbv - 1) / max(cpe - 1, eps);
end

FS = 18;  nm = 1e3;  faint = [0.55 0.65 0.95];
tb = [cfg.t_hold, cfg.t_hold + cfg.t_descend_override];
fig = figure('Color','w','Position',[60 40 960 1240]);

% (1) a_z, production
ax1 = subplot(4,1,1); hold(ax1,'on');
hx = plot(ax1, t, sA.a_xm_out(:,az)*nm, 'Color',[0.6 0.6 0.6],'LineWidth',0.5);
ht = plot(ax1, t, sA.a_true_out(:,az)*nm, 'r','LineWidth',2.6);
he = plot(ax1, t, sA.a_hat_out(:,az)*nm, 'b','LineWidth',2.0);
ylabel(ax1,'a_z  [nm/pN]','FontSize',FS,'FontWeight','bold');
legend([ht he hx],{'a_{true}  seed 7','a_{hat}  seed 7 (after)','a_{xm} raw (clipped)'}, ...
       'Location','northoutside','Orientation','horizontal','FontSize',FS-5);
% y range matched to a_true / a_hat so the estimate detail is readable;
% a_xm raw actually spans 0-40 nm/pN and is clipped here on purpose.
ylim(ax1,[4 17]); sty(ax1,FS,t,tb);

% (2) p_hat BEFORE
ax2 = subplot(4,1,2); hold(ax2,'on');
for q = 1:ns
    plot(ax2, t, PB(:,q), 'Color',faint,'LineWidth',1.0,'HandleVisibility','off');
end
hb1 = plot(ax2, t, PB(:,1), 'b','LineWidth',2.0);
hr1 = plot(ax2, t, p_eff, 'r','LineWidth',2.6);
ylabel(ax2,'p_z  BEFORE','FontSize',FS,'FontWeight','bold');
legend([hr1 hb1],{'p_{eff} true','p_{hat}  seed 7 (5 more faint)'},'Location','northoutside', ...
       'Orientation','horizontal','FontSize',FS-5);
ylim(ax2,[-0.1 1.7]); sty(ax2,FS,t,tb);

% (3) p_hat AFTER, same y scale
ax3 = subplot(4,1,3); hold(ax3,'on');
for q = 1:ns
    plot(ax3, t, PA(:,q), 'Color',faint,'LineWidth',1.0,'HandleVisibility','off');
end
hb2 = plot(ax3, t, PA(:,1), 'b','LineWidth',2.0);
hr2 = plot(ax3, t, p_eff, 'r','LineWidth',2.6);
ylabel(ax3,'p_z  AFTER','FontSize',FS,'FontWeight','bold');
legend([hr2 hb2],{'p_{eff} true','p_{hat}  seed 7 (5 more faint)'},'Location','northoutside', ...
       'Orientation','horizontal','FontSize',FS-5);
ylim(ax3,[-0.1 1.7]); sty(ax3,FS,t,tb);

% (4) claimed vs realised sigma_p
ax4 = subplot(4,1,4); hold(ax4,'on');
h1 = plot(ax4, t, std(PB,0,2), 'r--','LineWidth',2.2);
h2 = plot(ax4, t, mean(SB,2),  'b--','LineWidth',1.8);
h3 = plot(ax4, t, std(PA,0,2), 'r','LineWidth',2.6);
h4 = plot(ax4, t, mean(SA,2),  'b','LineWidth',2.0);
ylabel(ax4,'\sigma_p','FontSize',FS,'FontWeight','bold');
xlabel(ax4,'time  [s]','FontSize',FS,'FontWeight','bold');
lg4 = legend([h1 h3 h2 h4], {'realised BEFORE','realised AFTER', ...
        'claimed BEFORE','claimed AFTER'}, ...
       'Location','northoutside','FontSize',FS-6);
lg4.NumColumns = 2;
ylabel(ax4,'\sigma_p   (6-seed)','FontSize',FS,'FontWeight','bold');
ylim(ax4,[0 0.62]); sty(ax4,FS,t,tb);

outp = fullfile(here, 'temp_powerlaw_A12_beforeafter.png');
exportgraphics(fig, outp, 'Resolution', 140);
close(fig);
fprintf('FIGURE saved: %s\n', outp);
fprintf('BEFORE: descent p_hat sd %.4f, osc %.4f +- %.4f\n', ...
        mean(std(PB(t>0.5 & t<1.5,:),0,1)), mean(mean(PB(t>1.6,:),1)), ...
        std(mean(PB(t>1.6,:),1)));
fprintf('AFTER : descent p_hat sd %.4f, osc %.4f +- %.4f   (true p_eff ~ 1.003)\n', ...
        mean(std(PA(t>0.5 & t<1.5,:),0,1)), mean(mean(PA(t>1.6,:),1)), ...
        std(mean(PA(t>1.6,:),1)));

function sty(ax, FS, t, tb)
    set(ax,'FontSize',FS-4,'FontWeight','bold','LineWidth',1.3,'Box','on');
    grid(ax,'off'); xlim(ax,[t(1) t(end)]);
    yl = ylim(ax);
    for x = tb
        plot(ax,[x x],yl,'--','Color',[0.55 0.55 0.55],'LineWidth',1.0,'HandleVisibility','off');
    end
    ylim(ax, yl);
end
