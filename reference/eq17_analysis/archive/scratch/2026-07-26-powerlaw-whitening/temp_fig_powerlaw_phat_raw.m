%TEMP_FIG_POWERLAW_PHAT_RAW  Raw p_hat signal + why it swings in the descent.
%   Panel 1  a_z : a_true / a_hat / a_xm raw  (the y2 channel the KF actually eats)
%   Panel 2  p_hat RAW, 6 seeds overlaid, vs the TRUE effective exponent p_eff(hbar)
%   Panel 3  p-update gain K2(5): total vs the P45 path vs the real-information path
%   Panel 4  claimed sigma_p vs realised seed-to-seed scatter
%   Scratch; delete after the diagnosis.

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
az = 3;  seeds = [7 11 23 42 101 777];

SS = cell(numel(seeds),1);
for q = 1:numel(seeds)
    SS{q} = temp_run_powerlaw_diag(cfg, struct('seed', seeds(q), 'verbose', false));
end
s0 = SS{1};  t = s0.tout(:);  P = s0.meta.params_value;
w = P.wall.w_hat(:);  pz = P.wall.pz;  R = P.common.R;
hb_d = (s0.p_d_out * w - pz) / R;

% true effective exponent along the trajectory: p_eff = -c'(hb)*(hb-1)/(c-1)
p_eff = nan(numel(t),1);
for i = 1:numel(t)
    hbv = max(hb_d(i), 1.001);
    [~, cpe, dv] = calc_correction_functions(hbv, true);
    p_eff(i) = -dv.dc_perp_dh * (hbv - 1) / max(cpe - 1, eps);
end

Pmat = zeros(numel(t), numel(seeds));
for q = 1:numel(seeds); Pmat(:,q) = SS{q}.p_hat_out(:,az); end

FS = 18;  nm = 1e3;
tb = [cfg.t_hold, cfg.t_hold + cfg.t_descend_override];
fig = figure('Color','w','Position',[60 40 960 1240]);

% (1) a_z
ax1 = subplot(4,1,1); hold(ax1,'on');
hx = plot(ax1, t, s0.a_xm_out(:,az)*nm, 'Color',[0.6 0.6 0.6],'LineWidth',0.5);
ht = plot(ax1, t, s0.a_true_out(:,az)*nm, 'r','LineWidth',2.6);
he = plot(ax1, t, s0.a_hat_out(:,az)*nm, 'b','LineWidth',2.0);
ylabel(ax1,'a_z  [nm/pN]','FontSize',FS,'FontWeight','bold');
legend([ht he hx],{'a_{true}','a_{hat}','a_{xm} raw (y_2)'},'Location','northoutside', ...
       'Orientation','horizontal','FontSize',FS-5);
ylim(ax1,[0 40]); sty(ax1,FS,t,tb);

% (2) p_hat raw, all seeds
ax2 = subplot(4,1,2); hold(ax2,'on');
for q = 1:numel(seeds)
    plot(ax2, t, Pmat(:,q), 'Color',[0.55 0.65 0.95], 'LineWidth',1.0, 'HandleVisibility','off');
end
hs = plot(ax2, t, Pmat(:,1), 'b','LineWidth',2.0);
hpe = plot(ax2, t, p_eff, 'r','LineWidth',2.6);
ylabel(ax2,'p_z  (raw)','FontSize',FS,'FontWeight','bold');
legend([hpe hs],{'p_{eff} true','p_{hat} (seed 7; 6 seeds behind)'}, ...
       'Location','northoutside','Orientation','horizontal','FontSize',FS-5);
ylim(ax2,[-0.1 1.7]); sty(ax2,FS,t,tb);

% (3) K2(5) decomposition
ax3 = subplot(4,1,3); hold(ax3,'on');
K1 = abs(s0.DG.dg_K25_via_P45(:,az));
K2v = abs(s0.DG.dg_K25_via_P55H(:,az));
hk1 = plot(ax3, t, max(K1,1e-12), 'b','LineWidth',2.0);
hk2 = plot(ax3, t, max(K2v,1e-12), 'r','LineWidth',2.0);
set(ax3,'YScale','log'); ylim(ax3,[1e-8 1e1]);
ylabel(ax3,'|K_2(5)| terms','FontSize',FS,'FontWeight','bold');
legend([hk1 hk2],{'via P_{45} (a_h-error leak)','via P_{55}H_{25} (real p info)'}, ...
       'Location','northoutside','Orientation','horizontal','FontSize',FS-5);
sty(ax3,FS,t,tb);

% (4) claimed vs realised uncertainty
ax4 = subplot(4,1,4); hold(ax4,'on');
hc = plot(ax4, t, s0.P_p_out(:,az), 'b','LineWidth',2.0);
hr = plot(ax4, t, std(Pmat,0,2), 'r','LineWidth',2.6);
ylabel(ax4,'\sigma_p','FontSize',FS,'FontWeight','bold');
xlabel(ax4,'time  [s]','FontSize',FS,'FontWeight','bold');
legend([hr hc],{'realised (6-seed scatter)','claimed  \surd P_{55}'}, ...
       'Location','northoutside','Orientation','horizontal','FontSize',FS-5);
ylim(ax4,[0 0.62]); sty(ax4,FS,t,tb);

outp = fullfile(here, 'temp_powerlaw_phat_raw.png');
exportgraphics(fig, outp, 'Resolution', 140);
close(fig);
fprintf('FIGURE saved: %s\n', outp);

function sty(ax, FS, t, tb)
    set(ax,'FontSize',FS-4,'FontWeight','bold','LineWidth',1.3,'Box','on');
    grid(ax,'off'); xlim(ax,[t(1) t(end)]);
    yl = ylim(ax);
    for x = tb
        plot(ax,[x x],yl,'--','Color',[0.55 0.55 0.55],'LineWidth',1.0,'HandleVisibility','off');
    end
    ylim(ax,yl);
end
