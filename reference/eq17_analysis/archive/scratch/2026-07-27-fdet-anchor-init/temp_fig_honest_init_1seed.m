%TEMP_FIG_HONEST_INIT_1SEED  Single-seed look at the HONEST init
%   (a_h[0] = a_nom, no c(h_bar) peek anywhere), against the current peeking init.
%   Panel 1: a_z over the whole run.  Panel 2: zoom on the 0.5 s far-field hold.
%   Panel 3: a_hat relative error, same zoom.
%   Everything is seed 7, raw, no averaging.
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
cfg.h_bar_safe = 1.5;  cfg.Pf_p_std = 0.035;
az = 3;  seed = 7;

% col 2 = init mode: 0 peek (cheat) / 1 flat a_nom / 2 far-field asymptote
arms = { 'peek a_nom/c  (作弊)      ', 0, 0.03, [0.55 0.55 0.55]; ...
         'flat a_nom    (誠實,粗)   ', 1, 0.06, [0 0.6 0]; ...
         'asym 1+(9/8)/hbar (誠實)  ', 2, 0.01, [0 0 1] };
S = cell(3,1);
for r = 1:3
    c = cfg;  c.Pf_a_frac = arms{r,3};
    c.a_init_honest = (arms{r,2} == 1);
    c.a_init_asym   = (arms{r,2} == 2);
    S{r} = temp_run_powerlaw_diag(c, struct('seed', seed, 'verbose', false));
end
t = S{1}.tout(:);  nm = 1e3;
aT = S{1}.a_true_out(:,az);

FS = 18;  tb = [cfg.t_hold, cfg.t_hold + cfg.t_descend_override];
fig = figure('Color','w','Position',[70 50 940 1060]);

ax1 = subplot(3,1,1); hold(ax1,'on');
ht = plot(ax1, t, aT*nm, 'r','LineWidth',2.6);
hh = gobjects(3,1);
for r = 1:3
    hh(r) = plot(ax1, t, S{r}.a_hat_out(:,az)*nm, 'Color',arms{r,4},'LineWidth',1.8);
end
ylabel(ax1,'a_z  [nm/pN]','FontSize',FS,'FontWeight','bold');
legend([ht; hh], [{'a_{true}'}, arms(:,1).'], 'Location','northoutside', ...
       'Orientation','horizontal','FontSize',FS-8);
ylim(ax1,[4 17]); sty(ax1,FS,[t(1) t(end)],tb);

ax2 = subplot(3,1,2); hold(ax2,'on');
plot(ax2, t, aT*nm, 'r','LineWidth',2.6);
for r = 1:3
    plot(ax2, t, S{r}.a_hat_out(:,az)*nm, 'Color',arms{r,4},'LineWidth',1.8);
end
ylabel(ax2,'a_z  [nm/pN]  (zoom)','FontSize',FS,'FontWeight','bold');
ylim(ax2,[11.5 15.5]); sty(ax2,FS,[0 0.5],[]);

ax3 = subplot(3,1,3); hold(ax3,'on');
for r = 1:3
    rel = 100*(S{r}.a_hat_out(:,az) - aT)./aT;
    plot(ax3, t, rel, 'Color',arms{r,4},'LineWidth',1.8);
end
plot(ax3, [0 0.5], [0 0], 'r','LineWidth',2.0);
plot(ax3, [0 0.5], [2 2], 'k--','LineWidth',1.2);
plot(ax3, [0 0.5], [-2 -2], 'k--','LineWidth',1.2);
ylabel(ax3,'a_{hat} error  [%]','FontSize',FS,'FontWeight','bold');
xlabel(ax3,'time  [s]','FontSize',FS,'FontWeight','bold');
ylim(ax3,[-10 8]); sty(ax3,FS,[0 0.5],[]);

outp = fullfile(here, 'temp_honest_init_1seed.png');
exportgraphics(fig, outp, 'Resolution', 140);
close(fig);
fprintf('FIGURE saved: %s\n\n', outp);

fprintf('seed %d, z axis, a_h[0] assumption error = %+.2f %%\n', seed, ...
        100*(S{2}.a_hat_out(1,az)/aT(1) - 1));
fprintf('%-26s | %s\n', 'arm', '  t=0     0.02    0.05    0.10    0.30    0.50   | osc mean | trk nm');
for r = 1:3
    rel = 100*(S{r}.a_hat_out(:,az) - aT)./aT;
    str = '';
    for tq = [0 0.02 0.05 0.10 0.30 0.50]
        [~,i1] = min(abs(t-tq));  str = [str sprintf('%+7.2f ', rel(i1))]; %#ok<AGROW>
    end
    mo = t > 1.6;
    e = S{r}.p_d_out(2:end,:) - S{r}.p_true_out(1:end-1,:);  te = t(2:end);
    fprintf('%-26s | %s|  %6.4f  | %6.2f\n', arms{r,1}, str, ...
            mean(S{r}.a_hat_out(mo,az)./aT(mo)), 1e3*std(e(te>1.6,az)));
end

function sty(ax, FS, xl, tb)
    set(ax,'FontSize',FS-4,'FontWeight','bold','LineWidth',1.3,'Box','on');
    grid(ax,'off'); xlim(ax,xl);
    yl = ylim(ax);
    for x = tb
        plot(ax,[x x],yl,'--','Color',[0.55 0.55 0.55],'LineWidth',1.0,'HandleVisibility','off');
    end
    ylim(ax,yl);
end
