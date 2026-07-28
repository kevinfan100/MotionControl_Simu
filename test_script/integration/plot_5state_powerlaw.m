% plot_5state_powerlaw.m — z-axis result, lab convention, BASIC colours,
% no (a)(b)(c). Panels:
%   1. a_z     : a_true (r) / a_hat (b) / a_xm (grey, raw noisy measurement)
%   2. a_z err : (a_hat - a_true)/a_true  [%]
%   3. a'_z    : a'_true (r) vs a'_hat (b)
%   4. p_z     : RAW p_hat (per-step KF output, no smoothing) +/- 1 sigma, zoomed
here = fileparts(mfilename('fullpath'));
S = load(fullfile(here, 'temp_smoke_5state_powerlaw.mat'));
sim = S.sim;  cfg = S.cfg;  P = sim.meta.params_value;

t   = sim.tout(:);  nm = 1e3;
aTz = sim.a_true_out(:,3);   aHz = sim.a_hat_out(:,3);   aXz = sim.a_xm_out(:,3);   % um/pN
apH = sim.a_prime_out(:,3);  pH = sim.p_hat_out(:,3);   Pp = sim.P_p_out(:,3);
errz = 100 * (aHz - aTz) ./ aTz;

% --- true slope a'_true = da/dh = -(K_h_perp/R)*a_true, from true h_bar ---
w_hat = P.wall.w_hat(:);  pz = P.wall.pz;  R = P.common.R;
hbar_true = max((sim.p_true_out * w_hat - pz) / R, 1.001);
Khp = zeros(numel(t),1);
for i = 1:numel(t)
    [~, ~, dv] = calc_correction_functions(hbar_true(i), true);
    Khp(i) = dv.K_h_perp;
end
apT = -(Khp / R) .* aTz;       % a'_true [1/pN]

% --- true effective exponent the real wall curve demands, along the DESIRED
%     trajectory: p_eff(hbar) = -c'(hbar)*(hbar-1)/(c(hbar)-1). This, not the
%     p = 1 modelling assumption, is what p_hat should be compared against. ---
hbar_d = max((sim.p_d_out * w_hat - pz) / R, 1.001);
pEff = zeros(numel(t),1);
for i = 1:numel(t)
    [~, cpe_i, dv_i] = calc_correction_functions(hbar_d(i), true);
    pEff(i) = -dv_i.dc_perp_dh * (hbar_d(i) - 1) / max(cpe_i - 1, eps);
end

FS = 18;
r = 'r';  b = 'b';  grey = [0.6 0.6 0.6];  band = [0.80 0.86 1.0];
tb = [cfg.t_hold, cfg.t_hold + cfg.t_descend_override];

fig = figure('Color','w','Position',[80 40 900 1240]);

% (1) a_z : true / hat / a_xm
ax1 = subplot(4,1,1); hold(ax1,'on');
hx = plot(ax1, t, aXz*nm, 'Color',grey, 'LineWidth',0.5);
ht = plot(ax1, t, aTz*nm, r, 'LineWidth',2.6);
he = plot(ax1, t, aHz*nm, b, 'LineWidth',2.0);
ylabel(ax1, 'a_z  [nm/pN]', 'FontSize',FS,'FontWeight','bold');
legend([ht he hx], {'a_{true}','a_{hat}','a_{xm} (raw)'}, 'Location','northoutside', ...
       'Orientation','horizontal','FontSize',FS-4);
ylim(ax1,[0 24]); style_ax(ax1,FS); xlim(ax1,[t(1) t(end)]); add_phase(ax1,tb);

% (2) a_z relative error
ax2 = subplot(4,1,2); hold(ax2,'on');
he2 = plot(ax2, t, errz, b, 'LineWidth',2.0);
h0  = plot(ax2, t, zeros(size(t)), r, 'LineWidth',2.0);
plot(ax2, t,  2*ones(size(t)), 'k--','LineWidth',1.2,'HandleVisibility','off');
plot(ax2, t, -2*ones(size(t)), 'k--','LineWidth',1.2,'HandleVisibility','off');
ylabel(ax2, 'a_z error  [%]', 'FontSize',FS,'FontWeight','bold');
legend([h0 he2], {'zero','(a_{hat}-a_{true})/a_{true}'}, 'Location','northoutside', ...
       'Orientation','horizontal','FontSize',FS-4);
ylim(ax2,[-8 8]); style_ax(ax2,FS); xlim(ax2,[t(1) t(end)]); add_phase(ax2,tb);

% (3) a'_z : true vs hat
ax3 = subplot(4,1,3); hold(ax3,'on');
ht3 = plot(ax3, t, apT*nm, r, 'LineWidth',2.6);
he3 = plot(ax3, t, apH*nm, b, 'LineWidth',2.0);
ylabel(ax3, 'a''_z  [nm/pN/\mum]', 'FontSize',FS,'FontWeight','bold');
legend([ht3 he3], {'a''_{true}','a''_{hat}'}, 'Location','northoutside', ...
       'Orientation','horizontal','FontSize',FS-4);
style_ax(ax3,FS); xlim(ax3,[t(1) t(end)]); add_phase(ax3,tb);

% (4) p_z : RAW per-step estimate, zoomed so the actual signal is visible
ax4 = subplot(4,1,4); hold(ax4,'on');
tt = [t; flipud(t)];  bb = [pH+Pp; flipud(pH-Pp)];
fill(ax4, tt, bb, band, 'EdgeColor','none', 'HandleVisibility','off');
hpt = plot(ax4, t, pEff, r, 'LineWidth',2.6);
hp = plot(ax4, t, pH, b, 'LineWidth',1.6);
h1 = plot(ax4, t, ones(size(t)), 'k--', 'LineWidth',1.2);
ylabel(ax4, 'p_z  (raw)', 'FontSize',FS,'FontWeight','bold');
xlabel(ax4, 'time  [s]', 'FontSize',FS,'FontWeight','bold');
legend([hpt hp h1], {'p_{eff} true','p_{hat} raw \pm 1\sigma','p = 1 (assumed)'}, ...
       'Location','northoutside','Orientation','horizontal','FontSize',FS-5);
ylim(ax4,[0.94 1.06]); style_ax(ax4,FS); xlim(ax4,[t(1) t(end)]); add_phase(ax4,tb);

fig_dir = fullfile(here, '..', '..', 'reference','eq17_analysis','derivation','figures');
if ~exist(fig_dir,'dir'); mkdir(fig_dir); end
outp = fullfile(fig_dir, 'powerlaw_5state_sim.png');
exportgraphics(fig, outp, 'Resolution', 150);
close(fig);
fprintf('FIGURE saved: %s\n', outp);
fprintf('a_z error : osc-window mean %+.2f %%, max |err| %.2f %%\n', ...
        mean(errz(t>1.6)), max(abs(errz(t>1.6))));
fprintf('p_z raw   : mean %.4f, std %.4f, range [%.4f %.4f]\n', ...
        mean(pH(t>1.6)), std(pH(t>1.6)), min(pH), max(pH));
fprintf('p_eff true: range [%.4f %.4f], osc-window mean %.4f\n', ...
        min(pEff), max(pEff), mean(pEff(t>1.6)));
fprintf('p_hat - p_eff : osc-window mean %+.4f, max |err| %.4f\n', ...
        mean(pH(t>1.6)-pEff(t>1.6)), max(abs(pH-pEff)));


function style_ax(ax, FS)
    set(ax, 'FontSize',FS-2, 'FontWeight','bold', 'LineWidth',1.3, 'Box','on');
    grid(ax, 'off');
end

function add_phase(ax, tb)
    yl = ylim(ax);
    for x = tb
        plot(ax, [x x], yl, '--', 'Color',[0.55 0.55 0.55], 'LineWidth',1.0, 'HandleVisibility','off');
    end
    ylim(ax, yl);
end
