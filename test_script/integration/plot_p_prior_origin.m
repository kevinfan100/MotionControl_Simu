%PLOT_P_PRIOR_ORIGIN  Where the p prior 0.025 comes from.
%   The power-law gain model says   s_h = (p/(hbar-1)) * a_h(a_o-a_h)/a_o
%   The truth is                    s_h = da_h/dhbar = -a_o*c'(hbar)/c(hbar)^2
%   Equating them defines the effective exponent the real wall curve demands:
%       p_eff(hbar) = -c'(hbar)*(hbar-1)/(c(hbar)-1)
%   Panel 1: p_eff over the trajectory range, vs the p=1 assumption.
%   Panel 2: the slope error that the p=1 assumption therefore commits.
%   The prior width is read off panel 2 -- it is a property of the wall curve,
%   computed offline, not a number tuned against any simulation output.
%   Scratch; delete with the rest of this round's temp files.

clear; clc;
here = fileparts(mfilename('fullpath'));
proj = fullfile(here, '..', '..');
addpath(genpath(proj));

hb = linspace(1.5, 25, 3000).';
pe_perp = zeros(size(hb));  pe_para = zeros(size(hb));
er_perp = zeros(size(hb));  er_para = zeros(size(hb));
for i = 1:numel(hb)
    [cpa, cpe, dv] = calc_correction_functions(hb(i), true);
    pe_perp(i) = -dv.dc_perp_dh * (hb(i)-1) / (cpe - 1);
    pe_para(i) = -dv.dc_para_dh * (hb(i)-1) / (cpa - 1);
    % exponent error of the p=1 assumption -- THIS is what the prior on p covers
    er_perp(i) = pe_perp(i) - 1;
    er_para(i) = pe_para(i) - 1;
    % (the slope error it induces is 1/p_eff - 1, since s_h(p=1)/s_h_true = 1/p_eff)
end
hb_lo = 2.0;  hb_hi = 22.22;                 % the trajectory's actual range
in = hb >= hb_lo & hb <= hb_hi;
sig_p = max(abs(er_perp(in)));

FS = 18;
fig = figure('Color','w','Position',[80 80 900 760]);

ax1 = subplot(2,1,1); hold(ax1,'on');
yl = [0.4 1.15];
fill(ax1, [hb_lo hb_hi hb_hi hb_lo], [yl(1) yl(1) yl(2) yl(2)], [0.92 0.92 0.92], ...
     'EdgeColor','none','HandleVisibility','off');
h1 = plot(ax1, hb, pe_perp, 'r', 'LineWidth',2.6);
h2 = plot(ax1, hb, pe_para, 'b', 'LineWidth',2.0);
h3 = plot(ax1, hb, ones(size(hb)), 'k--', 'LineWidth',1.4);
ylabel(ax1,'p_{eff}(h/R)','FontSize',FS,'FontWeight','bold');
legend([h1 h2 h3], {'z  (wall-normal, \perp)','x / y  (parallel)','p = 1  assumed'}, ...
       'Location','northoutside','Orientation','horizontal','FontSize',FS-5);
ylim(ax1, yl); sty(ax1, FS, hb);
text(ax1, 11, 0.52, 'trajectory range', 'FontSize',FS-6,'FontWeight','bold', ...
     'HorizontalAlignment','center','Color',[0.35 0.35 0.35]);

ax2 = subplot(2,1,2); hold(ax2,'on');
yl2 = [-45 10];
fill(ax2, [hb_lo hb_hi hb_hi hb_lo], [yl2(1) yl2(1) yl2(2) yl2(2)], [0.92 0.92 0.92], ...
     'EdgeColor','none','HandleVisibility','off');
g1 = plot(ax2, hb, 100*er_perp, 'r', 'LineWidth',2.6);
g2 = plot(ax2, hb, 100*er_para, 'b', 'LineWidth',2.0);
g3 = plot(ax2, hb, 100*sig_p*ones(size(hb)), 'k--', 'LineWidth',1.4);
plot(ax2, hb, -100*sig_p*ones(size(hb)), 'k--', 'LineWidth',1.4,'HandleVisibility','off');
ylabel(ax2,'p_{eff} - 1   [%]','FontSize',FS,'FontWeight','bold');
xlabel(ax2,'h / R','FontSize',FS,'FontWeight','bold');
legend([g1 g2 g3], {'z  (\perp)','x / y  (parallel)', ...
       sprintf('\\pm %.1f %% = prior \\sigma_p', 100*sig_p)}, ...
       'Location','southoutside','Orientation','horizontal','FontSize',FS-5);
ylim(ax2, yl2); sty(ax2, FS, hb);

outp = fullfile(here, 'temp_p_prior_origin.png');
exportgraphics(fig, outp, 'Resolution', 140);
close(fig);
fprintf('FIGURE saved: %s\n', outp);
fprintf('over h/R in [%.1f, %.2f]:\n', hb_lo, hb_hi);
fprintf('  perp : p_eff %.4f .. %.4f | exponent err max %.2f %% rms %.2f %% -> sigma_p = %.3f\n', ...
        min(pe_perp(in)), max(pe_perp(in)), 100*max(abs(er_perp(in))), ...
        100*rms(er_perp(in)), sig_p);
fprintf('         induced SLOPE err (1/p_eff-1) max %.2f %%\n', ...
        100*max(abs(1./pe_perp(in) - 1)));
fprintf('  para : p_eff %.4f .. %.4f | exponent err max %.2f %% rms %.2f %%\n', ...
        min(pe_para(in)), max(pe_para(in)), 100*max(abs(er_para(in))), ...
        100*rms(er_para(in)));
fprintf('         induced SLOPE err (1/p_eff-1) max %.2f %%\n', ...
        100*max(abs(1./pe_para(in) - 1)));
fprintf('old prior 0.300 claimed %.0fx more ignorance than the curve actually has\n', 0.3/sig_p);

function sty(ax, FS, hb)
    set(ax,'FontSize',FS-4,'FontWeight','bold','LineWidth',1.3,'Box','on');
    grid(ax,'off'); xlim(ax,[hb(1) hb(end)]);
end
