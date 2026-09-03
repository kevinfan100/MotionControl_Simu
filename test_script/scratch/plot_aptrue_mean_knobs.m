% FORK OF test_script/scratch/plot_aptrue_long_hold.m (2026-09-03) | PURPOSE: the three mean-only-knob
%   arms of run_aptrue_mean_knobs.m -- seed mean of (a_hat - a)/a_nom, 0.5 s window, +-2 SEM band,
%   one axes per trajectory; dashed = hold start (knobs on), dotted = end of the short run.
%   | EXPIRES: with the parent
function plot_aptrue_mean_knobs()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();  TR = {'canon','meng'};  FS = 15;  SM = round(0.5 / pc.Ts);
    ARM = {'base','pslope','y1leg'};  COL = {[0.47 0.67 0.19], [0.49 0.18 0.56], [0.85 0.45 0.10]};
    LAB = {'base (pred\_mean2)', 'a'' M scaled 1.10 in hold', 'y_1 gain leg scaled 1.05 in hold'};
    f = figure('Units','inches','Position',[0 0 14 5.5], 'Color','w', 'Visible','off'); tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
    for it = 1:2
        D0 = load(fullfile(od, sprintf('aptrue_mean_knobs_%s.mat', TR{it})));
        nexttile; hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off'); H = gobjects(1,3); L = cell(1,3);
        for a = 1:3
            D = D0.(ARM{a});  t = D.t;  nS = size(D.E, 2);
            Es = movmean(D.E, SM, 1);  m = mean(Es, 2);  s = std(Es, 0, 2) / sqrt(nS);
            fill([t; flipud(t)], [m + 2*s; flipud(m - 2*s)], COL{a}, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            H(a) = plot(t, m, '-', 'Color', COL{a}, 'LineWidth', 2.2);  L{a} = sprintf('%s  (%s)', LAB{a}, TR{it});
        end
        xline(D0.phases(3), '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, 'HandleVisibility', 'off');
        xline(D0.phases(4) - 4, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, 'HandleVisibility', 'off');
        legend(H, L, 'Location','northoutside','Orientation','horizontal','FontSize',9,'FontWeight','bold','Box','on');
        if it == 1; ylabel('(\^a_z - a_z)/a_{nom}:  10-seed mean, 0.5 s window,  \pm 2 SEM', 'FontSize', 13, 'FontWeight','bold'); end
        xlabel('time  [s]   (dashed: hold start = knobs on, dotted: end of the short run)', 'FontSize', 12, 'FontWeight','bold');
        xlim([0 ceil(t(end))]); ylim([-8e-3 8e-3]);
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
    end
    png = fullfile(od, 'aptrue_mean_knobs_band.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('wrote %s\n', png);
end
