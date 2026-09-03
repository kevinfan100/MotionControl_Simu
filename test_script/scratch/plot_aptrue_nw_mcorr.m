% FORK OF test_script/scratch/plot_aptrue_mean_knobs.m (2026-09-03) | PURPOSE: the two arms of
%   run_aptrue_nw_mcorr.m -- seed mean of (a_hat - a)/a_nom, 0.5 s window, +-2 SEM band,
%   one axes per trajectory; dashed = hold start (knobs on), dotted = end of the short run.
%   | EXPIRES: with the parent
function plot_aptrue_nw_mcorr()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();  TR = {'canon','meng'};  FS = 15;  SM = round(0.5 / pc.Ts);
    ARM = {'base','nwmcorr'};  COL = {[0.47 0.67 0.19], [0.49 0.18 0.56]};
    LAB = {'base (exact + @est + pred\_mean2)', '+ nw\_mcorr (correlated-noise predict)'};
    f = figure('Units','inches','Position',[0 0 14 5.5], 'Color','w', 'Visible','off'); tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
    for it = 1:2
        D0 = load(fullfile(od, sprintf('aptrue_nw_mcorr_%s.mat', TR{it})));
        nexttile; hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off'); H = gobjects(1,2); L = cell(1,2);
        for a = 1:2
            D = D0.(ARM{a});  t = D.t;  nS = size(D.E, 2);
            Es = movmean(D.E, SM, 1);  m = mean(Es, 2);  s = std(Es, 0, 2) / sqrt(nS);
            fill([t; flipud(t)], [m + 2*s; flipud(m - 2*s)], COL{a}, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            H(a) = plot(t, m, '-', 'Color', COL{a}, 'LineWidth', 2.2);  L{a} = sprintf('%s  (%s)', LAB{a}, TR{it});
        end
        xline(D0.phases(3), '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, 'HandleVisibility', 'off');
        xline(D0.phases(4) - 4, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, 'HandleVisibility', 'off');
        legend(H, L, 'Location','northoutside','Orientation','horizontal','FontSize',9,'FontWeight','bold','Box','on');
        if it == 1; ylabel('(\^a_z - a_z)/a_{nom}:  10-seed mean, 0.5 s window,  \pm 2 SEM', 'FontSize', 13, 'FontWeight','bold'); end
        xlabel('time  [s]   (dashed: hold start, dotted: end of the short run)', 'FontSize', 12, 'FontWeight','bold');
        xlim([0 ceil(t(end))]); ylim([-3e-3 4e-3]);
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
    end
    png = fullfile(od, 'aptrue_nw_mcorr_band.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('wrote %s\n', png);
end
