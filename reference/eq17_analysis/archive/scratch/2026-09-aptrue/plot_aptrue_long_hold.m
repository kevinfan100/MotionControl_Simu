% FORK OF test_script/scratch/plot_aptrue_kr1_full_mean_band.m (2026-09-03) | PURPOSE: seed mean of
%   (a_hat - a)/a_nom with +-2 SEM band (0.5 s window) for the long-hold runs of run_aptrue_long_hold.m,
%   one axes per trajectory; the short-run end is marked. | EXPIRES: with the parent
function plot_aptrue_long_hold()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();  TR = {'canon','meng'};  C2 = [0.49 0.18 0.56];  FS = 15;  SM = round(0.5 / pc.Ts);
    f = figure('Units','inches','Position',[0 0 14 5.5], 'Color','w', 'Visible','off'); tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
    for it = 1:2
        nexttile; hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off');
        SF = {'', '_mean2'};  COL = {C2, [0.47 0.67 0.19]};  LAB = {'pred\_mean2 + kr1\_full', 'pred\_mean2 only (no feedthrough term)'};  H = gobjects(1,0);  L = {};
        for a = 1:2
            fn = fullfile(od, sprintf('aptrue_long_hold_%s%s.mat', TR{it}, SF{a}));  if ~exist(fn, 'file'); continue; end
            D = load(fn);  t = D.t;  nS = size(D.E, 2);
            Es = movmean(D.E, SM, 1);  m = mean(Es, 2);  s = std(Es, 0, 2) / sqrt(nS);
            fill([t; flipud(t)], [m + 2*s; flipud(m - 2*s)], COL{a}, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            H(end+1) = plot(t, m, '-', 'Color', COL{a}, 'LineWidth', 2.4);  L{end+1} = sprintf('%s  (%s)', LAB{a}, TR{it}); %#ok<AGROW>
        end
        xline(D.phases(3), '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, 'HandleVisibility', 'off');
        xline(D.phases(4) - 4, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, 'HandleVisibility', 'off');
        legend(H, L, 'Location','northoutside','Orientation','horizontal','FontSize',10,'FontWeight','bold','Box','on');
        if it == 1; ylabel('(\^a_z - a_z)/a_{nom}:  10-seed mean, 0.5 s window,  \pm 2 SEM', 'FontSize', 13, 'FontWeight','bold'); end
        xlabel('time  [s]   (dashed: hold start, dotted: end of the short run)', 'FontSize', 13, 'FontWeight','bold');
        xlim([0 ceil(t(end))]); ylim([-3e-3 4e-3]);
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
    end
    png = fullfile(od, 'aptrue_long_hold_mean_band.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('wrote %s\n', png);
end
