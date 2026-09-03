% FORK OF test_script/scratch/run_aptrue_kr1.m (plot part, 2026-09-03) | PURPOSE: the seed MEAN of
%   (a_hat - a)/a_nom with its +-2 SEM(t) band, kr1 (green) vs kr1_full (purple) overlaid, one
%   axes per trajectory -- to read whether the wander of the 10-seed mean in aptrue_kr1_full_*.png
%   row 1 is bias or seed-mean noise. Prints window means +- SEM. | EXPIRES: with the parent
function plot_aptrue_kr1_full_mean_band()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();  TR = {'canon','meng'};  C1 = [0.47 0.67 0.19]; C2 = [0.49 0.18 0.56];  FS = 15;
    SM = round(0.5 / pc.Ts);                                  % 0.5 s moving mean
    f = figure('Units','inches','Position',[0 0 14 5.5], 'Color','w', 'Visible','off'); tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
    for it = 1:2
        D = load(fullfile(od, sprintf('aptrue_kr1_full_%s.mat', TR{it})));  t = D.left.t;  nS = size(D.left.E, 2);
        nexttile; hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off');
        A = {D.left.E, D.right.E};  COL = {C1, C2};  H = gobjects(1,2);
        for a = 1:2
            Es = movmean(A{a}, SM, 1);                              % per-seed 0.5 s moving mean
            m = mean(Es, 2);  s = std(Es, 0, 2) / sqrt(nS);         % seed mean and its SEM, per time
            fill([t; flipud(t)], [m + 2*s; flipud(m - 2*s)], COL{a}, 'FaceAlpha', 0.18, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            H(a) = plot(t, m, '-', 'Color', COL{a}, 'LineWidth', 2.4);
        end
        legend(H, {'kr1  a''''(1-\lambda_c)^2 K_{31} R_1', 'kr1\_full  a''''(1-\lambda_c) K_{31} R_1'}, 'Location','northoutside','Orientation','horizontal','FontSize',11,'FontWeight','bold','Box','on');
        if it == 1; ylabel('(\^a_z - a_z)/a_{nom}:  10-seed mean, 0.5 s window,  \pm 2 SEM', 'FontSize', 13, 'FontWeight','bold'); end
        xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold'); xlim([0 ceil(t(end))]); ylim([-2e-3 2e-3]);
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
        % window means
        switch TR{it}
            case 'canon'; W = {'desc 0.5-1.5', t>0.5&t<=1.5; 'osc 1.5-3.5', t>1.5&t<=3.5; 'hold 3.5-4.2', t>3.5&t<=4.2; 'hold 4.2-4.8', t>4.2};
            case 'meng';  W = {'far 0.5-6', t>0.5&t<=6; 'near 6-8', t>6&t<=8; 'near 8-10.5', t>8&t<=10.5; 'hold 10.5-11.5', t>10.5&t<=11.5; 'hold 11.5-12.5', t>11.5};
        end
        fprintf('[%s] window means of (a_hat - a)/a_nom, 10 seeds: mean (SEM over seeds) | per-time-point SEM ~ %.4f\n', TR{it}, mean(std(A{2},0,2))/sqrt(nS));
        for w = 1:size(W,1)
            m = W{w,2};  p1 = mean(A{1}(m,:),1);  p2 = mean(A{2}(m,:),1);
            fprintf('   %-16s kr1 %+.5f (%.5f)   kr1_full %+.5f (%.5f)   paired diff %+.5f (%.5f)\n', W{w,1}, mean(p1), std(p1)/sqrt(nS), mean(p2), std(p2)/sqrt(nS), mean(p1-p2), std(p1-p2)/sqrt(nS));
        end
    end
    png = fullfile(od, 'aptrue_kr1_full_mean_band.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('wrote %s\n', png);
end
