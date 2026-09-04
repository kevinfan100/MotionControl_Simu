% FORK OF test_script/scratch/plot_btrue_nw_mcorr.m (2026-09-04) | PURPOSE: the three b_true arms side by side --
%   base (exact + pred_mean2), + nw_mcorr, + pred_mean2_e4 -- seed mean of (a_hat - a)/a_nom, 0.5 s window,
%   +-2 SEM band, one axes per trajectory (top row), and the PAIRED difference e4 - nwmcorr (10-seed mean, band)
%   against the input the flag adds (running sum of the code's own pred_mean2 difference, exact) and the probe's
%   estimate of it (probe_btrue_e4_line.m; F_dw approximated) (bottom row).
%   Dashed vertical = hold start, dotted = end of the standard run. | EXPIRES: with the b_true rung
function plot_btrue_e4()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();  TR = {'canon','meng'};  FS = 14;  SM = round(0.5 / pc.Ts);
    COL = {[0.47 0.67 0.19], [0.49 0.18 0.56], [0.85 0.33 0.10]};
    LAB = {'b_{true}: exact + pred\_mean2', '+ nw\_mcorr', '+ pred\_mean2\_e4'};
    f = figure('Units','inches','Position',[0 0 14 9], 'Color','w', 'Visible','off'); tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    for it = 1:2
        D0 = load(fullfile(od, sprintf('btrue_nw_mcorr_%s.mat', TR{it})));
        D1 = load(fullfile(od, sprintf('btrue_e4_%s.mat', TR{it})));
        Pb = load(fullfile(od, sprintf('probe_btrue_e4_line_%s.mat', TR{it})));
        A = {D0.base, D0.nwmcorr, D1.e4};  t = D0.base.t;  nS = size(D0.base.E, 2);
        nexttile(it); hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off'); H = gobjects(1,3); L = cell(1,3);
        for a = 1:3
            Es = movmean(A{a}.E, SM, 1);  m = mean(Es, 2);  s = std(Es, 0, 2) / sqrt(nS);
            fill([t; flipud(t)], [m + 2*s; flipud(m - 2*s)], COL{a}, 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            H(a) = plot(t, m, '-', 'Color', COL{a}, 'LineWidth', 2.0);  L{a} = sprintf('%s (%s)', LAB{a}, TR{it});
        end
        xline(D0.phases(3), '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, 'HandleVisibility', 'off');
        xline(D0.phases(4) - 4, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, 'HandleVisibility', 'off');
        legend(H, L, 'Location','northoutside','Orientation','horizontal','FontSize',9,'FontWeight','bold','Box','on');
        if it == 1; ylabel('(\^a_z - a_z)/a_{nom}:  10-seed mean, 0.5 s window, \pm 2 SEM', 'FontSize', 12, 'FontWeight','bold'); end
        xlim([0 ceil(t(end))]); ylim([-8e-3 6e-3]);
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
        % bottom: paired difference vs prediction
        nexttile(it + 2); hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off');
        dE = D1.e4.E - D1.nwmcorr.E;  md = mean(dE, 2);  sd = std(dE, 0, 2) / sqrt(nS);
        fill([t; flipud(t)], [md + 2*sd; flipud(md - 2*sd)], COL{3}, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        h1 = plot(t, md, '-', 'Color', COL{3}, 'LineWidth', 2.0);
        inp  = mean(cumsum(D1.e4.pm2 - D1.nwmcorr.pm2, 1), 2);      % the input the flag actually adds (code's own pred_mean2 log)
        pred = -mean(cumsum(Pb.TERM(:,:,6), 1), 2);                 % probe estimate of the same (F_dw approximated by the raw force history)
        h2 = plot(t, inp, '--', 'Color', [0 0 0], 'LineWidth', 1.8);
        h3 = plot(Pb.t, pred, ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.6);
        xline(D0.phases(3), '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, 'HandleVisibility', 'off');
        legend([h1 h2 h3], {'paired response  (+e_4 line) - (+nw\_mcorr)', 'input added  \Sigma [(\partial\^a''/\partial\^a)Cov(e_4,u) + \^a''''Cov(e_3,u)]', 'probe estimate of the input'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',8,'FontWeight','bold','Box','on');
        if it == 1; ylabel('paired difference of (\^a_z - a_z)/a_{nom}', 'FontSize', 12, 'FontWeight','bold'); end
        xlabel('time  [s]   (dashed: hold start, dotted: end of the standard run)', 'FontSize', 11, 'FontWeight','bold');
        xlim([0 ceil(t(end))]);
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
    end
    png = fullfile(od, 'btrue_e4_band.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('wrote %s\n', png);
end
