% FORK OF test_script/scratch/plot_aptrue_nw_mcorr.m (2026-09-04) | PURPOSE: the two arms of
%   run_btrue_nw_mcorr.m (b_true@true + exact + pred_mean2, +/- nw_mcorr) -- seed mean of (a_hat - a)/a_nom,
%   0.5 s window, +-2 SEM band, one axes per trajectory; dashed vertical = hold start, dotted = end of the
%   standard run. Black dashed curve = the e4-line prediction of probe_btrue_e4_line.m: the running sum of
%   2 b (1 - a_hat) Cov(e4 + a' e3, u) (est - true), i.e. the mean term the gain-reading slope carries that
%   pred_mean2's height-reading start-point term does not (base u; the mcorr u differs by < 10%).
%   | EXPIRES: with the b_true rung
function plot_btrue_nw_mcorr()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();  TR = {'canon','meng'};  FS = 15;  SM = round(0.5 / pc.Ts);
    ARM = {'base','nwmcorr'};  COL = {[0.47 0.67 0.19], [0.49 0.18 0.56]};
    LAB = {'b_{true}: exact + pred\_mean2', '+ nw\_mcorr'};
    f = figure('Units','inches','Position',[0 0 14 5.5], 'Color','w', 'Visible','off'); tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
    for it = 1:2
        D0 = load(fullfile(od, sprintf('btrue_nw_mcorr_%s.mat', TR{it})));
        Pb = load(fullfile(od, sprintf('probe_btrue_e4_line_%s.mat', TR{it})));
        nexttile; hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off'); H = gobjects(1,3); L = cell(1,3);
        for a = 1:2
            D = D0.(ARM{a});  t = D.t;  nS = size(D.E, 2);
            Es = movmean(D.E, SM, 1);  m = mean(Es, 2);  s = std(Es, 0, 2) / sqrt(nS);
            fill([t; flipud(t)], [m + 2*s; flipud(m - 2*s)], COL{a}, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            H(a) = plot(t, m, '-', 'Color', COL{a}, 'LineWidth', 2.2);  L{a} = sprintf('%s  (%s)', LAB{a}, TR{it});
        end
        cs = mean(cumsum(Pb.TERM(:,:,3), 1), 2);   % RESID base, est - true
        H(3) = plot(Pb.t, cs, '--', 'Color', [0 0 0], 'LineWidth', 1.8);  L{3} = 'e_4-line prediction  \Sigma 2b(1-\^a)Cov(e_4+\^a''e_3, u)';
        xline(D0.phases(3), '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, 'HandleVisibility', 'off');
        xline(D0.phases(4) - 4, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, 'HandleVisibility', 'off');
        legend(H, L, 'Location','northoutside','Orientation','horizontal','FontSize',9,'FontWeight','bold','Box','on');
        if it == 1; ylabel('(\^a_z - a_z)/a_{nom}:  10-seed mean, 0.5 s window,  \pm 2 SEM', 'FontSize', 13, 'FontWeight','bold'); end
        xlabel('time  [s]   (dashed: hold start, dotted: end of the standard run)', 'FontSize', 12, 'FontWeight','bold');
        xlim([0 ceil(t(end))]); ylim([-8e-3 6e-3]);
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
        % console: measured vs predicted at the end of the descent and at hold start
        t2 = D0.phases(2); t3 = D0.phases(3);
        for a = 1:2
            E = D0.(ARM{a}).E; m2 = mean(E(t > t2 - 0.25 & t <= t2, :), 'all'); s2 = std(mean(E(t > t2 - 0.25 & t <= t2, :), 1)) / sqrt(size(E,2));
            fprintf('[%s %-7s] est-true at end of descent (last 0.25 s) %+.5f (SEM %.5f) | at hold start (first 0.25 s) %+.5f\n', ...
                TR{it}, ARM{a}, m2, s2, mean(E(t > t3 & t <= t3 + 0.25, :), 'all'));
        end
        fprintf('[%s e4-line] predicted cum at end of descent %+.5f | at hold start %+.5f | hold last-3-s slope %+.3f e-6/step\n', ...
            TR{it}, cs(find(Pb.t > t2, 1)), cs(find(Pb.t > t3, 1)), 1e6 * mean(mean(Pb.TERM(Pb.t > Pb.phases(4) - 3, :, 3), 1)));
    end
    png = fullfile(od, 'btrue_nw_mcorr_band.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('wrote %s\n', png);
end
