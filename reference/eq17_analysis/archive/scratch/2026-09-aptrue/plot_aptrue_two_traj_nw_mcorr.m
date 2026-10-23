% FORK OF test_script/scratch/plot_aptrue_two_traj_single_vs_seeds.m (2026-09-03) | PURPOSE: the same
%   four rows (seed 7 absolute / 10-seed mean +- sigma absolute / gain error with band / same-instant
%   tracking error R dw3 [um]), left Meng, right canon, with the two arms of run_aptrue_nw_mcorr_full.m
%   OVERLAID: green = exact + @est + pred_mean2, purple = + nw_mcorr. y shared per row.
%   | EXPIRES: with the parent | 產線改動不會自動跟上
function plot_aptrue_two_traj_nw_mcorr()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    D = {load(fullfile(od, 'aptrue_nw_mcorr_full_meng.mat')), load(fullfile(od, 'aptrue_nw_mcorr_full_canon.mat'))};
    NM = {'Meng', 'canon'}; R_um = 2.25; SEED = 7;
    COL_TRUE = [0.8 0 0]; COLA = {[0.47 0.67 0.19], [0.49 0.18 0.56]}; ARM = {'base','nwmcorr'}; LAB = {'exact + @est + pred\_mean2', '+ nw\_mcorr'};
    FS = 15; LFS = 10; AXLW = 1.8;
    f = figure('Units','inches','Position',[0 0 13 15.5], 'Color','w', 'Visible','off');
    tiledlayout(4, 2, 'TileSpacing','compact', 'Padding','compact'); ax = gobjects(4,2);
    for a = 1:2
        d = D{a}; t = d.base.t(:); T_END = ceil(t(end)); mh = t > d.t_hold; nS = size(d.base.E, 2);
        % row 1: one run, both arms + true
        ax(1,a) = nexttile(a); hold on; H = gobjects(1,3);
        H(3) = plot(t, d.base.AT(:,SEED), '-', 'Color', COL_TRUE, 'LineWidth', 2.0);
        for b = 1:2; H(b) = plot(t, d.(ARM{b}).AH(:,SEED), '-', 'Color', COLA{b}, 'LineWidth', 1.0); end
        legend(H, {sprintf('\\^a_z/a_{nom} seed %d  %s', SEED, LAB{1}), sprintf('seed %d  %s', SEED, LAB{2}), 'a_z/a_{nom} true'}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('a_z / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        % row 2: 10-seed mean +- sigma, both arms + true
        ax(2,a) = nexttile(2+a); hold on; H = gobjects(1,3);
        for b = 1:2
            m = mean(d.(ARM{b}).AH,2); s = std(d.(ARM{b}).AH,0,2);
            fill([t; flipud(t)], [m+s; flipud(m-s)], COLA{b}, 'FaceAlpha', 0.18, 'EdgeColor','none', 'HandleVisibility','off');
            H(b) = plot(t, m, '-', 'Color', COLA{b}, 'LineWidth', 1.5);
        end
        H(3) = plot(t, mean(d.base.AT,2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0);
        legend(H, {sprintf('mean \\^a_z/a_{nom} (%d seeds) \\pm\\sigma  %s', nS, LAB{1}), LAB{2}, 'a_z/a_{nom} true'}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('a_z / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        % row 3: gain error, seed mean +- sigma, both arms
        ax(3,a) = nexttile(4+a); hold on; H = gobjects(1,2);
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility','off');
        for b = 1:2
            E = d.(ARM{b}).E; mE = mean(E,2); sE = std(E,0,2);
            fill([t; flipud(t)], [mE+sE; flipud(mE-sE)], COLA{b}, 'FaceAlpha', 0.18, 'EdgeColor','none', 'HandleVisibility','off');
            H(b) = plot(t, movmean(mE, round(0.2/(t(2)-t(1)))), '-', 'Color', COLA{b}, 'LineWidth', 2.0);   % 0.2 s moving mean of the seed mean (the raw per-step 10-seed mean jitters +-0.002)
        end
        legend(H, {sprintf('seed mean (\\^a_z - a_z)/a_{nom}, 0.2 s, \\pm\\sigma  %s', LAB{1}), LAB{2}}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('(\^a_z - a_z) / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        % row 4: same-instant tracking error R dw3 [um], both arms
        ax(4,a) = nexttile(6+a); hold on; H = gobjects(1,2);
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility','off');
        for b = 1:2
            TR = (d.(ARM{b}).hd(:) - d.(ARM{b}).HB) * R_um; ok = all(isfinite(TR),2); mT = mean(TR,2); sT = std(TR,0,2);
            fill([t(ok); flipud(t(ok))], [mT(ok)+sT(ok); flipud(mT(ok)-sT(ok))], COLA{b}, 'FaceAlpha', 0.18, 'EdgeColor','none', 'HandleVisibility','off');
            H(b) = plot(t, mT, '-', 'Color', COLA{b}, 'LineWidth', 2.0);
            if b == 1; TR1 = TR; else; TR2 = TR; end
        end
        legend(H, {sprintf('seed mean R\\deltaw_3 \\pm\\sigma  %s', LAB{1}), LAB{2}}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('R\deltaw_3  [\mum]', 'FontSize', FS, 'FontWeight','bold'); end
        xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        for b = 1:2
            E = d.(ARM{b}).E; TR = (d.(ARM{b}).hd(:) - d.(ARM{b}).HB) * R_um;
            fprintf('[%s %-7s] hold: gain bias %+.5f (SEM %.5f) | sigma_seed %.5f | seed %d hold rms %.5f | motion err hold mean %+.4f um, sigma_seed %.4f um\n', ...
                NM{a}, ARM{b}, mean(E(mh,:),'all'), std(mean(E(mh,:),1))/sqrt(nS), mean(std(E(mh,:),0,2)), SEED, rms(E(mh,SEED)), mean(TR(mh,:),'all'), mean(std(TR(mh,:),0,2)));
        end
    end
    linkaxes(ax(1,:),'y'); linkaxes(ax(2,:),'y'); linkaxes(ax(3,:),'y'); linkaxes(ax(4,:),'y');
    png = fullfile(od, 'aptrue_two_traj_nw_mcorr_comp.png');
    exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
end
