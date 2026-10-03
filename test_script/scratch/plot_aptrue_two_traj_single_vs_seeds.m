% STATUS: ACTIVE (scratch) | PURPOSE: the compensated a'_true arm on BOTH trajectories
%   side by side (left Meng, right canon), four rows: one run (seed 7) absolute /
%   10-seed mean +/- sigma band absolute / gain error with band / motion (tracking)
%   error R*delta_w3 [um], same-instant pairing hd[k]-w[k] (NOT the logged trk_true, which
%   pairs the command at k with the post-advance position and so carries +v*Ts, 2026-09-03). No arm or trajectory label in the figure (user request
%   2026-09-03). y shared per row.
function plot_aptrue_two_traj_single_vs_seeds()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    A = load(fullfile(od, 'arms10_seedtruth_comp_meng.mat')); B = load(fullfile(od, 'aptrue_canon_lr.mat'));
    D = {A.aptrue, B.right}; NM = {'Meng', 'canon'}; TH = [10.6 3.5]; R_um = 2.25;
    SEED = 7; COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95]; COL_SEED = [0.55 0.74 0.96];
    FS = 15; LFS = 11; AXLW = 1.8;
    f = figure('Units','inches','Position',[0 0 13 15.5], 'Color','w', 'Visible','off');
    tiledlayout(4, 2, 'TileSpacing','compact', 'Padding','compact'); ax = gobjects(4,2);
    for a = 1:2
        d = D{a}; t = d.t(:); T_END = ceil(t(end)); mh = t > TH(a);
        AH = d.C.a_bar_hat_out; AT = d.C.a_true_norm; E = d.E; nS = size(E,2);
        TR = (d.hd(:) - d.C.h_bar_true) * R_um;   % SAME-INSTANT tracking error hd[k]-w[k]; the logged trk_true is hd[k]-w[k+1] (post-advance) = this + one command step
        ax(1,a) = nexttile(a); hold on;
        ht = plot(t, AT(:,SEED), '-', 'Color', COL_TRUE, 'LineWidth', 2.0);
        hh = plot(t, AH(:,SEED), '-', 'Color', COL_HAT,  'LineWidth', 1.2);
        legend([hh ht], {sprintf('\\^a_z / a_{nom}   seed %d', SEED), 'a_z / a_{nom}  true'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('a_z / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        ax(2,a) = nexttile(2+a); hold on;
        m = mean(AH,2); s = std(AH,0,2);
        hb = fill([t; flipud(t)], [m+s; flipud(m-s)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor','none');
        ht = plot(t, mean(AT,2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0);
        hh = plot(t, m, '-', 'Color', COL_HAT, 'LineWidth', 1.5);
        legend([hh ht hb], {sprintf('mean \\^a_z / a_{nom}  (%d seeds)', nS), 'a_z / a_{nom}  true', '\pm \sigma_{seed}'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('a_z / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        ax(3,a) = nexttile(4+a); hold on;
        mE = mean(E,2); sE = std(E,0,2);
        hb = fill([t; flipud(t)], [mE+sE; flipud(mE-sE)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor','none');
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility','off');
        h1 = plot(t, E(:,SEED), '-', 'Color', COL_SEED, 'LineWidth', 0.6);
        hm = plot(t, mE, '-', 'Color', COL_HAT, 'LineWidth', 2.0);
        legend([hm hb h1], {'seed mean (\^a_z - a_z)/a_{nom}', '\pm \sigma_{seed}', sprintf('seed %d', SEED)}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('(\^a_z - a_z) / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        ax(4,a) = nexttile(6+a); hold on;
        ok = all(isfinite(TR),2); mT = mean(TR,2); sT = std(TR,0,2);
        hb = fill([t(ok); flipud(t(ok))], [mT(ok)+sT(ok); flipud(mT(ok)-sT(ok))], BANDC, 'FaceAlpha', 0.30, 'EdgeColor','none');
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility','off');
        h1 = plot(t, TR(:,SEED), '-', 'Color', COL_SEED, 'LineWidth', 0.6);
        hm = plot(t, mT, '-', 'Color', COL_HAT, 'LineWidth', 2.0);
        legend([hm hb h1], {'seed mean  R\deltaw_3', '\pm \sigma_{seed}', sprintf('seed %d', SEED)}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('R\deltaw_3  [\mum]', 'FontSize', FS, 'FontWeight','bold'); end
        xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        fprintf('[%s] hold: gain bias %+.5f | sigma_seed %.5f | seed %d hold rms %.5f | motion err hold mean %+.4f um, within-sd %.4f um, sigma_seed %.4f um\n', ...
            NM{a}, mean(E(mh,:),'all'), mean(sE(mh)), SEED, rms(E(mh,SEED)), mean(TR(mh&ok,:),'all'), mean(std(TR(mh&ok,:),0,1)), mean(sT(mh&ok)));
    end
    linkaxes(ax(1,:),'y'); linkaxes(ax(2,:),'y'); linkaxes(ax(3,:),'y'); linkaxes(ax(4,:),'y');
    png = fullfile(od, 'aptrue_two_traj_single_vs_seeds.png');
    exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
end
