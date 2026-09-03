% FORK OF test_script/scratch/plot_aptrue_two_traj_single_vs_seeds.m (2026-09-03) | PURPOSE: the same
%   four rows (seed 7 absolute / 10-seed mean +- sigma absolute / gain error with band + seed 7 /
%   same-instant tracking error R dw3 [um]), left Meng, right canon, for the FINAL a'_true recipe
%   (exact + slope@est + pred_mean2 + nw_mcorr, the nwmcorr arm of run_aptrue_nw_mcorr_full.m).
%   No arm or trajectory label in the figure (user request 2026-09-03). y shared per row.
%   | EXPIRES: with the parent | 產線改動不會自動跟上
function plot_aptrue_two_traj_final(three_rows)
    if nargin < 1; three_rows = false; end   % true: drop row 2 (10-seed mean absolute) -> 3x2, for the derivation's last page
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    A = load(fullfile(od, 'aptrue_nw_mcorr_full_meng.mat')); B = load(fullfile(od, 'aptrue_nw_mcorr_full_canon.mat'));
    D = {A.nwmcorr, B.nwmcorr}; NM = {'Meng', 'canon'}; TH = [A.t_hold B.t_hold]; R_um = 2.25;
    SEED = 7; COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95]; COL_SEED = [0.55 0.74 0.96];
    FS = 15; LFS = 11; AXLW = 1.8;
    NR = 4; if three_rows; NR = 3; end
    f = figure('Units','inches','Position',[0 0 13 3.9*NR], 'Color','w', 'Visible','off');
    tiledlayout(NR, 2, 'TileSpacing','compact', 'Padding','compact'); ax = gobjects(4,2);
    for a = 1:2
        d = D{a}; t = d.t(:); T_END = ceil(t(end)); mh = t > TH(a);
        AH = d.AH; AT = d.AT; E = d.E; nS = size(E,2);
        TR = (d.hd(:) - d.HB) * R_um;   % SAME-INSTANT tracking error hd[k]-w[k]
        ax(1,a) = nexttile(a); hold on;
        ht = plot(t, AT(:,SEED), '-', 'Color', COL_TRUE, 'LineWidth', 2.0);
        hh = plot(t, AH(:,SEED), '-', 'Color', COL_HAT,  'LineWidth', 1.2);
        legend([hh ht], {sprintf('\\^a_z / a_{nom}   seed %d', SEED), 'a_z / a_{nom}  true'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('a_z / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        if ~three_rows
        ax(2,a) = nexttile(2+a); hold on;
        m = mean(AH,2); s = std(AH,0,2);
        hb = fill([t; flipud(t)], [m+s; flipud(m-s)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor','none');
        ht = plot(t, mean(AT,2), '-', 'Color', COL_TRUE, 'LineWidth', 2.0);
        hh = plot(t, m, '-', 'Color', COL_HAT, 'LineWidth', 1.5);
        legend([hh ht hb], {sprintf('mean \\^a_z / a_{nom}  (%d seeds)', nS), 'a_z / a_{nom}  true', '\pm \sigma_{seed}'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('a_z / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        end
        ax(3,a) = nexttile(2*(NR-2)+a); hold on;
        mE = mean(E,2); sE = std(E,0,2);
        hb = fill([t; flipud(t)], [mE+sE; flipud(mE-sE)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor','none');
        yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility','off');
        h1 = plot(t, E(:,SEED), '-', 'Color', COL_SEED, 'LineWidth', 0.6);
        hm = plot(t, mE, '-', 'Color', COL_HAT, 'LineWidth', 2.0);
        legend([hm hb h1], {'seed mean (\^a_z - a_z)/a_{nom}', '\pm \sigma_{seed}', sprintf('seed %d', SEED)}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if a == 1; ylabel('(\^a_z - a_z) / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
        ax(4,a) = nexttile(2*(NR-1)+a); hold on;
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
        fprintf('[%s] hold: gain bias %+.5f (SEM %.5f) | sigma_seed %.5f | seed %d hold rms %.5f | motion err hold mean %+.4f um, within-sd %.4f um, sigma_seed %.4f um\n', ...
            NM{a}, mean(E(mh,:),'all'), std(mean(E(mh,:),1))/sqrt(nS), mean(sE(mh)), SEED, rms(E(mh,SEED)), mean(TR(mh&ok,:),'all'), mean(std(TR(mh&ok,:),0,1)), mean(sT(mh&ok)));
    end
    linkaxes(ax(1,:),'y'); if ~three_rows; linkaxes(ax(2,:),'y'); end; linkaxes(ax(3,:),'y'); linkaxes(ax(4,:),'y');
    if three_rows; png = fullfile(od, 'aptrue_two_traj_final_3row.png'); else; png = fullfile(od, 'aptrue_two_traj_final.png'); end
    exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
end
