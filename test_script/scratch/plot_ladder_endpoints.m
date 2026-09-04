% FORK OF test_script/scratch/plot_aptrue_two_traj_final.m (three_rows branch, 2026-09-04) | PURPOSE: one figure per
%   end point of the September oracle ladder (apcmd / btcmd / apest / btest / bhat, see run_ladder_endpoints.m), each 3 x 2:
%   left Meng, right canon; rows = seed 7 absolute gain vs true / gain error (10-seed mean +- sigma, seed 7) /
%   same-instant tracking error R dw3 [um] (mean +- sigma, seed 7). The y range of every row is SHARED ACROSS THE FOUR
%   FIGURES (max over arms), so the early arms' large errors and the current recipes are directly comparable.
%   No arm or trajectory label inside the figure (09-03 convention); the arm is in the file name ladder_<arm>_3row.png.
%   Colours and notation verbatim from the parent. | EXPIRES: with the ladder | 產線改動不會自動跟上
function plot_ladder_endpoints(arms)
    if nargin < 1 || isempty(arms); arms = {'apcmd','btcmd','apest','btest','bhat'}; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    A = load(fullfile(od, 'ladder_endpoints_meng.mat')); B = load(fullfile(od, 'ladder_endpoints_canon.mat'));
    NM = {'Meng', 'canon'}; TH = [A.t_hold B.t_hold]; R_um = 2.25;  SEED = 7;
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95]; COL_SEED = [0.55 0.74 0.96];
    FS = 15; LFS = 11; AXLW = 1.8;
    % shared y ranges per row over the four arms (rows 2 and 3; row 1 is the gain itself, [0 1])
    yE = [0 0]; yT = [0 0];
    for ia = 1:numel(arms)
        for src = {A, B}
            d = src{1}.(arms{ia});  mE = mean(d.E,2); sE = std(d.E,0,2);  TR = (d.hd(:) - d.HB) * R_um;  mT = mean(TR,2); sT = std(TR,0,2);
            yE = [min(yE(1), min([mE - sE; d.E(:,SEED)])), max(yE(2), max([mE + sE; d.E(:,SEED)]))];
            yT = [min(yT(1), min([mT - sT; TR(:,SEED)])), max(yT(2), max([mT + sT; TR(:,SEED)]))];
        end
    end
    yE = yE + 0.05 * diff(yE) * [-1 1];  yT = yT + 0.05 * diff(yT) * [-1 1];
    for ia = 1:numel(arms)
        arm = arms{ia};  D = {A.(arm), B.(arm)};
        f = figure('Units','inches','Position',[0 0 13 11.7], 'Color','w', 'Visible','off');
        tiledlayout(3, 2, 'TileSpacing','compact', 'Padding','compact'); ax = gobjects(3,2);
        for a = 1:2
            d = D{a}; t = d.t(:); T_END = ceil(t(end)); mh = t > TH(a);  AH = d.AH; AT = d.AT; E = d.E; nS = size(E,2);
            TR = (d.hd(:) - d.HB) * R_um;
            ax(1,a) = nexttile(a); hold on;
            ht = plot(t, AT(:,SEED), '-', 'Color', COL_TRUE, 'LineWidth', 2.0);
            hh = plot(t, AH(:,SEED), '-', 'Color', COL_HAT,  'LineWidth', 1.2);
            legend([hh ht], {sprintf('\\^a_z / a_{nom}   seed %d', SEED), 'a_z / a_{nom}  true'}, ...
                   'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
            if a == 1; ylabel('a_z / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
            set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]); ylim([0 1]);
            ax(2,a) = nexttile(2+a); hold on;
            mE = mean(E,2); sE = std(E,0,2);
            hb = fill([t; flipud(t)], [mE+sE; flipud(mE-sE)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor','none');
            yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility','off');
            h1 = plot(t, E(:,SEED), '-', 'Color', COL_SEED, 'LineWidth', 0.6);
            hm = plot(t, mE, '-', 'Color', COL_HAT, 'LineWidth', 2.0);
            legend([hm hb h1], {'seed mean (\^a_z - a_z)/a_{nom}', '\pm \sigma_{seed}', sprintf('seed %d', SEED)}, ...
                   'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
            if a == 1; ylabel('(\^a_z - a_z) / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
            set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]); ylim(yE);
            ax(3,a) = nexttile(4+a); hold on;
            ok = all(isfinite(TR),2); mT = mean(TR,2); sT = std(TR,0,2);
            hb = fill([t(ok); flipud(t(ok))], [mT(ok)+sT(ok); flipud(mT(ok)-sT(ok))], BANDC, 'FaceAlpha', 0.30, 'EdgeColor','none');
            yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility','off');
            h1 = plot(t, TR(:,SEED), '-', 'Color', COL_SEED, 'LineWidth', 0.6);
            hm = plot(t, mT, '-', 'Color', COL_HAT, 'LineWidth', 2.0);
            legend([hm hb h1], {'seed mean  R\deltaw_3', '\pm \sigma_{seed}', sprintf('seed %d', SEED)}, ...
                   'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
            if a == 1; ylabel('R\deltaw_3  [\mum]', 'FontSize', FS, 'FontWeight','bold'); end
            xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold');
            set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]); ylim(yT);
            fprintf('[%-5s %-5s] hold: est-true %+.5f (SEM %.5f) = %+.1f%% of a(wall) | sigma_seed %.5f | seed %d hold rms %.5f | R dw3 hold mean %+.4f um, sigma_seed %.4f um\n', ...
                arm, NM{a}, mean(E(mh,:),'all'), std(mean(E(mh,:),1))/sqrt(nS), 100*mean(E(mh,:),'all')/mean(AT(mh,:),'all'), mean(sE(mh)), SEED, rms(E(mh,SEED)), mean(TR(mh&ok,:),'all'), mean(sT(mh&ok)));
        end
        png = fullfile(od, sprintf('ladder_%s_3row.png', arm)); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
    end
end
