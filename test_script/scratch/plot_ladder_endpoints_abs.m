% FORK OF test_script/scratch/plot_ladder_endpoints.m (2026-09-04) | PURPOSE: the same ladder figure set with
%   (i) the gain error in ABSOLUTE physical units, a_hat_z - a_z [um/pN] (= a_nom x the normalized error), and
%   (ii) a fourth row for b: red = b_true at the particle's true height (the btest arm's b_used, same seeds and
%   trajectory), blue = the b this arm's law used -- a locked constant (apcmd / apest), the fed b_true curve
%   (btcmd / btest) or the ESTIMATE b_hat (bhat: 10-seed mean, band +- sqrt(P55) mean over seeds, dotted +- sd over seeds),
%   dashed grey = 8/9. Rows 1 and 3 as in the parent (seed 7 absolute gain vs true; R dw3 [um]). Shared y per row
%   across the arms. Output ladder_<arm>_4row_abs.png. | EXPIRES: with the ladder | 產線改動不會自動跟上
function plot_ladder_endpoints_abs(arms, tag)
    if nargin < 1 || isempty(arms); arms = {'apcmd','btcmd','apest','btest','bhat'}; end
    if nargin < 2; tag = ''; end                            % '' = the kappa = 1 set, '_k050' = production kappa = 0.5
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    A = load(fullfile(od, ['ladder_endpoints_meng' tag '.mat'])); B = load(fullfile(od, ['ladder_endpoints_canon' tag '.mat']));
    NM = {'Meng', 'canon'}; TH = [A.t_hold B.t_hold]; R_um = 2.25;  SEED = 7;
    COL_TRUE = [0.8 0 0]; COL_HAT = [0 0.2 0.9]; BANDC = [0.45 0.55 0.95]; COL_SEED = [0.55 0.74 0.96];
    FS = 15; LFS = 11; AXLW = 1.8;
    SRC = {A, B};
    yE = [0 0]; yT = [0 0]; yB = [1 1] * 8/9;
    for ia = 1:numel(arms)
        for s = 1:2
            d = SRC{s}.(arms{ia});  Ea = d.E * d.a_nom;  mE = mean(Ea,2); sE = std(Ea,0,2);  TR = (d.hd(:) - d.HB) * R_um;  mT = mean(TR,2); sT = std(TR,0,2);
            yE = [min(yE(1), min([mE - sE; Ea(:,SEED)])), max(yE(2), max([mE + sE; Ea(:,SEED)]))];
            yT = [min(yT(1), min([mT - sT; TR(:,SEED)])), max(yT(2), max([mT + sT; TR(:,SEED)]))];
            mb = mean(d.B,2); sp = mean(d.sP5,2);  yB = [min(yB(1), min(mb - sp)), max(yB(2), max(mb + sp))];
            bt = mean(SRC{s}.btest.B, 2);  yB = [min(yB(1), min(bt)), max(yB(2), max(bt))];
        end
    end
    yE = yE + 0.05 * diff(yE) * [-1 1];  yT = yT + 0.05 * diff(yT) * [-1 1];  yB = yB + 0.08 * diff(yB) * [-1 1];
    for ia = 1:numel(arms)
        arm = arms{ia};
        f = figure('Units','inches','Position',[0 0 13 15.6], 'Color','w', 'Visible','off');
        tiledlayout(4, 2, 'TileSpacing','compact', 'Padding','compact');
        for a = 1:2
            d = SRC{a}.(arm); t = d.t(:); T_END = ceil(t(end)); mh = t > TH(a);  AH = d.AH; AT = d.AT; nS = size(AH,2);
            Ea = d.E * d.a_nom;  TR = (d.hd(:) - d.HB) * R_um;  bt = mean(SRC{a}.btest.B, 2);
            nexttile(a); hold on;
            ht = plot(t, AT(:,SEED), '-', 'Color', COL_TRUE, 'LineWidth', 2.0);
            hh = plot(t, AH(:,SEED), '-', 'Color', COL_HAT,  'LineWidth', 1.2);
            legend([hh ht], {sprintf('\\^a_z / a_{nom}   seed %d', SEED), 'a_z / a_{nom}  true'}, ...
                   'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
            if a == 1; ylabel('a_z / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
            set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]); ylim([0 1]);
            nexttile(2+a); hold on;
            mE = mean(Ea,2); sE = std(Ea,0,2);
            hb = fill([t; flipud(t)], [mE+sE; flipud(mE-sE)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor','none');
            yline(0, '-', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, 'HandleVisibility','off');
            h1 = plot(t, Ea(:,SEED), '-', 'Color', COL_SEED, 'LineWidth', 0.6);
            hm = plot(t, mE, '-', 'Color', COL_HAT, 'LineWidth', 2.0);
            legend([hm hb h1], {'seed mean  \^a_z - a_z', '\pm \sigma_{seed}', sprintf('seed %d', SEED)}, ...
                   'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
            if a == 1; ylabel('\^a_z - a_z  [\mum/pN]', 'FontSize', FS, 'FontWeight','bold'); end
            set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]); ylim(yE);
            nexttile(4+a); hold on;
            hr = plot(t, bt, '-', 'Color', COL_TRUE, 'LineWidth', 2.0);
            hg = yline(8/9, '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.2);
            mb = mean(d.B,2); sb = std(d.B,0,2); sp = mean(d.sP5,2);
            if any(strcmp(arm, {'bhat','bhp0','bhq5','bhq5s','bhn1'}))
                hb = fill([t; flipud(t)], [mb+sp; flipud(mb-sp)], BANDC, 'FaceAlpha', 0.30, 'EdgeColor','none');
                hs = plot(t, mb + sb, ':', 'Color', COL_HAT, 'LineWidth', 1.0); plot(t, mb - sb, ':', 'Color', COL_HAT, 'LineWidth', 1.0);
                hm = plot(t, mb, '-', 'Color', COL_HAT, 'LineWidth', 2.0);
                legend([hm hb hs hr hg], {'\^b  seed mean', '\pm \surd P_{55}', '\pm \sigma_{seed}', 'b_{true}(w)  at the true height', '8/9'}, ...
                       'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
            else
                hm = plot(t, mb, '-', 'Color', COL_HAT, 'LineWidth', 1.6);
                legend([hm hr hg], {'b used by the law', 'b_{true}(w)  at the true height', '8/9'}, ...
                       'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
            end
            if a == 1; ylabel('b', 'FontSize', FS, 'FontWeight','bold'); end
            set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]); ylim(yB);
            nexttile(6+a); hold on;
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
            fprintf('[%-5s %-5s] hold: a_hat - a %+.3e um/pN (SEM %.1e) | sigma_seed %.1e um/pN | a_nom %.4e | b used/est hold mean %.4f (sd over seeds %.4f, sqrt(P55) %.4f) | b_true(w_hold) %.4f\n', ...
                arm, NM{a}, mean(Ea(mh,:),'all'), std(mean(Ea(mh,:),1))/sqrt(nS), mean(sE(mh)), d.a_nom, mean(mb(mh)), mean(sb(mh)), mean(sp(mh)), mean(bt(mh)));
        end
        png = fullfile(od, sprintf('ladder_%s_4row_abs%s.png', arm, tag)); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
    end
end
