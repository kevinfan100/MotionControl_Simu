% PURPOSE (2026-09-06): read-out of run_btrue_prior_vs_offset -- the b_true arm's fast-descent spread as the amplified PRIOR P44[0].
%   Meng | canon. Row 1: sigma_seed(t) (solid) and sqrt(P44) (dashed): standard prior, kappa 1 (grey; from btrue_aa_scale k100),
%   prior matched to the seed-at-truth start, kappa 1 (blue; Meng p0small 3e-4, canon p0tiny 1e-5), kappa 0.5 with the standard
%   prior (light blue dotted; btrue_aa_scale k050). Row 2 (Meng only has the offset arms): seed-mean error with a REAL +0.0031
%   initial offset at kappa 1 (red) and kappa 0.5 (orange), +- sigma_seed bands; canon panel repeats row 1's E_l instead.
%   Output btrue_prior_vs_offset.png | EXPIRES: with the b_true rung | 產線改動不會自動跟上
function plot_btrue_prior_vs_offset()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    TR = {'meng', 'canon'}; NM = {'Meng', 'canon'}; SMALL = {'p0small', 'p0tiny'};
    CG = [0.45 0.45 0.45]; CB = [0 0.2 0.9]; CLB = [0.55 0.65 0.95]; CR = [0.8 0 0]; CO = [0.9 0.5 0.1];
    FS = 14; LFS = 10; AXLW = 1.6;
    f = figure('Units','inches','Position',[0 0 13 8.5], 'Color','w', 'Visible','off'); tiledlayout(2, 2, 'TileSpacing','compact', 'Padding','compact');
    for a = 1:2
        K = load(fullfile(od, sprintf('btrue_aa_scale_%s.mat', TR{a})));
        P = load(fullfile(od, sprintf('btrue_prior_vs_offset_%s.mat', TR{a})));
        if ~isfield(P, SMALL{a}); Q = load(fullfile(od, sprintf('btrue_prior_vs_offset_%s_%s.mat', TR{a}, SMALL{a}))); P.(SMALL{a}) = Q.(SMALL{a}); end
        t = K.k100.t(:); T_END = ceil(t(end)); d = P.(SMALL{a});
        nexttile(a); hold on;
        h1 = plot(t, std(K.k100.E, 0, 2), '-', 'Color', CG, 'LineWidth', 1.4);  plot(t, mean(K.k100.sP, 2), '--', 'Color', CG, 'LineWidth', 1.0);
        h3 = plot(t, std(K.k050.E, 0, 2), ':', 'Color', CLB, 'LineWidth', 1.4);
        h2 = plot(t, std(d.E, 0, 2), '-', 'Color', CB, 'LineWidth', 1.8);  h4 = plot(t, mean(d.sP, 2), '--', 'Color', CR, 'LineWidth', 1.2);
        xline(K.win(1), ':', 'Color', [0.3 0.3 0.3]); xline(K.win(2), ':', 'Color', [0.3 0.3 0.3]); xline(K.t_hold, '--', 'Color', [0.3 0.3 0.3]);
        legend([h1 h3 h2 h4], {sprintf('standard prior \\surd P_{44}[0] = %.5f, \\kappa=1', mean(K.k100.sP(1,:))), 'standard prior, \kappa=0.5', sprintf('prior %.0e (seed-at-truth), \\kappa=1', mean(d.sP(1,:))), '\surd P_{44} of that arm'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on','NumColumns',2);
        if a == 1; ylabel('b_{true} arm:  \sigma_{seed}  (\^a_z - a_z)/a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]); ylim([0 0.04]);
        nexttile(2 + a); hold on; yline(0, '-', 'Color', [0.5 0.5 0.5], 'HandleVisibility','off');
        if a == 1 && isfield(P, 'off_k1')
            H = []; L = {};
            for ar = {'off_k1', 'off_k05'}
                if ~isfield(P, ar{1}); continue; end
                dd = P.(ar{1}); m = mean(dd.E, 2); s = std(dd.E, 0, 2); c = CR; if strcmp(ar{1}, 'off_k05'); c = CO; end
                fill([t; flipud(t)], [m+s; flipud(m-s)], c, 'FaceAlpha', 0.15, 'EdgeColor','none', 'HandleVisibility','off');
                H(end+1) = plot(t, m, '-', 'Color', c, 'LineWidth', 1.8); L{end+1} = sprintf('seed +%.4f offset, \\kappa=%.1f', dd.delta, dd.kappa);
                plot(t, mean(dd.sP, 2), '--', 'Color', c, 'LineWidth', 1.0);
            end
            xline(K.win(1), ':', 'Color', [0.3 0.3 0.3]); xline(K.win(2), ':', 'Color', [0.3 0.3 0.3]); xline(K.t_hold, '--', 'Color', [0.3 0.3 0.3]);
            legend(H, L, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
            ylabel('real initial offset:  seed mean \pm \sigma_{seed},  dashed \surd P_{44}', 'FontSize', FS, 'FontWeight','bold');
        else
            h1 = plot(t, mean(K.k100.E_l, 2), '-', 'Color', CG, 'LineWidth', 1.4); h2 = plot(t, mean(d.E_l, 2), '-', 'Color', CB, 'LineWidth', 1.8);
            xline(K.win(1), ':', 'Color', [0.3 0.3 0.3]); xline(K.win(2), ':', 'Color', [0.3 0.3 0.3]); xline(K.t_hold, '--', 'Color', [0.3 0.3 0.3]);
            legend([h1 h2], {'l_{41} + a''_z l_{31}   standard prior', 'l_{41} + a''_z l_{31}   prior 1e-5'}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
            ylabel('l_{41} + a''_z l_{31}  (seed mean)', 'FontSize', FS, 'FontWeight','bold');
        end
        xlabel(sprintf('time [s]   (%s; dotted = fast window, dashed = hold start)', NM{a}), 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T_END]);
    end
    png = fullfile(od, 'btrue_prior_vs_offset.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('saved %s\n', png);
end
