% FORK OF test_script/scratch/plot_btrue_e4.m (2026-09-04) | PURPOSE: the production rung side by side, left canon, right Meng.
%   Row 1: 10-seed mean of (a_hat - a)/a_nom, 0.5 s window, for the b_true-curve arm (A, btrue_e4 e4), b locked 8/9 (lockb),
%          b locked at the wall value (lockw, oracle discriminator), b estimated (prod), production as it was (hist).
%   Row 2: paired differences to the curve arm (lockb - A, lockw - A, prod - A), +-2 SEM bands.
%   Row 3: b_hat of the prod arm (10-seed mean +- sd) against b_true at the true height (curve arm's b_used) and 8/9.
%   Dashed vertical = hold start, dotted = end of the standard run. | EXPIRES: with the production rung
function plot_prod_ladder()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));  od = fullfile(root, 'test_results', 'apd_acov_meng');
    pc = physical_constants();  TR = {'canon','meng'};  FS = 13;  SM = round(0.5 / pc.Ts);
    ARM = {'A','lockb','lockw','prod','hist'};
    COL = {[0.85 0.33 0.10], [0 0.2 0.9], [0.49 0.18 0.56], [0 0.6 0.3], [0.55 0.55 0.55]};
    LAB = {'b_{true}(w) curve (A)', 'b locked 8/9', 'b locked at b_{true}(w_{hold})', 'b\_hat estimated (prod)', 'production as it was (Euler)'};
    f = figure('Units','inches','Position',[0 0 14 12.5], 'Color','w', 'Visible','off'); tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
    for it = 1:2
        P = load(fullfile(od, sprintf('prod_ladder_%s.mat', TR{it})));
        A = load(fullfile(od, sprintf('btrue_e4_%s.mat', TR{it})));
        C = load(fullfile(od, sprintf('btrue_nw_mcorr_%s.mat', TR{it})));
        D = struct('A', A.e4);  for a = 2:5; if isfield(P, ARM{a}); D.(ARM{a}) = P.(ARM{a}); end; end
        t = D.A.t;  nS = size(D.A.E, 2);  T_END = ceil(t(end));
        nexttile(it); hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off'); H = []; L = {};
        for a = 1:5
            if ~isfield(D, ARM{a}); continue; end
            Es = movmean(D.(ARM{a}).E, SM, 1);  m = mean(Es, 2);  s = std(Es, 0, 2) / sqrt(nS);
            fill([t; flipud(t)], [m + 2*s; flipud(m - 2*s)], COL{a}, 'FaceAlpha', 0.08, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            H(end+1) = plot(t, m, '-', 'Color', COL{a}, 'LineWidth', 1.8);  L{end+1} = LAB{a};
        end
        xline(P.phases(3), '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.1, 'HandleVisibility', 'off');
        xline(P.phases(4) - 4, ':', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.1, 'HandleVisibility', 'off');
        legend(H, L, 'Location','northoutside','Orientation','horizontal','FontSize',8,'FontWeight','bold','Box','on','NumColumns',3);
        if it == 1; ylabel('(\^a_z - a_z)/a_{nom}:  10-seed mean, 0.5 s window', 'FontSize', 12, 'FontWeight','bold'); end
        title(TR{it}, 'FontSize', FS, 'FontWeight', 'bold');
        xlim([0 T_END]); ylim([-0.010 0.030]);
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.5,'Box','on'); grid off;
        nexttile(it + 2); hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off'); H = []; L = {};
        for a = 2:4
            if ~isfield(D, ARM{a}); continue; end
            dE = D.(ARM{a}).E - D.A.E;  md = mean(dE, 2);  sd = std(dE, 0, 2) / sqrt(nS);
            fill([t; flipud(t)], [md + 2*sd; flipud(md - 2*sd)], COL{a}, 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            H(end+1) = plot(t, md, '-', 'Color', COL{a}, 'LineWidth', 1.8);  L{end+1} = sprintf('%s  minus  curve arm', LAB{a});
        end
        xline(P.phases(3), '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.1, 'HandleVisibility', 'off');
        legend(H, L, 'Location','northoutside','Orientation','horizontal','FontSize',8,'FontWeight','bold','Box','on');
        if it == 1; ylabel('paired difference of (\^a_z - a_z)/a_{nom}', 'FontSize', 12, 'FontWeight','bold'); end
        xlim([0 T_END]); ylim([-0.006 0.003]);
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.5,'Box','on'); grid off;
        nexttile(it + 4); hold on;
        bt = mean(C.nwmcorr.b_used, 2);   % b_true at the true height, curve arm (10-seed mean)
        h1 = plot(t, bt, '-', 'Color', [0.8 0 0], 'LineWidth', 2.0);
        h2 = yline(8/9, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2);
        H = [h1 h2]; L = {'b_{true}(w) at the true height', '8/9 (seed, locked value)'};
        if isfield(D, 'prod')
            mb = mean(D.prod.B, 2);  sb = std(D.prod.B, 0, 2);  sp = mean(D.prod.sP5, 2);
            fill([t; flipud(t)], [mb + sp; flipud(mb - sp)], COL{4}, 'FaceAlpha', 0.10, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            h3 = plot(t, mb, '-', 'Color', COL{4}, 'LineWidth', 1.8);
            h4 = plot(t, mb + sb, ':', 'Color', COL{4}, 'LineWidth', 1.0); plot(t, mb - sb, ':', 'Color', COL{4}, 'LineWidth', 1.0);
            H = [H h3 h4]; L = [L, {'\^b  prod (10-seed mean; band \pm\surd P_{55})', '\^b  \pm sd over seeds'}];
        end
        if isfield(D, 'hist')
            mbh = mean(D.hist.B, 2);  h5 = plot(t, mbh, '-', 'Color', COL{5}, 'LineWidth', 1.4);  H = [H h5]; L = [L, {'\^b  production as it was'}];
        end
        xline(P.phases(3), '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.1, 'HandleVisibility', 'off');
        legend(H, L, 'Location','northoutside','Orientation','horizontal','FontSize',8,'FontWeight','bold','Box','on','NumColumns',3);
        if it == 1; ylabel('b', 'FontSize', 12, 'FontWeight','bold'); end
        xlabel('time  [s]   (dashed: hold start, dotted: end of the standard run)', 'FontSize', 11, 'FontWeight','bold');
        xlim([0 T_END]); ylim([0.80 1.00]);
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.5,'Box','on'); grid off;
    end
    png = fullfile(od, 'prod_ladder_band.png'); exportgraphics(f, png, 'Resolution', 150); close(f); fprintf('wrote %s\n', png);
end
