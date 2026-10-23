% PURPOSE (2026-09-11, step 4 of the green plan): figure for run_amodel_fe_closed_loop.m -- orig (production) vs green (amodel_fe),
%   three arms (orig / green / orange), 10 seeds paired, canon (left) and Meng (right). Rows: true height (seed mean); gain error seed mean +- sigma_seed; the paired
%   difference green - orig (mean +- 2 SEM); a honesty
%   (sigma_seed of the gain error vs the filter's sqrt(P44)); b_hat seed mean +- sigma_seed with +- sqrt(P55).
%   Style copied from plot_ladder_tw_abs.m (09-08). Output test_results/apd_acov_meng/amodel_fe_closed_loop.png
%   | EXPIRES: with the green verdict | 產線改動不會自動跟上
function plot_amodel_fe_closed_loop()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    TR = {'canon', 'meng'};  ARM = {'orig', 'green', 'orange'};  NAME = {'original', 'green', 'orange'};
    COL = {[0 0.2 0.9], [0 0.6 0.25], [0.9 0.5 0]};  nA = numel(ARM);  FS = 15;  LFS = 11;  AXLW = 1.8;
    f = figure('Units','inches','Position',[0 0 13 16], 'Color','w', 'Visible','off');
    tiledlayout(5, 2, 'TileSpacing','compact', 'Padding','compact');
    for c = 1:2
        D = cell(1, nA);
        for a = 1:nA; D{a} = load(fullfile(od, sprintf('amodel_fe_cl_%s_%s.mat', TR{c}, ARM{a}))); end
        t = D{1}.t;  T = t(end);
        % row 1: height (seed-mean true height; the runner logs no commanded height, tracking error ~0.01 R)
        nexttile(c); hold on;
        h = plot(t, mean(D{1}.HB, 2), '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 2.0);
        legend(h, {TR{c}}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel('$\bar{w}$', 'Interpreter','latex', 'FontSize', FS+2); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T]);
        % row 2: gain error
        nexttile(2 + c); hold on;  H = gobjects(1, nA);
        for a = 1:nA
            mE = mean(D{a}.E, 2);  sE = std(D{a}.E, 0, 2);
            fill([t; flipud(t)], [mE+sE; flipud(mE-sE)], COL{a}, 'FaceAlpha', 0.18, 'EdgeColor','none');
            H(a) = plot(t, mE, '-', 'Color', COL{a}, 'LineWidth', 2.0);
        end
        plot([0 T], [0 0], 'k:', 'LineWidth', 1.2);
        legend(H, {'original  mean \pm \sigma_{seed}', 'green', 'orange'}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel('$\hat{\bar{a}} - \bar{a}$', 'Interpreter','latex', 'FontSize', FS+2); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T]); ylim([-0.06 0.06]);
        % row 3: paired difference green - original (same seed), seed mean +- SEM
        nexttile(4 + c); hold on;
        H = gobjects(1, nA-1);
        for a = 2:nA
            dE = D{a}.E - D{1}.E;  mD = mean(dE, 2);  sD = std(dE, 0, 2) / sqrt(size(dE, 2));
            fill([t; flipud(t)], [mD+2*sD; flipud(mD-2*sD)], COL{a}, 'FaceAlpha', 0.20, 'EdgeColor','none');
            H(a-1) = plot(t, mD, '-', 'Color', COL{a}, 'LineWidth', 1.8);
        end
        plot([0 T], [0 0], 'k:', 'LineWidth', 1.2);
        legend(H, {'green - original  mean \pm 2 SEM', 'orange - original'}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel('$\Delta(\hat{\bar{a}} - \bar{a})$', 'Interpreter','latex', 'FontSize', FS+2); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T]); ylim([-0.003 0.003]);
        % row 4: a honesty
        nexttile(6 + c); hold on;  H = gobjects(1, 2*nA);
        for a = 1:nA
            H(a)    = plot(t, std(D{a}.E, 0, 2), '-', 'Color', COL{a}, 'LineWidth', 2.0);
            H(nA+a) = plot(t, mean(D{a}.sP4, 2), '--', 'Color', COL{a}, 'LineWidth', 2.0);
        end
        legend(H, {'original \sigma_{seed}', 'green \sigma_{seed}', 'orange \sigma_{seed}', 'original \surd P_{44}', 'green \surd P_{44}', 'orange \surd P_{44}'}, 'Location','northoutside', ...
               'Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on','NumColumns',2);
        if c == 1; ylabel('spread of $\hat{\bar{a}}$', 'Interpreter','latex', 'FontSize', FS+2); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T]); ylim([0 0.05]);
        % row 5: b_hat
        nexttile(8 + c); hold on;  H = gobjects(1, 2*nA);
        for a = 1:nA
            mb = mean(D{a}.B, 2);  sb = std(D{a}.B, 0, 2);  sp = mean(D{a}.sP5, 2);
            fill([t; flipud(t)], [mb+sb; flipud(mb-sb)], COL{a}, 'FaceAlpha', 0.18, 'EdgeColor','none');
            H(a) = plot(t, mb, '-', 'Color', COL{a}, 'LineWidth', 2.0);
            H(nA+a) = plot(t, mb + sp, '--', 'Color', COL{a}, 'LineWidth', 1.5);  plot(t, mb - sp, '--', 'Color', COL{a}, 'LineWidth', 1.5);
        end
        legend(H, {'original  mean \pm \sigma_{seed}', 'green', 'orange', 'original \pm \surd P_{55}', 'green \pm \surd P_{55}', 'orange \pm \surd P_{55}'}, ...
               'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on','NumColumns',2);
        if c == 1; ylabel('$\hat{b}$', 'Interpreter','latex', 'FontSize', FS+2); end
        xlabel('time [s]', 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW,'Box','on'); grid off; xlim([0 T]); ylim([0.80 0.98]);
    end
    fn = fullfile(od, 'amodel_fe_closed_loop.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure: %s\n', fn);
end
