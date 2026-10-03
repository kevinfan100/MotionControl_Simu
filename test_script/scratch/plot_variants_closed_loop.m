% PURPOSE (2026-09-14): figure 2 of the comparison reading copy (0915_estb_variants_compare.tex) -- the closed loop of
%   the three versions on the plane plant, chord seed, 10 paired seeds (mats from run_amodel_fe_closed_loop.m).
%   Rows: true height; gain error a_hat - a_bar (seed mean +- sigma_seed); b_hat (seed mean +- sqrt(P55)).
%   Output reference/eq17_analysis/derivation/figures/variants_closed_loop.png | EXPIRES: with the b line | 產線改動不會自動跟上
function plot_variants_closed_loop()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    md = fullfile(root, 'test_results', 'apd_acov_meng');
    od = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    TR = {'canon', 'meng'};  ARM = {'orig', 'orange', '4state'};  NAME = {'original', 'substituted', '4-state'};
    COL = {[0 0.2 0.9], [0.9 0.5 0], [0.6 0 0.6]};  LW = [3.4 2.0 1.1];
    FS = 15;  LFS = 12;  AXLW = 1.8;  nA = 3;
    f = figure('Units','inches','Position',[0 0 12 9], 'Color','w', 'Visible','off');
    tiledlayout(3, 2, 'TileSpacing','compact', 'Padding','compact');
    for c = 1:2
        D = cell(1, nA);
        for a = 1:nA; D{a} = load(fullfile(md, sprintf('amodel_fe_cl_%s_%s.mat', TR{c}, ARM{a}))); end
        t = D{1}.t;  T = t(end);
        nexttile(c); hold on; box on;
        h = plot(t, mean(D{1}.HB, 2), '-', 'Color', [0.35 0.35 0.35], 'LineWidth', 2.0);
        legend(h, {TR{c}}, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel('$\bar{w}$', 'Interpreter','latex', 'FontSize', FS+3); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW); xlim([0 T]);
        nexttile(2 + c); hold on; box on;  H = gobjects(1, nA);
        for a = 1:nA
            mE = mean(D{a}.E, 2);  sE = std(D{a}.E, 0, 2);
            fill([t; flipud(t)], [mE+sE; flipud(mE-sE)], COL{a}, 'FaceAlpha', 0.14, 'EdgeColor','none');
            H(a) = plot(t, mE, '-', 'Color', COL{a}, 'LineWidth', LW(a));
        end
        plot([0 T], [0 0], 'k:', 'LineWidth', 1.2);
        legend(H, NAME, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel('$\hat{\bar{a}} - \bar{a}$', 'Interpreter','latex', 'FontSize', FS+3); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW); xlim([0 T]); ylim([-0.025 0.012]);
        nexttile(4 + c); hold on; box on;  H = gobjects(1, nA);
        for a = 1:nA
            mb = mean(D{a}.B, 2);  sp = mean(D{a}.sP5, 2);
            fill([t; flipud(t)], [mb+sp; flipud(mb-sp)], COL{a}, 'FaceAlpha', 0.12, 'EdgeColor','none');
            H(a) = plot(t, mb, '-', 'Color', COL{a}, 'LineWidth', LW(a));
        end
        legend(H, NAME, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel('$\hat{b}$', 'Interpreter','latex', 'FontSize', FS+3); end
        xlabel('time [s]', 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW); xlim([0 T]); ylim([0.83 0.94]);
    end
    if ~exist(od, 'dir'); mkdir(od); end
    fn = fullfile(od, 'variants_closed_loop.png');  exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure: %s\n', fn);
end
