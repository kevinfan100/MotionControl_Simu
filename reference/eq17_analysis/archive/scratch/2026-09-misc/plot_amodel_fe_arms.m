% PURPOSE (2026-09-12, extended 09-14 with the 4-state arm): the plain comparison of the
%   closed-loop arms from run_amodel_fe_closed_loop.m -- original, green (amodel_fe), orange (amodel_fe + amodel_h),
%   4-state (gain_algebraic: the gain is the law value, no a slot).
%   Two rows only: the gain error and b_hat, seed mean over the same 10 seeds. No bands, no sqrt(P), no paired difference.
%   Line widths decrease with the arm order so an overlapping curve stays visible.
%   Output test_results/apd_acov_meng/amodel_fe_arms.png | EXPIRES: with the b line | 產線改動不會自動跟上
function plot_amodel_fe_arms(pick)
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    TR = {'canon', 'meng'};  ARM = {'orig', 'green', 'orange', '4state'};
    NAME = {'original', 'green', 'orange', '4-state'};
    COL = {[0 0.2 0.9], [0 0.6 0.25], [0.9 0.5 0], [0.6 0 0.6]};  LW = [4.0 2.8 1.8 1.0];
    if nargin >= 1 && ~isempty(pick)
        ARM = ARM(pick);  NAME = NAME(pick);  COL = COL(pick);  LW = [3.6 1.6];
    end
    tag = '';  if nargin >= 1 && ~isempty(pick) && numel(pick) == 2; tag = '_two'; end
    FS = 15;  LFS = 12;  AXLW = 1.8;  nA = numel(ARM);
    f = figure('Units','inches','Position',[0 0 12 7], 'Color','w', 'Visible','off');
    tiledlayout(2, 2, 'TileSpacing','compact', 'Padding','compact');
    for c = 1:2
        D = cell(1, nA);
        for a = 1:nA; D{a} = load(fullfile(od, sprintf('amodel_fe_cl_%s_%s.mat', TR{c}, ARM{a}))); end
        t = D{1}.t;  T = t(end);
        nexttile(c); hold on; box on;  H = gobjects(1, nA);
        for a = 1:nA; H(a) = plot(t, mean(D{a}.E, 2), '-', 'Color', COL{a}, 'LineWidth', LW(a)); end
        plot([0 T], [0 0], 'k:', 'LineWidth', 1.2);
        legend(H, NAME, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel('$\hat{\bar{a}} - \bar{a}$', 'Interpreter','latex', 'FontSize', FS+3); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW); xlim([0 T]); ylim([-0.02 0.01]);
        nexttile(2 + c); hold on; box on;  H = gobjects(1, nA);
        for a = 1:nA; H(a) = plot(t, mean(D{a}.B, 2), '-', 'Color', COL{a}, 'LineWidth', LW(a)); end
        legend(H, NAME, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
        if c == 1; ylabel('$\hat{b}$', 'Interpreter','latex', 'FontSize', FS+3); end
        xlabel('time [s]', 'FontSize', FS, 'FontWeight','bold');
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW); xlim([0 T]); ylim([0.86 0.94]);
    end
    fn = fullfile(od, sprintf('amodel_fe_arms%s.png', tag));  exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure: %s\n', fn);
end
