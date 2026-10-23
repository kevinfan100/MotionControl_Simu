% PURPOSE (2026-09-14, user: "simulate the cell wall with our own a model and a cheated init, then look at b_hat"):
%   canon on the Fig-11 cell law (b_true 0.877, w_s,true -1.197) with the estimator seeded at exactly those numbers, so
%   the plant is a member of the estimator's own law family and there is no model error left. Two self-consistent arms:
%   original 5-state and the 4-state (gain computed). Top: gain error, seed mean +- sigma_seed. Bottom: b_hat seed mean
%   +- sqrt(P55) against the constant truth. Output test_results/apd_acov_meng/cell_wall_two_arms.png
%   | EXPIRES: with the b line | 產線改動不會自動跟上
function plot_cell_wall_two_arms(wall)
    if nargin < 1 || isempty(wall); wall = 'cell'; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    ARM = {'orig', '4state'};  NAME = {'original', '4-state'};  COL = {[0 0.2 0.9], [0.6 0 0.6]};  LW = [3.4 1.4];
    FS = 15;  LFS = 12;  AXLW = 1.8;  B_TRUE = 0.877;
    D = cell(1, 2);
    for a = 1:2; D{a} = load(fullfile(od, sprintf('amodel_fe_cl_canon_%s_%s.mat', ARM{a}, wall))); end
    ws_est = -0.1230;  if strcmp(wall, 'cellmis'); ws_est = 0.8694 - 1; end
    t = D{1}.t;  T = t(end);
    f = figure('Units','inches','Position',[0 0 7.5 7.5], 'Color','w', 'Visible','off');
    tiledlayout(2, 1, 'TileSpacing','compact', 'Padding','compact');
    nexttile; hold on; box on;  H = gobjects(1, 2);
    for a = 1:2
        mE = mean(D{a}.E, 2);  sE = std(D{a}.E, 0, 2);
        fill([t; flipud(t)], [mE+sE; flipud(mE-sE)], COL{a}, 'FaceAlpha', 0.15, 'EdgeColor','none');
        H(a) = plot(t, mE, '-', 'Color', COL{a}, 'LineWidth', LW(a));
    end
    plot([0 T], [0 0], 'k:', 'LineWidth', 1.2);
    legend(H, NAME, 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    ylabel('$\hat{\bar{a}} - \bar{a}$', 'Interpreter','latex', 'FontSize', FS+3);
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW); xlim([0 T]);
    if strcmp(wall, 'cellmis'); ylim([-0.3 0.05]); else; ylim([-0.05 0.05]); end
    nexttile; hold on; box on;  H = gobjects(1, 3);
    for a = 1:2
        mb = mean(D{a}.B, 2);  sp = mean(D{a}.sP5, 2);  sb = std(D{a}.B, 0, 2);
        fill([t; flipud(t)], [mb+sb; flipud(mb-sb)], COL{a}, 'FaceAlpha', 0.15, 'EdgeColor','none');
        H(a) = plot(t, mb, '-', 'Color', COL{a}, 'LineWidth', LW(a));
        plot(t, mb + sp, '--', 'Color', COL{a}, 'LineWidth', 1.2);  plot(t, mb - sp, '--', 'Color', COL{a}, 'LineWidth', 1.2);
    end
    if strcmp(wall, 'cellmis')
        hb = mean(D{1}.HB, 2);  a_t = 1 - 1./(B_TRUE*(hb + 1.197));
        H(3) = plot(t, 1 ./ ((1 - a_t) .* (hb - ws_est)), '-', 'Color', [0.85 0 0], 'LineWidth', 2.2);
        lab3 = 'b the wrong-origin law would need';
    else
        H(3) = plot([0 T], B_TRUE*[1 1], '-', 'Color', [0.85 0 0], 'LineWidth', 2.2);
        lab3 = 'b_{true} = 0.877';
    end
    legend(H, [NAME, {lab3}], 'Location','northoutside','Orientation','horizontal','FontSize',LFS,'FontWeight','bold','Box','on');
    ylabel('$\hat{b}$', 'Interpreter','latex', 'FontSize', FS+3);  xlabel('time [s]', 'FontSize', FS, 'FontWeight','bold');
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',AXLW); xlim([0 T]);
    if strcmp(wall, 'cellmis'); ylim([0.8 1.75]); else; ylim([0.80 0.95]); end
    fn = fullfile(od, sprintf('%s_wall_two_arms.png', wall));  exportgraphics(f, fn, 'Resolution', 150);  close(f);
    fprintf('figure: %s\n', fn);
end
