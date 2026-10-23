% STATUS: ACTIVE (scratch) | PURPOSE: Euler (arm 1) against the fully compensated
%   arm (arm 4: exact step + slope at the estimated height + pred_mean2) of the
%   a'_true cheat ladder (run_aptrue_cheat_ladder.m), same 8 seeds. Left/right
%   share the y axis (figure-style rule). Row 1: per-seed error and seed mean.
%   Row 2: paired difference Euler - final (what the three layers removed in total).
%   Row 3: spread across seeds, Euler vs final -- the compensation moves the mean,
%   not the spread. Output: test_results/apd_acov_meng/aptrue_euler_vs_final.png
function plot_aptrue_euler_vs_final()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    L = load(fullfile(od, 'aptrue_cheat_ladder.mat'));  t = L.t;  E1 = L.E{1};  E4 = L.E{4};  nS = size(E1, 2);
    A_TROUGH = 0.087;                       % a_bar at the canonical trough, for the relative reading
    m_h = t > 3.5;  m_o = t > 1.5 & t <= 3.5;
    stat = @(E, m) [mean(mean(E(m,:),1)), std(mean(E(m,:),1))/sqrt(nS), mean(std(E(m,:),0,2))];
    s1h = stat(E1, m_h);  s4h = stat(E4, m_h);  s1o = stat(E1, m_o);  s4o = stat(E4, m_o);
    dh = mean(E1(m_h,:),1) - mean(E4(m_h,:),1);
    fprintf('[Euler vs final] %d seeds, est - true in a_bar units (relative to trough a_bar = %.3f in brackets)\n', nS, A_TROUGH);
    fprintf('  hold  Euler %+.5f (%+.1f%%)  final %+.5f (%+.1f%%)  paired diff %+.5f (SEM %.5f)  |bias| ratio %.1fx\n', ...
        s1h(1), 100*s1h(1)/A_TROUGH, s4h(1), 100*s4h(1)/A_TROUGH, mean(dh), std(dh)/sqrt(nS), abs(s1h(1))/abs(s4h(1)));
    fprintf('  osc   Euler %+.5f (%+.1f%%)  final %+.5f (%+.1f%%)\n', s1o(1), 100*s1o(1)/A_TROUGH, s4o(1), 100*s4o(1)/A_TROUGH);
    fprintf('  hold spread across seeds: Euler %.5f  final %.5f  (ratio %.2f)\n', s1h(3), s4h(3), s4h(3)/s1h(3));
    COL_SEED = [0.55 0.74 0.96]; C1 = [0 0.2 0.9]; C4 = [0.47 0.67 0.19]; FS = 15;
    f = figure('Units','inches','Position',[0 0 13 11], 'Color','w', 'Visible','off');
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    YL1 = [-0.012 0.016];
    TT = {'1  Euler (tangent predict)', '4  exact + slope at estimated height + pred\_mean2'};  EE = {E1, E4};  CC = {C1, C4};
    for a = 1:2
        nexttile(a); hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off');
        hs = plot(t, EE{a}, '-', 'Color', COL_SEED, 'LineWidth', 0.5);
        hm = plot(t, mean(EE{a},2), '-', 'Color', CC{a}, 'LineWidth', 2.0);
        legend([hs(1) hm], {sprintf('each seed (%d)', nS), 'seed mean'}, 'Location','northoutside','Orientation','horizontal','FontSize',11,'FontWeight','bold','Box','on');
        title(TT{a}, 'FontSize', FS, 'FontWeight','bold'); xlim([0 5]); ylim(YL1);
        if a == 1; ylabel('(\^a_z - a_z) / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); end
        set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
    end
    nexttile(3, [1 2]); hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off');
    hd = plot(t, mean(E1 - E4, 2), '-', 'Color', C1, 'LineWidth', 2.0);
    legend(hd, {'Euler - final, paired seed mean  (= what the three layers removed)'}, 'Location','northoutside','Orientation','horizontal','FontSize',11,'FontWeight','bold','Box','on');
    ylabel('difference / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); xlim([0 5]);
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
    nexttile(5, [1 2]); hold on;
    h1 = plot(t, std(E1,0,2), '-', 'Color', C1, 'LineWidth', 2.0);
    h4 = plot(t, std(E4,0,2), '-', 'Color', C4, 'LineWidth', 2.0);
    legend([h1 h4], {'spread across seeds, Euler', 'spread across seeds, final'}, 'Location','northoutside','Orientation','horizontal','FontSize',11,'FontWeight','bold','Box','on');
    ylabel('sd((\^a_z - a_z)/a_{nom})', 'FontSize', FS, 'FontWeight','bold'); xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold'); xlim([0 5]);
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
    png = fullfile(od, 'aptrue_euler_vs_final.png');
    exportgraphics(f, png, 'Resolution', 150); close(f);
    fprintf('  wrote %s\n', png);
end
