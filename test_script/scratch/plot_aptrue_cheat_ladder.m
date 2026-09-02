% STATUS: ACTIVE (scratch) | PURPOSE: the four-arm cheat ladder (run_aptrue_cheat_ladder.m)
%   with the ZERO-PARAMETER predictions for arms 3 and 4 overlaid. The predictions
%   come from the exact-arm probe alone (aptrue_ladder_pred.mat): arm 3 = exact arm
%   plus the accumulated deterministic slope-evaluation injection it removes; arm 4 =
%   exact arm plus everything except the orthogonality term T2. Solid = measured
%   seed means; dashed = predicted. Table: hold / osc means, measured vs predicted.
function plot_aptrue_cheat_ladder()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'apd_acov_meng');
    L = load(fullfile(od, 'aptrue_cheat_ladder.mat'));  P = load(fullfile(od, 'aptrue_ladder_pred.mat'));
    t = L.t;  nS = numel(L.seeds);  E = L.E;  ARMS = L.arms;
    tp = P.t;  pred = {[], [], P.arm3, P.arm4};
    m_h = t > 3.5;  m_o = t > 1.5 & t <= 3.5;  mp_h = tp > 3.5;  mp_o = tp > 1.5 & tp <= 3.5;
    fprintf('%-38s %20s %20s\n', 'arm', 'osc  meas / pred', 'hold  meas / pred (SEM)');
    for a = 1:4
        pm = mean(E{a}(m_h,:),1);  po = mean(E{a}(m_o,:),1);
        if isempty(pred{a}); ps = '     --   '; ph = '     --   '; else; ps = sprintf('%+9.5f', mean(pred{a}(mp_o))); ph = sprintf('%+9.5f', mean(pred{a}(mp_h))); end
        fprintf('%-38s %+9.5f /%s %+9.5f /%s (%.5f)\n', ARMS{a}, mean(po), ps, mean(pm), ph, std(pm)/sqrt(nS));
    end
    COL = {[0.55 0.74 0.96], [0 0.2 0.9], [0.85 0.33 0.10], [0.47 0.67 0.19]};  FS = 15;
    f = figure('Units','inches','Position',[0 0 12 6.5], 'Color','w', 'Visible','off');
    hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off');
    H = gobjects(1,6);  LG = cell(1,6);
    for a = 1:4; H(a) = plot(t, mean(E{a},2), '-', 'Color', COL{a}, 'LineWidth', 1.4); LG{a} = ARMS{a}; end
    H(5) = plot(tp, P.arm3, '--', 'Color', [0.5 0.15 0.0], 'LineWidth', 2.0);  LG{5} = 'pred. arm 3 (probe, 0 param)';
    H(6) = plot(tp, P.arm4, '--', 'Color', [0.1 0.4 0.1], 'LineWidth', 2.0);  LG{6} = 'pred. arm 4 (probe, 0 param)';
    legend(H, LG, 'Location','northoutside','Orientation','horizontal','FontSize',10,'FontWeight','bold','Box','on','NumColumns',3);
    ylabel('mean (\^a_z - a_z) / a_{nom}', 'FontSize', FS, 'FontWeight','bold');
    xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold'); xlim([0 5]);
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
    png = fullfile(od, 'aptrue_cheat_ladder.png');
    exportgraphics(f, png, 'Resolution', 150); close(f);
    fprintf('[ladder] wrote %s\n', png);
end
