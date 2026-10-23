% STATUS: ACTIVE (scratch) | PURPOSE: show the a'_true (exact step, canon) residual
%   bias as the sum of its second-order pieces. Top: measured exact-arm mean error
%   (est - true) vs the accumulated predict-stage injection measured by the probe
%   (open loop, so it overshoots by the update's pull-back). Bottom: the injection
%   split into the S6 pieces of 0902_formC_aptrue_4state.tex, each accumulated:
%   D1 evaluation-point (slope read at the commanded height), D2 Jensen (noisy
%   step through a concave law), D3 orthogonality violation, and the curvature gap.
%   All evaluated from TRUE logged quantities; nothing fitted. Sign: est - true.
function plot_aptrue_injection_decomp()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(here);  od = fullfile(root, 'test_results', 'apd_acov_meng');
    acc = evalc_silent();
    nS = max(acc.seed);  n = sum(acc.seed == 1);  t = acc.t(acc.seed == 1);
    M = @(v) mean(reshape(v, n, nS), 2);                     % seed mean per time
    C = @(v) -cumsum(M(v));                                  % accumulated, flipped to est - true
    E  = M(acc.Eex) * -1;                                    % measured exact-arm error, est - true
    r  = C(acc.r);  T1 = C(acc.T1);  T2 = C(acc.T2);  T3 = C(acc.T3);  G = C(acc.gap);  Ts = C(acc.Tsum2);
    m = t > 3.5;
    fprintf('[decomp] hold means (est-true): measured E %+.5f | accumulated injection -sum r %+.5f | pieces: D1 %+.5f  D2 %+.5f  D3 %+.5f  gap %+.5f  sum %+.5f\n', ...
        mean(E(m)), mean(r(m)), mean(T1(m)), mean(T3(m)), mean(T2(m)), mean(G(m)), mean(Ts(m)));
    fprintf('[decomp] end: measured %+.5f | -sum r %+.5f | D1 %+.5f  D2 %+.5f  D3 %+.5f  gap %+.5f\n', E(end), r(end), T1(end), T3(end), T2(end), G(end));
    FS = 15; LW = 2;
    f = figure('Units','inches','Position',[0 0 12 9], 'Color','w', 'Visible','off');
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
    nexttile; hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off');
    h1 = plot(t, E, '-', 'Color', [0 0.2 0.9], 'LineWidth', 1.0);
    h2 = plot(t, r, '--', 'Color', [0.8 0 0], 'LineWidth', LW);
    legend([h1 h2], {sprintf('exact step: measured mean (\\^a_z - a_z)/a_{nom}, %d seeds', nS), ...
                     'accumulated predict-stage injection (probe, open loop)'}, ...
           'Location','northoutside','Orientation','horizontal','FontSize',11,'FontWeight','bold','Box','on');
    ylabel('(\^a_z - a_z) / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); xlim([0 5]); ylim([-5e-3 3e-3]);
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
    nexttile; hold on; yline(0,'-','Color',[0.55 0.55 0.55],'HandleVisibility','off');
    p1 = plot(t, T1, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', LW);
    p2 = plot(t, T3, '-', 'Color', [0.00 0.60 0.50], 'LineWidth', LW);
    p3 = plot(t, T2, '-', 'Color', [0.49 0.18 0.56], 'LineWidth', LW);
    p4 = plot(t, G,  '-', 'Color', [0.60 0.60 0.60], 'LineWidth', LW);
    p5 = plot(t, Ts, '--', 'Color', [0.8 0 0], 'LineWidth', LW);
    legend([p1 p2 p3 p4 p5], {'D1: slope read at commanded height', 'D2: Jensen (noisy step, concave law)', ...
                              'D3: orthogonality violation', 'curvature gap', 'sum of pieces'}, ...
           'Location','northoutside','Orientation','horizontal','FontSize',11,'FontWeight','bold','Box','on');
    ylabel('accumulated injection / a_{nom}', 'FontSize', FS, 'FontWeight','bold'); xlabel('time  [s]', 'FontSize', FS, 'FontWeight','bold'); xlim([0 5]);
    set(gca,'FontSize',FS,'FontWeight','bold','LineWidth',1.6,'Box','on'); grid off;
    png = fullfile(od, 'aptrue_injection_decomp.png');
    exportgraphics(f, png, 'Resolution', 150); close(f);
    fprintf('[decomp] wrote %s\n', png);
end
function acc = evalc_silent()
    [~, acc] = evalc('analyze_aptrue_predict_drift();');
end
