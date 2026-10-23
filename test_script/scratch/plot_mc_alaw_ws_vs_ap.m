% STATUS: SCRATCH -- 2026-10-19 (user): why the a law works with the w_s state and not with the a' state. Same a law (tau 0.1 s,
%   no speed cap, same slope gate), plane a_d 0.07, seed 1; files gainreg_plane_tau100_T0.5_alWS.mat (w_s) and
%   gainreg_plane_tau100_T0.00875_alAP.mat (a' state, up to one step before contact at ~11 ms). First 12 ms, one marker = one step.
%   Top: command h_d (red) and particle h (dashed), w_s | a' state, with the wall and the contact. Bottom: slope a-bar' handed to the law (solid) and the true slope at the particle (dashed), log.
%   Output test_results/temp_1009_ws/fig_mc_alaw_ws_vs_ap.png.
function plot_mc_alaw_ws_vs_ap()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(genpath(fullfile(root, 'model')));
    od = fullfile(root, 'test_results', 'temp_1009_ws');  pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;
    cA = [0 0.25 0.85];  cW = [0.95 0.5 0.05];  FS = 20;  LFS = 15;  AXLW = 2.2;  AB = char(257);  HAT = char(770);
    cols = {cW, cA};
    W = load(fullfile(od, 'gainreg_plane_tau100_T0.5_alWS.mat'));  A = load(fullfile(od, 'gainreg_plane_tau100_T0.00875_alAP.mat'));
    kW = 2:min(numel(W.t), 20);  kA = 2:numel(A.t);
    f = figure('Position', [40 40 1500 1600], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    cT = [0.85 0 0];  tc = 1000 * (A.t(end) + pc.Ts);  axt = gobjects(1, 2);
    for ic = 1:2                                              % 10-19 (user): command vs particle, one panel per estimator
        if ic == 1; S = W;  k = kW;  nm = ['w' char(772) '_s state']; else; S = A;  k = kA;  nm = [AB ''' state']; end
        axt(ic) = nexttile(tl, ic);  hold on; box on;
        h1 = plot(axt(ic), 1000 * S.t(k), S.wd(k) * pc.R, 'o-', 'Color', cT, 'LineWidth', 3.5, 'MarkerSize', 6, 'MarkerFaceColor', cT, 'DisplayName', 'z_d');
        h2 = plot(axt(ic), 1000 * S.t(k), S.h_true(k) * pc.R, 'o--', 'Color', cols{ic}, 'LineWidth', 2.0, 'MarkerSize', 6, 'MarkerFaceColor', cols{ic}, 'DisplayName', 'z');
        hw = yline(axt(ic), pc.R, '-', 'Color', 'k', 'LineWidth', 2.5, 'DisplayName', 'wall');
        hs = yline(axt(ic), 1.078 * pc.R, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5, 'DisplayName', 'z^*');
        hh = [h1 h2 hw hs];
        if ic == 2
            plot(axt(ic), [1000 * A.t(end) tc], [A.h_true(end) * pc.R, pc.R], ':', 'Color', cA, 'LineWidth', 2.2, 'HandleVisibility', 'off');
            hx = plot(axt(ic), tc, pc.R, 'x', 'Color', cA, 'MarkerSize', 22, 'LineWidth', 4, 'DisplayName', sprintf('contact with the wall (%.1f ms)', tc));  hh = [hh hx];
        end
        xlim(axt(ic), [0 12]);  ylim(axt(ic), [0 55]);
        lg = legend(axt(ic), hh, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'NumColumns', 3);  lg.Title.String = sprintf('plane wall, %s, %s_d = 0.07', nm, AB);
        if ic == 1; ylabel(axt(ic), ['z  [' char(181) 'm]'], 'FontSize', FS, 'FontWeight', 'bold'); end
    end
    axm = nexttile(tl, 3, [1 2]);  hold on; box on;                   % 10-19: the gain itself (the force is the wanted step / a_hat)
    plot(axm, 1000 * W.t(kW), W.a_hat(kW), 'o-', 'Color', cW, 'LineWidth', 2.2, 'MarkerSize', 7, 'MarkerFaceColor', cW, 'DisplayName', [AB HAT ', w' char(772) '_s state']);
    plot(axm, 1000 * W.t(kW), W.a_true(kW), '--', 'Color', cW, 'LineWidth', 2.0, 'DisplayName', [AB ', w' char(772) '_s state']);
    plot(axm, 1000 * A.t(kA), A.a_hat(kA), 'o-', 'Color', cA, 'LineWidth', 2.2, 'MarkerSize', 7, 'MarkerFaceColor', cA, 'DisplayName', [AB HAT ', ' AB ''' state']);
    plot(axm, 1000 * A.t(kA), A.a_true(kA), '--', 'Color', cA, 'LineWidth', 2.0, 'DisplayName', [AB ', ' AB ''' state']);
    xlim(axm, [0 12]);  ylim(axm, [0 1.1]);  ylabel(axm, AB, 'FontSize', FS + 2, 'FontWeight', 'bold');
    legend(axm, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'NumColumns', 2);
    ax2 = nexttile(tl, 5, [1 2]);  hold on; box on;
    y = W.ap_hat(kW) / a_nom;  plot(ax2, 1000 * W.t(kW), y, 'o-', 'Color', cW, 'LineWidth', 2.2, 'MarkerSize', 7, 'MarkerFaceColor', cW, 'DisplayName', [AB HAT ''', w' char(772) '_s state']);
    plot(ax2, 1000 * W.t(kW), W.ap_true(kW) / a_nom, '--', 'Color', cW, 'LineWidth', 2.0, 'DisplayName', [AB ''', w' char(772) '_s state']);
    yA = A.ap_hat(kA) / a_nom;  y = yA;  y(y <= 0) = NaN;  plot(ax2, 1000 * A.t(kA), y, 'o-', 'Color', cA, 'LineWidth', 2.2, 'MarkerSize', 7, 'MarkerFaceColor', cA, 'DisplayName', [AB HAT ''', ' AB ''' state']);
    plot(ax2, 1000 * A.t(kA), A.ap_true(kA) / a_nom, '--', 'Color', cA, 'LineWidth', 2.0, 'DisplayName', [AB ''', ' AB ''' state']);
    set(ax2, 'YScale', 'log');  xlim(ax2, [0 12]);  ylim(ax2, [1e-3 20]);
    ylabel(ax2, [AB ''''], 'FontSize', FS + 2, 'FontWeight', 'bold');  xlabel(ax2, 'Time (ms)', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax2, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'NumColumns', 2);
    for a_ = [axt axm ax2]; set(a_, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a_, 'off'); end
    out = fullfile(od, 'fig_mc_alaw_ws_vs_ap.png');  exportgraphics(f, out, 'Resolution', 150);  close(f);  fprintf('figure: %s\n', out);
end
