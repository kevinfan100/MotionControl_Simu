% STATUS: SCRATCH -- 2026-10-12, Fig. 2.4 of 1009_conly_wall_position.tex: which window explains the simulated result (descent, seed 1).
%   Columns plane | cell. Top: the filter's w_s-hat (run_1009_ws 'desc'); dashed line = the first time the window O(0, k) pins v6 better
%   than the prior (1/sigma_6 < prior std along v6). Bottom: sqrt(P) of w_s: the filter's own, the bound from prior + O(0, k) (no Q),
%   and the bound from prior + a 3-step window O(k0, k0+2) (= the prior 0.64: one window adds <= sigma_5^2 ~ 1e-5 of information
%   against the prior's 2.4). Data: test_results/temp_1009_ws/svdcum_<wall>.mat (probe_1009_ws_svd_cum.m), desc_ws_<wall>.mat.
function plot_1009_ws_which_window()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    WL = {'plane', 'cell'};
    cE = [0 0.25 0.85];  cG = [0.55 0.55 0.55];  FS = 22;  LFS = 17;  AXLW = 2.2;  WB = ['w' char(772)];
    f = figure('Position', [40 40 1600 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        C = load(fullfile(od, sprintf('svdcum_%s.mat', WL{ic})));  E = load(fullfile(od, sprintf('desc_ws_%s.mat', WL{ic})));
        pv = sqrt(C.v6(:, 5).^2 * C.sP0(5)^2 + C.v6(:, 6).^2 * C.sP0(6)^2);   % prior std along v6
        q = find(1 ./ C.sig(:, 6) < pv, 1);  tc = C.t(q);
        fprintf('%s: O(0,k) beats the prior along v6 from t = %.3f s (h_d %.2f R)\n', WL{ic}, tc, C.hd(q));
        ax1 = nexttile(tl, ic);  hold on; box on;
        h1 = plot(ax1, E.t, E.w_hat(:, 1), '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [WB char(770) '_s']);
        h2 = xline(ax1, tc, '--', 'Color', 'k', 'LineWidth', 1.8, 'DisplayName', 'O(0,k) beats prior');
        ylim(ax1, [-1.6 1.0]);
        legend(ax1, [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax2 = nexttile(tl, 2 + ic);  hold on; box on;
        h3 = plot(ax2, C.t, C.flt_w, '-', 'Color', cE, 'LineWidth', 2.5, 'DisplayName', 'filter');
        h4 = plot(ax2, C.t, C.bnd_w, '-', 'Color', 'k', 'LineWidth', 2.5, 'DisplayName', 'O(0,k)');
        h5 = yline(ax2, C.sP0(6), '--', 'Color', cG, 'LineWidth', 2.5, 'DisplayName', 'O(k_0,k_0+2)');
        xline(ax2, tc, '--', 'Color', 'k', 'LineWidth', 1.8, 'HandleVisibility', 'off');
        set(ax2, 'YScale', 'log');  ylim(ax2, [0.008 1]);  yticks(ax2, [0.01 0.03 0.1 0.3 0.64]);
        legend(ax2, [h3 h4 h5], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for a = [ax1 ax2]; xlim(a, [0 1.2]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a, 'off'); end
        if ic == 1
            ylabel(ax1, [WB '_s  [R]'], 'FontSize', FS, 'FontWeight', 'bold');
            ylabel(ax2, ['\surd P  of  ' WB '_s'], 'FontSize', FS, 'FontWeight', 'bold');
        end
    end
    out = fullfile(fd, 'fig_1009_ws_which_window_s1.png');
    exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
