% STATUS: SCRATCH -- 2026-10-11, figure for probe_1009_ws_obs_window.m (descent 22.2 -> 1.10 R in 1 s, c and w_s, seed 1).
%   Columns plane | cell. Rows: (1) h_d [R], log axis; (2) the two singular values of the prior-scaled (c, w_s) block of a fixed
%   window of N steps (others unknown), line at 1 = the prior; (3) weak direction dc/dw_s of that window (blue) and the one-height
%   alias 2(1 - c-hat)/(h_d - w_s-hat) (red). Window values are drawn at the window centre.
function plot_1009_ws_obs_window()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_1009_ws');
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    WL = {'plane', 'cell'};
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cL = [0.45 0.7 1];  FS = 22;  LFS = 17;  AXLW = 2.2;  T_END = 1.2;
    WB = ['w' char(772)];
    f = figure('Position', [40 40 1600 1500], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        r = load(fullfile(od, sprintf('obswin_%s.mat', WL{ic})));  ax = gobjects(3, 1);
        ax(1) = nexttile(tl, ic);  hold on; box on;
        plot(ax(1), r.t_full, r.hd_full, '-', 'Color', 'k', 'LineWidth', 2.5);
        set(ax(1), 'YScale', 'log', 'YTick', [1.1 2 5 10 22.2]);  ylim(ax(1), [1 25]);
        ax(2) = nexttile(tl, 2 + ic);  hold on; box on;
        h1 = plot(ax(2), r.t, r.sig(:, 1), '-', 'Color', cE, 'LineWidth', 2.5, 'DisplayName', [char(963) '_1']);
        h2 = plot(ax(2), r.t, r.sig(:, 2), '-', 'Color', cL, 'LineWidth', 2.5, 'DisplayName', [char(963) '_2']);
        h3 = yline(ax(2), 1, '--', 'Color', 'k', 'LineWidth', 1.8, 'DisplayName', 'prior');
        set(ax(2), 'YScale', 'log', 'YTick', 10.^(-4:2:2));  ylim(ax(2), [1e-4 1e3]);
        legend(ax(2), [h1 h2 h3], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax(3) = nexttile(tl, 4 + ic);  hold on; box on;
        h4 = plot(ax(3), r.t, r.alias, '-', 'Color', cT, 'LineWidth', 5.0, 'DisplayName', 'one height');
        h5 = plot(ax(3), r.t, r.ratio, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'window');
        ylim(ax(3), [0 2]);
        legend(ax(3), [h4 h5], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        xlabel(ax(3), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        for q = 1:3; xlim(ax(q), [0 T_END]);  set(ax(q), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(ax(q), 'off'); end
        if ic == 1
            ylabel(ax(1), 'h_d  [R]', 'FontSize', FS, 'FontWeight', 'bold');
            ylabel(ax(2), [char(963) '  (c, ' WB '_s)'], 'FontSize', FS, 'FontWeight', 'bold');
            ylabel(ax(3), ['dc / d' WB '_s'], 'FontSize', FS, 'FontWeight', 'bold');
        end
    end
    out = fullfile(fd, sprintf('fig_1009_ws_obs_desc_N%d_s1.png', r.N));
    exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
