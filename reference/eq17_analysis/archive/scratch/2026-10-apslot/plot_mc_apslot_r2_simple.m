% STATUS: SCRATCH -- 2026-10-18 (user: one simple figure for raw vs whitened y2): a' as a state, constant-velocity descent, plane, seed 1.
%   Columns: Q on the a' slot off | on. Rows: a-bar_z (red true, grey raw y2, blue whitened y2); e_a of the two.
%   Data: run_mc_apslot_r2 arms raw_q0 / white_q0 / raw / white. Output test_results/temp_1009_ws/fig_mc_apslot_r2_simple.png.
function plot_mc_apslot_r2_simple(seed)
    if nargin < 1 || isempty(seed); seed = 1; end
    od = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'test_results', 'temp_1009_ws');
    pairs = {{'raw_q0', 'white_q0'}, {'raw', 'white'}};  cols = {'q on a'' = 0', 'q on a'' on'};
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cR = [0.55 0.55 0.55];  FS = 20;  LFS = 15;  AB = char(257);  HAT = char(770);
    f = figure('Position', [40 40 1800 950], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        A = load(fullfile(od, sprintf('apr2_%s_plane_desc.mat', pairs{ic}{1})));  B = load(fullfile(od, sprintf('apr2_%s_plane_desc.mat', pairs{ic}{2})));
        t = A.t;  q = seed;
        ax = nexttile(tl, ic);  hold on; box on;
        plot(ax, t, B.a_true(:, q), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_z true']);
        plot(ax, t, A.a_hat(:, q), '-', 'Color', cR, 'LineWidth', 1.6, 'DisplayName', [AB HAT '_z raw y_2']);
        plot(ax, t, B.a_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [AB HAT '_z whitened y_2']);
        ylim(ax, [0 1.05]);  text(ax, 0.03, 0.08, cols{ic}, 'Units', 'normalized', 'FontSize', LFS + 2, 'FontWeight', 'bold');
        legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax2 = nexttile(tl, 2 + ic);  hold on; box on;
        plot(ax2, t, A.a_true(:, q) - A.a_hat(:, q), '-', 'Color', cR, 'LineWidth', 1.6);
        plot(ax2, t, B.a_true(:, q) - B.a_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.6);
        yline(ax2, 0, ':', 'Color', [0.3 0.3 0.3]);  ylim(ax2, [-0.8 0.2]);  xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
        if ic == 1; ylabel(ax, [AB '_z'], 'FontSize', FS, 'FontWeight', 'bold');  ylabel(ax2, ['e_{' AB '}'], 'FontSize', FS, 'FontWeight', 'bold'); end
        for a = [ax ax2]; xlim(a, [0 t(end)]);  set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(a, 'off'); end
    end
    out = fullfile(od, 'fig_mc_apslot_r2_simple.png');  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end
