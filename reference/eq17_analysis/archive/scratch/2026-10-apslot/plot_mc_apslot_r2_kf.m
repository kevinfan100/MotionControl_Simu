% STATUS: SCRATCH -- 2026-10-18 (user: show inside the Kalman filter how raw vs whitened y2 reaches a-hat). a' as a state, q on a' = 0,
%   constant-velocity descent 22.2 -> 1.10 R in 1 s, plane, seed 1 (run_mc_apslot_r2 arms raw_q0 / white_q0). Columns raw | whitened.
%   Row 1: y2 innovation divided by the filter's own sqrt(S2) (the filter assumes this is white with variance 1).
%   Row 2: the y2 correction of a-hat in one step, l42 e_am.
%   Row 3: the same correction summed over the last 50 ms (80 steps) = what y2 moved a-hat by in 50 ms.
%   Row 4: a-bar_z true (red), estimate (blue). Output test_results/temp_1009_ws/fig_mc_apslot_r2_kf.png.
function plot_mc_apslot_r2_kf(seed)
    if nargin < 1 || isempty(seed); seed = 1; end
    N_WIN = 80;                       % 50 ms at Ts = 1/1600 s
    od = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'test_results', 'temp_1009_ws');
    arms = {'raw_q0', 'white_q0'};  names = {'raw y_2', 'whitened y_2'};
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  FS = 18;  LFS = 15;  AB = char(257);  HAT = char(770);  EL = char(8467);
    ylab = {'e_{am} / \surdS_2', [EL '_{42} e_{am}  (per step)'], [EL '_{42} e_{am}  (50 ms sum)'], [AB '_z']};
    ylims = {[-6 6], [-2e-3 2e-3], [-0.06 0.06], [0 1.05]};
    f = figure('Position', [40 40 1900 1500], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        S = load(fullfile(od, sprintf('apr2_%s_plane_desc.mat', arms{ic})));  t = S.t;  q = seed;
        z = S.nu2(:, q) ./ sqrt(S.S2(:, q));  d = S.da_y2(:, q);  d(~isfinite(d)) = 0;  c = movsum(d, [N_WIN - 1, 0]);
        Y = {z, d, c};
        for ir = 1:4
            ax = nexttile(tl, (ir - 1) * 2 + ic);  hold on; box on;
            if ir < 4
                plot(ax, t, Y{ir}, '-', 'Color', cE, 'LineWidth', 1.0);
                yline(ax, 0, ':', 'Color', [0.3 0.3 0.3]);
            else
                plot(ax, t, S.a_true(:, q), '-', 'Color', cT, 'LineWidth', 4, 'DisplayName', [AB '_z true']);
                plot(ax, t, S.a_hat(:, q), '-', 'Color', cE, 'LineWidth', 1.6, 'DisplayName', [AB HAT '_z']);
                legend(ax, 'Location', 'southwest', 'FontSize', LFS, 'FontWeight', 'bold');
                xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
            end
            if ir == 1; text(ax, 0.98, 0.88, sprintf('%s, q on a'' = 0, seed %d', names{ic}, q), 'Units', 'normalized', 'HorizontalAlignment', 'right', 'FontSize', LFS + 1, 'FontWeight', 'bold'); end
            xlim(ax, [0 t(end)]);  ylim(ax, ylims{ir});
            if ic == 1; ylabel(ax, ylab{ir}, 'FontSize', FS - 2, 'FontWeight', 'bold'); end
            set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2);  grid(ax, 'off');
        end
    end
    out = fullfile(od, 'fig_mc_apslot_r2_kf.png');  exportgraphics(f, out, 'Resolution', 120);  fprintf('figure: %s\n', out);
end
