% PURPOSE (2026-10-02, user: "at T_d = 5 s, how does L42 differ between epsilon = 0 and 1e-5"): plane, 22.2 -> 1.10 R in
%   5 s, 10 seeds, cached runs q44w_plane_{r0,r1e-05}_s*.mat (they carry the logged K(4,2)).
%   Row 1: l42 per y2 reading = a_cov * K(4,2), log scale, with the steady value sqrt(epsilon) (dashed).
%   Row 2: sqrt(P44) / a_nom (the filter's own uncertainty).   STATUS: SCRATCH
function plot_5state_noap_l42_td5()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;  A_COV = 0.05;  T_d = 5;
    ARMS = {'r0', '\epsilon = 0  (Q_{44} = 0)', [0.4 0.4 0.4], 0; 'r1e-05', '\epsilon = 10^{-5}', [0 0.2 0.9], 1e-5};
    FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1300 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax1 = nexttile(1); hold(ax1, 'on'); box(ax1, 'on');  ax2 = nexttile(2); hold(ax2, 'on'); box(ax2, 'on');
    hl = plot(ax1, NaN, NaN, 'w.', 'DisplayName', 'plane, 22.2 -> 1.10 R in T_d = 5 s, 10 seeds');
    for m = 1:2
        L = [];  S = [];
        for s = 1:10
            r = load(fullfile(id, sprintf('q44w_plane_%s_s%d.mat', ARMS{m, 1}, s)));  t = r.t - 0.5;
            L(:, s) = A_COV * r.l42;  S(:, s) = sqrt(r.P_a) / a_nom; %#ok<AGROW>
        end
        hl(end + 1) = plot(ax1, t, mean(L, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 2.0, 'DisplayName', ['l_{42}, ' ARMS{m, 2}]); %#ok<AGROW>
        plot(ax2, t, mean(S, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 2.0);
        for w = {[-0.4 0], [1 2], [4.5 5], [5.5 7]}
            k = t >= w{1}(1) & t < w{1}(2);
            fprintf('%-7s t %4.1f-%4.1f | l42 %.2e (memory %.3f s) | sqrt(P44) %.4f\n', ARMS{m, 1}, w{1}(1), w{1}(2), mean(L(k, :), 'all'), ...
                pc.Ts / mean(L(k, :), 'all'), mean(S(k, :), 'all'));
        end
    end
    hl(end + 1) = plot(ax1, [-0.5 7], sqrt(1e-5) * [1 1], '--', 'Color', [0 0.2 0.9], 'LineWidth', 1.4, 'DisplayName', '\surd\epsilon = 3.2 \times 10^{-3} (steady, y_2 only)');
    set(ax1, 'YScale', 'log');  set(ax2, 'YScale', 'log');  ylim(ax1, [1e-5 1e-1]);
    for ax = [ax1 ax2]
        xlim(ax, [-0.5 7]);  yl = ylim(ax);
        for x = [0 T_d]; plot(ax, [x x], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid(ax, 'off');
    end
    ylabel(ax1, 'l_{42}  (share of one y_2 reading)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax2, '\surd P_{44} / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    legend(ax1, hl, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    out = fullfile(fd, 'fig_noap_l42_td5_plane.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
