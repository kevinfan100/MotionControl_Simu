% PURPOSE (2026-10-02, user: "only these two subplots"): T_d = 5 s column of fig_noap_l41_l42_plane, rows l42 | |l41|,
%   epsilon = 0 (grey) vs 1e-5 (blue), plane, 10 seeds. Reads l4142_plane_ramp5_{q0,eps}_s*.mat.   STATUS: SCRATCH
function plot_5state_noap_l41_l42_td5(seeds)
    if nargin < 1; seeds = 1:10; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;  A_COV = 0.05;  T_d = 5;
    ARMS = {'q0', '\epsilon = 0', [0.4 0.4 0.4]; 'eps', '\epsilon = Q_{44}/R_2 = 10^{-5}', [0 0.2 0.9]};
    FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1300 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax1 = nexttile(1); hold(ax1, 'on'); box(ax1, 'on');  ax2 = nexttile(2); hold(ax2, 'on'); box(ax2, 'on');
    hl = plot(ax1, NaN, NaN, 'w.', 'DisplayName', sprintf('plane, 22.2 -> 1.10 R in T_d = %g s, %d seeds', T_d, numel(seeds)));
    for m = 1:2
        L2 = [];  L1 = [];
        for q = 1:numel(seeds)
            r = load(fullfile(id, sprintf('l4142_plane_ramp5_%s_s%d.mat', ARMS{m, 1}, seeds(q))));
            L2(:, q) = A_COV * r.K42;  L1(:, q) = abs(r.K41) / a_nom * 1e-3; %#ok<AGROW>
        end
        t = r.t - 0.5;
        hl(end + 1) = plot(ax1, t, mean(L2, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 2.0, 'DisplayName', ARMS{m, 2}); %#ok<AGROW>
        plot(ax2, t, mean(L1, 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 1.2);
    end
    set(ax1, 'YScale', 'log');  set(ax2, 'YScale', 'log');  ylim(ax1, [3e-6 1e-2]);  ylim(ax2, [1e-6 3e-3]);
    ylabel(ax1, 'l_{42} (share of one y_2 reading)', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax2, '|l_{41}|  [a/a_{nom} per nm]', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    for ax = [ax1 ax2]
        xlim(ax, [t(1) t(end)]);  yl = ylim(ax);
        for x = [0 T_d]; plot(ax, [x x], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid(ax, 'off');
    end
    legend(ax1, hl, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 3, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    out = fullfile(fd, 'fig_noap_l41_l42_td5_plane.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
