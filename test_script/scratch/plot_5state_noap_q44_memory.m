% PURPOSE (2026-09-30, user: "we are tuning the Q/R ratio -- how to read it"): every Q44 design is a rule for
%   rho_k = Q44/R2, i.e. for the memory N_k ~ 1/l42. One plot, all arms on the same run (plane, T_d = 5 s, 10 seeds):
%   row 1: the filter's own y2 memory T_s / (a_cov K(4,2)) [s], log scale | row 2: e_a = a_true - a_hat (mean over seeds).
%   Reads the cached runs of plot_5state_noap_q44_walls.m (q44w_plane_<arm>_s*.mat).   STATUS: SCRATCH
function plot_5state_noap_q44_memory()
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;  Ts = pc.Ts;  A_COV = 0.05;  T_d = 5;
    arms = {'r0',      'Q_{44} = 0',                         [0.4 0.4 0.4];
            'r1e-06',  'fixed \rho = 10^{-6}',                [0.55 0.75 1.0];
            'r1e-05',  'fixed \rho = 10^{-5}',                [0.30 0.50 0.95];
            'r0p0001', 'fixed \rho = 10^{-4}',                [0.05 0.15 0.60];
            'ratr0p1', 'Q/R ratio, \surdP_0 = 0.1 a_{hat}',   [0.10 0.60 0.10];
            'opt1',    'best memory (reads a''_{true})',      [0.85 0.45 0]};
    FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1300 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax1 = nexttile(1); hold(ax1, 'on'); box(ax1, 'on');  ax2 = nexttile(2); hold(ax2, 'on'); box(ax2, 'on');
    hh = gobjects(1, size(arms, 1));
    for i = 1:size(arms, 1)
        M = [];  E = [];
        for s = 1:10
            r = load(fullfile(id, sprintf('q44w_plane_%s_s%d.mat', arms{i, 1}, s)));
            if isfield(r, 'crashed') && r.crashed; continue; end
            t = r.t - 0.5;  M(:, end + 1) = A_COV * r.l42;  E(:, end + 1) = (r.a_true - r.a_hat) / a_nom; %#ok<AGROW>
        end
        mem = Ts ./ mean(M, 2);
        hh(i) = plot(ax1, t, mem, '-', 'Color', arms{i, 3}, 'LineWidth', 2.0, 'DisplayName', arms{i, 2});
        plot(ax2, t, mean(E, 2), '-', 'Color', arms{i, 3}, 'LineWidth', 1.8);
        fprintf('%-10s memory: far (0.5-2 s) %.3f s, wall (4.8-5 s) %.3f s | e_a stop %+.3f\n', arms{i, 1}, ...
            mean(mem(t >= 0.5 & t < 2)), mean(mem(t >= 4.8 & t <= 5)), mean(E(find(t <= T_d, 1, 'last'), :)));
    end
    set(ax1, 'YScale', 'log');  ylim(ax1, [0.01 20]);  ylim(ax2, [-0.4 0.1]);
    for ax = [ax1 ax2]
        xlim(ax, [-0.5 7]);  yl = ylim(ax);
        for x = [0 T_d]; plot(ax, [x x], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off'); end
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid(ax, 'off');
    end
    yline(ax2, 0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
    ylabel(ax1, 'memory T_s / l_{42}  [s]', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax2, 'e_a = a_{true} - a_{hat}', 'FontSize', FS, 'FontWeight', 'bold');
    xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    h0 = plot(ax1, NaN, NaN, 'w.', 'DisplayName', 'plane, plane init, 22.2 -> 1.10 R in 5 s, 10 seeds');
    legend(ax1, [h0 hh], 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    out = fullfile(fd, 'fig_noap_q44_memory_plane.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
