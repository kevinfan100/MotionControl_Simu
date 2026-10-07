% PURPOSE (2026-10-02, user: "make the motion-variance difference obvious, drop the useless signals and the bottom row"):
%   same runs as plot_5state_noap_am_check_eps.m (plane, T_d = 5 s, 10 seeds, epsilon = 0 vs 1e-5), one column, two rows:
%   (1) size of the random motion: running sd of dx_r (0.1 s window), 10-seed mean, both arms, against the sd the readout
%       expects at a_true (red dashed) | (2) why: a_true vs a_hat of both arms.   Reads amchk_plane_td5_<arm>_s*.mat.
%   STATUS: SCRATCH
function plot_5state_noap_am_check_eps2(seeds)
    if nargin < 1; seeds = 1:10; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;  kBT = pc.k_B * pc.T;  NW = round(0.1 / pc.Ts);  WIN = [4.5 6.5];
    ARMS = {'q0', '\epsilon = 0', [0.4 0.4 0.4]; 'eps', '\epsilon = Q_{44}/R_2 = 10^{-5}', [0 0.2 0.9]};
    FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1300 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax1 = nexttile(1); hold(ax1, 'on'); box(ax1, 'on');  ax2 = nexttile(2); hold(ax2, 'on'); box(ax2, 'on');
    h0 = plot(ax1, NaN, NaN, 'w.', 'DisplayName', sprintf('plane, 22.2 -> 1.10 R in T_d = 5 s, stop at 5.5 s, %d seeds', numel(seeds)));
    hl1 = h0;  hl2 = gobjects(0);
    for m = 1:2
        SD = [];  EX = [];  AH = [];  AT = [];
        for q = 1:numel(seeds)
            r = load(fullfile(id, sprintf('amchk_plane_td5_%s_s%d.mat', ARMS{m, 1}, seeds(q))));
            SD(:, q) = movstd(r.dx_r, NW) * 1e3;  EX(:, q) = sqrt(r.C_dpmr * 4 * kBT * r.a_true + r.C_n * r.s2n) * 1e3; %#ok<AGROW>
            AH(:, q) = r.a_hat / a_nom;  AT(:, q) = r.a_true / a_nom; %#ok<AGROW>
        end
        t = r.t;  k = t >= WIN(1) & t <= WIN(2);
        hl1(end + 1) = plot(ax1, t(k), mean(SD(k, :), 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 2.4, 'DisplayName', ['sd of the random motion, ' ARMS{m, 2}]); %#ok<AGROW>
        if m == 1
            hl1(end + 1) = plot(ax1, t(k), mean(EX(k, :), 2), '--', 'Color', [0.8 0 0], 'LineWidth', 2.4, 'DisplayName', 'sd it should have at a_{true}'); %#ok<AGROW>
            hl2(end + 1) = plot(ax2, t(k), mean(AT(k, :), 2), '-', 'Color', [0.8 0 0], 'LineWidth', 2.4, 'DisplayName', 'a_{true}'); %#ok<AGROW>
        end
        hl2(end + 1) = plot(ax2, t(k), mean(AH(k, :), 2), '-', 'Color', ARMS{m, 3}, 'LineWidth', 2.2, 'DisplayName', ['a_{hat}, ' ARMS{m, 2}]); %#ok<AGROW>
        h = t > 6.0;
        fprintf('%-4s | hold t > 6 s: sd %.2f nm vs expected %.2f nm (ratio %.2f) | a_hat/a_true %.2f\n', ARMS{m, 1}, ...
            mean(SD(h, :), 'all'), mean(EX(h, :), 'all'), mean(SD(h, :), 'all') / mean(EX(h, :), 'all'), mean(AH(h, :), 'all') / mean(AT(h, :), 'all'));
    end
    ylim(ax1, [0 30]);  ylim(ax2, [0 1]);
    ylabel(ax1, 'random motion sd [nm]', 'FontSize', FS, 'FontWeight', 'bold');
    ylabel(ax2, 'a / a_{nom}', 'FontSize', FS, 'FontWeight', 'bold');  xlabel(ax2, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    for ax = [ax1 ax2]
        xlim(ax, WIN);  yl = ylim(ax);  plot(ax, [5.5 5.5], yl, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'HandleVisibility', 'off');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid(ax, 'off');
    end
    legend(ax1, hl1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 2, 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    legend(ax2, hl2, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
    out = fullfile(fd, 'fig_noap_am_check_eps_td5_v2.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
