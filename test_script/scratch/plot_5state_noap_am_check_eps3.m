% PURPOSE (2026-10-02, user: "show the motion signal itself"): same runs as plot_5state_noap_am_check_eps.m (plane, T_d = 5 s,
%   stop at 5.5 s). Rows epsilon = 0 | 1e-5, same y axis. Left: dx_r of seed 1 (the random part of the motion error) with
%   +- the sd it should have at a_true (red dashed). Right: histogram of dx_r in the final hold (t > 6 s, 10 seeds pooled)
%   against the Gaussian of that sd (red).   Reads amchk_plane_td5_<arm>_s*.mat.   STATUS: SCRATCH
function plot_5state_noap_am_check_eps3(seeds)
    if nargin < 1; seeds = 1:10; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    addpath(genpath(fullfile(root, 'model')));
    id = fullfile(root, 'test_results', 'temp_5state_noap');  fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  kBT = pc.k_B * pc.T;  WIN = [5.0 6.5];  YL = [-45 45];
    ARMS = {'q0', '\epsilon = 0', [0.4 0.4 0.4]; 'eps', '\epsilon = Q_{44}/R_2 = 10^{-5}', [0 0.2 0.9]};
    FS = 18;  LFS = 12;  AXLW = 2.0;
    f = figure('Position', [40 40 1400 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    edges = linspace(YL(1), YL(2), 46);  yc = 0.5 * (edges(1:end - 1) + edges(2:end));
    for m = 1:2
        pool = [];  exh = [];
        for q = 1:numel(seeds)
            r = load(fullfile(id, sprintf('amchk_plane_td5_%s_s%d.mat', ARMS{m, 1}, seeds(q))));
            ex = sqrt(r.C_dpmr * 4 * kBT * r.a_true + r.C_n * r.s2n) * 1e3;  h = r.t > 6.0;
            pool = [pool; r.dx_r(h) * 1e3]; exh = [exh; ex(h)]; %#ok<AGROW>
            if q == 1; r1 = r; ex1 = ex; end
        end
        t = r1.t;  k = t >= WIN(1) & t <= WIN(2);
        ax = nexttile(tl, m); hold(ax, 'on'); box(ax, 'on');
        h1 = plot(ax, t(k), r1.dx_r(k) * 1e3, '-', 'Color', ARMS{m, 3}, 'LineWidth', 0.8, 'DisplayName', ['random motion dx_r (seed 1), ' ARMS{m, 2}]);
        plot(ax, [5.5 5.5], YL, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);
        xlim(ax, WIN);  ylim(ax, YL);  ylabel(ax, 'dx_r [nm]', 'FontSize', FS, 'FontWeight', 'bold');
        if m == 2; xlabel(ax, 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold'); end
        legend(ax, h1, 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold', 'Box', 'on');
        set(ax, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW, 'Box', 'on');  grid(ax, 'off');
        s0 = mean(exh);
        fprintf('%-4s | final hold: sd of dx_r %.2f nm, expected %.2f nm, ratio %.2f | share beyond +-2 expected sd: %.3f (Gaussian 0.046)\n', ...
            ARMS{m, 1}, std(pool), s0, std(pool) / s0, mean(abs(pool) > 2 * s0));
    end
    out = fullfile(fd, 'fig_noap_am_check_eps_td5_v3.png');  exportgraphics(f, out, 'Resolution', 150);  fprintf('figure: %s\n', out);
end
