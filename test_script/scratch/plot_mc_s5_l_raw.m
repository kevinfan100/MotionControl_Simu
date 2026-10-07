% STATUS: SCRATCH -- 2026-10-07, Fig. 5.2 companion (user: why a 50 ms mean on l? what does the raw look like?). Same runs as
%   plot_mc_s5_split.m (Fig. 5.1, seed 1). Row 1: raw |l41|, |l42| every step, log axis, full 5 s. Row 2: signed l41 and l42,
%   linear axis, a 25 ms window (40 steps) at 2.0 s, markers on every step. Columns: first order | c-only.
function plot_mc_s5_l_raw(seed)
    if nargin < 1; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
    fd = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
    pc = physical_constants();  a_nom = pc.Ts / pc.gamma_N;
    R1 = run_mc_s4_first_order({'prod_xyz'}, seed, {'plane'});  r = R1.prod_xyz.plane(seed);
    S = load(fullfile(root, 'test_results', 'temp_plaw_walls', 'f1p2_plane_PI_mcs5_desc_ramp5_lc040.mat'));
    L = {[r.K_a_y1 * pc.R, r.K_a_y2 * a_nom], [S.K_a_y1(:, seed), S.K_a_y2(:, seed)]};  T = {r.t, S.t};
    cE = [0 0.25 0.85];  cM = [0.45 0.75 1.0];  FS = 20;  LFS = 15;  AXLW = 2.0;  EL = char(8467);  NM = {'first order', 'c-only'};
    f = figure('Position', [40 40 1600 1000], 'Color', 'w', 'NumberTitle', 'off', 'Visible', 'off');
    tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for ic = 1:2
        l = L{ic};  t = T{ic};
        ax = nexttile(tl, ic);  hold on; box on;
        h1 = plot(ax, t, abs(l(:, 1)), '-', 'Color', cE, 'LineWidth', 0.6, 'DisplayName', ['|' EL '_{41}| raw']);
        h2 = plot(ax, t, abs(l(:, 2)), '-', 'Color', cM, 'LineWidth', 0.6, 'DisplayName', ['|' EL '_{42}| raw']);
        set(ax, 'YScale', 'log');  ylim(ax, [1e-6 3]);  xlim(ax, [0 t(end)]);  title(ax, NM{ic}, 'FontSize', FS - 2);
        legend(ax, [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        ax2 = nexttile(tl, 2 + ic);  hold on; box on;  w = t >= 2.0 & t < 2.025;
        g1 = plot(ax2, t(w), l(w, 1), '-o', 'Color', cE, 'LineWidth', 1.5, 'MarkerSize', 5, 'DisplayName', [EL '_{41}']);
        g2 = plot(ax2, t(w), l(w, 2), '-s', 'Color', cM, 'LineWidth', 1.5, 'MarkerSize', 5, 'DisplayName', [EL '_{42}']);
        yline(ax2, 0, '-', 'Color', [0.6 0.6 0.6]);  xlabel(ax2, 'Time (sec)');
        legend(ax2, [g1 g2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
        for a = [ax ax2]; set(a, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', AXLW);  grid(a, 'off'); end
        m = t > 1 & t < 3;  s1 = sign(l(m, 1));
        fprintf('%-11s 1-3 s: l41 mean %+.3g, mean |.| %.3g, sign changes per step %.2f | l42 mean %+.3g, mean |.| %.3g\n', NM{ic}, ...
                mean(l(m, 1)), mean(abs(l(m, 1))), mean(abs(diff(s1)) > 0), mean(l(m, 2)), mean(abs(l(m, 2))));
    end
    out = fullfile(fd, sprintf('fig_mc_s5_l_raw_s%d.png', seed));  exportgraphics(f, out, 'Resolution', 130);  fprintf('figure: %s\n', out);
end
