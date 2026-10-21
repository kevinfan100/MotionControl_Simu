% STATUS: SCRATCH -- 2026-10-18, figure for run_t1b_angle_only in the house style of plot_mc_s8_b (seed 1, Unicode, bold, true = thick
%   red, estimate = blue, band = +- sqrt P, no grid / title). Rows: z_d, z [um] | theta-hat_y [deg] (red = wall 20 deg) |
%   theta-hat_x [deg] (red = 0) | a_Nm / a_N and a_Tm / a_T (u direction), mean over ALL seeds and 0.05 s blocks (red = 1).
%   -> test_results/temp_3d_wsonly/fig_t1b_angle_only_s1.png | run: plot_t1b_angle_only(seed)
function plot_t1b_angle_only(seed)
    if nargin < 1; seed = 1; end
    here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
    od = fullfile(root, 'test_results', 'temp_3d_wsonly');
    S = load(fullfile(od, 'temp_t1b_angle_only.mat'));  runs = [S.R.runs{:}];  r = runs([runs.seed] == seed);  th0 = S.R.opts.theta_y;
    cT = [0.85 0 0];  cE = [0 0.25 0.85];  cB = [0.75 0.85 1];  cG = [0 0.55 0.2];  FS = 22;  LFS = 17;  MU = char(181);
    AB = char(257);  HAT = char(770);  TH = char(952);  DG = char(176);
    t = r.t;  Ts = t(2) - t(1);  nb = round(0.05 / Ts);  nB = floor(numel(t) / nb);
    tb = mean(reshape(t(1:nB * nb), nb, nB), 1)';
    AN = [runs.aNm];  AT = [runs.aTu];                       % bottom row: all seeds, 0.05 s blocks (single seed is too noisy, sd ~0.25)
    bN = mean(reshape(AN(1:nB * nb, :), nb, nB, []), [1 3]);  bN = bN(:);
    bT = mean(reshape(AT(1:nB * nb, :), nb, nB, []), [1 3]);  bT = bT(:);
    f = figure('Position', [40 40 1500 2000], 'Color', 'w', 'Visible', 'off');
    tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax = gobjects(4, 1);
    ax(1) = nexttile(tl);  hold on;  box on;
    h1 = plot(ax(1), t, r.zd, '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', 'z_d');
    h2 = plot(ax(1), t, r.z, '--', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', 'z');
    ylim(ax(1), [0 50]);  legend(ax(1), [h1 h2], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    ylabel(ax(1), ['z  [' MU 'm]'], 'FontSize', FS, 'FontWeight', 'bold');
    ax(2) = nexttile(tl);  hold on;  box on;
    fill(ax(2), [t; flipud(t)], [r.thy + r.sPy; flipud(r.thy - r.sPy)], cB, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    h3 = plot(ax(2), t, th0 + 0 * t, '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', [TH '_y']);
    h4 = plot(ax(2), t, r.thy, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [TH HAT '_y']);
    ylim(ax(2), [-20 60]);  legend(ax(2), [h3 h4], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    ylabel(ax(2), [TH '_y  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold');
    ax(3) = nexttile(tl);  hold on;  box on;
    fill(ax(3), [t; flipud(t)], [r.thx + r.sPx; flipud(r.thx - r.sPx)], cB, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    h5 = plot(ax(3), t, 0 * t, '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', [TH '_x']);
    h6 = plot(ax(3), t, r.thx, '-', 'Color', cE, 'LineWidth', 1.8, 'DisplayName', [TH HAT '_x']);
    ylim(ax(3), [-40 40]);  legend(ax(3), [h5 h6], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    ylabel(ax(3), [TH '_x  [' DG ']'], 'FontSize', FS, 'FontWeight', 'bold');
    ax(4) = nexttile(tl);  hold on;  box on;
    h7 = plot(ax(4), t, 1 + 0 * t, '-', 'Color', cT, 'LineWidth', 5, 'DisplayName', 'true');
    h8 = plot(ax(4), tb, bN, 'o', 'Color', cE, 'MarkerFaceColor', cE, 'MarkerSize', 5, 'DisplayName', [AB '_{Nm} / ' AB '_N']);
    h9 = plot(ax(4), tb, bT, 's', 'Color', cG, 'MarkerFaceColor', cG, 'MarkerSize', 5, 'DisplayName', [AB '_{Tm} / ' AB '_T']);
    ylim(ax(4), [0.6 1.6]);  legend(ax(4), [h7 h8 h9], 'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', LFS, 'FontWeight', 'bold');
    ylabel(ax(4), 'measured / true', 'FontSize', FS, 'FontWeight', 'bold');  xlabel(ax(4), 'Time (sec)', 'FontSize', FS, 'FontWeight', 'bold');
    for k = 1:4; xlim(ax(k), [0 t(end)]);  set(ax(k), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 2.2);  grid(ax(k), 'off'); end
    out = fullfile(od, sprintf('fig_t1b_angle_only_s%d.png', seed));
    exportgraphics(f, out, 'Resolution', 140);  close(f);  fprintf('figure: %s\n', out);
end
