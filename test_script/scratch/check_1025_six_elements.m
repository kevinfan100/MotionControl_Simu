% STATUS: SCRATCH -- 2026-10-25 (user): are the six elements of A-hat estimated well? Computed angle (Section 5), fixed point 50 / 15 / 5 um,
%   20 deg wall, seeds 1-10, 10 s. Per element over 5-10 s: bias = mean over seeds of the time-mean error, SEM, sd across seeds,
%   honesty = rms error / mean sqrt P (1 = the filter's own uncertainty is right). Criterion written before the run: |bias| < 2 SEM
%   and honesty in [0.5, 2] for every element. Figure: 15 um, seed 1, six elements (red true, blue estimate, light band +-sqrt P).
%   Output test_results/temp_3d_wsonly/fig_1025_six_elements_15um.png | EXPIRES: when the element check is read
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  um = [50 15 5];  seeds = 1:10;  T = 10;  n = numel(seeds);
nm = {'xx', 'yy', 'zz', 'xy', 'xz', 'yz'};  ij = [1 1; 2 2; 3 3; 1 2; 1 3; 2 3];
keep = cell(1, n);
for ih = 1:numel(um)
    E = NaN(n, 6);  S = NaN(n, 6);  Ermsall = NaN(n, 6);
    for i = 1:n
        R = run_t3_xyz_fixed(struct('theta_y', 20, 'h0', um(ih) / Rr, 'seeds', seeds(i), 'T', T, 'aprime', true, 'tag', 'six'));  r = R.runs{1};
        if um(ih) == 15; keep{i} = r; end
        kk = find(r.t >= 5);  at = arrayfun(@(j) r.A_true(ij(j, 1), ij(j, 2)), 1:6);
        e = r.a(kk, :) - at;  E(i, :) = mean(e, 1);  Ermsall(i, :) = sqrt(mean(e.^2, 1));  S(i, :) = mean(r.sPa(kk, :), 1);
    end
    bias = mean(E, 1);  sem = std(E, 0, 1) / sqrt(n);  hon = sqrt(mean(Ermsall.^2, 1)) ./ mean(S, 1);
    fprintf('\n%2d um (a_par - a_perp = %.3f)\n', um(ih), r.aT_true(1) - r.aN_true(1));
    for j = 1:6
        ok = abs(bias(j)) < 2 * sem(j) && hon(j) > 0.5 && hon(j) < 2;
        fprintf('  a_%s: true %.4f | bias %+.4f (SEM %.4f, %+.1f SEM) | sd across seeds %.4f | mean sqrtP %.4f | honesty %.2f -> %s\n', nm{j}, ...
                at(j), bias(j), sem(j), bias(j) / sem(j), std(E(:, j)), mean(S(:, j)), hon(j), local_pf(ok));
    end
end
% figure: 15 um seed 1
r = keep{1};  at = arrayfun(@(j) r.A_true(ij(j, 1), ij(j, 2)), 1:6);  FS = 13;  cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];
fig = figure('Color', 'w', 'Position', [60 60 1300 900]);
tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for j = 1:6
    nexttile;  hold on;  box on;  t = r.t;
    fill([t; flipud(t)], [r.a(:, j) + r.sPa(:, j); flipud(r.a(:, j) - r.sPa(:, j))], cBl, 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    plot(t, at(j) + 0 * t, 'r', 'LineWidth', 2.5);  h = plot(t, r.a(:, j), 'Color', cB, 'LineWidth', 1.2);
    ylim(at(j) + [-0.08 0.08]);  xlim([0 T]);  set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
    legend(h, sprintf('%s_{%s},  seed 1, 15 %sm   (red = true, band = %sP)', [char(226)], nm{j}, char(181), char(8730)), 'Location', 'northoutside');
    if j >= 5; xlabel('Time (sec)'); end
end
exportgraphics(fig, fullfile(fo, 'fig_1025_six_elements_15um.png'), 'Resolution', 150);
close(fig);

function s = local_pf(ok)
    if ok; s = 'PASS'; else; s = 'FAIL'; end
end
