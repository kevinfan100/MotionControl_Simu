% STATUS: SCRATCH -- 2026-10-25 (user): document figure for the quality of the three elements used by the computed angle (a_xx, a_zz,
%   a_xz): unbiased and honest. Computed angle, 20 deg wall, fixed point 15 um, seeds 1-10, 10 s (parfor). Columns a_xx | a_zz | a_xz:
%   lines = error (true - estimate) of the 3 seeds with the smallest 5-10 s error (user 10-25: show the more accurate seeds; disclosed
%   in the document), best one dark; black solid = +-rms over seeds, black dashed = +-mean sqrt P (the filter's own).
%   Console: bias over 5-10 s (SEM multiple) and honesty = rms error / mean sqrt P, for the table.
%   Output reference/eq17_analysis/derivation/figures/fig_1025_a_quality.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
Rr = 2.25;  seeds = 1:10;  n = numel(seeds);  T = 10;  FS = 15;  abar = char(257);  hatc = char(770);
el = [1 3 5];  nm = {'xx', 'zz', 'xz'};  ij = [1 1; 3 3; 1 3];  cl = [0.62 0.72 0.95];
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
rr = cell(1, n);
parfor i = 1:n
    R = run_t3_xyz_fixed(struct('theta_y', 20, 'h0', 15 / Rr, 'seeds', seeds(i), 'T', T, 'aprime', true, 'tag', 'aq'));  rr{i} = R.runs{1};
end
t = rr{1}.t;  k = t >= 0.2;  k5 = t >= 5;
sc = zeros(1, n);                                          % selection score: rms error over 5-10 s, mean of the three elements
for i = 1:n; for c = 1:3; sc(i) = sc(i) + sqrt(mean((rr{i}.A_true(ij(c, 1), ij(c, 2)) - rr{i}.a(k5, el(c))).^2)); end; end
[~, ord] = sort(sc);  show = ord(1:3);  fprintf('seeds shown (smallest error): %s\n', mat2str(seeds(show)));
fig = figure('Color', 'w', 'Position', [80 80 1650 520]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for c = 1:3
    nexttile;  hold on;  box on;
    at = rr{1}.A_true(ij(c, 1), ij(c, 2));  E = NaN(numel(t), n);  S = E;
    for i = 1:n; E(:, i) = at - rr{i}.a(:, el(c));  S(:, i) = rr{i}.sPa(:, el(c)); end
    for i = show(end:-1:1); cc = cl; lw = 0.9; if i == show(1); cc = [0 0.25 0.85]; lw = 1.5; end; plot(t(k), E(k, i), 'Color', cc, 'LineWidth', lw); end
    rm = sqrt(mean(E.^2, 2));  sp = mean(S, 2);
    h1 = plot(t(k), rm(k), 'k-', 'LineWidth', 2.2);  plot(t(k), -rm(k), 'k-', 'LineWidth', 2.2);
    h2 = plot(t(k), sp(k), 'k--', 'LineWidth', 2.2);  plot(t(k), -sp(k), 'k--', 'LineWidth', 2.2);
    yline(0, 'r', 'LineWidth', 2);
    ylim([-0.04 0.04]);  xlim([0 T]);  set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  xlabel('Time (sec)');
    if c == 1; ylabel('true - estimate'); end
    legend([h1 h2], {sprintf('%s rms over 10 seeds', char(177)), sprintf('%s %sP (filter)', char(177), char(8730))}, ...
           'Location', 'southeast', 'FontSize', 12);
    text(0.97, 0.92, sprintf('%s_{%s} - %s_{%s}', abar, nm{c}, ['a' hatc], nm{c}), 'Units', 'normalized', 'FontSize', 17, 'FontWeight', 'bold', 'HorizontalAlignment', 'right', 'BackgroundColor', 'w', 'Margin', 1);
    b = mean(E(k5, :), 1);  bias = mean(b);  sem = std(b) / sqrt(n);
    hon = sqrt(mean(E(k5, :).^2, 'all')) / mean(S(k5, :), 'all');
    fprintf('a_%s: true %.4f | bias %+.4f (SEM %.4f, %.1f SEM) | rms err %.4f | mean sqrtP %.4f | honesty %.2f\n', nm{c}, at, bias, sem, ...
            abs(bias) / sem, sqrt(mean(E(k5, :).^2, 'all')), mean(S(k5, :), 'all'), hon);
end
exportgraphics(fig, fullfile(fo, 'fig_1025_a_quality.png'), 'Resolution', 150);
close(fig);
