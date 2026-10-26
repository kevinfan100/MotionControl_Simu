% STATUS: SCRATCH -- 2026-10-25 (user): document figure of 1025 Section 7 "Angle error of the computed angle". Computed angle, 20 deg wall,
%   fixed point 15 / 5 um, seeds 1-10, 10 s (parfor). Left (log y): e(t) = rms over seeds of e_N = N - N-hat and e_D = D - D-hat
%   (N = -2 a_xz, D = a_xx - a_zz), solid; dashed = 2 (a_par - a_perp) tan 5 deg; circle = first time e(t) stays below it.
%   Right: rms(theta_y - theta-hat_y) over seeds (solid) and e(t) / [2 (a_par - a_perp)] (dashed).
%   Output reference/eq17_analysis/derivation/figures/fig_1025_angle_err.png | used by 1025_computed_estimated_angle.tex Section 7
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
Rr = 2.25;  um = [15 5];  seeds = 1:10;  n = numel(seeds);  T = 10;  FS = 15;  abar = char(257);  hatc = char(770);
col = {[0 0.3 0.85], [0 0.55 0.25]};  dlab = sprintf('%s_%s - %s_%s', abar, char(8741), abar, char(8869));
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
fig = figure('Color', 'w', 'Position', [80 80 1500 600]);
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
a1 = nexttile(1);  hold on;  box on;  a2 = nexttile(2);  hold on;  box on;  hL = gobjects(0);  hR = gobjects(0);  lR = {};
for ih = 1:numel(um)
    rr = cell(1, n);  h0 = um(ih) / Rr;
    parfor i = 1:n
        R = run_t3_xyz_fixed(struct('theta_y', 20, 'h0', h0, 'seeds', seeds(i), 'T', T, 'aprime', true, 'tag', 'angerr'));  rr{i} = R.runs{1};
    end
    t = rr{1}.t;  A = rr{1}.A_true;  N0 = -2 * A(1, 3);  D0 = A(1, 1) - A(3, 3);  d = rr{1}.aT_true(1) - rr{1}.aN_true(1);
    EN = NaN(numel(t), n);  ED = EN;  TY = EN;
    for i = 1:n
        EN(:, i) = N0 - (-2 * rr{i}.a(:, 5));  ED(:, i) = D0 - (rr{i}.a(:, 1) - rr{i}.a(:, 3));  TY(:, i) = mod(20 - rr{i}.thy + 90, 180) - 90;
    end
    e = sqrt(mean([EN, ED].^2, 2));  ky = sqrt(mean(TY.^2, 2));  pred = e / (2 * d) * 180 / pi;  thr = 2 * d * tand(5);
    k = t >= 0.2;  kc = find(e > thr, 1, 'last');  tc = t(min(kc + 1, numel(t)));
    hL(end+1) = plot(a1, t(k), e(k), '-', 'Color', col{ih}, 'LineWidth', 2.2);
    plot(a1, t(k), thr + 0 * t(k), '--', 'Color', col{ih}, 'LineWidth', 2.2);
    plot(a1, tc, thr, 'o', 'MarkerSize', 13, 'LineWidth', 2.5, 'Color', col{ih});
    text(a1, tc + 0.25, thr * 1.25, sprintf('%.1f s', tc), 'Color', col{ih}, 'FontSize', 14, 'FontWeight', 'bold');
    if um(ih) == 15; xt = 0.3; ha = 'left'; yt = thr * 0.8; else; xt = 9.85; ha = 'right'; yt = thr * 0.82; end   % keep labels off the curves
    text(a1, xt, yt, sprintf('2(%s) tan 5%s = %.4f', dlab, char(176), thr), 'Color', col{ih}, 'HorizontalAlignment', ha, ...
         'FontSize', 12, 'FontWeight', 'bold');
    hR(end+1) = plot(a2, t(k), ky(k), '-', 'Color', col{ih}, 'LineWidth', 2.2);
    hR(end+1) = plot(a2, t(k), pred(k), '--', 'Color', col{ih}, 'LineWidth', 2.2);
    lR = [lR, {sprintf('%d %sm, simulation', um(ih), char(181)), sprintf('%d %sm, e(t) / [2(%s)]', um(ih), char(181), dlab)}];
    fprintf('%2d um: a_par %.3f a_perp %.3f d %.3f | threshold %.4f crossed at %.1f s | e at 1/5/10 s %s | rms angle %s | pred %s deg\n', um(ih), ...
            rr{1}.aT_true(1), rr{1}.aN_true(1), d, thr, tc, mat2str(round(e(arrayfun(@(s) find(t >= s - 1e-9, 1), [1 5 10]))', 3)), ...
            mat2str(round(ky(arrayfun(@(s) find(t >= s - 1e-9, 1), [1 5 10]))', 1)), mat2str(round(pred(arrayfun(@(s) find(t >= s - 1e-9, 1), [1 5 10]))', 1)));
end
set(a1, 'YScale', 'log', 'YLim', [0.006 0.15], 'XLim', [0 T], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
ylabel(a1, 'e(t)');  xlabel(a1, 'Time (sec)');
legend(a1, hL, arrayfun(@(u) sprintf('e(t),  %d %sm', u, char(181)), um, 'UniformOutput', false), 'Location', 'northoutside', 'Orientation', 'horizontal');
set(a2, 'YLim', [0 25], 'XLim', [0 T], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
ylabel(a2, sprintf('rms(%s_y - %s_y)  [%s]', char(952), ['θ' hatc], char(176)));  xlabel(a2, 'Time (sec)');
legend(a2, hR, lR, 'Location', 'northeast', 'FontSize', 12);
exportgraphics(fig, fullfile(fo, 'fig_1025_angle_err.png'), 'Resolution', 150);
close(fig);
