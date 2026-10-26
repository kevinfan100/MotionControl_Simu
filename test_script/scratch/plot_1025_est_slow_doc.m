% STATUS: SCRATCH -- 2026-10-25 (user): document figure of 1025 Section 9 "Angle error of the estimated angle". 20 deg wall, fixed point
%   15 / 5 um, seeds 1-10, 10 s (parfor); estimated angle with theta-hat[0] = 0 (prior 10 deg), computed angle on the same seeds.
%   Left: sqrt P_theta_y of the estimated angle (solid, mean over seeds) and e(t) / 2(a_par - a_perp) from the computed angle (dashed).
%   Right: rms(theta_y - theta-hat_y) of the estimated angle (solid) and the prediction sqrt( (P/P0 * 20 deg)^2 + P ) (dashed).
%   Output reference/eq17_analysis/derivation/figures/fig_1025_est_slow.png | used by 1025_computed_estimated_angle.tex Section 9
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'reference', 'eq17_analysis', 'derivation', 'figures');
Rr = 2.25;  um = [15 5];  seeds = 1:10;  n = numel(seeds);  T = 10;  FS = 15;  abar = char(257);  hatc = char(770);
col = {[0 0.3 0.85], [0 0.55 0.25]};  dlab = sprintf('%s_%s - %s_%s', abar, char(8741), abar, char(8869));  P0 = 10;  th_err0 = 20;
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
fig = figure('Color', 'w', 'Position', [80 80 1500 600]);
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
a1 = nexttile(1);  hold on;  box on;  a2 = nexttile(2);  hold on;  box on;  hL = gobjects(0);  lL = {};  hR = gobjects(0);  lR = {};
for ih = 1:numel(um)
    rc = cell(1, n);  re = cell(1, n);  h0 = um(ih) / Rr;
    parfor i = 1:n
        R = run_t3_xyz_fixed(struct('theta_y', 20, 'h0', h0, 'seeds', seeds(i), 'T', T, 'aprime', true, 'tag', 'es_c'));  rc{i} = R.runs{1};
        R = run_t3_xyz_fixed(struct('theta_y', 20, 'h0', h0, 'seeds', seeds(i), 'T', T, 'aprime', true, 'form', 'estimated', 'th0_deg', [0 0], 'tag', 'es_e'));
        re{i} = R.runs{1};
    end
    t = rc{1}.t;  A = rc{1}.A_true;  N0 = -2 * A(1, 3);  D0 = A(1, 1) - A(3, 3);  d = rc{1}.aT_true(1) - rc{1}.aN_true(1);
    EN = NaN(numel(t), n);  ED = EN;  EY = EN;  SP = EN;
    for i = 1:n
        EN(:, i) = N0 - (-2 * rc{i}.a(:, 5));  ED(:, i) = D0 - (rc{i}.a(:, 1) - rc{i}.a(:, 3));
        EY(:, i) = 20 - re{i}.thy;  SP(:, i) = re{i}.sPth(:, 2);
    end
    e = sqrt(mean([EN, ED].^2, 2));  bound = e / (2 * d) * 180 / pi;  sp = mean(SP, 2);
    ky = sqrt(mean(EY.^2, 2));  pred = sqrt(((sp / P0).^2 * th_err0).^2 + sp.^2);
    k = t >= 0.2;
    hL(end+1) = plot(a1, t(k), sp(k), '-', 'Color', col{ih}, 'LineWidth', 2.2);
    hL(end+1) = plot(a1, t(k), bound(k), '--', 'Color', col{ih}, 'LineWidth', 2.2);
    lL = [lL, {sprintf('%d %sm, %sP (estimated angle)', um(ih), char(181), char(8730)), sprintf('%d %sm, e(t) / [2(%s)] (computed)', um(ih), char(181), dlab)}];
    hR(end+1) = plot(a2, t(k), ky(k), '-', 'Color', col{ih}, 'LineWidth', 2.2);
    hR(end+1) = plot(a2, t(k), pred(k), '--', 'Color', col{ih}, 'LineWidth', 2.2);
    lR = [lR, {sprintf('%d %sm, simulation', um(ih), char(181)), sprintf('%d %sm, initial residual + %sP', um(ih), char(181), char(8730))}];
    ks = arrayfun(@(s) find(t >= s - 1e-9, 1), [1 5 10]);
    fprintf('%2d um: d %.3f | sqrtP at 1/5/10 s %s | e/(2d) %s | rms error %s | prediction %s deg\n', um(ih), d, mat2str(round(sp(ks)', 1)), ...
            mat2str(round(bound(ks)', 1)), mat2str(round(ky(ks)', 1)), mat2str(round(pred(ks)', 1)));
end
set(a1, 'YLim', [0 25], 'XLim', [0 T], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
ylabel(a1, '[\circ]');  xlabel(a1, 'Time (sec)');  legend(a1, hL, lL, 'Location', 'northeast', 'FontSize', 12);
set(a2, 'YLim', [0 25], 'XLim', [0 T], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
ylabel(a2, sprintf('rms(%s_y - %s_y)  [%s]', char(952), ['θ' hatc], char(176)));  xlabel(a2, 'Time (sec)');
legend(a2, hR, lR, 'Location', 'northeast', 'FontSize', 12);
exportgraphics(fig, fullfile(fo, 'fig_1025_est_slow.png'), 'Resolution', 150);
close(fig);
