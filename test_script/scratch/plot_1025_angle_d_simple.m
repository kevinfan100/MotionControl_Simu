% STATUS: SCRATCH -- 2026-10-25 (user: the 3x3 version was hard to read): one message, two panels. Computed angle, 20 deg wall, fixed point
%   50 / 15 / 5 um, seeds 1-10, 10 s (parfor). Left (log y): e(t) = rms over seeds of the numerator (-2 a_xz) and denominator
%   (a_xx - a_zz) errors, one solid line per height (they overlap), dashed = the signal size d = a_par - a_perp of that height.
%   Right: rms theta_y error over seeds (solid) and the formula's e(t) / (2 d) (dashed), same colors.
%   Output test_results/temp_3d_wsonly/fig_1025_angle_d_simple.png | EXPIRES: when the argument is written
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  um = [50 15 5];  seeds = 1:10;  n = numel(seeds);  T = 10;  FS = 15;  abar = char(257);
if exist('um_sel', 'var'); um = um_sel; end              % 10-25: um_sel = [15 5] (user)
colmap = containers.Map({50, 15, 5}, {[0.93 0.5 0.1], [0 0.3 0.85], [0 0.55 0.25]});  col = arrayfun(@(u) colmap(u), um, 'UniformOutput', false);
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
fig = figure('Color', 'w', 'Position', [80 80 1500 620]);
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
a1 = nexttile(1);  hold on;  box on;  a2 = nexttile(2);  hold on;  box on;  hL = gobjects(0);  hR = gobjects(0);
for ih = 1:numel(um)
    rr = cell(1, n);  h0 = um(ih) / Rr;
    parfor i = 1:n
        R = run_t3_xyz_fixed(struct('theta_y', 20, 'h0', h0, 'seeds', seeds(i), 'T', T, 'aprime', true, 'tag', 'angd'));  rr{i} = R.runs{1};
    end
    t = rr{1}.t;  A = rr{1}.A_true;  num0 = -2 * A(1, 3);  den0 = A(1, 1) - A(3, 3);  d = rr{1}.aT_true(1) - rr{1}.aN_true(1);
    EN = NaN(numel(t), n);  ED = EN;  TY = EN;
    for i = 1:n
        EN(:, i) = -2 * rr{i}.a(:, 5) - num0;  ED(:, i) = (rr{i}.a(:, 1) - rr{i}.a(:, 3)) - den0;  TY(:, i) = mod(rr{i}.thy - 20 + 90, 180) - 90;
    end
    e = sqrt(mean([EN, ED].^2, 2));  ky = sqrt(mean(TY.^2, 2));  pred = e / (2 * d) * 180 / pi;
    k = t >= 0.2;
    hL(end+1) = plot(a1, t(k), e(k), '-', 'Color', col{ih}, 'LineWidth', 2.2);
    thr = 2 * d * tand(5);                                     % e below this -> theta_y error below 5 deg (first order)
    plot(a1, t(k), thr + 0 * t(k), '--', 'Color', col{ih}, 'LineWidth', 2.2);
    kc = find(e > thr, 1, 'last');  tc = NaN;  if kc < numel(t); tc = t(kc + 1); end
    if isfinite(tc); plot(a1, tc, thr, 'o', 'MarkerSize', 12, 'LineWidth', 2.5, 'Color', col{ih});  xline(a1, tc, ':', 'Color', col{ih}, 'LineWidth', 1.5); end
    lbl = sprintf('5%s line = 2d tan5%s = %.4f  (%d %sm, d = %.3f)', char(176), char(176), thr, um(ih), char(181), d);
    if isfinite(tc); lbl = [lbl sprintf(', crossed at %.1f s', tc)]; else; lbl = [lbl ', not crossed in 10 s']; end
    text(a1, 9.9, thr * 1.13, lbl, 'Color', col{ih}, 'HorizontalAlignment', 'right', 'FontSize', 12, 'FontWeight', 'bold');
    hR(end+1) = plot(a2, t(k), ky(k), '-', 'Color', col{ih}, 'LineWidth', 2.2);
    plot(a2, t(k), pred(k), '--', 'Color', col{ih}, 'LineWidth', 2.2);
    fprintf('%2d um: d %.3f | e at 1/5/10 s %s | angle rms at 1/5/10 s %s deg | e/(2d) at 1/5/10 s %s deg\n', um(ih), d, ...
            mat2str(round(e(arrayfun(@(s) find(t >= s - 1e-9, 1), [1 5 10]))', 3)), mat2str(round(ky(arrayfun(@(s) find(t >= s - 1e-9, 1), [1 5 10]))', 1)), ...
            mat2str(round(pred(arrayfun(@(s) find(t >= s - 1e-9, 1), [1 5 10]))', 1)));
end
set(a1, 'YScale', 'log', 'YLim', [0.003 0.15], 'XLim', [0 T], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
ylabel(a1, 'numerator / denominator error');  xlabel(a1, 'Time (sec)');
legend(a1, hL, arrayfun(@(u) sprintf('error e(t), %d %sm', u, char(181)), um, 'UniformOutput', false), 'Location', 'northoutside', 'Orientation', 'horizontal');
if numel(um) == 2; yl2 = 25; else; yl2 = 40; end
set(a2, 'YLim', [0 yl2], 'XLim', [0 T], 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);
ylabel(a2, '\theta_y error, rms of 10 seeds  [\circ]');  xlabel(a2, 'Time (sec)');
legend(a2, hR, arrayfun(@(u) sprintf('%d %sm', u, char(181)), um, 'UniformOutput', false), 'Location', 'northoutside', 'Orientation', 'horizontal');
text(a2, 9.8, 0.92 * yl2, 'solid: simulation    dashed: e(t) / 2d', 'HorizontalAlignment', 'right', 'FontSize', 13, 'FontWeight', 'bold');
sfx = '';  if numel(um) == 2; sfx = '_15_5'; end
exportgraphics(fig, fullfile(fo, ['fig_1025_angle_d_simple' sfx '.png']), 'Resolution', 150);
close(fig);
