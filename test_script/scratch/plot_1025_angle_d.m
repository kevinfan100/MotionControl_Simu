% STATUS: SCRATCH -- 2026-10-25 (user): why the computed angle converges slowly at a fixed point, shown with the Section 4 formula
%   tan 2 theta_y = -2 a_xz / (a_xx - a_zz), numerator = d sin 2 theta_y, denominator = d cos 2 theta_y, d = a_par - a_perp.
%   Computed angle, 20 deg wall, fixed point 50 / 15 / 5 um (columns), seeds 1-10, 10 s (parfor). Rows:
%   (1) numerator -2 a-hat_xz (blue) and denominator a-hat_xx - a-hat_zz (green), seed 1, true values red dashed;
%   (2) their errors, all seeds (seed 1 dark), same y axis in every column, e(t) = rms over seeds and both errors (black);
%   (3) theta_y error, seed 1 dark / others light, gray band +-e(t) / (2 d) [deg] = the error the formula predicts.
%   Output test_results/temp_3d_wsonly/fig_1025_angle_d.png | EXPIRES: when the argument is written
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));  addpath(here);
fo = fullfile(root, 'test_results', 'temp_3d_wsonly');
Rr = 2.25;  um = [50 15 5];  seeds = 1:10;  n = numel(seeds);  T = 10;  FS = 13;  hatc = char(770);  abar = char(257);
cB = [0 0.25 0.85];  cBl = [0.6 0.72 1];  cG = [0 0.5 0.2];  cGl = [0.6 0.85 0.65];  cR = [0.85 0 0];
if isempty(gcp('nocreate')); parpool('Processes', min(10, feature('numcores'))); end
runs = cell(numel(um), n);
for ih = 1:numel(um)
    rr = cell(1, n);  h0 = um(ih) / Rr;
    parfor i = 1:n
        R = run_t3_xyz_fixed(struct('theta_y', 20, 'h0', h0, 'seeds', seeds(i), 'T', T, 'aprime', true, 'tag', 'angd'));  rr{i} = R.runs{1};
    end
    runs(ih, :) = rr;
end
fig = figure('Color', 'w', 'Position', [40 40 1800 1150]);
tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = gobjects(3, 3);
for ih = 1:numel(um)
    r1 = runs{ih, 1};  t = r1.t;  A = r1.A_true;  num0 = -2 * A(1, 3);  den0 = A(1, 1) - A(3, 3);  d = r1.aT_true(1) - r1.aN_true(1);
    EN = NaN(numel(t), n);  ED = EN;  TY = EN;
    for i = 1:n
        r = runs{ih, i};  EN(:, i) = -2 * r.a(:, 5) - num0;  ED(:, i) = (r.a(:, 1) - r.a(:, 3)) - den0;  TY(:, i) = mod(r.thy - 20 + 90, 180) - 90;
    end
    e = sqrt(mean([EN, ED].^2, 2));  band = e / (2 * d) * 180 / pi;
    k5 = t >= 5;
    fprintf('%2d um: d = %.3f | true numerator %.4f, denominator %.4f | e(t) at 1/5/10 s %s | predicted e/(2d) 5-10 s %.1f deg | measured rms theta_y error 5-10 s %.1f deg\n', ...
            um(ih), d, num0, den0, mat2str(round(e(arrayfun(@(s) find(t >= s - 1e-9, 1), [1 5 10]))', 4)), mean(band(k5)), sqrt(mean(TY(k5, :).^2, 'all')));
    % row 1: numerator and denominator
    ax(1, ih) = nexttile(ih);  hold on;  box on;
    plot(t, num0 + 0 * t, '--', 'Color', cR, 'LineWidth', 2);  plot(t, den0 + 0 * t, ':', 'Color', cR, 'LineWidth', 2.5);
    h1 = plot(t, -2 * r1.a(:, 5), 'Color', cB, 'LineWidth', 1.2);  h2 = plot(t, r1.a(:, 1) - r1.a(:, 3), 'Color', cG, 'LineWidth', 1.2);
    ylim([-0.05 0.25]);  if ih == 1; ylabel('numerator, denominator'); end
    legend([h1 h2], {sprintf('-2%s_{xz}', ['a' hatc]), sprintf('%s_{xx} - %s_{zz}', ['a' hatc], ['a' hatc])}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    text(0.97, 0.9, sprintf('%d %sm,  d = %s_%s - %s_%s = %.3f', um(ih), char(181), abar, char(8741), abar, char(8869), d), 'Units', 'normalized', ...
         'HorizontalAlignment', 'right', 'FontSize', 12, 'FontWeight', 'bold');
    % row 2: errors
    ax(2, ih) = nexttile(3 + ih);  hold on;  box on;
    for i = n:-1:2; plot(t, EN(:, i), 'Color', cBl, 'LineWidth', 0.7);  plot(t, ED(:, i), 'Color', cGl, 'LineWidth', 0.7); end
    h1 = plot(t, EN(:, 1), 'Color', cB, 'LineWidth', 1.4);  h2 = plot(t, ED(:, 1), 'Color', cG, 'LineWidth', 1.4);
    h3 = plot(t, e, 'k', 'LineWidth', 2);  plot(t, -e, 'k', 'LineWidth', 2);  yline(0, 'r', 'LineWidth', 2);
    ylim([-0.06 0.06]);  if ih == 1; ylabel('error'); end
    legend([h1 h2 h3], {'numerator error', 'denominator error', sprintf('%se(t), rms of 10 seeds', char(177))}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    % row 3: angle error with the predicted band
    ax(3, ih) = nexttile(6 + ih);  hold on;  box on;
    fill([t; flipud(t)], [band; -flipud(band)], [0.85 0.85 0.85], 'EdgeColor', 'none');
    for i = n:-1:2; plot(t, TY(:, i), 'Color', cBl, 'LineWidth', 0.7); end
    h1 = plot(t, TY(:, 1), 'Color', cB, 'LineWidth', 1.4);  yline(0, 'r', 'LineWidth', 2);
    ylim([-30 30]);  if ih == 1; ylabel('\theta_y error  [\circ]'); end;  xlabel('Time (sec)');
    legend(h1, sprintf('%s_y - %s_y   (band: %se(t) / 2d)', ['θ' hatc], char(952), char(177)), 'Location', 'northoutside');
end
for k = 1:numel(ax); set(ax(k), 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2, 'XLim', [0 T]); end
exportgraphics(fig, fullfile(fo, 'fig_1025_angle_d.png'), 'Resolution', 150);
close(fig);
