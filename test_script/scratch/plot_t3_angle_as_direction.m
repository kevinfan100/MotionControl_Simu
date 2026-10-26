% STATUS: SCRATCH -- 2026-10-24 (user: "why does the computed angle jump at 50 um"): the angle as the direction of a point.
%   Point = (A_xx - A_zz, -2 A_xz); distance from the origin = a_T - a_N, direction = 2 theta_y. Red = true point, blue = the estimates
%   A-hat of run_t3_pair (computed angle, seeds 1-3, every 20th step after 5 s). Left 15 um, right 50 um, same axes.
%   Output test_results/figs_unused/fig_t3_angle_as_direction.png | EXPIRES: when the explanation is done
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
S = load(fullfile(root, 'test_results', 'temp_3d_wsonly', 'temp_t3_pair.mat'));
Rr = 2.25;  um = [15 50];  L = 0.12;
fig = figure('Color', 'w', 'Position', [80 80 1300 620]);
tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for ih = 1:2
    nexttile;  hold on;  box on;  axis equal;
    sel = find(abs(S.HH - um(ih) / Rr) < 1e-6)';
    for i = sel
        a = S.xyz{i};  k = find(a.t > 5);  k = k(1:20:end);
        plot(a.a(k, 1) - a.a(k, 3), -2 * a.a(k, 5), '.', 'Color', [0.45 0.6 1], 'MarkerSize', 6);
    end
    At = S.xyz{sel(1)}.A_true;  px = At(1,1) - At(3,3);  py = -2 * At(1,3);
    plot([0 px], [0 py], 'r-', 'LineWidth', 2);  plot(px, py, 'r.', 'MarkerSize', 30);
    plot(0, 0, 'k+', 'MarkerSize', 14, 'LineWidth', 2);
    th = linspace(0, 2 * pi, 200);  plot(0.01 * cos(th) + px, 0.01 * sin(th) + py, 'r--', 'LineWidth', 1);
    xlim([-0.04 L]);  ylim([-0.04 L]);  xlabel('A_{xx} - A_{zz}');  ylabel('-2 A_{xz}');
    set(gca, 'FontSize', 15, 'FontWeight', 'bold', 'LineWidth', 1.2);
    text(0.03, 0.95, sprintf('%d %sm:  distance %.3f,  direction 2%s_y = 40%s', um(ih), char(181), hypot(px, py), char(952), char(176)), ...
         'Units', 'normalized', 'FontSize', 15, 'FontWeight', 'bold');
    fprintf('%d um: true point (%.4f, %.4f), distance %.4f\n', um(ih), px, py, hypot(px, py));
end
exportgraphics(fig, fullfile(root, 'test_results', 'figs_unused', 'fig_t3_angle_as_direction.png'), 'Resolution', 150);
close(fig);
