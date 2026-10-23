% STATUS: SCRATCH -- 2026-10-22 (user asked for the angle measurement from the start): synthetic random component in the x-z plane
%   (variance per direction = the motion gain, a-bar_T 0.6 along the wall, a-bar_N 0.3 along the normal; C_T kappa_T = 1), seen in the
%   estimated local coordinate whose ruler is off by e_theta = 10 deg.
%   (a) many samples: the ellipse leans by (a_T - a_N) e_theta; (b) the gain difference from ONE sample, u^2 - w^2, vs the true 0.3.
%   Console: five single samples with lean u w, one-sample difference u^2 - w^2, and the angle read with each denominator.
%   EXPIRES: when the explanation is done | output test_results/figs_unused/fig_angle_meas_onepoint.png
here = fileparts(mfilename('fullpath'));  root = fileparts(fileparts(here));
aT = 0.6;  aN = 0.3;  e_th = 10 * pi / 180;  N = 4000;
rng(1);
loc = [sqrt(aT) * randn(1, N); sqrt(aN) * randn(1, N)];      % true local coordinate (along wall, normal)
Re = [cos(e_th) -sin(e_th); sin(e_th) cos(e_th)];              % ruler off by e_theta
uw = Re * loc;  u = uw(1, :);  w = uw(2, :);                 % what the estimator sees (u-hat, w-hat components)
fig = figure('Color', 'w', 'Position', [80 80 1300 520]);
tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile;  hold on;  box on;  axis equal;
plot(u, w, '.', 'Color', [0.55 0.75 1], 'MarkerSize', 4, 'DisplayName', 'samples');
ph = linspace(0, 2 * pi, 200);  ell = Re * [sqrt(aT) * cos(ph); sqrt(aN) * sin(ph)];
plot(ell(1, :), ell(2, :), 'r', 'LineWidth', 2.5, 'DisplayName', '1\sigma ellipse (wall)');
plot([-3 3], [0 0], 'b', 'LineWidth', 1.5, 'DisplayName', 'estimated wall direction');
xlim([-3 3]);  ylim([-2 2]);  xlabel('u-hat component');  ylabel('w-hat component');  set(gca, 'FontSize', 14);
legend('Location', 'northoutside', 'Orientation', 'horizontal');
nexttile;  hold on;  box on;
d1 = u.^2 - w.^2;
histogram(d1, -3:0.1:5, 'FaceColor', [0.55 0.75 1], 'EdgeColor', 'none', 'DisplayName', 'u^2 - w^2 from one sample');
xline(aT - aN, 'r', 'LineWidth', 2.5, 'DisplayName', 'true a_T - a_N = 0.3');
xline(0, 'k:', 'HandleVisibility', 'off');
xlabel('gain difference');  ylabel('count');  set(gca, 'FontSize', 14);
legend('Location', 'northoutside', 'Orientation', 'horizontal');
fo = fullfile(root, 'test_results', 'figs_unused', 'fig_angle_meas_onepoint.png');
exportgraphics(fig, fo, 'Resolution', 150);
fprintf('many samples: mean(u w) = %.4f (theory (a_T - a_N) e = %.4f), mean(u^2 - w^2) = %.3f\n', mean(u .* w), (aT - aN) * e_th, mean(d1));
fprintf('one sample: u^2 - w^2 sd %.2f, fraction < 0 %.2f, fraction |.| < 0.03 %.3f\n', std(d1), mean(d1 < 0), mean(abs(d1) < 0.03));
fprintf(' k    u      w     lean u w   u^2-w^2   angle / 0.3 [deg]   angle / (u^2-w^2) [deg]\n');
for k = 1:5
    fprintf('%2d  %5.2f  %5.2f   %6.3f    %6.3f      %8.1f            %8.1f\n', k, u(k), w(k), u(k) * w(k), d1(k), ...
            u(k) * w(k) / 0.3 * 180 / pi, u(k) * w(k) / d1(k) * 180 / pi);
end
fprintf('average of the angle over all %d samples: / 0.3 -> %.1f deg, / (u^2-w^2) -> %.1f deg (median %.1f)\n', N, ...
        mean(u .* w / 0.3) * 180 / pi, mean(u .* w ./ d1) * 180 / pi, median(u .* w ./ d1) * 180 / pi);
