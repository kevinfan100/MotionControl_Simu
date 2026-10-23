% Figure for sweep_R2_trust_y2.m. Left: does trusting y2 more remove the
% near-wall bias? Right: what it costs in per-seed scatter.
cd('/Users/kevin/Code/MotionControl_Simu-motion-test');
L = load('test_results/am_r22_deep/sweep_R2_trust_y2.mat');
res = L.res;  f = [res.f];
mu  = 100 * cellfun(@mean, {res.bias});
sd  = 100 * cellfun(@std,  {res.bias});
sem = sd ./ sqrt(cellfun(@numel, {res.bias}));

fig = figure('Color', 'w', 'Position', [100 100 1000 420]);

ax1 = subplot(1, 2, 1);
errorbar(f, mu, sem, 'o-', 'LineWidth', 2, 'MarkerSize', 9, ...
         'MarkerFaceColor', [0 0.35 0.85], 'Color', [0 0.35 0.85]); hold on;
yline(0, 'k--', 'LineWidth', 1);
plot(1, mu(f == 1), 'rs', 'MarkerSize', 14, 'LineWidth', 2);
set(ax1, 'XScale', 'log', 'FontSize', 14, 'Box', 'on');
xlabel('amlpf\_var\_factor  (R_2 multiplier)', 'FontSize', 14);
ylabel('trough-hold bias on a-hat [%]', 'FontSize', 14);
xlim([0.005 20]);
legend({'bias \pm SEM', 'zero', 'derived R_2 (production)'}, ...
       'Orientation', 'horizontal', 'Location', 'northoutside', 'FontSize', 12);

ax2 = subplot(1, 2, 2);
plot(f, sd, 'o-', 'LineWidth', 2, 'MarkerSize', 9, ...
     'MarkerFaceColor', [0.85 0.33 0.10], 'Color', [0.85 0.33 0.10]); hold on;
plot(1, sd(f == 1), 'rs', 'MarkerSize', 14, 'LineWidth', 2);
set(ax2, 'XScale', 'log', 'FontSize', 14, 'Box', 'on');
xlabel('amlpf\_var\_factor  (R_2 multiplier)', 'FontSize', 14);
ylabel('per-seed scatter, sd [%]', 'FontSize', 14);
xlim([0.005 20]);
legend({'sd across 30 seeds', 'derived R_2 (production)'}, ...
       'Orientation', 'horizontal', 'Location', 'northoutside', 'FontSize', 12);

exportgraphics(fig, 'test_results/am_r22_deep/fig13_R2_trust_sweep.png', 'Resolution', 150);
fprintf('saved test_results/am_r22_deep/fig13_R2_trust_sweep.png\n');
for i = 1:numel(f)
    fprintf('  factor %6.3g : bias %+7.3f%%  sd %6.3f%%  |K2|/|K1| %.5f\n', ...
            f(i), mu(i), sd(i), res(i).K2/res(i).K1);
end
