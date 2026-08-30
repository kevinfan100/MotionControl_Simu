% L2 predict replay -- figures (one page per arm, z axis, seed 1 + 100-seed mean).
% panel 1 a_bar_hat vs a_true_n; panel 2 cumulative predict under left / trapezoid /
% midpoint; panel 3 per-step (left - trap) with the windows shaded.
% Style: no grid, no title, box on, legend northoutside horizontal, true red, estimate blue.
cd('/Users/kevin/code/MotionControl_Simu-motion-test');
RES = load('test_results/am_r22_deep/l2_routeB_results.mat');
W = {[0.5 1.5], 'descend'; [1.6 3.5], 'oscillate'; [3.8 4.8], 'trough'};
for arm = {'base', 'apknown'}
    R = RES.(arm{1});  t = R.t;
    f = figure('Visible', 'off', 'Position', [100 100 1100 1150], 'Color', 'w');
    tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    % panel 1
    ax1 = nexttile;  hold(ax1, 'on');
    shade(ax1, W, [-0.05 1.05]);
    plot(t, R.at(:, 1), 'r-', 'LineWidth', 1.6);
    plot(t, R.ah(:, 1), 'b-', 'LineWidth', 1.0);
    plot(t, mean(R.ah, 2), 'b--', 'LineWidth', 1.6);
    ylabel('a\_bar'); box on; ylim([-0.05 1.05]);
    legend({'true (seed 1)', 'estimate (seed 1)', 'estimate (100-seed mean)'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    % panel 2
    ax2 = nexttile;  hold(ax2, 'on');
    aT1 = R.aT(1);  aTm = mean(R.aT);
    cl = R.cum_left(:, 1) / aT1;  ct = R.cum_trap(:, 1) / aT1;  cm = R.cum_mid(:, 1) / aT1;
    clm = mean(R.cum_left ./ R.aT, 2);  ctm = mean(R.cum_trap ./ R.aT, 2);  cmm = mean(R.cum_mid ./ R.aT, 2);
    yl = [min([cl; ct; cm; clm; ctm; cmm]) max([cl; ct; cm; clm; ctm; cmm])];  yl = yl + 0.05 * diff(yl) * [-1 1];
    shade(ax2, W, yl);
    plot(t, cl, '-', 'Color', [0 0 0.8], 'LineWidth', 1.0);
    plot(t, ct, '-', 'Color', [0 0.6 0], 'LineWidth', 1.0);
    plot(t, cm, '-', 'Color', [0.85 0.4 0], 'LineWidth', 1.0);
    plot(t, clm, '--', 'Color', [0 0 0.8], 'LineWidth', 1.6);
    plot(t, ctm, '--', 'Color', [0 0.6 0], 'LineWidth', 1.6);
    plot(t, cmm, '--', 'Color', [0.85 0.4 0], 'LineWidth', 1.6);
    ylabel('cumulative predict / a\_T'); box on; ylim(yl);
    legend({'left (seed 1)', 'trapezoid (seed 1)', 'midpoint (seed 1)', 'left (mean)', 'trapezoid (mean)', 'midpoint (mean)'}, ...
           'Location', 'northoutside', 'Orientation', 'horizontal');
    % panel 3
    ax3 = nexttile;  hold(ax3, 'on');
    s1 = 100 * R.step_LmT(:, 1) / aT1;  sm = 100 * mean(R.step_LmT ./ R.aT, 2);
    yl = [min([s1; sm]) max([s1; sm])];  yl = yl + 0.05 * diff(yl) * [-1 1];
    shade(ax3, W, yl);
    plot(t, s1, '-', 'Color', [0.3 0.3 0.9], 'LineWidth', 0.8);
    plot(t, sm, '-', 'Color', [0 0 0], 'LineWidth', 1.4);
    ylabel('(left - trap) per step [% of a\_T]'); xlabel('t [s]'); box on; ylim(yl);
    legend({'seed 1', '100-seed mean'}, 'Location', 'northoutside', 'Orientation', 'horizontal');
    for ax = [ax1 ax2 ax3]; set(ax, 'FontSize', 12, 'XGrid', 'off', 'YGrid', 'off'); xlim(ax, [0 4.8]); end
    out = sprintf('test_results/am_r22_deep/l2_replay_%s.png', arm{1});
    exportgraphics(f, out, 'Resolution', 150);
    close(f);
    fprintf('saved %s\n', out);
end

function shade(ax, W, yl)
    for i = 1:size(W, 1)
        patch(ax, [W{i, 1}(1) W{i, 1}(2) W{i, 1}(2) W{i, 1}(1)], [yl(1) yl(1) yl(2) yl(2)], ...
              [0.92 0.92 0.92], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end
end
