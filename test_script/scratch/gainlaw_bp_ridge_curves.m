%GAINLAW_BP_RIDGE_CURVES  Visualize the (b,p) ridge of the Form B gain law.
%   Plots the true perpendicular gain curve against Form B with the anchored
%   pair (9/8, 1) and three points marching along the b = p ridge
%   (2.5, 25, 250).  Along b = p the near-wall slope p/b = 1 is pinned and
%   the law converges to the exponential limit 1 - exp(-(w-1)), so wildly
%   different parameter values produce nearly identical curves -- the
%   degeneracy that makes (b, p) unidentifiable for the filter.
%   Companion evidence for the Tier-1 c1/c2/c3 experiments (2026-07-31).
%   Style per .claude/rules/figure-style.md; stats to console.

clear; clc;
here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', '..')));

h = linspace(1.1, 10, 4000);
D = zeros(size(h));
for i = 1:numel(h)
    [~, c_perp] = calc_correction_functions(h(i), false);
    D(i) = 1 / c_perp;                       % true normalized gain (z, perp)
end

g = h - 1;                                    % gap, w_s = 1
formB = @(b, p) 1 - (1 + g./b).^(-p);
sets  = {9/8, 1, 'b=9/8, p=1 (anchors)', [0 0.447 0.741];
         2.5, 2.5, 'b=p=2.5',            [0.85 0.33 0.10];
         25,  25,  'b=p=25',             [0.47 0.67 0.19];
         250, 250, 'b=p=250',            [0.49 0.18 0.56]};

fig = figure('Position', [100 100 900 720], 'Color', 'w');
tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile(tl);  hold(ax1, 'on');
plot(ax1, h, D, 'r-', 'LineWidth', 2.5, 'DisplayName', 'true (c_{\perp})');
for k = 1:size(sets, 1)
    plot(ax1, h, formB(sets{k,1}, sets{k,2}), '-', 'LineWidth', 1.5, ...
        'Color', sets{k,4}, 'DisplayName', sets{k,3});
end
ylabel(ax1, 'a\_bar');  box(ax1, 'on');  grid(ax1, 'off');
legend(ax1, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
    'NumColumns', 5);

ax2 = nexttile(tl);  hold(ax2, 'on');
plot(ax2, h, zeros(size(h)), 'r-', 'LineWidth', 1.0, 'DisplayName', 'zero');
for k = 1:size(sets, 1)
    err = 100 * (formB(sets{k,1}, sets{k,2}) - D);
    plot(ax2, h, err, '-', 'LineWidth', 1.5, ...
        'Color', sets{k,4}, 'DisplayName', sets{k,3});
    fprintf('%-18s  sup|err| [1.1,10] = %6.3f %%   [2,10] = %6.3f %%\n', ...
        sets{k,3}, max(abs(err)), max(abs(err(h >= 2))));
end
xlabel(ax2, 'w\_bar');  ylabel(ax2, 'a\_bar error [%]');
box(ax2, 'on');  grid(ax2, 'off');
legend(ax2, 'Location', 'northoutside', 'Orientation', 'horizontal', ...
    'NumColumns', 5);

% ridge convergence: distance between successive b=p curves
d1 = max(abs(formB(2.5, 2.5) - formB(25, 25)));
d2 = max(abs(formB(25, 25)  - formB(250, 250)));
d3 = max(abs(formB(250, 250) - (1 - exp(-g))));
fprintf('ridge convergence: sup|2.5-25| = %.4f, sup|25-250| = %.4f, sup|250-exp limit| = %.4f\n', ...
    d1, d2, d3);

out_png = fullfile(here, '..', '..', 'test_results', 'gainlaw_bp_ridge_curves.png');
exportgraphics(fig, out_png, 'Resolution', 150);
fprintf('saved: %s\n', out_png);
