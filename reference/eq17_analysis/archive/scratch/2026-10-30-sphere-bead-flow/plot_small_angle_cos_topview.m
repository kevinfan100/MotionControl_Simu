% plot_small_angle_cos_topview.m
% View along the measurement y axis (the theta_y rotation axis): a theta_y error e_y is a plain
% 2-D rotation in the x-z plane, and the normal's x-z part has length cos(theta_x), so its tip
% moves cos(theta_x) * e_y. Columns theta_x = 0 | 60 | 90 deg; theta_y = 0, e_y exaggerated.

clear; close all;

FS = 15;  hatc = char(770);  th = ['θ' hatc];  cB = [0 0.25 0.85];
e_y = deg2rad(15);                 % exaggerated for visibility
thx_list = [0 60 90];

fig = figure('Color', 'w', 'Position', [60 80 1300 500]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for ic = 1:numel(thx_list)
    thx = deg2rad(thx_list(ic));
    r = cos(thx);                                  % length of the normal's x-z part
    w_xz  = r * [0 1];                             % true normal, theta_y = 0
    wh_xz = r * [sin(e_y) cos(e_y)];               % after rotating e_y about y
    nexttile;  hold on;  box on;  axis equal;
    tt = linspace(0, 2*pi, 200);
    plot(cos(tt), sin(tt), ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.2);
    h1 = quiver(0, 0, w_xz(1), w_xz(2), 0, 'r', 'LineWidth', 2.5, 'MaxHeadSize', 0.4);
    h2 = quiver(0, 0, wh_xz(1), wh_xz(2), 0, 'Color', cB, 'LineWidth', 2.5, 'MaxHeadSize', 0.4);
    ta = linspace(0, e_y, 40);
    h3 = plot(r * sin(ta), r * cos(ta), 'k', 'LineWidth', 3);
    plot(0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
    xlim([-0.4 1.1]);  ylim([-0.25 1.15]);
    xlabel('x');  if ic == 1; ylabel('z'); end
    legend([h1 h2 h3], {'ŵ', ['ŵ' hatc], sprintf('arc = %.2f e_y', r)}, ...
        'Location', 'northoutside', 'Orientation', 'horizontal');
    text(-0.36, 1.07, sprintf('θ_x = %d\\circ', thx_list(ic)), 'FontSize', FS, 'FontWeight', 'bold');
    set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(gca().Toolbar, 'Visible', 'off');
end

out_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'test_results', 'figs_unused');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
out_png = fullfile(out_dir, 'fig_small_angle_cos_topview.png');
exportgraphics(fig, out_png, 'Resolution', 150);
fprintf('saved %s\n', out_png);
