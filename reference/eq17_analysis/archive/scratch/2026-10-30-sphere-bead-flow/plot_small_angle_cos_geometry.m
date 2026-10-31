% plot_small_angle_cos_geometry.m
% Pure-geometry check of the small-angle form of V_hat' * V (1026 Sec. 1-2):
% no particle, no noise, no filter. Exact rotation products vs the
% first-order a13 = -(a_par - a_perp) cos(theta_hat_x) e_y, with and without cos.

clear; close all;

rx = @(t) [1 0 0; 0 cos(t) -sin(t); 0 sin(t) cos(t)];
ry = @(t) [cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)];

a_par  = 0.6;            % a_bar parallel (illustrative)
a_perp = 0.3;            % a_bar perpendicular (illustrative)
lam    = diag([a_par a_par a_perp]);
th_y   = deg2rad(20);    % true theta_y (drops out of a13)
e_y    = deg2rad(1);     % theta_y - theta_hat_y, small
thx_deg = 0:1:85;        % theta_hat_x = theta_x (e_x = 0)

a13_exact = zeros(size(thx_deg));
for i = 1:numel(thx_deg)
    thx = deg2rad(thx_deg(i));
    v_hat = ry(th_y - e_y) * rx(thx);
    v_true = ry(th_y) * rx(thx);
    a_meas = v_true * lam * v_true';
    va = v_hat' * a_meas * v_hat;
    a13_exact(i) = va(1, 3);
end
a13_cos   = -(a_par - a_perp) * cosd(thx_deg) * e_y;
a13_nocos = -(a_par - a_perp) * ones(size(thx_deg)) * e_y;

fprintf('theta_hat_x   exact      with cos   without cos\n');
for t = [0 30 60 80]
    i = find(thx_deg == t);
    fprintf('%5d     %9.5f  %9.5f  %9.5f\n', t, a13_exact(i), a13_cos(i), a13_nocos(i));
end

% style of plot_1026_doc_figs.m: default tex font, bold, Unicode a-bar / hat
FS = 15;  hatc = char(770);  th = ['θ' hatc];  abar = char(257);  cB = [0 0.25 0.85];
fig = figure('Color', 'w', 'Position', [60 80 1000 520]);
hold on;  box on;
h1 = plot(thx_deg, a13_exact * 1e3, 'r', 'LineWidth', 2.5);
h2 = plot(thx_deg, a13_cos * 1e3, '--', 'Color', cB, 'LineWidth', 2);
h3 = plot(thx_deg, a13_nocos * 1e3, 'k:', 'LineWidth', 2);
xlim([0 85]);  ylim([-6 0]);
ylabel([abar '_{13}  [\times10^{-3}]']);  xlabel([th '_x  [\circ]']);
legend([h1 h2 h3], {'exact', ['first order with cos ' th '_x'], ['first order without cos ' th '_x']}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal');
set(gca, 'FontSize', FS, 'FontWeight', 'bold', 'LineWidth', 1.2);  set(gca().Toolbar, 'Visible', 'off');

out_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'test_results', 'figs_unused');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
out_png = fullfile(out_dir, 'fig_small_angle_cos_geometry.png');
exportgraphics(fig, out_png, 'Resolution', 150);
fprintf('saved %s\n', out_png);
