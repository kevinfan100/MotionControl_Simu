%% run_trajectory_test.m - Trajectory Generator Module Validation Test
%
% This script validates the Trajectory Generator module by:
% 1. Visualizing z_move trajectory in 3D with wall plane
% 2. Visualizing xy_circle trajectory in 3D with wall plane
% 3. Showing h_bar(t) safety curves for both trajectories
%
% Output: 3 PNG files
%   1. z_move_3d.png - 3D trajectory with wall plane
%   2. xy_circle_3d.png - 3D trajectory with wall plane
%   3. h_bar_safety.png - h_bar vs time for both trajectories

clear; close all; clc;

%% Add paths (relative to project root)
[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'thermal_force'));
addpath(fullfile(project_root, 'model', 'trajectory'));
cd(project_root);

%% Test parameters
% Wall parameters (simple case: wall parallel to xy plane)
theta = 0;
phi = 0;
pz = 0;

% Common simulation parameters
T_sim = 5;

% z_move parameters
z_config = struct(...
    'theta', theta, 'phi', phi, 'pz', pz, 'h_bar_min', 1.5, ...
    'traj_type', 'z_move', 'h_margin', 5, ...
    'delta_z', 10, 'direction', 'away', 'speed', 5, ...
    'radius', 5, 'period', 1, 'n_circles', 3, ...
    'lambda_c', 0.7, 'thermal_enable', false, 'T_sim', T_sim ...
);

% xy_circle parameters
xy_config = struct(...
    'theta', theta, 'phi', phi, 'pz', pz, 'h_bar_min', 1.5, ...
    'traj_type', 'xy_circle', 'h_margin', 5, ...
    'delta_z', 10, 'direction', 'away', 'speed', 5, ...
    'radius', 5, 'period', 1, 'n_circles', 3, ...
    'lambda_c', 0.7, 'thermal_enable', false, 'T_sim', T_sim ...
);

% Get params
params_z_slx = calc_simulation_params(z_config);
params_xy_slx = calc_simulation_params(xy_config);
params_z = params_z_slx.Value;
params_xy = params_xy_slx.Value;

%% Generate trajectories
Ts = params_z.common.Ts;
t_vec = 0:Ts:T_sim;
N = length(t_vec);

% Calculate initial positions
p0_z = calc_initial_position(params_z);
p0_xy = calc_initial_position(params_xy);

% Generate z_move trajectory
traj_z = zeros(3, N);
h_bar_z = zeros(1, N);
for i = 1:N
    traj_z(:, i) = trajectory_generator(t_vec(i), p0_z, params_z);
    h = dot(traj_z(:, i), params_z.wall.w_hat) - params_z.wall.pz;
    h_bar_z(i) = h / params_z.common.R;
end

% Generate xy_circle trajectory
traj_xy = zeros(3, N);
h_bar_xy = zeros(1, N);
for i = 1:N
    traj_xy(:, i) = trajectory_generator(t_vec(i), p0_xy, params_xy);
    h = dot(traj_xy(:, i), params_xy.wall.w_hat) - params_xy.wall.pz;
    h_bar_xy(i) = h / params_xy.common.R;
end

%% Safety check
[is_safe_z, h_bar_min_z, t_crit_z] = check_trajectory_safety(p0_z, params_z);
[is_safe_xy, h_bar_min_xy, t_crit_xy] = check_trajectory_safety(p0_xy, params_xy);

%% Figure style settings
axis_linewidth = 1.5;
xlabel_fontsize = 14;
ylabel_fontsize = 14;
title_fontsize = 16;
tick_fontsize = 12;
legend_fontsize = 11;
line_width = 2;

%% Create output directory
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
output_dir = fullfile('test_results', 'trajectory', ['test_' timestamp]);
mkdir(output_dir);

%% Helper function for wall plane
wall_half_size = 15;  % Half size of wall plane visualization

%% Figure 1: z_move 3D Trajectory
fig1 = figure('Position', [100 100 800 600], 'Visible', 'off');

% Plot trajectory
plot3(traj_z(1, :), traj_z(2, :), traj_z(3, :), 'b-', 'LineWidth', line_width);
hold on;

% Plot start point (green circle)
plot3(p0_z(1), p0_z(2), p0_z(3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);

% Plot end point (red square)
plot3(traj_z(1, end), traj_z(2, end), traj_z(3, end), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 2);

% Draw wall plane (at z=0 for this simple case)
[X_wall, Y_wall] = meshgrid(linspace(-wall_half_size, wall_half_size, 2));
Z_wall = zeros(size(X_wall));
surf(X_wall, Y_wall, Z_wall, 'FaceColor', [0.8 0.8 0.8], 'FaceAlpha', 0.3, 'EdgeColor', 'none');

% Draw coordinate axes from start point
arrow_scale = 3;
w_hat = params_z.wall.w_hat;
u_hat = params_z.wall.u_hat;
v_hat = params_z.wall.v_hat;
quiver3(p0_z(1), p0_z(2), p0_z(3), w_hat(1), w_hat(2), w_hat(3), arrow_scale, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.8);
quiver3(p0_z(1), p0_z(2), p0_z(3), u_hat(1), u_hat(2), u_hat(3), arrow_scale, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.8);
quiver3(p0_z(1), p0_z(2), p0_z(3), v_hat(1), v_hat(2), v_hat(3), arrow_scale, 'Color', [0 0.5 1], 'LineWidth', 2, 'MaxHeadSize', 0.8);

hold off;

xlabel('x [um]', 'FontSize', xlabel_fontsize);
ylabel('y [um]', 'FontSize', ylabel_fontsize);
zlabel('z [um]', 'FontSize', ylabel_fontsize);
title('z\_move Trajectory (direction: away)', 'FontSize', title_fontsize);
legend({'Trajectory', 'Start', 'End', 'Wall', 'w (normal)', 'u (parallel)', 'v (parallel)'}, ...
    'Location', 'northeast', 'FontSize', legend_fontsize);
grid on;
axis equal;
view(30, 30);
set(gca, 'FontSize', tick_fontsize, 'LineWidth', axis_linewidth);
xlim([-wall_half_size wall_half_size]);
ylim([-wall_half_size wall_half_size]);
zlim([0 25]);

exportgraphics(fig1, fullfile(output_dir, 'z_move_3d.png'), 'Resolution', 150);
close(fig1);

%% Figure 2: xy_circle 3D Trajectory
fig2 = figure('Position', [100 100 800 600], 'Visible', 'off');

% Plot trajectory
plot3(traj_xy(1, :), traj_xy(2, :), traj_xy(3, :), 'b-', 'LineWidth', line_width);
hold on;

% Plot start point (green circle)
plot3(p0_xy(1), p0_xy(2), p0_xy(3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);

% Draw wall plane
surf(X_wall, Y_wall, Z_wall, 'FaceColor', [0.8 0.8 0.8], 'FaceAlpha', 0.3, 'EdgeColor', 'none');

% Draw coordinate axes from start point
quiver3(p0_xy(1), p0_xy(2), p0_xy(3), w_hat(1), w_hat(2), w_hat(3), arrow_scale, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.8);
quiver3(p0_xy(1), p0_xy(2), p0_xy(3), u_hat(1), u_hat(2), u_hat(3), arrow_scale, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.8);
quiver3(p0_xy(1), p0_xy(2), p0_xy(3), v_hat(1), v_hat(2), v_hat(3), arrow_scale, 'Color', [0 0.5 1], 'LineWidth', 2, 'MaxHeadSize', 0.8);

hold off;

xlabel('x [um]', 'FontSize', xlabel_fontsize);
ylabel('y [um]', 'FontSize', ylabel_fontsize);
zlabel('z [um]', 'FontSize', ylabel_fontsize);
title(sprintf('xy\\_circle Trajectory (radius=%.1f um, %d circles)', params_xy.traj.radius, params_xy.traj.n_circles), ...
    'FontSize', title_fontsize);
legend({'Trajectory', 'Start', 'Wall', 'w (normal)', 'u (parallel)', 'v (parallel)'}, ...
    'Location', 'northeast', 'FontSize', legend_fontsize);
grid on;
axis equal;
view(30, 30);
set(gca, 'FontSize', tick_fontsize, 'LineWidth', axis_linewidth);
xlim([-wall_half_size wall_half_size]);
ylim([-wall_half_size wall_half_size]);
zlim([0 15]);

exportgraphics(fig2, fullfile(output_dir, 'xy_circle_3d.png'), 'Resolution', 150);
close(fig2);

%% Figure 3: h_bar Safety Curves
fig3 = figure('Position', [100 100 900 600], 'Visible', 'off');

% Subplot 1: z_move h_bar
subplot(2, 1, 1);
plot(t_vec, h_bar_z, 'b-', 'LineWidth', line_width);
hold on;
yline(params_z.wall.h_bar_min, 'r--', 'LineWidth', 1.5);
hold off;
xlabel('Time [sec]', 'FontSize', xlabel_fontsize);
ylabel('h/R', 'FontSize', ylabel_fontsize);
title(sprintf('z\\_move: h/R vs Time (min=%.2f, safe=%d)', h_bar_min_z, is_safe_z), 'FontSize', title_fontsize);
legend({'h/R(t)', sprintf('h/R_{min}=%.1f', params_z.wall.h_bar_min)}, ...
    'Location', 'northeast', 'FontSize', legend_fontsize);
grid on;
set(gca, 'FontSize', tick_fontsize, 'LineWidth', axis_linewidth);
ylim([0 max(h_bar_z)*1.1]);

% Subplot 2: xy_circle h_bar
subplot(2, 1, 2);
plot(t_vec, h_bar_xy, 'b-', 'LineWidth', line_width);
hold on;
yline(params_xy.wall.h_bar_min, 'r--', 'LineWidth', 1.5);
hold off;
xlabel('Time [sec]', 'FontSize', xlabel_fontsize);
ylabel('h/R', 'FontSize', ylabel_fontsize);
title(sprintf('xy\\_circle: h/R vs Time (min=%.2f, safe=%d)', h_bar_min_xy, is_safe_xy), 'FontSize', title_fontsize);
legend({'h/R(t)', sprintf('h/R_{min}=%.1f', params_xy.wall.h_bar_min)}, ...
    'Location', 'northeast', 'FontSize', legend_fontsize);
grid on;
set(gca, 'FontSize', tick_fontsize, 'LineWidth', axis_linewidth);
ylim([0 max(h_bar_xy)*1.1]);

exportgraphics(fig3, fullfile(output_dir, 'h_bar_safety.png'), 'Resolution', 150);
close(fig3);

%% Save result data
result.t_vec = t_vec;
result.traj_z = traj_z;
result.traj_xy = traj_xy;
result.h_bar_z = h_bar_z;
result.h_bar_xy = h_bar_xy;
result.p0_z = p0_z;
result.p0_xy = p0_xy;
result.params_z = params_z;
result.params_xy = params_xy;
result.safety.is_safe_z = is_safe_z;
result.safety.is_safe_xy = is_safe_xy;
result.safety.h_bar_min_z = h_bar_min_z;
result.safety.h_bar_min_xy = h_bar_min_xy;

save(fullfile(output_dir, 'result.mat'), 'result');

%% Print summary
fprintf('\n=== Trajectory Test Summary ===\n');
fprintf('Simulation time: T_sim = %.1f sec\n', T_sim);
fprintf('Sampling rate: fs = %.0f Hz\n', 1/Ts);

fprintf('\nz_move trajectory:\n');
fprintf('  Start: [%.2f, %.2f, %.2f] um\n', p0_z);
fprintf('  End:   [%.2f, %.2f, %.2f] um\n', traj_z(:, end));
fprintf('  Displacement: %.2f um\n', norm(traj_z(:, end) - p0_z));
dir_str = 'away';
if params_z.traj.direction > 0.5
    dir_str = 'toward';
end
fprintf('  Direction: %s\n', dir_str);
fprintf('  Safety: %s (min h/R = %.2f)\n', yesno(is_safe_z), h_bar_min_z);

fprintf('\nxy_circle trajectory:\n');
fprintf('  Center: [%.2f, %.2f, %.2f] um\n', p0_xy);
fprintf('  Radius: %.2f um\n', params_xy.traj.radius);
fprintf('  Circles: %d (period=%.2f sec)\n', params_xy.traj.n_circles, params_xy.traj.period);
fprintf('  Safety: %s (min h/R = %.2f)\n', yesno(is_safe_xy), h_bar_min_xy);

fprintf('\nResults saved to: %s\n', output_dir);

function s = yesno(b)
    if b
        s = 'SAFE';
    else
        s = 'UNSAFE';
    end
end
