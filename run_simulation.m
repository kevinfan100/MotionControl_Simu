%% run_simulation.m - Main Simulation Script
%
% This script runs the complete motion control simulation including:
% - Trajectory generation
% - Control force computation
% - Thermal force disturbance
% - Particle dynamics with Wall Effect
%
% The simulation uses a hybrid discrete-continuous approach:
% - Discrete: Trajectory, Controller, Thermal Force (Ts = 1/1606 sec)
% - Continuous: Particle dynamics (ODE solver with ZOH inputs)

clear; close all; clc;

%% SECTION 1: High-Level Parameter Configuration
% === Wall Parameters ===
theta = 0;          % Azimuth angle [rad]
phi = 0;            % Elevation angle [rad]
pz = 0;             % Wall displacement [um]
h_bar_min = 2;      % Minimum safe normalized distance

% === Trajectory Parameters ===
traj_type = 'xy_circle';   % 'z_move' or 'xy_circle'
h_margin = 10;           % Safety margin [um]
delta_z = 5;            % z_move: travel distance [um]
direction = 'toward';   % z_move: 'away' or 'toward'
speed = 50;              % z_move: travel speed [um/sec]
radius = 10;             % xy_circle: radius [um]
period = 0.1;             % xy_circle: period [sec]
n_circles = 3;          % xy_circle: number of circles

% === Controller Parameters ===
ctrl_enable = true;    % true = closed-loop, false = open-loop
lambda_c = 0.4;         % Closed-loop pole (0 < lambda_c < 1)

% === Thermal Force ===
thermal_enable = true;  % Enable Brownian motion disturbance

% === Simulation Parameters ===
T_margin = 0.5;         % Buffer time after trajectory completes [sec]

% Auto-calculate simulation time based on trajectory
if strcmp(traj_type, 'z_move')
    T_traj = delta_z / speed;
else  % xy_circle
    T_traj = period * n_circles;
end
T_sim = T_traj + T_margin;

% === Figure Style Settings ===
colors = [
    0.0000, 0.4470, 0.7410;  % Blue
    0.8500, 0.3250, 0.0980;  % Orange
    0.9290, 0.6940, 0.1250;  % Yellow
    0.4940, 0.1840, 0.5560;  % Purple
    0.4660, 0.6740, 0.1880;  % Green
    0.3010, 0.7450, 0.9330;  % Cyan
];
axis_linewidth = 1.5;
xlabel_fontsize = 14;
ylabel_fontsize = 14;
title_fontsize = 16;
tick_fontsize = 12;
legend_fontsize = 11;
line_width_main = 2.0;
line_width_ref = 1.5;

%% SECTION 2: Package Configuration and Calculate Parameters
config = struct(...
    'theta', theta, 'phi', phi, 'pz', pz, 'h_bar_min', h_bar_min, ...
    'traj_type', traj_type, 'h_margin', h_margin, ...
    'delta_z', delta_z, 'direction', direction, 'speed', speed, ...
    'radius', radius, 'period', period, 'n_circles', n_circles, ...
    'ctrl_enable', ctrl_enable, 'lambda_c', lambda_c, ...
    'thermal_enable', thermal_enable, 'T_sim', T_sim ...
);

% Add paths
addpath('model');
addpath('model/wall_effect');
addpath('model/thermal_force');
addpath('model/trajectory');
addpath('model/controller');

% Calculate simulation parameters
params = calc_simulation_params(config);

%% SECTION 3: Calculate Initial Position and Safety Check
p0 = calc_initial_position(params.Value);

fprintf('Initial position: [%.3f, %.3f, %.3f] um\n', p0);
fprintf('Initial h/R: %.2f\n', (dot(p0, params.Value.wall.w_hat) - params.Value.wall.pz) / params.Value.common.R);

% Safety check
[is_safe, h_bar_min_actual, t_critical] = check_trajectory_safety(p0, params.Value);

if ~is_safe
    warning('Trajectory unsafe! Min h/R = %.2f at t = %.3f sec', h_bar_min_actual, t_critical);
    fprintf('Continuing anyway for demonstration...\n');
end

%% SECTION 4: Run Simulation
fprintf('\nStarting simulation...\n');
ctrl_mode_str = 'Open-loop';
if params.Value.ctrl.enable > 0.5
    ctrl_mode_str = 'Closed-loop';
end
thermal_str = 'Disabled';
if params.Value.thermal.enable > 0.5
    thermal_str = 'Enabled';
end
traj_type_str = 'z_move';
if params.Value.traj.type > 0.5
    traj_type_str = 'xy_circle';
end
fprintf('  Mode: %s\n', ctrl_mode_str);
fprintf('  Thermal: %s\n', thermal_str);
fprintf('  Trajectory: %s\n', traj_type_str);
fprintf('  Duration: %.1f sec\n', T_sim);

% Assign parameters to base workspace for Simulink
assignin('base', 'params', params);
assignin('base', 'p0', p0);

% Run Simulink model
simOut = sim('model/system_model', 'StopTime', num2str(T_sim), ...
    'SaveTime', 'on', 'TimeSaveName', 'tout', ...
    'SaveOutput', 'on', 'OutputSaveName', 'yout');

% Extract results from simulation output
% Use discrete time axis based on controller sample rate (Ts)
% p_d, f_d, F_th are discrete outputs (sampled at Ts)
% p_m is continuous output (needs to be resampled at discrete time points)

Ts = params.Value.common.Ts;
N_discrete = size(simOut.p_d_out, 1);
t_sample = (0:(N_discrete-1)) * Ts;

% Extract discrete outputs [N x 3] -> [3 x N]
p_d_log = simOut.p_d_out';
f_d_log = simOut.f_d_out';
F_th_log = simOut.F_th_out';

% Resample continuous p_m at discrete time points
t_cont = simOut.tout;
p_m_cont = simOut.p_m_out;
p_m_log = zeros(3, N_discrete);

for i = 1:N_discrete
    [~, idx] = min(abs(t_cont - t_sample(i)));
    p_m_log(:, i) = p_m_cont(idx, :)';
end

% Calculate h_bar from p_m
w_hat = params.Value.wall.w_hat;
pz_wall = params.Value.wall.pz;
R = params.Value.common.R;
h_bar_log = (p_m_log' * w_hat - pz_wall) / R;
h_bar_log = h_bar_log';

% Calculate tracking error
error = vecnorm(p_m_log - p_d_log, 2, 1);

N_samples = length(t_sample);

fprintf('Simulation completed.\n');

%% SECTION 5: Results Visualization (Tabbed Figure)
fprintf('\nGenerating figures...\n');

% Create tabbed figure
fig = uifigure('Name', 'Simulation Results', 'Position', [100 100 1200 800]);
tabgroup = uitabgroup(fig);
tabgroup.Units = 'normalized';
tabgroup.Position = [0 0 1 1];

% ==================== Tab 1: 3D Trajectory ====================
tab1 = uitab(tabgroup, 'Title', '3D Trajectory');
ax1 = uiaxes(tab1);
ax1.Units = 'normalized';
ax1.Position = [0.08 0.10 0.88 0.82];

plot3(ax1, p_d_log(1, :), p_d_log(2, :), p_d_log(3, :), 'b-', 'LineWidth', line_width_main);
hold(ax1, 'on');
plot3(ax1, p_m_log(1, :), p_m_log(2, :), p_m_log(3, :), 'r--', 'LineWidth', line_width_ref);
plot3(ax1, p0(1), p0(2), p0(3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);
hold(ax1, 'off');

legend(ax1, {'p_d (desired)', 'p_m (measured)', 'Start'}, ...
    'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
xlabel(ax1, 'x [um]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax1, 'y [um]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
zlabel(ax1, 'z [um]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax1, sprintf('3D Trajectory (%s)', traj_type_str), 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax1.LineWidth = axis_linewidth;
ax1.FontSize = tick_fontsize;
ax1.FontWeight = 'bold';
ax1.Box = 'on';
grid(ax1, 'on');
view(ax1, 30, 30);
axis(ax1, 'equal');

fprintf('  Tab 1: 3D Trajectory\n');

% ==================== Tab 2: Tracking Error ====================
tab2 = uitab(tabgroup, 'Title', 'Tracking Error');
ax2 = uiaxes(tab2);
ax2.Units = 'normalized';
ax2.Position = [0.08 0.10 0.88 0.82];

plot(ax2, t_sample, error * 1000, 'k-', 'LineWidth', line_width_main);

xlabel(ax2, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax2, '||e|| [nm]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax2, sprintf('Tracking Error (RMSE = %.4f um = %.2f nm)', rms(error), rms(error)*1000), ...
    'FontSize', title_fontsize, 'FontWeight', 'bold');
ax2.LineWidth = axis_linewidth;
ax2.FontSize = tick_fontsize;
ax2.FontWeight = 'bold';
ax2.Box = 'on';
grid(ax2, 'on');

% Add statistics annotation
stats_str = sprintf('Max: %.2f nm\nMean: %.2f nm\nFinal: %.2f nm', ...
    max(error)*1000, mean(error)*1000, error(end)*1000);
text(ax2, 0.95, 0.95, stats_str, 'Units', 'normalized', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontSize', legend_fontsize, 'FontWeight', 'bold', ...
    'BackgroundColor', [1 1 1 0.8], 'EdgeColor', [0.3 0.3 0.3], 'Margin', 5);

fprintf('  Tab 2: Tracking Error\n');

% ==================== Tab 3: Position vs Time ====================
tab3 = uitab(tabgroup, 'Title', 'Position');
ax3 = uiaxes(tab3);
ax3.Units = 'normalized';
ax3.Position = [0.08 0.10 0.88 0.82];

plot(ax3, t_sample, p_d_log(1, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_ref);
hold(ax3, 'on');
plot(ax3, t_sample, p_d_log(2, :), '-', 'Color', colors(2, :), 'LineWidth', line_width_ref);
plot(ax3, t_sample, p_d_log(3, :), '-', 'Color', colors(3, :), 'LineWidth', line_width_ref);
plot(ax3, t_sample, p_m_log(1, :), '--', 'Color', colors(1, :), 'LineWidth', line_width_main);
plot(ax3, t_sample, p_m_log(2, :), '--', 'Color', colors(2, :), 'LineWidth', line_width_main);
plot(ax3, t_sample, p_m_log(3, :), '--', 'Color', colors(3, :), 'LineWidth', line_width_main);
hold(ax3, 'off');

legend(ax3, {'x_d', 'y_d', 'z_d', 'x_m', 'y_m', 'z_m'}, ...
    'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold', 'NumColumns', 2);
xlabel(ax3, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax3, 'Position [um]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax3, 'Position vs Time', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax3.LineWidth = axis_linewidth;
ax3.FontSize = tick_fontsize;
ax3.FontWeight = 'bold';
ax3.Box = 'on';
grid(ax3, 'on');

fprintf('  Tab 3: Position\n');

% ==================== Tab 4: Control Force ====================
tab4 = uitab(tabgroup, 'Title', 'Control Force');
ax4 = uiaxes(tab4);
ax4.Units = 'normalized';
ax4.Position = [0.08 0.10 0.88 0.82];

plot(ax4, t_sample, f_d_log(1, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_main);
hold(ax4, 'on');
plot(ax4, t_sample, f_d_log(2, :), '-', 'Color', colors(2, :), 'LineWidth', line_width_main);
plot(ax4, t_sample, f_d_log(3, :), '-', 'Color', colors(3, :), 'LineWidth', line_width_main);
hold(ax4, 'off');

legend(ax4, {'f_x', 'f_y', 'f_z'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
xlabel(ax4, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax4, 'Force [pN]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax4, sprintf('Control Force (%s)', ctrl_mode_str), 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax4.LineWidth = axis_linewidth;
ax4.FontSize = tick_fontsize;
ax4.FontWeight = 'bold';
ax4.Box = 'on';
grid(ax4, 'on');

% Add statistics
f_max = max(abs(f_d_log), [], 2);
stats_str = sprintf('Max |f_x|: %.4f pN\nMax |f_y|: %.4f pN\nMax |f_z|: %.4f pN', f_max(1), f_max(2), f_max(3));
text(ax4, 0.95, 0.95, stats_str, 'Units', 'normalized', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontSize', legend_fontsize, 'FontWeight', 'bold', ...
    'BackgroundColor', [1 1 1 0.8], 'EdgeColor', [0.3 0.3 0.3], 'Margin', 5);

fprintf('  Tab 4: Control Force\n');

% ==================== Tab 5: Thermal Force ====================
tab5 = uitab(tabgroup, 'Title', 'Thermal Force');
ax5 = uiaxes(tab5);
ax5.Units = 'normalized';
ax5.Position = [0.08 0.10 0.88 0.82];

plot(ax5, t_sample, F_th_log(1, :), '-', 'Color', colors(1, :), 'LineWidth', 1.0);
hold(ax5, 'on');
plot(ax5, t_sample, F_th_log(2, :), '-', 'Color', colors(2, :), 'LineWidth', 1.0);
plot(ax5, t_sample, F_th_log(3, :), '-', 'Color', colors(3, :), 'LineWidth', 1.0);
hold(ax5, 'off');

legend(ax5, {'F_{th,x}', 'F_{th,y}', 'F_{th,z}'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
xlabel(ax5, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax5, 'Force [pN]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax5, sprintf('Thermal Force (Brownian Motion) - %s', thermal_str), 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax5.LineWidth = axis_linewidth;
ax5.FontSize = tick_fontsize;
ax5.FontWeight = 'bold';
ax5.Box = 'on';
grid(ax5, 'on');

% Add statistics
F_std = std(F_th_log, 0, 2);
stats_str = sprintf('Std F_x: %.4f pN\nStd F_y: %.4f pN\nStd F_z: %.4f pN', F_std(1), F_std(2), F_std(3));
text(ax5, 0.95, 0.95, stats_str, 'Units', 'normalized', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontSize', legend_fontsize, 'FontWeight', 'bold', ...
    'BackgroundColor', [1 1 1 0.8], 'EdgeColor', [0.3 0.3 0.3], 'Margin', 5);

fprintf('  Tab 5: Thermal Force\n');

% ==================== Tab 6: Wall Distance (h/R) ====================
tab6 = uitab(tabgroup, 'Title', 'Wall Distance');
ax6 = uiaxes(tab6);
ax6.Units = 'normalized';
ax6.Position = [0.08 0.10 0.88 0.82];

plot(ax6, t_sample, h_bar_log, 'b-', 'LineWidth', line_width_main);
hold(ax6, 'on');
yline(ax6, params.Value.wall.h_bar_min, 'r--', 'LineWidth', line_width_ref);
hold(ax6, 'off');

legend(ax6, {'h/R(t)', sprintf('h/R_{min}=%.1f', params.Value.wall.h_bar_min)}, ...
    'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
xlabel(ax6, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax6, 'h/R', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax6, 'Normalized Distance from Wall', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax6.LineWidth = axis_linewidth;
ax6.FontSize = tick_fontsize;
ax6.FontWeight = 'bold';
ax6.Box = 'on';
grid(ax6, 'on');

% Add safety status
if min(h_bar_log) >= params.Value.wall.h_bar_min
    safety_str = 'SAFE';
    safety_color = [0 0.5 0];
else
    safety_str = 'UNSAFE';
    safety_color = [0.8 0 0];
end
stats_str = sprintf('Status: %s\nMin h/R: %.2f\nMax h/R: %.2f', safety_str, min(h_bar_log), max(h_bar_log));
text(ax6, 0.95, 0.95, stats_str, 'Units', 'normalized', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontSize', legend_fontsize, 'FontWeight', 'bold', 'Color', safety_color, ...
    'BackgroundColor', [1 1 1 0.8], 'EdgeColor', [0.3 0.3 0.3], 'Margin', 5);

fprintf('  Tab 6: Wall Distance\n');

%% SECTION 6: Save Results
fprintf('\nSaving results...\n');

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
output_dir = fullfile('test_results', 'simulation', ['sim_' timestamp]);
mkdir(output_dir);

% Save individual figures as PNG (using traditional figure for export)
fig_export = figure('Visible', 'off', 'Position', [100 100 1000 700]);

% Export Tab 1: 3D Trajectory
ax_exp = axes(fig_export, 'Position', [0.12 0.12 0.82 0.80]);
plot3(ax_exp, p_d_log(1, :), p_d_log(2, :), p_d_log(3, :), 'b-', 'LineWidth', line_width_main);
hold(ax_exp, 'on');
plot3(ax_exp, p_m_log(1, :), p_m_log(2, :), p_m_log(3, :), 'r--', 'LineWidth', line_width_ref);
plot3(ax_exp, p0(1), p0(2), p0(3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2);
hold(ax_exp, 'off');
legend(ax_exp, {'p_d (desired)', 'p_m (measured)', 'Start'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
xlabel(ax_exp, 'x [um]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax_exp, 'y [um]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
zlabel(ax_exp, 'z [um]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax_exp, sprintf('3D Trajectory (%s)', traj_type_str), 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax_exp.LineWidth = axis_linewidth; ax_exp.FontSize = tick_fontsize; ax_exp.FontWeight = 'bold'; ax_exp.Box = 'on'; grid(ax_exp, 'on');
view(ax_exp, 30, 30);
axis(ax_exp, 'equal');
exportgraphics(fig_export, fullfile(output_dir, '1_trajectory_3d.png'), 'Resolution', 150);
clf(fig_export);

% Export Tab 2: Tracking Error
ax_exp = axes(fig_export, 'Position', [0.12 0.12 0.82 0.80]);
plot(ax_exp, t_sample, error * 1000, 'k-', 'LineWidth', line_width_main);
xlabel(ax_exp, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax_exp, '||e|| [nm]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax_exp, sprintf('Tracking Error (RMSE = %.2f nm)', rms(error)*1000), 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax_exp.LineWidth = axis_linewidth; ax_exp.FontSize = tick_fontsize; ax_exp.FontWeight = 'bold'; ax_exp.Box = 'on'; grid(ax_exp, 'on');
exportgraphics(fig_export, fullfile(output_dir, '2_tracking_error.png'), 'Resolution', 150);
clf(fig_export);

% Export Tab 3: Position
ax_exp = axes(fig_export, 'Position', [0.12 0.12 0.82 0.80]);
plot(ax_exp, t_sample, p_m_log(1, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_main); hold(ax_exp, 'on');
plot(ax_exp, t_sample, p_m_log(2, :), '-', 'Color', colors(2, :), 'LineWidth', line_width_main);
plot(ax_exp, t_sample, p_m_log(3, :), '-', 'Color', colors(3, :), 'LineWidth', line_width_main);
plot(ax_exp, t_sample, p_d_log(3, :), '--k', 'LineWidth', line_width_ref); hold(ax_exp, 'off');
legend(ax_exp, {'x', 'y', 'z', 'z_d'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
xlabel(ax_exp, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax_exp, 'Position [um]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax_exp, 'Position vs Time', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax_exp.LineWidth = axis_linewidth; ax_exp.FontSize = tick_fontsize; ax_exp.FontWeight = 'bold'; ax_exp.Box = 'on'; grid(ax_exp, 'on');
exportgraphics(fig_export, fullfile(output_dir, '3_position.png'), 'Resolution', 150);
clf(fig_export);

% Export Tab 4: Control Force
ax_exp = axes(fig_export, 'Position', [0.12 0.12 0.82 0.80]);
plot(ax_exp, t_sample, f_d_log(1, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_main); hold(ax_exp, 'on');
plot(ax_exp, t_sample, f_d_log(2, :), '-', 'Color', colors(2, :), 'LineWidth', line_width_main);
plot(ax_exp, t_sample, f_d_log(3, :), '-', 'Color', colors(3, :), 'LineWidth', line_width_main); hold(ax_exp, 'off');
legend(ax_exp, {'f_x', 'f_y', 'f_z'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
xlabel(ax_exp, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax_exp, 'Force [pN]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax_exp, sprintf('Control Force (%s)', ctrl_mode_str), 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax_exp.LineWidth = axis_linewidth; ax_exp.FontSize = tick_fontsize; ax_exp.FontWeight = 'bold'; ax_exp.Box = 'on'; grid(ax_exp, 'on');
exportgraphics(fig_export, fullfile(output_dir, '4_control_force.png'), 'Resolution', 150);
clf(fig_export);

% Export Tab 5: Thermal Force
ax_exp = axes(fig_export, 'Position', [0.12 0.12 0.82 0.80]);
plot(ax_exp, t_sample, F_th_log(1, :), '-', 'Color', colors(1, :), 'LineWidth', 1.0); hold(ax_exp, 'on');
plot(ax_exp, t_sample, F_th_log(2, :), '-', 'Color', colors(2, :), 'LineWidth', 1.0);
plot(ax_exp, t_sample, F_th_log(3, :), '-', 'Color', colors(3, :), 'LineWidth', 1.0); hold(ax_exp, 'off');
legend(ax_exp, {'F_{th,x}', 'F_{th,y}', 'F_{th,z}'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
xlabel(ax_exp, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax_exp, 'Force [pN]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax_exp, sprintf('Thermal Force - %s', thermal_str), 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax_exp.LineWidth = axis_linewidth; ax_exp.FontSize = tick_fontsize; ax_exp.FontWeight = 'bold'; ax_exp.Box = 'on'; grid(ax_exp, 'on');
exportgraphics(fig_export, fullfile(output_dir, '5_thermal_force.png'), 'Resolution', 150);
clf(fig_export);

% Export Tab 6: Wall Distance
ax_exp = axes(fig_export, 'Position', [0.12 0.12 0.82 0.80]);
plot(ax_exp, t_sample, h_bar_log, 'b-', 'LineWidth', line_width_main); hold(ax_exp, 'on');
yline(ax_exp, params.Value.wall.h_bar_min, 'r--', 'LineWidth', line_width_ref); hold(ax_exp, 'off');
legend(ax_exp, {'h/R(t)', sprintf('h/R_{min}=%.1f', params.Value.wall.h_bar_min)}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
xlabel(ax_exp, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax_exp, 'h/R', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax_exp, 'Normalized Distance from Wall', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax_exp.LineWidth = axis_linewidth; ax_exp.FontSize = tick_fontsize; ax_exp.FontWeight = 'bold'; ax_exp.Box = 'on'; grid(ax_exp, 'on');
exportgraphics(fig_export, fullfile(output_dir, '6_wall_distance.png'), 'Resolution', 150);

close(fig_export);
fprintf('  Figures saved (.png)\n');

% Save data
result.t = t_sample;
result.p_m = p_m_log;
result.p_d = p_d_log;
result.f_d = f_d_log;
result.F_th = F_th_log;
result.h_bar = h_bar_log;
result.error = error;
result.params = params.Value;
result.p0 = p0;
result.config = config;
result.tracking_error_rmse = rms(error);
result.meta.timestamp = datestr(now);
result.meta.ctrl_mode = ctrl_mode_str;
result.meta.thermal_mode = thermal_str;
result.meta.traj_type = traj_type_str;

save(fullfile(output_dir, 'result.mat'), 'result');
fprintf('  Data saved (.mat)\n');

%% Print Summary
fprintf('\n');
fprintf('================================================================\n');
fprintf('                    Simulation Summary\n');
fprintf('================================================================\n');
fprintf('\n');
fprintf('Configuration:\n');
fprintf('  Mode: %s\n', ctrl_mode_str);
fprintf('  Thermal: %s\n', thermal_str);
fprintf('  Trajectory: %s\n', traj_type_str);
fprintf('  Duration: %.1f sec (%d samples)\n', T_sim, N_samples);
fprintf('\n');
fprintf('Tracking Performance:\n');
fprintf('  RMSE: %.4f um (%.2f nm)\n', rms(error), rms(error)*1000);
fprintf('  Max error: %.4f um (%.2f nm)\n', max(error), max(error)*1000);
fprintf('  Final error: %.4f um (%.2f nm)\n', error(end), error(end)*1000);
fprintf('\n');
fprintf('Safety:\n');
fprintf('  Status: %s\n', safety_str);
fprintf('  Min h/R: %.2f (threshold: %.1f)\n', min(h_bar_log), params.Value.wall.h_bar_min);
fprintf('  Max h/R: %.2f\n', max(h_bar_log));
fprintf('\n');
fprintf('Control Force:\n');
fprintf('  Max |f_z|: %.4f pN\n', max(abs(f_d_log(3, :))));
fprintf('  Mean |f_z|: %.4f pN\n', mean(abs(f_d_log(3, :))));
fprintf('\n');
fprintf('Results saved to: %s\n', output_dir);
fprintf('================================================================\n');
