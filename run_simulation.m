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
phi = pi/3;            % Elevation angle [rad]
pz = 2;             % Wall displacement [um]
h_bar_min = 2;      % Minimum safe normalized distance

% === Trajectory Parameters ===
traj_type = 'z_move';   % 'z_move' or 'xy_circle'
h_margin = 15;           % Safety margin [um]
delta_z = 8;            % z_move: travel distance [um]
direction = 'toward';   % z_move: 'away' or 'toward'
speed = 40;              % z_move: travel speed [um/sec]
radius = 5;             % xy_circle: radius [um]
period = 1;             % xy_circle: period [sec]
n_circles = 3;          % xy_circle: number of circles

% === Controller Parameters ===
ctrl_enable = true;    % true = closed-loop, false = open-loop
lambda_c = 0.4;         % Closed-loop pole (0 < lambda_c < 1)

% === Thermal Force ===
thermal_enable = false;  % Enable Brownian motion disturbance

% === Simulation Parameters ===
T_margin = 0.5;         % Buffer time after trajectory completes [sec]

% Auto-calculate simulation time based on trajectory
if strcmp(traj_type, 'z_move')
    T_traj = delta_z / speed;
else  % xy_circle
    T_traj = period * n_circles;
end
T_sim = T_traj + T_margin;

% === Figure Style Settings (r_controller style) ===
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
title_fontsize = 15;
tick_fontsize = 12;
legend_fontsize = 11;
line_width_main = 3.0;
line_width_ref = 2.5;

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

% ==================== Tab 2: X-Axis Analysis ====================
tab2 = uitab(tabgroup, 'Title', 'X-Axis Analysis');

% Calculate X-axis error in nm
error_x = (p_m_log(1, :) - p_d_log(1, :)) * 1000;

% Subplot positions [left bottom width height]
pos_top = [0.10 0.70 0.85 0.25];
pos_mid = [0.10 0.40 0.85 0.25];
pos_bot = [0.10 0.08 0.85 0.25];

% --- Subplot 1: X Position Tracking ---
ax2_pos = uiaxes(tab2, 'Position', [pos_top(1)*1200, pos_top(2)*800, pos_top(3)*1200, pos_top(4)*800]);
plot(ax2_pos, t_sample, p_m_log(1, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_main);
hold(ax2_pos, 'on');
plot(ax2_pos, t_sample, p_d_log(1, :), '--', 'Color', colors(2, :), 'LineWidth', line_width_ref);
hold(ax2_pos, 'off');
legend(ax2_pos, {'p_{m,x}', 'p_{d,x}'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
ylabel(ax2_pos, 'Position [um]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax2_pos, 'X-Axis Position Tracking', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax2_pos.LineWidth = axis_linewidth; ax2_pos.FontSize = tick_fontsize; ax2_pos.FontWeight = 'bold';
ax2_pos.Box = 'on'; grid(ax2_pos, 'on');
ax2_pos.XTickLabel = [];

% --- Subplot 2: X Tracking Error ---
ax2_err = uiaxes(tab2, 'Position', [pos_mid(1)*1200, pos_mid(2)*800, pos_mid(3)*1200, pos_mid(4)*800]);
plot(ax2_err, t_sample, error_x, 'k-', 'LineWidth', line_width_main);
ylabel(ax2_err, 'e_x [nm]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax2_err, 'X-Axis Error', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax2_err.LineWidth = axis_linewidth; ax2_err.FontSize = tick_fontsize; ax2_err.FontWeight = 'bold';
ax2_err.Box = 'on'; grid(ax2_err, 'on');
ax2_err.XTickLabel = [];

% --- Subplot 3: X Control Force ---
ax2_force = uiaxes(tab2, 'Position', [pos_bot(1)*1200, pos_bot(2)*800, pos_bot(3)*1200, pos_bot(4)*800]);
plot(ax2_force, t_sample, f_d_log(1, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_main);
xlabel(ax2_force, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax2_force, 'f_x [pN]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax2_force, 'X-Axis Control Force', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax2_force.LineWidth = axis_linewidth; ax2_force.FontSize = tick_fontsize; ax2_force.FontWeight = 'bold';
ax2_force.Box = 'on'; grid(ax2_force, 'on');

% Link X axes
linkaxes([ax2_pos, ax2_err, ax2_force], 'x');

fprintf('  Tab 2: X-Axis Analysis\n');

% ==================== Tab 3: Y-Axis Analysis ====================
tab3 = uitab(tabgroup, 'Title', 'Y-Axis Analysis');

% Calculate Y-axis error in nm
error_y = (p_m_log(2, :) - p_d_log(2, :)) * 1000;

% --- Subplot 1: Y Position Tracking ---
ax3_pos = uiaxes(tab3, 'Position', [pos_top(1)*1200, pos_top(2)*800, pos_top(3)*1200, pos_top(4)*800]);
plot(ax3_pos, t_sample, p_m_log(2, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_main);
hold(ax3_pos, 'on');
plot(ax3_pos, t_sample, p_d_log(2, :), '--', 'Color', colors(2, :), 'LineWidth', line_width_ref);
hold(ax3_pos, 'off');
legend(ax3_pos, {'p_{m,y}', 'p_{d,y}'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
ylabel(ax3_pos, 'Position [um]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax3_pos, 'Y-Axis Position Tracking', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax3_pos.LineWidth = axis_linewidth; ax3_pos.FontSize = tick_fontsize; ax3_pos.FontWeight = 'bold';
ax3_pos.Box = 'on'; grid(ax3_pos, 'on');
ax3_pos.XTickLabel = [];

% --- Subplot 2: Y Tracking Error ---
ax3_err = uiaxes(tab3, 'Position', [pos_mid(1)*1200, pos_mid(2)*800, pos_mid(3)*1200, pos_mid(4)*800]);
plot(ax3_err, t_sample, error_y, 'k-', 'LineWidth', line_width_main);
ylabel(ax3_err, 'e_y [nm]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax3_err, 'Y-Axis Error', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax3_err.LineWidth = axis_linewidth; ax3_err.FontSize = tick_fontsize; ax3_err.FontWeight = 'bold';
ax3_err.Box = 'on'; grid(ax3_err, 'on');
ax3_err.XTickLabel = [];

% --- Subplot 3: Y Control Force ---
ax3_force = uiaxes(tab3, 'Position', [pos_bot(1)*1200, pos_bot(2)*800, pos_bot(3)*1200, pos_bot(4)*800]);
plot(ax3_force, t_sample, f_d_log(2, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_main);
xlabel(ax3_force, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax3_force, 'f_y [pN]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax3_force, 'Y-Axis Control Force', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax3_force.LineWidth = axis_linewidth; ax3_force.FontSize = tick_fontsize; ax3_force.FontWeight = 'bold';
ax3_force.Box = 'on'; grid(ax3_force, 'on');

% Link X axes
linkaxes([ax3_pos, ax3_err, ax3_force], 'x');

fprintf('  Tab 3: Y-Axis Analysis\n');

% ==================== Tab 4: Z-Axis Analysis ====================
tab4 = uitab(tabgroup, 'Title', 'Z-Axis Analysis');

% Calculate Z-axis error in nm
error_z = (p_m_log(3, :) - p_d_log(3, :)) * 1000;

% --- Subplot 1: Z Position Tracking ---
ax4_pos = uiaxes(tab4, 'Position', [pos_top(1)*1200, pos_top(2)*800, pos_top(3)*1200, pos_top(4)*800]);
plot(ax4_pos, t_sample, p_m_log(3, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_main);
hold(ax4_pos, 'on');
plot(ax4_pos, t_sample, p_d_log(3, :), '--', 'Color', colors(2, :), 'LineWidth', line_width_ref);
hold(ax4_pos, 'off');
legend(ax4_pos, {'p_{m,z}', 'p_{d,z}'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
ylabel(ax4_pos, 'Position [um]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax4_pos, 'Z-Axis Position Tracking', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax4_pos.LineWidth = axis_linewidth; ax4_pos.FontSize = tick_fontsize; ax4_pos.FontWeight = 'bold';
ax4_pos.Box = 'on'; grid(ax4_pos, 'on');
ax4_pos.XTickLabel = [];

% --- Subplot 2: Z Tracking Error ---
ax4_err = uiaxes(tab4, 'Position', [pos_mid(1)*1200, pos_mid(2)*800, pos_mid(3)*1200, pos_mid(4)*800]);
plot(ax4_err, t_sample, error_z, 'k-', 'LineWidth', line_width_main);
ylabel(ax4_err, 'e_z [nm]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax4_err, 'Z-Axis Error', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax4_err.LineWidth = axis_linewidth; ax4_err.FontSize = tick_fontsize; ax4_err.FontWeight = 'bold';
ax4_err.Box = 'on'; grid(ax4_err, 'on');
ax4_err.XTickLabel = [];

% --- Subplot 3: Z Control Force ---
ax4_force = uiaxes(tab4, 'Position', [pos_bot(1)*1200, pos_bot(2)*800, pos_bot(3)*1200, pos_bot(4)*800]);
plot(ax4_force, t_sample, f_d_log(3, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_main);
xlabel(ax4_force, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax4_force, 'f_z [pN]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax4_force, 'Z-Axis Control Force', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax4_force.LineWidth = axis_linewidth; ax4_force.FontSize = tick_fontsize; ax4_force.FontWeight = 'bold';
ax4_force.Box = 'on'; grid(ax4_force, 'on');

% Link X axes
linkaxes([ax4_pos, ax4_err, ax4_force], 'x');

fprintf('  Tab 4: Z-Axis Analysis\n');

% ==================== Tab 5: Position vs Time ====================
tab5 = uitab(tabgroup, 'Title', 'Position');
ax5 = uiaxes(tab5);
ax5.Units = 'normalized';
ax5.Position = [0.08 0.10 0.88 0.82];

plot(ax5, t_sample, p_d_log(1, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_ref);
hold(ax5, 'on');
plot(ax5, t_sample, p_d_log(2, :), '-', 'Color', colors(2, :), 'LineWidth', line_width_ref);
plot(ax5, t_sample, p_d_log(3, :), '-', 'Color', colors(3, :), 'LineWidth', line_width_ref);
plot(ax5, t_sample, p_m_log(1, :), '--', 'Color', colors(1, :), 'LineWidth', line_width_main);
plot(ax5, t_sample, p_m_log(2, :), '--', 'Color', colors(2, :), 'LineWidth', line_width_main);
plot(ax5, t_sample, p_m_log(3, :), '--', 'Color', colors(3, :), 'LineWidth', line_width_main);
hold(ax5, 'off');

legend(ax5, {'x_d', 'y_d', 'z_d', 'x_m', 'y_m', 'z_m'}, ...
    'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold', 'NumColumns', 2);
xlabel(ax5, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax5, 'Position [um]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax5, 'Position vs Time', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax5.LineWidth = axis_linewidth;
ax5.FontSize = tick_fontsize;
ax5.FontWeight = 'bold';
ax5.Box = 'on';
grid(ax5, 'on');

fprintf('  Tab 5: Position\n');

% ==================== Tab 6: Control Force ====================
tab6 = uitab(tabgroup, 'Title', 'Control Force');
ax6 = uiaxes(tab6);
ax6.Units = 'normalized';
ax6.Position = [0.08 0.10 0.88 0.82];

plot(ax6, t_sample, f_d_log(1, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_main);
hold(ax6, 'on');
plot(ax6, t_sample, f_d_log(2, :), '-', 'Color', colors(2, :), 'LineWidth', line_width_main);
plot(ax6, t_sample, f_d_log(3, :), '-', 'Color', colors(3, :), 'LineWidth', line_width_main);
hold(ax6, 'off');

legend(ax6, {'f_x', 'f_y', 'f_z'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
xlabel(ax6, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax6, 'Force [pN]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax6, sprintf('Control Force (%s)', ctrl_mode_str), 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax6.LineWidth = axis_linewidth;
ax6.FontSize = tick_fontsize;
ax6.FontWeight = 'bold';
ax6.Box = 'on';
grid(ax6, 'on');

% Add statistics
f_max = max(abs(f_d_log), [], 2);
stats_str = sprintf('Max |f_x|: %.4f pN\nMax |f_y|: %.4f pN\nMax |f_z|: %.4f pN', f_max(1), f_max(2), f_max(3));
text(ax6, 0.95, 0.95, stats_str, 'Units', 'normalized', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
    'FontSize', legend_fontsize, 'FontWeight', 'bold', ...
    'BackgroundColor', [1 1 1 0.8], 'EdgeColor', [0.3 0.3 0.3], 'Margin', 5);

fprintf('  Tab 6: Control Force\n');

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

% Export Tab 2-4: Axis Analysis (3x1 subplots each)
axis_names = {'X', 'Y', 'Z'};
axis_errors = {error_x, error_y, error_z};

for axis_idx = 1:3
    fig_export.Position = [100 100 800 900];  % Taller for 3x1 subplots

    % Subplot 1: Position Tracking
    ax_pos = subplot(3, 1, 1);
    plot(ax_pos, t_sample, p_m_log(axis_idx, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_main);
    hold(ax_pos, 'on');
    plot(ax_pos, t_sample, p_d_log(axis_idx, :), '--', 'Color', colors(2, :), 'LineWidth', line_width_ref);
    hold(ax_pos, 'off');
    legend(ax_pos, {sprintf('p_{m,%s}', lower(axis_names{axis_idx})), sprintf('p_{d,%s}', lower(axis_names{axis_idx}))}, ...
        'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
    ylabel(ax_pos, 'Position [um]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
    title(ax_pos, sprintf('%s-Axis Position Tracking', axis_names{axis_idx}), 'FontSize', title_fontsize, 'FontWeight', 'bold');
    ax_pos.LineWidth = axis_linewidth; ax_pos.FontSize = tick_fontsize; ax_pos.FontWeight = 'bold';
    ax_pos.Box = 'on'; grid(ax_pos, 'on');

    % Subplot 2: Tracking Error
    ax_err = subplot(3, 1, 2);
    plot(ax_err, t_sample, axis_errors{axis_idx}, 'k-', 'LineWidth', line_width_main);
    ylabel(ax_err, sprintf('e_%s [nm]', lower(axis_names{axis_idx})), 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
    title(ax_err, sprintf('%s-Axis Error', axis_names{axis_idx}), ...
        'FontSize', title_fontsize, 'FontWeight', 'bold');
    ax_err.LineWidth = axis_linewidth; ax_err.FontSize = tick_fontsize; ax_err.FontWeight = 'bold';
    ax_err.Box = 'on'; grid(ax_err, 'on');

    % Subplot 3: Control Force
    ax_force = subplot(3, 1, 3);
    plot(ax_force, t_sample, f_d_log(axis_idx, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_main);
    xlabel(ax_force, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
    ylabel(ax_force, sprintf('f_%s [pN]', lower(axis_names{axis_idx})), 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
    title(ax_force, sprintf('%s-Axis Control Force', axis_names{axis_idx}), 'FontSize', title_fontsize, 'FontWeight', 'bold');
    ax_force.LineWidth = axis_linewidth; ax_force.FontSize = tick_fontsize; ax_force.FontWeight = 'bold';
    ax_force.Box = 'on'; grid(ax_force, 'on');

    exportgraphics(fig_export, fullfile(output_dir, sprintf('%d_%s_axis_analysis.png', axis_idx+1, lower(axis_names{axis_idx}))), 'Resolution', 150);
    clf(fig_export);
end

fig_export.Position = [100 100 1000 700];  % Reset to normal size

% Export Tab 5: Position
ax_exp = axes(fig_export, 'Position', [0.12 0.12 0.82 0.80]);
plot(ax_exp, t_sample, p_d_log(1, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_ref); hold(ax_exp, 'on');
plot(ax_exp, t_sample, p_d_log(2, :), '-', 'Color', colors(2, :), 'LineWidth', line_width_ref);
plot(ax_exp, t_sample, p_d_log(3, :), '-', 'Color', colors(3, :), 'LineWidth', line_width_ref);
plot(ax_exp, t_sample, p_m_log(1, :), '--', 'Color', colors(1, :), 'LineWidth', line_width_main);
plot(ax_exp, t_sample, p_m_log(2, :), '--', 'Color', colors(2, :), 'LineWidth', line_width_main);
plot(ax_exp, t_sample, p_m_log(3, :), '--', 'Color', colors(3, :), 'LineWidth', line_width_main); hold(ax_exp, 'off');
legend(ax_exp, {'x_d', 'y_d', 'z_d', 'x_m', 'y_m', 'z_m'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold', 'NumColumns', 2);
xlabel(ax_exp, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax_exp, 'Position [um]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax_exp, 'Position vs Time', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax_exp.LineWidth = axis_linewidth; ax_exp.FontSize = tick_fontsize; ax_exp.FontWeight = 'bold'; ax_exp.Box = 'on'; grid(ax_exp, 'on');
exportgraphics(fig_export, fullfile(output_dir, '5_position.png'), 'Resolution', 150);
clf(fig_export);

% Export Tab 6: Control Force
ax_exp = axes(fig_export, 'Position', [0.12 0.12 0.82 0.80]);
plot(ax_exp, t_sample, f_d_log(1, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_main); hold(ax_exp, 'on');
plot(ax_exp, t_sample, f_d_log(2, :), '-', 'Color', colors(2, :), 'LineWidth', line_width_main);
plot(ax_exp, t_sample, f_d_log(3, :), '-', 'Color', colors(3, :), 'LineWidth', line_width_main); hold(ax_exp, 'off');
legend(ax_exp, {'f_x', 'f_y', 'f_z'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
xlabel(ax_exp, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax_exp, 'Force [pN]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax_exp, sprintf('Control Force (%s)', ctrl_mode_str), 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax_exp.LineWidth = axis_linewidth; ax_exp.FontSize = tick_fontsize; ax_exp.FontWeight = 'bold'; ax_exp.Box = 'on'; grid(ax_exp, 'on');
exportgraphics(fig_export, fullfile(output_dir, '6_control_force.png'), 'Resolution', 150);

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
result.error_x = error_x;
result.error_y = error_y;
result.error_z = error_z;
result.params = params.Value;
result.p0 = p0;
result.config = config;
result.tracking_error_rmse = rms(error);
result.tracking_error_rmse_x = rms(error_x);
result.tracking_error_rmse_y = rms(error_y);
result.tracking_error_rmse_z = rms(error_z);
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
fprintf('Tracking Performance (3D):\n');
fprintf('  Max error: %.4f um (%.2f nm)\n', max(error), max(error)*1000);
fprintf('\n');
fprintf('Tracking Performance (per axis):\n');
fprintf('  X-axis Max: %.2f nm\n', max(abs(error_x)));
fprintf('  Y-axis Max: %.2f nm\n', max(abs(error_y)));
fprintf('  Z-axis Max: %.2f nm\n', max(abs(error_z)));
fprintf('\n');
fprintf('Control Force:\n');
fprintf('  Max |f_x|: %.4f pN\n', max(abs(f_d_log(1, :))));
fprintf('  Max |f_y|: %.4f pN\n', max(abs(f_d_log(2, :))));
fprintf('  Max |f_z|: %.4f pN\n', max(abs(f_d_log(3, :))));
fprintf('\n');
fprintf('Wall Distance:\n');
fprintf('  Min h/R: %.2f (threshold: %.1f)\n', min(h_bar_log), params.Value.wall.h_bar_min);
fprintf('  Max h/R: %.2f\n', max(h_bar_log));
fprintf('\n');
fprintf('Results saved to: %s\n', output_dir);
fprintf('================================================================\n');
