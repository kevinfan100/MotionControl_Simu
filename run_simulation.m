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
clear motion_control_law;  % Reset persistent variables in controller

%% SECTION 1: High-Level Parameter Configuration
% === Wall Parameters ===
theta = 0;          % Azimuth angle [rad]
phi = 0;         % Elevation angle [rad]
pz = 0;             % Wall displacement along w_hat [um]
h_min = 1 * 2.25;      % Minimum safe distance [um] (default: 1.1 * R)

% === Trajectory Parameters ===
traj_type = 'z_sine';   % 'z_sine' or 'xy_circle'
h_init = 5;            % Initial distance from wall [um]
amplitude = 2.5;          % z_sine: oscillation amplitude [um]
frequency = 10;          % z_sine: oscillation frequency [Hz]
n_cycles = 2;           % z_sine/xy_circle: number of cycles
radius = 5;             % xy_circle: radius [um]
period = 1;             % xy_circle: period [sec]

% === Controller Parameters ===
ctrl_enable = true;    % true = closed-loop, false = open-loop
lambda_c = 0.4;         % Closed-loop pole (0 < lambda_c < 1)

noise_filter_enable = true;   % Enable low-pass filter on controller feedback
noise_filter_cutoff = min(frequency * 20, 1606 / 10);       % Cutoff frequency [Hz]

% === Thermal Force ===
thermal_enable = true;  % Enable Brownian motion disturbance

% === Simulation Parameters ===
T_margin = 0.3;         % Buffer time after trajectory completes [sec]

% === Open-loop Thermal Analysis Parameters ===
openloop_cutoff_freq = min(frequency * 20, 1606 / 10);  % Drift/Noise cutoff frequency [Hz] (adjustable)

% === Closed-loop Thermal Analysis Parameters ===
closedloop_cutoff_freq = min(frequency * 20, 1606 / 10);   % High-pass cutoff frequency [Hz]

% Auto-calculate simulation time based on trajectory
if strcmp(traj_type, 'z_sine')
    T_traj = n_cycles / frequency;
else  % xy_circle
    T_traj = period * n_cycles;
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
    'theta', theta, 'phi', phi, 'pz', pz, 'h_min', h_min, ...
    'traj_type', traj_type, 'h_init', h_init, ...
    'amplitude', amplitude, 'frequency', frequency, 'n_cycles', n_cycles, ...
    'radius', radius, 'period', period, ...
    'ctrl_enable', ctrl_enable, 'lambda_c', lambda_c, ...
    'noise_filter_enable', noise_filter_enable, ...
    'noise_filter_cutoff', noise_filter_cutoff, ...
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
[is_safe, h_min_actual, t_critical] = check_trajectory_safety(p0, params.Value);

if ~is_safe
    warning('Trajectory unsafe! Min h = %.2f um at t = %.3f sec (threshold: %.2f um)', ...
        h_min_actual, t_critical, params.Value.wall.h_min);
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
traj_type_str = 'z_sine';
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
% Note: Random seed for thermal force is set in params.thermal.seed
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

% Post-processing: Calculate p_m_filtered (for visualization when noise_filter_enable = true)
% This replicates the single-stage IIR filter used in motion_control_law.m
if noise_filter_enable
    alpha = params.Value.ctrl.filter_alpha;
    p_m_filtered = zeros(3, N_samples);
    p_m_filtered(:, 1) = p_m_log(:, 1);
    for k = 2:N_samples
        p_m_filtered(:, k) = alpha * p_m_log(:, k) + (1 - alpha) * p_m_filtered(:, k-1);
    end
else
    p_m_filtered = p_m_log;  % When filter disabled, use raw p_m
end

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

% --- Add wall plane visualization ---
w_hat = params.Value.wall.w_hat;
u_hat = params.Value.wall.u_hat;
v_hat = params.Value.wall.v_hat;
pz_wall = params.Value.wall.pz;

% Wall center point
wall_center = pz_wall * w_hat;

% Calculate plane size based on trajectory bounding box
all_pos = [p_d_log, p_m_log];
traj_range = max([range(all_pos(1,:)), range(all_pos(2,:)), range(all_pos(3,:))]);
plane_size = max(traj_range * 1.5, 10);  % At least 10 um

% Wall plane corners (in u-v plane centered at wall_center)
corners = [
    wall_center + plane_size/2 * u_hat + plane_size/2 * v_hat, ...
    wall_center - plane_size/2 * u_hat + plane_size/2 * v_hat, ...
    wall_center - plane_size/2 * u_hat - plane_size/2 * v_hat, ...
    wall_center + plane_size/2 * u_hat - plane_size/2 * v_hat ...
];

% Draw wall plane (semi-transparent gray)
fill3(ax1, corners(1,:), corners(2,:), corners(3,:), ...
    [0.7 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 1.5);
hold(ax1, 'off');

legend(ax1, {'p_d (desired)', 'p_m (measured)', 'Start', 'Wall'}, ...
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

% Calculate X-axis error in nm (use p_m_filtered for controller's perspective)
error_x = (p_m_filtered(1, :) - p_d_log(1, :)) * 1000;

% Subplot positions [left bottom width height]
pos_top = [0.10 0.70 0.85 0.25];
pos_mid = [0.10 0.40 0.85 0.25];
pos_bot = [0.10 0.08 0.85 0.25];

% --- Subplot 1: X Position Tracking ---
ax2_pos = uiaxes(tab2, 'Position', [pos_top(1)*1200, pos_top(2)*800, pos_top(3)*1200, pos_top(4)*800]);
plot(ax2_pos, t_sample, p_m_filtered(1, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_main);
hold(ax2_pos, 'on');
plot(ax2_pos, t_sample, p_d_log(1, :), '--', 'Color', colors(2, :), 'LineWidth', line_width_ref);
hold(ax2_pos, 'off');
if noise_filter_enable
    legend(ax2_pos, {'p_{m,x} (filtered)', 'p_{d,x}'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
else
    legend(ax2_pos, {'p_{m,x}', 'p_{d,x}'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
end
ylabel(ax2_pos, 'Position [um]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax2_pos, 'X-Axis Position Tracking', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax2_pos.LineWidth = axis_linewidth; ax2_pos.FontSize = tick_fontsize; ax2_pos.FontWeight = 'bold';
ax2_pos.Box = 'on'; grid(ax2_pos, 'on');
ax2_pos.XTickLabel = [];

% --- Subplot 2: X Tracking Error ---
ax2_err = uiaxes(tab2, 'Position', [pos_mid(1)*1200, pos_mid(2)*800, pos_mid(3)*1200, pos_mid(4)*800]);
plot(ax2_err, t_sample, error_x, 'k-', 'LineWidth', line_width_main);
ylabel(ax2_err, 'e_x [nm]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax2_err, 'X-Axis Error (Controller View)', 'FontSize', title_fontsize, 'FontWeight', 'bold');
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

% Calculate Y-axis error in nm (use p_m_filtered for controller's perspective)
error_y = (p_m_filtered(2, :) - p_d_log(2, :)) * 1000;

% --- Subplot 1: Y Position Tracking ---
ax3_pos = uiaxes(tab3, 'Position', [pos_top(1)*1200, pos_top(2)*800, pos_top(3)*1200, pos_top(4)*800]);
plot(ax3_pos, t_sample, p_m_filtered(2, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_main);
hold(ax3_pos, 'on');
plot(ax3_pos, t_sample, p_d_log(2, :), '--', 'Color', colors(2, :), 'LineWidth', line_width_ref);
hold(ax3_pos, 'off');
if noise_filter_enable
    legend(ax3_pos, {'p_{m,y} (filtered)', 'p_{d,y}'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
else
    legend(ax3_pos, {'p_{m,y}', 'p_{d,y}'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
end
ylabel(ax3_pos, 'Position [um]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax3_pos, 'Y-Axis Position Tracking', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax3_pos.LineWidth = axis_linewidth; ax3_pos.FontSize = tick_fontsize; ax3_pos.FontWeight = 'bold';
ax3_pos.Box = 'on'; grid(ax3_pos, 'on');
ax3_pos.XTickLabel = [];

% --- Subplot 2: Y Tracking Error ---
ax3_err = uiaxes(tab3, 'Position', [pos_mid(1)*1200, pos_mid(2)*800, pos_mid(3)*1200, pos_mid(4)*800]);
plot(ax3_err, t_sample, error_y, 'k-', 'LineWidth', line_width_main);
ylabel(ax3_err, 'e_y [nm]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax3_err, 'Y-Axis Error (Controller View)', 'FontSize', title_fontsize, 'FontWeight', 'bold');
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

% Calculate Z-axis error in nm (use p_m_filtered for controller's perspective)
error_z = (p_m_filtered(3, :) - p_d_log(3, :)) * 1000;

% --- Subplot 1: Z Position Tracking ---
ax4_pos = uiaxes(tab4, 'Position', [pos_top(1)*1200, pos_top(2)*800, pos_top(3)*1200, pos_top(4)*800]);
plot(ax4_pos, t_sample, p_m_filtered(3, :), '-', 'Color', colors(1, :), 'LineWidth', line_width_main);
hold(ax4_pos, 'on');
plot(ax4_pos, t_sample, p_d_log(3, :), '--', 'Color', colors(2, :), 'LineWidth', line_width_ref);
hold(ax4_pos, 'off');
if noise_filter_enable
    legend(ax4_pos, {'p_{m,z} (filtered)', 'p_{d,z}'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
else
    legend(ax4_pos, {'p_{m,z}', 'p_{d,z}'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
end
ylabel(ax4_pos, 'Position [um]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax4_pos, 'Z-Axis Position Tracking', 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax4_pos.LineWidth = axis_linewidth; ax4_pos.FontSize = tick_fontsize; ax4_pos.FontWeight = 'bold';
ax4_pos.Box = 'on'; grid(ax4_pos, 'on');
ax4_pos.XTickLabel = [];

% --- Subplot 2: Z Tracking Error ---
ax4_err = uiaxes(tab4, 'Position', [pos_mid(1)*1200, pos_mid(2)*800, pos_mid(3)*1200, pos_mid(4)*800]);
plot(ax4_err, t_sample, error_z, 'k-', 'LineWidth', line_width_main);
ylabel(ax4_err, 'e_z [nm]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax4_err, 'Z-Axis Error (Controller View)', 'FontSize', title_fontsize, 'FontWeight', 'bold');
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

% ==================== Tab 7: FFT Spectrum Analysis ====================
% Shows p_m FFT spectrum for verifying cutoff frequency selection
% Uses same format as Open-loop FFT for consistency
tab7_fft = uitab(tabgroup, 'Title', 'FFT Spectrum');

% Use fft_drift_noise_separation for proper spectrum calculation (same as open-loop)
Fs = 1 / Ts;
time_vec_fft = t_sample(:);  % Column vector for fft function

% Convert p_m to nm for FFT analysis
p_m_x_nm = p_m_log(1, :)' * 1000;  % um -> nm, column vector
p_m_y_nm = p_m_log(2, :)' * 1000;
p_m_z_nm = p_m_log(3, :)' * 1000;

% Calculate FFT spectrum using fft_drift_noise_separation
[~, ~, ~, freq_x, spectrum_x] = fft_drift_noise_separation(p_m_x_nm, time_vec_fft, noise_filter_cutoff);
[~, ~, ~, freq_y, spectrum_y] = fft_drift_noise_separation(p_m_y_nm, time_vec_fft, noise_filter_cutoff);
[~, ~, ~, freq_z, spectrum_z] = fft_drift_noise_separation(p_m_z_nm, time_vec_fft, noise_filter_cutoff);

% Calculate common Y-axis limits for all spectra (for easier comparison)
all_spectrum = [spectrum_x(2:end); spectrum_y(2:end); spectrum_z(2:end)];
y_min_fft = min(all_spectrum(all_spectrum > 0)) * 0.5;
y_max_fft = max(all_spectrum) * 2;

% Subplot positions (pixel coordinates for uiaxes)
pos_fft_top = [120 560 1020 200];
pos_fft_mid = [120 320 1020 200];
pos_fft_bot = [120 64 1020 200];

% --- X-axis FFT (loglog like open-loop) ---
ax_fft_x = uiaxes(tab7_fft, 'Position', pos_fft_top);
loglog(ax_fft_x, freq_x(2:end), spectrum_x(2:end), 'b-', 'LineWidth', 1.5);
hold(ax_fft_x, 'on');
xline(ax_fft_x, noise_filter_cutoff, 'r--', 'LineWidth', 2, 'Label', sprintf('cutoff = %.0f Hz', noise_filter_cutoff));
hold(ax_fft_x, 'off');
ylabel(ax_fft_x, 'X Amp. (nm)', 'FontSize', 16, 'FontWeight', 'bold');
title(ax_fft_x, 'Position FFT Spectrum (Closed-loop)', 'FontSize', 18, 'FontWeight', 'bold');
ax_fft_x.FontSize = 14; ax_fft_x.FontWeight = 'bold'; ax_fft_x.LineWidth = 1.5;
ax_fft_x.Box = 'on';
xlim(ax_fft_x, [freq_x(2), Fs/2]);
ylim(ax_fft_x, [y_min_fft, y_max_fft]);
ax_fft_x.XTickLabel = [];

% --- Y-axis FFT ---
ax_fft_y = uiaxes(tab7_fft, 'Position', pos_fft_mid);
loglog(ax_fft_y, freq_y(2:end), spectrum_y(2:end), 'g-', 'LineWidth', 1.5);
hold(ax_fft_y, 'on');
xline(ax_fft_y, noise_filter_cutoff, 'r--', 'LineWidth', 2);
hold(ax_fft_y, 'off');
ylabel(ax_fft_y, 'Y Amp. (nm)', 'FontSize', 16, 'FontWeight', 'bold');
ax_fft_y.FontSize = 14; ax_fft_y.FontWeight = 'bold'; ax_fft_y.LineWidth = 1.5;
ax_fft_y.Box = 'on';
xlim(ax_fft_y, [freq_x(2), Fs/2]);
ylim(ax_fft_y, [y_min_fft, y_max_fft]);
ax_fft_y.XTickLabel = [];

% --- Z-axis FFT ---
ax_fft_z = uiaxes(tab7_fft, 'Position', pos_fft_bot);
loglog(ax_fft_z, freq_z(2:end), spectrum_z(2:end), 'r-', 'LineWidth', 1.5);
hold(ax_fft_z, 'on');
xline(ax_fft_z, noise_filter_cutoff, 'r--', 'LineWidth', 2);
hold(ax_fft_z, 'off');
xlabel(ax_fft_z, 'Frequency (Hz)', 'FontSize', 16, 'FontWeight', 'bold');
ylabel(ax_fft_z, 'Z Amp. (nm)', 'FontSize', 16, 'FontWeight', 'bold');
ax_fft_z.FontSize = 14; ax_fft_z.FontWeight = 'bold'; ax_fft_z.LineWidth = 1.5;
ax_fft_z.Box = 'on';
xlim(ax_fft_z, [freq_x(2), Fs/2]);
ylim(ax_fft_z, [y_min_fft, y_max_fft]);

fprintf('  Tab 7: FFT Spectrum\n');

% ==================== Tab 8: Drift/Noise Separation (3-axis) ====================
% Shows deterministic (drift) and stochastic (noise) components for each axis
tab8 = uitab(tabgroup, 'Title', 'Drift/Noise Sep.');

% Use tracking error for closed-loop analysis
error_x_nm = (p_m_log(1, :) - p_d_log(1, :))' * 1000;  % um -> nm
error_y_nm = (p_m_log(2, :) - p_d_log(2, :))' * 1000;
error_z_nm = (p_m_log(3, :) - p_d_log(3, :))' * 1000;

% Perform drift/noise separation for each axis
[std_x, drift_x, noise_x, ~, ~] = fft_drift_noise_separation(error_x_nm, t_sample', noise_filter_cutoff);
[std_y, drift_y, noise_y, ~, ~] = fft_drift_noise_separation(error_y_nm, t_sample', noise_filter_cutoff);
[std_z, drift_z, noise_z_sep, ~, ~] = fft_drift_noise_separation(error_z_nm, t_sample', noise_filter_cutoff);

% Handle length mismatch (fft_drift_noise_separation may truncate odd-length data)
N_sep = length(drift_x);
t_sep = t_sample(1:N_sep);

% Subplot positions (2 columns x 3 rows)
fig_w = 1200; fig_h = 800;
col1_x = 0.08; col2_x = 0.55;
row_w = 0.40; row_h = 0.24;
row1_y = 0.70; row2_y = 0.40; row3_y = 0.10;

% --- Left column: Deterministic (Drift) ---
ax8_drift_x = uiaxes(tab8, 'Position', [col1_x*fig_w, row1_y*fig_h, row_w*fig_w, row_h*fig_h]);
plot(ax8_drift_x, t_sep, drift_x, 'b-', 'LineWidth', 1.5);
ylabel(ax8_drift_x, 'X [nm]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax8_drift_x, sprintf('Deterministic (LP < %.0f Hz)', noise_filter_cutoff), 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax8_drift_x.FontSize = tick_fontsize; ax8_drift_x.FontWeight = 'bold';
ax8_drift_x.LineWidth = axis_linewidth; ax8_drift_x.Box = 'on';
grid(ax8_drift_x, 'on'); xlim(ax8_drift_x, [0, t_sep(end)]);

ax8_drift_y = uiaxes(tab8, 'Position', [col1_x*fig_w, row2_y*fig_h, row_w*fig_w, row_h*fig_h]);
plot(ax8_drift_y, t_sep, drift_y, 'Color', colors(2,:), 'LineWidth', 1.5);
ylabel(ax8_drift_y, 'Y [nm]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
ax8_drift_y.FontSize = tick_fontsize; ax8_drift_y.FontWeight = 'bold';
ax8_drift_y.LineWidth = axis_linewidth; ax8_drift_y.Box = 'on';
grid(ax8_drift_y, 'on'); xlim(ax8_drift_y, [0, t_sep(end)]);

ax8_drift_z = uiaxes(tab8, 'Position', [col1_x*fig_w, row3_y*fig_h, row_w*fig_w, row_h*fig_h]);
plot(ax8_drift_z, t_sep, drift_z, 'r-', 'LineWidth', 1.5);
xlabel(ax8_drift_z, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax8_drift_z, 'Z [nm]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
ax8_drift_z.FontSize = tick_fontsize; ax8_drift_z.FontWeight = 'bold';
ax8_drift_z.LineWidth = axis_linewidth; ax8_drift_z.Box = 'on';
grid(ax8_drift_z, 'on'); xlim(ax8_drift_z, [0, t_sep(end)]);

% --- Right column: Stochastic (Noise) ---
ax8_noise_x = uiaxes(tab8, 'Position', [col2_x*fig_w, row1_y*fig_h, row_w*fig_w, row_h*fig_h]);
plot(ax8_noise_x, t_sep, noise_x, 'b-', 'LineWidth', 0.8);
ylabel(ax8_noise_x, 'X [nm]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
title(ax8_noise_x, sprintf('Stochastic (HP > %.0f Hz)', noise_filter_cutoff), 'FontSize', title_fontsize, 'FontWeight', 'bold');
ax8_noise_x.FontSize = tick_fontsize; ax8_noise_x.FontWeight = 'bold';
ax8_noise_x.LineWidth = axis_linewidth; ax8_noise_x.Box = 'on';
grid(ax8_noise_x, 'on'); xlim(ax8_noise_x, [0, t_sep(end)]);

ax8_noise_y = uiaxes(tab8, 'Position', [col2_x*fig_w, row2_y*fig_h, row_w*fig_w, row_h*fig_h]);
plot(ax8_noise_y, t_sep, noise_y, 'Color', colors(2,:), 'LineWidth', 0.8);
ylabel(ax8_noise_y, 'Y [nm]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
ax8_noise_y.FontSize = tick_fontsize; ax8_noise_y.FontWeight = 'bold';
ax8_noise_y.LineWidth = axis_linewidth; ax8_noise_y.Box = 'on';
grid(ax8_noise_y, 'on'); xlim(ax8_noise_y, [0, t_sep(end)]);

ax8_noise_z = uiaxes(tab8, 'Position', [col2_x*fig_w, row3_y*fig_h, row_w*fig_w, row_h*fig_h]);
plot(ax8_noise_z, t_sep, noise_z_sep, 'r-', 'LineWidth', 0.8);
xlabel(ax8_noise_z, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
ylabel(ax8_noise_z, 'Z [nm]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
ax8_noise_z.FontSize = tick_fontsize; ax8_noise_z.FontWeight = 'bold';
ax8_noise_z.LineWidth = axis_linewidth; ax8_noise_z.Box = 'on';
grid(ax8_noise_z, 'on'); xlim(ax8_noise_z, [0, t_sep(end)]);

fprintf('  Tab 8: Drift/Noise Separation\n');

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
% Add wall plane
fill3(ax_exp, corners(1,:), corners(2,:), corners(3,:), ...
    [0.7 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 1.5);
hold(ax_exp, 'off');
legend(ax_exp, {'p_d (desired)', 'p_m (measured)', 'Start', 'Wall'}, 'Location', 'best', 'FontSize', legend_fontsize, 'FontWeight', 'bold');
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

%% SECTION 7: Open-loop Thermal Force Analysis (as Tabs)
% Triggered when: ctrl_enable = false AND thermal_enable = true
% Analyzes p_m (position response) characteristics
% F_th STD is calculated from theoretical formula

is_openloop_thermal = (params.Value.ctrl.enable < 0.5) && ...
                      (params.Value.thermal.enable > 0.5);

if is_openloop_thermal
    fprintf('\n');
    fprintf('================================================================\n');
    fprintf('  Open-loop Thermal Force Analysis\n');
    fprintf('================================================================\n\n');

    % Get sample rate
    Fs = 1 / Ts;

    % Calculate theoretical F_th STD from formula:
    % F_th ~ N(0, Variance), Variance = (4 * k_B * T * gamma_N / Ts) * C^2
    % STD = sqrt(Variance) = sqrt(4 * k_B * T * gamma_N / Ts) * |C|
    k_B = params.Value.thermal.k_B;
    T_temp = params.Value.thermal.T;
    gamma_N = params.Value.common.gamma_N;
    variance_coeff = 4 * k_B * T_temp * gamma_N / Ts;

    % Get correction coefficients at initial position
    h_bar_init = (dot(p0, params.Value.wall.w_hat) - params.Value.wall.pz) / params.Value.common.R;
    [c_para, c_perp] = calc_correction_functions(h_bar_init);
    C_vec = c_para * (params.Value.wall.u_hat + params.Value.wall.v_hat) + c_perp * params.Value.wall.w_hat;
    std_Fth_theory = sqrt(variance_coeff) * abs(C_vec);  % [3x1] pN

    % Prepare p_m data for analysis (convert to column vectors, nm)
    p_m_x_nm = p_m_log(1, :)' * 1000;  % um -> nm
    p_m_y_nm = p_m_log(2, :)' * 1000;
    p_m_z_nm = p_m_log(3, :)' * 1000;
    time_vec = t_sample';

    % FFT Analysis for p_m only
    fprintf('Performing FFT analysis (cutoff = %.1f Hz)...\n', openloop_cutoff_freq);

    [std_pm_x, drift_pm_x, ~, freq_pm_x, spectrum_pm_x] = ...
        fft_drift_noise_separation(p_m_x_nm, time_vec, openloop_cutoff_freq);
    [std_pm_y, drift_pm_y, ~, freq_pm_y, spectrum_pm_y] = ...
        fft_drift_noise_separation(p_m_y_nm, time_vec, openloop_cutoff_freq);
    [std_pm_z, drift_pm_z, ~, freq_pm_z, spectrum_pm_z] = ...
        fft_drift_noise_separation(p_m_z_nm, time_vec, openloop_cutoff_freq);

    % Calculate Drift Peak-to-Peak
    pp_pm_x = max(drift_pm_x) - min(drift_pm_x);
    pp_pm_y = max(drift_pm_y) - min(drift_pm_y);
    pp_pm_z = max(drift_pm_z) - min(drift_pm_z);

    fprintf('  Done.\n\n');

    % Adjust data length if needed (FFT may truncate odd-length data)
    N_fft = length(drift_pm_x);
    time_plot = time_vec(1:N_fft);
    p_m_x_plot = p_m_x_nm(1:N_fft);
    p_m_y_plot = p_m_y_nm(1:N_fft);
    p_m_z_plot = p_m_z_nm(1:N_fft);
    duration_plot = time_plot(end);

    % ==================== Tab 7: Open-loop Time Response ====================
    tab7 = uitab(tabgroup, 'Title', 'Open-loop Time');

    % Subplot positions (pixel-based for uiaxes, assuming 1200x800 figure)
    fig_w = 1200; fig_h = 800;
    pos7_top = [0.10*fig_w, 0.70*fig_h, 0.85*fig_w, 0.25*fig_h];
    pos7_mid = [0.10*fig_w, 0.40*fig_h, 0.85*fig_w, 0.25*fig_h];
    pos7_bot = [0.10*fig_w, 0.08*fig_h, 0.85*fig_w, 0.25*fig_h];

    % Remove initial offset (drift's first point) for better visualization
    % This allows direct comparison of fluctuations across all axes
    offset_x = drift_pm_x(1);
    offset_y = drift_pm_y(1);
    offset_z = drift_pm_z(1);
    p_m_x_centered = p_m_x_plot - offset_x;
    p_m_y_centered = p_m_y_plot - offset_y;
    p_m_z_centered = p_m_z_plot - offset_z;
    drift_x_centered = drift_pm_x - offset_x;
    drift_y_centered = drift_pm_y - offset_y;
    drift_z_centered = drift_pm_z - offset_z;

    % Calculate unified Y-axis range based on max deviation across all axes
    max_dev_x = max(abs(p_m_x_centered));
    max_dev_y = max(abs(p_m_y_centered));
    max_dev_z = max(abs(p_m_z_centered));
    max_dev = max([max_dev_x, max_dev_y, max_dev_z]);
    y_limit = max_dev * 1.15;  % Add 15% margin

    % Calculate nice tick interval
    tick_interval = 10 ^ floor(log10(y_limit));
    if y_limit / tick_interval < 2
        tick_interval = tick_interval / 2;
    elseif y_limit / tick_interval > 5
        tick_interval = tick_interval * 2;
    end

    % --- X Position ---
    ax7_x = uiaxes(tab7, 'Position', pos7_top);
    h7x = plot(ax7_x, time_plot, p_m_x_centered, 'b-', 'LineWidth', 1.2);
    h7x.Color(4) = 0.5;
    hold(ax7_x, 'on');
    plot(ax7_x, time_plot, drift_x_centered, 'k-', 'LineWidth', 1.8);
    hold(ax7_x, 'off');
    ax7_x.FontSize = 14; ax7_x.FontWeight = 'bold'; ax7_x.LineWidth = 1.5;
    ax7_x.Box = 'on';
    ylabel(ax7_x, '\Deltap_{m,x} (nm)', 'FontSize', 16, 'FontWeight', 'bold');
    title(ax7_x, 'Open-loop Position Response (Thermal Force Only, DC removed)', 'FontSize', 18, 'FontWeight', 'bold');
    xlim(ax7_x, [0, duration_plot]);
    ylim(ax7_x, [-y_limit, y_limit]);
    ax7_x.YTick = -ceil(y_limit/tick_interval)*tick_interval : tick_interval : ceil(y_limit/tick_interval)*tick_interval;
    ax7_x.XTickLabel = [];
    text(ax7_x, 0.98, 0.95, sprintf('STD = %.2f nm', std_pm_x), ...
        'Units', 'normalized', 'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'top', 'FontSize', 14, 'FontWeight', 'bold', ...
        'BackgroundColor', 'white', 'EdgeColor', 'black', 'LineWidth', 1.5);

    % --- Y Position ---
    ax7_y = uiaxes(tab7, 'Position', pos7_mid);
    h7y = plot(ax7_y, time_plot, p_m_y_centered, 'g-', 'LineWidth', 1.2);
    h7y.Color(4) = 0.5;
    hold(ax7_y, 'on');
    plot(ax7_y, time_plot, drift_y_centered, 'k-', 'LineWidth', 1.8);
    hold(ax7_y, 'off');
    ax7_y.FontSize = 14; ax7_y.FontWeight = 'bold'; ax7_y.LineWidth = 1.5;
    ax7_y.Box = 'on';
    ylabel(ax7_y, '\Deltap_{m,y} (nm)', 'FontSize', 16, 'FontWeight', 'bold');
    xlim(ax7_y, [0, duration_plot]);
    ylim(ax7_y, [-y_limit, y_limit]);
    ax7_y.YTick = -ceil(y_limit/tick_interval)*tick_interval : tick_interval : ceil(y_limit/tick_interval)*tick_interval;
    ax7_y.XTickLabel = [];
    text(ax7_y, 0.98, 0.95, sprintf('STD = %.2f nm', std_pm_y), ...
        'Units', 'normalized', 'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'top', 'FontSize', 14, 'FontWeight', 'bold', ...
        'BackgroundColor', 'white', 'EdgeColor', 'black', 'LineWidth', 1.5);

    % --- Z Position ---
    ax7_z = uiaxes(tab7, 'Position', pos7_bot);
    h7z = plot(ax7_z, time_plot, p_m_z_centered, 'r-', 'LineWidth', 1.2);
    h7z.Color(4) = 0.5;
    hold(ax7_z, 'on');
    plot(ax7_z, time_plot, drift_z_centered, 'k-', 'LineWidth', 1.8);
    hold(ax7_z, 'off');
    ax7_z.FontSize = 14; ax7_z.FontWeight = 'bold'; ax7_z.LineWidth = 1.5;
    ax7_z.Box = 'on';
    xlabel(ax7_z, 'Time (s)', 'FontSize', 16, 'FontWeight', 'bold');
    ylabel(ax7_z, '\Deltap_{m,z} (nm)', 'FontSize', 16, 'FontWeight', 'bold');
    xlim(ax7_z, [0, duration_plot]);
    ylim(ax7_z, [-y_limit, y_limit]);
    ax7_z.YTick = -ceil(y_limit/tick_interval)*tick_interval : tick_interval : ceil(y_limit/tick_interval)*tick_interval;
    text(ax7_z, 0.98, 0.95, sprintf('STD = %.2f nm', std_pm_z), ...
        'Units', 'normalized', 'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'top', 'FontSize', 14, 'FontWeight', 'bold', ...
        'BackgroundColor', 'white', 'EdgeColor', 'black', 'LineWidth', 1.5);

    fprintf('  Tab 7: Open-loop Time Response\n');

    % ==================== Tab 8: Open-loop FFT Spectrum ====================
    tab8 = uitab(tabgroup, 'Title', 'Open-loop FFT');

    % Calculate common Y-axis limits for all spectra (for easier comparison)
    all_spectrum = [spectrum_pm_x(2:end); spectrum_pm_y(2:end); spectrum_pm_z(2:end)];
    y_min_fft = min(all_spectrum(all_spectrum > 0)) * 0.5;
    y_max_fft = max(all_spectrum) * 2;

    % --- X Spectrum ---
    ax8_x = uiaxes(tab8, 'Position', pos7_top);
    loglog(ax8_x, freq_pm_x(2:end), spectrum_pm_x(2:end), 'b-', 'LineWidth', 1.5);
    ax8_x.FontSize = 14; ax8_x.FontWeight = 'bold'; ax8_x.LineWidth = 1.5;
    ax8_x.Box = 'on';
    ylabel(ax8_x, 'X Amp. (nm)', 'FontSize', 16, 'FontWeight', 'bold');
    title(ax8_x, 'Position FFT Spectrum (Open-loop)', 'FontSize', 18, 'FontWeight', 'bold');
    xlim(ax8_x, [freq_pm_x(2), Fs/2]);
    ylim(ax8_x, [y_min_fft, y_max_fft]);
    ax8_x.XTickLabel = [];

    % --- Y Spectrum ---
    ax8_y = uiaxes(tab8, 'Position', pos7_mid);
    loglog(ax8_y, freq_pm_y(2:end), spectrum_pm_y(2:end), 'g-', 'LineWidth', 1.5);
    ax8_y.FontSize = 14; ax8_y.FontWeight = 'bold'; ax8_y.LineWidth = 1.5;
    ax8_y.Box = 'on';
    ylabel(ax8_y, 'Y Amp. (nm)', 'FontSize', 16, 'FontWeight', 'bold');
    xlim(ax8_y, [freq_pm_x(2), Fs/2]);
    ylim(ax8_y, [y_min_fft, y_max_fft]);
    ax8_y.XTickLabel = [];

    % --- Z Spectrum ---
    ax8_z = uiaxes(tab8, 'Position', pos7_bot);
    loglog(ax8_z, freq_pm_z(2:end), spectrum_pm_z(2:end), 'r-', 'LineWidth', 1.5);
    ax8_z.FontSize = 14; ax8_z.FontWeight = 'bold'; ax8_z.LineWidth = 1.5;
    ax8_z.Box = 'on';
    xlabel(ax8_z, 'Frequency (Hz)', 'FontSize', 16, 'FontWeight', 'bold');
    ylabel(ax8_z, 'Z Amp. (nm)', 'FontSize', 16, 'FontWeight', 'bold');
    xlim(ax8_z, [freq_pm_x(2), Fs/2]);
    ylim(ax8_z, [y_min_fft, y_max_fft]);

    fprintf('  Tab 8: Open-loop FFT Spectrum\n');

    % ==================== Statistics Output ====================
    fprintf('\n');
    fprintf('========================================\n');
    fprintf('  Open-loop Thermal Analysis Results\n');
    fprintf('========================================\n\n');

    fprintf('Thermal Force (F_th) - Theoretical STD:\n');
    fprintf('  X: STD = %.4f pN\n', std_Fth_theory(1));
    fprintf('  Y: STD = %.4f pN\n', std_Fth_theory(2));
    fprintf('  Z: STD = %.4f pN\n\n', std_Fth_theory(3));

    fprintf('Position Response (p_m) - Measured:\n');
    fprintf('  X: STD = %.2f nm, Drift P-P = %.2f nm\n', std_pm_x, pp_pm_x);
    fprintf('  Y: STD = %.2f nm, Drift P-P = %.2f nm\n', std_pm_y, pp_pm_y);
    fprintf('  Z: STD = %.2f nm, Drift P-P = %.2f nm\n\n', std_pm_z, pp_pm_z);

    fprintf('Analysis Parameters:\n');
    fprintf('  Cutoff Frequency: %.1f Hz\n', openloop_cutoff_freq);
    fprintf('  Sample Rate: %.0f Hz\n', Fs);
    fprintf('  Duration: %.2f sec\n', duration_plot);
    fprintf('  FFT Resolution: %.4f Hz\n', Fs / N_fft);
    fprintf('========================================\n');

    % ==================== Save Open-loop Analysis Figures ====================
    % Tab 7: Time Response
    fig_export = figure('Visible', 'off', 'Position', [100, 100, 1000, 800]);

    subplot(3,1,1);
    h1 = plot(time_plot, p_m_x_centered, 'b-', 'LineWidth', 1.2); h1.Color(4) = 0.5;
    hold on; plot(time_plot, drift_x_centered, 'k-', 'LineWidth', 1.8); hold off;
    ylabel('\Deltap_{m,x} (nm)'); title('Open-loop Position Response (Thermal Force Only, DC removed)');
    xlim([0, duration_plot]); ylim([-y_limit, y_limit]);
    text(0.98, 0.95, sprintf('STD = %.2f nm', std_pm_x), 'Units', 'normalized', ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
        'BackgroundColor', 'white', 'EdgeColor', 'black');
    set(gca, 'FontSize', 12, 'FontWeight', 'bold', 'Box', 'on');

    subplot(3,1,2);
    h2 = plot(time_plot, p_m_y_centered, 'g-', 'LineWidth', 1.2); h2.Color(4) = 0.5;
    hold on; plot(time_plot, drift_y_centered, 'k-', 'LineWidth', 1.8); hold off;
    ylabel('\Deltap_{m,y} (nm)');
    xlim([0, duration_plot]); ylim([-y_limit, y_limit]);
    text(0.98, 0.95, sprintf('STD = %.2f nm', std_pm_y), 'Units', 'normalized', ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
        'BackgroundColor', 'white', 'EdgeColor', 'black');
    set(gca, 'FontSize', 12, 'FontWeight', 'bold', 'Box', 'on');

    subplot(3,1,3);
    h3 = plot(time_plot, p_m_z_centered, 'r-', 'LineWidth', 1.2); h3.Color(4) = 0.5;
    hold on; plot(time_plot, drift_z_centered, 'k-', 'LineWidth', 1.8); hold off;
    xlabel('Time (s)'); ylabel('\Deltap_{m,z} (nm)');
    xlim([0, duration_plot]); ylim([-y_limit, y_limit]);
    text(0.98, 0.95, sprintf('STD = %.2f nm', std_pm_z), 'Units', 'normalized', ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
        'BackgroundColor', 'white', 'EdgeColor', 'black');
    set(gca, 'FontSize', 12, 'FontWeight', 'bold', 'Box', 'on');

    exportgraphics(fig_export, fullfile(output_dir, '7_openloop_time_response.png'), 'Resolution', 150);
    close(fig_export);

    % Tab 8: FFT Spectrum
    fig_export = figure('Visible', 'off', 'Position', [100, 100, 1000, 800]);

    subplot(3,1,1);
    loglog(freq_pm_x(2:end), spectrum_pm_x(2:end), 'b-', 'LineWidth', 1.5);
    ylabel('X Amp. (nm)'); title('Position FFT Spectrum (Open-loop)');
    xlim([freq_pm_x(2), Fs/2]); ylim([y_min_fft, y_max_fft]);
    set(gca, 'FontSize', 12, 'FontWeight', 'bold', 'Box', 'on');

    subplot(3,1,2);
    loglog(freq_pm_y(2:end), spectrum_pm_y(2:end), 'g-', 'LineWidth', 1.5);
    ylabel('Y Amp. (nm)');
    xlim([freq_pm_x(2), Fs/2]); ylim([y_min_fft, y_max_fft]);
    set(gca, 'FontSize', 12, 'FontWeight', 'bold', 'Box', 'on');

    subplot(3,1,3);
    loglog(freq_pm_z(2:end), spectrum_pm_z(2:end), 'r-', 'LineWidth', 1.5);
    xlabel('Frequency (Hz)'); ylabel('Z Amp. (nm)');
    xlim([freq_pm_x(2), Fs/2]); ylim([y_min_fft, y_max_fft]);
    set(gca, 'FontSize', 12, 'FontWeight', 'bold', 'Box', 'on');

    exportgraphics(fig_export, fullfile(output_dir, '8_openloop_fft_spectrum.png'), 'Resolution', 150);
    close(fig_export);

    fprintf('\n  Open-loop figures saved (7, 8)\n');

end

%% SECTION 8: Closed-loop Thermal Force Analysis (as Tabs)
% Triggered when: ctrl_enable = true AND thermal_enable = true
% Analyzes p_m high-frequency components (thermal noise) using sliding window
% Verifies that position perturbation STD is independent of h/R (Einstein relation)

is_closedloop_thermal = (params.Value.ctrl.enable > 0.5) && ...
                        (params.Value.thermal.enable > 0.5);

if is_closedloop_thermal
    fprintf('\n');
    fprintf('================================================================\n');
    fprintf('  Closed-loop Thermal Force Analysis\n');
    fprintf('================================================================\n\n');

    % Get sample rate
    Fs = 1 / Ts;

    % Extract Z-axis tracking error (most relevant for wall effect analysis)
    error_z_nm = (p_m_log(3, :) - p_d_log(3, :))' * 1000;  % um -> nm

    % High-pass filter to extract thermal noise component
    fprintf('Applying high-pass filter (cutoff = %.1f Hz)...\n', closedloop_cutoff_freq);
    noise_z = highpass_fft(error_z_nm, Fs, closedloop_cutoff_freq);

    % Calculate overall STD
    overall_std = std(noise_z);
    fprintf('  Done. Overall STD = %.2f nm\n\n', overall_std);

    % ==================== Tab 9: Tracking Error (Time Domain) ====================
    tab9 = uitab(tabgroup, 'Title', 'CL Tracking Error');

    % Subplot positions
    fig_w = 1200; fig_h = 800;
    pos9_top = [0.10*fig_w, 0.55*fig_h, 0.85*fig_w, 0.38*fig_h];
    pos9_bot = [0.10*fig_w, 0.08*fig_h, 0.85*fig_w, 0.38*fig_h];

    % --- Top: High-frequency noise component only ---
    ax9_top = uiaxes(tab9, 'Position', pos9_top);
    plot(ax9_top, t_sample, noise_z, 'k-', 'LineWidth', 1.5);
    ylabel(ax9_top, 'High-freq noise [nm]', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
    title(ax9_top, sprintf('Z-Axis High-Frequency Noise (HP cutoff = %.0f Hz)', closedloop_cutoff_freq), ...
        'FontSize', title_fontsize, 'FontWeight', 'bold');
    ax9_top.LineWidth = axis_linewidth;
    ax9_top.FontSize = tick_fontsize;
    ax9_top.FontWeight = 'bold';
    ax9_top.Box = 'on';
    grid(ax9_top, 'on');
    xlim(ax9_top, [0, t_sample(end)]);

    % --- Bottom: h/R vs time ---
    ax9_bot = uiaxes(tab9, 'Position', pos9_bot);
    plot(ax9_bot, t_sample, h_bar_log, 'b-', 'LineWidth', line_width_main);
    xlabel(ax9_bot, 'Time [sec]', 'FontSize', xlabel_fontsize, 'FontWeight', 'bold');
    ylabel(ax9_bot, 'h/R', 'FontSize', ylabel_fontsize, 'FontWeight', 'bold');
    title(ax9_bot, 'Normalized Wall Distance', 'FontSize', title_fontsize, 'FontWeight', 'bold');
    ax9_bot.LineWidth = axis_linewidth;
    ax9_bot.FontSize = tick_fontsize;
    ax9_bot.FontWeight = 'bold';
    ax9_bot.Box = 'on';
    grid(ax9_bot, 'on');
    xlim(ax9_bot, [0, t_sample(end)]);

    % Add h/R range annotation
    text(ax9_bot, 0.98, 0.95, sprintf('h/R: %.2f ~ %.2f', min(h_bar_log), max(h_bar_log)), ...
        'Units', 'normalized', 'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'top', 'FontSize', legend_fontsize, 'FontWeight', 'bold', ...
        'BackgroundColor', [1 1 1 0.8], 'EdgeColor', [0.3 0.3 0.3], 'Margin', 5);

    fprintf('  Tab 9: Closed-loop Tracking Error\n');

    % ==================== Save Closed-loop Analysis Figures ====================
    % Tab 9: Tracking Error
    fig_export = figure('Visible', 'off', 'Position', [100, 100, 1000, 700]);

    subplot(2,1,1);
    plot(t_sample, noise_z, 'k-', 'LineWidth', 1.5);
    ylabel('High-freq noise [nm]');
    title(sprintf('Z-Axis High-Frequency Noise (HP cutoff = %.0f Hz)', closedloop_cutoff_freq));
    xlim([0, t_sample(end)]);
    text(0.98, 0.95, sprintf('Overall STD = %.2f nm', overall_std), 'Units', 'normalized', ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
        'BackgroundColor', 'white', 'EdgeColor', 'black');
    set(gca, 'FontSize', 12, 'FontWeight', 'bold', 'Box', 'on');
    grid on;

    subplot(2,1,2);
    plot(t_sample, h_bar_log, 'b-', 'LineWidth', 2);
    xlabel('Time [sec]');
    ylabel('h/R');
    title('Normalized Wall Distance');
    xlim([0, t_sample(end)]);
    text(0.98, 0.95, sprintf('h/R: %.2f ~ %.2f', min(h_bar_log), max(h_bar_log)), ...
        'Units', 'normalized', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
        'BackgroundColor', 'white', 'EdgeColor', 'black');
    set(gca, 'FontSize', 12, 'FontWeight', 'bold', 'Box', 'on');
    grid on;

    exportgraphics(fig_export, fullfile(output_dir, '9_closedloop_tracking_error.png'), 'Resolution', 150);
    close(fig_export);

    fprintf('\n  Closed-loop figures saved (9)\n');

    % Save closed-loop analysis data
    result.closedloop_analysis.noise_z = noise_z;
    result.closedloop_analysis.cutoff_freq = closedloop_cutoff_freq;
    result.closedloop_analysis.overall_std = overall_std;

    % Re-save result.mat with additional data
    save(fullfile(output_dir, 'result.mat'), 'result');
    fprintf('  Analysis data appended to result.mat\n');

end

%% ========== Local Functions ==========

function [std_noise, data_drift, data_noise, f, P1] = ...
    fft_drift_noise_separation(data, time, cutoff_freq)
% FFT_DRIFT_NOISE_SEPARATION - Separate drift and noise using FFT
%
%   Separates low-frequency drift and high-frequency noise components
%   using FFT-based filtering with endpoint extension for improved drift
%   estimation.
%
%   Inputs:
%       data        - Time series data (column vector)
%       time        - Time axis (column vector)
%       cutoff_freq - Cutoff frequency for drift/noise separation [Hz]
%
%   Outputs:
%       std_noise   - Standard deviation of noise component
%       data_drift  - Low-frequency drift component
%       data_noise  - High-frequency noise component
%       f           - Frequency axis for spectrum
%       P1          - Single-sided amplitude spectrum

    N = length(data);

    % Ensure N is even
    if mod(N, 2) == 1
        data = data(1:end-1);
        time = time(1:end-1);
        N = N - 1;
    end

    % Sample rate
    Fs = 1 / mean(diff(time));

    % ========== Stage 1: FFT on original data ==========
    data_mean = mean(data);
    data_demean = data - data_mean;

    % Direct FFT
    Y_original = fft(data_demean);

    % Frequency axis
    f = Fs * (0:(N/2)) / N;
    f = f(:);  % Ensure column vector

    % Single-sided amplitude spectrum
    P2 = abs(Y_original / N);
    P1 = P2(1:N/2+1);
    P1(2:end-1) = 2 * P1(2:end-1);

    % Find cutoff frequency index
    cutoff_idx = find(f <= cutoff_freq, 1, 'last');
    if isempty(cutoff_idx)
        cutoff_idx = 1;
    end

    % ========== Stage 2: Extract Noise ==========
    Y_highfreq = Y_original;
    Y_highfreq(1:cutoff_idx) = 0;
    Y_highfreq(N-cutoff_idx+2:N) = 0;

    % IFFT to get noise
    data_noise = ifft(Y_highfreq, 'symmetric');

    % Calculate noise standard deviation
    std_noise = std(data_noise);

    % ========== Stage 3: Extract Drift (with extension + windowing) ==========
    % Endpoint extension
    extend_len = round(N * 0.1);
    left_extend = linspace(0, data_demean(1), extend_len)';
    right_extend = linspace(data_demean(end), 0, extend_len)';
    data_extended = [left_extend; data_demean; right_extend];
    N_ext = length(data_extended);

    % Tukey window
    alpha = 0.1;
    window = tukeywin(N_ext, alpha);
    data_windowed = data_extended .* window;

    % FFT for drift
    Y_drift = fft(data_windowed);

    % Low-pass filter
    Y_drift_lowfreq = Y_drift;
    cutoff_idx_ext = round(cutoff_idx * N_ext / N);
    Y_drift_lowfreq(cutoff_idx_ext+1:N_ext-cutoff_idx_ext+1) = 0;

    % IFFT and extract original region
    drift_extended = ifft(Y_drift_lowfreq, 'symmetric');
    data_drift = drift_extended(extend_len+1:extend_len+N) + data_mean;

end


function data_hp = highpass_fft(data, Fs, cutoff_freq)
% HIGHPASS_FFT - High-pass filter using FFT
%
%   Extracts high-frequency components above the cutoff frequency.
%   Uses FFT-based filtering with smooth transition to avoid ringing.
%
%   Inputs:
%       data        - Time series data (column vector)
%       Fs          - Sample rate [Hz]
%       cutoff_freq - High-pass cutoff frequency [Hz]
%
%   Outputs:
%       data_hp     - High-pass filtered data

    N = length(data);

    % Ensure even length
    if mod(N, 2) == 1
        data = [data; data(end)];
        N = N + 1;
        truncate = true;
    else
        truncate = false;
    end

    % Remove mean
    data_mean = mean(data);
    data_demean = data - data_mean;

    % FFT
    Y = fft(data_demean);

    % Frequency axis
    f = (0:N-1)' * Fs / N;

    % Create high-pass filter (smooth transition using cosine taper)
    transition_width = cutoff_freq * 0.3;  % 30% of cutoff as transition band
    H = zeros(N, 1);
    for k = 1:N
        freq = f(k);
        if freq > Fs/2
            freq = Fs - freq;  % Mirror for negative frequencies
        end

        if freq >= cutoff_freq
            H(k) = 1;
        elseif freq >= cutoff_freq - transition_width
            % Smooth cosine transition
            H(k) = 0.5 * (1 - cos(pi * (freq - (cutoff_freq - transition_width)) / transition_width));
        else
            H(k) = 0;
        end
    end

    % Apply filter
    Y_hp = Y .* H;

    % IFFT
    data_hp = ifft(Y_hp, 'symmetric');

    % Truncate if needed
    if truncate
        data_hp = data_hp(1:end-1);
    end

end
