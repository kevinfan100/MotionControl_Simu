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
h_bar_min = 1.5;    % Minimum safe normalized distance

% === Trajectory Parameters ===
traj_type = 'z_move';   % 'z_move' or 'xy_circle'
h_margin = 5;           % Safety margin [um]
delta_z = 10;           % z_move: travel distance [um]
direction = 'away';     % z_move: 'away' or 'toward'
speed = 5;              % z_move: travel speed [um/sec]
radius = 5;             % xy_circle: radius [um]
period = 1;             % xy_circle: period [sec]
n_circles = 3;          % xy_circle: number of circles

% === Controller Parameters ===
ctrl_enable = true;     % true = closed-loop, false = open-loop
lambda_c = 0.7;         % Closed-loop pole (0 < lambda_c < 1)

% === Thermal Force ===
thermal_enable = true;  % Enable Brownian motion disturbance

% === Simulation Parameters ===
T_sim = 5;              % Simulation time [sec]

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
addpath('model/wall_effect');
addpath('model/thermal_force');
addpath('model/trajectory');
addpath('model/controller');

% Calculate simulation parameters
params = calc_simulation_params(config);

%% SECTION 3: Calculate Initial Position and Safety Check
p0 = calc_initial_position(params);

fprintf('Initial position: [%.3f, %.3f, %.3f] um\n', p0);
fprintf('Initial h/R: %.2f\n', (dot(p0, params.wall.w_hat) - params.wall.pz) / params.common.R);

% Safety check
[is_safe, h_bar_min_actual, t_critical] = check_trajectory_safety(p0, params);

if ~is_safe
    warning('Trajectory unsafe! Min h/R = %.2f at t = %.3f sec', h_bar_min_actual, t_critical);
    fprintf('Continuing anyway for demonstration...\n');
end

%% SECTION 4: Run Simulation
fprintf('\nStarting simulation...\n');
fprintf('  Mode: %s\n', ternary(params.ctrl.enable, 'Closed-loop', 'Open-loop'));
fprintf('  Thermal: %s\n', ternary(params.thermal.enable, 'Enabled', 'Disabled'));
fprintf('  Trajectory: %s\n', params.traj.type);
fprintf('  Duration: %.1f sec\n', T_sim);

% Reset controller persistent variable
clear motion_control_law

% Simulation time parameters
Ts = params.common.Ts;
t_sample = 0:Ts:T_sim;
N_samples = length(t_sample);

% Pre-allocate output arrays
p_m_log = zeros(3, N_samples);      % Measured position
p_d_log = zeros(3, N_samples);      % Desired position
f_d_log = zeros(3, N_samples);      % Control force
F_th_log = zeros(3, N_samples);     % Thermal force
h_bar_log = zeros(1, N_samples);    % Normalized distance

% Initialize
p_m = p0;

% Main simulation loop
for k = 1:N_samples
    t = t_sample(k);

    % 1. Generate desired trajectory
    p_d = trajectory_generator(t, p0, params);

    % 2. Compute control force
    f_d = motion_control_law(p_d, p_m, params);

    % 3. Generate thermal force
    if params.thermal.enable
        F_th = calc_thermal_force(p_m, params);
    else
        F_th = zeros(3, 1);
    end

    % 4. Total force
    F_total = f_d + F_th;

    % Log data before dynamics update
    p_m_log(:, k) = p_m;
    p_d_log(:, k) = p_d;
    f_d_log(:, k) = f_d;
    F_th_log(:, k) = F_th;
    h = dot(p_m, params.wall.w_hat) - params.wall.pz;
    h_bar_log(k) = h / params.common.R;

    % 5. Particle dynamics: integrate over one sample period
    % Using simple Euler integration for discrete-time approximation
    % (Matches the ZOH + integrator in Simulink)
    if k < N_samples
        [Gamma_inv, ~] = calc_gamma_inv(p_m, params);
        p_dot = Gamma_inv * F_total;
        p_m = p_m + p_dot * Ts;
    end
end

fprintf('Simulation completed.\n');

%% SECTION 5: Results Visualization
figure('Name', 'Simulation Results', 'Position', [100 100 1200 800]);

% Subplot 1: 3D Trajectory
subplot(2, 2, 1);
plot3(p_d_log(1, :), p_d_log(2, :), p_d_log(3, :), 'b-', 'LineWidth', 2);
hold on;
plot3(p_m_log(1, :), p_m_log(2, :), p_m_log(3, :), 'r--', 'LineWidth', 1.5);
plot3(p0(1), p0(2), p0(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
hold off;
legend('p_d (desired)', 'p_m (measured)', 'Start', 'Location', 'best');
xlabel('x [um]'); ylabel('y [um]'); zlabel('z [um]');
title('3D Trajectory');
grid on; axis equal; view(30, 30);

% Subplot 2: Tracking Error
subplot(2, 2, 2);
error = vecnorm(p_m_log - p_d_log, 2, 1);
plot(t_sample, error, 'k-', 'LineWidth', 1.5);
xlabel('Time [sec]'); ylabel('||e|| [um]');
title(sprintf('Tracking Error (RMSE = %.4f um)', rms(error)));
grid on;

% Subplot 3: Control Force
subplot(2, 2, 3);
plot(t_sample, f_d_log(1, :), 'r-', 'LineWidth', 1);
hold on;
plot(t_sample, f_d_log(2, :), 'g-', 'LineWidth', 1);
plot(t_sample, f_d_log(3, :), 'b-', 'LineWidth', 1.5);
hold off;
legend('f_x', 'f_y', 'f_z', 'Location', 'best');
xlabel('Time [sec]'); ylabel('Force [pN]');
title('Control Force');
grid on;

% Subplot 4: h/R Safety
subplot(2, 2, 4);
plot(t_sample, h_bar_log, 'b-', 'LineWidth', 1.5);
hold on;
yline(params.wall.h_bar_min, 'r--', 'LineWidth', 1.5);
hold off;
legend('h/R(t)', sprintf('h/R_{min}=%.1f', params.wall.h_bar_min), 'Location', 'best');
xlabel('Time [sec]'); ylabel('h/R');
title('Distance from Wall');
grid on;

%% SECTION 6: Save Results
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
output_dir = fullfile('test_results', 'simulation', ['sim_' timestamp]);
mkdir(output_dir);

% Save figure
savefig(fullfile(output_dir, 'results.fig'));
exportgraphics(gcf, fullfile(output_dir, 'results.png'), 'Resolution', 150);

% Save data
result.t = t_sample;
result.p_m = p_m_log;
result.p_d = p_d_log;
result.f_d = f_d_log;
result.F_th = F_th_log;
result.h_bar = h_bar_log;
result.params = params;
result.p0 = p0;
result.config = config;
result.tracking_error_rmse = rms(error);

save(fullfile(output_dir, 'result.mat'), 'result');

%% Print Summary
fprintf('\n=== Simulation Summary ===\n');
fprintf('Mode: %s\n', ternary(params.ctrl.enable, 'Closed-loop', 'Open-loop'));
fprintf('Thermal: %s\n', ternary(params.thermal.enable, 'Enabled', 'Disabled'));
fprintf('Trajectory: %s\n', params.traj.type);
fprintf('Duration: %.1f sec (%d samples)\n', T_sim, N_samples);
fprintf('\nTracking Performance:\n');
fprintf('  RMSE: %.4f um\n', rms(error));
fprintf('  Max error: %.4f um\n', max(error));
fprintf('  Final error: %.4f um\n', error(end));
fprintf('\nSafety:\n');
fprintf('  Min h/R: %.2f (threshold: %.1f)\n', min(h_bar_log), params.wall.h_bar_min);
fprintf('  Max h/R: %.2f\n', max(h_bar_log));
fprintf('\nControl Force (z-component):\n');
fprintf('  Max: %.4f pN\n', max(abs(f_d_log(3, :))));
fprintf('  Mean: %.4f pN\n', mean(abs(f_d_log(3, :))));
fprintf('\nResults saved to: %s\n', output_dir);

%% Helper function
function s = ternary(cond, true_val, false_val)
    if cond
        s = true_val;
    else
        s = false_val;
    end
end
