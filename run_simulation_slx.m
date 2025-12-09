%% run_simulation_slx.m - Simulink-based Simulation Script
%
% This script runs the motion control simulation using Simulink.
% If Simulink model is not properly configured, it falls back to pure MATLAB.
%
% Prerequisites:
%   1. Run create_simulation_buses() once to create Bus Objects
%   2. Build and configure system_model.slx (run build_system_model.m)

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
end

%% SECTION 4: Setup for Simulink
fprintf('\nPreparing Simulink simulation...\n');

% Create Bus Objects
try
    create_simulation_buses();
catch ME
    warning('Failed to create Bus Objects: %s', ME.message);
end

% Convert params for Simulink (string -> numeric encoding)
params_slx = convert_params_for_simulink(params);

% Assign to base workspace for Simulink
assignin('base', 'params_slx', params_slx);
assignin('base', 'p0', p0);
assignin('base', 'T_sim', T_sim);

%% SECTION 5: Run Simulation
model_path = fullfile('model', 'system_model.slx');
use_simulink = false;

% Try to use Simulink
if exist(model_path, 'file')
    try
        fprintf('Attempting to run Simulink model...\n');

        % Load model
        load_system('model/system_model');

        % Reset controller persistent variable
        clear motion_control_law

        % Run simulation
        simOut = sim('model/system_model', 'StopTime', num2str(T_sim));

        % Extract results
        t = simOut.tout;
        p_m = simOut.p_m_out';
        p_d = simOut.p_d_out';
        f_d = simOut.f_d_out';

        use_simulink = true;
        fprintf('Simulink simulation completed successfully.\n');

        % Close model
        close_system('model/system_model', 0);

    catch ME
        warning('Simulink simulation failed: %s\nFalling back to pure MATLAB...', ME.message);
        use_simulink = false;
    end
end

% Fallback to pure MATLAB simulation
if ~use_simulink
    fprintf('Running pure MATLAB simulation...\n');

    % Reset controller
    clear motion_control_law

    % Simulation parameters
    Ts = params.common.Ts;
    t_sample = 0:Ts:T_sim;
    N_samples = length(t_sample);

    % Pre-allocate
    p_m_log = zeros(3, N_samples);
    p_d_log = zeros(3, N_samples);
    f_d_log = zeros(3, N_samples);

    % Initialize
    p_m = p0;

    % Main loop
    for k = 1:N_samples
        t_k = t_sample(k);

        % Trajectory
        p_d = trajectory_generator(t_k, p0, params);

        % Control
        f_d = motion_control_law(p_d, p_m, params);

        % Thermal force
        if params.thermal.enable
            F_th = calc_thermal_force(p_m, params);
        else
            F_th = zeros(3, 1);
        end

        % Log
        p_m_log(:, k) = p_m;
        p_d_log(:, k) = p_d;
        f_d_log(:, k) = f_d;

        % Dynamics
        if k < N_samples
            [Gamma_inv, ~] = calc_gamma_inv(p_m, params);
            F_total = f_d + F_th;
            p_m = p_m + (Gamma_inv * F_total) * Ts;
        end
    end

    % Format output
    t = t_sample';
    p_m = p_m_log;
    p_d = p_d_log;
    f_d = f_d_log;

    fprintf('MATLAB simulation completed.\n');
end

%% SECTION 6: Calculate h_bar
w_hat = params.wall.w_hat;
pz_wall = params.wall.pz;
R = params.common.R;

h_bar = zeros(1, size(p_m, 2));
for i = 1:size(p_m, 2)
    h = dot(p_m(:, i), w_hat) - pz_wall;
    h_bar(i) = h / R;
end

%% SECTION 7: Results Visualization
figure('Name', 'Simulation Results', 'Position', [100 100 1200 800]);

% Subplot 1: 3D Trajectory
subplot(2, 2, 1);
plot3(p_d(1, :), p_d(2, :), p_d(3, :), 'b-', 'LineWidth', 2);
hold on;
plot3(p_m(1, :), p_m(2, :), p_m(3, :), 'r--', 'LineWidth', 1.5);
plot3(p0(1), p0(2), p0(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
hold off;
legend('p_d (desired)', 'p_m (measured)', 'Start', 'Location', 'best');
xlabel('x [um]'); ylabel('y [um]'); zlabel('z [um]');
title('3D Trajectory');
grid on; axis equal; view(30, 30);

% Subplot 2: Tracking Error
subplot(2, 2, 2);
error = vecnorm(p_m - p_d, 2, 1);
plot(t, error, 'k-', 'LineWidth', 1.5);
xlabel('Time [sec]'); ylabel('||e|| [um]');
title(sprintf('Tracking Error (RMSE = %.4f um)', rms(error)));
grid on;

% Subplot 3: Control Force
subplot(2, 2, 3);
plot(t, f_d(1, :), 'r-', 'LineWidth', 1);
hold on;
plot(t, f_d(2, :), 'g-', 'LineWidth', 1);
plot(t, f_d(3, :), 'b-', 'LineWidth', 1.5);
hold off;
legend('f_x', 'f_y', 'f_z', 'Location', 'best');
xlabel('Time [sec]'); ylabel('Force [pN]');
title('Control Force');
grid on;

% Subplot 4: h/R Safety
subplot(2, 2, 4);
plot(t, h_bar, 'b-', 'LineWidth', 1.5);
hold on;
yline(params.wall.h_bar_min, 'r--', 'LineWidth', 1.5);
hold off;
legend('h/R(t)', sprintf('h/R_{min}=%.1f', params.wall.h_bar_min), 'Location', 'best');
xlabel('Time [sec]'); ylabel('h/R');
title('Distance from Wall');
grid on;

%% SECTION 8: Save Results
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
output_dir = fullfile('test_results', 'simulation', ['sim_slx_' timestamp]);
mkdir(output_dir);

savefig(fullfile(output_dir, 'results.fig'));
exportgraphics(gcf, fullfile(output_dir, 'results.png'), 'Resolution', 150);

result.t = t;
result.p_m = p_m;
result.p_d = p_d;
result.f_d = f_d;
result.h_bar = h_bar;
result.params = params;
result.p0 = p0;
result.config = config;
result.use_simulink = use_simulink;
result.tracking_error_rmse = rms(error);

save(fullfile(output_dir, 'result.mat'), 'result');

%% Print Summary
fprintf('\n=== Simulation Summary ===\n');
fprintf('Method: %s\n', ternary(use_simulink, 'Simulink', 'Pure MATLAB'));
fprintf('Mode: %s\n', ternary(params.ctrl.enable, 'Closed-loop', 'Open-loop'));
fprintf('Thermal: %s\n', ternary(params.thermal.enable, 'Enabled', 'Disabled'));
fprintf('Trajectory: %s\n', params.traj.type);
fprintf('Duration: %.1f sec\n', T_sim);
fprintf('\nTracking Performance:\n');
fprintf('  RMSE: %.4f um\n', rms(error));
fprintf('  Max error: %.4f um\n', max(error));
fprintf('\nSafety:\n');
fprintf('  Min h/R: %.2f (threshold: %.1f)\n', min(h_bar), params.wall.h_bar_min);
fprintf('\nResults saved to: %s\n', output_dir);

%% Helper function
function s = ternary(cond, true_val, false_val)
    if cond
        s = true_val;
    else
        s = false_val;
    end
end
