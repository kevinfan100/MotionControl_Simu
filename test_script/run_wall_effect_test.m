%RUN_WALL_EFFECT_TEST Main script for wall effect simulation
%
%   1. Set parameters here
%   2. Run this script to initialize workspace
%   3. Then run Simulink model
%
%   Reference: Drag_MATLAB.pdf

clear; close all; clc;

%% ========== Parameter Settings (modify here) ==========
theta   = pi/6;            % Inclined plane angle around z-axis [rad]
phi     = pi/6;         % Inclined plane angle around x-axis [rad] (30 deg inclined plane)
p_z     = 5;            % Distance from origin to inclined plane [um]
Ts      = 1/1606;       % Sampling time [sec]
p0      = [0; 0; 10];   % Initial position [um], 3x1 vector (start at z=10um)
%% ======================================================

%% Add path (use script location to find project root)
scriptDir = fileparts(mfilename('fullpath'));  % Get this script's directory
projectRoot = fileparts(scriptDir);            % Go up one level to project root
addpath(fullfile(projectRoot, 'model', 'wall_effect'));

%% Calculate fixed parameters (w_hat, W, etc.)
params = calc_wall_params(theta, phi, p_z, Ts, p0);

%% ========== f_T Calculation (TODO) ==========
% f_T depends on Gamma (drag coefficient matrix)
% Placeholder: f_T = zeros(3,1) for now
% TODO: Implement f_T calculation based on Gamma
%% ============================================

%% Display
fprintf('\nParameters loaded into workspace.\n');
fprintf('You can now run the Simulink model.\n');
fprintf('\nSimulink block inputs:\n');
fprintf('  - f_M: Magnetic force (from Control Law)\n');
fprintf('  - f_T: Thermal force (TODO, currently zeros)\n');
fprintf('  - p: Position feedback (from Integrator)\n');
fprintf('  - params: Parameter struct (from workspace)\n');


% Get params data from Simulink.Parameter
params_data = params.Value;

[p_dot_test, Gamma_inv_test, h_bar_test] = wall_effect_integrated(f_test, p_test, params_data);

fprintf('Test input:\n');
fprintf('  f = [%.2f; %.2f; %.2f] pN\n', f_test(1), f_test(2), f_test(3));
fprintf('  p = [%.2f; %.2f; %.2f] um\n', p_test(1), p_test(2), p_test(3));
fprintf('\nTest output:\n');
fprintf('  h_bar = %.4f (normalized distance to wall)\n', h_bar_test);
fprintf('  p_dot = [%.4f; %.4f; %.4f] um/sec\n', p_dot_test(1), p_dot_test(2), p_dot_test(3));
fprintf('==================\n');

%% ========== Run Simulink Simulation ==========
fprintf('\n=== Running Simulink Simulation ===\n');

% Model path
modelPath = fullfile(projectRoot, 'model', 'system_model.slx');

% Check if model exists
if ~exist(modelPath, 'file')
    error('Model not found: %s\nRun init_model.m first.', modelPath);
end

% Load model (don't open window)
load_system(modelPath);

% Run simulation
simOut = sim('system_model');

fprintf('Simulation completed.\n');

%% ========== Plot Results ==========
fprintf('\n=== Plotting Results ===\n');

% Get data from simOut (simulation output object)
% Try to get p_sim from simOut first, then from workspace
if isfield(simOut, 'p_sim') || isprop(simOut, 'p_sim')
    p_sim = simOut.p_sim;
elseif evalin('base', 'exist(''p_sim'', ''var'')')
    p_sim = evalin('base', 'p_sim');
else
    % Check what's in simOut
    fprintf('Available outputs in simOut:\n');
    disp(simOut.who);
    error('p_sim not found. Check To Workspace block settings.');
end

t = p_sim.Time;
p_data = squeeze(p_sim.Data);  % 3 x N or N x 3

% Handle different data orientations
if size(p_data, 1) == 3
    % p_data is 3 x N, correct
elseif size(p_data, 2) == 3
    % p_data is N x 3, transpose it
    p_data = p_data';
end

% Create figure for position
figure('Name', 'Position vs Time');

subplot(3,1,1);
plot(t, p_data(1,:), 'LineWidth', 1.5);
xlabel('Time [sec]');
ylabel('p_x [um]');
title('Position X');
grid on;

subplot(3,1,2);
plot(t, p_data(2,:), 'LineWidth', 1.5);
xlabel('Time [sec]');
ylabel('p_y [um]');
title('Position Y');
grid on;

subplot(3,1,3);
plot(t, p_data(3,:), 'LineWidth', 1.5);
xlabel('Time [sec]');
ylabel('p_z [um]');
title('Position Z');
grid on;

% Create figure for Gamma_inv (9 elements of 3x3 matrix)
% Try to get Gamma_inv_sim from simOut first, then from workspace
Gamma_inv_sim = [];
if isfield(simOut, 'Gamma_inv_sim') || isprop(simOut, 'Gamma_inv_sim')
    Gamma_inv_sim = simOut.Gamma_inv_sim;
elseif evalin('base', 'exist(''Gamma_inv_sim'', ''var'')')
    Gamma_inv_sim = evalin('base', 'Gamma_inv_sim');
end

if ~isempty(Gamma_inv_sim)
    figure('Name', 'Gamma_inv vs Time');

    Gamma_data = squeeze(Gamma_inv_sim.Data);  % 3 x 3 x N or other
    t_gamma = Gamma_inv_sim.Time;

    % Handle different data orientations
    dims = size(Gamma_data);
    if length(dims) == 3
        if dims(1) == 3 && dims(2) == 3
            % Gamma_data is 3 x 3 x N, correct
        else
            % Try to reshape
            Gamma_data = permute(Gamma_data, [2, 3, 1]);
        end
    end

    for i = 1:3
        for j = 1:3
            subplot(3,3,(i-1)*3+j);
            plot(t_gamma, squeeze(Gamma_data(i,j,:)), 'LineWidth', 1.5);
            xlabel('Time [sec]');
            ylabel(sprintf('\\Gamma^{-1}_{%d%d}', i, j));
            grid on;
        end
    end
    sgtitle('Inverse Drag Coefficient Matrix \Gamma^{-1}');
else
    fprintf('Warning: Gamma_inv_sim not found.\n');
    fprintf('Available outputs in simOut:\n');
    disp(simOut.who);
    fprintf('Make sure To Workspace block variable name is "Gamma_inv_sim".\n');
end

fprintf('Plots generated.\n');
