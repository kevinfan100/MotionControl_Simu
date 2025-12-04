%RUN_WALL_EFFECT_TEST Main script for wall effect simulation
%
%   1. Set parameters here
%   2. Run this script to initialize workspace
%   3. Then run Simulink model
%
%   Reference: Drag_MATLAB.pdf

clear; close all; clc;

%% ========== Parameter Settings (modify here) ==========
theta   = 0;            % Inclined plane angle around z-axis [rad]
phi     = pi/6;         % Inclined plane angle around x-axis [rad] (30 deg inclined plane)
p_z     = 5;            % Distance from origin to inclined plane [um]
Ts      = 1/1606;       % Sampling time [sec]
p0      = [0; 0; 10];   % Initial position [um], 3x1 vector (start at z=10um) 
%% ======================================================

%% Add path (go up one level from test_script to project root)
projectRoot = fileparts(pwd);
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

%% ========== Quick Test (print p_dot for initial position) ==========
fprintf('\n=== Quick Test ===\n');
f_test = [0; 0; -1];  % Test force (downward)
p_test = p0;          % Initial position

% Get params data from Simulink.Parameter
params_data = params.Value;

[p_dot_test, Gamma_inv_test] = wall_effect_integrated(f_test, p_test, params_data);

% Save Gamma_inv to workspace
Gamma_inv = Gamma_inv_test;

%% result
fprintf('Test input:\n');
fprintf('  f = [%.2f; %.2f; %.2f] pN\n', f_test(1), f_test(2), f_test(3));
fprintf('  p = [%.2f; %.2f; %.2f] um\n', p_test(1), p_test(2), p_test(3));
fprintf('\nTest output:\n');
fprintf('  p_dot = [%.4f; %.4f; %.4f] um/sec\n', p_dot_test(1), p_dot_test(2), p_dot_test(3));
fprintf('==================\n');
