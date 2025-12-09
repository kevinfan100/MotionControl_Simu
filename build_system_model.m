%% build_system_model.m - Build Simulink Model Programmatically
%
% This script creates the system_model.slx Simulink model from scratch.
% Run this script once to generate the model, then it can be used for simulation.
%
% Model Architecture:
%   - Trajectory Generator (MATLAB Function, Ts=1/1606)
%   - Controller (MATLAB Function, Ts=1/1606)
%   - Thermal Force (MATLAB Function, Ts=1/1606)
%   - Particle Dynamics (MATLAB Function, continuous)
%   - Integrator (1/s, IC=p0)

clear; close all; clc;

%% Configuration
model_name = 'system_model';
model_path = fullfile('model', [model_name, '.slx']);
Ts = 1/1606;  % Sampling period

%% Delete existing model if it exists
if exist(model_path, 'file')
    delete(model_path);
    fprintf('Deleted existing model: %s\n', model_path);
end

% Close model if open
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

%% Create new model
new_system(model_name);
open_system(model_name);
fprintf('Created new model: %s\n', model_name);

%% Set model parameters
set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'SolverType', 'Variable-step');
set_param(model_name, 'MaxStep', '6e-5');
set_param(model_name, 'RelTol', '1e-6');
set_param(model_name, 'StopTime', 'T_sim');
set_param(model_name, 'SaveTime', 'on');
set_param(model_name, 'TimeSaveName', 't_out');
set_param(model_name, 'SaveOutput', 'on');
set_param(model_name, 'OutputSaveName', 'yout');

%% Add blocks

% === Clock ===
add_block('simulink/Sources/Clock', [model_name '/Clock']);
set_param([model_name '/Clock'], 'Position', [50, 200, 80, 230]);

% === Constant blocks for params and p0 ===
add_block('simulink/Sources/Constant', [model_name '/params_const']);
set_param([model_name '/params_const'], 'Position', [50, 50, 120, 80]);
set_param([model_name '/params_const'], 'Value', 'params_slx');
set_param([model_name '/params_const'], 'OutDataTypeStr', 'Bus: ParamsBus');
set_param([model_name '/params_const'], 'SampleTime', 'inf');

add_block('simulink/Sources/Constant', [model_name '/p0_const']);
set_param([model_name '/p0_const'], 'Position', [50, 130, 120, 160]);
set_param([model_name '/p0_const'], 'Value', 'p0');

% === Trajectory Generator (MATLAB Function) ===
add_block('simulink/User-Defined Functions/MATLAB Function', [model_name '/Trajectory_Generator']);
set_param([model_name '/Trajectory_Generator'], 'Position', [200, 170, 350, 250]);

% Get the MATLAB Function block handle and set code
traj_block = [model_name '/Trajectory_Generator'];
traj_code = sprintf([...
    'function p_d = trajectory_fcn(t, p0, params)\n' ...
    '%%#codegen\n' ...
    '    %% Convert numeric type back to string for trajectory_generator\n' ...
    '    params_m = params;\n' ...
    '    if params.traj.type == 0\n' ...
    '        params_m.traj.type = ''z_move'';\n' ...
    '    else\n' ...
    '        params_m.traj.type = ''xy_circle'';\n' ...
    '    end\n' ...
    '    if params.traj.direction == 0\n' ...
    '        params_m.traj.direction = ''away'';\n' ...
    '    else\n' ...
    '        params_m.traj.direction = ''toward'';\n' ...
    '    end\n' ...
    '    p_d = trajectory_generator(t, p0, params_m);\n' ...
    'end\n']);

% === Controller (MATLAB Function) ===
add_block('simulink/User-Defined Functions/MATLAB Function', [model_name '/Controller']);
set_param([model_name '/Controller'], 'Position', [450, 170, 600, 250]);

ctrl_code = sprintf([...
    'function f_d = controller_fcn(p_d, p_m, params)\n' ...
    '%%#codegen\n' ...
    '    %% Convert enable to logical\n' ...
    '    params_m = params;\n' ...
    '    params_m.ctrl.enable = logical(params.ctrl.enable);\n' ...
    '    f_d = motion_control_law(p_d, p_m, params_m);\n' ...
    'end\n']);

% === Thermal Force (MATLAB Function) ===
add_block('simulink/User-Defined Functions/MATLAB Function', [model_name '/Thermal_Force']);
set_param([model_name '/Thermal_Force'], 'Position', [450, 300, 600, 380]);

thermal_code = sprintf([...
    'function F_th = thermal_fcn(p_m, params)\n' ...
    '%%#codegen\n' ...
    '    if params.thermal.enable > 0.5\n' ...
    '        F_th = calc_thermal_force(p_m, params);\n' ...
    '    else\n' ...
    '        F_th = zeros(3, 1);\n' ...
    '    end\n' ...
    'end\n']);

% === Sum block (f_d + F_th) ===
add_block('simulink/Math Operations/Add', [model_name '/Sum_Forces']);
set_param([model_name '/Sum_Forces'], 'Position', [700, 220, 730, 280]);
set_param([model_name '/Sum_Forces'], 'Inputs', '++');

% === Particle Dynamics (MATLAB Function) ===
add_block('simulink/User-Defined Functions/MATLAB Function', [model_name '/Particle_Dynamics']);
set_param([model_name '/Particle_Dynamics'], 'Position', [800, 200, 950, 280]);

dynamics_code = sprintf([...
    'function p_dot = dynamics_fcn(F_total, p_m, params)\n' ...
    '%%#codegen\n' ...
    '    [Gamma_inv, ~] = calc_gamma_inv(p_m, params);\n' ...
    '    p_dot = Gamma_inv * F_total;\n' ...
    'end\n']);

% === Integrator ===
add_block('simulink/Continuous/Integrator', [model_name '/Integrator']);
set_param([model_name '/Integrator'], 'Position', [1020, 220, 1070, 260]);
set_param([model_name '/Integrator'], 'InitialCondition', 'p0');

% === To Workspace blocks ===
add_block('simulink/Sinks/To Workspace', [model_name '/p_m_out']);
set_param([model_name '/p_m_out'], 'Position', [1150, 230, 1220, 260]);
set_param([model_name '/p_m_out'], 'VariableName', 'p_m_out');
set_param([model_name '/p_m_out'], 'SaveFormat', 'Array');

add_block('simulink/Sinks/To Workspace', [model_name '/p_d_out']);
set_param([model_name '/p_d_out'], 'Position', [450, 100, 520, 130]);
set_param([model_name '/p_d_out'], 'VariableName', 'p_d_out');
set_param([model_name '/p_d_out'], 'SaveFormat', 'Array');

add_block('simulink/Sinks/To Workspace', [model_name '/f_d_out']);
set_param([model_name '/f_d_out'], 'Position', [700, 140, 770, 170]);
set_param([model_name '/f_d_out'], 'VariableName', 'f_d_out');
set_param([model_name '/f_d_out'], 'SaveFormat', 'Array');

%% Add lines (connections)
% Note: Line routing is complex, so we use add_line with autorouting

% Clock -> Trajectory Generator
add_line(model_name, 'Clock/1', 'Trajectory_Generator/1', 'autorouting', 'smart');

% p0 -> Trajectory Generator
add_line(model_name, 'p0_const/1', 'Trajectory_Generator/2', 'autorouting', 'smart');

% params -> Trajectory Generator
add_line(model_name, 'params_const/1', 'Trajectory_Generator/3', 'autorouting', 'smart');

% Trajectory Generator -> Controller
add_line(model_name, 'Trajectory_Generator/1', 'Controller/1', 'autorouting', 'smart');

% Trajectory Generator -> p_d_out
add_line(model_name, 'Trajectory_Generator/1', 'p_d_out/1', 'autorouting', 'smart');

% params -> Controller
add_line(model_name, 'params_const/1', 'Controller/3', 'autorouting', 'smart');

% params -> Thermal Force
add_line(model_name, 'params_const/1', 'Thermal_Force/2', 'autorouting', 'smart');

% params -> Particle Dynamics
add_line(model_name, 'params_const/1', 'Particle_Dynamics/3', 'autorouting', 'smart');

% Controller -> Sum_Forces
add_line(model_name, 'Controller/1', 'Sum_Forces/1', 'autorouting', 'smart');

% Controller -> f_d_out
add_line(model_name, 'Controller/1', 'f_d_out/1', 'autorouting', 'smart');

% Thermal Force -> Sum_Forces
add_line(model_name, 'Thermal_Force/1', 'Sum_Forces/2', 'autorouting', 'smart');

% Sum_Forces -> Particle Dynamics
add_line(model_name, 'Sum_Forces/1', 'Particle_Dynamics/1', 'autorouting', 'smart');

% Particle Dynamics -> Integrator
add_line(model_name, 'Particle_Dynamics/1', 'Integrator/1', 'autorouting', 'smart');

% Integrator -> p_m_out
add_line(model_name, 'Integrator/1', 'p_m_out/1', 'autorouting', 'smart');

% Feedback: Integrator -> Controller (p_m input)
add_line(model_name, 'Integrator/1', 'Controller/2', 'autorouting', 'smart');

% Feedback: Integrator -> Thermal Force
add_line(model_name, 'Integrator/1', 'Thermal_Force/1', 'autorouting', 'smart');

% Feedback: Integrator -> Particle Dynamics
add_line(model_name, 'Integrator/1', 'Particle_Dynamics/2', 'autorouting', 'smart');

%% Save model
save_system(model_name, model_path);
fprintf('Saved model to: %s\n', model_path);

%% Print instructions
fprintf('\n=== Simulink Model Built Successfully ===\n');
fprintf('\nIMPORTANT: You need to manually configure the MATLAB Function blocks.\n');
fprintf('For each MATLAB Function block, double-click to open and paste the corresponding code:\n\n');

fprintf('1. Trajectory_Generator:\n');
fprintf('----------------------------------------\n');
fprintf('%s\n', traj_code);

fprintf('2. Controller:\n');
fprintf('----------------------------------------\n');
fprintf('%s\n', ctrl_code);

fprintf('3. Thermal_Force:\n');
fprintf('----------------------------------------\n');
fprintf('%s\n', thermal_code);

fprintf('4. Particle_Dynamics:\n');
fprintf('----------------------------------------\n');
fprintf('%s\n', dynamics_code);

fprintf('\nAfter configuring MATLAB Function blocks, save the model.\n');
fprintf('Then use run_simulation_slx.m to run simulations.\n');
