%INIT_MODEL Create Simulink model for wall effect simulation
%
%   Run this script ONCE to generate system_model.slx
%   After that, you can open system_model.slx directly.
%
%   Model structure:
%
%   f_M (discrete) ---> [ZOH] ---+
%                                |
%                               [+] ---> [Gamma_inv_Block] ---> [1/s] ---> p
%                                |            p_dot             Integrator
%   f_T (discrete) ---> [ZOH] ---+              ^                   |
%                                               |                   |
%                                               +-------------------+
%                                                    p (feedback)
%
%   p (continuous) ---> [Sampler] ---> p_discrete (to Control Law, f_T calc)

%% Setup paths
modelName = 'system_model';
scriptDir = fileparts(mfilename('fullpath'));
modelPath = fullfile(scriptDir, [modelName '.slx']);

%% Check if model already exists
if exist(modelPath, 'file')
    fprintf('Model already exists: %s\n', modelPath);
    fprintf('Opening existing model...\n');
    open_system(modelPath);
    return;  % Exit script, don't recreate
end

%% Close existing model if open in memory
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

%% Create new model
fprintf('Creating new model: %s\n', modelPath);
new_system(modelName);
open_system(modelName);

%% Set model parameters (continuous solver for integration)
set_param(modelName, 'Solver', 'ode45');
set_param(modelName, 'StopTime', '10');

%% ==================== Add Blocks ====================

%% Input: f_M (magnetic force) - discrete, from Control Law
add_block('simulink/Sources/Constant', [modelName '/f_M_discrete']);
set_param([modelName '/f_M_discrete'], 'Value', '[1; 0; 0]');  % Test input
set_param([modelName '/f_M_discrete'], 'Position', [50, 100, 100, 130]);

%% Input: f_T (thermal force) - discrete, TODO
add_block('simulink/Sources/Constant', [modelName '/f_T_discrete']);
set_param([modelName '/f_T_discrete'], 'Value', '[0; 0; 0]');  % Placeholder
set_param([modelName '/f_T_discrete'], 'Position', [50, 180, 100, 210]);

%% Input: params (from workspace, set by run_wall_effect_test.m)
add_block('simulink/Sources/Constant', [modelName '/params_const']);
set_param([modelName '/params_const'], 'Value', 'params');  % Read from workspace
set_param([modelName '/params_const'], 'OutDataTypeStr', 'Bus: WallEffectParamsBus');  % Bus type for struct
set_param([modelName '/params_const'], 'Position', [250, 220, 300, 250]);

%% ZOH for f_M (discrete to continuous)
add_block('simulink/Discrete/Zero-Order Hold', [modelName '/ZOH_fM']);
set_param([modelName '/ZOH_fM'], 'Position', [150, 100, 200, 130]);

%% ZOH for f_T (discrete to continuous)
add_block('simulink/Discrete/Zero-Order Hold', [modelName '/ZOH_fT']);
set_param([modelName '/ZOH_fT'], 'Position', [150, 180, 200, 210]);

%% Sum block (f_M + f_T)
add_block('simulink/Math Operations/Add', [modelName '/Force_Sum']);
set_param([modelName '/Force_Sum'], 'Position', [250, 135, 280, 175]);
set_param([modelName '/Force_Sum'], 'Inputs', '++');

%% MATLAB Function block for Gamma_inv calculation
add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/Gamma_inv_Block']);
set_param([modelName '/Gamma_inv_Block'], 'Position', [330, 120, 450, 190]);

%% Integrator (1/s) - continuous integration
add_block('simulink/Continuous/Integrator', [modelName '/Integrator']);
set_param([modelName '/Integrator'], 'Position', [500, 140, 550, 170]);
set_param([modelName '/Integrator'], 'InitialCondition', 'params.p0');

%% Rate Transition (continuous p to discrete p for feedback)
add_block('simulink/Signal Attributes/Rate Transition', [modelName '/Sampler']);
set_param([modelName '/Sampler'], 'Position', [600, 140, 650, 170]);

%% Output: Scope for p
add_block('simulink/Sinks/Scope', [modelName '/Position_Scope']);
set_param([modelName '/Position_Scope'], 'Position', [700, 100, 750, 130]);

%% Output: Scope for h_bar
add_block('simulink/Sinks/Scope', [modelName '/h_bar_Scope']);
set_param([modelName '/h_bar_Scope'], 'Position', [550, 220, 600, 250]);

%% Output: To Workspace for p
add_block('simulink/Sinks/To Workspace', [modelName '/p_out']);
set_param([modelName '/p_out'], 'VariableName', 'p_sim');
set_param([modelName '/p_out'], 'SaveFormat', 'Timeseries');
set_param([modelName '/p_out'], 'Position', [700, 170, 780, 200]);

%% Output: p_discrete for Control Law (to workspace)
add_block('simulink/Sinks/To Workspace', [modelName '/p_discrete_out']);
set_param([modelName '/p_discrete_out'], 'VariableName', 'p_discrete');
set_param([modelName '/p_discrete_out'], 'SaveFormat', 'Timeseries');
set_param([modelName '/p_discrete_out'], 'Position', [700, 240, 780, 270]);

%% ==================== Connect Blocks ====================
% NOTE: MATLAB Function Block starts with only 1 input/output.
%       After configuring the function code, additional ports will appear.
%       We only connect the basic lines here; user must manually connect:
%         - Integrator -> Gamma_inv_Block (input 2: p)
%         - params_const -> Gamma_inv_Block (input 3: params)
%         - Gamma_inv_Block (output 3: h_bar) -> h_bar_Scope

% f_M path: f_M_discrete -> ZOH_fM -> Force_Sum
add_line(modelName, 'f_M_discrete/1', 'ZOH_fM/1');
add_line(modelName, 'ZOH_fM/1', 'Force_Sum/1');

% f_T path: f_T_discrete -> ZOH_fT -> Force_Sum
add_line(modelName, 'f_T_discrete/1', 'ZOH_fT/1');
add_line(modelName, 'ZOH_fT/1', 'Force_Sum/2');

% Force_Sum -> Gamma_inv_Block (input 1: f)
add_line(modelName, 'Force_Sum/1', 'Gamma_inv_Block/1');

% Gamma_inv_Block -> Integrator (output 1: p_dot)
add_line(modelName, 'Gamma_inv_Block/1', 'Integrator/1');

% Integrator -> outputs
add_line(modelName, 'Integrator/1', 'Sampler/1');
add_line(modelName, 'Integrator/1', 'Position_Scope/1');
add_line(modelName, 'Integrator/1', 'p_out/1');

% Sampler -> p_discrete_out
add_line(modelName, 'Sampler/1', 'p_discrete_out/1');

% NOTE: The following lines must be connected MANUALLY after setting up
%       the MATLAB Function Block code:
%   - Integrator/1 -> Gamma_inv_Block/2 (p feedback)
%   - params_const/1 -> Gamma_inv_Block/3 (params)
%   - Gamma_inv_Block/3 -> h_bar_Scope/1 (h_bar output)

%% Save model
save_system(modelName, modelPath);

%% Display message
fprintf('\n');
fprintf('=====================================================\n');
fprintf('  Simulink model created: model/system_model.slx\n');
fprintf('=====================================================\n');
fprintf('\n');
fprintf('Architecture:\n');
fprintf('  f_M (discrete) -> [ZOH] -+\n');
fprintf('                          |-> [+] -> [Gamma_inv*f] -> [1/s] -> p\n');
fprintf('  f_T (discrete) -> [ZOH] -+              ^                  |\n');
fprintf('                                          +------------------+\n');
fprintf('                                             p (feedback)\n');
fprintf('\n');
fprintf('Next steps:\n');
fprintf('1. Run test_script/run_wall_effect_test.m to load params\n');
fprintf('2. Open model/system_model.slx\n');
fprintf('3. Double-click "Gamma_inv_Block" and paste:\n');
fprintf('\n');
fprintf('   function [p_dot, Gamma_inv, h_bar] = fcn(f, p, params)\n');
fprintf('       [p_dot, Gamma_inv, h_bar] = wall_effect_integrated(f, p, params);\n');
fprintf('   end\n');
fprintf('\n');
fprintf('4. Press Ctrl+S to save the MATLAB Function\n');
fprintf('5. MANUALLY connect the following lines:\n');
fprintf('   - Integrator output -> Gamma_inv_Block input 2 (p)\n');
fprintf('   - params_const output -> Gamma_inv_Block input 3 (params)\n');
fprintf('   - Gamma_inv_Block output 3 (h_bar) -> h_bar_Scope\n');
fprintf('6. Save model (Ctrl+S) and run simulation\n');
fprintf('=====================================================\n');
