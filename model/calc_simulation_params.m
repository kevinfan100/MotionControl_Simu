function params = calc_simulation_params(config)
%CALC_SIMULATION_PARAMS Calculate simulation parameters and create Simulink Bus Objects
%
%   params = calc_simulation_params(config)
%
%   This function:
%   1. Loads physical constants from physical_constants()
%   2. Merges user config with defaults from user_config()
%   3. Delegates to sub-calculators for each parameter group
%   4. Creates Simulink Bus Object definitions
%   5. Returns Simulink.Parameter object
%
%   Inputs:
%       config - (optional) Configuration structure with user-adjustable
%                parameters. See user_config() for available fields and defaults.
%                Only fields that differ from defaults need to be specified.
%
%   Outputs:
%       params - Simulink.Parameter with DataType 'Bus: ParamsBus'


    %% SECTION 1: Load constants and merge config with defaults

    constants = physical_constants();

    defaults = user_config();

    if nargin < 1 || isempty(config)
        config = struct();
    end

    fields = fieldnames(defaults);
    for i = 1:length(fields)
        if ~isfield(config, fields{i})
            config.(fields{i}) = defaults.(fields{i});
        end
    end


    %% SECTION 2: Calculate derived parameters via sub-functions


    % --- common sub-structure ---
    params_data.common.R = constants.R;
    params_data.common.gamma_N = constants.gamma_N;
    params_data.common.Ts = constants.Ts;
    params_data.common.T_sim = config.T_sim;

    % --- wall sub-structure ---
    params_data.wall = calc_wall_params(config, constants);

    % --- traj sub-structure ---
    params_data.traj = calc_traj_params(config);

    % --- initial position (depends on wall + traj) ---
    params_data.common.p0 = calc_initial_position(params_data);

    % --- ctrl sub-structure ---
    params_data.ctrl = calc_ctrl_params(config, constants);

    % --- thermal sub-structure ---
    params_data.thermal = calc_thermal_params(config, constants);


    %% SECTION 3: Create Nested Bus Objects


    % --- CommonBus ---
    elems_common = Simulink.BusElement.empty(0, 5);
    elems_common(1) = Simulink.BusElement; elems_common(1).Name = 'R';
    elems_common(1).Dimensions = [1 1]; elems_common(1).DataType = 'double';
    elems_common(2) = Simulink.BusElement; elems_common(2).Name = 'gamma_N';
    elems_common(2).Dimensions = [1 1]; elems_common(2).DataType = 'double';
    elems_common(3) = Simulink.BusElement; elems_common(3).Name = 'Ts';
    elems_common(3).Dimensions = [1 1]; elems_common(3).DataType = 'double';
    elems_common(4) = Simulink.BusElement; elems_common(4).Name = 'T_sim';
    elems_common(4).Dimensions = [1 1]; elems_common(4).DataType = 'double';
    elems_common(5) = Simulink.BusElement; elems_common(5).Name = 'p0';
    elems_common(5).Dimensions = [3 1]; elems_common(5).DataType = 'double';

    CommonBus = Simulink.Bus;
    CommonBus.Elements = elems_common;
    assignin('base', 'CommonBus', CommonBus);

    % --- WallBus ---
    elems_wall = Simulink.BusElement.empty(0, 8);
    elems_wall(1) = Simulink.BusElement; elems_wall(1).Name = 'theta';
    elems_wall(1).Dimensions = [1 1]; elems_wall(1).DataType = 'double';
    elems_wall(2) = Simulink.BusElement; elems_wall(2).Name = 'phi';
    elems_wall(2).Dimensions = [1 1]; elems_wall(2).DataType = 'double';
    elems_wall(3) = Simulink.BusElement; elems_wall(3).Name = 'pz';
    elems_wall(3).Dimensions = [1 1]; elems_wall(3).DataType = 'double';
    elems_wall(4) = Simulink.BusElement; elems_wall(4).Name = 'h_min';
    elems_wall(4).Dimensions = [1 1]; elems_wall(4).DataType = 'double';
    elems_wall(5) = Simulink.BusElement; elems_wall(5).Name = 'h_bar_min';
    elems_wall(5).Dimensions = [1 1]; elems_wall(5).DataType = 'double';
    elems_wall(6) = Simulink.BusElement; elems_wall(6).Name = 'w_hat';
    elems_wall(6).Dimensions = [3 1]; elems_wall(6).DataType = 'double';
    elems_wall(7) = Simulink.BusElement; elems_wall(7).Name = 'u_hat';
    elems_wall(7).Dimensions = [3 1]; elems_wall(7).DataType = 'double';
    elems_wall(8) = Simulink.BusElement; elems_wall(8).Name = 'v_hat';
    elems_wall(8).Dimensions = [3 1]; elems_wall(8).DataType = 'double';

    WallBus = Simulink.Bus;
    WallBus.Elements = elems_wall;
    assignin('base', 'WallBus', WallBus);

    % --- TrajBus ---
    elems_traj = Simulink.BusElement.empty(0, 4);
    elems_traj(1) = Simulink.BusElement; elems_traj(1).Name = 'h_init';
    elems_traj(1).Dimensions = [1 1]; elems_traj(1).DataType = 'double';
    elems_traj(2) = Simulink.BusElement; elems_traj(2).Name = 'amplitude';
    elems_traj(2).Dimensions = [1 1]; elems_traj(2).DataType = 'double';
    elems_traj(3) = Simulink.BusElement; elems_traj(3).Name = 'frequency';
    elems_traj(3).Dimensions = [1 1]; elems_traj(3).DataType = 'double';
    elems_traj(4) = Simulink.BusElement; elems_traj(4).Name = 'n_cycles';
    elems_traj(4).Dimensions = [1 1]; elems_traj(4).DataType = 'double';

    TrajBus = Simulink.Bus;
    TrajBus.Elements = elems_traj;
    assignin('base', 'TrajBus', TrajBus);

    % --- CtrlBus ---
    elems_ctrl = Simulink.BusElement.empty(0, 7);
    elems_ctrl(1) = Simulink.BusElement; elems_ctrl(1).Name = 'enable';
    elems_ctrl(1).Dimensions = [1 1]; elems_ctrl(1).DataType = 'double';
    elems_ctrl(2) = Simulink.BusElement; elems_ctrl(2).Name = 'lambda_c';
    elems_ctrl(2).Dimensions = [1 1]; elems_ctrl(2).DataType = 'double';
    elems_ctrl(3) = Simulink.BusElement; elems_ctrl(3).Name = 'gamma';
    elems_ctrl(3).Dimensions = [1 1]; elems_ctrl(3).DataType = 'double';
    elems_ctrl(4) = Simulink.BusElement; elems_ctrl(4).Name = 'Ts';
    elems_ctrl(4).Dimensions = [1 1]; elems_ctrl(4).DataType = 'double';
    elems_ctrl(5) = Simulink.BusElement; elems_ctrl(5).Name = 'meas_noise_enable';
    elems_ctrl(5).Dimensions = [1 1]; elems_ctrl(5).DataType = 'double';
    elems_ctrl(6) = Simulink.BusElement; elems_ctrl(6).Name = 'meas_noise_std';
    elems_ctrl(6).Dimensions = [3 1]; elems_ctrl(6).DataType = 'double';
    elems_ctrl(7) = Simulink.BusElement; elems_ctrl(7).Name = 'meas_noise_seed';
    elems_ctrl(7).Dimensions = [1 1]; elems_ctrl(7).DataType = 'double';

    CtrlBus = Simulink.Bus;
    CtrlBus.Elements = elems_ctrl;
    assignin('base', 'CtrlBus', CtrlBus);

    % --- ThermalBus ---
    elems_thermal = Simulink.BusElement.empty(0, 6);
    elems_thermal(1) = Simulink.BusElement; elems_thermal(1).Name = 'enable';
    elems_thermal(1).Dimensions = [1 1]; elems_thermal(1).DataType = 'double';
    elems_thermal(2) = Simulink.BusElement; elems_thermal(2).Name = 'k_B';
    elems_thermal(2).Dimensions = [1 1]; elems_thermal(2).DataType = 'double';
    elems_thermal(3) = Simulink.BusElement; elems_thermal(3).Name = 'T';
    elems_thermal(3).Dimensions = [1 1]; elems_thermal(3).DataType = 'double';
    elems_thermal(4) = Simulink.BusElement; elems_thermal(4).Name = 'Ts';
    elems_thermal(4).Dimensions = [1 1]; elems_thermal(4).DataType = 'double';
    elems_thermal(5) = Simulink.BusElement; elems_thermal(5).Name = 'variance_coeff';
    elems_thermal(5).Dimensions = [1 1]; elems_thermal(5).DataType = 'double';
    elems_thermal(6) = Simulink.BusElement; elems_thermal(6).Name = 'seed';
    elems_thermal(6).Dimensions = [1 1]; elems_thermal(6).DataType = 'double';

    ThermalBus = Simulink.Bus;
    ThermalBus.Elements = elems_thermal;
    assignin('base', 'ThermalBus', ThermalBus);

    % --- ParamsBus (parent) ---
    elems_params = Simulink.BusElement.empty(0, 5);
    elems_params(1) = Simulink.BusElement; elems_params(1).Name = 'common';
    elems_params(1).Dimensions = [1 1]; elems_params(1).DataType = 'Bus: CommonBus';
    elems_params(2) = Simulink.BusElement; elems_params(2).Name = 'wall';
    elems_params(2).Dimensions = [1 1]; elems_params(2).DataType = 'Bus: WallBus';
    elems_params(3) = Simulink.BusElement; elems_params(3).Name = 'traj';
    elems_params(3).Dimensions = [1 1]; elems_params(3).DataType = 'Bus: TrajBus';
    elems_params(4) = Simulink.BusElement; elems_params(4).Name = 'ctrl';
    elems_params(4).Dimensions = [1 1]; elems_params(4).DataType = 'Bus: CtrlBus';
    elems_params(5) = Simulink.BusElement; elems_params(5).Name = 'thermal';
    elems_params(5).Dimensions = [1 1]; elems_params(5).DataType = 'Bus: ThermalBus';

    ParamsBus = Simulink.Bus;
    ParamsBus.Description = 'Motion Control Simulation Parameters';
    ParamsBus.Elements = elems_params;
    assignin('base', 'ParamsBus', ParamsBus);


    %% SECTION 4: Package as Simulink.Parameter

    params = Simulink.Parameter(params_data);
    params.DataType = 'Bus: ParamsBus';
    params.Description = 'Motion Control Parameters for Simulink';

end
