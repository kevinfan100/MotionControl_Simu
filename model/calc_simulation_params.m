function params = calc_simulation_params(config)
%CALC_SIMULATION_PARAMS Calculate simulation parameters and create Simulink Bus Objects
%
%   params = calc_simulation_params(config)
%
%   This function:
%   1. Calculates all simulation parameters from config
%   2. Converts string parameters to numeric (Simulink compatible)
%   3. Creates Simulink Bus Object definitions
%   4. Returns Simulink.Parameter object
%
%   Inputs:
%       config - Configuration structure with user-adjustable parameters
%
%   Outputs:
%       params - Simulink.Parameter with DataType 'Bus: ParamsBus'
%
%   Config fields (all optional, defaults shown):
%       % Wall parameters
%       theta       = 0         % Azimuth angle [rad]
%       phi         = 0         % Elevation angle [rad]
%       pz          = 0         % Wall displacement along w_hat [um]
%       h_min       = 3.375     % Minimum safe distance [um] (default: 1.5 * R)
%
%       % Trajectory parameters
%       traj_type   = 'z_sine'  % 'z_sine' or 'xy_circle'
%       h_init      = 5         % Initial distance from wall [um]
%       amplitude   = 5         % z_sine: oscillation amplitude [um]
%       frequency   = 1         % z_sine: oscillation frequency [Hz]
%       n_cycles    = 3         % z_sine/xy_circle: number of cycles
%       radius      = 5         % xy_circle: radius [um]
%       period      = 1         % xy_circle: period [sec]
%
%       % Controller parameters
%       ctrl_enable = true      % Enable controller (false = open-loop)
%       lambda_c    = 0.7       % Closed-loop pole (0 < lambda_c < 1)
%       noise_filter_enable = false  % Enable low-pass filter on feedback
%       noise_filter_cutoff = 5      % Cutoff frequency [Hz]
%
%       % Thermal force parameters
%       thermal_enable = true   % Enable thermal force
%
%       % Simulation parameters
%       T_sim       = 5         % Simulation time [sec]

    
    %% SECTION 1: Fixed physical constants
    
    R = 2.25;                    % Particle radius [um]
    gamma_N = 0.0425;            % Stokes drag coefficient [pN*sec/um]
    Ts = 1/1606;                 % Sampling period [sec]
    k_B = 1.3806503e-5;          % Boltzmann constant [pN*um/K]
    T_temp = 310.15;             % Temperature [K] (37 C)


    %% SECTION 2: Default values and merge

    defaults = struct(...
        'theta', 0, ...
        'phi', 0, ...
        'pz', 0, ...
        'h_min', 1.5 * R, ...        % Minimum safe distance [um] (h_bar_min=1.5 * R=3.375)
        'traj_type', 'z_sine', ...
        'h_init', 5, ...             % Initial distance from wall [um]
        'amplitude', 5, ...          % z_sine: oscillation amplitude [um]
        'frequency', 1, ...          % z_sine: oscillation frequency [Hz]
        'n_cycles', 3, ...           % z_sine/xy_circle: number of cycles
        'radius', 5, ...             % xy_circle: radius [um]
        'period', 1, ...             % xy_circle: period [sec]
        'ctrl_enable', true, ...
        'lambda_c', 0.7, ...
        'noise_filter_enable', false, ...  % Noise filter switch
        'noise_filter_cutoff', 5, ...      % Cutoff frequency [Hz]
        'meas_noise_enable', false, ...    % Measurement noise switch
        'meas_noise_std', [0.01; 0.01; 0.01], ...  % Measurement noise std [um] per axis
        'thermal_enable', true, ...
        'T_sim', 5 ...
    );

    if nargin < 1 || isempty(config)
        config = struct();
    end

    fields = fieldnames(defaults);
    for i = 1:length(fields)
        if ~isfield(config, fields{i})
            config.(fields{i}) = defaults.(fields{i});
        end
    end

    
    %% SECTION 3: Calculate derived parameters
    

    % --- common sub-structure ---
    params_data.common.R = R;
    params_data.common.gamma_N = gamma_N;
    params_data.common.Ts = Ts;
    params_data.common.T_sim = config.T_sim;

    % --- wall sub-structure ---
    theta = config.theta;
    phi = config.phi;

    params_data.wall.theta = theta;
    params_data.wall.phi = phi;
    params_data.wall.pz = config.pz;
    params_data.wall.h_min = config.h_min;                 % [um] minimum safe distance
    params_data.wall.h_bar_min = config.h_min / R;         % [unitless] for Wall Effect calc
    params_data.wall.w_hat = [cos(theta)*sin(phi); sin(theta)*sin(phi); cos(phi)];
    params_data.wall.u_hat = [-cos(theta)*cos(phi); -sin(theta)*cos(phi); sin(phi)];
    params_data.wall.v_hat = [sin(theta); -cos(theta); 0];

    % --- traj sub-structure (numeric encoding for Simulink) ---
    % type: 'z_sine' -> 0, 'xy_circle' -> 1
    switch config.traj_type
        case 'z_sine'
            params_data.traj.type = 0;
        case 'xy_circle'
            params_data.traj.type = 1;
        otherwise
            error('Unknown trajectory type: %s', config.traj_type);
    end

    params_data.traj.h_init = config.h_init;           % [um] initial distance from wall
    params_data.traj.amplitude = config.amplitude;     % [um] z_sine oscillation amplitude
    params_data.traj.frequency = config.frequency;     % [Hz] z_sine oscillation frequency
    params_data.traj.n_cycles = config.n_cycles;       % number of cycles (z_sine/xy_circle)
    params_data.traj.radius = config.radius;           % [um] xy_circle radius
    params_data.traj.period = config.period;           % [sec] xy_circle period

    % --- ctrl sub-structure ---
    params_data.ctrl.enable = double(config.ctrl_enable);  % Convert to double
    params_data.ctrl.lambda_c = config.lambda_c;
    params_data.ctrl.gamma = gamma_N;
    params_data.ctrl.Ts = Ts;
    params_data.ctrl.noise_filter_enable = double(config.noise_filter_enable);
    params_data.ctrl.noise_filter_cutoff = config.noise_filter_cutoff;
    % Pre-calculate filter coefficient: alpha = Ts / (Ts + 1/(2*pi*fc))
    fc = config.noise_filter_cutoff;
    params_data.ctrl.filter_alpha = Ts / (Ts + 1/(2*pi*fc));
    % Measurement noise parameters
    params_data.ctrl.meas_noise_enable = double(config.meas_noise_enable);
    params_data.ctrl.meas_noise_std = config.meas_noise_std;  % [3x1]
    params_data.ctrl.meas_noise_seed = randi(2^31-1);         % Independent seed

    % --- thermal sub-structure ---
    params_data.thermal.enable = double(config.thermal_enable);  % Convert to double
    params_data.thermal.k_B = k_B;
    params_data.thermal.T = T_temp;
    params_data.thermal.Ts = Ts;
    params_data.thermal.variance_coeff = 4 * k_B * T_temp * gamma_N / Ts;
    params_data.thermal.seed = randi(2^31-1);  % Random seed for thermal force

    
    %% SECTION 4: Create Nested Bus Objects
    

    % --- CommonBus ---
    elems_common = Simulink.BusElement.empty(0, 4);
    elems_common(1) = Simulink.BusElement; elems_common(1).Name = 'R';
    elems_common(1).Dimensions = [1 1]; elems_common(1).DataType = 'double';
    elems_common(2) = Simulink.BusElement; elems_common(2).Name = 'gamma_N';
    elems_common(2).Dimensions = [1 1]; elems_common(2).DataType = 'double';
    elems_common(3) = Simulink.BusElement; elems_common(3).Name = 'Ts';
    elems_common(3).Dimensions = [1 1]; elems_common(3).DataType = 'double';
    elems_common(4) = Simulink.BusElement; elems_common(4).Name = 'T_sim';
    elems_common(4).Dimensions = [1 1]; elems_common(4).DataType = 'double';

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
    elems_traj = Simulink.BusElement.empty(0, 7);
    elems_traj(1) = Simulink.BusElement; elems_traj(1).Name = 'type';
    elems_traj(1).Dimensions = [1 1]; elems_traj(1).DataType = 'double';
    elems_traj(2) = Simulink.BusElement; elems_traj(2).Name = 'h_init';
    elems_traj(2).Dimensions = [1 1]; elems_traj(2).DataType = 'double';
    elems_traj(3) = Simulink.BusElement; elems_traj(3).Name = 'amplitude';
    elems_traj(3).Dimensions = [1 1]; elems_traj(3).DataType = 'double';
    elems_traj(4) = Simulink.BusElement; elems_traj(4).Name = 'frequency';
    elems_traj(4).Dimensions = [1 1]; elems_traj(4).DataType = 'double';
    elems_traj(5) = Simulink.BusElement; elems_traj(5).Name = 'n_cycles';
    elems_traj(5).Dimensions = [1 1]; elems_traj(5).DataType = 'double';
    elems_traj(6) = Simulink.BusElement; elems_traj(6).Name = 'radius';
    elems_traj(6).Dimensions = [1 1]; elems_traj(6).DataType = 'double';
    elems_traj(7) = Simulink.BusElement; elems_traj(7).Name = 'period';
    elems_traj(7).Dimensions = [1 1]; elems_traj(7).DataType = 'double';

    TrajBus = Simulink.Bus;
    TrajBus.Elements = elems_traj;
    assignin('base', 'TrajBus', TrajBus);

    % --- CtrlBus ---
    elems_ctrl = Simulink.BusElement.empty(0, 10);
    elems_ctrl(1) = Simulink.BusElement; elems_ctrl(1).Name = 'enable';
    elems_ctrl(1).Dimensions = [1 1]; elems_ctrl(1).DataType = 'double';
    elems_ctrl(2) = Simulink.BusElement; elems_ctrl(2).Name = 'lambda_c';
    elems_ctrl(2).Dimensions = [1 1]; elems_ctrl(2).DataType = 'double';
    elems_ctrl(3) = Simulink.BusElement; elems_ctrl(3).Name = 'gamma';
    elems_ctrl(3).Dimensions = [1 1]; elems_ctrl(3).DataType = 'double';
    elems_ctrl(4) = Simulink.BusElement; elems_ctrl(4).Name = 'Ts';
    elems_ctrl(4).Dimensions = [1 1]; elems_ctrl(4).DataType = 'double';
    elems_ctrl(5) = Simulink.BusElement; elems_ctrl(5).Name = 'noise_filter_enable';
    elems_ctrl(5).Dimensions = [1 1]; elems_ctrl(5).DataType = 'double';
    elems_ctrl(6) = Simulink.BusElement; elems_ctrl(6).Name = 'noise_filter_cutoff';
    elems_ctrl(6).Dimensions = [1 1]; elems_ctrl(6).DataType = 'double';
    elems_ctrl(7) = Simulink.BusElement; elems_ctrl(7).Name = 'filter_alpha';
    elems_ctrl(7).Dimensions = [1 1]; elems_ctrl(7).DataType = 'double';
    elems_ctrl(8) = Simulink.BusElement; elems_ctrl(8).Name = 'meas_noise_enable';
    elems_ctrl(8).Dimensions = [1 1]; elems_ctrl(8).DataType = 'double';
    elems_ctrl(9) = Simulink.BusElement; elems_ctrl(9).Name = 'meas_noise_std';
    elems_ctrl(9).Dimensions = [3 1]; elems_ctrl(9).DataType = 'double';
    elems_ctrl(10) = Simulink.BusElement; elems_ctrl(10).Name = 'meas_noise_seed';
    elems_ctrl(10).Dimensions = [1 1]; elems_ctrl(10).DataType = 'double';

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

    
    %% SECTION 5: Package as Simulink.Parameter
    
    params = Simulink.Parameter(params_data);
    params.DataType = 'Bus: ParamsBus';
    params.Description = 'Motion Control Parameters for Simulink';

end
