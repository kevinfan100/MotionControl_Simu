function params = calc_simulation_params(config)
%CALC_SIMULATION_PARAMS Calculate simulation parameters from config
%
%   params = calc_simulation_params(config)
%
%   Inputs:
%       config - Configuration structure with user-adjustable parameters
%
%   Outputs:
%       params - Complete parameter structure for simulation
%
%   Config fields (all optional, defaults shown):
%       % Wall parameters
%       theta       = 0         % Azimuth angle [rad]
%       phi         = 0         % Elevation angle [rad]
%       pz          = 0         % Wall displacement [um]
%       h_bar_min   = 1.5       % Minimum safe normalized distance
%
%       % Trajectory parameters
%       traj_type   = 'z_move'  % 'z_move' or 'xy_circle'
%       h_margin    = 5         % Additional safety margin [um]
%       delta_z     = 10        % z_move: displacement [um]
%       direction   = 'away'    % z_move: 'away' or 'toward'
%       speed       = 5         % z_move: velocity [um/sec]
%       radius      = 5         % xy_circle: radius [um]
%       period      = 1         % xy_circle: period [sec]
%       n_circles   = 3         % xy_circle: number of circles
%
%       % Controller parameters
%       ctrl_enable = true      % Enable controller (false = open-loop)
%       lambda_c    = 0.7       % Closed-loop pole (0 < lambda_c < 1)
%
%       % Thermal force parameters
%       thermal_enable = true   % Enable thermal force
%
%       % Simulation parameters
%       T_sim       = 5         % Simulation time [sec]

    %% Fixed physical constants
    R = 2.25;                    % Particle radius [um]
    gamma_N = 0.0425;            % Stokes drag coefficient [pN*sec/um]
    Ts = 1/1606;                 % Sampling period [sec]
    k_B = 1.3806503e-5;          % Boltzmann constant [pN*um/K]
    T = 310.15;                  % Temperature [K] (37 C)

    %% Default values
    defaults = struct(...
        'theta', 0, ...
        'phi', 0, ...
        'pz', 0, ...
        'h_bar_min', 1.5, ...
        'traj_type', 'z_move', ...
        'h_margin', 5, ...
        'delta_z', 10, ...
        'direction', 'away', ...
        'speed', 5, ...
        'radius', 5, ...
        'period', 1, ...
        'n_circles', 3, ...
        'ctrl_enable', true, ...
        'lambda_c', 0.7, ...
        'thermal_enable', true, ...
        'T_sim', 5 ...
    );

    %% Merge config with defaults
    if nargin < 1 || isempty(config)
        config = struct();
    end

    fields = fieldnames(defaults);
    for i = 1:length(fields)
        if ~isfield(config, fields{i})
            config.(fields{i}) = defaults.(fields{i});
        end
    end

    %% Common parameters (shared by all modules)
    params.common.R = R;
    params.common.gamma_N = gamma_N;
    params.common.Ts = Ts;
    params.common.T_sim = config.T_sim;

    %% Wall parameters
    theta = config.theta;
    phi = config.phi;

    params.wall.theta = theta;
    params.wall.phi = phi;
    params.wall.pz = config.pz;
    params.wall.h_bar_min = config.h_bar_min;

    % Calculate orthonormal basis vectors
    % w_hat: normal to wall (pointing away from wall)
    % u_hat, v_hat: parallel to wall
    params.wall.w_hat = [cos(theta)*sin(phi); sin(theta)*sin(phi); cos(phi)];
    params.wall.u_hat = [-cos(theta)*cos(phi); -sin(theta)*cos(phi); sin(phi)];
    params.wall.v_hat = [sin(theta); -cos(theta); 0];

    %% Trajectory parameters
    params.traj.type = config.traj_type;
    params.traj.h_margin = config.h_margin;
    params.traj.delta_z = config.delta_z;
    params.traj.direction = config.direction;
    params.traj.speed = config.speed;
    params.traj.radius = config.radius;
    params.traj.period = config.period;
    params.traj.n_circles = config.n_circles;

    %% Controller parameters
    params.ctrl.enable = config.ctrl_enable;
    params.ctrl.lambda_c = config.lambda_c;
    params.ctrl.gamma = gamma_N;  % Initial version: fixed value
    params.ctrl.Ts = Ts;

    %% Thermal force parameters
    params.thermal.enable = config.thermal_enable;
    params.thermal.k_B = k_B;
    params.thermal.T = T;
    params.thermal.Ts = Ts;
    params.thermal.variance_coeff = 4 * k_B * T * gamma_N / Ts;

end
