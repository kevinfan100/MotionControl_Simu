function config = user_config()
%USER_CONFIG Return default user-adjustable parameters
%
%   config = user_config()
%
%   Returns a struct with all user-adjustable simulation parameters
%   and their default values. Override specific fields before passing
%   to calc_simulation_params().
%
%   Fields:
%       % Wall
%       theta       = 0         % Azimuth angle [deg]
%       phi         = 0         % Elevation angle [deg]
%       pz          = 0         % Wall displacement along w_hat [um]
%       h_min       = 3.375     % Minimum safe distance [um] (1.5 * R)
%
%       % Trajectory (sinusoidal along wall normal w_hat)
%       h_init      = 5         % Initial distance from wall [um]
%       amplitude   = 5         % Oscillation amplitude in h direction [um]
%       frequency   = 1         % Oscillation frequency [Hz]
%       n_cycles    = 3         % Number of cycles
%
%       % Controller
%       ctrl_enable = true      % Enable controller (false = open-loop)
%       lambda_c    = 0.7       % Closed-loop pole (0 < lambda_c < 1)
%       meas_noise_enable = false    % Enable measurement noise injection
%       meas_noise_std = [0.01; 0.01; 0.01]  % Noise std [um] per axis
%
%       % Thermal
%       thermal_enable = true   % Enable thermal force
%
%       % Simulation
%       T_sim       = 5         % Simulation time [sec]

    % Wall (angles in degrees)
    config.theta = 0;              % Azimuth angle [deg]
    config.phi = 0;                % Elevation angle [deg]
    config.pz = 0;
    config.h_min = 1.5 * 2.25;     % 1.5 * R [um]

    % Trajectory (sinusoidal along wall normal w_hat)
    config.h_init = 5;
    config.amplitude = 5;
    config.frequency = 1;
    config.n_cycles = 3;

    % Controller
    config.ctrl_enable = true;
    config.lambda_c = 0.7;
    config.meas_noise_enable = false;
    config.meas_noise_std = [0.01; 0.01; 0.01];

    % Thermal
    config.thermal_enable = true;

    % Simulation
    config.T_sim = 5;

end
