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
%       theta       = 0         % Azimuth angle [rad]
%       phi         = 0         % Elevation angle [rad]
%       pz          = 0         % Wall displacement along w_hat [um]
%       h_min       = 3.375     % Minimum safe distance [um] (1.5 * R)
%
%       % Trajectory
%       traj_type   = 'z_sine'  % 'z_sine' or 'xy_circle'
%       h_init      = 5         % Initial distance from wall [um]
%       amplitude   = 5         % z_sine: oscillation amplitude [um]
%       frequency   = 1         % z_sine: oscillation frequency [Hz]
%       n_cycles    = 3         % Number of cycles
%       radius      = 5         % xy_circle: radius [um]
%       period      = 1         % xy_circle: period [sec]
%
%       % Controller
%       ctrl_enable = true      % Enable controller (false = open-loop)
%       lambda_c    = 0.7       % Closed-loop pole (0 < lambda_c < 1)
%       noise_filter_enable = false  % Enable low-pass filter on feedback
%       noise_filter_cutoff = 5      % Cutoff frequency [Hz]
%       meas_noise_enable = false    % Enable measurement noise injection
%       meas_noise_std = [0.01; 0.01; 0.01]  % Noise std [um] per axis
%
%       % Thermal
%       thermal_enable = true   % Enable thermal force
%
%       % Simulation
%       T_sim       = 5         % Simulation time [sec]

    % Wall
    config.theta = 0;
    config.phi = 0;
    config.pz = 0;
    config.h_min = 1.5 * 2.25;     % 1.5 * R [um]

    % Trajectory
    config.traj_type = 'z_sine';
    config.h_init = 5;
    config.amplitude = 5;
    config.frequency = 1;
    config.n_cycles = 3;
    config.radius = 5;
    config.period = 1;

    % Controller
    config.ctrl_enable = true;
    config.lambda_c = 0.7;
    config.noise_filter_enable = false;
    config.noise_filter_cutoff = 5;
    config.meas_noise_enable = false;
    config.meas_noise_std = [0.01; 0.01; 0.01];

    % Thermal
    config.thermal_enable = true;

    % Simulation
    config.T_sim = 5;

end
