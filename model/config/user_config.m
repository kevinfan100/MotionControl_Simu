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
%       enable_wall_effect = true  % Enable wall effect (false = isotropic Stokes)
%
%       % Trajectory (hold + descent + cosine oscillation along w_hat)
%       t_hold      = 0.5       % Initial hold time at h_init [sec]
%       h_init      = 20        % Initial distance from wall [um]
%       h_bottom    = 2.5       % Lowest point / oscillation trough [um]
%       amplitude   = 2.5       % Oscillation half-amplitude in h direction [um]
%       frequency   = 1         % Oscillation frequency [Hz]
%       n_cycles    = 3         % Number of cycles
%       trajectory_type = 'osc' % Trajectory type ('osc' or 'positioning')
%
%       % Controller
%       ctrl_enable = true      % Enable controller (false = open-loop)
%       lambda_c    = 0.7       % Closed-loop pole (0 < lambda_c < 1)
%       meas_noise_enable = false    % Enable measurement noise injection
%       meas_noise_std = [0.01; 0.01; 0.01]  % Noise std [um] per axis
%       a_pd        = 0.1       % EMA smoothing for deterministic component
%       a_prd       = 0.1       % EMA smoothing for random-deterministic component
%       a_cov       = 0.1       % EMA smoothing for covariance estimation
%       epsilon     = 0.01      % Anisotropy threshold for theta measurement
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
    config.enable_wall_effect = true;  % Enable wall effect (false = isotropic Stokes)

    % Trajectory (hold + descent + cosine oscillation along wall normal w_hat)
    config.t_hold = 0.5;           % Initial hold time at h_init [sec]
    config.h_init = 20;            % Starting height [um]
    config.h_bottom = 2.5;         % Lowest point / oscillation trough [um]
    config.amplitude = 2.5;        % Oscillation half-amplitude [um]
    config.frequency = 1;
    config.n_cycles = 3;
    config.trajectory_type = 'osc';    % 'osc' or 'positioning'

    % Controller
    config.ctrl_enable = true;
    config.lambda_c = 0.7;
    config.controller_type = 23;        % 23 or 7
    config.lambda_e = 0;               % Observer pole (0 = deadbeat, used by controller_type=2)
    config.kf_R = 0;                   % KF measurement noise variance (0 = use default, used by controller_type=4)
    config.meas_noise_enable = false;
    config.meas_noise_std = [0.01; 0.01; 0.01];

    % EKF estimation parameters (IIR single-layer HP + variance)
    config.a_pd = 0.05;             % EMA smoothing for LP (deterministic removal)
    config.a_prd = 0.05;            % EMA smoothing for HP residual mean
    config.a_cov = 0.05;            % EMA smoothing for HP residual mean-square
                                    %   (Phase 3 tested 0.005-0.1: free-space static
                                    %   benefits from smaller values, but near-wall
                                    %   dynamic loses precision due to lag. 0.05 is
                                    %   the best balance for mixed scenarios.)
    config.epsilon = 0.01;          % Anisotropy threshold for theta measurement

    % 7-state EKF specific parameters
    config.beta = 0;                    % z-axis chart-extension coupling (0 = x/y Jordan-block form canonical; 0.5 = experimental predictor, off by default)
    config.lamdaF = 1.0;                % EKF forgetting factor (1.0 = no forgetting, standard KF)
    config.Pf_init_diag = [0; 0; 1e-4; 1e-4; 0; 10*(0.0147)^2; 0];  % 7x1
    % === Derived Q/R (reference/for_test/qr_theoretical_values.md, 2026-04-17) ===
    % closed-loop, no-tradeoff framework under backward-diff beta interpretation.
    % Q(1,1), Q(2,2) = 0: structural (paper Eq.14 delay chain)
    % Q(3,3) = 1*sigma2_dXT: free-space thermal (paper Eq.21, constant approx of
    %                        adaptive (a/a_nom)^2 since Stateflow blocks adaptive)
    % Q(4,4), Q(5,5) = 0: tuning bucket (x_D has no physical source in our sim)
    % Q(6,6) = Q(7,7) = Var(d2a)/sigma2_dXT ~= 1.34e-11 (backward-diff beta,
    %                   diagonal approx; from compute_q77_from_trajectory.m)
    % R(1,1) = sigma2_n/sigma2_dXT = 0.397 (noise ON, sensor spec)
    % R(2,2) = 1.72 (closed-loop self-consistent; compute_r22_self_consistent.m)
    config.Qz_diag_scaling = [0; 0; 1; 0; 0; 1.3444e-11; 1.3444e-11];
    config.Rz_diag_scaling = [0.3970; 1.7185];
    % =============================================================================
    % [historical, pre-derivation 2026-04-15 — empirical near-wall tuning]
    % config.Qz_diag_scaling = [0; 0; 1e4; 1e-1; 0; 1e-4; 0];
    % config.Rz_diag_scaling = [1e-2; 1e0];

    % Thermal
    config.thermal_enable = true;

    % Simulation
    config.T_sim = 5;

end
