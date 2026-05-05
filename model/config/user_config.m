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
%       a_pd        = 0.05      % LP for δp_md mean (v2) / deterministic removal (v1)
%       a_prd       = 0.05      % HP residual mean (v1 only)
%       a_cov       = 0.05      % EWMA for σ²_δxr variance / covariance estimation
%       epsilon     = 0.01      % Anisotropy threshold for theta measurement
%       sigma2_w_fD = 0         % f_D random-walk innovation variance [pN^2/step]
%                                 (Phase 5 §5.4, eq17_7state v2 only)
%       sigma2_w_fA = 0         % a_x random-walk innovation variance [(um/pN)^2/step]
%                                 (Phase 5 §5.5, eq17_7state v2 only)
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
    config.meas_noise_enable = false;
    config.meas_noise_std = [0.01; 0.01; 0.01];

    % EKF estimation parameters (IIR single-layer HP + variance)
    %   v1 (legacy 7-state) HP-LP: a_pd  = LP coefficient (deterministic removal)
    %                              a_prd = HP residual mean coefficient
    %                              a_cov = HP residual mean-square coefficient
    %   v2 (eq17_7state, design_v2 §6): a_pd  = LP for δp_md mean estimation
    %                                   a_cov = EWMA for σ²_δxr variance estimation
    %                                   (a_prd unused in v2)
    %   Phase 0 §6 lock: v2 baseline a_pd = a_cov = 0.05.
    config.a_pd = 0.05;             % LP for δp_md mean (v2) / deterministic removal (v1)
    config.a_prd = 0.05;            % HP residual mean (v1 only; ignored by eq17_7state)
    config.a_cov = 0.05;            % EWMA for σ²_δxr variance estimation (v1+v2)
    config.epsilon = 0.01;          % Anisotropy threshold for theta measurement

    % Wave 2D: f_D random-walk innovation variance (Phase 5 §5.4)
    %   Used in Q55,i = a_nom_axis^2 * sigma2_w_fD (eq17_7state v2 only).
    %   Baseline 0 → Q55 = 0 (no f_D process noise injection).
    config.sigma2_w_fD = 0;         % [pN^2/step], Phase 5 §5.4 baseline 0

    % Phase 5 §5.5: a_x random-walk innovation variance (analogue of §5.4 σ²_w_fD)
    %   Used in Q77_phase5_floor,i = a_nom_axis^2 * sigma2_w_fA (eq17_7state v2 only).
    %   Provides Q77 floor so KF P77 doesn't degenerate when wall-coupling Q77 ≈ 0
    %   (e.g. far-from-wall positioning at h=50). Baseline 0 → no floor injection.
    config.sigma2_w_fA = 0;         % [(um/pN)^2/step], Phase 5 §5.5 baseline 0

    % iir_warmup_mode: how to initialize IIR LP / EWMA states.
    %   'legacy'  - dx_bar_m=0, sigma2_dxr_hat=0, warmup_count=2
    %               (controller emits f_d=0 for first 3 calls; IIR accumulates from 0)
    %   'prefill' - dx_bar_m=0, sigma2_dxr_hat = 4*kBT*a_x_init.*C_dpmr_eff
    %               + C_np_eff.*sigma2_n_s (per-axis steady-state at h_init);
    %               warmup_count=0 (controller emits real f_d from call 2 onward).
    %               Requires fixed h_init at start; ramp/motion lag <= 1 IIR tau.
    config.iir_warmup_mode = 'prefill';

    % 7-state EKF specific parameters
    config.beta = 0.5;                  % Disturbance/gain coupling parameter
    config.lamdaF = 1.0;                % EKF forgetting factor
    config.Pf_init_diag = [0; 0; 1e-4; 1e-4; 0; 10*(0.0147)^2; 0];  % 7x1
    config.Qz_diag_scaling = [0; 0; 1e4; 1e-1; 0; 1e-4; 0];           % 7x1 (tuned near-wall)
    config.Rz_diag_scaling = [1e-2; 1e0];                              % 2x1

    % Thermal
    config.thermal_enable = true;

    % Simulation
    config.T_sim = 5;

end
