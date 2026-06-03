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
    config.epsilon = 0.01;          % Anisotropy threshold for theta measurement

    % 7-state EKF specific parameters
    config.beta = 0;                    % z-axis chart-extension coupling (0 = x/y Jordan-block form canonical; 0.5 = experimental predictor, off by default)
    config.lamdaF = 1.0;                % EKF forgetting factor (1.0 = no forgetting, standard KF)

    % === Q/R Preset Selector (Session 7, 2026-04-19) ===
    % Choose one of:
    %   'frozen_correct' - paper-level positioning (Sessions 4-7).
    %                      a_hat_z bias < 1%, std 1-5%. Requires P6 controller
    %                      (P5 IIR pre-fill + warmup_count=2). 3D RMSE 35/62 nm.
    %                      Validated for static positioning at h=2.5 and h=50.
    %   'empirical'      - historical near-wall tuning (pre-derivation, 2026-04-15).
    %                      a_hat_z bias ~0%, std ~20%. 3D RMSE same as frozen.
    %                      Use for dynamic / motion scenarios where frozen Q can't
    %                      track changing a.
    %   'beta'           - derived backward-diff beta interpretation (2026-04-17).
    %                      Q(6,6) = Q(7,7) = Var(d2a). Tested in Sessions 4-5,
    %                      drifts (a_hat std 22-39%) — not recommended for use.
    config.qr_preset = 'frozen_correct';
    config = apply_qr_preset(config);   % sets Qz/Rz/Pf_init_diag/a_cov

    % ----------------------------------------------------------------
    % eq17 EKF Q-matrix defaults (controller_type=17 only; eq6/23-state
    % use the qr_preset Qz/Rz above and ignore these flags)
    %
    % Q66 mode selection (mutually exclusive in priority order, see
    % motion_control_law_eq17_core.m Q66 selection block):
    %   Q66_OL_mode (true, default)  -> open-loop thermal:
    %       Q66 = (a*K_h/R)^2 * sigma2_dh_thermal
    %       sigma2_dh_thermal = 4*kBT*Ts/(gamma_N*c_perp(h_bar))
    %       + optional sigma2_w_a_direct floor.
    %   Q66_physical_mode (false)    -> tracking-residual driven:
    %       Q66 = (a*K_h/R)^2 * (sigma2_dxr_hat(z) - sigma2_n_proj)
    %       + optional sigma2_w_a_direct floor (kept as fallback).
    %   else (both false)            -> legacy engineering margin only:
    %       Q66 = a_nom^2 * sigma2_w_a_direct
    config.Q66_OL_mode = true;          % unified default (Layer 1 physical thermal)
    config.Q66_physical_mode = false;   % alternative branch (kept as fallback)
    config.sigma2_w_a_direct = 0;       % [(um/pN)^2/step], optional floor on Q66

    % Force Q77 = 0 hard override (motion_control_law_eq17_core.m force_Q77_zero).
    % Default true: under positioning / wall-disabled scenarios the sinusoidal-
    % derived Q77 (a^2 * (K_h^2 - K_h')^2 * h_dot_max^4 / (8 R^4) + ...) is not
    % the dominant a-uncertainty source — let sigma2_w_fA (Q77 floor) or the
    % Q66_OL_mode physical thermal channel handle it instead.
    % Empirical: any Q77 floor (1e-9..1e-7) degrades a_z rstd from ~3-5% to
    % 15-18% via Jordan-block accumulation (2026-05-06 sweep).
    config.force_Q77_zero = true;

    % t_warmup_kf: G1 (KF warm-up gate) duration in seconds.
    %   t_warmup_kf > 0: during t < t_warmup_kf, R(2,2) = R_OFF (y_2 gated)
    %                    AND K_kf(6,:)=K_kf(7,:)=0 (Stage 10 Option A: a_hat
    %                    locked at init, slot 6/7 measurement update blocked).
    %   t_warmup_kf = 0: G1 disabled, EKF runs normally from t=0.
    %   Default 0 since 477bb0a/c5f3dcc validation: with iir_warmup_mode='prefill'
    %   + Pf_init=Riccati, Wave 4 mode never triggers under h=50 positioning,
    %   ramp 50->5, or 5-seed sweep. Set > 0 if running a scenario where you
    %   want G1 protection (e.g., reproducing pre-2026-05-05 baseline).
    %   (build_eq17_constants' own fallback default is 0.2 — this config
    %   default takes precedence for all dual_track runs.)
    config.t_warmup_kf = 0;

    % iir_warmup_mode: how to initialize IIR LP / EWMA states.
    %   'legacy'  - dx_bar_m=0, sigma2_dxr_hat=0, warmup_count=2
    %               (controller emits f_d=0 for first 3 calls; IIR accumulates from 0)
    %   'prefill' - dx_bar_m=0, sigma2_dxr_hat = 4*kBT*a_x_init.*C_dpmr_eff
    %               + C_np_eff.*sigma2_n_s (per-axis steady-state at h_init);
    %               warmup_count=0 (controller emits real f_d from call 2 onward).
    %               Requires fixed h_init at start; ramp/motion lag <= 1 IIR tau.
    config.iir_warmup_mode = 'prefill';

    % NOTE (a_cov cross-controller conflict, tracked for 議題 1):
    %   eq17's verified baseline uses a_cov = 0.05 (Phase 0 §6 lock), but
    %   apply_qr_preset above sets a_cov = 0.005 (eq6 frozen_correct value).
    %   eq17 run scripts must override config.a_cov = 0.05 explicitly until
    %   the Q/R philosophy unification (議題 1) resolves the shared field.
    % ----------------------------------------------------------------

    % Thermal
    config.thermal_enable = true;

    % Simulation
    config.T_sim = 5;

end
