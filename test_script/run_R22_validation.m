function results = run_R22_validation(scenario, seed)
%RUN_R22_VALIDATION  Phase 9 R(2,2) validation single-shot wrapper.
%
%   results = run_R22_validation(scenario, seed)
%
%   Drives one Phase 9 positioning run (h_init=50, no oscillation, 2-step
%   warmup, a_hat frozen at the per-axis Stokes truth) and returns the
%   diag time-series + config + ctrl_const for downstream R(2,2) analysis
%   by Agent Z.
%
%   ----- Inputs -----
%       scenario - struct with fields (all optional):
%           .a_cov            EWMA weight for sigma2_dxr (default 0.05)
%           .sigma2_n_factor  Multiplier on meas-noise variance (default 1.0)
%                             meas_noise_std is scaled by sqrt(factor) so
%                             that variance scales by `factor`.
%           .T_sim            Sim duration [sec]   (default 30)
%           .h_init           Initial wall distance [um] (default 50)
%       seed     - RNG seed (integer, required)
%
%   ----- Output (struct results) -----
%       .diag              - per-step diag time series (from
%                            run_pure_simulation, opts.collect_diag=true)
%       .simOut            - full simOut struct (p_d_out, p_m_out, ekf_out, ...)
%       .config            - effective config used for the run
%       .ctrl_const        - controller offline constants
%       .params_value      - resolved params (P struct)
%       .a_true_per_axis   - 3x1 [um/pN] per-axis Stokes truth at h_init
%       .scenario          - input scenario struct (echoed)
%       .seed              - seed (echoed)
%       .meta              - timestamps, file fingerprints
%
%   ----- Phase 9 common config overrides (per plan §3) -----
%       trajectory_type='positioning', amplitude=0, frequency=0, t_hold=0
%       meas_noise_enable = TRUE (default false in user_config)
%       meas_noise_std    = [0.62e-3; 0.057e-3; 3.31e-3] um (Phase 6 §3.2)
%       thermal_enable    = true
%       sigma2_w_fA = 0, sigma2_w_fD = 0
%       theta = 0, phi = 0
%       lambda_c = 0.7, controller_type = 23
%       a_pd = a_cov            (Phase 0 §6 lock — explicitly enforced)
%       ctrl_const.a_hat_freeze = a_true_per_axis (3x1)
%
%   ----- Notes -----
%       * rng(seed) is called BEFORE calc_simulation_params (Memory bug
%         project_phase1_axm_audit.md: parallel batch ordering).
%       * a_true_per_axis is computed analytically from h_init via
%         calc_correction_functions(h_bar = h_init/R).
%
%   See also: run_pure_simulation, motion_control_law_eq17_7state,
%             calc_correction_functions

    if nargin < 2 || isempty(seed)
        error('run_R22_validation:missingSeed', 'seed is required.');
    end
    if nargin < 1 || isempty(scenario); scenario = struct(); end
    if ~isfield(scenario, 'a_cov')           ; scenario.a_cov           = 0.05; end
    if ~isfield(scenario, 'sigma2_n_factor') ; scenario.sigma2_n_factor = 1.0;  end
    if ~isfield(scenario, 'T_sim')           ; scenario.T_sim           = 30;   end
    if ~isfield(scenario, 'h_init')          ; scenario.h_init          = 50;   end

    % --- 1. Path setup (mirror run_v2_h50_e2e.m) ---
    [script_dir, ~, ~] = fileparts(mfilename('fullpath'));
    project_root = fileparts(script_dir);
    addpath(fullfile(project_root, 'model'));
    addpath(fullfile(project_root, 'model', 'config'));
    addpath(fullfile(project_root, 'model', 'wall_effect'));
    addpath(fullfile(project_root, 'model', 'thermal_force'));
    addpath(fullfile(project_root, 'model', 'trajectory'));
    addpath(fullfile(project_root, 'model', 'controller'));
    addpath(fullfile(project_root, 'model', 'pure_matlab'));
    addpath(fullfile(project_root, 'model', 'diag'));

    % --- 2. Build base config from user_config defaults, then override ---
    config = user_config();

    % Geometry
    config.theta = 0;
    config.phi   = 0;

    % Trajectory: positioning hold at h_init
    config.trajectory_type = 'positioning';
    config.amplitude       = 0;
    config.frequency       = 0;
    config.t_hold          = 0;
    config.h_init          = scenario.h_init;
    config.h_bottom        = scenario.h_init;
    config.T_sim           = scenario.T_sim;

    % Controller
    config.ctrl_enable     = true;
    config.lambda_c        = 0.7;
    config.controller_type = 23;
    config.a_cov           = scenario.a_cov;
    config.a_pd            = scenario.a_cov;     % Phase 0 §6 lock
    config.sigma2_w_fA     = 0;                  % Phase 5 §5.5 baseline
    config.sigma2_w_fD     = 0;                  % Phase 5 §5.4 baseline

    % Measurement noise — explicitly enable (default false)
    base_meas_std = [0.62e-3; 0.057e-3; 3.31e-3];   % Phase 6 §3.2 [um]
    sigma_scale   = sqrt(scenario.sigma2_n_factor); % factor scales σ²; σ scales by sqrt
    config.meas_noise_enable = true;
    config.meas_noise_std    = base_meas_std * sigma_scale;

    % Thermal
    config.thermal_enable = true;

    % --- 3. RNG seed BEFORE calc_simulation_params (parallel-batch bug) ---
    rng(seed);

    % --- 4. Resolve params, compute per-axis a_true at h_init ---
    params = calc_simulation_params(config);
    P = params.Value;

    Ts        = P.common.Ts;
    gamma_N   = P.common.gamma_N;
    R_radius  = P.common.R;
    w_hat     = P.wall.w_hat;
    pz_wall   = P.wall.pz;
    p0        = P.common.p0;

    a_freespace = Ts / gamma_N;                          % [um/pN]
    h_init_um   = dot(p0, w_hat) - pz_wall;
    h_bar_init  = max(h_init_um / R_radius, 1.001);
    [c_para, c_perp] = calc_correction_functions(h_bar_init);
    a_true_per_axis = [a_freespace / c_para; ...
                       a_freespace / c_para; ...
                       a_freespace / c_perp];            % 3x1 [um/pN]

    % --- 5. Build run_pure_simulation opts (a_hat freeze + diag) ---
    run_opts.seed         = seed;
    run_opts.verbose      = false;
    run_opts.collect_diag = true;
    % freeze_a_hat default true; pass false in scenario to disable freeze
    if isfield(scenario, 'freeze_a_hat') && ~scenario.freeze_a_hat
        % no freeze: KF updates a_hat normally (production-like)
    else
        run_opts.a_hat_freeze = a_true_per_axis;         % lock state(6) per axis
    end

    % --- 6. Run sim ---
    simOut = run_pure_simulation(config, run_opts);

    % --- 7. Package results ---
    results.diag            = simOut.diag;
    results.simOut          = simOut;
    results.config          = config;
    results.ctrl_const      = simOut.ctrl_const;
    results.params_value    = P;
    results.a_true_per_axis = a_true_per_axis;
    results.scenario        = scenario;
    results.seed            = seed;
    results.meta            = struct( ...
        'created',         char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), ...
        'driver',          'run_R22_validation', ...
        'driver_version',  '1.0', ...
        'h_bar_init',      h_bar_init, ...
        'c_para',          c_para, ...
        'c_perp',          c_perp);
end
