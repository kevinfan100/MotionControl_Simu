function [params, cfg] = config(scenario, overrides)
%CONFIG All scenario parameters + nested params builder (single source).
%
%   [params, cfg] = config(scenario)
%   [params, cfg] = config(scenario, overrides)
%
%   scenario  : 'h50' (default) | 'h10' | 'osc1hz'
%   overrides : optional struct of non-rippling tunable overrides from the
%               main_run settings block (PACKAGING_PLAN.md 9a). Recognised
%               fields (any subset; empty values ignored):
%                   .lambda_c .a_pd .a_cov   controller / IIR poles
%                   .meas_noise .thermal     logical on/off
%               Scenario GEOMETRY (h_init / h_bottom / h_min / trajectory
%               type) and TIMING (T_sim, t_hold) are fixed by the scenario
%               name and are NOT overridable here. Overrides are applied to
%               the scalar tunables BEFORE the params builder, so the RNG
%               contract below (exactly two randi draws, fixed order) is
%               unaffected.
%
%   This file is THE place a human edits. It replaces the mother repo's
%   user_config + calc_simulation_params chain with fully explicit values
%   (no presets, no Simulink Bus objects) and builds the nested params
%   struct consumed by physics / sim / controller:
%
%       params.common   {Ts, R, gamma_N, T_sim, p0}
%       params.wall     {theta, phi, pz, h_min, w_hat, u_hat, v_hat,
%                        enable_wall_effect}
%       params.traj     {t_hold, h_init, h_bottom, amplitude, frequency,
%                        n_cycles, trajectory_type}
%       params.ctrl     {enable, lambda_c, gamma, Ts, k_B, T,
%                        meas_noise_enable, meas_noise_std, meas_noise_seed,
%                        a_pd, a_cov, sigma2_noise}
%       params.thermal  {enable, k_B, T, Ts, variance_coeff, seed}
%
%   cfg returns run-level knobs the driver reads directly
%   (scenario name, T_sim, d).
%
%   RNG CONTRACT (bit-exact equivalence with the mother repo):
%   the caller MUST call rng(seed) BEFORE config(). This builder then
%   consumes exactly two randi(2^31-1) draws from the global stream, in
%   the mother repo's order:
%       draw 1 -> params.ctrl.meas_noise_seed   (calc_ctrl_params:28)
%       draw 2 -> params.thermal.seed           (calc_thermal_params:19)
%   thermal_force re-seeds the global stream with params.thermal.seed on
%   its first call; all in-loop randomness then flows from that stream.
%   Do NOT reorder the blocks below.

    if nargin < 1 || isempty(scenario); scenario = 'h50'; end
    if nargin < 2 || isempty(overrides); overrides = struct(); end

    % ------------------------------------------------------------------
    % Physical constants (mother repo physical_constants.m, verbatim)
    % ------------------------------------------------------------------
    R       = 2.25;            % Particle radius [um]
    gamma_N = 0.0425;          % Stokes drag coefficient [pN*sec/um]
    Ts      = 1 / 1600;        % Sampling period [sec]
    k_B     = 1.3806503e-5;    % Boltzmann constant [pN*um/K]
    T       = 310.15;          % Temperature [K] (37 C)

    % ------------------------------------------------------------------
    % Scenario table (PACKAGING_PLAN.md decision 11)
    % ------------------------------------------------------------------
    switch lower(scenario)
        case 'h50'
            sc = struct('traj_type', 2, 'h_init', 50, 'h_bottom', 50, ...
                        'h_min', 1.5 * R, 'T_sim', 4, ...
                        'amplitude', 0, 'frequency', 1, 'n_cycles', 3, 't_descend', 0);
        case 'h10'
            sc = struct('traj_type', 2, 'h_init', 10, 'h_bottom', 10, ...
                        'h_min', 1.5 * R, 'T_sim', 4, ...
                        'amplitude', 0, 'frequency', 1, 'n_cycles', 3, 't_descend', 0);
        case 'osc1hz'
            % motion-test 1 Hz oscillation (gain_compare "osc_aggr"):
            % hold 0.5 s -> cosine descend 1.0 s -> 1 Hz osc (2 cycles) -> hold.
            % Trough h_bottom = 1.2*R = 2.7 um (h_bar 1.2); amplitude 2.5 um
            % -> osc h_bar in [1.2, 3.42] (crosses the wall region). The package
            % has NO guards, so near the trough a_hat degrades (L2 limitation).
            % h_min == h_bottom (trough sits on the L2 floor, no margin); the
            % plant errors only if the TRUE position falls below h_bar=1 (~15
            % sigma; see README sec 8).
            sc = struct('traj_type', 1, 'h_init', 50, 'h_bottom', 1.2 * R, ...
                        'h_min', 1.2 * R, 'T_sim', 4, ...
                        'amplitude', 2.5, 'frequency', 1, 'n_cycles', 2, 't_descend', 1.0);
        otherwise
            error('config:badScenario', ...
                  'Unknown scenario "%s" (use h50 | h10 | osc1hz).', scenario);
    end

    % ------------------------------------------------------------------
    % Tunables (explicit; mother repo values, y noise = 0.57 nm corrected)
    % ------------------------------------------------------------------
    lambda_c       = 0.7;                          % closed-loop pole
    a_pd           = 0.05;                         % IIR mean-EWMA pole
    a_cov          = 0.05;                         % IIR variance-EWMA pole
    meas_noise_std = [0.00062; 0.00057; 0.00331];  % [um] per-axis sensor std
    ctrl_enable    = true;
    meas_noise_on  = true;
    thermal_on     = true;
    d              = 2;                            % sensor delay [steps], hardcoded

    % ------------------------------------------------------------------
    % Apply main_run overrides (PACKAGING_PLAN 9a) to the scalar tunables
    % only -- this happens BEFORE the params builder, so the two randi
    % draws below stay in the same place and order (RNG contract intact).
    % Scenario geometry AND timing are not overridable here (see header).
    % ------------------------------------------------------------------
    if isfield(overrides, 'lambda_c')   && ~isempty(overrides.lambda_c);   lambda_c      = overrides.lambda_c;             end
    if isfield(overrides, 'a_pd')       && ~isempty(overrides.a_pd);       a_pd          = overrides.a_pd;                 end
    if isfield(overrides, 'a_cov')      && ~isempty(overrides.a_cov);      a_cov         = overrides.a_cov;                end
    if isfield(overrides, 'meas_noise') && ~isempty(overrides.meas_noise); meas_noise_on = logical(overrides.meas_noise); end
    if isfield(overrides, 'thermal')    && ~isempty(overrides.thermal);    thermal_on    = logical(overrides.thermal);     end

    % ==================================================================
    % params builder -- block order REPLICATES calc_simulation_params
    % (wall -> traj -> p0 -> ctrl[randi 1] -> thermal[randi 2]).
    % ==================================================================

    % --- wall (calc_wall_params; default orientation theta = phi = 0) ---
    theta = deg2rad(0);
    phi   = deg2rad(0);
    params.wall.theta = theta;
    params.wall.phi   = phi;
    params.wall.pz    = 0;
    params.wall.h_min = sc.h_min;
    params.wall.h_bar_min = sc.h_min / R;
    params.wall.w_hat = [cos(theta)*sin(phi);  sin(theta)*sin(phi);  cos(phi)];
    params.wall.u_hat = [-cos(theta)*cos(phi); -sin(theta)*cos(phi); sin(phi)];
    params.wall.v_hat = [sin(theta); -cos(theta); 0];
    params.wall.enable_wall_effect = 1;

    % --- traj (calc_traj_params; types: 1 = osc, 2 = positioning, 3 = ramp) ---
    params.traj.t_hold    = 0.5;          % [sec] initial hold / EKF warm-up
    params.traj.h_init    = sc.h_init;
    params.traj.h_bottom  = sc.h_bottom;
    params.traj.amplitude = sc.amplitude; % [um] osc half-swing (trough->mid)
    params.traj.frequency = sc.frequency; % [Hz] osc frequency
    params.traj.n_cycles  = sc.n_cycles;  %      osc cycle count
    params.traj.t_descend_override = sc.t_descend; % [sec] osc descent (0=unused)
    params.traj.trajectory_type = sc.traj_type;

    % --- common + initial position (calc_initial_position) ---
    params.common.Ts      = Ts;
    params.common.R       = R;
    params.common.gamma_N = gamma_N;
    params.common.T_sim   = sc.T_sim;
    params.common.p0      = (params.wall.pz + sc.h_init) * params.wall.w_hat;

    % --- ctrl (calc_ctrl_params subset; randi draw 1) ---
    params.ctrl.enable            = double(ctrl_enable);
    params.ctrl.lambda_c          = lambda_c;
    params.ctrl.gamma             = gamma_N;
    params.ctrl.Ts                = Ts;
    params.ctrl.meas_noise_enable = double(meas_noise_on);
    params.ctrl.meas_noise_std    = meas_noise_std;
    params.ctrl.meas_noise_seed   = randi(2^31-1);     % RNG draw 1 (stream parity)
    params.ctrl.a_pd              = a_pd;
    params.ctrl.a_cov             = a_cov;
    params.ctrl.k_B               = k_B;
    params.ctrl.T                 = T;
    params.ctrl.sigma2_noise      = meas_noise_std.^2; % 3x1 [um^2]

    % --- thermal (calc_thermal_params; randi draw 2) ---
    params.thermal.enable = double(thermal_on);
    params.thermal.k_B    = k_B;
    params.thermal.T      = T;
    params.thermal.Ts     = Ts;
    params.thermal.variance_coeff = 4 * k_B * T * gamma_N / Ts;
    params.thermal.seed   = randi(2^31-1);             % RNG draw 2 (thermal stream)

    % ------------------------------------------------------------------
    % Run-level knobs for the driver
    % ------------------------------------------------------------------
    cfg = struct('scenario', lower(scenario), 'T_sim', sc.T_sim, 'd', d);
end
