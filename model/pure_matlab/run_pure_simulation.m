function simOut = run_pure_simulation(config, opts)
%RUN_PURE_SIMULATION Pure-MATLAB time-stepping driver for the eq17_7state path.
%
%   simOut = run_pure_simulation(config)
%   simOut = run_pure_simulation(config, opts)
%
%   Replaces sim('system_model') for the eq17_7state controller. Mirrors
%   the Simulink ToWorkspace schema so downstream analysis scripts are
%   drop-in compatible (see agent_docs/dual-track-simulation-design.md
%   decision 5).
%
%   This driver supports ONLY controller_type=='eq17_7state'. type=7 /
%   type=23 stay on the Simulink path.
%
%   Inputs:
%       config - Same struct as run_simulation.m's config (after
%                user_config() + overrides). Must contain at minimum:
%                  config.T_sim
%                  config.lambda_c
%                  config.meas_noise_enable
%                  config.meas_noise_std (3x1 [um])
%                  config.a_cov
%                Plus all wall/traj/ctrl fields user_config() defines.
%       opts   - (optional) struct with fields (all defaulted):
%                  opts.seed     - RNG seed                   (default 42)
%                  opts.verbose  - print progress every 10%   (default false)
%                  opts.scheme   - integration scheme tag     (default 'ode4_step10us')
%
%   Output simOut struct (mirrors Simulink ToWorkspace, all [Nx3] except
%   tout / ekf_out / meta):
%       simOut.p_d_out    - Desired position pd[k]      [N x 3, um]
%       simOut.f_d_out    - Control force f_d[k]        [N x 3, pN]
%       simOut.F_th_out   - Thermal force F_th[k]       [N x 3, pN]
%       simOut.p_m_out    - Measured position           [N x 3, um]
%                              (after measurement noise injection,
%                               before sensor delay — matches Simulink
%                               p_m_out tap point)
%       simOut.tout       - Time vector                 [N x 1, sec]
%       simOut.ekf_out    - EKF diagnostic              [N x 4]
%                              [a_hat_x; a_hat_z; a_hat_y; h_bar]
%       simOut.meta       - struct(config, params_value, scheme, seed,
%                                  driver, driver_version)
%
%   See also: step_dynamics, motion_control_law_eq17_7state,
%             trajectory_generator, calc_thermal_force, calc_gamma_inv

    % ------------------------------------------------------------------
    % 0. Defaults for opts
    % ------------------------------------------------------------------
    if nargin < 2 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seed');         opts.seed         = 42;                 end
    if ~isfield(opts, 'verbose');      opts.verbose      = false;              end
    if ~isfield(opts, 'scheme');       opts.scheme       = 'ode4_step10us';    end
    % Phase 9 R(2,2) validation extensions:
    %   opts.a_hat_freeze  - 3x1 [um/pN] to lock state(6) per axis (default empty)
    %   opts.collect_diag  - bool, accumulate per-step diag time series
    if ~isfield(opts, 'a_hat_freeze'); opts.a_hat_freeze = [];                 end
    if ~isfield(opts, 'collect_diag'); opts.collect_diag = false;              end

    % ------------------------------------------------------------------
    % 1. Resolve params via existing builder (Simulink.Parameter wrapper)
    % ------------------------------------------------------------------
    params = calc_simulation_params(config);
    P = params.Value;   % unwrap to plain struct (used by all .m functions)

    % ------------------------------------------------------------------
    % 2. RNG seeding (decision 6 — independent randn, same concept seed)
    % ------------------------------------------------------------------
    rng(opts.seed);

    % ------------------------------------------------------------------
    % 3. Clear controller / trajectory persistent state
    %    (mirrors run_simulation.m line 6 pattern)
    % ------------------------------------------------------------------
    clear motion_control_law motion_control_law_23state ...
          motion_control_law_7state motion_control_law_eq17_7state ...
          trajectory_generator calc_thermal_force; %#ok<CLFUNC>

    % ------------------------------------------------------------------
    % 4. Build offline constants for eq17_7state controller
    % ------------------------------------------------------------------
    eq17_opts.lambda_c   = config.lambda_c;
    eq17_opts.option     = 'A_MA2_full';
    eq17_opts.sigma2_n_s = (config.meas_noise_std(:)).^2;     % 3x1 [um^2]
    eq17_opts.kBT        = P.ctrl.k_B * P.ctrl.T;              % [pN*um]
    eq17_opts.t_warmup_kf = 0.2;
    if isfield(config, 't_warmup_kf') && ~isempty(config.t_warmup_kf)
        eq17_opts.t_warmup_kf = config.t_warmup_kf;   % Phase 3 sweep override
    end
    eq17_opts.h_bar_safe  = 1.5;
    eq17_opts.d           = 2;
    eq17_opts.a_cov       = config.a_cov;
    % Wave 2D: separate a_pd (LP for δp_md mean) from a_cov (EWMA for σ²_δxr).
    eq17_opts.a_pd        = config.a_pd;
    % Wave 2D: f_D random-walk innovation variance (Phase 5 §5.4 Q55, baseline 0)
    if isfield(config, 'sigma2_w_fD')
        eq17_opts.sigma2_w_fD = config.sigma2_w_fD;
    else
        eq17_opts.sigma2_w_fD = 0;
    end
    % Phase 5 §5.5: a_x random-walk innovation variance (Q77 floor, baseline 0)
    if isfield(config, 'sigma2_w_fA')
        eq17_opts.sigma2_w_fA = config.sigma2_w_fA;
    else
        eq17_opts.sigma2_w_fA = 0;
    end
    % Stage 11 Option I: per-axis effective C_dpmr_eff / C_np_eff from calc_ctrl_params
    if isfield(P.ctrl, 'C_dpmr_eff_per_axis')
        eq17_opts.C_dpmr_eff_per_axis = P.ctrl.C_dpmr_eff_per_axis;
    end
    if isfield(P.ctrl, 'C_np_eff_per_axis')
        eq17_opts.C_np_eff_per_axis = P.ctrl.C_np_eff_per_axis;
    end
    % X2a: per-axis empirical IF_eff calibration (Phase 9 Stage I).
    if isfield(P.ctrl, 'IF_eff_calibrated_per_axis') && ...
            ~isempty(P.ctrl.IF_eff_calibrated_per_axis)
        eq17_opts.IF_eff_calibrated = P.ctrl.IF_eff_calibrated_per_axis;
    end
    % Wave 4 motion-ramp test pass-throughs
    if isfield(config, 'force_Q77_zero')
        eq17_opts.force_Q77_zero = config.force_Q77_zero;
    end
    if isfield(config, 'h_dot_max_override')
        eq17_opts.h_dot_max_override = config.h_dot_max_override;
    end
    if isfield(config, 'h_ddot_max_override')
        eq17_opts.h_ddot_max_override = config.h_ddot_max_override;
    end
    if isfield(config, 'Pf_init_slot7')
        eq17_opts.Pf_init_slot7 = config.Pf_init_slot7;
    end
    if isfield(config, 'iir_warmup_mode') && ~isempty(config.iir_warmup_mode)
        eq17_opts.iir_warmup_mode = config.iir_warmup_mode;
    end
    ctrl_const = build_eq17_constants(eq17_opts);

    % Phase 9 R(2,2) validation: optional a_hat freeze (locks state(6) per axis)
    if ~isempty(opts.a_hat_freeze)
        ctrl_const.a_hat_freeze = opts.a_hat_freeze(:);   % 3x1 [um/pN]
    end

    % ------------------------------------------------------------------
    % 5. Time grid (matches Simulink: discrete samples 0, Ts, 2Ts, ...)
    % ------------------------------------------------------------------
    Ts = P.common.Ts;
    T_sim = config.T_sim;
    N = round(T_sim / Ts) + 1;            % number of discrete samples
    tout = (0:N-1)' * Ts;                 % column vector [Nx1, sec]

    % ------------------------------------------------------------------
    % 6. Initialize state
    % ------------------------------------------------------------------
    p0 = P.common.p0;                     % 3x1 [um]
    p_curr = p0;                          % continuous state at sample boundary

    % Sensor-delay buffer of length d+1. Newest at end, oldest at index 1.
    % Initial condition matches Simulink Delay(d, Ts) IC=p0.
    %
    % Phase 8 Wave 2D fix (2026-04-29): a buffer of length d implements only
    % a (d-1)-step delay because the per-step ordering is
    %   (i)  read p_m_delayed = buffer(:,1)            <-- read BEFORE shift
    %   (ii) shift: buffer = [buffer(:,2:end), p_m_raw]
    % so on step k, buffer(:,1) = p_m at step (k - (d-1)).
    %
    % To realize a true d-step delay (matches design spec + Simulink Delay(d)),
    % buffer must hold (d+1) entries: the read at (i) returns p_m at step
    % (k - d). Agent B's controller has pd_km1/pd_km2 aligned to d=2 (Eq.17 v2).
    %
    % References: phase8_eq17_state_audit.md §K item 2; phase8_wave2D_driver.md
    d_delay = ctrl_const.d;
    p_m_buffer = repmat(p0, 1, d_delay + 1);  % [3 x (d+1)]

    % Trajectory unit-delay buffer:
    %   trajectory_generator returns p_d[k+1]; controller wants pd[k] and
    %   p_d_out logs pd[k]. Unit Delay IC = p0 → pd_for_ctrl[1] = p0.
    pd_for_ctrl = p0;                     % 3x1 [um]

    % ------------------------------------------------------------------
    % 7. Allocate logs (mirror Simulink ToWorkspace schema, [N x 3])
    % ------------------------------------------------------------------
    p_d_out  = zeros(N, 3);
    f_d_out  = zeros(N, 3);
    F_th_out = zeros(N, 3);
    p_m_out  = zeros(N, 3);
    ekf_out  = zeros(N, 4);

    % Phase 9 R(2,2) validation: optional diag time-series accumulators
    if opts.collect_diag
        diag_log.dx_r              = zeros(N, 3);
        diag_log.sigma2_dxr_hat    = zeros(N, 3);
        diag_log.a_xm              = zeros(N, 3);
        diag_log.delta_x_m         = zeros(N, 3);
        diag_log.innovation_y2     = zeros(N, 3);
        diag_log.K_kf_a_y2         = zeros(N, 3);
        diag_log.K_kf_dx_y1        = zeros(N, 3);
        diag_log.P_a               = zeros(N, 3);
        diag_log.P_dx              = zeros(N, 3);
        diag_log.P77               = zeros(N, 3);
        diag_log.Q77               = zeros(N, 3);
        diag_log.a_hat             = zeros(N, 3);
        diag_log.x_D_hat           = zeros(N, 3);
        diag_log.delta_a_hat       = zeros(N, 3);
        diag_log.gate_active       = false(N, 3);
        diag_log.guards_individual = false(N, 3, 3);   % [N x guard x axis]
        diag_log.h_bar             = zeros(N, 1);
    end

    % ------------------------------------------------------------------
    % 8. Time-stepping loop  (matches Simulink ordering)
    % ------------------------------------------------------------------
    if opts.verbose
        fprintf('[run_pure_simulation] N=%d steps, Ts=%.4e s, scheme=%s, seed=%d\n', ...
                N, Ts, opts.scheme, opts.seed);
    end

    for k = 1:N
        t_now = tout(k);

        % --- (a) Trajectory: returns pd[k+1] and del_pd[k] = pd[k+1]-pd[k]
        [pd_kp1, del_pd_k] = trajectory_generator(t_now, P);

        % --- (b) Controller input pd[k] is the unit-delayed trajectory
        pd_k = pd_for_ctrl;

        % --- (c) Sensor-delayed p_m for controller (head of buffer)
        p_m_delayed = p_m_buffer(:, 1);

        % --- (d) Controller + EKF
        if opts.collect_diag
            [f_d_k, ekf_k, diag_k] = motion_control_law_eq17_7state( ...
                                del_pd_k, pd_k, p_m_delayed, P, ctrl_const);
        else
            [f_d_k, ekf_k] = motion_control_law_eq17_7state( ...
                                del_pd_k, pd_k, p_m_delayed, P, ctrl_const);
        end

        % --- (e) Thermal force at current continuous-state position
        %   NOTE: calc_thermal_force does NOT check P.thermal.enable internally;
        %   gate is at driver level (matches Simulink block-level enable).
        if P.thermal.enable > 0.5
            f_th_k = calc_thermal_force(p_curr, P);
        else
            f_th_k = zeros(3, 1);
        end

        % --- (f) Total force (ZOH constant over [t_k, t_k + Ts])
        F_total = f_d_k + f_th_k;

        % --- (g) Continuous dynamics: integrate one Ts using ode4 substeps
        p_curr = step_dynamics(p_curr, F_total, P, Ts);

        % --- (h) Measurement noise injection (matches Simulink Measurement_Noise)
        if config.meas_noise_enable
            n_meas = config.meas_noise_std(:) .* randn(3, 1);
        else
            n_meas = zeros(3, 1);
        end
        p_m_raw = p_curr + n_meas;        % tapped to p_m_out (before delay)

        % --- (i) Shift sensor-delay buffer: drop oldest, append newest
        p_m_buffer = [p_m_buffer(:, 2:end), p_m_raw];

        % --- (j) Logging
        p_d_out(k, :)  = pd_k.';
        f_d_out(k, :)  = f_d_k.';
        F_th_out(k, :) = f_th_k.';
        p_m_out(k, :)  = p_m_raw.';
        ekf_out(k, :)  = ekf_k.';

        % Phase 9 R(2,2) validation: log diag time-series
        if opts.collect_diag
            diag_log.dx_r(k, :)              = diag_k.dx_r.';
            diag_log.sigma2_dxr_hat(k, :)    = diag_k.sigma2_dxr_hat.';
            diag_log.a_xm(k, :)              = diag_k.a_xm.';
            diag_log.delta_x_m(k, :)         = diag_k.delta_x_m.';
            diag_log.innovation_y2(k, :)     = diag_k.innovation_y2.';
            diag_log.K_kf_a_y2(k, :)         = diag_k.K_kf_a_y2.';
            diag_log.K_kf_dx_y1(k, :)        = diag_k.K_kf_dx_y1.';
            diag_log.P_a(k, :)               = diag_k.P_a.';
            diag_log.P_dx(k, :)              = diag_k.P_dx.';
            diag_log.P77(k, :)               = diag_k.P77.';
            diag_log.Q77(k, :)               = diag_k.Q77.';
            diag_log.a_hat(k, :)             = diag_k.a_hat.';
            diag_log.x_D_hat(k, :)           = diag_k.x_D_hat.';
            diag_log.delta_a_hat(k, :)       = diag_k.delta_a_hat.';
            diag_log.gate_active(k, :)       = diag_k.gate_active_per_axis(:).';
            diag_log.guards_individual(k, :, :) = diag_k.guards_individual;
            diag_log.h_bar(k)                = diag_k.h_bar;
        end

        % --- (k) Update unit-delay for next step's controller input
        pd_for_ctrl = pd_kp1;

        % --- (l) Verbose progress
        if opts.verbose && N >= 10 && mod(k, max(1, round(N/10))) == 0
            fprintf('  step %d/%d (t=%.3fs)\n', k, N, t_now);
        end
    end

    % ------------------------------------------------------------------
    % 9. Pack output (mirror Simulink ToWorkspace)
    % ------------------------------------------------------------------
    simOut.p_d_out  = p_d_out;
    simOut.f_d_out  = f_d_out;
    simOut.F_th_out = F_th_out;
    simOut.p_m_out  = p_m_out;
    simOut.tout     = tout;
    simOut.ekf_out  = ekf_out;
    simOut.meta = struct( ...
        'config',         config, ...
        'params_value',   P, ...
        'scheme',         opts.scheme, ...
        'seed',           opts.seed, ...
        'driver',         'run_pure_simulation', ...
        'driver_version', '1.1');

    % Phase 9 R(2,2) validation: optional diag time-series + ctrl_const
    if opts.collect_diag
        simOut.diag = diag_log;
    end
    simOut.ctrl_const = ctrl_const;
end
