function out = run_simulation(scenario, opts)
%RUN_SIMULATION Time-stepping driver (discrete controller + continuous plant).
%
%   out = run_simulation(scenario)
%   out = run_simulation(scenario, opts)
%
%   scenario : 'h50' | 'h10' | 'ramp2p7'   (see config.m)
%   opts     : .seed (default 42), .verbose (default false),
%              .ctrl_enable_override (default [] = use config value),
%              .T_sim (default [] = scenario value; gate scripts use
%               shorter runs -- note the ramp rate is set by the
%               SCENARIO T_sim in config.m, so a shorter opts.T_sim
%               truncates the descent rather than rescaling it)
%
%   Two-rate structure (mirrors the mother repo / Simulink model):
%       1600 Hz  discrete: trajectory, controller, thermal sample,
%                          measurement noise, sensor-delay buffer
%       10 us    continuous: ode4 integration of p_dot = Gamma_inv*F (ZOH)
%
%   Per-step order (a)-(k) is FAITHFUL to the mother repo driver
%   run_pure_simulation.m -- do not reorder; the RNG consumption sequence
%   (thermal randn then measurement randn each step) and the d-step
%   sensor-delay buffer (length d+1, read-before-shift, IC = p0) are
%   part of the bit-exact equivalence contract.
%
%   Output struct (Simulink ToWorkspace schema, all [N x 3] unless noted):
%       out.p_d_out     desired position p_d[k]            [um]
%       out.f_d_out     control force f_d[k]               [pN]
%       out.F_th_out    thermal force F_th[k]              [pN]
%       out.p_m_out     measured position (noise injected,
%                       BEFORE sensor delay)               [um]
%       out.p_true_out  noise-free true position (probe)   [um]
%       out.tout        time vector [N x 1, sec]
%       out.ekf_out     [a_hat_x a_hat_y a_hat_z h_bar]    [N x 4]
%                       (parts 3-4 interim build: measurement-chain probe
%                        [delta_x_m_z sigma2_dxr_hat_z a_xm_z h_bar] --
%                        final form arrives with the EKF in part 5/7)
%       out.meta        struct(scenario, params, seed, driver_version)
%
%   See also: config, controller_6state, step_dynamics, thermal_force,
%             trajectory_ref

    if nargin < 1 || isempty(scenario); scenario = 'h50'; end
    if nargin < 2 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seed');    opts.seed    = 42;    end
    if ~isfield(opts, 'verbose'); opts.verbose = false; end
    if ~isfield(opts, 'ctrl_enable_override'); opts.ctrl_enable_override = []; end
    if ~isfield(opts, 'T_sim');   opts.T_sim   = [];    end

    % ------------------------------------------------------------------
    % 1. RNG seeding -- MUST precede config(): the params builder consumes
    %    two randi draws (meas_noise_seed, thermal.seed) from this stream;
    %    see the RNG CONTRACT note in config.m.
    % ------------------------------------------------------------------
    rng(opts.seed);

    % ------------------------------------------------------------------
    % 2. Build params (explicit, no presets)
    % ------------------------------------------------------------------
    [params, cfg] = config(scenario);
    if ~isempty(opts.ctrl_enable_override)
        params.ctrl.enable = double(opts.ctrl_enable_override);
    end

    % ------------------------------------------------------------------
    % 3. Reset persistent state in stateful functions
    % ------------------------------------------------------------------
    clear controller_6state trajectory_ref thermal_force;

    % ------------------------------------------------------------------
    % 4. Time grid (discrete samples 0, Ts, 2Ts, ...)
    % ------------------------------------------------------------------
    Ts = params.common.Ts;
    T_run = cfg.T_sim;
    if ~isempty(opts.T_sim); T_run = opts.T_sim; end
    N = round(T_run / Ts) + 1;
    tout = (0:N-1)' * Ts;

    % ------------------------------------------------------------------
    % 5. Initialize state
    % ------------------------------------------------------------------
    p0 = params.common.p0;
    p_curr = p0;

    % Sensor-delay buffer, length d+1 (true d-step delay): the per-step
    % order is (i) read p_m_delayed = buffer(:,1), (ii) shift-append, so
    % buffer(:,1) at step k holds p_m from step k-d. IC = p0.
    d = cfg.d;
    % The controller hardcodes d = 2 (pd_km2 + two-term Sigma a*f_d); a
    % different cfg.d would silently desynchronize plant delay vs
    % compensation -- fail loudly instead (PACKAGING_PLAN decision 6).
    assert(d == 2, 'run_simulation:dContract', 'cfg.d must be 2 (got %g).', d);
    p_m_buffer = repmat(p0, 1, d + 1);

    % Trajectory unit-delay: trajectory_ref returns p_d[k+1]; the
    % controller and p_d_out want p_d[k]. IC = p0.
    pd_for_ctrl = p0;

    % ------------------------------------------------------------------
    % 6. Allocate logs
    % ------------------------------------------------------------------
    p_d_out    = zeros(N, 3);
    f_d_out    = zeros(N, 3);
    F_th_out   = zeros(N, 3);
    p_m_out    = zeros(N, 3);
    p_true_out = zeros(N, 3);
    ekf_out    = zeros(N, 4);

    % ------------------------------------------------------------------
    % 7. Time-stepping loop (order faithful to mother repo)
    % ------------------------------------------------------------------
    if opts.verbose
        fprintf('[run_simulation:%s] N=%d steps, Ts=%.4e s, seed=%d\n', ...
                scenario, N, Ts, opts.seed);
    end

    for k = 1:N
        t_now = tout(k);

        % --- (a) Trajectory: returns p_d[k+1] and del_pd[k]
        [pd_kp1, del_pd_k] = trajectory_ref(t_now, params);

        % --- (b) Controller input p_d[k] (unit-delayed trajectory)
        pd_k = pd_for_ctrl;

        % --- (c) Sensor-delayed measurement (head of buffer, before shift)
        p_m_delayed = p_m_buffer(:, 1);

        % --- (d) Controller + EKF
        [f_d_k, ekf_k] = controller_6state(del_pd_k, pd_k, p_m_delayed, params);

        % --- (e) Thermal force at current true position (driver-level gate)
        if params.thermal.enable > 0.5
            f_th_k = thermal_force(p_curr, params);
        else
            f_th_k = zeros(3, 1);
        end

        % --- (f) Total force, ZOH over [t_k, t_k + Ts]
        F_total = f_d_k + f_th_k;

        % --- (g) Continuous dynamics, one Ts of ode4 substeps
        p_curr = step_dynamics(p_curr, F_total, params, Ts);

        % --- (h) Measurement noise injection
        if params.ctrl.meas_noise_enable > 0.5
            n_meas = params.ctrl.meas_noise_std(:) .* randn(3, 1);
        else
            n_meas = zeros(3, 1);
        end
        p_m_raw = p_curr + n_meas;       % tapped to p_m_out (before delay)

        % --- (i) Shift sensor-delay buffer (drop oldest, append newest)
        p_m_buffer = [p_m_buffer(:, 2:end), p_m_raw];

        % --- (j) Logging
        p_d_out(k, :)    = pd_k.';
        f_d_out(k, :)    = f_d_k.';
        F_th_out(k, :)   = f_th_k.';
        p_m_out(k, :)    = p_m_raw.';
        p_true_out(k, :) = p_curr.';
        ekf_out(k, :)    = ekf_k.';

        % --- (k) Unit-delay update for next step's controller input
        pd_for_ctrl = pd_kp1;

        if opts.verbose && N >= 10 && mod(k, max(1, round(N/10))) == 0
            fprintf('  step %d/%d (t=%.3fs)\n', k, N, t_now);
        end
    end

    % ------------------------------------------------------------------
    % 8. Pack output
    % ------------------------------------------------------------------
    out.p_d_out    = p_d_out;
    out.f_d_out    = f_d_out;
    out.F_th_out   = F_th_out;
    out.p_m_out    = p_m_out;
    out.p_true_out = p_true_out;
    out.tout       = tout;
    out.ekf_out    = ekf_out;
    out.meta = struct('scenario', scenario, 'params', params, ...
                      'seed', opts.seed, 'T_run', T_run, ...
                      'driver_version', 'standalone-1.0');
end
