function simOut = run_5state_expgain(config, opts)
%RUN_5STATE_EXPGAIN  Pure-MATLAB driver for the 5-state exponential-gain eq17
%   controller (motion_control_law_5state_expgain).
%
%   Mirrors run_pure_simulation.m's per-step ordering (RNG seed -> params ->
%   trajectory -> controller -> thermal -> step_dynamics -> sensor-delay
%   buffer) but hard-dispatches to the exponential-gain controller instead of
%   going through the shared driver's variant switch.
%
%   This driver is COMMITTED on purpose: the derivation document
%   reference/eq17_analysis/derivation/5state_expgain_hd.tex and its figures
%   depend on it, and a permanent artefact must not depend on a gitignored
%   temp_* script (see the scratch-hygiene note, 2026-07-27).
%
%   simOut = run_5state_expgain(config)
%   simOut = run_5state_expgain(config, opts)
%
%   opts fields:
%       .seed          RNG seed                                  (default 42)
%       .verbose       progress printout                         (default false)
%       .ctrl_const_override  struct merged into ctrl_const after it is built,
%                      used to flip the controller knobs (y2_whiten,
%                      fe_row4_full, use_fdet, Pf_a_frac, Pf_b_std, b_init,
%                      a_init_mode, h_bar_min_traj, Q55_floor)
%       .a_ctrl_override  3x1 [um/pN] fed to the control law only
%
%   Output (superset of the ToWorkspace schema the analysis scripts need):
%       p_d_out, f_d_out, F_th_out, p_m_out, p_true_out, a_true_out, tout,
%       ekf_out, a_hat_out, b_hat_out, h_bar_out, h_bar_d_out, gate_out,
%       a_xm_out, a_prime_out, a_prime_true_out, P_a_out, P_b_out,
%       innov_y2_out, K_a_y2_out, R2_out, ctrl_const, meta.
%   Gains in the logs are in um/pN (physical); a_prime_* are d a_h / d h_bar in
%   the SAME um/pN convention (i.e. non-dimensional slope times R).
%
%   See also: motion_control_law_5state_expgain, smoke_5state_expgain

    if nargin < 2 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seed');    opts.seed    = 42;    end
    if ~isfield(opts, 'verbose'); opts.verbose = false; end
    if ~isfield(opts, 'ctrl_const_override'); opts.ctrl_const_override = struct(); end
    if ~isfield(opts, 'a_ctrl_override');     opts.a_ctrl_override = [];    end
    % 'diff' = motion_control_law_5state_expgain (a_h a state, swept by a')
    % 'alg'  = motion_control_law_5state_expgain_alg (a_h evaluated; states a_o, b)
    if ~isfield(opts, 'variant');             opts.variant = 'diff';        end

    % --- RNG seed BEFORE params (thermal/meas seeds drawn from global rng) ---
    rng(opts.seed);

    params = calc_simulation_params(config);
    P = params.Value;

    switch lower(opts.variant)
        case 'diff'
            ctrl_fcn = @motion_control_law_5state_expgain;
        case 'alg'
            ctrl_fcn = @motion_control_law_5state_expgain_alg;
        otherwise
            error('run_5state_expgain:badVariant', ...
                  'opts.variant must be ''diff'' or ''alg'', got ''%s''.', opts.variant);
    end
    clear motion_control_law_5state_expgain motion_control_law_5state_expgain_alg ...
          trajectory_generator calc_thermal_force;

    % --- Offline constants (dimension-agnostic; reuse the 6-state builder) ---
    eq17_opts = struct();
    eq17_opts.lambda_c    = config.lambda_c;
    eq17_opts.option      = 'A_MA2_full';
    eq17_opts.sigma2_n_s  = (config.meas_noise_std(:)).^2;
    eq17_opts.kBT         = P.ctrl.k_B * P.ctrl.T;
    eq17_opts.d           = 2;
    eq17_opts.a_cov       = config.a_cov;
    eq17_opts.a_pd        = config.a_pd;
    eq17_opts.t_warmup_kf = 0;                 % prefill init: no warm-up gate
    eq17_opts.h_bar_safe  = 1.5;
    if isfield(config, 'h_bar_safe') && ~isempty(config.h_bar_safe)
        eq17_opts.h_bar_safe = config.h_bar_safe;
    end
    eq17_opts.iir_warmup_mode = 'prefill';
    ctrl_const = build_eq17_6state_constants(eq17_opts);

    % knob overrides (controller defaults are the honest ones; these are for
    % ablations and the acceptance sweeps)
    ov = opts.ctrl_const_override;
    fn = fieldnames(ov);
    for i = 1:numel(fn)
        ctrl_const.(fn{i}) = ov.(fn{i});
    end

    % --- Time grid ---
    Ts = P.common.Ts;
    N = round(config.T_sim / Ts) + 1;
    tout = (0:N-1)' * Ts;

    % --- State init ---
    p0 = P.common.p0;
    p_curr = p0;
    d_delay = ctrl_const.d;
    p_m_buffer = repmat(p0, 1, d_delay + 1);   % d+1 slots -> true d-step delay
    pd_for_ctrl = p0;

    R_drv       = P.common.R;
    a_nom_drv   = P.common.Ts / P.common.gamma_N;      % [um/pN] far-field
    wall_on_drv = isfield(P, 'wall') && P.wall.enable_wall_effect > 0.5;
    if wall_on_drv && isfield(P.wall, 'h_bar_min')
        h_bar_floor_drv = P.wall.h_bar_min;
    else
        h_bar_floor_drv = 1.001;
    end

    % --- Logs ---
    p_d_out  = zeros(N, 3);
    f_d_out  = zeros(N, 3);
    F_th_out = zeros(N, 3);
    p_m_out  = zeros(N, 3);
    p_true_out = zeros(N, 3);
    a_true_out = zeros(N, 3);
    a_prime_true_out = zeros(N, 3);
    ekf_out  = zeros(N, 4);
    a_hat_out = zeros(N, 3);
    b_hat_out = zeros(N, 3);
    ao_hat_out = zeros(N, 3);
    h_bar_out = zeros(N, 1);
    h_bar_true_out = zeros(N, 1);
    h_bar_d_out = zeros(N, 1);
    gate_out  = false(N, 3);
    a_xm_out  = zeros(N, 3);
    a_prime_out = zeros(N, 3);
    P_a_out     = zeros(N, 3);
    P_b_out     = zeros(N, 3);
    innov_y2_out = zeros(N, 3);
    K_a_y2_out   = zeros(N, 3);
    R2_out       = zeros(N, 3);
    dx_r_out     = zeros(N, 3);   % IIR residual the gain readout is built from [um]
    dh_m_out     = zeros(N, 3);   % delayed position error fed to the controller [um]

    for k = 1:N
        t_now = tout(k);

        [pd_kp1, del_pd_k] = trajectory_generator(t_now, P);
        pd_k = pd_for_ctrl;
        p_m_delayed = p_m_buffer(:, 1);

        if wall_on_drv
            h_bar_true_k = max((dot(p_curr, P.wall.w_hat) - P.wall.pz) / R_drv, h_bar_floor_drv);
            [c_para_k, c_perp_k] = calc_correction_functions(h_bar_true_k);
            a_true_k = [a_nom_drv / c_para_k; a_nom_drv / c_para_k; a_nom_drv / c_perp_k];
            % true d a_h / d h_bar by central difference on the exact curve
            a_prime_true_out(k, :) = local_a_prime_true(h_bar_true_k, a_nom_drv, h_bar_floor_drv).';
        else
            h_bar_true_k = Inf;
            a_true_k = a_nom_drv * ones(3, 1);
        end

        % opts.a_ctrl_override = 'true' feeds the EXACT gain to the control law
        % every step (the EKF still estimates its own). This pins the closed
        % loop at the ideal one the C_dpmr / C_n constants were derived for, so
        % any residual readout bias cannot be blamed on a_hat being wrong.
        if ischar(opts.a_ctrl_override) || isstring(opts.a_ctrl_override)
            a_ov_k = a_true_k;
        else
            a_ov_k = opts.a_ctrl_override;
        end
        [f_d_k, ekf_k, diag_k] = ctrl_fcn(del_pd_k, pd_k, p_m_delayed, P, ...
                                          ctrl_const, a_ov_k);

        if P.thermal.enable > 0.5
            f_th_k = calc_thermal_force(p_curr, P);
        else
            f_th_k = zeros(3, 1);
        end

        F_total = f_d_k + f_th_k;
        p_curr = step_dynamics(p_curr, F_total, P, Ts);

        if config.meas_noise_enable
            n_meas = config.meas_noise_std(:) .* randn(3, 1);
        else
            n_meas = zeros(3, 1);
        end
        p_m_raw = p_curr + n_meas;
        p_m_buffer = [p_m_buffer(:, 2:end), p_m_raw];

        p_d_out(k, :)  = pd_k.';
        f_d_out(k, :)  = f_d_k.';
        F_th_out(k, :) = f_th_k.';
        p_m_out(k, :)  = p_m_raw.';
        p_true_out(k, :) = p_curr.';
        a_true_out(k, :) = a_true_k.';
        ekf_out(k, :)  = ekf_k.';
        a_hat_out(k, :) = diag_k.a_hat.';
        b_hat_out(k, :) = diag_k.b_hat.';
        if isfield(diag_k, 'a_o_hat'); ao_hat_out(k, :) = diag_k.a_o_hat.'; end
        h_bar_out(k)    = diag_k.h_bar;
        h_bar_true_out(k) = h_bar_true_k;
        h_bar_d_out(k)  = diag_k.h_bar_d;
        gate_out(k, :)  = diag_k.gate_active_per_axis(:).';
        a_xm_out(k, :)  = diag_k.a_xm(:).';
        a_prime_out(k, :) = diag_k.a_prime_hat(:).' * R_drv;   % 1/pN -> um/pN
        P_a_out(k, :)     = sqrt(max(diag_k.P_a(:).', 0));
        P_b_out(k, :)     = sqrt(max(diag_k.P77(:).', 0));
        innov_y2_out(k, :) = diag_k.innovation_y2(:).';
        K_a_y2_out(k, :)   = diag_k.K_kf_a_y2(:).';
        R2_out(k, :)       = diag_k.R2(:).';
        dx_r_out(k, :)     = diag_k.dx_r(:).';
        dh_m_out(k, :)     = diag_k.delta_x_m(:).';

        pd_for_ctrl = pd_kp1;

        if opts.verbose && N >= 10 && mod(k, max(1, round(N/10))) == 0
            fprintf('  step %d/%d (t=%.3fs)\n', k, N, t_now);
        end
    end

    simOut.p_d_out    = p_d_out;
    simOut.f_d_out    = f_d_out;
    simOut.F_th_out   = F_th_out;
    simOut.p_m_out    = p_m_out;
    simOut.p_true_out = p_true_out;
    simOut.a_true_out = a_true_out;
    simOut.a_prime_true_out = a_prime_true_out;
    simOut.tout       = tout;
    simOut.ekf_out    = ekf_out;
    simOut.a_hat_out  = a_hat_out;
    simOut.b_hat_out  = b_hat_out;
    simOut.ao_hat_out = ao_hat_out;
    simOut.p_hat_out  = b_hat_out;      % alias for the shared analysis scripts
    simOut.h_bar_out  = h_bar_out;
    simOut.h_bar_true_out = h_bar_true_out;
    simOut.h_bar_d_out = h_bar_d_out;
    simOut.gate_out   = gate_out;
    simOut.a_xm_out   = a_xm_out;
    simOut.a_prime_out = a_prime_out;
    simOut.P_a_out     = P_a_out;
    simOut.P_b_out     = P_b_out;
    simOut.innov_y2_out = innov_y2_out;
    simOut.K_a_y2_out   = K_a_y2_out;
    simOut.R2_out       = R2_out;
    simOut.dx_r_out     = dx_r_out;
    simOut.dh_m_out     = dh_m_out;
    simOut.a_nom        = a_nom_drv;
    simOut.R            = R_drv;
    simOut.meta = struct('config', config, 'params_value', P, ...
                         'seed', opts.seed, 'driver', 'run_5state_expgain', ...
                         'ctrl_const_override', ov);
    simOut.ctrl_const = ctrl_const;
end


function ap = local_a_prime_true(h_bar, a_nom, h_floor)
%LOCAL_A_PRIME_TRUE  d a_h / d h_bar on the EXACT correction curve [um/pN].
%   Central difference; used only as the reference in the analysis, never by
%   the controller.
    dh = 1e-4 * max(h_bar, 1);
    hp = h_bar + dh;
    hm = max(h_bar - dh, h_floor);
    [cp_p, ce_p] = calc_correction_functions(hp);
    [cp_m, ce_m] = calc_correction_functions(hm);
    den = hp - hm;
    ap_para = a_nom * (1/cp_p - 1/cp_m) / den;
    ap_perp = a_nom * (1/ce_p - 1/ce_m) / den;
    ap = [ap_para; ap_para; ap_perp];
end
