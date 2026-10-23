% FORK OF test_script/integration/run_5state_powerlaw.m @ ae41395 | PURPOSE: driver for temp_mcl_powerlaw_diag | EXPIRES: powerlaw handoff open items resolved | 產線改動不會自動跟上
function simOut = temp_run_powerlaw_diag(config, opts)
%TEMP_RUN_5STATE_POWERLAW  Standalone pure-MATLAB driver for the NEW 5-state
%   power-law-gain eq17 controller (temp_mcl_powerlaw_diag).
%
%   Mirrors run_pure_simulation.m's per-step ordering (RNG seed -> params ->
%   trajectory -> controller -> thermal -> step_dynamics -> sensor-delay buffer)
%   but hard-dispatches to the new controller instead of touching the shared
%   driver's variant switch. Scratch smoke-test harness -- delete after the
%   power-law verification (chat 2026-07-24).
%
%   simOut = temp_run_powerlaw_diag(config)
%   simOut = temp_run_powerlaw_diag(config, opts)   % opts.seed, opts.verbose
%
%   Output fields (subset of the ToWorkspace schema needed for the smoke test):
%       p_d_out, f_d_out, F_th_out, p_m_out, p_true_out, a_true_out, tout,
%       ekf_out, a_hat_out (Nx3, x/y/z), p_hat_out (Nx3), h_bar_out (Nx1),
%       gate_out (Nx3 logical), meta.
%
%   See also: run_pure_simulation, temp_mcl_powerlaw_diag

    if nargin < 2 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seed');    opts.seed    = 42;    end
    if ~isfield(opts, 'verbose'); opts.verbose = false; end

    % --- RNG seed BEFORE params (thermal/meas seeds drawn from global rng) ---
    rng(opts.seed);

    params = calc_simulation_params(config);
    P = params.Value;

    clear temp_mcl_powerlaw_diag trajectory_generator calc_thermal_force;

    % --- Offline constants (dimension-agnostic; reuse the 6-state builder) ---
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
    % scratch diagnostic hooks: allow overriding Pf-init knobs from config
    if isfield(config, 'Pf_a_frac') && ~isempty(config.Pf_a_frac)
        ctrl_const.Pf_a_frac = config.Pf_a_frac;
    end
    if isfield(config, 'Pf_p_std') && ~isempty(config.Pf_p_std)
        ctrl_const.Pf_p_std = config.Pf_p_std;
    end
    if isfield(config, 'Q55_floor_hook') && ~isempty(config.Q55_floor_hook)
        ctrl_const.Q55_floor = config.Q55_floor_hook;
    end
    if isfield(config, 'p_fb_off') && ~isempty(config.p_fb_off)
        ctrl_const.p_fb_off = config.p_fb_off;
    end
    for fn = {'r2_scale','y2_decim','h24_fix','y2_whiten','a_init_honest','a_init_asym','drift_off','fe_use_fdet'}
        if isfield(config, fn{1}) && ~isempty(config.(fn{1}))
            ctrl_const.(fn{1}) = config.(fn{1});
        end
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

    a_nom_drv   = P.common.Ts / P.common.gamma_N;
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
    ekf_out  = zeros(N, 4);
    a_hat_out = zeros(N, 3);
    p_hat_out = zeros(N, 3);
    h_bar_out = zeros(N, 1);
    gate_out  = false(N, 3);
    a_xm_out  = zeros(N, 3);
    a_prime_out = zeros(N, 3);   % da_h/dh implied by power law (s_h/R)
    P_a_out     = zeros(N, 3);   % sqrt posterior var of a_h (slot 4)
    P_p_out     = zeros(N, 3);   % sqrt posterior var of p   (slot 5)
    innov_y2_out = zeros(N, 3);  % a_xm innovation
    K_a_y2_out   = zeros(N, 3);  % Kalman gain on a_h from y2
    DG = struct();
    dgf = {'dg_P45','dg_P44','dg_P55','dg_K25','dg_S2','dg_R2','dg_H25', ...
           'dg_sh','dg_sa','dg_K25_via_P45','dg_K25_via_P55H','dg_H24_drop', ...
           'delta_x_hat_3','P_dx','dx_r','sigma2_dxr_hat','dg_dA_y1','dg_dA_y2','dg_dA_pred'};
    for jj = 1:numel(dgf); DG.(dgf{jj}) = zeros(N, 3); end

    for k = 1:N
        t_now = tout(k);

        [pd_kp1, del_pd_k] = trajectory_generator(t_now, P);
        pd_k = pd_for_ctrl;
        p_m_delayed = p_m_buffer(:, 1);

        if wall_on_drv
            h_bar_true_k = max((dot(p_curr, P.wall.w_hat) - P.wall.pz) / P.common.R, h_bar_floor_drv);
            [c_para_k, c_perp_k] = calc_correction_functions(h_bar_true_k);
            a_true_k = [a_nom_drv / c_para_k; a_nom_drv / c_para_k; a_nom_drv / c_perp_k];
        else
            a_true_k = a_nom_drv * ones(3, 1);
        end

        if isfield(config, 'a_ctrl_true') && config.a_ctrl_true
            [f_d_k, ekf_k, diag_k] = temp_mcl_powerlaw_diag( ...
                                        del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_true_k);
        else
            [f_d_k, ekf_k, diag_k] = temp_mcl_powerlaw_diag( ...
                                        del_pd_k, pd_k, p_m_delayed, P, ctrl_const);
        end

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
        p_hat_out(k, :) = diag_k.p_hat.';
        h_bar_out(k)    = diag_k.h_bar;
        gate_out(k, :)  = diag_k.gate_active_per_axis(:).';
        a_xm_out(k, :)  = diag_k.a_xm(:).';
        a_prime_out(k, :) = diag_k.a_prime_hat(:).';
        P_a_out(k, :)     = sqrt(max(diag_k.P_a(:).', 0));
        P_p_out(k, :)     = sqrt(max(diag_k.P77(:).', 0));
        innov_y2_out(k, :) = diag_k.innovation_y2(:).';
        K_a_y2_out(k, :)   = diag_k.K_kf_a_y2(:).';
        for jj = 1:numel(dgf); DG.(dgf{jj})(k, :) = diag_k.(dgf{jj})(:).'; end

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
    simOut.tout       = tout;
    simOut.ekf_out    = ekf_out;
    simOut.a_hat_out  = a_hat_out;
    simOut.p_hat_out  = p_hat_out;
    simOut.h_bar_out  = h_bar_out;
    simOut.gate_out   = gate_out;
    simOut.a_xm_out   = a_xm_out;
    simOut.a_prime_out = a_prime_out;
    simOut.P_a_out     = P_a_out;
    simOut.P_p_out     = P_p_out;
    simOut.innov_y2_out = innov_y2_out;
    simOut.K_a_y2_out   = K_a_y2_out;
    simOut.DG           = DG;
    simOut.meta = struct('config', config, 'params_value', P, ...
                         'seed', opts.seed, 'driver', 'temp_run_powerlaw_diag');
    simOut.ctrl_const = ctrl_const;
end
