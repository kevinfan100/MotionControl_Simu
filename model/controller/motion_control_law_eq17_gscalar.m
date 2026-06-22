function [f_d, ekf_out, diag] = motion_control_law_eq17_gscalar(del_pd, pd, p_m, params, ctrl_const, a_ctrl_override)
%MOTION_CONTROL_LAW_EQ17_GSCALAR  Scalar-gain (pooled) eq17 controller.
%   The per-axis EKF gain state is REMOVED. The motion gain factorizes exactly
%
%       a_x,i[k] = g * m_i[k],   g := Ts/gamma_N (one slow scalar),
%                                m_i[k] := 1/c_i(h_bar[k])  (KNOWN wall shape)
%
%   so instead of estimating 3 fast per-axis gains from the noisy a_xm channel,
%   we estimate ONE slow scalar  s ~ g/a_nom (~1)  pooled across the 3 axes and
%   the whole trajectory (where SNR is strong), then COMPUTE the gain used by
%   the control law:
%
%       a_ctrl,i[k] = s_hat[k] * a_det,i[k],   a_det,i = a_nom / c_i(h_bar_d)
%
%   Near-wall samples auto-downweight (large R22 -> tiny inverse-variance
%   weight), so s_hat is pinned by the far-field; near-wall a_hat inherits the
%   far-field accuracy. The near-wall a_hat collapse is dissolved, not masked.
%
%   s estimator (information / weighted-least-squares with forgetting beta_s):
%       a_xm,i = s*a_det,i + n_a,i,  Var(n_a,i) = R22,i
%       p_i    = a_det,i^2 / R22,i                       (inverse-variance weight)
%       I[k]   = (1-beta_s)*I[k-1] + sum_i p_i           (pooled precision)
%       J[k]   = (1-beta_s)*J[k-1] + sum_i a_det,i*a_xm,i/R22,i
%       s_hat  = clamp(J/I, s_min, s_max),   Var(s_hat) = 1/I
%   Weights use a_det (deterministic) -> unbiased WLS, no s_hat->R22 loop.
%   Only the warmup guard (G1) gates the estimator; G2 (a_xm<=0) and G3 (wall)
%   are intentionally NOT applied (truncating the noise tail biases s high).
%
%   Built as a reduction of motion_control_law_eq17_4state: the control law and
%   the IIR a_xm chain are unchanged; the EKF predict/update, Q/R matrix builder
%   and DARE init are dropped (slots 1-3 never enter the control law).
%
%   [f_d, ekf_out]       = motion_control_law_eq17_gscalar(del_pd, pd, p_m, params, ctrl_const)
%   [f_d, ekf_out, diag] = motion_control_law_eq17_gscalar(..., a_ctrl_override)
%
%   ctrl_const extras (with fallbacks): beta_s (0.002), use_hbar_meas_for_adet
%   (false), I_prior (200), s_min/s_max (0.5/2).
%
%   See also: motion_control_law_eq17_4state, run_pure_simulation,
%             build_eq17_6state_constants, calc_correction_functions

    % ------------------------------------------------------------------
    % Optional gain override (true-gain A/B experiment).
    % ------------------------------------------------------------------
    if nargin < 6
        a_ctrl_override = [];
    end
    has_override = ~isempty(a_ctrl_override);
    if has_override
        a_ctrl_override = a_ctrl_override(:);
        assert(numel(a_ctrl_override) == 3 && all(isfinite(a_ctrl_override)) && all(a_ctrl_override > 0), ...
               'motion_control_law_eq17_gscalar:badOverride', ...
               'a_ctrl_override must be a 3x1 finite positive vector.');
    end

    % ------------------------------------------------------------------
    % Open-loop bypass
    % ------------------------------------------------------------------
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = [1; 1; 1; 0];
        if nargout >= 3
            diag = empty_diag_gscalar();
            diag.f_d = f_d;
        end
        return;
    end

    % ------------------------------------------------------------------
    % Persistent state
    % ------------------------------------------------------------------
    persistent dx_bar_m            % 3x1 IIR LP mean of delta_x_m [um]
    persistent sigma2_dxr_hat      % 3x1 EWMA variance of dx_r    [um^2]
    persistent pd_km1 pd_km2       % trajectory delay buffers (also h_bar_d history)
    persistent f_d_km1 f_d_km2     % past control buffers (for Sigma a*f_d[k-i])
    persistent a_ctrl_km1 a_ctrl_km2          % control-law gain history
    persistent var_da_ram_km1 var_da_ram_km2  % past var(delta_a_ram[k-i]) (R22 delay)
    persistent warmup_count k_step
    persistent I_g J_g s_hat       % scalar-s information filter

    persistent initialized
    persistent lambda_c d_delay Ts kBT R_radius gamma_N_p a_nom_p
    persistent a_pd a_cov C_dpmr C_n K_var IF_abc xi_per_axis var_da_inc_factor
    persistent t_warmup_kf amlpf_var_factor
    persistent sigma2_n_s a_x_init enable_wall w_hat_n pz_wall
    persistent beta_s I_prior s_min s_max use_hbar_meas_for_adet

    % ------------------------------------------------------------------
    % [0] Initialization on first call
    % ------------------------------------------------------------------
    if isempty(initialized)
        initialized = true;

        % --- 0A. params constants ---
        Ts        = params.ctrl.Ts;
        kBT       = params.ctrl.k_B * params.ctrl.T;     % [pN*um]
        R_radius  = params.common.R;
        gamma_N_p = params.ctrl.gamma;                   % [pN*sec/um]
        sigma2_n_s = params.ctrl.sigma2_noise;           % 3x1 [um^2]
        a_nom_p   = Ts / gamma_N_p;                       % [um/pN]

        % --- 0B. ctrl_const (offline scalars; shared with 4/5/6-state) ---
        lambda_c        = ctrl_const.lambda_c;
        d_delay         = ctrl_const.d;
        C_dpmr          = ctrl_const.C_dpmr;
        C_n             = ctrl_const.C_n;
        K_var           = ctrl_const.K_var;
        IF_abc          = ctrl_const.IF_abc(:);     % [A;B;C] for exact per-step IF_eff
        xi_per_axis     = ctrl_const.xi_per_axis(:);
        t_warmup_kf     = ctrl_const.t_warmup_kf;
        a_cov           = ctrl_const.a_cov;
        a_pd            = ctrl_const.a_pd;
        if isfield(ctrl_const, 'amlpf_var_factor') && ~isempty(ctrl_const.amlpf_var_factor)
            amlpf_var_factor = ctrl_const.amlpf_var_factor;
        else
            amlpf_var_factor = 1;
        end
        if isfield(ctrl_const, 'var_da_increment_factor') && ~isempty(ctrl_const.var_da_increment_factor)
            var_da_inc_factor = ctrl_const.var_da_increment_factor;
        else
            var_da_inc_factor = 2 / (1 + lambda_c);     % closed-form fallback
        end

        % --- 0B'. gscalar-only knobs (with fallbacks) ---
        if isfield(ctrl_const, 'beta_s') && ~isempty(ctrl_const.beta_s)
            beta_s = ctrl_const.beta_s;
        else
            beta_s = 0.002;
        end
        if isfield(ctrl_const, 'use_hbar_meas_for_adet') && ~isempty(ctrl_const.use_hbar_meas_for_adet)
            use_hbar_meas_for_adet = ctrl_const.use_hbar_meas_for_adet > 0.5;
        else
            use_hbar_meas_for_adet = false;
        end
        if isfield(ctrl_const, 's_I_prior') && ~isempty(ctrl_const.s_I_prior)
            I_prior = ctrl_const.s_I_prior;
        else
            I_prior = 200;          % prior precision: s ~ N(1, ~0.07^2)
        end
        s_min = 0.5;
        s_max = 2.0;

        % --- 0C. Wall geometry ---
        if isfield(params, 'wall')
            w_hat_n     = params.wall.w_hat;
            pz_wall     = params.wall.pz;
            enable_wall = params.wall.enable_wall_effect > 0.5;
        else
            w_hat_n     = [0; 0; 1];
            pz_wall     = 0;
            enable_wall = false;
        end

        % --- 0E. Wall-aware a_x[0] seeding (for IIR prefill + R22 delay init) ---
        if enable_wall && isfield(params, 'common') && isfield(params.common, 'p0')
            p0_init    = params.common.p0(:);
            h_init_um  = dot(p0_init, w_hat_n) - pz_wall;
            h_bar_init = max(h_init_um / R_radius, 1.001);
            [c_para0, c_perp0, derivs0] = calc_correction_functions(h_bar_init, true);
            a_x_init   = [a_nom_p / c_para0; a_nom_p / c_para0; a_nom_p / c_perp0];
            K_h_init   = [derivs0.K_h_para; derivs0.K_h_para; derivs0.K_h_perp];
            a_perp_init = a_nom_p / c_perp0;
        else
            a_x_init    = [a_nom_p; a_nom_p; a_nom_p];
            K_h_init    = zeros(3, 1);
            a_perp_init = a_nom_p;
        end

        % --- 0F. R22-delay history seed (var(delta_a_ram) at init) ---
        sigma2_dh_init  = 4 * kBT * a_perp_init;
        var_da_init_vec = var_da_inc_factor .* (a_x_init .* K_h_init / R_radius).^2 * sigma2_dh_init;

        % --- 0G. IIR states (prefill default) ---
        dx_bar_m = zeros(3, 1);
        iir_mode = 'prefill';
        if isfield(ctrl_const, 'iir_warmup_mode') && ~isempty(ctrl_const.iir_warmup_mode)
            iir_mode = ctrl_const.iir_warmup_mode;
        end
        if strcmpi(iir_mode, 'prefill')
            sigma2_dxr_hat = 4 * kBT * a_x_init * C_dpmr + C_n * sigma2_n_s;
            warmup_count   = 0;
        else
            sigma2_dxr_hat = zeros(3, 1);
            warmup_count   = 2;
        end

        % --- 0H. Delay / control buffers ---
        pd_km1  = pd;
        pd_km2  = pd;
        f_d_km1 = zeros(3, 1);
        f_d_km2 = zeros(3, 1);
        a_ctrl_km1 = a_x_init;            a_ctrl_km2 = a_x_init;
        var_da_ram_km1 = var_da_init_vec; var_da_ram_km2 = var_da_init_vec;

        % --- 0I. Scalar-s filter init: prior centered at s=1 ---
        I_g   = I_prior;
        J_g   = I_prior;            % J/I = 1 -> s_hat = 1
        s_hat = 1;

        % --- 0J. Misc ---
        k_step = 1;

        % --- 0K. First call returns zeros (no f_d yet) ---
        f_d = zeros(3, 1);
        ekf_out = [a_x_init(1); a_x_init(3); a_x_init(2); 0];
        if nargout >= 3
            diag = empty_diag_gscalar();
            diag.f_d            = f_d;
            diag.a_hat          = a_x_init;
            diag.sigma2_dxr_hat = sigma2_dxr_hat;
            diag.s_hat          = s_hat;
            diag.var_s          = 1 / I_g;
            if has_override
                diag.a_ctrl_used = a_ctrl_override;
            else
                diag.a_ctrl_used = a_x_init;
            end
        end
        return;
    end

    one_minus_lc = 1 - lambda_c;

    % ------------------------------------------------------------------
    % Per-step: measurement, desired-height gain, control gain
    % ------------------------------------------------------------------
    if d_delay == 2
        pd_km_d = pd_km2;
    elseif d_delay == 1
        pd_km_d = pd_km1;
    else
        error('motion_control_law_eq17_gscalar:unsupportedDelay', ...
              'Only d=1 or d=2 supported, got d=%g.', d_delay);
    end
    delta_x_m = pd_km_d - p_m;          % 3x1 [um]
    pd_kp1    = pd + del_pd;             % 3x1 [um]

    % h_bar from current measurement (wall functions + R22 delay term)
    if enable_wall
        h_bar = (dot(p_m, w_hat_n) - pz_wall) / R_radius;
    else
        h_bar = Inf;
    end

    % a_det = a_nom / c(h_bar). Default: DESIRED height (noise-free, delay-aligned,
    % matches the proven 4-state feedforward). Flag switches to measured height.
    if use_hbar_meas_for_adet
        a_det = local_a_x_det(p_m, w_hat_n, pz_wall, R_radius, enable_wall, a_nom_p);
    else
        a_det = local_a_x_det(pd, w_hat_n, pz_wall, R_radius, enable_wall, a_nom_p);
    end

    % Control gain from the PREVIOUS-step scalar estimate (posterior[k-1]).
    s_hat_prev = s_hat;
    if has_override
        a_ctrl = a_ctrl_override;        % true-gain (or externally supplied) gain
    else
        a_ctrl = s_hat_prev .* a_det;    % computed gain (scalar x known wall shape)
    end

    % ------------------------------------------------------------------
    % [1] IIR a_xm (paper 2025 Eq.9-13) -- unchanged from 4-state
    % ------------------------------------------------------------------
    dx_bar_m_new = (1 - a_pd) * dx_bar_m + a_pd * delta_x_m;
    dx_r = delta_x_m - dx_bar_m_new;
    sigma2_dxr_hat_new = (1 - a_cov) * sigma2_dxr_hat + a_cov * dx_r.^2;
    a_xm = (sigma2_dxr_hat_new - C_n * sigma2_n_s) / (C_dpmr * 4 * kBT);   % 3x1 [um/pN]

    % ------------------------------------------------------------------
    % [2] Warmup gate (legacy mode only; prefill -> warmup_count=0 skips this)
    % ------------------------------------------------------------------
    if warmup_count > 0
        f_d = zeros(3, 1);
        dx_bar_m       = dx_bar_m_new;
        sigma2_dxr_hat = sigma2_dxr_hat_new;
        pd_km2 = pd_km1; pd_km1 = pd;
        f_d_km2 = f_d_km1; f_d_km1 = f_d;
        a_ctrl_km2 = a_ctrl_km1; a_ctrl_km1 = a_ctrl;
        warmup_count = warmup_count - 1;
        k_step = k_step + 1;
        a_hat_report = s_hat .* a_det;
        h_bar_now = local_h_bar_out(enable_wall, h_bar);
        ekf_out = [a_hat_report(1); a_hat_report(3); a_hat_report(2); h_bar_now];
        if nargout >= 3
            diag = empty_diag_gscalar();
            diag.f_d = f_d; diag.a_hat = a_hat_report;
            diag.a_ctrl_used = a_ctrl;
            diag.sigma2_dxr_hat = sigma2_dxr_hat_new; diag.a_xm = a_xm;
            diag.delta_x_m = delta_x_m; diag.h_bar = h_bar; diag.dx_r = dx_r;
            diag.a_x_det = a_det; diag.s_hat = s_hat; diag.var_s = 1 / I_g;
        end
        return;
    end

    % ------------------------------------------------------------------
    % [3] Control law (B2 ACTIVE form, Vpersonal p.2; NO -x_hat_D term) -- verbatim
    %   f_d = a_ctrl^-1 { Delta_x_d[k;d] + (1-lc)[dx_m - sum a_ctrl[k-i]*f_d[k-i]] }
    % ------------------------------------------------------------------
    if d_delay == 2
        sum_a_fd_past = a_ctrl_km1 .* f_d_km1 + a_ctrl_km2 .* f_d_km2;
    else
        sum_a_fd_past = a_ctrl_km1 .* f_d_km1;
    end
    inv_a_ctrl = 1 ./ a_ctrl;
    f_d = inv_a_ctrl .* (pd_kp1 - lambda_c * pd - one_minus_lc * pd_km_d ...
                         + one_minus_lc * delta_x_m ...
                         - one_minus_lc * sum_a_fd_past);

    % ------------------------------------------------------------------
    % [4] R22 per axis + inverse-variance accumulation for scalar s
    %   R22 / weights use a_det (deterministic) -> unbiased WLS, loop-free.
    % ------------------------------------------------------------------
    if enable_wall && isfinite(h_bar) && h_bar > 1
        [~, c_perp_h, derivs] = calc_correction_functions(h_bar, true);
        K_h_axis = [derivs.K_h_para; derivs.K_h_para; derivs.K_h_perp];
        a_perp_meas = Ts / (gamma_N_p * c_perp_h);
    else
        K_h_axis = zeros(3, 1);
        a_perp_meas = Ts / gamma_N_p;
    end
    sigma2_dh = 4 * kBT * a_perp_meas;   % wall-normal thermal motion variance

    t_now = (k_step - 1) * Ts;
    G1 = (t_now < t_warmup_kf);           % warmup guard (only gate on s-estimator)

    var_da_ram = zeros(3, 1);
    R22_v      = zeros(3, 1);
    sum_p = 0;
    sum_J = 0;
    for ax = 1:3
        a_op = a_det(ax);
        var_da_ram(ax) = var_da_inc_factor * (a_op * K_h_axis(ax) / R_radius)^2 * sigma2_dh;
        IF_eff_i = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a_op, sigma2_n_s(ax));
        R2_intrinsic = amlpf_var_factor * K_var * IF_eff_i * (a_op + xi_per_axis(ax))^2;
        if d_delay == 2
            R22_delay = var_da_ram_km1(ax) + var_da_ram_km2(ax);
        else
            R22_delay = var_da_ram_km1(ax);
        end
        R22 = R2_intrinsic + R22_delay;
        R22_v(ax) = R22;
        p_i = a_op^2 / R22;
        sum_p = sum_p + p_i;
        sum_J = sum_J + a_op * a_xm(ax) / R22;   % = p_i * (a_xm/a_det); raw a_xm (no G2/G3)
    end

    % ------------------------------------------------------------------
    % [5] Scalar-s information update (forgetting beta_s); frozen during warmup
    % ------------------------------------------------------------------
    if ~G1
        I_g = (1 - beta_s) * I_g + sum_p;
        J_g = (1 - beta_s) * J_g + sum_J;
        s_raw = J_g / I_g;
        s_hat = min(max(s_raw, s_min), s_max);
    end
    var_s = 1 / I_g;

    % ------------------------------------------------------------------
    % [6] Bookkeeping: shift buffers, IIR states, step counter
    % ------------------------------------------------------------------
    pd_km2 = pd_km1; pd_km1 = pd;
    f_d_km2 = f_d_km1; f_d_km1 = f_d;
    a_ctrl_km2 = a_ctrl_km1; a_ctrl_km1 = a_ctrl;
    var_da_ram_km2 = var_da_ram_km1; var_da_ram_km1 = var_da_ram;
    dx_bar_m = dx_bar_m_new;
    sigma2_dxr_hat = sigma2_dxr_hat_new;
    k_step = k_step + 1;

    % ------------------------------------------------------------------
    % [7] Output: a_hat = (updated) s_hat * a_det
    % ------------------------------------------------------------------
    a_hat_report = s_hat .* a_det;
    h_bar_now = local_h_bar_out(enable_wall, h_bar);
    ekf_out = [a_hat_report(1); a_hat_report(3); a_hat_report(2); h_bar_now];

    if nargout >= 3
        G_flags = false(3, 3);
        G_flags(1, :) = G1;
        diag = empty_diag_gscalar();
        diag.sigma2_dxr_hat       = sigma2_dxr_hat_new;
        diag.a_xm                 = a_xm;
        diag.delta_x_m            = delta_x_m;
        diag.gate_active_per_axis = false(3, 1);   % no per-axis gate-off in gscalar
        diag.guards_individual    = G_flags;
        diag.h_bar                = h_bar;
        diag.f_d                  = f_d;
        diag.dx_r                 = dx_r;
        diag.a_hat                = a_hat_report;
        diag.a_ctrl_used          = a_ctrl;
        diag.var_da_ram           = var_da_ram;
        diag.a_x_det              = a_det;
        diag.s_hat                = s_hat;
        diag.var_s                = var_s;
        diag.R22                  = R22_v;
    end
end


%% =================== Local Helpers ===================

function a_det = local_a_x_det(p_q, w_hat_n, pz_wall, R_radius, enable_wall, a_nom)
%LOCAL_A_X_DET  Deterministic gain a_x_det = a_nom/c(h_bar) at the position p_q.
%   Per axis: x,y use c_para; z uses c_perp (w_hat-normal). p_q is the DESIRED
%   position p_d by default (exogenous), or the measured p_m when the
%   use_hbar_meas_for_adet flag is set. Returns a_nom for all axes when the wall
%   is disabled. h_bar clamped to >1 (c diverges at the wall).
    if enable_wall
        h_bar_q = (dot(p_q, w_hat_n) - pz_wall) / R_radius;
        h_bar_q = max(h_bar_q, 1.001);
        [c_para_q, c_perp_q] = calc_correction_functions(h_bar_q);
        a_det = [a_nom / c_para_q; a_nom / c_para_q; a_nom / c_perp_q];
    else
        a_det = a_nom * ones(3, 1);
    end
end


function IF = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a, sigma2_nx)
%IF_EFF_EVAL  Exact color-inflation factor IF_eff for R22 (R22_derivation S4-S6).
    sxT = 4 * kBT * a;
    num = sxT^2 * IF_abc(1) + 2 * sxT * sigma2_nx * IF_abc(2) + sigma2_nx^2 * IF_abc(3);
    den = (C_dpmr * sxT + C_n * sigma2_nx)^2;
    IF  = 1 + 2 * num / den;
end


function h = local_h_bar_out(enable_wall, h_bar)
    if enable_wall
        h = h_bar;
    else
        h = 0;
    end
end


function d = empty_diag_gscalar()
%EMPTY_DIAG_GSCALAR  Zeroed diagnostic struct (driver-compatible field set).
    d = struct();
    d.sigma2_dxr_hat    = zeros(3, 1);
    d.a_xm              = zeros(3, 1);
    d.delta_x_m         = zeros(3, 1);
    d.innovation_y2     = zeros(3, 1);
    d.K_kf_a_y2         = zeros(3, 1);
    d.K_kf_dx_y1        = zeros(3, 1);
    d.P_a               = zeros(3, 1);
    d.P_dx              = zeros(3, 1);
    d.x_D_hat              = zeros(3, 1);   % no disturbance state (driver compat)
    d.delta_a_hat          = zeros(3, 1);   % no rate state (driver compat)
    d.da_x_ff              = zeros(3, 1);   % no predict feedforward (gain computed)
    d.a_x_det              = zeros(3, 1);
    d.gate_active_per_axis = false(3, 1);
    d.guards_individual    = false(3, 3);
    d.h_bar                = 0;
    d.f_d                  = zeros(3, 1);
    d.dx_r                 = zeros(3, 1);
    d.a_hat                = zeros(3, 1);
    d.P77                  = zeros(3, 1);   % driver compat
    d.Q77                  = zeros(3, 1);
    d.var_da_ram           = zeros(3, 1);
    d.delta_x_hat_1        = zeros(3, 1);
    d.P_dx1                = zeros(3, 1);
    d.a_ctrl_used          = zeros(3, 1);
    d.s_hat                = 1;             % scalar gain estimate
    d.var_s                = 0;             % Var(s_hat) = 1/I_g
    d.R22                  = zeros(3, 1);
end
