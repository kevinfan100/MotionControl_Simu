function [f_d, ekf_out, diag] = motion_control_law_eq17_4state_para(del_pd, pd, p_m, params, ctrl_const)
%MOTION_CONTROL_LAW_EQ17_4STATE_PARA  Integrated para-c 4-state EKF (EXPLORATORY).
%
%   [f_d, ekf_out]       = motion_control_law_eq17_4state_para(del_pd, pd, p_m, params, ctrl_const)
%   [f_d, ekf_out, diag] = motion_control_law_eq17_4state_para(...)
%
%   Per-axis 4-state EKF whose 4th state is the wall-shape CONSTANT theta of the
%   parametric gain model  a(h_bar; theta) = a_nom / c(h_bar; theta), instead of
%   the gain a_x itself (the production motion_control_law_eq17_4state estimates
%   a_x directly). The control gain is read as a_ctrl = a(h_bar; theta_hat).
%
%   This is the INTEGRATED counterpart of the validated DECOUPLED study
%   (test_script/integration/plot_para_aprime.m, where theta was estimated by a
%   standalone scalar RLS feeding the controller's a_ctrl_override hook). Here
%   theta lives INSIDE the EKF, so the theta<->dx cross-covariance is consistent.
%
%   Per-axis state (column = axis x,y,z):
%       x = [delta_x_1; delta_x_2; delta_x_3; theta]      (theta constant)
%
%   Parametric gain model (per axis):
%       basis:   z   -> phi = 1/(h_bar-1)   ;   x,y -> phi = 1/h_bar
%       gain:    c = 1 + theta*phi ;  a = a_nom/c ;  a_nom = Ts/gamma_N
%       deriv:   da/dtheta = -(a/c)*phi      (measurement Jacobian AND F_e(3,4))
%
%   F_e (4x4 per axis, only Row 3 time-varying through f_d history):
%       F_dx[k] = f_d[k] + (1-lc)*sum_{i=1..d} f_d[k-i]
%       F_e = [0 1 0        0;
%              0 0 1        0;
%              0 0 lc      -F_dx*(da/dtheta);
%              0 0 0        1];
%
%   Measurement (y1 linear, y2 NONLINEAR in theta):
%       y = [delta_x_m ; a_xm],  predicted = [dx1_hat ; a(h_bar; theta_hat)]
%       H = [1 0 0 0;
%            0 0 0  da/dtheta]
%       Because the controller's h_bar is computed from the DELAYED p_m[k-d], the
%       a_xm[k]~a[k-d] delay is handled by evaluating a() at this delayed h_bar.
%       => no sum_da_ff measurement correction (unlike the base 4-state).
%
%   Q = diag(0,0,Q33,0)  (Q33 identical to the base 4-state; Q_theta = 0).
%   R = diag(R11,R22)    (R11 = sigma2_n ; R22 identical to the base 4-state).
%   Gate (h_bar<h_bar_safe OR warmup OR sigma2_dxr<=C_n*sigma2_n): use y1 only.
%
%   Control law (Vpersonal B2 ACTIVE eq17 form, x_D = 0), a_ctrl = a(h_bar;theta_hat):
%       f_d = a_ctrl^-1 { pd[k+1] - lc*pd[k] - (1-lc)*pd[k-d] + (1-lc)*dx_m
%                         - (1-lc)*sum_i a_ctrl[k-i]*f_d[k-i] }
%
%   Reuses verbatim from motion_control_law_eq17_4state: the a_xm IIR chain
%   (dx_bar_m EWMA, sigma2_dxr_hat EWMA, a_xm linear inversion), Q33, R22, the
%   3-guard gate, delay buffers, and the eq17 control-law structure. ONLY slot 4
%   changed (a_x -> theta) plus the consequent nonlinear measurement.
%
%   NOT production. New file; modifies nothing existing.
%
%   See also: motion_control_law_eq17_4state, build_eq17_6state_constants,
%             calc_correction_functions, plot_para_aprime

    % ------------------------------------------------------------------
    % Open-loop bypass
    % ------------------------------------------------------------------
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = [1; 1; 1; 0];
        if nargout >= 3
            diag = empty_diag_para();
            diag.f_d = f_d;
        end
        return;
    end

    % ------------------------------------------------------------------
    % Persistent state
    % ------------------------------------------------------------------
    persistent x_e_per_axis        % 4x3 EKF state (col = axis); slot 4 = theta
    persistent P_per_axis          % cell{3} of 4x4 covariance
    persistent dx_bar_m            % 3x1 IIR LP mean of delta_x_m [um]
    persistent sigma2_dxr_hat      % 3x1 EWMA variance of dx_r    [um^2]
    persistent pd_km1 pd_km2       % trajectory delay buffers
    persistent f_d_km1 f_d_km2     % past control buffers (for Sigma f_d[k-i])
    persistent a_hat_km1 a_hat_km2          % past implied gains a(h_bar;theta)[k-i] (Q33)
    persistent var_da_inc_km1 var_da_inc_km2 % past increment var (Q33/R22 delay terms)
    persistent a_ctrl_km1 a_ctrl_km2   % control-law gain history
    persistent warmup_count k_step

    persistent initialized
    persistent lambda_c d_delay Ts kBT R_radius gamma_N_p a_nom_p
    persistent a_pd a_cov C_dpmr C_n K_var IF_abc xi_per_axis var_da_inc_factor
    persistent t_warmup_kf h_bar_safe R_OFF amlpf_var_factor
    persistent sigma2_n_s enable_wall w_hat_n pz_wall

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

        % --- 0B. ctrl_const (offline scalars; shared with base 4/6-state) ---
        lambda_c        = ctrl_const.lambda_c;
        d_delay         = ctrl_const.d;
        C_dpmr          = ctrl_const.C_dpmr;
        C_n             = ctrl_const.C_n;
        K_var           = ctrl_const.K_var;
        IF_abc          = ctrl_const.IF_abc(:);     % [A;B;C] for exact per-step IF_eff
        xi_per_axis     = ctrl_const.xi_per_axis(:);
        t_warmup_kf     = ctrl_const.t_warmup_kf;
        h_bar_safe      = ctrl_const.h_bar_safe;
        a_cov           = ctrl_const.a_cov;
        a_pd            = ctrl_const.a_pd;
        amlpf_var_factor = get_const(ctrl_const, 'amlpf_var_factor', 1);
        var_da_inc_factor = get_const(ctrl_const, 'var_da_increment_factor', 2 / (1 + lambda_c));

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

        % --- 0D. theta init (leading-order wall shape; overridable) ---
        %   z   -> Brenner 9/8  = 1.125  (c_perp ~ 1 + (9/8)/(h_bar-1))
        %   x,y -> Goldman 9/16 = 0.5625 (c_para ~ 1 + (9/16)/h_bar)
        theta_init = get_const(ctrl_const, 'theta_init', [0.5625; 0.5625; 1.125]);
        theta_init = theta_init(:);
        P_theta0   = get_const(ctrl_const, 'P_theta0', 9);

        % --- 0E. h_bar at init from p0 ---
        if enable_wall && isfield(params, 'common') && isfield(params.common, 'p0')
            p0_init    = params.common.p0(:);
            h_init_um  = dot(p0_init, w_hat_n) - pz_wall;
            h_bar_init = max(h_init_um / R_radius, 1.001);
            [~, c_perp0, derivs0] = calc_correction_functions(h_bar_init, true);
            K_h_init   = [derivs0.K_h_para; derivs0.K_h_para; derivs0.K_h_perp];
            a_perp_init = a_nom_p / c_perp0;
        else
            h_bar_init = Inf;
            K_h_init   = zeros(3, 1);
            a_perp_init = a_nom_p;
        end

        % --- 0F. implied gains a(h_bar_init; theta_init) per axis ---
        a_hat_init = zeros(3, 1);
        for ax = 1:3
            a_hat_init(ax) = para_gain_local(h_bar_init, theta_init(ax), ax == 3, a_nom_p);
        end

        % --- 0G. EKF state init (theta in slot 4; dx zero) ---
        x_e_per_axis = zeros(4, 3);
        x_e_per_axis(4, :) = theta_init.';

        % --- 0H. Pf init: dx block from 3-state DARE, theta block = P_theta0 ---
        one_minus_lc = 1 - lambda_c;
        F_dx3 = [0 1 0; 0 0 1; 0 0 lambda_c];
        H_dx3 = [1 0 0];
        sigma2_dh_init = 4 * kBT * a_perp_init;
        var_da_init_vec = zeros(3, 1);
        P_per_axis = cell(3, 1);
        for ax = 1:3
            a_i = a_hat_init(ax);
            var_da_init = var_da_inc_factor * (a_i * K_h_init(ax) / R_radius)^2 * sigma2_dh_init;
            var_da_init_vec(ax) = var_da_init;
            Q33_ss = 4 * kBT * a_i * (1 + one_minus_lc^2 * d_delay) ...
                     + one_minus_lc^2 * sigma2_n_s(ax);
            Q_dx3 = zeros(3); Q_dx3(3, 3) = Q33_ss;
            R_dx3 = sigma2_n_s(ax);
            P_dx3 = solve_dare_kf_local(F_dx3, H_dx3, Q_dx3, R_dx3);
            P_full = zeros(4);
            P_full(1:3, 1:3) = P_dx3;
            P_full(4, 4) = P_theta0;
            P_per_axis{ax} = P_full;
        end

        % --- 0I. IIR states (prefill) ---
        dx_bar_m = zeros(3, 1);
        iir_mode = get_const(ctrl_const, 'iir_warmup_mode', 'prefill');
        if strcmpi(iir_mode, 'prefill')
            sigma2_dxr_hat = 4 * kBT * a_hat_init * C_dpmr + C_n * sigma2_n_s;
            warmup_count   = 0;
        else
            sigma2_dxr_hat = zeros(3, 1);
            warmup_count   = 2;
        end

        % --- 0J. Delay buffers ---
        pd_km1  = pd;
        pd_km2  = pd;
        f_d_km1 = zeros(3, 1);
        f_d_km2 = zeros(3, 1);
        a_hat_km1 = a_hat_init;            a_hat_km2 = a_hat_init;
        a_ctrl_km1 = a_hat_init;           a_ctrl_km2 = a_hat_init;
        var_da_inc_km1 = var_da_init_vec;  var_da_inc_km2 = var_da_init_vec;

        % --- 0K. Misc ---
        k_step = 1;
        R_OFF  = 1e10;

        % --- 0L. First call returns zeros (no f_d yet) ---
        f_d = zeros(3, 1);
        ekf_out = [a_hat_init(1); a_hat_init(3); a_hat_init(2); 0];
        if nargout >= 3
            diag = empty_diag_para();
            diag.f_d            = f_d;
            diag.a_hat          = a_hat_init;
            diag.a_ctrl_used    = a_hat_init;
            diag.theta_hat      = theta_init;
            diag.sigma2_dxr_hat = sigma2_dxr_hat;
        end
        return;
    end

    % ------------------------------------------------------------------
    % Per-step: extract per-axis theta and implied gain
    % ------------------------------------------------------------------
    theta_hat = x_e_per_axis(4, :).';     % 3x1 wall-shape constant

    % delta_x_m[k] = p_d[k-d] - p_m[k]
    if d_delay == 2
        pd_km_d = pd_km2;
    elseif d_delay == 1
        pd_km_d = pd_km1;
    else
        error('motion_control_law_eq17_4state_para:unsupportedDelay', ...
              'Only d=1 or d=2 supported, got d=%g.', d_delay);
    end
    delta_x_m = pd_km_d - p_m;           % 3x1 [um]
    pd_kp1    = pd + del_pd;             % 3x1 [um]

    % h_bar from current (delayed) measurement
    if enable_wall
        h_bar = (dot(p_m, w_hat_n) - pz_wall) / R_radius;
    else
        h_bar = Inf;
    end
    one_minus_lc = 1 - lambda_c;

    % Implied gain a(h_bar; theta_hat), Jacobian da/dtheta, per axis
    a_gain = zeros(3, 1);
    dadth  = zeros(3, 1);
    for ax = 1:3
        [a_gain(ax), ~, dadth(ax)] = para_gain_local(h_bar, theta_hat(ax), ax == 3, a_nom_p);
    end
    a_ctrl = a_gain;                      % control gain = implied gain (no override)

    % ------------------------------------------------------------------
    % Wall functions at measured h_bar (Q33/R22 random part; avoids bias loop)
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

    % ------------------------------------------------------------------
    % [1] IIR a_xm (paper 2025 Eq.9-13) -- verbatim from base 4-state
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
        if warmup_count == 1
            for ax = 1:3
                x_e_per_axis(1, ax) = dx_bar_m_new(ax);
                x_e_per_axis(2, ax) = dx_bar_m_new(ax);
                x_e_per_axis(3, ax) = dx_bar_m_new(ax);
            end
        end
        dx_bar_m       = dx_bar_m_new;
        sigma2_dxr_hat = sigma2_dxr_hat_new;
        pd_km2 = pd_km1; pd_km1 = pd;
        f_d_km2 = f_d_km1; f_d_km1 = f_d;
        a_hat_km2 = a_hat_km1; a_hat_km1 = a_gain;
        a_ctrl_km2 = a_ctrl_km1; a_ctrl_km1 = a_ctrl;
        warmup_count = warmup_count - 1;
        k_step = k_step + 1;
        h_bar_now = local_h_bar_out(enable_wall, h_bar);
        ekf_out = [a_gain(1); a_gain(3); a_gain(2); h_bar_now];
        if nargout >= 3
            diag = empty_diag_para();
            diag.f_d = f_d; diag.a_hat = a_gain; diag.a_ctrl_used = a_ctrl;
            diag.theta_hat = theta_hat;
            diag.sigma2_dxr_hat = sigma2_dxr_hat_new; diag.a_xm = a_xm;
            diag.delta_x_m = delta_x_m; diag.h_bar = h_bar; diag.dx_r = dx_r;
        end
        return;
    end

    % ------------------------------------------------------------------
    % [3] Control law (Vpersonal B2 ACTIVE eq17, NO -x_hat_D term)
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
    % [4] Q (4x4 diag) and R (2x2) per axis -- Q33/R22 verbatim from base
    % ------------------------------------------------------------------
    Q_per_axis = cell(3, 1);
    R_per_axis = cell(3, 1);
    gate_off   = false(3, 1);
    G_flags    = false(3, 3);
    var_da_inc = zeros(3, 1);   % true increment var Var(delta_a_ram), for Q33/R22
    t_now = (k_step - 1) * Ts;
    for ax = 1:3
        a_i = a_gain(ax);
        var_da_inc(ax) = var_da_inc_factor * (a_i * K_h_axis(ax) / R_radius)^2 * sigma2_dh;

        if d_delay == 2
            Q33_thermal  = 4 * kBT * (a_i + one_minus_lc^2 * (a_hat_km1(ax) + a_hat_km2(ax)));
            Q33_randgain = one_minus_lc^2 * ( 4 * f_d_km1(ax)^2 * var_da_inc_km1(ax) ...
                                            + 1 * f_d_km2(ax)^2 * var_da_inc_km2(ax) );
        else
            Q33_thermal  = 4 * kBT * (a_i + one_minus_lc^2 * a_hat_km1(ax));
            Q33_randgain = one_minus_lc^2 * (f_d_km1(ax)^2 * var_da_inc_km1(ax));
        end
        Q33_nx = one_minus_lc^2 * sigma2_n_s(ax);

        Q_i = zeros(4);
        Q_i(3, 3) = Q33_thermal + Q33_randgain + Q33_nx;
        Q_i(4, 4) = 0;                                  % Q_theta = 0 (constant)
        Q_per_axis{ax} = Q_i;

        R11_i = sigma2_n_s(ax);
        IF_eff_i = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a_i, sigma2_n_s(ax));
        R2_intrinsic_i = amlpf_var_factor * K_var * IF_eff_i * (a_i + xi_per_axis(ax))^2;
        if d_delay == 2
            R22_delay_i = var_da_inc_km1(ax) + var_da_inc_km2(ax);
        else
            R22_delay_i = var_da_inc_km1(ax);
        end
        R2_eff_i = R2_intrinsic_i + R22_delay_i;

        G1 = (t_now < t_warmup_kf);
        G2 = ((sigma2_dxr_hat_new(ax) - C_n * sigma2_n_s(ax)) <= 0);
        G3 = (h_bar < h_bar_safe);
        G_flags(:, ax) = [G1; G2; G3];
        gate_off(ax) = G1 || G2 || G3;

        if gate_off(ax)
            R22_i = R_OFF;
        else
            R22_i = R2_eff_i;
        end
        R_ax = zeros(2);
        R_ax(1, 1) = R11_i;
        R_ax(2, 2) = R22_i;
        R_per_axis{ax} = R_ax;
    end

    % ------------------------------------------------------------------
    % [5] EKF predict + update per axis (NONLINEAR y2 in theta)
    % ------------------------------------------------------------------
    H_y1 = [1 0 0 0];

    K_th_y2_v  = zeros(3, 1);
    K_dx_y1_v  = zeros(3, 1);
    innov_y2_v = zeros(3, 1);

    for ax = 1:3
        % F_e (Row 3 time-varying via f_d history)
        if d_delay == 2
            F_1_i = f_d_km1(ax) + f_d_km2(ax);
        else
            F_1_i = f_d_km1(ax);
        end
        F_dx = f_d(ax) + one_minus_lc * F_1_i;
        Fe3_th = -F_dx * dadth(ax);          % col 4 (theta) coupling
        F_e = [0 1 0        0; ...
               0 0 1        0; ...
               0 0 lambda_c Fe3_th; ...
               0 0 0        1];

        x_curr = x_e_per_axis(:, ax);
        P_curr = P_per_axis{ax};

        % --- Predict (theta constant) ---
        x_pred = [x_curr(2); ...
                  x_curr(3); ...
                  lambda_c * x_curr(3); ...
                  x_curr(4)];
        P_pred = F_e * P_curr * F_e' + Q_per_axis{ax};
        P_pred = 0.5 * (P_pred + P_pred');

        % --- Measurement: y2 = a(h_bar; theta_pred) nonlinear (theta unchanged
        %     in predict so a_pred == a_gain(ax), Jacobian == dadth(ax)) ---
        if gate_off(ax)
            H_use = H_y1;
            y_use = delta_x_m(ax);
            y_pred = x_pred(1);
            R_use = sigma2_n_s(ax);
        else
            H_use = [1 0 0 0; 0 0 0 dadth(ax)];
            y_use = [delta_x_m(ax); a_xm(ax)];
            y_pred = [x_pred(1); a_gain(ax)];
            R_use = R_per_axis{ax};
        end

        innov  = y_use - y_pred;
        S_inn  = H_use * P_pred * H_use' + R_use;
        S_inn  = 0.5 * (S_inn + S_inn');
        K_kf   = (P_pred * H_use') / S_inn;

        % Warmup gate: freeze theta state during G1 (slot 4)
        if G_flags(1, ax)
            K_kf(4, :) = 0;
        end

        x_post = x_pred + K_kf * innov;
        ImKH   = eye(4) - K_kf * H_use;
        P_post = ImKH * P_pred * ImKH' + K_kf * R_use * K_kf';   % Joseph form
        P_post = 0.5 * (P_post + P_post');

        % Diagnostics
        K_dx_y1_v(ax) = K_kf(3, 1);
        if gate_off(ax)
            K_th_y2_v(ax)  = 0;
            innov_y2_v(ax) = 0;
        else
            K_th_y2_v(ax)  = K_kf(4, 2);
            innov_y2_v(ax) = innov(2);
        end

        x_e_per_axis(:, ax) = x_post;
        P_per_axis{ax} = P_post;
    end

    % ------------------------------------------------------------------
    % [6] Bookkeeping
    % ------------------------------------------------------------------
    pd_km2 = pd_km1; pd_km1 = pd;
    f_d_km2 = f_d_km1; f_d_km1 = f_d;
    a_hat_km2 = a_hat_km1; a_hat_km1 = a_gain;
    a_ctrl_km2 = a_ctrl_km1; a_ctrl_km1 = a_ctrl;
    var_da_inc_km2 = var_da_inc_km1; var_da_inc_km1 = var_da_inc;
    dx_bar_m = dx_bar_m_new;
    sigma2_dxr_hat = sigma2_dxr_hat_new;
    k_step = k_step + 1;

    % ------------------------------------------------------------------
    % [7] Output
    % ------------------------------------------------------------------
    theta_post = x_e_per_axis(4, :).';
    a_hat_post = zeros(3, 1);             % implied gain at posterior theta
    for ax = 1:3
        a_hat_post(ax) = para_gain_local(h_bar, theta_post(ax), ax == 3, a_nom_p);
    end
    h_bar_now = local_h_bar_out(enable_wall, h_bar);
    ekf_out = [a_hat_post(1); a_hat_post(3); a_hat_post(2); h_bar_now];

    if nargout >= 3
        P_th_v = zeros(3, 1); P_dx_v = zeros(3, 1); P_dx1_v = zeros(3, 1);
        for ax = 1:3
            P_th_v(ax)  = P_per_axis{ax}(4, 4);
            P_dx_v(ax)  = P_per_axis{ax}(3, 3);
            P_dx1_v(ax) = P_per_axis{ax}(1, 1);
        end
        diag = empty_diag_para();
        diag.sigma2_dxr_hat = sigma2_dxr_hat_new;
        diag.a_xm           = a_xm;
        diag.delta_x_m      = delta_x_m;
        diag.innovation_y2  = innov_y2_v;
        diag.K_kf_theta_y2  = K_th_y2_v;
        diag.K_kf_dx_y1     = K_dx_y1_v;
        diag.P_theta        = P_th_v;
        diag.P_dx           = P_dx_v;
        diag.P_dx1          = P_dx1_v;
        diag.theta_hat      = theta_post;
        diag.a_hat          = a_hat_post;
        diag.a_ctrl_used    = a_ctrl;
        diag.gate_active_per_axis = gate_off;
        diag.guards_individual    = G_flags;
        diag.h_bar          = h_bar;
        diag.f_d            = f_d;
        diag.dx_r           = dx_r;
        diag.delta_x_hat_1  = x_e_per_axis(1, :).';
        diag.delta_x_hat_3  = x_e_per_axis(3, :).';
        diag.var_da_inc     = var_da_inc;
    end
end


%% =================== Local Helpers ===================

function [a, c, dadth] = para_gain_local(h_bar, theta, is_z, a_nom)
%PARA_GAIN_LOCAL  Parametric gain a(h_bar; theta) and dtheta-Jacobian (per axis).
%   z   -> phi = 1/(h_bar-1)  (pole basis, clamp h_bar-1 >= 0.05)
%   x,y -> phi = 1/h_bar       (clamp h_bar >= 1.001)
%   c = 1 + theta*phi ; a = a_nom/c ; da/dtheta = -(a/c)*phi.
%   c is floored at 0.05 to keep the control gain finite if theta goes negative.
    if is_z
        hh = max(h_bar - 1, 0.05);
    else
        hh = max(h_bar, 1.001);
    end
    phi = 1 / hh;                 % -> 0 when h_bar = Inf (wall disabled)
    c   = 1 + theta * phi;
    c   = max(c, 0.05);
    a   = a_nom / c;
    dadth = -(a / c) * phi;
end


function IF = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a, sigma2_nx)
%IF_EFF_EVAL  Exact color-inflation factor IF_eff for R22 (R22_derivation S4-S6).
    sxT = 4 * kBT * a;
    num = sxT^2 * IF_abc(1) + 2 * sxT * sigma2_nx * IF_abc(2) + sigma2_nx^2 * IF_abc(3);
    den = (C_dpmr * sxT + C_n * sigma2_nx)^2;
    IF  = 1 + 2 * num / den;
end


function P_post = solve_dare_kf_local(F, H, Q, R)
%SOLVE_DARE_KF_LOCAL  Discrete-time KF Riccati steady-state (fixed-point).
    n = size(F, 1);
    P_post = eye(n);
    max_iter = 10000;
    tol = 1e-13;
    for k = 1:max_iter
        P_pred = F * P_post * F' + Q;
        P_pred = 0.5 * (P_pred + P_pred');
        S = H * P_pred * H' + R;
        K = (P_pred * H') / S;
        P_new = (eye(n) - K * H) * P_pred;
        P_new = 0.5 * (P_new + P_new');
        if max(abs(P_new(:) - P_post(:))) < tol
            P_post = P_new;
            return;
        end
        P_post = P_new;
    end
end


function v = get_const(s, name, default)
%GET_CONST  Return s.(name) if present and non-empty, else default.
    if isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end


function h = local_h_bar_out(enable_wall, h_bar)
    if enable_wall
        h = h_bar;
    else
        h = 0;
    end
end


function d = empty_diag_para()
%EMPTY_DIAG_PARA  Zeroed diagnostic struct for the integrated para controller.
    d = struct();
    d.sigma2_dxr_hat    = zeros(3, 1);
    d.a_xm              = zeros(3, 1);
    d.delta_x_m         = zeros(3, 1);
    d.innovation_y2     = zeros(3, 1);
    d.K_kf_theta_y2     = zeros(3, 1);
    d.K_kf_dx_y1        = zeros(3, 1);
    d.P_theta           = zeros(3, 1);
    d.P_dx              = zeros(3, 1);
    d.P_dx1             = zeros(3, 1);
    d.theta_hat         = zeros(3, 1);
    d.a_hat             = zeros(3, 1);
    d.a_ctrl_used       = zeros(3, 1);
    d.gate_active_per_axis = false(3, 1);
    d.guards_individual    = false(3, 3);
    d.h_bar             = 0;
    d.f_d               = zeros(3, 1);
    d.dx_r              = zeros(3, 1);
    d.delta_x_hat_1     = zeros(3, 1);
    d.delta_x_hat_3     = zeros(3, 1);
    d.var_da_inc        = zeros(3, 1);
end
