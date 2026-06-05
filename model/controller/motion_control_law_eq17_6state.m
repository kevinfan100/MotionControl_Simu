function [f_d, ekf_out, diag] = motion_control_law_eq17_6state(del_pd, pd, p_m, params, ctrl_const)
%MOTION_CONTROL_LAW_EQ17_6STATE Per-axis 6-state EKF controller
%   (RevisedControl_Vpersonal: paper 2023 Eq.17, disturbance pre-combined).
%
%   [f_d, ekf_out]       = motion_control_law_eq17_6state(del_pd, pd, p_m, params, ctrl_const)
%   [f_d, ekf_out, diag] = motion_control_law_eq17_6state(...)
%
%   Sibling of motion_control_law_eq17_core (7-state). Implements the
%   RevisedControl_Vpersonal 6-state architecture: the disturbance is
%   pre-combined into a single state delta_x_D^d, so the augmented state is
%
%       x = [delta_x_1; delta_x_2; delta_x_3; delta_x_D^d; a_x; delta_a_x]
%
%       delta_x_1   = delta_x[k-2]   (oldest; matches y_1 direct measurement)
%       delta_x_2   = delta_x[k-1]
%       delta_x_3   = delta_x[k]     (current)
%       delta_x_D^d = combined disturbance (delta_x_D[k] + (1-lc) sum delta_x_D[k-i])
%       a_x         = motion gain    [um/pN]
%       delta_a_x   = a_x rate
%
%   Measurements (per axis):
%       y_1 = delta_x_m = p_d[k-2] - p_m[k]    (delayed tracking error)
%       y_2 = a_xm      = (sigma2_dxr_hat - C_n*sigma2_n_s) / (C_dpmr*4kBT)
%
%   BUILD STAGE: B1 (skeleton). Uses the PASSIVE control law and a
%   production-mirrored Q (Q33 = 4kBT*a_hat thermal only). The structural
%   pieces (6-state vector, F_e 6x6, H 2x6, deterministic-map predict,
%   3-guard update, Joseph form, prefill+Riccati init) are final. Stage B2
%   switches the control law to the active (a_hat-weighted) form and Q33 to
%   the full per-step Var(epsilon). S cross-covariance (Stage B3) is 0 here.
%
%   F_e Row 3 (6-state, Vpersonal p.5):
%       [0, 0, lc, -1, -F_dx, dF_dx]
%       F_dx  = f_d[k] + (1-lc)*sum_{i=1..d} f_d[k-i]
%       dF_dx = (1-lc)*sum_{i=1..d} i*f_d[k-i]
%
%   Inputs / outputs match the 7-state core (drop-in for run_pure_simulation
%   dispatch). ctrl_const from build_eq17_6state_constants.
%
%   See also: motion_control_law_eq17_core, build_eq17_6state_constants,
%             calc_correction_functions

    % ------------------------------------------------------------------
    % Open-loop bypass
    % ------------------------------------------------------------------
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = [1; 1; 1; 0];
        if nargout >= 3
            diag = empty_diag_6state();
            diag.f_d = f_d;
        end
        return;
    end

    % ------------------------------------------------------------------
    % Persistent state
    % ------------------------------------------------------------------
    persistent x_e_per_axis        % 6x3 EKF state (col = axis)
    persistent P_per_axis          % cell{3} of 6x6 covariance
    persistent dx_bar_m            % 3x1 IIR LP mean of delta_x_m [um]
    persistent sigma2_dxr_hat      % 3x1 EWMA variance of dx_r    [um^2]
    persistent pd_km1 pd_km2       % trajectory delay buffers
    persistent f_d_km1 f_d_km2     % past control buffers (for Sigma f_d[k-i])
    persistent warmup_count k_step

    persistent initialized
    persistent lambda_c d_delay Ts kBT R_radius gamma_N_p
    persistent a_pd a_cov C_dpmr C_n K_var IF_eff_per_axis xi_per_axis
    persistent t_warmup_kf h_bar_safe R_OFF
    persistent sigma2_n_s a_x_init enable_wall w_hat_n pz_wall

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

        % --- 0B. ctrl_const (offline scalars) ---
        lambda_c        = ctrl_const.lambda_c;
        d_delay         = ctrl_const.d;
        C_dpmr          = ctrl_const.C_dpmr;
        C_n             = ctrl_const.C_n;
        K_var           = ctrl_const.K_var;
        IF_eff_per_axis = ctrl_const.IF_eff_per_axis(:);
        xi_per_axis     = ctrl_const.xi_per_axis(:);
        t_warmup_kf     = ctrl_const.t_warmup_kf;
        h_bar_safe      = ctrl_const.h_bar_safe;
        a_cov           = ctrl_const.a_cov;
        a_pd            = ctrl_const.a_pd;

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

        % --- 0E. Wall-aware a_x[0] seeding ---
        a_nom = Ts / gamma_N_p;
        if enable_wall && isfield(params, 'common') && isfield(params.common, 'p0')
            p0_init    = params.common.p0(:);
            h_init_um  = dot(p0_init, w_hat_n) - pz_wall;
            h_bar_init = max(h_init_um / R_radius, 1.001);
            [c_para0, c_perp0, derivs0] = calc_correction_functions(h_bar_init, true);
            a_x_init   = [a_nom / c_para0; a_nom / c_para0; a_nom / c_perp0];
            K_h_init   = [derivs0.K_h_para; derivs0.K_h_para; derivs0.K_h_perp];
            a_perp_init = a_nom / c_perp0;
        else
            a_x_init    = [a_nom; a_nom; a_nom];
            K_h_init    = zeros(3, 1);
            a_perp_init = a_nom;
        end

        % --- 0F. EKF state init (a_x seeded; rest zero) ---
        x_e_per_axis = zeros(6, 3);
        x_e_per_axis(5, :) = a_x_init.';        % slot 5 = a_x

        % --- 0G. Riccati Pf (DARE steady state at h_init, positioning f_d=0) ---
        F_e_ss = build_F_e_6state(lambda_c, 0, 0, 0);
        H_ss   = [1 0 0 0 0 0; 0 0 0 0 1 -d_delay];
        sigma2_dh_init = 4 * kBT * a_perp_init;            % wall-normal thermal motion var
        P_per_axis = cell(3, 1);
        for ax = 1:3
            a_init_ax   = a_x_init(ax);
            var_da_init = (a_init_ax * K_h_init(ax) / R_radius)^2 * sigma2_dh_init;
            Q_ss = zeros(6);
            Q_ss(3, 3) = 4 * kBT * a_init_ax;              % Q33 thermal
            Q_ss(5, 5) = var_da_init;                      % Q55 = var(delta_a_ram)
            R22_ss = K_var * IF_eff_per_axis(ax) * (a_init_ax + xi_per_axis(ax))^2 ...
                     + d_delay * var_da_init;
            R_ss = [sigma2_n_s(ax), 0; 0, R22_ss];
            P_per_axis{ax} = solve_dare_kf_local(F_e_ss, H_ss, Q_ss, R_ss);
        end

        % --- 0H. IIR states (prefill default: seed sigma2_dxr to steady state) ---
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

        % --- 0I. Delay buffers ---
        pd_km1  = pd;
        pd_km2  = pd;
        f_d_km1 = zeros(3, 1);
        f_d_km2 = zeros(3, 1);

        % --- 0K. Misc ---
        k_step = 1;
        R_OFF  = 1e10;

        % --- 0L. First call returns zeros (no f_d yet) ---
        f_d = zeros(3, 1);
        ekf_out = [a_x_init(1); a_x_init(3); a_x_init(2); 0];
        if nargout >= 3
            diag = empty_diag_6state();
            diag.f_d           = f_d;
            diag.a_hat         = a_x_init;
            diag.sigma2_dxr_hat = sigma2_dxr_hat;
        end
        return;
    end

    % ------------------------------------------------------------------
    % Per-step: extract per-axis state
    % ------------------------------------------------------------------
    a_hat    = x_e_per_axis(5, :).';     % 3x1 [um/pN]
    xD_comb  = x_e_per_axis(4, :).';     % 3x1 [um]  (delta_x_D^d)

    % delta_x_m[k] = p_d[k-d] - p_m[k]
    if d_delay == 2
        pd_km_d = pd_km2;
    elseif d_delay == 1
        pd_km_d = pd_km1;
    else
        error('motion_control_law_eq17_6state:unsupportedDelay', ...
              'Only d=1 or d=2 supported, got d=%g.', d_delay);
    end
    delta_x_m = pd_km_d - p_m;           % 3x1 [um]
    pd_kp1    = pd + del_pd;             % 3x1 [um]

    % h_bar from current measurement (Guard 3 + wall functions)
    if enable_wall
        h_bar = (dot(p_m, w_hat_n) - pz_wall) / R_radius;
    else
        h_bar = Inf;
    end

    one_minus_lc = 1 - lambda_c;

    % ------------------------------------------------------------------
    % Wall functions at measured h_bar (deterministic; avoids bias loop)
    % ------------------------------------------------------------------
    if enable_wall && isfinite(h_bar) && h_bar > 1
        [~, c_perp_h, derivs] = calc_correction_functions(h_bar, true);
        K_h_axis = [derivs.K_h_para; derivs.K_h_para; derivs.K_h_perp];
        a_perp_meas = Ts / (gamma_N_p * c_perp_h);
    else
        K_h_axis = zeros(3, 1);
        a_perp_meas = Ts / gamma_N_p;
    end
    sigma2_dh = 4 * kBT * a_perp_meas;   % wall-normal thermal motion variance (shared 3 axes)

    % ------------------------------------------------------------------
    % [1] IIR a_xm (paper 2025 Eq.9-13)
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
        warmup_count = warmup_count - 1;
        k_step = k_step + 1;
        a_hat_post = x_e_per_axis(5, :).';
        h_bar_now = local_h_bar_out(enable_wall, h_bar);
        ekf_out = [a_hat_post(1); a_hat_post(3); a_hat_post(2); h_bar_now];
        if nargout >= 3
            diag = empty_diag_6state();
            diag.f_d = f_d; diag.a_hat = a_hat_post;
            diag.sigma2_dxr_hat = sigma2_dxr_hat_new; diag.a_xm = a_xm;
            diag.delta_x_m = delta_x_m; diag.h_bar = h_bar; diag.dx_r = dx_r;
        end
        return;
    end

    % ------------------------------------------------------------------
    % [3] Control law (B1 PASSIVE form; B2 -> active a_hat-weighted)
    %   f_d = a_hat^-1 {pd[k+1] - lc*pd[k] - (1-lc)*pd[k-d] + (1-lc)*dx_m}
    %         - (1-lc)*sum f_d[k-i]  - delta_x_D^d / a_hat
    % ------------------------------------------------------------------
    if d_delay == 2
        sum_fd_past = f_d_km1 + f_d_km2;
    else
        sum_fd_past = f_d_km1;
    end
    inv_a_hat = 1 ./ a_hat;
    f_d = inv_a_hat .* (pd_kp1 - lambda_c * pd - one_minus_lc * pd_km_d ...
                        + one_minus_lc * delta_x_m) ...
          - one_minus_lc * sum_fd_past ...
          - xD_comb .* inv_a_hat;

    % ------------------------------------------------------------------
    % [4] Q (6x6 diagonal) and R (2x2) per axis
    %   B1: Q33 = 4kBT*a_hat (thermal only, production mirror)
    %       Q55 = var(delta_a_ram) = (a_hat*K_h/R)^2 * sigma2_dh   (OL)
    %       Q44 = Q66 = 0
    % ------------------------------------------------------------------
    Q_per_axis = cell(3, 1);
    R_per_axis = cell(3, 1);
    gate_off   = false(3, 1);
    G_flags    = false(3, 3);
    var_da_ram = zeros(3, 1);
    t_now = (k_step - 1) * Ts;
    for ax = 1:3
        a_hat_i = a_hat(ax);
        var_da_ram(ax) = (a_hat_i * K_h_axis(ax) / R_radius)^2 * sigma2_dh;

        Q_i = zeros(6);
        Q_i(3, 3) = 4 * kBT * a_hat_i;
        Q_i(5, 5) = var_da_ram(ax);
        Q_per_axis{ax} = Q_i;

        R11_i = sigma2_n_s(ax);
        R2_intrinsic_i = K_var * IF_eff_per_axis(ax) * (a_hat_i + xi_per_axis(ax))^2;
        R2_eff_i = R2_intrinsic_i + d_delay * var_da_ram(ax);   % delay term (B1 simple d*var)

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
    % [5] EKF predict + update per axis
    % ------------------------------------------------------------------
    H_full = [1 0 0 0 0 0; 0 0 0 0 1 -d_delay];
    H_y1   = H_full(1, :);

    K_a_y2_v  = zeros(3, 1);
    K_dx_y1_v = zeros(3, 1);
    innov_y2_v = zeros(3, 1);

    for ax = 1:3
        % F_e (time-varying Row 3 via f_d history)
        if d_delay == 2
            F_1_i = f_d_km1(ax) + f_d_km2(ax);
            F_2_i = f_d_km1(ax) + 2 * f_d_km2(ax);
        else
            F_1_i = f_d_km1(ax);
            F_2_i = f_d_km1(ax);
        end
        F_e = build_F_e_6state(lambda_c, f_d(ax), F_1_i, F_2_i);

        x_curr = x_e_per_axis(:, ax);
        P_curr = P_per_axis{ax};

        % --- Predict: state via deterministic map (cross-terms are
        %     estimation errors -> E[.]=0); covariance via full F_e ---
        x_pred = [x_curr(2); ...
                  x_curr(3); ...
                  lambda_c * x_curr(3); ...
                  x_curr(4); ...
                  x_curr(5) + x_curr(6); ...
                  x_curr(6)];
        P_pred = F_e * P_curr * F_e' + Q_per_axis{ax};
        P_pred = 0.5 * (P_pred + P_pred');

        % --- Update (1D if y_2 gated, else 2D) ---
        if gate_off(ax)
            H_use = H_y1;
            y_use = delta_x_m(ax);
            R_use = sigma2_n_s(ax);
        else
            H_use = H_full;
            y_use = [delta_x_m(ax); a_xm(ax)];
            R_use = R_per_axis{ax};
        end

        y_pred = H_use * x_pred;
        innov  = y_use - y_pred;
        S_inn  = H_use * P_pred * H_use' + R_use;
        S_inn  = 0.5 * (S_inn + S_inn');
        K_kf   = (P_pred * H_use') / S_inn;

        % Warmup gate: freeze gain states during G1 (slots 5,6)
        if G_flags(1, ax)
            K_kf(5, :) = 0;
            K_kf(6, :) = 0;
        end

        x_post = x_pred + K_kf * innov;
        ImKH   = eye(6) - K_kf * H_use;
        P_post = ImKH * P_pred * ImKH' + K_kf * R_use * K_kf';   % Joseph form
        P_post = 0.5 * (P_post + P_post');

        % Diagnostics
        K_dx_y1_v(ax) = K_kf(3, 1);
        if gate_off(ax)
            K_a_y2_v(ax)  = 0;
            innov_y2_v(ax) = 0;
        else
            K_a_y2_v(ax)  = K_kf(5, 2);
            innov_y2_v(ax) = innov(2);
        end

        x_e_per_axis(:, ax) = x_post;
        P_per_axis{ax} = P_post;
    end

    % ------------------------------------------------------------------
    % [6] Bookkeeping: shift delay buffers, IIR states, step counter
    % ------------------------------------------------------------------
    pd_km2 = pd_km1; pd_km1 = pd;
    f_d_km2 = f_d_km1; f_d_km1 = f_d;
    dx_bar_m = dx_bar_m_new;
    sigma2_dxr_hat = sigma2_dxr_hat_new;
    k_step = k_step + 1;

    % ------------------------------------------------------------------
    % [7] Output
    % ------------------------------------------------------------------
    a_hat_post = x_e_per_axis(5, :).';
    h_bar_now = local_h_bar_out(enable_wall, h_bar);
    ekf_out = [a_hat_post(1); a_hat_post(3); a_hat_post(2); h_bar_now];

    if nargout >= 3
        P_a_v = zeros(3, 1); P_dx_v = zeros(3, 1); P_dx1_v = zeros(3, 1);
        for ax = 1:3
            P_a_v(ax)   = P_per_axis{ax}(5, 5);
            P_dx_v(ax)  = P_per_axis{ax}(3, 3);
            P_dx1_v(ax) = P_per_axis{ax}(1, 1);
        end
        diag = empty_diag_6state();
        diag.sigma2_dxr_hat = sigma2_dxr_hat_new;
        diag.a_xm           = a_xm;
        diag.delta_x_m      = delta_x_m;
        diag.innovation_y2  = innov_y2_v;
        diag.K_kf_a_y2      = K_a_y2_v;
        diag.K_kf_dx_y1     = K_dx_y1_v;
        diag.P_a            = P_a_v;
        diag.P_dx           = P_dx_v;
        diag.x_D_hat              = x_e_per_axis(4, :).';   % delta_x_D^d (slot 4)
        diag.delta_a_hat          = x_e_per_axis(6, :).';
        diag.gate_active_per_axis = gate_off;
        diag.guards_individual    = G_flags;
        diag.h_bar                = h_bar;
        diag.f_d                  = f_d;
        diag.dx_r                 = dx_r;
        diag.a_hat                = a_hat_post;
        diag.P77                  = zeros(3, 1);            % no slot 7 (driver compat)
        diag.Q77                  = zeros(3, 1);
        diag.var_da_ram           = var_da_ram;
        diag.delta_x_hat_1        = x_e_per_axis(1, :).';
        diag.P_dx1                = P_dx1_v;
    end
end


%% =================== Local Helpers ===================

function F_e = build_F_e_6state(lambda_c, f_d_i, F_1_i, F_2_i)
%BUILD_F_E_6STATE  6x6 error-dynamics matrix (per axis), Vpersonal p.5.
%   Row 3 = [0 0 lc -1 -F_dx dF_dx]
%       F_dx  = f_d[k] + (1-lc)*F_1   (col 5, a_x)
%       dF_dx = (1-lc)*F_2            (col 6, delta_a_x)
%   F_1 = sum_{i=1..d} f_d[k-i], F_2 = sum_{i=1..d} i*f_d[k-i].
%   (d-step dependence is carried entirely by F_1/F_2; the col-4 entry -1 is
%    d-independent in the combined-disturbance form.)
    one_minus_lc = 1 - lambda_c;
    Fe3_a  = -f_d_i - one_minus_lc * F_1_i;     % col 5 (a_x): -F_dx
    Fe3_da = one_minus_lc * F_2_i;              % col 6 (delta_a_x): dF_dx
    F_e = [0 1 0        0  0      0; ...
           0 0 1        0  0      0; ...
           0 0 lambda_c -1 Fe3_a  Fe3_da; ...
           0 0 0        1  0      0; ...
           0 0 0        0  1      1; ...
           0 0 0        0  0      1];
end


function P_post = solve_dare_kf_local(F, H, Q, R)
%SOLVE_DARE_KF_LOCAL  Discrete-time KF Riccati steady-state (fixed-point).
%   No Control System Toolbox dependency.
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


function h = local_h_bar_out(enable_wall, h_bar)
    if enable_wall
        h = h_bar;
    else
        h = 0;
    end
end


function d = empty_diag_6state()
%EMPTY_DIAG_6STATE  Zeroed diagnostic struct for 6-state controller.
    d = struct();
    d.sigma2_dxr_hat    = zeros(3, 1);
    d.a_xm              = zeros(3, 1);
    d.delta_x_m         = zeros(3, 1);
    d.innovation_y2     = zeros(3, 1);
    d.K_kf_a_y2         = zeros(3, 1);
    d.K_kf_dx_y1        = zeros(3, 1);
    d.P_a               = zeros(3, 1);
    d.P_dx              = zeros(3, 1);
    % Driver-compatible field names (run_pure_simulation diag_log) + 6-state extras
    d.x_D_hat              = zeros(3, 1);   % = delta_x_D^d estimate (slot 4)
    d.delta_a_hat          = zeros(3, 1);
    d.gate_active_per_axis = false(3, 1);
    d.guards_individual    = false(3, 3);
    d.h_bar                = 0;
    d.f_d                  = zeros(3, 1);
    d.dx_r                 = zeros(3, 1);
    d.a_hat                = zeros(3, 1);
    d.P77                  = zeros(3, 1);   % no slot 7 in 6-state (placeholder, driver compat)
    d.Q77                  = zeros(3, 1);
    d.var_da_ram           = zeros(3, 1);   % 6-state extra: Q55 = var(delta_a_ram)
    d.delta_x_hat_1        = zeros(3, 1);
    d.P_dx1                = zeros(3, 1);
end
