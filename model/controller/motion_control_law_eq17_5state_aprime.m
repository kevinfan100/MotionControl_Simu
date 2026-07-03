function [f_d, ekf_out, diag] = motion_control_law_eq17_5state_aprime(del_pd, pd, p_m, params, ctrl_const, a_ctrl_override)
%MOTION_CONTROL_LAW_EQ17_5STATE_APRIME Per-axis 5-state EKF that ESTIMATES the
%   gain slope a'_x (general / unknown wall effect). See
%   reference/eq17_analysis/derivation/5state_est_aprime.tex.
%
%   [f_d, ekf_out]       = motion_control_law_eq17_5state_aprime(del_pd, pd, p_m, params, ctrl_const)
%   [f_d, ekf_out, diag] = motion_control_law_eq17_5state_aprime(...)
%   [f_d, ekf_out, diag] = motion_control_law_eq17_5state_aprime(..., ctrl_const, a_ctrl_override)
%
%   Built on motion_control_law_eq17_5state. Slot 5 is REINTERPRETED: instead of
%   the free gain RATE delta_a_x, it is the gain SLOPE a'_x = da_x/dh (a property
%   of the unknown wall), modelled as a random walk a'_x[k+1] = a'_x[k] + w_a'.
%   The per-axis state is:
%
%       x = [delta_x_1; delta_x_2; delta_x_3; a_x; a'_x]
%
%   The gain advances by the slope times the KNOWN desired height increment:
%
%       a_x[k+1] = a_x[k] + a'_x[k]*Delta_h_d[k] + delta_a_ram[k]
%       Delta_h_d[k] = h_d[k+1]-h_d[k] = dot(del_pd, w_hat)   (known, scalar)
%
%   so the slope multiplying Delta_h_d is LEARNED from a_xm rather than looked up
%   from c(h_bar): the controller no longer needs the wall model. a'_x couples to
%   the rest of the state only through the height motion, so it is observable only
%   while the desired trajectory moves in h (Delta_h_d != 0).
%
%   Differences vs the existing 5-state (rate version), all in slot 5:
%       predict  : a_x += a'_x * Delta_h_d        (was a_x += delta_a_x)
%       F_e(4,5) : Delta_h_d[k]                    (was 1)
%       F_e(3,5) : dF_dx^h = (1-lc)*sum (h_d[k]-h_d[k-i]) f_d[k-i]  (was (1-lc)*sum i*f_d[k-i])
%       H(2,5)   : -Delta_H_d[k] = -(h_d[k]-h_d[k-d])   (was -d)   [TIME-VARYING H]
%       Q(5,5)   : Var(w_a') baseline (was 0)
%
%   ctrl_const.use_selfmod (optional, default false; see
%   reference/eq17_analysis/derivation/5state_aprime_unified.tex Sec.3-5,7):
%       false (baseline, unchanged): F_e(4,5)=Delta_h_d, H(2,5)=-Delta_H_d;
%           a'_x is structurally UNOBSERVABLE on a height hold (Delta_h_d=0).
%           Q(5,5) uses the wall-peeking a''=(a/R^2)(K_h^2-K_h') formula.
%       true: adds the thermal self-dither coupling, restoring hold-
%           observability (rank(O_N) 4->5):
%               F_e(4,5) = Delta_h_d + (1-lc)*delta_x_hat_3
%               H(2,5)   = -Delta_H_d + (delta_x_hat_3-delta_x_hat_1)
%           and replaces Q(5,5) with the honest dimensional-anchor formula
%           sigma_a''~a_nom/R^2 (probe/fluid constants only, no c(h_bar)):
%               Q(5,5) = Q_aprime_factor * (a_nom/R^2)^2 * (Delta_h_d^2 + sigma2_dh)
%           (sigma2_dh here uses the ESTIMATE 4*kBT*a_hat_i, not the shared
%           wall-peeking sigma2_dh used elsewhere in this file).
%
%   Q(5,5) baseline (per axis, the slope random-walk drive):
%       a''_x   = (a_x / R^2) * (K_h^2 - K_h')          [d^2 a_x / dh^2]
%       Q55     = Q_aprime_factor * [ (a''_x*Delta_h_d)^2 + a''_x^2 * sigma2_dh ]
%   The deterministic-drift term (a''*Delta_h_d)^2 dominates while moving; the
%   random term a''^2*sigma2_dh is the floor. a'' uses the MEASURED h_bar (same
%   bias-free policy as Q44/R22) and only sets the estimator BANDWIDTH on a'_x --
%   it does not leak a'_x into the estimate (a'_x is read purely from a_xm).
%
%   Slot-4 (a_x) Q44/R22 and slots 1-3 are IDENTICAL to the 5-state. Offline
%   constants are dimension-agnostic: this controller reuses
%   build_eq17_6state_constants verbatim.
%
%   ctrl_const knobs (all optional, sensible defaults):
%       .Q_aprime_factor   kappa scaling Q55 baseline           (default 1)
%       .Pf_aprime_scale   scales the a'_x[0] prior variance    (default 1)
%       .freeze_aprime     true -> a'_x held at its seed, Q55=0, no KF update on
%                          slot 5 (L0 regression: reduces to a constant slope)
%
%   a_ctrl_override (true-gain A/B) is identical to the 5-state and is ALSO the
%   open-loop estimability hook (L1): feed the model/true gain so the control law
%   does not use a'_x, while the EKF still estimates a'_x and logs it in
%   diag.delta_a_hat (= a'_x) / diag.aprime_hat.
%
%   See also: motion_control_law_eq17_5state, motion_control_law_eq17_4state,
%             build_eq17_6state_constants, calc_correction_functions

    % ------------------------------------------------------------------
    % Optional gain override (true-gain A/B + open-loop estimability hook).
    % ------------------------------------------------------------------
    if nargin < 6
        a_ctrl_override = [];
    end
    has_override = ~isempty(a_ctrl_override);
    if has_override
        a_ctrl_override = a_ctrl_override(:);
        assert(numel(a_ctrl_override) == 3 && all(isfinite(a_ctrl_override)) && all(a_ctrl_override > 0), ...
               'motion_control_law_eq17_5state_aprime:badOverride', ...
               'a_ctrl_override must be a 3x1 finite positive vector.');
    end

    % ------------------------------------------------------------------
    % Open-loop bypass
    % ------------------------------------------------------------------
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = [1; 1; 1; 0];
        if nargout >= 3
            diag = empty_diag_aprime();
            diag.f_d = f_d;
        end
        return;
    end

    % ------------------------------------------------------------------
    % Persistent state
    % ------------------------------------------------------------------
    persistent x_e_per_axis        % 5x3 EKF state (col = axis); slot 5 = a'_x
    persistent P_per_axis          % cell{3} of 5x5 covariance
    persistent dx_bar_m            % 3x1 IIR LP mean of delta_x_m [um]
    persistent sigma2_dxr_hat      % 3x1 EWMA variance of dx_r    [um^2]
    persistent a_m_det             % 3x1 post-LPF of a_xm (a_m_det) [um/pN]
    persistent pd_km1 pd_km2       % trajectory delay buffers (also give h_d history)
    persistent f_d_km1 f_d_km2     % past control buffers (for Sigma f_d[k-i])
    persistent a_hat_km1 a_hat_km2          % past gain estimates a_hat[k-i] (active law + Q33)
    persistent var_da_ram_km1 var_da_ram_km2 % past var(delta_a_ram[k-i]) (Q33 randgain)
    persistent a_ctrl_km1 a_ctrl_km2   % control-law gain history (= a_hat history unless override)
    persistent warmup_count k_step

    persistent initialized
    persistent lambda_c d_delay Ts kBT R_radius gamma_N_p a_nom_p
    persistent a_pd a_cov C_dpmr C_n K_var IF_abc xi_per_axis var_da_inc_factor
    persistent t_warmup_kf h_bar_safe R_OFF use_am_lpf a_det amlpf_var_factor
    persistent sigma2_n_s a_x_init enable_wall w_hat_n pz_wall
    persistent Q_aprime_factor freeze_aprime aprime_init use_selfmod

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

        % --- 0B. ctrl_const (offline scalars; shared with 5/6-state) ---
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
        use_am_lpf       = get_field_default(ctrl_const, 'use_am_lpf', false);
        a_det            = get_field_default(ctrl_const, 'a_det', 0.005);
        amlpf_var_factor = get_field_default(ctrl_const, 'amlpf_var_factor', 1);
        var_da_inc_factor = get_field_default(ctrl_const, 'var_da_increment_factor', 2 / (1 + lambda_c));
        % --- aprime-specific knobs ---
        Q_aprime_factor  = get_field_default(ctrl_const, 'Q_aprime_factor', 1);
        Pf_aprime_scale  = get_field_default(ctrl_const, 'Pf_aprime_scale', 1);
        freeze_aprime    = logical(get_field_default(ctrl_const, 'freeze_aprime', false));
        % Self-modulation (hold-observability) + honest Q55, per
        % reference/eq17_analysis/derivation/5state_aprime_unified.tex Sec.3-5,7.
        % Default false: bit-identical to the pre-existing baseline.
        use_selfmod      = logical(get_field_default(ctrl_const, 'use_selfmod', false));
        % NOTE: ctrl_const.suppress_xD is ignored (no disturbance term exists).

        % --- 0C. Wall geometry ---
        if isfield(params, 'wall')
            w_hat_n     = params.wall.w_hat(:);
            pz_wall     = params.wall.pz;
            enable_wall = params.wall.enable_wall_effect > 0.5;
        else
            w_hat_n     = [0; 0; 1];
            pz_wall     = 0;
            enable_wall = false;
        end

        % --- 0E. Wall-aware a_x[0] + a'_x[0] seeding (one-time nominal; the
        %         ongoing estimation never re-reads c(h_bar)) ---
        a_nom_p = Ts / gamma_N_p;      % persistent: also needed every step by the
                                       % honest Q55 dimensional anchor (Step 4 below)
        a_nom = a_nom_p;               % local alias, keeps the rest of this block unchanged
        if enable_wall && isfield(params, 'common') && isfield(params.common, 'p0')
            p0_init    = params.common.p0(:);
            h_init_um  = dot(p0_init, w_hat_n) - pz_wall;
            h_bar_init = max(h_init_um / R_radius, 1.001);
            [c_para0, c_perp0, derivs0] = calc_correction_functions(h_bar_init, true);
            a_x_init   = [a_nom / c_para0; a_nom / c_para0; a_nom / c_perp0];
            K_h_init   = [derivs0.K_h_para; derivs0.K_h_para; derivs0.K_h_perp];
            a_perp_init = a_nom / c_perp0;
            % a'_x[0] = da_x/dh|_init = -a_x*K_h/R   (per axis)
            aprime_init = -a_x_init .* K_h_init / R_radius;
        else
            a_x_init    = [a_nom; a_nom; a_nom];
            K_h_init    = zeros(3, 1);
            a_perp_init = a_nom;
            aprime_init = zeros(3, 1);
        end
        if freeze_aprime
            % L0 regression: hold slope at its seed (no process noise, no update)
        end

        % --- 0F. EKF state init (a_x in slot 4, a'_x in slot 5) ---
        x_e_per_axis = zeros(5, 3);
        x_e_per_axis(4, :) = a_x_init.';        % slot 4 = a_x
        x_e_per_axis(5, :) = aprime_init.';     % slot 5 = a'_x

        % --- 0G. Pf init: 4x4 DARE on the observable block (slots 1-4) at the
        %     positioning operating point (Delta_h_d=0 -> a'_x unobservable, kept
        %     out of the DARE), with a finite prior on slot 5. ---
        one_minus_lc = 1 - lambda_c;
        F_e_ss4 = [0 1 0 0; 0 0 1 0; 0 0 lambda_c 0; 0 0 0 1];
        H_ss4   = [1 0 0 0; 0 0 0 1];
        sigma2_dh_init = 4 * kBT * a_perp_init;            % wall-normal thermal motion var
        P_per_axis = cell(3, 1);
        for ax = 1:3
            a_init_ax   = a_x_init(ax);
            var_da_init = var_da_inc_factor * (a_init_ax * K_h_init(ax) / R_radius)^2 * sigma2_dh_init;
            Q33_ss = 4 * kBT * a_init_ax * (1 + one_minus_lc^2 * d_delay) ...
                     + one_minus_lc^2 * sigma2_n_s(ax);
            Q_ss4 = zeros(4);
            Q_ss4(3, 3) = Q33_ss;
            Q_ss4(4, 4) = var_da_init;
            IF_ss  = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a_init_ax, sigma2_n_s(ax));
            R22_ss = amlpf_var_factor * K_var * IF_ss * (a_init_ax + xi_per_axis(ax))^2 ...
                     + d_delay * var_da_init;
            R_ss = [sigma2_n_s(ax), 0; 0, R22_ss];
            P4 = solve_dare_kf_local(F_e_ss4, H_ss4, Q_ss4, R_ss);
            P5 = zeros(5);
            P5(1:4, 1:4) = P4;
            % slot-5 prior: ~order-unity relative uncertainty on the seed slope,
            % with a small absolute floor for the far-field a'~0 case.
            P5(5, 5) = Pf_aprime_scale * (aprime_init(ax)^2 + 1e-6);
            P_per_axis{ax} = P5;
        end

        % --- 0H. IIR states (prefill default) ---
        dx_bar_m = zeros(3, 1);
        iir_mode = get_field_default(ctrl_const, 'iir_warmup_mode', 'prefill');
        if strcmpi(iir_mode, 'prefill')
            sigma2_dxr_hat = 4 * kBT * a_x_init * C_dpmr + C_n * sigma2_n_s;
            warmup_count   = 0;
        else
            sigma2_dxr_hat = zeros(3, 1);
            warmup_count   = 2;
        end
        a_m_det = a_x_init;

        % --- 0I. Delay buffers ---
        pd_km1  = pd;
        pd_km2  = pd;
        f_d_km1 = zeros(3, 1);
        f_d_km2 = zeros(3, 1);
        a_hat_km1 = a_x_init;             a_hat_km2 = a_x_init;
        a_ctrl_km1 = a_x_init;            a_ctrl_km2 = a_x_init;
        var_da_init_vec = zeros(3, 1);
        for ax = 1:3
            var_da_init_vec(ax) = var_da_inc_factor * (a_x_init(ax) * K_h_init(ax) / R_radius)^2 * sigma2_dh_init;
        end
        var_da_ram_km1 = var_da_init_vec; var_da_ram_km2 = var_da_init_vec;

        % --- 0K. Misc ---
        k_step = 1;
        R_OFF  = 1e10;

        % --- 0L. First call returns zeros (no f_d yet) ---
        f_d = zeros(3, 1);
        ekf_out = [a_x_init(1); a_x_init(3); a_x_init(2); 0];
        if nargout >= 3
            diag = empty_diag_aprime();
            diag.f_d            = f_d;
            diag.a_hat          = a_x_init;
            diag.aprime_hat     = aprime_init;
            diag.delta_a_hat    = aprime_init;
            diag.sigma2_dxr_hat = sigma2_dxr_hat;
            if has_override
                diag.a_ctrl_used = a_ctrl_override;
            else
                diag.a_ctrl_used = a_x_init;
            end
        end
        return;
    end

    % ------------------------------------------------------------------
    % Per-step: extract per-axis state
    % ------------------------------------------------------------------
    a_hat = x_e_per_axis(4, :).';     % 3x1 [um/pN]  (slot 4); a'_x (slot 5) read in-loop

    if has_override
        a_ctrl = a_ctrl_override;        % true/model gain (also open-loop hook)
    else
        a_ctrl = a_hat;                  % normal mode: EKF posterior[k-1]
    end

    % delta_x_m[k] = p_d[k-d] - p_m[k]
    if d_delay == 2
        pd_km_d = pd_km2;
    elseif d_delay == 1
        pd_km_d = pd_km1;
    else
        error('motion_control_law_eq17_5state_aprime:unsupportedDelay', ...
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
    % Known desired-trajectory height increments (wall-normal projections).
    %   Delta_h_d[k]    = h_d[k+1]-h_d[k]   = dot(del_pd, w_hat)   (predict F_e(4,5))
    %   dh_i = h_d[k]-h_d[k-i] = dot(pd - pd_km_i, w_hat)          (dF_dx^h, Delta_H_d)
    %   Delta_H_d[k]    = h_d[k]-h_d[k-d]                          (measurement H(2,5))
    % All scalars (shared across axes); a'_x is per-axis.
    % ------------------------------------------------------------------
    Delta_h_d = dot(del_pd, w_hat_n);                    % [um]
    dh1       = dot(pd - pd_km1, w_hat_n);               % h_d[k]-h_d[k-1]
    if d_delay == 2
        dh2       = dot(pd - pd_km2, w_hat_n);           % h_d[k]-h_d[k-2]
        Delta_H_d = dh2;                                 % = h_d[k]-h_d[k-d]
    else
        dh2       = 0;
        Delta_H_d = dh1;
    end

    % ------------------------------------------------------------------
    % Wall functions at measured h_bar (deterministic; avoids bias loop).
    % K_h and K_h' feed Q44 (var_da_ram) and the Q55 a'' baseline.
    % ------------------------------------------------------------------
    if enable_wall && isfinite(h_bar) && h_bar > 1
        [~, c_perp_h, derivs] = calc_correction_functions(h_bar, true);
        K_h_axis  = [derivs.K_h_para; derivs.K_h_para; derivs.K_h_perp];
        K_hp_axis = [derivs.K_h_prime_para; derivs.K_h_prime_para; derivs.K_h_prime_perp];
        a_perp_meas = Ts / (gamma_N_p * c_perp_h);
    else
        K_h_axis  = zeros(3, 1);
        K_hp_axis = zeros(3, 1);
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
    a_m_det_new = (1 - a_det) * a_m_det + a_det * a_xm;   % 3x1 [um/pN]
    if use_am_lpf
        a_meas = a_m_det_new;
    else
        a_meas = a_xm;
    end

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
        a_m_det        = a_m_det_new;
        pd_km2 = pd_km1; pd_km1 = pd;
        f_d_km2 = f_d_km1; f_d_km1 = f_d;
        a_hat_km2 = a_hat_km1; a_hat_km1 = a_hat;
        a_ctrl_km2 = a_ctrl_km1; a_ctrl_km1 = a_ctrl;
        warmup_count = warmup_count - 1;
        k_step = k_step + 1;
        a_hat_post = x_e_per_axis(4, :).';
        h_bar_now = local_h_bar_out(enable_wall, h_bar);
        ekf_out = [a_hat_post(1); a_hat_post(3); a_hat_post(2); h_bar_now];
        if nargout >= 3
            diag = empty_diag_aprime();
            diag.f_d = f_d; diag.a_hat = a_hat_post;
            diag.aprime_hat = x_e_per_axis(5, :).'; diag.delta_a_hat = x_e_per_axis(5, :).';
            diag.a_ctrl_used = a_ctrl;
            diag.sigma2_dxr_hat = sigma2_dxr_hat_new; diag.a_xm = a_xm;
            diag.delta_x_m = delta_x_m; diag.h_bar = h_bar; diag.dx_r = dx_r;
        end
        return;
    end

    % ------------------------------------------------------------------
    % [3] Control law (B2 ACTIVE form, Vpersonal p.2; NO -x_hat_D term)
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
    % [4] Q (5x5 diagonal) and R (2x2) per axis
    %   Q33 = Var(epsilon); Q44 = var(delta_a_ram); Q55 = Var(w_a') baseline
    % ------------------------------------------------------------------
    Q_per_axis = cell(3, 1);
    R_per_axis = cell(3, 1);
    gate_off   = false(3, 1);
    G_flags    = false(3, 3);
    var_da_ram = zeros(3, 1);
    Q55_vec    = zeros(3, 1);
    t_now = (k_step - 1) * Ts;
    for ax = 1:3
        a_hat_i = a_hat(ax);
        var_da_ram(ax) = var_da_inc_factor * (a_hat_i * K_h_axis(ax) / R_radius)^2 * sigma2_dh;

        if d_delay == 2
            Q33_thermal  = 4 * kBT * (a_hat_i + one_minus_lc^2 * (a_hat_km1(ax) + a_hat_km2(ax)));
            Q33_randgain = one_minus_lc^2 * ( 4 * f_d_km1(ax)^2 * var_da_ram_km1(ax) ...
                                            + 1 * f_d_km2(ax)^2 * var_da_ram_km2(ax) );
        else
            Q33_thermal  = 4 * kBT * (a_hat_i + one_minus_lc^2 * a_hat_km1(ax));
            Q33_randgain = one_minus_lc^2 * (f_d_km1(ax)^2 * var_da_ram_km1(ax));
        end
        Q33_nx = one_minus_lc^2 * sigma2_n_s(ax);

        % Q55 = Var(w_a'). use_selfmod=false (baseline): a''=(a/R^2)(K_h^2-K_h')
        % from the REAL wall model (calc_correction_functions) -- wall-peeking,
        % kept only for backward compatibility with the pre-existing baseline.
        % use_selfmod=true (honest): dimensional anchor sigma_a''~a_nom/R^2
        % (probe/fluid constants only, no c(h_bar)) AND the thermal variance
        % term uses the ESTIMATE a_hat_i (4*kBT*a_hat_i), NOT the shared
        % sigma2_dh (which is wall-peeking: it is built from a_perp_meas,
        % which calls calc_correction_functions a few lines above -- reusing
        % it here would silently smuggle c(h_bar) back into the "honest" Q55).
        % 5state_aprime_unified.tex Section 7.
        if use_selfmod
            sigma2_a2prime     = (a_nom_p / R_radius^2)^2;
            sigma2_dh_honest_i = 4 * kBT * a_hat_i;
            Q55_i = Q_aprime_factor * sigma2_a2prime * (Delta_h_d^2 + sigma2_dh_honest_i);
        else
            a_dprime_i = (a_hat_i / R_radius^2) * (K_h_axis(ax)^2 - K_hp_axis(ax));
            Q55_i = Q_aprime_factor * ((a_dprime_i * Delta_h_d)^2 + a_dprime_i^2 * sigma2_dh);
        end
        if freeze_aprime
            Q55_i = 0;
        end
        Q55_vec(ax) = Q55_i;

        Q_i = zeros(5);
        Q_i(3, 3) = Q33_thermal + Q33_randgain + Q33_nx;
        Q_i(4, 4) = var_da_ram(ax);
        Q_i(5, 5) = Q55_i;
        Q_per_axis{ax} = Q_i;

        R11_i = sigma2_n_s(ax);
        IF_eff_i = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a_hat_i, sigma2_n_s(ax));
        R2_intrinsic_i = amlpf_var_factor * K_var * IF_eff_i * (a_hat_i + xi_per_axis(ax))^2;
        if d_delay == 2
            R22_delay_i = var_da_ram_km1(ax) + var_da_ram_km2(ax);
        else
            R22_delay_i = var_da_ram_km1(ax);
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
    % [5] EKF predict + update per axis
    % ------------------------------------------------------------------
    H_y1 = [1 0 0 0 0];

    K_a_y2_v  = zeros(3, 1);
    K_dx_y1_v = zeros(3, 1);
    innov_y2_v = zeros(3, 1);

    for ax = 1:3
        x_curr = x_e_per_axis(:, ax);
        P_curr = P_per_axis{ax};

        % F_e Row 3: -F_dx (col 4), dF_dx^h (col 5); Row 4 col 5 = Delta_h_d
        % [+ (1-lc)*delta_x_hat_3 when use_selfmod, restoring hold-observability].
        if d_delay == 2
            F_1_i  = f_d_km1(ax) + f_d_km2(ax);                       % sum f_d[k-i]
            dFh_i  = one_minus_lc * (dh1 * f_d_km1(ax) + dh2 * f_d_km2(ax));
        else
            F_1_i  = f_d_km1(ax);
            dFh_i  = one_minus_lc * (dh1 * f_d_km1(ax));
        end
        if use_selfmod
            dxh3_sm = x_curr(3);   % current (prior) delta_x_hat_3 estimate
        else
            dxh3_sm = 0;
        end
        F_e = build_F_e_5state_aprime(lambda_c, f_d(ax), F_1_i, dFh_i, Delta_h_d, dxh3_sm);

        % --- Predict: deterministic map. a_x advances by a'_x * Delta_h_d
        %     (estimated slope x known increment); a'_x random walk. ---
        x_pred = [x_curr(2); ...
                  x_curr(3); ...
                  lambda_c * x_curr(3); ...
                  x_curr(4) + x_curr(5) * Delta_h_d; ...
                  x_curr(5)];
        P_pred = F_e * P_curr * F_e' + Q_per_axis{ax};
        P_pred = 0.5 * (P_pred + P_pred');

        % --- Update (1D if y_2 gated, else 2D). a_xm measures a_x[k-d] directly;
        %     the d-step gain drift is carried by H(2,5) = -Delta_H_d
        %     [+ (delta_x_hat_3-delta_x_hat_1) when use_selfmod, matching the
        %     F_e(4,5) thermal-dither term above -- both evaluated at the
        %     PREDICTED state x_pred, standard EKF linearization convention].
        if use_selfmod
            H25_i = -Delta_H_d + (x_pred(3) - x_pred(1));
        else
            H25_i = -Delta_H_d;
        end
        H_full_i = [1 0 0 0 0; 0 0 0 1 H25_i];

        if gate_off(ax)
            H_use = H_y1;
            y_use = delta_x_m(ax);
            R_use = sigma2_n_s(ax);
        else
            H_use = H_full_i;
            y_use = [delta_x_m(ax); a_meas(ax)];
            R_use = R_per_axis{ax};
        end

        y_pred = H_use * x_pred;
        innov  = y_use - y_pred;
        S_inn  = H_use * P_pred * H_use' + R_use;
        S_inn  = 0.5 * (S_inn + S_inn');
        K_kf   = (P_pred * H_use') / S_inn;

        % Warmup gate (G1): freeze a_x; freeze_aprime: freeze a'_x always.
        if G_flags(1, ax)
            K_kf(4, :) = 0;
            K_kf(5, :) = 0;
        end
        if freeze_aprime
            K_kf(5, :) = 0;
        end

        x_post = x_pred + K_kf * innov;
        ImKH   = eye(5) - K_kf * H_use;
        P_post = ImKH * P_pred * ImKH' + K_kf * R_use * K_kf';   % Joseph form
        P_post = 0.5 * (P_post + P_post');

        % Diagnostics
        K_dx_y1_v(ax) = K_kf(3, 1);
        if gate_off(ax)
            K_a_y2_v(ax)  = 0;
            innov_y2_v(ax) = 0;
        else
            K_a_y2_v(ax)  = K_kf(4, 2);
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
    a_hat_km2 = a_hat_km1; a_hat_km1 = a_hat;
    a_ctrl_km2 = a_ctrl_km1; a_ctrl_km1 = a_ctrl;
    var_da_ram_km2 = var_da_ram_km1; var_da_ram_km1 = var_da_ram;
    dx_bar_m = dx_bar_m_new;
    sigma2_dxr_hat = sigma2_dxr_hat_new;
    a_m_det = a_m_det_new;
    k_step = k_step + 1;

    % ------------------------------------------------------------------
    % [7] Output
    % ------------------------------------------------------------------
    a_hat_post      = x_e_per_axis(4, :).';
    aprime_hat_post = x_e_per_axis(5, :).';
    h_bar_now = local_h_bar_out(enable_wall, h_bar);
    ekf_out = [a_hat_post(1); a_hat_post(3); a_hat_post(2); h_bar_now];

    if nargout >= 3
        P_a_v = zeros(3, 1); P_dx_v = zeros(3, 1); P_dx1_v = zeros(3, 1);
        P_ap_v = zeros(3, 1);
        for ax = 1:3
            P_a_v(ax)   = P_per_axis{ax}(4, 4);
            P_dx_v(ax)  = P_per_axis{ax}(3, 3);
            P_dx1_v(ax) = P_per_axis{ax}(1, 1);
            P_ap_v(ax)  = P_per_axis{ax}(5, 5);
        end
        diag = empty_diag_aprime();
        diag.sigma2_dxr_hat = sigma2_dxr_hat_new;
        diag.a_xm           = a_xm;
        diag.delta_x_m      = delta_x_m;
        diag.innovation_y2  = innov_y2_v;
        diag.K_kf_a_y2      = K_a_y2_v;
        diag.K_kf_dx_y1     = K_dx_y1_v;
        diag.P_a            = P_a_v;
        diag.P_dx           = P_dx_v;
        diag.x_D_hat              = zeros(3, 1);            % no disturbance state (driver compat)
        diag.aprime_hat           = aprime_hat_post;       % a'_x estimate (slot 5)
        diag.delta_a_hat          = aprime_hat_post;       % driver-log alias (slot 5)
        diag.gate_active_per_axis = gate_off;
        diag.guards_individual    = G_flags;
        diag.h_bar                = h_bar;
        diag.f_d                  = f_d;
        diag.dx_r                 = dx_r;
        diag.a_hat                = a_hat_post;
        diag.a_ctrl_used          = a_ctrl;
        diag.P77                  = P_ap_v;                 % a'_x posterior variance
        diag.Q77                  = Q55_vec;                % Q55 baseline used
        diag.var_da_ram           = var_da_ram;
        diag.Delta_h_d            = Delta_h_d;
        diag.Delta_H_d            = Delta_H_d;
        diag.delta_x_hat_1        = x_e_per_axis(1, :).';
        diag.P_dx1                = P_dx1_v;
    end
end


%% =================== Local Helpers ===================
%   (build_F_e_5state_aprime is a standalone file so the L0 Jacobian check can
%    exercise the exact matrix used here.)

function v = get_field_default(s, name, default)
%GET_FIELD_DEFAULT  s.(name) if present and non-empty, else default.
    if isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
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


function h = local_h_bar_out(enable_wall, h_bar)
    if enable_wall
        h = h_bar;
    else
        h = 0;
    end
end


function d = empty_diag_aprime()
%EMPTY_DIAG_APRIME  Zeroed diagnostic struct for the aprime controller.
    d = struct();
    d.sigma2_dxr_hat    = zeros(3, 1);
    d.a_xm              = zeros(3, 1);
    d.delta_x_m         = zeros(3, 1);
    d.innovation_y2     = zeros(3, 1);
    d.K_kf_a_y2         = zeros(3, 1);
    d.K_kf_dx_y1        = zeros(3, 1);
    d.P_a               = zeros(3, 1);
    d.P_dx              = zeros(3, 1);
    d.x_D_hat              = zeros(3, 1);   % no disturbance state (placeholder, driver compat)
    d.aprime_hat           = zeros(3, 1);   % a'_x estimate (slot 5)
    d.delta_a_hat          = zeros(3, 1);   % driver-log alias for slot 5
    d.gate_active_per_axis = false(3, 1);
    d.guards_individual    = false(3, 3);
    d.h_bar                = 0;
    d.f_d                  = zeros(3, 1);
    d.dx_r                 = zeros(3, 1);
    d.a_hat                = zeros(3, 1);
    d.P77                  = zeros(3, 1);   % a'_x posterior variance
    d.Q77                  = zeros(3, 1);   % Q55 baseline
    d.var_da_ram           = zeros(3, 1);
    d.Delta_h_d            = 0;
    d.Delta_H_d            = 0;
    d.delta_x_hat_1        = zeros(3, 1);
    d.P_dx1                = zeros(3, 1);
    d.a_ctrl_used          = zeros(3, 1);
end
