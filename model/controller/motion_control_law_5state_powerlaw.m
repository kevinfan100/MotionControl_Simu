function [f_d, ekf_out, diag] = motion_control_law_5state_powerlaw(del_pd, pd, p_m, params, ctrl_const, a_ctrl_override)
%MOTION_CONTROL_LAW_5STATE_POWERLAW Per-axis 5-state EKF eq17 controller whose
%   motion gain a_h follows a two-parameter POWER-LAW (gain level a_h + exponent
%   p), with slope s_h = (p/(h_bar_d-1))*a_h*(a_o-a_h)/a_o (R-free). The wall
%   correction c(h_bar) and its amplitude never enter the recursion (a_h's level
%   carries them). Authoritative spec:
%   reference/eq17_analysis/derivation/5state_powerlaw_hd.tex.
%
%   [f_d, ekf_out]       = motion_control_law_5state_powerlaw(del_pd, pd, p_m, params, ctrl_const)
%   [f_d, ekf_out, diag] = motion_control_law_5state_powerlaw(...)
%   [f_d, ekf_out, diag] = motion_control_law_5state_powerlaw(..., ctrl_const, a_ctrl_override)
%
%   Per-axis (x, y, z; z = wall-normal / perpendicular) state:
%
%       x = [delta_h_1; delta_h_2; delta_h_3; a_h; p]
%
%   delta_h_1 = delta_h[k-2] (measured, d=2 delay), delta_h_3 = delta_h[k]
%   (current), a_h = motion gain [um/pN], p = power-law exponent (Brenner p=1).
%
%   Gain model (a_o = a_nom = far-field gain, h_bar_d = h_d/R desired height):
%       s_h[k]  = (p/(h_bar_d-1)) * a_h*(a_o-a_h)/a_o    (da_h/d h_bar, R-free)
%       a_h'    = s_h / R                                (da_h/dh, physical)
%       s_a     = d s_h / d a_h = (p/(h_bar_d-1))*(a_o-2 a_h)/a_o
%       s_h/p   = (1/(h_bar_d-1)) * a_h*(a_o-a_h)/a_o    (p-free; d s_h/d p)
%
%   Estimator PREDICT (slot 4 swept by the estimated slope; slot 5 constant):
%       a_h[k+1] = a_h + s_h*Delta_hbar_d + (1-lc)*(s_h/R)*delta_h_3
%       p[k+1]   = p
%   Control law (eq17, implementable; no disturbance term):
%       f_dh[k]  = a_ctrl^-1 { Delta_h_d^d[k]
%                              + (1-lc)[delta_h_m[k] - sum_i a_ctrl[k-i] f_dh[k-i]] }
%       Delta_h_d^d[k] = Delta_h_d[k] + (1-lc) sum_{i=1}^d Delta_h_d[k-i].
%   Measurements. a_xm is NOT a white measurement of a_h: it is an EXACT AR(1)
%   filter of the single-sample gain readout u[k] with pole 1-a_cov,
%       a_xm[k] = (1-a_cov)*a_xm[k-1] + a_cov*u[k],   E[u[k]] = a_h[k-d],
%   so feeding a_xm itself over-states its information by (2-a_cov)/a_cov
%   (=39 at a_cov=0.05). The KF is fed the WHITENED increment instead:
%       y1 = delta_h_m = delta_h_1 + n_h
%       y2 = a_xm[k] - (1-a_cov)*a_xm[k-1] = a_cov*u[k],  E[y2] = a_cov*a_h[k-d]
%   H = [1 0 0    0             0                  ;
%        0 0 0  a_cov  -a_cov*Delta_Hbar_d*(s_h/p)]  (Delta_Hbar_d=(h_d[k]-h_d[k-d])/R)
%   Whitening also removes the EWMA group delay ((1-a_cov)/a_cov ~ 19 steps),
%   which the d-step back-off alone did not cover.
%   F_e (5x5) is built by local_build_F_e from the DETERMINISTIC control mirror
%   F_dh_det (delta_h_m := 0), not the realised f_d -- see [2b] in the body.
%
%   Q (Path C strict, rank-1 gain block + Q33):
%       Q33 = 4*kB*T*a_h ,  Q44 = a_h'^2*Q33 ,  Q34 = Q43 = -a_h'*Q33 ,  Q55 = 0.
%   R (isolated in one function):
%       R1 = sigma2_n_h (sensor variance on delta_h_m),
%       R2 = compute_R2_whitened(...)  (variance of the whitened y2 increment).
%
%   ctrl_const is the offline-scalars struct from build_eq17_6state_constants
%   (dimension-agnostic; only slot 5 differs from the aprime sibling). Optional
%   knobs:
%       .Q55_floor   tiny floor on Q(5,5) for conditioning   (default 0)
%       .Pf_a_frac   sqrt(Pf(4,4))/a_h init frac    (default 5/h_bar_0^2,
%                    i.e. matched to the asymptotic seed's own accuracy)
%       .Pf_p_std    sqrt(Pf(5,5)) init exponent std         (default 0.035)
%       .p_init      initial exponent estimate               (default 1, Brenner)
%
%   Pf_p_std default 0.035 is NOT tuned, and does NOT require knowing c(h_bar).
%   p = 1 is pinned by two analytically known regimes: Brenner lubrication
%   (c_perp -> 1/(h_bar-1) as h_bar -> 1) and the far-field reflection series
%   (c_perp - 1 = (9/8)/h_bar). Both force p = 1, verified numerically as
%   p_eff(1+) = 0.9994 and p_eff(inf) = 1.0001, so the prior only covers the
%   interpolation gap between the two anchors: max|p_eff - 1| = 0.034 at
%   h_bar = 1.22, over the WHOLE domain (hence trajectory-independent).
%   The parallel axes have no near-wall anchor -- Goldman's near-wall law is
%   logarithmic, p_eff(1+) -> 0 -- so neither this prior nor Q55 = 0 transfers
%   to them; that is a structural failure of the power law for x/y, not a prior
%   width issue.
%
%   a_ctrl_override (3x1, optional): feed the model/true gain to the CONTROL LAW
%   (open-loop estimability A/B); the EKF still estimates a_h / p from a_xm.
%
%   See also: motion_control_law_eq17_5state_aprime, build_eq17_6state_constants,
%             calc_correction_functions

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
               'motion_control_law_5state_powerlaw:badOverride', ...
               'a_ctrl_override must be a 3x1 finite positive vector.');
    end

    % ------------------------------------------------------------------
    % Open-loop bypass
    % ------------------------------------------------------------------
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = [1; 1; 1; 0];
        if nargout >= 3
            diag = empty_diag_powerlaw();
            diag.f_d = f_d;
        end
        return;
    end

    % ------------------------------------------------------------------
    % Persistent state
    % ------------------------------------------------------------------
    persistent x_e_per_axis        % 5x3 EKF state (col = axis); slot 4=a_h, slot 5=p
    persistent P_per_axis          % cell{3} of 5x5 covariance
    persistent dx_bar_m            % 3x1 IIR LP mean of delta_h_m [um]
    persistent sigma2_dxr_hat      % 3x1 EWMA variance of dx_r    [um^2]
    persistent a_xm_km1            % 3x1 a_xm[k-1], for the whitening increment
    persistent fdet_km1 fdet_km2   % noise-free control mirror, own history
    persistent pd_km1 pd_km2       % trajectory delay buffers (pd[k-1], pd[k-2])
    persistent f_d_km1 f_d_km2     % past control buffers (Sigma f_dh[k-i])
    persistent a_ctrl_km1 a_ctrl_km2        % control-law gain history
    persistent k_step

    persistent initialized
    persistent lambda_c d_delay Ts kBT R_radius gamma_N_p a_nom_p
    persistent a_pd a_cov C_dpmr C_n K_var IF_abc xi_per_axis amlpf_var_factor
    persistent t_warmup_kf h_bar_safe sigma2_n_s
    persistent enable_wall w_hat_n pz_wall
    persistent Q55_floor a_floor delay_R2_factor

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
        IF_abc          = ctrl_const.IF_abc(:);
        xi_per_axis     = ctrl_const.xi_per_axis(:);
        t_warmup_kf     = ctrl_const.t_warmup_kf;
        h_bar_safe      = ctrl_const.h_bar_safe;
        a_cov           = ctrl_const.a_cov;
        a_pd            = ctrl_const.a_pd;
        amlpf_var_factor = get_field_default(ctrl_const, 'amlpf_var_factor', 1);

        % Power-law knobs (defaults keep it conservative per the anti-blowup note).
        Q55_floor = get_field_default(ctrl_const, 'Q55_floor', 0);

        % 0.035 = the interpolation gap between the two p=1 asymptotic anchors
        % (see the header note); a prior, not a knob, and domain-wide.
        Pf_p_std  = get_field_default(ctrl_const, 'Pf_p_std', 0.035);
        p_init    = get_field_default(ctrl_const, 'p_init', 1.0);   % Brenner

        % d-step delay R2 factor: 7-state "5*Q77" generalized to sum_{j=1}^d (d-j+1)^2
        % (=5 for d=2, 1 for d=1); reused with Q77 -> Q44 in the R2 placeholder.
        delay_R2_factor = sum(((d_delay:-1:1)).^2);

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

        % --- 0D. Far-field gain + a_h[0] seeding from the FAR-FIELD ASYMPTOTE.
        %   c(h_bar) is NEVER read, here or later. The seed uses only the
        %   leading method-of-reflections term -- the same published asymptote
        %   that pins p = 1 in the gain model:
        %       c_perp ~ 1 + (9/8)/h_bar ,   c_para ~ 1 + (9/16)/h_bar
        %   and h_bar_0, i.e. the wall position, which is assumed known.
        %   Accuracy at the start height (verified against the exact curve):
        %       h_bar_0    50     22.2      10       5
        %       err       0.05%   0.25%   1.23%   4.91%     ~ 1.25/h_bar_0^2
        %   versus 2.3% / 5.3% / 12.6% / 28.5% for the flat a_nom assumption.
        a_nom_p = Ts / gamma_N_p;      % a_o = far-field gain [um/pN]
        h_bar_init = Inf;
        if enable_wall && isfield(params, 'common') && isfield(params.common, 'p0')
            p0_init    = params.common.p0(:);
            h_init_um  = dot(p0_init, w_hat_n) - pz_wall;
            h_bar_init = max(h_init_um / R_radius, 1.001);
            a_h_init   = [a_nom_p / (1 + (9/16) / h_bar_init); ...
                          a_nom_p / (1 + (9/16) / h_bar_init); ...
                          a_nom_p / (1 + (9/8)  / h_bar_init)];
        else
            a_h_init    = [a_nom_p; a_nom_p; a_nom_p];
        end

        % Pf_a_frac must MATCH the seed's own accuracy, not be loosened to buy
        % correction room: an over-tight prior converges far too slowly (5.33%
        % init error under a 1% prior moved only 0.9% in 12 s), an over-loose
        % one lets the noisy a_xm pull the estimate around. The asymptotic seed
        % above is good to ~1.25/h_bar_0^2, so the default is that residual with
        % a 4x safety factor. Far from the wall this is a tight, honest prior.
        if isfinite(h_bar_init)
            Pf_a_default = min(max(5 / h_bar_init^2, 0.002), 0.3);
        else
            Pf_a_default = 0.02;       % no wall: a_h[0] = a_nom is exact
        end
        Pf_a_frac = get_field_default(ctrl_const, 'Pf_a_frac', Pf_a_default);

        % gain floor used to keep 1/a_ctrl and Q33 well-conditioned (numerical
        % safety only; physically a_h stays a_nom/c > 0).
        a_floor = 0.05 * a_nom_p;

        % --- 0E. EKF state init: slot 4 = a_h (wall-aware), slot 5 = p (Brenner) ---
        x_e_per_axis = zeros(5, 3);
        x_e_per_axis(4, :) = a_h_init.';
        x_e_per_axis(5, :) = p_init;

        % --- 0F. Pf init: DARE on the observable position 3-block (slots 1-3),
        %     MODEST gain/exponent priors (slot 4/5) to avoid the a_hat blow-up
        %     the aprime sibling saw with oversized Pf. ---
        F3 = [0 1 0; 0 0 1; 0 0 lambda_c];
        H3 = [1 0 0];
        P_per_axis = cell(3, 1);
        for ax = 1:3
            a_init_ax = a_h_init(ax);
            Q33_ss = 4 * kBT * a_init_ax;
            Q3 = zeros(3); Q3(3, 3) = Q33_ss;
            R3 = sigma2_n_s(ax);
            P3 = solve_dare_kf_local(F3, H3, Q3, R3);
            P5 = zeros(5);
            P5(1:3, 1:3) = P3;
            P5(4, 4) = (Pf_a_frac * a_init_ax)^2;   % ~(0.3*a_h)^2, modest
            P5(5, 5) = Pf_p_std^2;                  % (0.035)^2 = anchor interp. gap
            P_per_axis{ax} = P5;
        end

        % --- 0G. IIR states (prefill to the closed-loop dx_r variance) ---
        dx_bar_m = zeros(3, 1);
        sigma2_dxr_hat = 4 * kBT * a_h_init * C_dpmr + C_n * sigma2_n_s;
        % the prefill above is exactly a_xm[0] = a_h_init, so seed the whitening
        % delay slot with it (first increment is then noise-only, not a step).
        a_xm_km1 = a_h_init;

        % --- 0H. Delay buffers ---
        pd_km1  = pd;
        pd_km2  = pd;
        f_d_km1 = zeros(3, 1);
        f_d_km2 = zeros(3, 1);
        fdet_km1 = zeros(3, 1);
        fdet_km2 = zeros(3, 1);
        a_ctrl_km1 = a_h_init;  a_ctrl_km2 = a_h_init;
        k_step = 1;

        % --- 0I. First call returns zeros (no f_d yet) ---
        f_d = zeros(3, 1);
        ekf_out = [a_h_init(1); a_h_init(3); a_h_init(2); 0];
        if nargout >= 3
            diag = empty_diag_powerlaw();
            diag.f_d      = f_d;
            diag.a_hat    = a_h_init;
            diag.p_hat    = p_init * ones(3, 1);
            diag.delta_a_hat = p_init * ones(3, 1);
            diag.sigma2_dxr_hat = sigma2_dxr_hat;
            if has_override
                diag.a_ctrl_used = a_ctrl_override;
            else
                diag.a_ctrl_used = a_h_init;
            end
        end
        return;
    end

    % ------------------------------------------------------------------
    % Per-step: extract per-axis state
    % ------------------------------------------------------------------
    a_hat = x_e_per_axis(4, :).';     % 3x1 [um/pN]  (slot 4)

    if has_override
        a_ctrl = a_ctrl_override;
    else
        a_ctrl = a_hat;
    end
    a_ctrl = max(a_ctrl, a_floor);    % numerical safety on 1/a_ctrl

    % delta_h_m[k] = p_d[k-d] - p_m[k]
    if d_delay == 2
        pd_km_d = pd_km2;
    elseif d_delay == 1
        pd_km_d = pd_km1;
    else
        error('motion_control_law_5state_powerlaw:unsupportedDelay', ...
              'Only d=1 or d=2 supported, got d=%g.', d_delay);
    end
    delta_x_m = pd_km_d - p_m;           % 3x1 [um]
    pd_kp1    = pd + del_pd;             % 3x1 [um]

    % h_bar from current measurement (Guard 3)
    if enable_wall
        h_bar = (dot(p_m, w_hat_n) - pz_wall) / R_radius;
    else
        h_bar = Inf;
    end

    one_minus_lc = 1 - lambda_c;

    % ------------------------------------------------------------------
    % Known desired-trajectory height increments (wall-normal projections).
    %   Delta_h_d[k] = h_d[k+1]-h_d[k]                      (predict / F_e)
    %   dh_i = h_d[k]-h_d[k-i]                              (Delta_h_d^d, Delta_H_d)
    %   h_bar_d = h_d[k]/R  (desired height; slope operating point, noise-free)
    % All scalars (shared across axes); a_h / p are per-axis.
    % ------------------------------------------------------------------
    Delta_h_d = dot(del_pd, w_hat_n);                    % [um]
    if d_delay == 2
        Delta_H_d = dot(pd - pd_km2, w_hat_n);           % h_d[k]-h_d[k-d]
    else
        Delta_H_d = dot(pd - pd_km1, w_hat_n);           % h_d[k]-h_d[k-d]
    end
    if enable_wall
        h_bar_d = (dot(pd, w_hat_n) - pz_wall) / R_radius;
    else
        h_bar_d = Inf;
    end
    Delta_hbar_d = Delta_h_d / R_radius;                 % dimensionless
    Delta_Hbar_d = Delta_H_d / R_radius;                 % dimensionless

    % ------------------------------------------------------------------
    % [1] IIR a_xm (paper 2025 Eq.9-13) -- reused verbatim from the sibling.
    % ------------------------------------------------------------------
    dx_bar_m_new = (1 - a_pd) * dx_bar_m + a_pd * delta_x_m;
    dx_r = delta_x_m - dx_bar_m_new;
    sigma2_dxr_hat_new = (1 - a_cov) * sigma2_dxr_hat + a_cov * dx_r.^2;
    a_xm = (sigma2_dxr_hat_new - C_n * sigma2_n_s) / (C_dpmr * 4 * kBT);   % 3x1 [um/pN]
    % a_xm is an EXACT AR(1) filter of the single-sample readout u[k] with pole
    % 1-a_cov (subtracting/dividing by constants leaves the EWMA recursion
    % invariant). Feed the KF the WHITENED increment y2 = a_cov*u[k] so the pole
    % is cancelled and a_cov stops leaking into the estimates.
    a_meas = a_xm - (1 - a_cov) * a_xm_km1;      % 3x1 = y2 (whitened) [um/pN]

    % ------------------------------------------------------------------
    % [2] Control law (eq17 implementable; NO disturbance term)
    %   f_dh = a_ctrl^-1 { Delta_h_d^d[k] + (1-lc)[dh_m - sum a_ctrl[k-i]*f_dh[k-i]] }
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
    % [2b] Deterministic mirror of the control law (delta_h_m := 0, running on
    %   its OWN history). Purely trajectory-driven, hence EXOGENOUS.
    %   F_e must use THIS, not the realised f_d: the error dynamics
    %       delta_h[k+1] = lc*delta_h - F_dh*e_ah - eps_h
    %   is exact with the realised force, but handing that F_dh to the KF makes
    %   F_e a random matrix sharing the thermal noise of eps_h. P and K then
    %   become innovation-correlated and E[K*innov] != 0 -- a RECTIFICATION that
    %   is one-signed regardless of the noise sign, so it accumulates instead of
    %   averaging out. Measured with the realised f_d: y1 dragged a_hat down by
    %   24.3% of a_true over 12 s of positioning, monotonically. With the mirror
    %   the same contribution is +0.0%. See 5state_powerlaw_hd.tex, F_e section.
    % ------------------------------------------------------------------
    if d_delay == 2
        sum_af_det = a_ctrl_km1 .* fdet_km1 + a_ctrl_km2 .* fdet_km2;
    else
        sum_af_det = a_ctrl_km1 .* fdet_km1;
    end
    f_det = inv_a_ctrl .* (pd_kp1 - lambda_c * pd - one_minus_lc * pd_km_d ...
                           - one_minus_lc * sum_af_det);
    if d_delay == 2
        F_dh_det = f_det + one_minus_lc * (fdet_km1 + fdet_km2);
    else
        F_dh_det = f_det + one_minus_lc * fdet_km1;
    end

    % ------------------------------------------------------------------
    % [3] Per-axis slope, Q, R, gate; then EKF predict + sequential 1-D update.
    % ------------------------------------------------------------------
    I5 = eye(5);
    t_now = (k_step - 1) * Ts;

    K_a_y2_v   = zeros(3, 1);
    K_dx_y1_v  = zeros(3, 1);
    innov_y2_v = zeros(3, 1);
    gate_off   = false(3, 1);
    G_flags    = false(3, 3);
    a_prime_v  = zeros(3, 1);
    Q44_v      = zeros(3, 1);

    for ax = 1:3
        x_curr = x_e_per_axis(:, ax);
        P_curr = P_per_axis{ax};
        a_h_i  = max(x_curr(4), a_floor);
        p_i    = x_curr(5);

        % --- Power-law slope at the desired operating point (a_o = a_nom) ---
        [s_h, s_a, s_h_over_p] = local_slope(a_h_i, p_i, h_bar_d, a_nom_p, enable_wall);
        a_prime_i = s_h / R_radius;                       % da_h/dh
        a_prime_v(ax) = a_prime_i;

        % --- Q (5x5): Path C strict rank-1 gain block + Q33 ---
        Q33 = 4 * kBT * a_h_i;
        Q_i = zeros(5);
        Q_i(3, 3) = Q33;
        Q_i(4, 4) = a_prime_i^2 * Q33;
        Q_i(3, 4) = -a_prime_i * Q33;
        Q_i(4, 3) = -a_prime_i * Q33;
        Q_i(5, 5) = Q55_floor;
        Q44_v(ax) = Q_i(4, 4);

        % --- R: R1 sensor, R2 for the whitened y2 increment (isolated) ---
        R1_i = sigma2_n_s(ax);
        R2_i = compute_R2_whitened(a_h_i, sigma2_n_s(ax), IF_abc, C_dpmr, C_n, ...
                                   K_var, amlpf_var_factor, xi_per_axis(ax), kBT, ...
                                   Q_i(4, 4), delay_R2_factor, a_cov);

        % --- Gates (OR): warm-up / a_xm NaN guard / near wall ---
        G1 = (t_now < t_warmup_kf);
        G2 = ((sigma2_dxr_hat_new(ax) - C_n * sigma2_n_s(ax)) <= 0);
        G3 = (h_bar < h_bar_safe);
        G_flags(:, ax) = [G1; G2; G3];
        gate_off(ax) = G1 || G2 || G3;

        % --- F_dh (col-4 sensitivity) and F_e (5x5) ---
        F_dh = F_dh_det(ax);      % EXOGENOUS regressor (see [2b])
        F_e = local_build_F_e(lambda_c, F_dh, s_h, s_a, s_h_over_p, ...
                              Delta_hbar_d, R_radius);

        % --- EKF predict: deterministic map (slot 4 swept by estimated slope) ---
        x_pred = [x_curr(2); ...
                  x_curr(3); ...
                  lambda_c * x_curr(3); ...
                  x_curr(4) + s_h * Delta_hbar_d + one_minus_lc * a_prime_i * x_curr(3); ...
                  x_curr(5)];
        P_pred = F_e * P_curr * F_e' + Q_i;
        P_pred = 0.5 * (P_pred + P_pred');

        % --- Sequential 1-D measurement updates (R diagonal => exact) ---
        freeze_gain = G1;   % during KF warm-up, hold gain/exponent

        % (a) y1 = delta_h_m observes delta_h_1
        H1 = [1 0 0 0 0];
        S1 = H1 * P_pred * H1' + R1_i;
        K1 = (P_pred * H1') / S1;
        if freeze_gain; K1(4:5) = 0; end
        innov1 = delta_x_m(ax) - H1 * x_pred;
        x_upd  = x_pred + K1 * innov1;
        ImKH1  = I5 - K1 * H1;
        P_upd  = ImKH1 * P_pred * ImKH1' + K1 * R1_i * K1';   % Joseph form
        P_upd  = 0.5 * (P_upd + P_upd');
        K_dx_y1_v(ax) = K1(3);

        % (b) y2 = whitened a_xm increment, observes a_cov*a_h[k-d];
        %     skipped when gated (KF ignores y_2). a_xm_km1 is still shifted every
        %     step below, so the increment stays valid when the gate reopens.
        if ~gate_off(ax)
            H25 = -Delta_Hbar_d * s_h_over_p;               % H(2,5)/a_cov
            H2  = a_cov * [0 0 0 1 H25];
            S2  = H2 * P_upd * H2' + R2_i;
            K2  = (P_upd * H2') / S2;
            if freeze_gain; K2(4:5) = 0; end
            innov2 = a_meas(ax) - H2 * x_upd;
            x_upd  = x_upd + K2 * innov2;
            ImKH2  = I5 - K2 * H2;
            P_upd  = ImKH2 * P_upd * ImKH2' + K2 * R2_i * K2';   % Joseph form
            P_upd  = 0.5 * (P_upd + P_upd');
            K_a_y2_v(ax)   = K2(4);
            innov_y2_v(ax) = innov2;
        end

        % keep exponent from wandering to non-physical values (numerical guard)
        x_upd(5) = min(max(x_upd(5), 0), 5);

        x_e_per_axis(:, ax) = x_upd;
        P_per_axis{ax} = P_upd;
    end

    % ------------------------------------------------------------------
    % [4] Bookkeeping: shift delay buffers, IIR states, step counter
    % ------------------------------------------------------------------
    pd_km2 = pd_km1; pd_km1 = pd;
    f_d_km2 = f_d_km1; f_d_km1 = f_d;
    fdet_km2 = fdet_km1; fdet_km1 = f_det;
    a_ctrl_km2 = a_ctrl_km1; a_ctrl_km1 = a_ctrl;
    dx_bar_m = dx_bar_m_new;
    sigma2_dxr_hat = sigma2_dxr_hat_new;
    a_xm_km1 = a_xm;
    k_step = k_step + 1;

    % ------------------------------------------------------------------
    % [5] Output
    % ------------------------------------------------------------------
    a_hat_post = x_e_per_axis(4, :).';
    p_hat_post = x_e_per_axis(5, :).';
    h_bar_now  = local_h_bar_out(enable_wall, h_bar);
    ekf_out = [a_hat_post(1); a_hat_post(3); a_hat_post(2); h_bar_now];

    if nargout >= 3
        P_a_v = zeros(3, 1); P_dx_v = zeros(3, 1); P_dx1_v = zeros(3, 1);
        P_p_v = zeros(3, 1);
        for ax = 1:3
            P_a_v(ax)   = P_per_axis{ax}(4, 4);
            P_dx_v(ax)  = P_per_axis{ax}(3, 3);
            P_dx1_v(ax) = P_per_axis{ax}(1, 1);
            P_p_v(ax)   = P_per_axis{ax}(5, 5);
        end
        diag = empty_diag_powerlaw();
        diag.sigma2_dxr_hat = sigma2_dxr_hat_new;
        diag.a_xm           = a_xm;
        diag.delta_x_m      = delta_x_m;
        diag.innovation_y2  = innov_y2_v;
        diag.K_kf_a_y2      = K_a_y2_v;
        diag.K_kf_dx_y1     = K_dx_y1_v;
        diag.P_a            = P_a_v;
        diag.P_dx           = P_dx_v;
        diag.x_D_hat        = zeros(3, 1);        % no disturbance state (driver compat)
        diag.p_hat          = p_hat_post;         % exponent estimate (slot 5)
        diag.delta_a_hat    = p_hat_post;         % driver-log alias (slot 5)
        diag.a_prime_hat    = a_prime_v;          % da_h/dh implied by the power law
        diag.gate_active_per_axis = gate_off;
        diag.guards_individual    = G_flags;
        diag.h_bar          = h_bar;
        diag.f_d            = f_d;
        diag.dx_r           = dx_r;
        diag.a_hat          = a_hat_post;
        diag.a_ctrl_used    = a_ctrl;
        diag.P77            = P_p_v;              % exponent posterior variance
        diag.Q77            = Q44_v;             % Q44 gain process noise used
        diag.var_da_ram     = Q44_v;
        diag.Delta_h_d      = Delta_h_d;
        diag.Delta_H_d      = Delta_H_d;
        diag.delta_x_hat_1  = x_e_per_axis(1, :).';
        diag.delta_x_hat_3  = x_e_per_axis(3, :).';
        diag.P_dx1          = P_dx1_v;
    end
end


%% =================== Local Helpers ===================

function [s_h, s_a, s_h_over_p] = local_slope(a_h, p, h_bar_d, a_o, enable_wall)
%LOCAL_SLOPE  Power-law slope and its Jacobians (5state_powerlaw_hd.tex).
%   s_h        = (p/(h_bar_d-1)) * a_h*(a_o-a_h)/a_o        (da_h/d h_bar)
%   s_a        = d s_h / d a_h = (p/(h_bar_d-1))*(a_o-2 a_h)/a_o
%   s_h_over_p = d s_h / d p   = (1/(h_bar_d-1))*a_h*(a_o-a_h)/a_o   (p-free)
%   Far from the wall (or wall off) the slope vanishes: gain stays a_o.
    if ~enable_wall || ~isfinite(h_bar_d)
        s_h = 0; s_a = 0; s_h_over_p = 0;
        return;
    end
    denom = h_bar_d - 1;
    denom = sign_nonzero(denom) * max(abs(denom), 0.05);   % guard h_bar_d -> 1
    logistic   = a_h * (a_o - a_h) / a_o;                  % a_h*(a_o-a_h)/a_o
    s_h_over_p = logistic / denom;
    s_h        = p * s_h_over_p;
    s_a        = (p / denom) * (a_o - 2 * a_h) / a_o;
end


function s = sign_nonzero(x)
%SIGN_NONZERO  sign(x) but returns +1 for x==0 (keep the guard positive).
    if x < 0
        s = -1;
    else
        s = 1;
    end
end


function F_e = local_build_F_e(lambda_c, F_dh, s_h, s_a, s_h_over_p, Delta_hbar_d, R_radius)
%LOCAL_BUILD_F_E  5x5 error-dynamics Jacobian (boxed 5x5 F_e in the spec).
%   cols: 1=dh1 2=dh2 3=dh3 4=a_h 5=p
%   Row 3 = [0 0 lc      -F_dh                         0]
%   Row 4 = [0 0 (1-lc)s_h/R   1+Dhbar*s_a+(s_h/R)F_dh   Dhbar*(s_h/p)]
%   Row 5 = [0 0 0        0                             1]
    one_minus_lc = 1 - lambda_c;
    a_prime = s_h / R_radius;
    Fe34 = -F_dh;
    Fe43 = one_minus_lc * a_prime;
    Fe44 = 1 + Delta_hbar_d * s_a + a_prime * F_dh;
    Fe45 = Delta_hbar_d * s_h_over_p;
    F_e = [0 1 0        0     0; ...
           0 0 1        0     0; ...
           0 0 lambda_c Fe34  0; ...
           0 0 Fe43     Fe44  Fe45; ...
           0 0 0        0     1];
end


function R2 = compute_R2_whitened(a_h, sigma2_n, IF_abc, C_dpmr, C_n, ...
                                 K_var, amlpf_var_factor, xi, kBT, Q44, ...
                                 delay_factor, a_cov)
%COMPUTE_R2_WHITENED  R(2,2) for the WHITENED a_xm increment y2 = a_cov*u[k].
%   Spec: 5state_powerlaw_hd.tex, "Measurement noise R".
%
%   R2_intrinsic is the eq17 7-state / 5state_aprime IIR chi-squared chain
%   (a_x -> a_h) and equals the MARGINAL variance of a_xm's error:
%       R2_intrinsic = amlpf_var_factor * K_var * IF_eff(a_h) * (a_h + xi)^2
%   with K_var = 2*a_cov/(2-a_cov). It already carries u's own residual colour
%   through IF_eff, so the only conversion needed is marginal -> whitened.
%
%   For the AR(1) x[k] = (1-a_cov)*x[k-1] + a_cov*u[k] the stationary variance is
%       Var(x) = a_cov*Var(u)/(2-a_cov)   =>   Var(a_cov*u) = a_cov*(2-a_cov)*Var(x)
%   The d-step q4 accumulation (7-state "5*Q77" structure with Q77 -> Q44) rides
%   on y2 = a_cov*u and so picks up a_cov^2.
%
%   Feeding a_xm directly instead would over-state the per-sample information by
%   (2-a_cov)/a_cov (=39 at a_cov=0.05; measured 33.6 from the a_xm error ACF,
%   fitted pole 0.9457 vs 1-a_cov=0.95).
    IF_eff = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a_h, sigma2_n);
    R2_intrinsic = amlpf_var_factor * K_var * IF_eff * (a_h + xi)^2;
    R2 = a_cov * (2 - a_cov) * R2_intrinsic + a_cov^2 * delay_factor * Q44;
end


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


function d = empty_diag_powerlaw()
%EMPTY_DIAG_POWERLAW  Zeroed diagnostic struct (driver collect_diag compatible).
    d = struct();
    d.sigma2_dxr_hat    = zeros(3, 1);
    d.a_xm              = zeros(3, 1);
    d.delta_x_m         = zeros(3, 1);
    d.innovation_y2     = zeros(3, 1);
    d.K_kf_a_y2         = zeros(3, 1);
    d.K_kf_dx_y1        = zeros(3, 1);
    d.P_a               = zeros(3, 1);
    d.P_dx              = zeros(3, 1);
    d.x_D_hat           = zeros(3, 1);
    d.p_hat             = ones(3, 1);
    d.delta_a_hat       = ones(3, 1);
    d.a_prime_hat       = zeros(3, 1);
    d.gate_active_per_axis = false(3, 1);
    d.guards_individual    = false(3, 3);
    d.h_bar             = 0;
    d.f_d               = zeros(3, 1);
    d.dx_r              = zeros(3, 1);
    d.a_hat             = zeros(3, 1);
    d.a_ctrl_used       = zeros(3, 1);
    d.P77               = zeros(3, 1);
    d.Q77               = zeros(3, 1);
    d.var_da_ram        = zeros(3, 1);
    d.Delta_h_d         = 0;
    d.Delta_H_d         = 0;
    d.delta_x_hat_1     = zeros(3, 1);
    d.delta_x_hat_3     = zeros(3, 1);
    d.P_dx1             = zeros(3, 1);
end
