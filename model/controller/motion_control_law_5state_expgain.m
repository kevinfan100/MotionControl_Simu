function [f_d, ekf_out, diag] = motion_control_law_5state_expgain(del_pd, pd, p_m, params, ctrl_const, a_ctrl_override)
%MOTION_CONTROL_LAW_5STATE_EXPGAIN  Per-axis 5-state EKF eq17 controller whose
%   motion gain follows a SATURATING EXPONENTIAL law
%
%       a_h(h_bar) = a_o * [1 - exp(-b*phi(h_bar))] ,   phi(h_bar) = ln(h_bar)
%                  = a_o * [1 - h_bar^(-b)]
%
%   Authoritative spec:
%   reference/eq17_analysis/derivation/5state_expgain_hd.tex
%
%   Sibling: motion_control_law_5state_powerlaw (same skeleton, power-law gain
%   in (h_bar-1); this one replaces the gain law and the second sub-state).
%
%   FULLY NON-DIMENSIONAL INTERNALS. Every length is divided by the probe
%   radius R, exactly as in the spec, so each F_e / H / Q / R entry is the
%   boxed formula verbatim. R survives in three defining relations only:
%
%       h_bar = h/R ,   a_o = Ts/(gamma_N*R) [1/pN] ,   4*kB*T/R [pN]
%
%   The unit boundary is crossed in four places, all of them "divide by R once"
%   (marked [U1]..[U4] in the body):
%       [U1] inputs pd / p_m / del_pd  [um]      -> /R
%       [U2] sigma2_n_s               [um^2]     -> /R^2
%       [U3] thermal scale 4*kB*T     [pN*um]    -> /R
%       [U4] outputs a_hat            [1/pN]     -> *R  (report as um/pN)
%   Forces stay in pN and are NOT scaled; f_d is returned in pN as usual.
%
%   Per-axis (x, y, z; z = wall-normal) state, d = 2 measurement delay:
%
%       x = [dh_1; dh_2; dh_3; a_h; b]
%
%   dh_1 = delta_h[k-2] (measured), dh_3 = delta_h[k] (current), all in units
%   of R; a_h = motion gain [1/pN]; b = the exponent (a state, constant model).
%
%   Gain model (shape function phi isolated behind B, per the spec):
%       B[k]        = b * phi'(h_bar_d[k]) = b / h_bar_d[k]
%       a_h'[k]     = B[k] * (a_o - a_h[k])              (d a_h / d h_bar)
%       d a_h'/d a_h = -B[k]
%       d a_h'/d b   = a_h'/b = (a_o - a_h)/h_bar_d      (b-free form used)
%
%   Estimator PREDICT (slot 4 swept by the step the estimator knows the TRUE
%   position takes; slot 5 constant):
%       a_h[k+1] = a_h + a_h' * ( Delta_hbar_d + (1-lc)*dh_3 )
%       b[k+1]   = b
%
%   Control law (eq17 implementable, no disturbance term), non-dimensional:
%       f_dh[k] = a_ctrl^-1 { Delta_hbar_d^d[k]
%                             + (1-lc)[dh_m[k] - sum_i a_ctrl[k-i] f_dh[k-i]] }
%
%   Measurements. a_hm is NOT a white measurement of a_h: it is an EXACT AR(1)
%   filter of the single-sample gain readout u[k] with pole 1-a_cov,
%       a_hm[k] = (1-a_cov)*a_hm[k-1] + a_cov*u[k],   E[u[k]] = a_h[k-d],
%   so feeding a_hm itself over-states its information by (2-a_cov)/a_cov
%   (= 39 at a_cov = 0.05). The KF is fed the WHITENED increment by default:
%       y1 = dh_m = dh_1 + n_h
%       y2 = a_hm[k] - (1-a_cov)*a_hm[k-1] = a_cov*u[k]
%       H  = [1 0 0    0                0                   ;
%             0 0 0  a_cov  -a_cov*Grad_Hbar_d*(a_o-a_h)/h_bar_d]
%   Set ctrl_const.y2_whiten = false to feed the raw a_hm instead (the form the
%   spec body carries); the acceptance test for the choice is invariance of the
%   results to a_cov.
%
%   F_e (5x5) uses the DETERMINISTIC control mirror F_dh_det (dh_m := 0), not
%   the realised f_d -- see [2b]. The row-4 multiplier is the full
%   Delta_hbar_d + (1-lc)*dh_3_hat, i.e. the deterministic part of the step the
%   TRUE position takes; dropping the second term would zero F_e(4,5) at every
%   trajectory turning point, and F_e(4,5) is b's main identification channel
%   (y2's is only ~0.0075/sample). Set ctrl_const.fe_row4_full = false to drop
%   it (the power-law sibling's convention) for A/B.
%
%   Q (Path C strict, rank-1 gain block + Q33), non-dimensional:
%       Q33 = (4*kB*T/R)*a_h ,  Q44 = a_h'^2*Q33 ,  Q34 = Q43 = -a_h'*Q33 ,
%       Q55 = 0  (b constant).
%   R = diag(R1, R2):
%       R1 = sigma2_n_h  (= sigma2_n_s/R^2),
%       R2 = compute_R2_expgain(...).
%
%   INIT -- no c(h_bar) anywhere. The level state and the exponent state are
%   seeded from the SAME published method-of-reflections leading term, but they
%   answer different questions, so they are anchored separately.
%
%   LEVEL. The far-field asymptote is a_h = a_o*[1 - K/h_bar], K = 9/8 (perp)
%   and 9/16 (parallel):
%
%       a_h[0] = a_o*[1 - K/h_bar_0]
%
%   This is ~50x more accurate than a_o/(1 + K/h_bar_0): a_h = a_o*D(h_bar) and
%   D IS the reflection polynomial, whose expansion has no h_bar^-2 term, while
%   inverting c manufactures a spurious K^2/h_bar^2. Verified against the exact
%   curve: -0.0007% at h_bar_0 = 22.2 (vs +0.25% for the inverted form).
%
%   EXPONENT. b enters the filter ONLY through the slope a_h' -- the level is
%   carried by the a_h state -- so b must be anchored on the slope, not on the
%   level. Both known regimes then give the same value:
%
%       far field  a_h = a_o[1 - K/h_bar]  =>  a_h' = a_o*K/h_bar^2
%                  model a_h' = (b/h_bar)(a_o - a_h) = b*a_o*K/h_bar^2  =>  b = 1
%       near wall  D_perp -> (h_bar - 1)   =>  a_h' -> a_o
%                  model a_h' -> b*a_o                                  =>  b = 1
%
%   so  b[0] = 1, INDEPENDENT of K and therefore the same on all three axes --
%   the exact analogue of the power-law sibling's p = 1 (Brenner). Checked
%   against the exact slope at h_bar_0 = 22.2: b = 1 reproduces a_h' to +0.25%.
%   (Anchoring b on the LEVEL instead would give 1 - ln(K)/ln(h_bar_0) = 0.962
%   and miss the slope by -3.6%; the two effective exponents differ because the
%   true curve is not exactly of this form.)
%
%   Neither seed reads c(h_bar); both use only K and the wall position.
%
%   Optional ctrl_const knobs:
%       .y2_whiten     whitened y2 increment                (default true)
%       .fe_row4_full  full row-4 multiplier                (default true)
%       .use_fdet      deterministic F_e regressor          (default true)
%       .y2_off        drop the gain-readout channel        (default false)
%       .y1_gain_off   let y1 update positions only         (default false)
%                      -- the 2x2 of these two answers "which measurement is
%                      the gain's level anchor", which cannot be read off the
%                      Kalman gains alone
%       .Q55_floor     tiny floor on Q(5,5) for conditioning(default 0)
%       .Pf_a_frac     sqrt(Pf(4,4))/a_h init frac          (default 2/h_bar_0^3,
%                      floored at 0.002: the seed's own residual x4)
%       .Pf_b_std      sqrt(Pf(5,5)) init exponent std      (default 0.10, the
%                      interpolation dip between the two b = 1 anchors)
%       .b_init        initial exponent                     (default 1)
%       .a_init_mode   'asym_model' (default) | 'flat'      ('flat' = a_o, used
%                      to test whether the filter can FIND a_h rather than just
%                      not lose it)
%       .h_bar_min_traj  trajectory trough, for Pf_b_std    (default params.traj)
%
%   a_ctrl_override (3x1, um/pN, optional): feed a chosen gain to the CONTROL
%   LAW only; the EKF still estimates a_h / b.
%
%   See also: motion_control_law_5state_powerlaw, build_eq17_6state_constants

    if nargin < 6
        a_ctrl_override = [];
    end
    has_override = ~isempty(a_ctrl_override);
    if has_override
        a_ctrl_override = a_ctrl_override(:);
        assert(numel(a_ctrl_override) == 3 && all(isfinite(a_ctrl_override)) && all(a_ctrl_override > 0), ...
               'motion_control_law_5state_expgain:badOverride', ...
               'a_ctrl_override must be a 3x1 finite positive vector [um/pN].');
    end

    % ------------------------------------------------------------------
    % Open-loop bypass
    % ------------------------------------------------------------------
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = [1; 1; 1; 0];
        if nargout >= 3
            diag = empty_diag_expgain();
            diag.f_d = f_d;
        end
        return;
    end

    % ------------------------------------------------------------------
    % Persistent state  (all EKF/IIR quantities NON-DIMENSIONAL)
    % ------------------------------------------------------------------
    persistent x_e_per_axis        % 5x3 EKF state (col = axis); slot 4=a_h [1/pN], slot 5=b
    persistent P_per_axis          % cell{3} of 5x5 covariance
    persistent dh_bar_m            % 3x1 IIR LP mean of dh_m   [-]
    persistent sigma2_dhr_hat      % 3x1 EWMA variance of dh_r [-]
    persistent a_hm_km1            % 3x1 a_hm[k-1], for the whitening increment
    persistent fdet_km1 fdet_km2   % noise-free control mirror, own history [pN]
    persistent pd_km1 pd_km2       % trajectory delay buffers [um]
    persistent f_d_km1 f_d_km2     % past control buffers [pN]
    persistent a_ctrl_km1 a_ctrl_km2        % control-law gain history [1/pN]
    persistent k_step

    persistent initialized
    persistent lambda_c d_delay Ts kBT_over_R R_radius a_o
    persistent a_pd a_cov C_dpmr C_n K_var IF_abc xi_nd amlpf_var_factor
    persistent t_warmup_kf h_bar_safe sigma2_n_nd
    persistent enable_wall w_hat_n pz_wall
    persistent Q55_floor a_floor a_cap delay_R2_factor
    persistent y2_whiten fe_row4_full use_fdet y2_off y1_gain_off

    % ------------------------------------------------------------------
    % [0] Initialization on first call
    % ------------------------------------------------------------------
    if isempty(initialized)
        initialized = true;

        % --- 0A. params constants; the unit boundary is crossed HERE ---
        Ts        = params.ctrl.Ts;
        R_radius  = params.common.R;                     % [um]
        kBT       = params.ctrl.k_B * params.ctrl.T;     % [pN*um]
        kBT_over_R = kBT / R_radius;                     % [pN]        [U3]
        gamma_N_p = params.ctrl.gamma;                   % [pN*sec/um]
        sigma2_n_nd = params.ctrl.sigma2_noise(:) / R_radius^2;   % [-]  [U2]

        % a_o = far-field NON-DIMENSIONAL gain [1/pN]
        a_o = Ts / (gamma_N_p * R_radius);

        % --- 0B. ctrl_const (offline scalars; shared with the 5/6-state family) ---
        lambda_c        = ctrl_const.lambda_c;
        d_delay         = ctrl_const.d;
        C_dpmr          = ctrl_const.C_dpmr;
        C_n             = ctrl_const.C_n;
        K_var           = ctrl_const.K_var;
        IF_abc          = ctrl_const.IF_abc(:);
        t_warmup_kf     = ctrl_const.t_warmup_kf;
        h_bar_safe      = ctrl_const.h_bar_safe;
        a_cov           = ctrl_const.a_cov;
        a_pd            = ctrl_const.a_pd;
        amlpf_var_factor = get_field_default(ctrl_const, 'amlpf_var_factor', 1);
        % xi = (C_n/C_dpmr)*sigma2_n/(4kBT) is a gain, so it scales like a_h: /R
        xi_nd = ctrl_const.xi_per_axis(:) / R_radius;

        y2_whiten    = logical(get_field_default(ctrl_const, 'y2_whiten', true));
        fe_row4_full = logical(get_field_default(ctrl_const, 'fe_row4_full', true));
        use_fdet     = logical(get_field_default(ctrl_const, 'use_fdet', true));
        % channel ablation (2x2 "which measurement anchors the gain"): zero the
        % gain/exponent rows of the corresponding Kalman gain, leaving the
        % position states updated as usual.
        y2_off       = logical(get_field_default(ctrl_const, 'y2_off', false));
        y1_gain_off  = logical(get_field_default(ctrl_const, 'y1_gain_off', false));
        Q55_floor    = get_field_default(ctrl_const, 'Q55_floor', 0);

        % d-step delay R2 factor: sum_{j=1}^d (d-j+1)^2  (= 5 for d=2)
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

        % --- 0D. Seed (a_h[0], b_0) from the far-field reflection asymptote.
        %   c(h_bar) is NEVER read. See the header for the derivation; the pair
        %   sits exactly on the model manifold because
        %       b_0 = 1 - ln(K)/ln(h_bar_0)  =>  h_bar_0^(-b_0) == K/h_bar_0.
        K_refl = [9/16; 9/16; 9/8];        % method-of-reflections leading term
        b_default = ones(3, 1);            % see the header: BOTH anchors give b = 1
        h_bar_init = Inf;
        if enable_wall && isfield(params, 'common') && isfield(params.common, 'p0')
            p0_init    = params.common.p0(:);
            h_init_um  = dot(p0_init, w_hat_n) - pz_wall;
            h_bar_init = max(h_init_um / R_radius, 1.001);
            a_h_init   = a_o * (1 - K_refl / h_bar_init);
        else
            a_h_init   = a_o * ones(3, 1);      % no wall: a_h = a_o exactly
        end
        b_init = get_field_default(ctrl_const, 'b_init', b_default);
        if isscalar(b_init); b_init = b_init * ones(3, 1); end
        b_init = b_init(:);

        a_init_mode = get_field_default(ctrl_const, 'a_init_mode', 'asym_model');
        if strcmpi(a_init_mode, 'flat')
            % Deliberately WRONG seed (ignores the wall entirely). Used to ask
            % whether the filter can FIND a_h, which a correct seed cannot test.
            a_h_init = a_o * ones(3, 1);
        end

        % Pf_a_frac matches the seed's own residual (4x margin), it is not
        % loosened to buy correction room: a mismatched prior is what breaks,
        % in either direction. The asymptotic seed's residual is set by the
        % first neglected term of D (the u^3 term), ~ 0.5/h_bar_0^3.
        if isfinite(h_bar_init)
            Pf_a_default = min(max(2 / h_bar_init^3, 0.002), 0.3);
        else
            Pf_a_default = 0.002;
        end
        if strcmpi(a_init_mode, 'flat') && isfinite(h_bar_init)
            % The flat seed misses by the whole wall correction, K/h_bar_0,
            % and that magnitude is KNOWN even though its value is not -- so
            % the matched prior is 1x it, not the 4x margin used above for the
            % asymptotic seed (whose residual is only bounded, not known).
            % An over-loose prior here lets the noisy gain readout overshoot.
            Pf_a_default = min(max(max(K_refl) / h_bar_init, 0.002), 0.5);
        end
        Pf_a_frac = get_field_default(ctrl_const, 'Pf_a_frac', Pf_a_default);

        % Pf_b_std covers only the interpolation dip between the two b = 1
        % anchors. Evaluated offline once on the exact correction curves
        % (the controller carries the number, never the curve):
        %       max|b_slope - 1|      perp            para
        %       whole domain          0.094 @ 1.42    0.732 @ 1.02 (Goldman log)
        %       h_bar >= 2            0.082           0.034
        % 0.10 covers perp globally and para over the working range with
        % margin. Parallel motion below h_bar ~ 1.5 is a structural failure of
        % any power/exponential law, not a prior-width problem.
        Pf_b_default = 0.10 * ones(3, 1);
        Pf_b_std = get_field_default(ctrl_const, 'Pf_b_std', Pf_b_default);
        if isscalar(Pf_b_std); Pf_b_std = Pf_b_std * ones(3, 1); end
        Pf_b_std = Pf_b_std(:);

        % Numerical guards only (physically a_h stays in (0, a_o)).
        a_floor = 0.05 * a_o;
        a_cap   = 0.9999 * a_o;         % keeps a_o - a_h > 0 in the slope

        % --- 0E. EKF state init ---
        x_e_per_axis = zeros(5, 3);
        x_e_per_axis(4, :) = min(max(a_h_init, a_floor), a_cap).';
        x_e_per_axis(5, :) = b_init.';

        % --- 0F. Pf init: DARE on the observable 3-block, matched priors on 4/5 ---
        F3 = [0 1 0; 0 0 1; 0 0 lambda_c];
        H3 = [1 0 0];
        P_per_axis = cell(3, 1);
        for ax = 1:3
            a_init_ax = x_e_per_axis(4, ax);
            Q3 = zeros(3); Q3(3, 3) = kBT_over_R * 4 * a_init_ax;
            P3 = solve_dare_kf_local(F3, H3, Q3, sigma2_n_nd(ax));
            P5 = zeros(5);
            P5(1:3, 1:3) = P3;
            P5(4, 4) = (Pf_a_frac * a_init_ax)^2;
            P5(5, 5) = Pf_b_std(ax)^2;
            P_per_axis{ax} = P5;
        end

        % --- 0G. IIR states (prefill to the closed-loop dh_r variance) ---
        dh_bar_m = zeros(3, 1);
        sigma2_dhr_hat = 4 * kBT_over_R * x_e_per_axis(4, :).' * C_dpmr + C_n * sigma2_n_nd;
        % the prefill is exactly a_hm[0] = a_h[0], so seed the whitening delay
        % slot with it (the first increment is then noise-only, not a step).
        a_hm_km1 = x_e_per_axis(4, :).';

        % --- 0H. Delay buffers ---
        pd_km1  = pd;
        pd_km2  = pd;
        f_d_km1 = zeros(3, 1);
        f_d_km2 = zeros(3, 1);
        fdet_km1 = zeros(3, 1);
        fdet_km2 = zeros(3, 1);
        a_ctrl_km1 = x_e_per_axis(4, :).';
        a_ctrl_km2 = a_ctrl_km1;
        k_step = 1;

        % --- 0I. First call returns zeros (no f_d yet) ---
        f_d = zeros(3, 1);
        a_hat_phys = x_e_per_axis(4, :).' * R_radius;      % [U4]
        ekf_out = [a_hat_phys(1); a_hat_phys(3); a_hat_phys(2); 0];
        if nargout >= 3
            diag = empty_diag_expgain();
            diag.f_d      = f_d;
            diag.a_hat    = a_hat_phys;
            diag.a_hat_nd = x_e_per_axis(4, :).';
            diag.b_hat    = b_init;
            diag.p_hat    = b_init;                 % driver-log alias
            diag.delta_a_hat = b_init;              % driver-log alias
            diag.sigma2_dxr_hat = sigma2_dhr_hat;
            diag.P_a  = (Pf_a_frac * x_e_per_axis(4, :).').^2 * R_radius^2;
            diag.P77  = Pf_b_std.^2;
            diag.h_bar_init = h_bar_init;
            diag.Pf_a_frac  = Pf_a_frac;
            diag.Pf_b_std   = Pf_b_std;
            if has_override
                diag.a_ctrl_used = a_ctrl_override;
            else
                diag.a_ctrl_used = a_hat_phys;
            end
        end
        return;
    end

    % ------------------------------------------------------------------
    % Per-step: extract per-axis state
    % ------------------------------------------------------------------
    a_hat_nd = x_e_per_axis(4, :).';     % 3x1 [1/pN]

    if has_override
        a_ctrl = a_ctrl_override / R_radius;    % um/pN -> 1/pN
    else
        a_ctrl = a_hat_nd;
    end
    a_ctrl = min(max(a_ctrl, a_floor), a_cap);

    % --- [U1] every length crossing the boundary is divided by R here ---
    if d_delay == 2
        pd_km_d = pd_km2;
    elseif d_delay == 1
        pd_km_d = pd_km1;
    else
        error('motion_control_law_5state_expgain:unsupportedDelay', ...
              'Only d=1 or d=2 supported, got d=%g.', d_delay);
    end
    delta_h_m = (pd_km_d - p_m) / R_radius;      % 3x1 dh_m[k]  [-]
    pd_kp1    = pd + del_pd;                     % 3x1 [um]

    if enable_wall
        h_bar = (dot(p_m, w_hat_n) - pz_wall) / R_radius;
    else
        h_bar = Inf;
    end

    one_minus_lc = 1 - lambda_c;

    % ------------------------------------------------------------------
    % Known desired-trajectory height increments (wall-normal, dimensionless).
    %   Delta_hbar_d[k] = (h_d[k+1]-h_d[k])/R          (predict / F_e)
    %   Grad_Hbar_d[k]  = (h_d[k]-h_d[k-d])/R          (y2 delay back-off)
    %   h_bar_d         = h_d[k]/R                     (slope operating point)
    % All scalars (shared across axes); a_h / b are per-axis.
    % ------------------------------------------------------------------
    Delta_hbar_d = dot(del_pd, w_hat_n) / R_radius;
    Grad_Hbar_d  = dot(pd - pd_km_d, w_hat_n) / R_radius;
    if enable_wall
        h_bar_d = (dot(pd, w_hat_n) - pz_wall) / R_radius;
    else
        h_bar_d = Inf;
    end

    % ------------------------------------------------------------------
    % [1] IIR gain readout a_hm (paper 2025 Eq.9-13), non-dimensional.
    % ------------------------------------------------------------------
    dh_bar_m_new = (1 - a_pd) * dh_bar_m + a_pd * delta_h_m;
    dh_r = delta_h_m - dh_bar_m_new;
    sigma2_dhr_hat_new = (1 - a_cov) * sigma2_dhr_hat + a_cov * dh_r.^2;
    a_hm = (sigma2_dhr_hat_new - C_n * sigma2_n_nd) / (C_dpmr * 4 * kBT_over_R);   % [1/pN]
    if y2_whiten
        y2 = a_hm - (1 - a_cov) * a_hm_km1;      % = a_cov*u[k]
        H2_scale = a_cov;
    else
        y2 = a_hm;                               % raw AR(1) output (spec body)
        H2_scale = 1;
    end

    % ------------------------------------------------------------------
    % [2] Control law (eq17 implementable; NO disturbance term), dimensionless
    %   f_dh = a_ctrl^-1 { Dhbar_d^d[k] + (1-lc)[dh_m - sum a_ctrl[k-i] f_dh[k-i]] }
    % ------------------------------------------------------------------
    if d_delay == 2
        sum_a_fd_past = a_ctrl_km1 .* f_d_km1 + a_ctrl_km2 .* f_d_km2;
    else
        sum_a_fd_past = a_ctrl_km1 .* f_d_km1;
    end
    inv_a_ctrl = 1 ./ a_ctrl;
    traj_term = (pd_kp1 - lambda_c * pd - one_minus_lc * pd_km_d) / R_radius;
    f_d = inv_a_ctrl .* (traj_term ...
                         + one_minus_lc * delta_h_m ...
                         - one_minus_lc * sum_a_fd_past);

    % ------------------------------------------------------------------
    % [2b] Deterministic mirror of the control law (dh_m := 0, running on its
    %   OWN history). Purely trajectory-driven, hence EXOGENOUS.
    %   F_e must use THIS, not the realised f_d: the error dynamics
    %       dh[k+1] = lc*dh - F_dh*e_ah - eps_h
    %   is an identity with the realised force, but handing that F_dh to the KF
    %   makes F_e a random matrix sharing eps_h's thermal noise. P and K then
    %   become innovation-correlated and E[K*innov] != 0 -- a RECTIFICATION that
    %   is one-signed regardless of the noise sign, so it accumulates instead of
    %   averaging out. On the power-law sibling this dragged a_hat down by 24.3%
    %   of a_true over 12 s of pure positioning, monotonically and without
    %   settling. The multiplicative noise dropped here belongs in Q33 and is
    %   three orders of magnitude below it, so it is not added.
    % ------------------------------------------------------------------
    if d_delay == 2
        sum_af_det = a_ctrl_km1 .* fdet_km1 + a_ctrl_km2 .* fdet_km2;
    else
        sum_af_det = a_ctrl_km1 .* fdet_km1;
    end
    f_det = inv_a_ctrl .* (traj_term - one_minus_lc * sum_af_det);
    if d_delay == 2
        F_dh_det = f_det + one_minus_lc * (fdet_km1 + fdet_km2);
        F_dh_raw = f_d   + one_minus_lc * (f_d_km1 + f_d_km2);
    else
        F_dh_det = f_det + one_minus_lc * fdet_km1;
        F_dh_raw = f_d   + one_minus_lc * f_d_km1;
    end
    if use_fdet
        F_dh_vec = F_dh_det;
    else
        F_dh_vec = F_dh_raw;      % ablation only: reproduces the rectification
    end

    % ------------------------------------------------------------------
    % [3] Per-axis slope, Q, R, gate; EKF predict + sequential 1-D updates.
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
    R2_v       = zeros(3, 1);

    for ax = 1:3
        x_curr = x_e_per_axis(:, ax);
        P_curr = P_per_axis{ax};
        a_h_i  = min(max(x_curr(4), a_floor), a_cap);
        b_i    = x_curr(5);

        % --- Exponential-gain slope and its Jacobians (spec, Gain model) ---
        [a_prime_i, dap_da, dap_db] = local_gain_slope(a_h_i, b_i, h_bar_d, a_o, enable_wall);
        a_prime_v(ax) = a_prime_i;

        % --- Q (5x5): Path C strict, rank-1 gain block + Q33 ---
        Q33 = 4 * kBT_over_R * a_h_i;
        Q_i = zeros(5);
        Q_i(3, 3) = Q33;
        Q_i(4, 4) = a_prime_i^2 * Q33;
        Q_i(3, 4) = -a_prime_i * Q33;
        Q_i(4, 3) = -a_prime_i * Q33;
        Q_i(5, 5) = Q55_floor;
        Q44_v(ax) = Q_i(4, 4);

        % --- R ---
        R1_i = sigma2_n_nd(ax);
        R2_i = compute_R2_expgain(a_h_i, sigma2_n_nd(ax), IF_abc, C_dpmr, C_n, ...
                                  K_var, amlpf_var_factor, xi_nd(ax), kBT_over_R, ...
                                  Q_i(4, 4), delay_R2_factor, a_cov, y2_whiten);
        R2_v(ax) = R2_i;

        % --- Gates (OR): warm-up / readout NaN guard / near wall ---
        G1 = (t_now < t_warmup_kf);
        G2 = ((sigma2_dhr_hat_new(ax) - C_n * sigma2_n_nd(ax)) <= 0);
        G3 = (h_bar < h_bar_safe);
        G_flags(:, ax) = [G1; G2; G3];
        gate_off(ax) = G1 || G2 || G3;

        % --- Row-4 multiplier: the deterministic part of the step the TRUE
        %     position takes. Dropping (1-lc)*dh_3_hat zeroes F_e(4,5) at every
        %     trajectory turning point, where Delta_hbar_d = 0.
        if fe_row4_full
            M_row4 = Delta_hbar_d + one_minus_lc * x_curr(3);
        else
            M_row4 = Delta_hbar_d;
        end

        F_dh = F_dh_vec(ax);
        F_e = local_build_F_e(lambda_c, F_dh, a_prime_i, dap_da, dap_db, M_row4);

        % --- EKF predict ---
        x_pred = [x_curr(2); ...
                  x_curr(3); ...
                  lambda_c * x_curr(3); ...
                  x_curr(4) + a_prime_i * (Delta_hbar_d + one_minus_lc * x_curr(3)); ...
                  x_curr(5)];
        P_pred = F_e * P_curr * F_e' + Q_i;
        P_pred = 0.5 * (P_pred + P_pred');

        % --- Sequential 1-D measurement updates (R diagonal => exact) ---
        freeze_gain = G1;

        % (a) y1 = dh_m observes dh_1
        H1 = [1 0 0 0 0];
        S1 = H1 * P_pred * H1' + R1_i;
        K1 = (P_pred * H1') / S1;
        if freeze_gain || y1_gain_off; K1(4:5) = 0; end
        innov1 = delta_h_m(ax) - H1 * x_pred;
        x_upd  = x_pred + K1 * innov1;
        ImKH1  = I5 - K1 * H1;
        P_upd  = ImKH1 * P_pred * ImKH1' + K1 * R1_i * K1';   % Joseph form
        P_upd  = 0.5 * (P_upd + P_upd');
        K_dx_y1_v(ax) = K1(3);

        % (b) y2 = gain readout (whitened increment by default).
        %     a_hm_km1 is shifted every step regardless, so the increment stays
        %     valid when the gate reopens.
        if ~gate_off(ax) && ~y2_off
            H25 = -Grad_Hbar_d * dap_db;
            H2  = H2_scale * [0 0 0 1 H25];
            S2  = H2 * P_upd * H2' + R2_i;
            K2  = (P_upd * H2') / S2;
            if freeze_gain; K2(4:5) = 0; end
            innov2 = y2(ax) - H2 * x_upd;
            x_upd  = x_upd + K2 * innov2;
            ImKH2  = I5 - K2 * H2;
            P_upd  = ImKH2 * P_upd * ImKH2' + K2 * R2_i * K2';   % Joseph form
            P_upd  = 0.5 * (P_upd + P_upd');
            K_a_y2_v(ax)   = K2(4);
            innov_y2_v(ax) = innov2;
        end

        % numerical guards (physical range, not tuning)
        x_upd(4) = min(max(x_upd(4), a_floor), a_cap);
        x_upd(5) = min(max(x_upd(5), 0.05), 5);

        x_e_per_axis(:, ax) = x_upd;
        P_per_axis{ax} = P_upd;
    end

    % ------------------------------------------------------------------
    % [4] Bookkeeping
    % ------------------------------------------------------------------
    pd_km2 = pd_km1; pd_km1 = pd;
    f_d_km2 = f_d_km1; f_d_km1 = f_d;
    fdet_km2 = fdet_km1; fdet_km1 = f_det;
    a_ctrl_km2 = a_ctrl_km1; a_ctrl_km1 = a_ctrl;
    dh_bar_m = dh_bar_m_new;
    sigma2_dhr_hat = sigma2_dhr_hat_new;
    a_hm_km1 = a_hm;
    k_step = k_step + 1;

    % ------------------------------------------------------------------
    % [5] Output   ([U4]: gains reported in um/pN for driver/analysis compat)
    % ------------------------------------------------------------------
    a_hat_nd_post = x_e_per_axis(4, :).';
    a_hat_post    = a_hat_nd_post * R_radius;
    b_hat_post    = x_e_per_axis(5, :).';
    h_bar_now     = local_h_bar_out(enable_wall, h_bar);
    ekf_out = [a_hat_post(1); a_hat_post(3); a_hat_post(2); h_bar_now];

    if nargout >= 3
        P_a_v = zeros(3, 1); P_dx_v = zeros(3, 1); P_dx1_v = zeros(3, 1);
        P_b_v = zeros(3, 1);
        for ax = 1:3
            P_a_v(ax)   = P_per_axis{ax}(4, 4);
            P_dx_v(ax)  = P_per_axis{ax}(3, 3);
            P_dx1_v(ax) = P_per_axis{ax}(1, 1);
            P_b_v(ax)   = P_per_axis{ax}(5, 5);
        end
        diag = empty_diag_expgain();
        diag.sigma2_dxr_hat = sigma2_dhr_hat_new;
        diag.a_xm           = a_hm * R_radius;    % report in um/pN
        diag.a_hm_nd        = a_hm;
        diag.y2             = y2;
        diag.delta_x_m      = delta_h_m * R_radius;
        diag.innovation_y2  = innov_y2_v;
        diag.K_kf_a_y2      = K_a_y2_v;
        diag.K_kf_dx_y1     = K_dx_y1_v;
        diag.P_a            = P_a_v * R_radius^2;     % (um/pN)^2
        diag.P_a_nd         = P_a_v;
        diag.P_dx           = P_dx_v * R_radius^2;
        diag.x_D_hat        = zeros(3, 1);
        diag.b_hat          = b_hat_post;
        diag.p_hat          = b_hat_post;             % driver-log alias
        diag.delta_a_hat    = b_hat_post;             % driver-log alias
        diag.a_prime_hat    = a_prime_v;              % da_h/d h_bar [1/pN]
        diag.gate_active_per_axis = gate_off;
        diag.guards_individual    = G_flags;
        diag.h_bar          = h_bar;
        diag.h_bar_d        = h_bar_d;
        diag.f_d            = f_d;
        diag.f_det          = f_det;
        diag.F_dh           = F_dh_vec;
        diag.dx_r           = dh_r * R_radius;    % [um], same convention as delta_x_m
        diag.a_hat          = a_hat_post;
        diag.a_hat_nd       = a_hat_nd_post;
        diag.a_ctrl_used    = a_ctrl * R_radius;
        diag.P77            = P_b_v;                  % exponent posterior var
        diag.Q77            = Q44_v;
        diag.var_da_ram     = Q44_v;
        diag.R2             = R2_v;
        diag.Delta_h_d      = Delta_hbar_d * R_radius;
        diag.Delta_H_d      = Grad_Hbar_d * R_radius;
        diag.delta_x_hat_1  = x_e_per_axis(1, :).' * R_radius;
        diag.delta_x_hat_3  = x_e_per_axis(3, :).' * R_radius;
        diag.P_dx1          = P_dx1_v * R_radius^2;
    end
end


%% =================== Local Helpers ===================

function [a_prime, dap_da, dap_db, B] = local_gain_slope(a_h, b, h_bar_d, a_o, enable_wall)
%LOCAL_GAIN_SLOPE  Exponential-gain slope and Jacobians (5state_expgain_hd.tex).
%   a_h(h_bar) = a_o*[1 - h_bar^(-b)]  with phi = ln(h_bar), so
%       B        = b*phi'(h_bar_d) = b/h_bar_d
%       a_prime  = d a_h / d h_bar = B*(a_o - a_h)
%       dap_da   = d a_prime / d a_h = -B
%       dap_db   = d a_prime / d b   = a_prime/b = (a_o - a_h)/h_bar_d   (b-free)
%   The b-free form is used so the entry stays finite as b -> 0. Unlike the
%   power-law sibling there is no 1/(h_bar-1) singularity to guard.
    if ~enable_wall || ~isfinite(h_bar_d) || h_bar_d <= 0
        a_prime = 0; dap_da = 0; dap_db = 0; B = 0;
        return;
    end
    B       = b / h_bar_d;
    a_prime = B * (a_o - a_h);
    dap_da  = -B;
    dap_db  = (a_o - a_h) / h_bar_d;
end


function F_e = local_build_F_e(lambda_c, F_dh, a_prime, dap_da, dap_db, M_row4)
%LOCAL_BUILD_F_E  5x5 error-dynamics Jacobian (boxed F_e in the spec).
%   cols: 1=dh1 2=dh2 3=dh3 4=a_h 5=b
%   Row 3 = [0 0 lc            -F_dh                        0            ]
%   Row 4 = [0 0 (1-lc)*a'   1 + M*dap_da + a'*F_dh      M*dap_db        ]
%   Row 5 = [0 0 0             0                           1            ]
%   M = Delta_hbar_d + (1-lc)*dh_3_hat is the deterministic part of the step
%   the TRUE position takes (the gain is evaluated at the true height, which
%   trails the command by dh_3).
    one_minus_lc = 1 - lambda_c;
    Fe34 = -F_dh;
    Fe43 = one_minus_lc * a_prime;
    Fe44 = 1 + M_row4 * dap_da + a_prime * F_dh;
    Fe45 = M_row4 * dap_db;
    F_e = [0 1 0        0     0; ...
           0 0 1        0     0; ...
           0 0 lambda_c Fe34  0; ...
           0 0 Fe43     Fe44  Fe45; ...
           0 0 0        0     1];
end


function R2 = compute_R2_expgain(a_h, sigma2_n, IF_abc, C_dpmr, C_n, ...
                                 K_var, amlpf_var_factor, xi, kBT_over_R, Q44, ...
                                 delay_factor, a_cov, whitened)
%COMPUTE_R2_EXPGAIN  R(2,2) for the gain readout channel, non-dimensional.
%   R2_intrinsic is the eq17 IIR chi-squared chain and equals the MARGINAL
%   variance of a_hm's error (spec, "Measurement noise R"):
%       R2_int = amlpf * K_var * IF_var(a_h) * (a_h + xi)^2,  K_var = 2a_cov/(2-a_cov)
%
%   whitened = false : y2 = a_hm itself, so R2 = R2_int + d_factor*Q44.
%   whitened = true  : y2 = a_hm[k] - (1-a_cov)*a_hm[k-1] = a_cov*u[k]. For the
%       AR(1) x[k] = (1-a_cov)x[k-1] + a_cov*u[k] the stationary variance is
%       Var(x) = a_cov*Var(u)/(2-a_cov), so Var(a_cov*u) = a_cov*(2-a_cov)*Var(x),
%       and the d-step q4 accumulation rides on a_cov*u, picking up a_cov^2.
    IF_eff = if_eff_eval(IF_abc, C_dpmr, C_n, kBT_over_R, a_h, sigma2_n);
    R2_int = amlpf_var_factor * K_var * IF_eff * (a_h + xi)^2;
    if whitened
        R2 = a_cov * (2 - a_cov) * R2_int + a_cov^2 * delay_factor * Q44;
    else
        R2 = R2_int + delay_factor * Q44;
    end
end


function IF = if_eff_eval(IF_abc, C_dpmr, C_n, kBT_over_R, a, sigma2_n)
%IF_EFF_EVAL  Exact color-inflation factor for R22 (R22_derivation S4-S6).
%   Homogeneous of degree 0 in (sxT, sigma2_n), so it is invariant under the
%   non-dimensionalisation; both arguments are simply passed in the same units.
    sxT = 4 * kBT_over_R * a;
    num = sxT^2 * IF_abc(1) + 2 * sxT * sigma2_n * IF_abc(2) + sigma2_n^2 * IF_abc(3);
    den = (C_dpmr * sxT + C_n * sigma2_n)^2;
    IF  = 1 + 2 * num / den;
end


function v = get_field_default(s, name, default)
%GET_FIELD_DEFAULT  s.(name) if present and non-empty, else default.
    if isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end


function P_post = solve_dare_kf_local(F, H, Q, R)
%SOLVE_DARE_KF_LOCAL  Discrete-time KF Riccati steady-state (fixed-point).
    n = size(F, 1);
    P_post = eye(n);
    max_iter = 10000;
    tol = 1e-16;
    for k = 1:max_iter
        P_pred = F * P_post * F' + Q;
        P_pred = 0.5 * (P_pred + P_pred');
        S = H * P_pred * H' + R;
        K = (P_pred * H') / S;
        P_new = (eye(n) - K * H) * P_pred;
        P_new = 0.5 * (P_new + P_new');
        if max(abs(P_new(:) - P_post(:))) < tol * max(1, max(abs(P_post(:))))
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


function d = empty_diag_expgain()
%EMPTY_DIAG_EXPGAIN  Zeroed diagnostic struct (driver collect_diag compatible).
    d = struct();
    d.sigma2_dxr_hat    = zeros(3, 1);
    d.a_xm              = zeros(3, 1);
    d.a_hm_nd           = zeros(3, 1);
    d.y2                = zeros(3, 1);
    d.delta_x_m         = zeros(3, 1);
    d.innovation_y2     = zeros(3, 1);
    d.K_kf_a_y2         = zeros(3, 1);
    d.K_kf_dx_y1        = zeros(3, 1);
    d.P_a               = zeros(3, 1);
    d.P_a_nd            = zeros(3, 1);
    d.P_dx              = zeros(3, 1);
    d.x_D_hat           = zeros(3, 1);
    d.b_hat             = ones(3, 1);
    d.p_hat             = ones(3, 1);
    d.delta_a_hat       = ones(3, 1);
    d.a_prime_hat       = zeros(3, 1);
    d.gate_active_per_axis = false(3, 1);
    d.guards_individual    = false(3, 3);
    d.h_bar             = 0;
    d.h_bar_d           = 0;
    d.f_d               = zeros(3, 1);
    d.f_det             = zeros(3, 1);
    d.F_dh              = zeros(3, 1);
    d.dx_r              = zeros(3, 1);
    d.a_hat             = zeros(3, 1);
    d.a_hat_nd          = zeros(3, 1);
    d.a_ctrl_used       = zeros(3, 1);
    d.P77               = zeros(3, 1);
    d.Q77               = zeros(3, 1);
    d.var_da_ram        = zeros(3, 1);
    d.R2                = zeros(3, 1);
    d.Delta_h_d         = 0;
    d.Delta_H_d         = 0;
    d.delta_x_hat_1     = zeros(3, 1);
    d.delta_x_hat_3     = zeros(3, 1);
    d.P_dx1             = zeros(3, 1);
    d.h_bar_init        = 0;
    d.Pf_a_frac         = 0;
    d.Pf_b_std          = zeros(3, 1);
end
