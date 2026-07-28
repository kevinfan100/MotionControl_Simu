% STATUS: ACTIVE (gain-law mainline 7a algebraic arm; defect-1 fix) -- code ahead of tex, see CLAUDE.md mainline section
function [f_d, ekf_out, diag] = motion_control_law_5state_expgain_alg(del_pd, pd, p_m, params, ctrl_const, a_ctrl_override)
%MOTION_CONTROL_LAW_5STATE_EXPGAIN_ALG  ALGEBRAIC-gain variant of the 5-state
%   exponential-gain eq17 controller. PROTOTYPE for the 7a/7b decision; the
%   differential sibling is motion_control_law_5state_expgain.
%
%   Both use the same two-parameter saturating family
%       a_h(h_bar) = a_o * [1 - h_bar^(-b)]
%   but they USE it differently:
%
%     7b (sibling)  a_h is a STATE, propagated by  a_h[k+1] = a_h + a_h'*(step).
%                   b is a state too, so the gain block is [a_h, b].
%     7a (here)     a_h is NOT a state. It is EVALUATED from the model at the
%                   known height, and the states are the two CONSTANTS [a_o, b].
%
%   Why the change (measured on the canonical trajectory, z axis):
%
%   1. ERROR NO LONGER INTEGRATES. 7b propagates the level through a slope that
%      carries a 1.65% RMS model error; over a 10x height traverse that
%      accumulates to a 5.2% level error (measured as the predict-only floor).
%      7a evaluates the level directly, so the error is the family's own level
%      truncation: 0.5% RMS / 1.6% max with the best constant b.
%
%   2. b BECOMES MEASURABLE. In 7b, b reaches y2 only through the d-step delay
%      back-off, so dy2/db is proportional to the height CHANGE over d steps
%      (~0.009). Here y2 sees a_h[k-d] = a_o[1-h_bar^(-b)] directly, so
%      dy2/db = (a_o - a_h)*ln(h_bar) = O(1). Per-sample SNR on b:
%          h_bar          2       3       5      10    22.2
%          7b       0.00185 0.00063 0.00019 0.00004 0.00001
%          7a       0.29427 0.23660 0.17386 0.11018 0.06272
%      i.e. 159x to 7886x, and the INFORMATION ratio is the square of that.
%      In 7b b is effectively unmeasurable and ends up being driven by the
%      position channel instead, which is where the descent transient came from.
%
%   3. Q_bb = 0 BECOMES TRUE. b's constancy holds in the LEVEL sense (b_level
%      spreads 0.045 over the traversed range) but NOT in the slope sense
%      (b_slope spreads 0.080). 7b uses the slope reading, so asserting a
%      constant b there is false and needs a process-noise closure. 7a uses the
%      level reading, where the assertion is the model's own definition.
%
%   Two structural simplifications follow, neither of them designed for:
%
%   * F_e row 3 gains the exact position feedthrough. The true gain sits at the
%     true height while the model evaluates at the commanded one, differing by
%     -a_h'*delta_h_3 -- and delta_h_3 is a STATE, so this is deterministic and
%     computable, belonging in F_e rather than in Q:
%         F_e(3,3) = lambda_c + F_dh * a_h'
%     (the same term the 4-state line records as F_e(4,4) = lc + a'*F_dx).
%
%   * H row 2 grows H(2,1) = -a_h' by itself. y2 reports the gain at step k-d,
%     and the height error AT THAT STEP IS the state delta_h_1. The sibling's
%     spec lists this as an optional exact replacement for R2's d*Q44 delay
%     term; here it is not optional, it is what the measurement equation says,
%     and R2 then needs no delay term at all.
%
%   State (per axis, fully non-dimensional; lengths divided by R):
%
%       x = [dh_1; dh_2; dh_3; a_o; b]
%
%   a_o = far-field gain [1/pN] (bulk drag, known to its calibration accuracy),
%   b = wall-shape exponent. BOTH ARE CONSTANTS: F_e rows 4 and 5 are identity
%   and Q(4,4) = Q(5,5) = 0 exactly. They are separated by the trajectory's
%   height sweep -- at a single height only the product is constrained.
%
%   Evaluated quantities (h_bar_d = commanded height, exogenous):
%       a_h(h)       = a_o*[1 - h^(-b)]
%       a_h'(h)      = (b/h)*(a_o - a_h)          d a_h / d h_bar
%       da_h/da_o    = a_h/a_o = 1 - h^(-b)
%       da_h/db      = (a_o - a_h)*ln(h)
%
%   Q reduces to a SINGLE entry. The gain's "process noise" in the sibling was
%   only ever the reflection of position noise (q4 = a_h'*eps_h, giving the
%   rank-1 block); with the gain no longer a state that reflection is carried by
%   the position states themselves, so
%       Q33 = (4*kB*T/R)*a_h ,   everything else 0.
%
%   Optional ctrl_const knobs:
%       .y2_whiten    whitened y2 increment                  (default true)
%       .use_fdet     deterministic F_e regressor            (default true)
%       .Pf_ao_frac   sqrt(Pf(4,4))/a_o, the drag-calibration
%                     accuracy                               (default 0.01)
%       .Pf_b_std     sqrt(Pf(5,5))                          (default 0.10)
%       .b_init       initial exponent                       (default 1)
%       .fe33_feed    include F_e(3,3) = lc + F_dh*a_h'       (default true)
%       .y2_off / .y1_gain_off   channel ablation            (default false)
%
%   See also: motion_control_law_5state_expgain, build_eq17_6state_constants

    if nargin < 6
        a_ctrl_override = [];
    end
    has_override = ~isempty(a_ctrl_override);

    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = [1; 1; 1; 0];
        if nargout >= 3
            diag = empty_diag_alg();
            diag.f_d = f_d;
        end
        return;
    end

    persistent x_e_per_axis P_per_axis
    persistent dh_bar_m sigma2_dhr_hat a_hm_km1
    persistent fdet_km1 fdet_km2 pd_km1 pd_km2 f_d_km1 f_d_km2
    persistent a_ctrl_km1 a_ctrl_km2 k_step
    persistent initialized
    persistent lambda_c d_delay Ts kBT_over_R R_radius a_nom_nd
    persistent a_pd a_cov C_dpmr C_n K_var IF_abc xi_nd amlpf_var_factor
    persistent t_warmup_kf h_bar_safe sigma2_n_nd
    persistent enable_wall w_hat_n pz_wall
    persistent y2_whiten use_fdet fe33_feed y2_off y1_gain_off
    persistent a_floor

    % ------------------------------------------------------------------
    % [0] Initialization
    % ------------------------------------------------------------------
    if isempty(initialized)
        initialized = true;

        Ts         = params.ctrl.Ts;
        R_radius   = params.common.R;
        kBT_over_R = params.ctrl.k_B * params.ctrl.T / R_radius;      % [pN]
        gamma_N_p  = params.ctrl.gamma;
        sigma2_n_nd = params.ctrl.sigma2_noise(:) / R_radius^2;
        a_nom_nd   = Ts / (gamma_N_p * R_radius);                     % [1/pN]

        lambda_c   = ctrl_const.lambda_c;
        d_delay    = ctrl_const.d;
        C_dpmr     = ctrl_const.C_dpmr;
        C_n        = ctrl_const.C_n;
        K_var      = ctrl_const.K_var;
        IF_abc     = ctrl_const.IF_abc(:);
        t_warmup_kf = ctrl_const.t_warmup_kf;
        h_bar_safe = ctrl_const.h_bar_safe;
        a_cov      = ctrl_const.a_cov;
        a_pd       = ctrl_const.a_pd;
        amlpf_var_factor = get_field_default(ctrl_const, 'amlpf_var_factor', 1);
        xi_nd      = ctrl_const.xi_per_axis(:) / R_radius;

        y2_whiten   = logical(get_field_default(ctrl_const, 'y2_whiten', true));
        use_fdet    = logical(get_field_default(ctrl_const, 'use_fdet', true));
        fe33_feed   = logical(get_field_default(ctrl_const, 'fe33_feed', true));
        y2_off      = logical(get_field_default(ctrl_const, 'y2_off', false));
        y1_gain_off = logical(get_field_default(ctrl_const, 'y1_gain_off', false));

        if isfield(params, 'wall')
            w_hat_n = params.wall.w_hat(:);
            pz_wall = params.wall.pz;
            enable_wall = params.wall.enable_wall_effect > 0.5;
        else
            w_hat_n = [0; 0; 1]; pz_wall = 0; enable_wall = false;
        end

        % --- Seeds. a_o is the bulk drag, known to its calibration accuracy;
        %     b = 1 is pinned by the two published asymptotes (far-field
        %     reflections and near-wall lubrication both force b = 1, and the
        %     value is independent of the reflection coefficient K, hence the
        %     same on all three axes). No c(h_bar) anywhere.
        b_init = get_field_default(ctrl_const, 'b_init', 1.0);
        if isscalar(b_init); b_init = b_init * ones(3, 1); end
        Pf_ao_frac = get_field_default(ctrl_const, 'Pf_ao_frac', 0.01);
        Pf_b_std   = get_field_default(ctrl_const, 'Pf_b_std', 0.10);
        if isscalar(Pf_b_std); Pf_b_std = Pf_b_std * ones(3, 1); end

        a_floor = 0.02 * a_nom_nd;

        x_e_per_axis = zeros(5, 3);
        x_e_per_axis(4, :) = a_nom_nd;
        x_e_per_axis(5, :) = b_init(:).';

        % Pf: DARE on the observable 3-block, constants' priors on 4/5
        F3 = [0 1 0; 0 0 1; 0 0 lambda_c];
        H3 = [1 0 0];
        P_per_axis = cell(3, 1);
        h_bar_seed = local_h_bar_d(params, w_hat_n, pz_wall, R_radius, enable_wall);
        for ax = 1:3
            a_seed = local_gain(a_nom_nd, b_init(ax), h_bar_seed, enable_wall);
            Q3 = zeros(3); Q3(3, 3) = 4 * kBT_over_R * a_seed;
            P5 = zeros(5);
            P5(1:3, 1:3) = solve_dare_kf_local(F3, H3, Q3, sigma2_n_nd(ax));
            P5(4, 4) = (Pf_ao_frac * a_nom_nd)^2;
            P5(5, 5) = Pf_b_std(ax)^2;
            P_per_axis{ax} = P5;
            a_seed_v(ax, 1) = a_seed; %#ok<AGROW>
        end

        dh_bar_m = zeros(3, 1);
        sigma2_dhr_hat = 4 * kBT_over_R * a_seed_v * C_dpmr + C_n * sigma2_n_nd;
        a_hm_km1 = a_seed_v;

        pd_km1 = pd; pd_km2 = pd;
        f_d_km1 = zeros(3, 1); f_d_km2 = zeros(3, 1);
        fdet_km1 = zeros(3, 1); fdet_km2 = zeros(3, 1);
        a_ctrl_km1 = a_seed_v; a_ctrl_km2 = a_seed_v;
        k_step = 1;

        f_d = zeros(3, 1);
        a_hat_phys = a_seed_v * R_radius;
        ekf_out = [a_hat_phys(1); a_hat_phys(3); a_hat_phys(2); 0];
        if nargout >= 3
            diag = empty_diag_alg();
            diag.f_d       = f_d;
            diag.a_hat     = a_hat_phys;
            diag.a_o_hat   = a_nom_nd * R_radius * ones(3, 1);
            diag.b_hat     = b_init(:);
            diag.p_hat     = b_init(:);
            diag.a_ctrl_used = a_hat_phys;
            diag.sigma2_dxr_hat = sigma2_dhr_hat;
            diag.P77       = Pf_b_std(:).^2;
        end
        return;
    end

    % ------------------------------------------------------------------
    % Per-step
    % ------------------------------------------------------------------
    if d_delay == 2
        pd_km_d = pd_km2;
    elseif d_delay == 1
        pd_km_d = pd_km1;
    else
        error('motion_control_law_5state_expgain_alg:unsupportedDelay', ...
              'Only d=1 or d=2 supported, got d=%g.', d_delay);
    end
    delta_h_m = (pd_km_d - p_m) / R_radius;
    pd_kp1    = pd + del_pd;
    one_minus_lc = 1 - lambda_c;

    if enable_wall
        h_bar   = (dot(p_m, w_hat_n) - pz_wall) / R_radius;
        h_bar_d = max((dot(pd, w_hat_n) - pz_wall) / R_radius, 1.001);
        h_bar_dm = max((dot(pd_km_d, w_hat_n) - pz_wall) / R_radius, 1.001);
    else
        h_bar = Inf; h_bar_d = Inf; h_bar_dm = Inf;
    end

    % --- Gain EVALUATED from the model (not a state) ---
    ao_hat = x_e_per_axis(4, :).';
    b_hat  = x_e_per_axis(5, :).';
    a_hat_nd = zeros(3, 1); a_prime = zeros(3, 1);
    dA_dao = zeros(3, 1);   dA_db = zeros(3, 1);
    a_km_d = zeros(3, 1);   ap_km_d = zeros(3, 1);
    dAd_dao = zeros(3, 1);  dAd_db = zeros(3, 1);
    for ax = 1:3
        [a_hat_nd(ax), a_prime(ax), dA_dao(ax), dA_db(ax)] = ...
            local_gain_jac(ao_hat(ax), b_hat(ax), h_bar_d, enable_wall);
        [a_km_d(ax), ap_km_d(ax), dAd_dao(ax), dAd_db(ax)] = ...
            local_gain_jac(ao_hat(ax), b_hat(ax), h_bar_dm, enable_wall);
    end

    if has_override
        a_ctrl = a_ctrl_override(:) / R_radius;
    else
        a_ctrl = a_hat_nd;
    end
    a_ctrl = max(a_ctrl, a_floor);

    % --- IIR gain readout ---
    dh_bar_m_new = (1 - a_pd) * dh_bar_m + a_pd * delta_h_m;
    dh_r = delta_h_m - dh_bar_m_new;
    sigma2_dhr_hat_new = (1 - a_cov) * sigma2_dhr_hat + a_cov * dh_r.^2;
    a_hm = (sigma2_dhr_hat_new - C_n * sigma2_n_nd) / (C_dpmr * 4 * kBT_over_R);
    if y2_whiten
        y2 = a_hm - (1 - a_cov) * a_hm_km1;
        H2_scale = a_cov;
    else
        y2 = a_hm;
        H2_scale = 1;
    end

    % --- Control law (eq17 implementable) + deterministic mirror ---
    if d_delay == 2
        sum_a_fd_past = a_ctrl_km1 .* f_d_km1 + a_ctrl_km2 .* f_d_km2;
        sum_af_det    = a_ctrl_km1 .* fdet_km1 + a_ctrl_km2 .* fdet_km2;
    else
        sum_a_fd_past = a_ctrl_km1 .* f_d_km1;
        sum_af_det    = a_ctrl_km1 .* fdet_km1;
    end
    inv_a_ctrl = 1 ./ a_ctrl;
    traj_term  = (pd_kp1 - lambda_c * pd - one_minus_lc * pd_km_d) / R_radius;
    f_d   = inv_a_ctrl .* (traj_term + one_minus_lc * delta_h_m - one_minus_lc * sum_a_fd_past);
    f_det = inv_a_ctrl .* (traj_term - one_minus_lc * sum_af_det);
    if d_delay == 2
        F_dh_det = f_det + one_minus_lc * (fdet_km1 + fdet_km2);
        F_dh_raw = f_d   + one_minus_lc * (f_d_km1 + f_d_km2);
    else
        F_dh_det = f_det + one_minus_lc * fdet_km1;
        F_dh_raw = f_d   + one_minus_lc * f_d_km1;
    end
    if use_fdet; F_dh_vec = F_dh_det; else; F_dh_vec = F_dh_raw; end

    % ------------------------------------------------------------------
    % EKF
    % ------------------------------------------------------------------
    I5 = eye(5);
    t_now = (k_step - 1) * Ts;
    K_a_y2_v = zeros(3,1); K_dx_y1_v = zeros(3,1); innov_y2_v = zeros(3,1);
    gate_off = false(3,1); G_flags = false(3,3); R2_v = zeros(3,1);

    for ax = 1:3
        x_curr = x_e_per_axis(:, ax);
        P_curr = P_per_axis{ax};
        a_h_i  = max(a_hat_nd(ax), a_floor);

        % --- Q: a single entry. The gain is no longer a state, so the
        %     reflection of position noise into the gain (the sibling's rank-1
        %     block) is carried by the position states themselves.
        Q_i = zeros(5);
        Q_i(3, 3) = 4 * kBT_over_R * a_h_i;

        % --- R ---
        R1_i = sigma2_n_nd(ax);
        R2_i = compute_R2_alg(a_km_d(ax), sigma2_n_nd(ax), IF_abc, C_dpmr, C_n, ...
                              K_var, amlpf_var_factor, xi_nd(ax), kBT_over_R, ...
                              a_cov, y2_whiten);
        R2_v(ax) = R2_i;

        G1 = (t_now < t_warmup_kf);
        G2 = ((sigma2_dhr_hat_new(ax) - C_n * sigma2_n_nd(ax)) <= 0);
        G3 = (h_bar < h_bar_safe);
        G_flags(:, ax) = [G1; G2; G3];
        gate_off(ax) = G1 || G2 || G3;

        % --- F_e.  Rows 4 and 5 are identity: a_o and b are constants.
        %     Row 3 carries the exact position feedthrough (see the header).
        F_dh = F_dh_vec(ax);
        if fe33_feed
            Fe33 = lambda_c + F_dh * a_prime(ax);
        else
            Fe33 = lambda_c;
        end
        F_e = [0 1 0     0                   0; ...
               0 0 1     0                   0; ...
               0 0 Fe33  -F_dh*dA_dao(ax)   -F_dh*dA_db(ax); ...
               0 0 0     1                   0; ...
               0 0 0     0                   1];

        % --- Predict: the constants do not move; the gain is re-evaluated.
        %   Slot 3 must carry the SAME feedthrough as F_e(3,3): the estimator's
        %   best prediction of dh_3[k+1] is (lc + F_dh*a')*dh_3_hat, because
        %   E[e_ah | x_hat] = -a'*dh_3_hat (the parameter errors are zero-mean
        %   by definition, the height offset is not).
        x_pred = [x_curr(2); x_curr(3); Fe33 * x_curr(3); x_curr(4); x_curr(5)];
        P_pred = F_e * P_curr * F_e' + Q_i;
        P_pred = 0.5 * (P_pred + P_pred');

        freeze_gain = G1;

        % (a) y1
        H1 = [1 0 0 0 0];
        S1 = H1 * P_pred * H1' + R1_i;
        K1 = (P_pred * H1') / S1;
        if freeze_gain || y1_gain_off; K1(4:5) = 0; end
        x_upd = x_pred + K1 * (delta_h_m(ax) - H1 * x_pred);
        ImKH1 = I5 - K1 * H1;
        P_upd = ImKH1 * P_pred * ImKH1' + K1 * R1_i * K1';
        P_upd = 0.5 * (P_upd + P_upd');
        K_dx_y1_v(ax) = K1(3);

        % (b) y2 observes a_h at step k-d, i.e. at height h_bar_d[k-d] - dh_1.
        %     The height error at THAT step is the state dh_1, so H(2,1) = -a'
        %     appears exactly and R2 needs no delay term.
        if ~gate_off(ax) && ~y2_off
            H2 = H2_scale * [-ap_km_d(ax), 0, 0, dAd_dao(ax), dAd_db(ax)];
            y2_pred = H2_scale * (a_km_d(ax) - ap_km_d(ax) * x_upd(1) ...
                                  + dAd_dao(ax) * (x_upd(4) - ao_hat(ax)) ...
                                  + dAd_db(ax)  * (x_upd(5) - b_hat(ax)));
            S2 = H2 * P_upd * H2' + R2_i;
            K2 = (P_upd * H2') / S2;
            if freeze_gain; K2(4:5) = 0; end
            innov2 = y2(ax) - y2_pred;
            x_upd  = x_upd + K2 * innov2;
            ImKH2  = I5 - K2 * H2;
            P_upd  = ImKH2 * P_upd * ImKH2' + K2 * R2_i * K2';
            P_upd  = 0.5 * (P_upd + P_upd');
            K_a_y2_v(ax)   = K2(4);
            innov_y2_v(ax) = innov2;
        end

        x_upd(4) = max(x_upd(4), a_floor);
        x_upd(5) = min(max(x_upd(5), 0.2), 3);
        x_e_per_axis(:, ax) = x_upd;
        P_per_axis{ax} = P_upd;
    end

    % --- Bookkeeping ---
    pd_km2 = pd_km1; pd_km1 = pd;
    f_d_km2 = f_d_km1; f_d_km1 = f_d;
    fdet_km2 = fdet_km1; fdet_km1 = f_det;
    a_ctrl_km2 = a_ctrl_km1; a_ctrl_km1 = a_ctrl;
    dh_bar_m = dh_bar_m_new;
    sigma2_dhr_hat = sigma2_dhr_hat_new;
    a_hm_km1 = a_hm;
    k_step = k_step + 1;

    % --- Output: report the gain evaluated with the POSTERIOR constants ---
    ao_post = x_e_per_axis(4, :).';
    b_post  = x_e_per_axis(5, :).';
    a_post  = zeros(3, 1); ap_post = zeros(3, 1);
    for ax = 1:3
        [a_post(ax), ap_post(ax)] = local_gain_jac(ao_post(ax), b_post(ax), h_bar_d, enable_wall);
    end
    a_hat_phys = a_post * R_radius;
    ekf_out = [a_hat_phys(1); a_hat_phys(3); a_hat_phys(2); local_h_bar_out(enable_wall, h_bar)];

    if nargout >= 3
        P_ao = zeros(3,1); P_b = zeros(3,1); P_dx = zeros(3,1); P_dx1 = zeros(3,1);
        for ax = 1:3
            P_ao(ax) = P_per_axis{ax}(4,4);
            P_b(ax)  = P_per_axis{ax}(5,5);
            P_dx(ax) = P_per_axis{ax}(3,3);
            P_dx1(ax)= P_per_axis{ax}(1,1);
        end
        diag = empty_diag_alg();
        diag.sigma2_dxr_hat = sigma2_dhr_hat_new;
        diag.a_xm        = a_hm * R_radius;
        diag.y2          = y2;
        diag.delta_x_m   = delta_h_m * R_radius;
        diag.innovation_y2 = innov_y2_v;
        diag.K_kf_a_y2   = K_a_y2_v;
        diag.K_kf_dx_y1  = K_dx_y1_v;
        diag.P_a         = P_ao * R_radius^2;
        diag.P_dx        = P_dx * R_radius^2;
        diag.P_dx1       = P_dx1 * R_radius^2;
        diag.x_D_hat     = zeros(3,1);
        diag.a_hat       = a_hat_phys;
        diag.a_o_hat     = ao_post * R_radius;
        diag.b_hat       = b_post;
        diag.p_hat       = b_post;
        diag.delta_a_hat = b_post;
        diag.a_prime_hat = ap_post;
        diag.gate_active_per_axis = gate_off;
        diag.guards_individual = G_flags;
        diag.h_bar       = h_bar;
        diag.h_bar_d     = h_bar_d;
        diag.f_d         = f_d;
        diag.f_det       = f_det;
        diag.F_dh        = F_dh_vec;
        diag.dx_r        = dh_r * R_radius;      % [um], same convention as delta_x_m
        diag.a_ctrl_used = a_ctrl * R_radius;
        diag.P77         = P_b;
        diag.R2          = R2_v;
        diag.delta_x_hat_1 = x_e_per_axis(1,:).' * R_radius;
        diag.delta_x_hat_3 = x_e_per_axis(3,:).' * R_radius;
    end
end


%% =================== Local Helpers ===================

function a = local_gain(a_o, b, h_bar, enable_wall)
    if ~enable_wall || ~isfinite(h_bar) || h_bar <= 1
        a = a_o;
    else
        a = a_o * (1 - h_bar^(-b));
    end
end


function [a, ap, dA_dao, dA_db] = local_gain_jac(a_o, b, h_bar, enable_wall)
%LOCAL_GAIN_JAC  a_h(h_bar) and its Jacobians for the ALGEBRAIC model
%       a_h    = a_o*[1 - h^(-b)]
%       a_h'   = (b/h)*(a_o - a_h)                 d a_h / d h_bar
%       dA/da_o = 1 - h^(-b) = a_h/a_o
%       dA/db   = (a_o - a_h)*ln(h)                <- O(1); this is the entry
%                 that makes b measurable at all (see the header SNR table)
    if ~enable_wall || ~isfinite(h_bar) || h_bar <= 1
        a = a_o; ap = 0; dA_dao = 1; dA_db = 0;
        return;
    end
    w      = h_bar^(-b);
    a      = a_o * (1 - w);
    ap     = (b / h_bar) * (a_o - a);
    dA_dao = 1 - w;
    dA_db  = (a_o - a) * log(h_bar);
end


function h = local_h_bar_d(params, w_hat_n, pz_wall, R_radius, enable_wall)
    if enable_wall && isfield(params, 'common') && isfield(params.common, 'p0')
        h = max((dot(params.common.p0(:), w_hat_n) - pz_wall) / R_radius, 1.001);
    else
        h = Inf;
    end
end


function R2 = compute_R2_alg(a_h, sigma2_n, IF_abc, C_dpmr, C_n, K_var, ...
                             amlpf_var_factor, xi, kBT_over_R, a_cov, whitened)
%COMPUTE_R2_ALG  R(2,2) for the gain readout. Identical to the sibling EXCEPT
%   that the d-step delay term is gone: the delay is now represented exactly by
%   H(2,1) = -a_h', because the height error at step k-d IS the state dh_1.
    sxT = 4 * kBT_over_R * a_h;
    num = sxT^2*IF_abc(1) + 2*sxT*sigma2_n*IF_abc(2) + sigma2_n^2*IF_abc(3);
    den = (C_dpmr*sxT + C_n*sigma2_n)^2;
    IF  = 1 + 2*num/den;
    R2_int = amlpf_var_factor * K_var * IF * (a_h + xi)^2;
    if whitened
        R2 = a_cov * (2 - a_cov) * R2_int;
    else
        R2 = R2_int;
    end
end


function v = get_field_default(s, name, default)
    if isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end


function P_post = solve_dare_kf_local(F, H, Q, R)
    n = size(F, 1);
    P_post = eye(n);
    for k = 1:10000
        P_pred = F * P_post * F' + Q;
        P_pred = 0.5 * (P_pred + P_pred');
        S = H * P_pred * H' + R;
        K = (P_pred * H') / S;
        P_new = (eye(n) - K * H) * P_pred;
        P_new = 0.5 * (P_new + P_new');
        if max(abs(P_new(:) - P_post(:))) < 1e-16 * max(1, max(abs(P_post(:))))
            P_post = P_new; return;
        end
        P_post = P_new;
    end
end


function h = local_h_bar_out(enable_wall, h_bar)
    if enable_wall; h = h_bar; else; h = 0; end
end


function d = empty_diag_alg()
    d = struct();
    d.sigma2_dxr_hat = zeros(3,1);
    d.a_xm           = zeros(3,1);
    d.y2             = zeros(3,1);
    d.delta_x_m      = zeros(3,1);
    d.innovation_y2  = zeros(3,1);
    d.K_kf_a_y2      = zeros(3,1);
    d.K_kf_dx_y1     = zeros(3,1);
    d.P_a            = zeros(3,1);
    d.P_dx           = zeros(3,1);
    d.P_dx1          = zeros(3,1);
    d.x_D_hat        = zeros(3,1);
    d.a_hat          = zeros(3,1);
    d.a_o_hat        = zeros(3,1);
    d.b_hat          = ones(3,1);
    d.p_hat          = ones(3,1);
    d.delta_a_hat    = ones(3,1);
    d.a_prime_hat    = zeros(3,1);
    d.gate_active_per_axis = false(3,1);
    d.guards_individual    = false(3,3);
    d.h_bar          = 0;
    d.h_bar_d        = 0;
    d.f_d            = zeros(3,1);
    d.f_det          = zeros(3,1);
    d.F_dh           = zeros(3,1);
    d.dx_r           = zeros(3,1);
    d.a_ctrl_used    = zeros(3,1);
    d.P77            = zeros(3,1);
    d.R2             = zeros(3,1);
    d.delta_x_hat_1  = zeros(3,1);
    d.delta_x_hat_3  = zeros(3,1);
    d.Delta_h_d      = 0;
    d.Delta_H_d      = 0;
    d.a_hat_nd       = zeros(3,1);
    d.var_da_ram     = zeros(3,1);
    d.Q77            = zeros(3,1);
end
