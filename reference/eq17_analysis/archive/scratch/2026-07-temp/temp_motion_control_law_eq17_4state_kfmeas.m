% FORK OF model/controller/motion_control_law_eq17_4state.m @ pre-b5dbbc2 | PURPOSE: kfmeas 5-state (a_d/a' in H(2,5)) + C2 arms | EXPIRES: C2 channel (block C) resolved | 產線改動不會自動跟上
function [f_d, ekf_out, diag] = temp_motion_control_law_eq17_4state_kfmeas(del_pd, pd, p_m, params, ctrl_const, a_ctrl_override)
%TEMP_MOTION_CONTROL_LAW_EQ17_4STATE_KFMEAS  Copy of the centered controller
%   plus a NEW aprime_source='kfmeas' mode (5state_aprime_kf_meas.tex): a' is a
%   PROPER KF state updated by the y2 measurement (a' lives in the measurement
%   Jacobian H(2,5), NOT the predict). z-axis only (slot 5); x/y stay inert.
%
%   THREE changes vs aprime_source='state' (the current as-state), all gated to
%   the z axis (ax==3) with kfmeas_on = ap_on && strcmp(aprime_source,'kfmeas'):
%     1. Slot-4 state is a_d (DESIRED-height gain). Its predict is DETERMINISTIC
%        sweep only: a_d[k+1] = a_d[k] + a_hat'[k]*dh_d_step. No thermal
%        deviation term (1-lc)*a'*dxhat3 (that term random-walks the level in
%        the a_x-state formulation; here the thermal part moves to y2).
%     2. y2 is BILINEAR and carries a'. With DH_d = h_d[k]-h_d[k-d] (known):
%          h2(x) = a_d - a'*(DH_d + dh1)             (nonlinear prediction)
%          y2    = a_xm  (RAW; NO sum_da_ff delay correction)
%          H2    = [ -a_hat', 0, 0, 1, -(DH_d + dxhat1) ]   (Jacobian)
%        y1 unchanged: y1 = dh1 + n_x, H1 = [1 0 0 0 0]. Standard EKF gain,
%        Joseph-form P update. R(2,2) = R2_intrinsic (the d-step drift is now
%        modeled in h2, not lumped into R -> the telescoped R22_delay is dropped
%        on z).
%     3. Control law reconstructs the ACTUAL gain a_hat_x = a_d - a'*dxhat3 for
%        f_d = (1/a_hat_x){...} (a_d is desired-height; the plant needs the
%        actual-height gain). ekf_out / diag.a_hat report the reconstructed a_x
%        on z. F_e Row 4 = [0 0 0 1 Dh_d] (pure integrator + sweep coupling).
%
%   New knobs: aprime_state_init_scale (slot-5 init = scale*a'_oracle(h_init);
%   default 0 = backward-compat with 'state'), freeze_level_kf (predict-only
%   level test: zeroes K(4,:) on z). Everything else (IIR a_xm chain, R1=sigma2_n,
%   gating, Q55) as in the centered controller.
%   TEMP diagnostic (chat 2026-07-20); delete after the kfmeas verification.
%
%MOTION_CONTROL_LAW_EQ17_4STATE Per-axis 4-state EKF controller
%   (RevisedControl_Vpersonal, disturbance state removed + gain RATE fed
%   forward from the desired trajectory). See
%   reference/eq17_analysis/derivation/4state_del_hd.tex.
%
%   [f_d, ekf_out]       = motion_control_law_eq17_4state(del_pd, pd, p_m, params, ctrl_const)
%   [f_d, ekf_out, diag] = motion_control_law_eq17_4state(...)
%   [f_d, ekf_out, diag] = motion_control_law_eq17_4state(..., ctrl_const, a_ctrl_override)
%
%   Built on motion_control_law_eq17_5state. This variant DROPS the gain-rate
%   state delta_a_x (5-state slot 5). The augmented per-axis state is therefore
%   4-dimensional:
%
%       x = [delta_x_1; delta_x_2; delta_x_3; a_x]
%
%   Instead of estimating the gain rate, it is computed as a KNOWN feedforward
%   from the desired trajectory through the wall function (exogenous, not the
%   estimate -- no bias loop):
%
%       a_x_det[m]   = a_nom / c(h_bar_d[m]),   h_bar_d from p_d (desired)
%       delta_a_x[k] = a_x_det[k] - a_x_det[k-1]            (predict feedforward)
%       Sum_i delta_a_x[k-i] = a_x_det[k] - a_x_det[k-d]    (measurement correction)
%
%   Consequences vs the 5-state:
%     - F_e Row 3 loses the col-5 "dF_dx" rate-error coupling:
%           5-state Row 3 = [0 0 lc -F_dx dF_dx]
%           4-state Row 3 = [0 0 lc -F_dx]       (col 4 = a_x)
%     - a_x predict adds the KNOWN delta_a_x[k] (feedforward) instead of an
%       estimated rate state.
%     - a_xm is unchanged; the d-step gain drift is removed via a KNOWN
%       measurement correction (a_xm + Sum_i delta_a_x[k-i]), so H's second row
%       is clean [0 0 0 1] (no -d coupling).
%     - H, Q, Pf shrink to 4x4 / 2x4.
%
%   Q44 (= 5-state Q44 = var(delta_a_ram)) and R22 use the MEASURED h_bar (same
%   as the 5-state) -- only the deterministic feedforward uses the DESIRED
%   h_bar_d. Offline constants are dimension-agnostic: this controller reuses
%   build_eq17_6state_constants verbatim.
%
%   Inputs/outputs match the 5-state (drop-in for run_pure_simulation dispatch).
%   diag.delta_a_hat and diag.x_D_hat are returned as zeros (no such states) for
%   driver-log compatibility.
%
%   See also: motion_control_law_eq17_5state, motion_control_law_eq17_6state,
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
               'motion_control_law_eq17_4state:badOverride', ...
               'a_ctrl_override must be a 3x1 finite positive vector.');
    end

    % ------------------------------------------------------------------
    % Open-loop bypass
    % ------------------------------------------------------------------
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = [1; 1; 1; 0];
        if nargout >= 3
            diag = empty_diag_4state();
            diag.f_d = f_d;
        end
        return;
    end

    % ------------------------------------------------------------------
    % Persistent state
    % ------------------------------------------------------------------
    persistent x_e_per_axis        % 4x3 EKF state (col = axis)
    persistent P_per_axis          % cell{3} of 4x4 covariance
    persistent dx_bar_m            % 3x1 IIR LP mean of delta_x_m [um]
    persistent sigma2_dxr_hat      % 3x1 EWMA variance of dx_r    [um^2]
    persistent kappa_hat           % 3x1 EWMA of de-blurred kappa = a_true/a_det [-]
    persistent a_m_det             % 3x1 post-LPF of a_xm (a_m_det) [um/pN]
    persistent pd_km1 pd_km2       % trajectory delay buffers (also give h_bar_d history)
    persistent f_d_km1 f_d_km2     % past control buffers (for Sigma f_d[k-i])
    persistent a_hat_km1 a_hat_km2          % past gain estimates a_hat[k-i] (active law + Q33)
    persistent var_da_inc_km1 var_da_inc_km2 % past increment var(delta_a_ram[k-i]) (Q33/R22)
    persistent a_ctrl_km1 a_ctrl_km2   % control-law gain history (= a_hat history unless override)
    persistent warmup_count k_step

    persistent initialized
    persistent lambda_c d_delay Ts kBT R_radius gamma_N_p a_nom_p
    persistent a_pd a_cov C_dpmr C_n K_var IF_abc xi_per_axis var_da_inc_factor r22_delay_factor
    persistent t_warmup_kf h_bar_safe R_OFF use_am_lpf a_det_lp amlpf_var_factor use_deblur use_aprime_ff use_q44_cap use_q44_ar1 use_exact_fe44
    persistent use_taylor_gain aprime_source ap_beta ap_gate_um ap_clamp_pos ap_pos_only  % taylor-gain suite (4state_del_hd.tex taylor section)
    persistent ap_scale                                                       % TEMP (chat 2026-07-15): freeze a' at scale x computed value (E2)
    persistent g4_selfdiff_ar1                                                % TEMP (chat 2026-07-15): G4 blue arm = AR1-revert-to-known-a_det level + self-diff a'
    persistent vc_beta vc_warm vc_avar vc_acov vc_abar vc_dbar vc_s2ar vc_s2dh vc_ap_hat  % TEMP (chat 2026-07-15): closed-loop var-channel a' (aprime_source='varchan')
    persistent vc_warm_steps                                                  % TEMP (chat 2026-07-17): fixed-step ignition guard (denominator birth-clock lag)
    persistent vc_adet vc_adet_hist                                          % TEMP (chat 2026-07-16): centered maintained reference (aprime_source='varchan_centered')
    persistent aprime_eval_xhat                                               % EXPERIMENT (chat 2026-07-13): a' at x_hat = p_d - dxhat3 instead of p_d
    persistent n_aug ap_kappa ap_P55_0 ap_learn_t0                            % 5state a'-as-state (5state_taylor_aprime.pdf): slope promoted to state 5 (z only)
    persistent ap_Q55_floor                                                   % TEMP (chat 2026-07-19, AG): Q55 floor (PLACEHOLDER, forgetting on low-excitation)
    persistent ap_selfmod                                                     % TEMP (chat 2026-07-17): full F(4,5) Jacobian (hold-observable selfmod coupling)
    persistent ap_state_init_scale freeze_level_kf                            % NEW (chat 2026-07-20, kfmeas): a'-state init scale + predict-only level freeze
    persistent ap_on y2_ar1 phi_v r2f v_slot                                  % TEMP (chat 2026-07-20): colored-y2 AR(1) v-state port (production 44349d0; v = LAST slot)
    persistent a_prime_diff                                                   % 'diff' EWMA slope state (selfrw convention)
    persistent use_c2 c2_Fmin c2_knu3 c2_r3_scale F_dx_det                     % TEMP (chat 2026-07-22): C2 det-level channel (y3=dx_bar_m, kf_meas.tex §9)
    persistent c2_window_s c2_Wsteps c2_SFy c2_SFF c2_count                    % TEMP (chat 2026-07-22): C2 multi-window weighted-LS accumulators
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
        a_nom_p   = Ts / gamma_N_p;                       % [um/pN]

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
        if isfield(ctrl_const, 'use_am_lpf') && ~isempty(ctrl_const.use_am_lpf)
            use_am_lpf = ctrl_const.use_am_lpf;
        else
            use_am_lpf = false;
        end
        if isfield(ctrl_const, 'a_det') && ~isempty(ctrl_const.a_det)
            a_det_lp = ctrl_const.a_det;
        else
            a_det_lp = 0.005;
        end
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
        % r22_delay_factor (audit 2026-07-10): telescoped 2-step-difference
        % correction for the R22 d-step delay term (shared by ALL Q44 modes).
        % Backward-compat: absent -> 1 (old independent-sum, ~9.5% low at d=2).
        if isfield(ctrl_const, 'r22_delay_sum_factor') && ~isempty(ctrl_const.r22_delay_sum_factor)
            r22_delay_factor = ctrl_const.r22_delay_sum_factor;
        else
            r22_delay_factor = 1;
        end
        if isfield(ctrl_const, 'use_deblur') && ~isempty(ctrl_const.use_deblur)
            use_deblur = ctrl_const.use_deblur;
        else
            use_deblur = false;
        end
        if isfield(ctrl_const, 'use_aprime_ff') && ~isempty(ctrl_const.use_aprime_ff)
            use_aprime_ff = ctrl_const.use_aprime_ff;
        else
            use_aprime_ff = false;
        end
        if isfield(ctrl_const, 'use_q44_cap') && ~isempty(ctrl_const.use_q44_cap)
            use_q44_cap = ctrl_const.use_q44_cap;
        else
            use_q44_cap = false;
        end
        if isfield(ctrl_const, 'use_q44_ar1') && ~isempty(ctrl_const.use_q44_ar1)
            use_q44_ar1 = ctrl_const.use_q44_ar1;
        else
            use_q44_ar1 = false;
        end
        % use_exact_fe44 (chat 2026-07-08): exact AR(1) row-4 pole for the
        % COVARIANCE propagation, F_e(4,4) = lc + a'*F_dx[k] with the known-wall
        % slope a' = -a_hat*K_h/R (keeps the F_dx*e_ax feedthrough; see
        % 4state_del_hd.tex p.7). Mean predict UNCHANGED (feedthrough vanishes
        % at the estimate). Only active with use_q44_ar1.
        if isfield(ctrl_const, 'use_exact_fe44') && ~isempty(ctrl_const.use_exact_fe44)
            use_exact_fe44 = ctrl_const.use_exact_fe44;
        else
            use_exact_fe44 = false;
        end
        % use_taylor_gain (chat 2026-07-10): Taylor-gain full suite
        % (4state_del_hd.tex taylor section). z axis: predict slot 4 adds a'*dh_d +
        % (1-lc)*a'*dxhat3, F_e row 4 = [0 0 (1-lc)a' 1+a'F_dx], rank-1
        % Q(3:4,3:4) (q3 = -eps and q4 = a'*eps share the same sample).
        % x/y axes: the gain argument is the wall-normal height, so z's dxhat3
        % enters their predict as a known exogenous input; row 4 stays
        % [0 0 0 1], Q34 = 0, Q44 = a'_i^2 * Q33_z. Default off -> bit-identical.
        if isfield(ctrl_const, 'use_taylor_gain') && ~isempty(ctrl_const.use_taylor_gain)
            use_taylor_gain = ctrl_const.use_taylor_gain;
        else
            use_taylor_gain = false;
        end
        % aprime_source: 'known' -a_det(h_bar_d)*K_h(h_bar_d)/R (oracle level) |
        %                'ahat'  -a_hat*K_h(h_bar_d)/R (level self-anchored)   |
        %                'diff'  per-step self-diff of a_hat vs h_d, gate +
        %                        EWMA (model-free, selfrw convention 2026-07-07)
        if isfield(ctrl_const, 'aprime_source') && ~isempty(ctrl_const.aprime_source)
            aprime_source = lower(ctrl_const.aprime_source);
        else
            aprime_source = 'known';
        end
        if use_taylor_gain
            assert(any(strcmp(aprime_source, {'known', 'ahat', 'diff', 'state', 'kfmeas', 'varchan', 'varchan_centered'})), ...
                   'motion_control_law_eq17_4state:badAprimeSource', ...
                   'aprime_source must be ''known''|''ahat''|''diff''|''state''|''kfmeas''|''varchan''|''varchan_centered'', got ''%s''.', aprime_source);
            assert(~use_q44_ar1 && ~use_q44_cap && ~use_aprime_ff && ~use_deblur, ...
                   'motion_control_law_eq17_4state:taylorComboUnsupported', ...
                   'use_taylor_gain cannot combine with use_q44_ar1/use_q44_cap/use_aprime_ff/use_deblur.');
        end
        % --- 5state a'-as-state knobs (5state_taylor_aprime.pdf) ---
        %   Only aprime_source='state' promotes a' to a 5th state (z axis only;
        %   x/y keep an inert slot 5). Everything else stays 4-dimensional.
        if isfield(ctrl_const, 'aprime_state_kappa') && ~isempty(ctrl_const.aprime_state_kappa)
            ap_kappa = ctrl_const.aprime_state_kappa;
        else
            ap_kappa = 1;                          % O(1) a'' bound scale in Q55
        end
        if isfield(ctrl_const, 'aprime_state_P0') && ~isempty(ctrl_const.aprime_state_P0)
            ap_P55_0 = ctrl_const.aprime_state_P0;
        else
            % NEW default (chat 2026-07-22): 100x smaller variance (10x smaller std)
            % than the old (0.1*a_nom/R)^2 -> confident far-field a'~0 prior,
            % smaller onset dump. Old value = (0.1*a_nom/R)^2.
            ap_P55_0 = (0.01 * a_nom_p / R_radius)^2;   % default a' prior variance
        end
        if isfield(ctrl_const, 'aprime_learn_t0') && ~isempty(ctrl_const.aprime_learn_t0)
            ap_learn_t0 = ctrl_const.aprime_learn_t0;  % [s] freeze state 5 until t >= this
        else
            ap_learn_t0 = 0;
        end
        % TEMP (chat 2026-07-19, AG): Q55 floor (PLACEHOLDER pending derivation).
        % Q55_z = max(computed Q55_z, floor) whenever the learn gate is NOT
        % active (kept 0 during the gate). Gives the a'-state a minimum
        % process-noise "forgetting" so P55 cannot collapse (osc) / re-opens
        % during low-excitation holds. Default 0 -> bit-identical no-op.
        if isfield(ctrl_const, 'aprime_state_Q55_floor') && ~isempty(ctrl_const.aprime_state_Q55_floor)
            ap_Q55_floor = ctrl_const.aprime_state_Q55_floor;
        else
            ap_Q55_floor = 0;
        end
        % TEMP (chat 2026-07-17, E1SO): complete the F(4,5) Jacobian. The
        % predict mean uses slot 5 twice (a'*Dh_d sweep FF + (1-lc)*a'*dxhat3
        % deviation correction) but F(4,5) linearizes only the first term.
        % selfmod adds the second -> thermal-fluctuation coupling keeps a'
        % weakly observable during holds (selfmod line, chat 2026-06-27).
        if isfield(ctrl_const, 'aprime_state_selfmod') && ~isempty(ctrl_const.aprime_state_selfmod)
            ap_selfmod = logical(ctrl_const.aprime_state_selfmod);
        else
            ap_selfmod = false;
        end
        % NEW (chat 2026-07-20, kfmeas): a'-state init scale (slot-5 init =
        % scale*a'_oracle(h_init); default 0 = 'state' backward-compat, a'=0) and
        % predict-only level freeze (zeroes K(4,:) on z for TEST 2).
        if isfield(ctrl_const, 'aprime_state_init_scale') && ~isempty(ctrl_const.aprime_state_init_scale)
            ap_state_init_scale = ctrl_const.aprime_state_init_scale;
        else
            ap_state_init_scale = 0;
        end
        if isfield(ctrl_const, 'freeze_level_kf') && ~isempty(ctrl_const.freeze_level_kf)
            freeze_level_kf = logical(ctrl_const.freeze_level_kf);
        else
            freeze_level_kf = false;
        end
        % TEMP (chat 2026-07-22): C2 deterministic level channel (kf_meas.tex §9).
        % Third measurement z3 = a_d, extracted from the deterministic tracking
        % residual dx_bar_m: y3 = dx_bar_m[k]-lc*dx_bar_m[k-1] = -F_dh*e_ad + nu3,
        % z3 = a_d_pred - y3/F_dh_det, H3 = [0 0 0 1 0], R3 = knu3*sigma2_dxr/F_dh^2
        % (route C = white). Gate |F_dh_det| < c2_Fmin. z axis only. Default off.
        if isfield(ctrl_const, 'use_c2') && ~isempty(ctrl_const.use_c2)
            use_c2 = logical(ctrl_const.use_c2);
        else
            use_c2 = false;
        end
        if isfield(ctrl_const, 'c2_Fmin') && ~isempty(ctrl_const.c2_Fmin)
            c2_Fmin = ctrl_const.c2_Fmin;
        else
            c2_Fmin = 0.6;                          % [pN] applied-force gate (prior C2 T3 optimum)
        end
        if isfield(ctrl_const, 'c2_knu3') && ~isempty(ctrl_const.c2_knu3)
            c2_knu3 = ctrl_const.c2_knu3;
        else
            c2_knu3 = 0.004;                        % sigma2_nu3 = knu3*sigma2_dxr (LP+diff of dx_r)
        end
        if isfield(ctrl_const, 'c2_r3_scale') && ~isempty(ctrl_const.c2_r3_scale)
            c2_r3_scale = ctrl_const.c2_r3_scale;
        else
            c2_r3_scale = 1.0;                      % R3 multiplier (verification sweep knob)
        end
        % C2 multi-window: c2_window_s > 0 -> weighted-LS batch over W steps
        % (weight F_dh^2 handles zero-crossings), one measurement per window.
        % 0 -> per-step (F_dh gate). Window should stay << osc period.
        if isfield(ctrl_const, 'c2_window_s') && ~isempty(ctrl_const.c2_window_s)
            c2_window_s = ctrl_const.c2_window_s;
        else
            c2_window_s = 0;
        end
        c2_Wsteps = max(1, round(c2_window_s / Ts));
        c2_SFy = 0; c2_SFF = 0; c2_count = 0;
        n_aug = 4;
        if use_taylor_gain && (strcmp(aprime_source, 'state') || strcmp(aprime_source, 'kfmeas'))
            n_aug = 5;                             % z-axis a'-as-state active
        end
        ap_on = (n_aug == 5);                      % a'-state flag (slot 5 fixed)
        % TEMP (chat 2026-07-20, honesty audit): colored-y2 AR(1) v-state port
        % from production 44349d0. a_xm estimator noise is EWMA-colored
        % (measured innovation rho(1)=0.986 = derived YW lag-1 0.9864; booked
        % as white -> IF~45x info recount -> P44/P55 honesty 0.11-0.13).
        % 'ar1' appends a per-axis noise state v as the LAST slot:
        % v[k+1] = phi_v*v[k] + w_v, H row 2 = [0 0 0 1 (0) 1],
        % Qvv = (1-phi^2)(1-r2f)*R2_intrinsic[k], R22 = r2f*R2_int + delay.
        if isfield(ctrl_const, 'y2_noise_model') && ~isempty(ctrl_const.y2_noise_model)
            assert(any(strcmpi(ctrl_const.y2_noise_model, {'white', 'ar1'})), ...
                   'temp_eq17_4state_centered:badY2NoiseModel', ...
                   'y2_noise_model must be ''white''|''ar1''.');
            y2_ar1 = strcmpi(ctrl_const.y2_noise_model, 'ar1');
        else
            y2_ar1 = false;
        end
        if isfield(ctrl_const, 'y2_ar1_phi') && ~isempty(ctrl_const.y2_ar1_phi)
            phi_v = ctrl_const.y2_ar1_phi;
        else
            phi_v = 0.9864;   % YW lag-1 (lc=0.7, a_pd=a_cov=0.05), audit-confirmed 0.986
        end
        if isfield(ctrl_const, 'y2_ar1_r2_floor_frac') && ~isempty(ctrl_const.y2_ar1_r2_floor_frac)
            r2f = ctrl_const.y2_ar1_r2_floor_frac;
        else
            r2f = 0.05;       % white-remainder floor (production F2 default)
        end
        if y2_ar1
            n_aug  = n_aug + 1;
            v_slot = n_aug;                        % v = last slot (5 or 6)
        else
            v_slot = 0;
        end
        if ap_on && strcmp(aprime_source, 'kfmeas')
            assert(~y2_ar1, 'temp_eq17_4state_kfmeas:kfmeasNoAr1', ...
                   'aprime_source=''kfmeas'' is incompatible with y2_noise_model=''ar1''.');
        end
        if isfield(ctrl_const, 'aprime_diff_beta') && ~isempty(ctrl_const.aprime_diff_beta)
            ap_beta = ctrl_const.aprime_diff_beta;
        else
            ap_beta = 0.05;                     % EWMA weight (selfrw convention)
        end
        if isfield(ctrl_const, 'aprime_diff_gate_um') && ~isempty(ctrl_const.aprime_diff_gate_um)
            ap_gate_um = ctrl_const.aprime_diff_gate_um;
        else
            ap_gate_um = 1e-3;                  % [um] |dh_d| excitation gate (selfrw convention)
        end
        if isfield(ctrl_const, 'aprime_clamp_pos') && ~isempty(ctrl_const.aprime_clamp_pos)
            ap_clamp_pos = logical(ctrl_const.aprime_clamp_pos);
        else
            ap_clamp_pos = false;               % a'>0 post-EWMA clamp (chat 2026-07-10) off by default
        end
        if isfield(ctrl_const, 'aprime_pos_only') && ~isempty(ctrl_const.aprime_pos_only)
            ap_pos_only = logical(ctrl_const.aprime_pos_only);
        else
            ap_pos_only = false;                % input-side gate: negative raw skipped, no update
        end
        if isfield(ctrl_const, 'aprime_eval_xhat') && ~isempty(ctrl_const.aprime_eval_xhat)
            aprime_eval_xhat = logical(ctrl_const.aprime_eval_xhat);
        else
            aprime_eval_xhat = false;           % EXPERIMENT: default off (a' at p_d, production)
        end
        if isfield(ctrl_const, 'aprime_scale') && ~isempty(ctrl_const.aprime_scale)
            ap_scale = ctrl_const.aprime_scale;  % TEMP (chat 2026-07-15): E2 a'-freeze scale
        else
            ap_scale = 1;
        end
        if isfield(ctrl_const, 'g4_selfdiff_ar1') && ~isempty(ctrl_const.g4_selfdiff_ar1)
            g4_selfdiff_ar1 = logical(ctrl_const.g4_selfdiff_ar1);  % TEMP (chat 2026-07-15): G4 blue arm
        else
            g4_selfdiff_ar1 = false;
        end
        % TEMP (chat 2026-07-15): closed-loop var-channel a' (aprime_source='varchan').
        vc_tau  = 0.35;   % [s] a'_hd EWMA time constant
        vc_warm = 0.5;    % [s] warm-up freeze
        if isfield(ctrl_const, 'varchan_tau')  && ~isempty(ctrl_const.varchan_tau);  vc_tau  = ctrl_const.varchan_tau;  end
        if isfield(ctrl_const, 'varchan_warm') && ~isempty(ctrl_const.varchan_warm); vc_warm = ctrl_const.varchan_warm; end
        vc_ap_init_scale = 1;   % TEMP (chat 2026-07-16, E): scale the varchan a' init seed
        if isfield(ctrl_const, 'varchan_ap_init_scale') && ~isempty(ctrl_const.varchan_ap_init_scale)
            vc_ap_init_scale = ctrl_const.varchan_ap_init_scale;
        end
        % TEMP (chat 2026-07-17): fixed-step ignition guard. The numerator EWMA
        % is alive from step 1 (y2 prefill) while the denominator input dxhat3
        % is structurally silent for the first d steps (sensor-buffer IC) and
        % its EWMA needs ~1/a_cov samples of real content; until then the
        % sqrt-ratio divides real jitter by floating-point dust (1e12 spike).
        % Default 2 keeps the generic 2-step convention (previous behavior).
        vc_warm_steps = 2;
        if isfield(ctrl_const, 'varchan_warm_steps') && ~isempty(ctrl_const.varchan_warm_steps)
            vc_warm_steps = ctrl_const.varchan_warm_steps;
        end
        % TEMP (chat 2026-07-17, E100): charter-legal init basis. Mirrors the
        % production init_from_anom knob (motion_control_law_eq17_4state.m 0E):
        % gain seeds at bulk a_nom, wall shape K_h unknown (=0) at init, and the
        % varchan a' seed collapses to 0. p44_prior_frac breaks the resulting
        % Q44_ss=0 DARE degeneracy (P44=0, a_hat frozen) with an a_nom-tolerance
        % prior P44(0) >= (frac*a_nom)^2 ("a_nom as prior, not anchor").
        init_from_anom = false;
        if isfield(ctrl_const, 'init_from_anom') && ~isempty(ctrl_const.init_from_anom)
            init_from_anom = logical(ctrl_const.init_from_anom);
        end
        p44_prior_frac = [];
        if isfield(ctrl_const, 'p44_prior_frac') && ~isempty(ctrl_const.p44_prior_frac)
            p44_prior_frac = ctrl_const.p44_prior_frac;
        end
        vc_beta = 1 - Ts / vc_tau;
        vc_avar = a_cov; vc_acov = a_cov;             % a_var=a_cov=0.05 (same as V batch)
        vc_abar = 0; vc_dbar = 0; vc_s2ar = 0; vc_s2dh = 0; vc_ap_hat = 0;   % vc_ap_hat seeded in 0F
        a_prime_diff = zeros(3, 1);
        % NOTE: ctrl_const.suppress_xD is ignored (no disturbance term exists).

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
        if use_taylor_gain
            % Per-axis realization assumes the wall normal is the z axis
            % (x/y taylor coupling reads z's dxhat3 as the height error).
            assert(abs(w_hat_n(3) - 1) < 1e-12, ...
                   'motion_control_law_eq17_4state:taylorWallOrientation', ...
                   'use_taylor_gain requires w_hat = [0;0;1] (got [%g;%g;%g]).', ...
                   w_hat_n(1), w_hat_n(2), w_hat_n(3));
        end

        % --- 0E. Wall-aware a_x[0] seeding ---
        if enable_wall && isfield(params, 'common') && isfield(params.common, 'p0')
            p0_init    = params.common.p0(:);
            h_init_um  = dot(p0_init, w_hat_n) - pz_wall;
            h_bar_init = max(h_init_um / R_radius, 1.001);
            if init_from_anom
                % Charter-legal: wall position known but c(h_bar) unknown at
                % init -> bulk Stokes gain, zero wall-shape sensitivity.
                a_x_init    = [a_nom_p; a_nom_p; a_nom_p];
                K_h_init    = zeros(3, 1);
                a_perp_init = a_nom_p;
            else
                [c_para0, c_perp0, derivs0] = calc_correction_functions(h_bar_init, true);
                a_x_init   = [a_nom_p / c_para0; a_nom_p / c_para0; a_nom_p / c_perp0];
                K_h_init   = [derivs0.K_h_para; derivs0.K_h_para; derivs0.K_h_perp];
                a_perp_init = a_nom_p / c_perp0;
            end
        else
            a_x_init    = [a_nom_p; a_nom_p; a_nom_p];
            K_h_init    = zeros(3, 1);
            a_perp_init = a_nom_p;
        end

        % --- 0F. EKF state init (a_x in slot 4; rest zero) ---
        % TEMP (chat 2026-07-16, D2): deliberately-wrong gain-state init scale to
        % watch the y2 measurement restoring force pull a_hat back (default 1).
        if isfield(ctrl_const, 'a_hat_init_scale') && ~isempty(ctrl_const.a_hat_init_scale)
            a_hat_init_scale = ctrl_const.a_hat_init_scale;
        else
            a_hat_init_scale = 1;
        end
        x_e_per_axis = zeros(n_aug, 3);
        x_e_per_axis(4, :) = (a_hat_init_scale * a_x_init).';   % slot 4 = a_x/a_d; slot 5 (if present) = a'
        if ap_on
            % slot 5 (a') init: scale * oracle a'(h_init) on z; x/y stay 0.
            % scale=0 (default) -> 'state' backward-compat (learn from 0);
            % scale=1 correct init; scale=2 wrong (2x) for the recovery test.
            ap_oracle_init = -a_perp_init * K_h_init(3) / R_radius;   % a'(h_init) z [um/pN/um]
            x_e_per_axis(5, 3) = ap_state_init_scale * ap_oracle_init;
        end

        % TEMP (chat 2026-07-15): seed the closed-loop var-channel a' at a'(h_d[0])
        vc_ap_hat = vc_ap_init_scale * (-a_perp_init * K_h_init(3) / R_radius);   % a'(h_d[0]) z-axis (E init-scale)
        vc_abar   = a_x_init(3);                             % a_hat_z EWMA mean seed
        vc_dbar   = 0;                                       % dxhat3_z EWMA mean seed
        % TEMP (chat 2026-07-16): centered maintained reference seed (far-field a)
        vc_adet      = a_x_init(3);
        vc_adet_hist = a_x_init(3) * ones(d_delay + 1, 1);

        % --- 0G. Riccati Pf (DARE steady state at h_init, positioning f_d=0) ---
        one_minus_lc = 1 - lambda_c;
        F_e_ss = build_F_e_4state(lambda_c, 0, 0);
        H_ss   = [1 0 0 0; 0 0 0 1];
        sigma2_dh_init = 4 * kBT * a_perp_init;            % wall-normal thermal motion var
        var_da_init_vec = zeros(3, 1);
        P_per_axis = cell(3, 1);
        for ax = 1:3
            a_init_ax   = a_x_init(ax);
            var_da_init = var_da_inc_factor * (a_init_ax * K_h_init(ax) / R_radius)^2 * sigma2_dh_init;
            var_da_init_vec(ax) = var_da_init;
            % Q33 at f_d=0 limit: full epsilon (thermal history + n_x feedthrough)
            Q33_ss = 4 * kBT * a_init_ax * (1 + one_minus_lc^2 * d_delay) ...
                     + one_minus_lc^2 * sigma2_n_s(ax);
            Q_ss = zeros(4);
            Q_ss(3, 3) = Q33_ss;
            Q_ss(4, 4) = var_da_init;                      % Q on a_x slot (= 5-state Q44)
            IF_ss  = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a_init_ax, sigma2_n_s(ax));
            R2_int_ss = amlpf_var_factor * K_var * IF_ss * (a_init_ax + xi_per_axis(ax))^2;
            if y2_ar1
                % colored-y2: DARE on the (core+v) system; v carries (1-r2f)
                % of the intrinsic variance, R22 keeps the F2 white floor +
                % telescoped delay remainder.
                F_ss_use = blkdiag(F_e_ss, phi_v);
                H_ss_use = [H_ss, [0; 1]];
                Q_ss_use = blkdiag(Q_ss, (1 - phi_v^2) * (1 - r2f) * R2_int_ss);
                R22_ss   = r2f * R2_int_ss + r22_delay_factor * d_delay * var_da_init;
            else
                F_ss_use = F_e_ss; H_ss_use = H_ss; Q_ss_use = Q_ss;
                R22_ss = R2_int_ss ...
                         + r22_delay_factor * d_delay * var_da_init;   % telescoped delay-sum (audit 2026-07-10)
            end
            R_ss = [sigma2_n_s(ax), 0; 0, R22_ss];
            P_dare = solve_dare_kf_local(F_ss_use, H_ss_use, Q_ss_use, R_ss);
            if ~isempty(p44_prior_frac)
                % a_nom-tolerance prior floor on the gain-slot covariance
                % (under init_from_anom the strict DARE gives P44 -> 0).
                P_dare(4, 4) = max(P_dare(4, 4), (p44_prior_frac * a_nom_p)^2);
            end
            % Embed into the full n_aug: core 1:4; v (if on) at v_slot (last,
            % = DARE index 5); a' (if on) seeded diffuse at slot 5 (learned,
            % not at steady state).
            P_full = zeros(n_aug);
            if y2_ar1
                core_idx = [1:4, v_slot];
            else
                core_idx = 1:4;
            end
            P_full(core_idx, core_idx) = P_dare;
            if ap_on
                P_full(5, 5) = ap_P55_0;
            end
            P_per_axis{ax} = P_full;
        end

        % --- 0H. IIR states (prefill default) ---
        dx_bar_m = zeros(3, 1);
        F_dx_det = zeros(3, 1);                    % C2: LP of F_dh (deterministic applied force)
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
        kappa_hat = ones(3, 1);
        a_m_det = a_x_init;

        % --- 0I. Delay buffers ---
        pd_km1  = pd;
        pd_km2  = pd;
        f_d_km1 = zeros(3, 1);
        f_d_km2 = zeros(3, 1);
        a_hat_km1 = a_x_init;             a_hat_km2 = a_x_init;
        a_ctrl_km1 = a_x_init;            a_ctrl_km2 = a_x_init;
        var_da_inc_km1 = var_da_init_vec; var_da_inc_km2 = var_da_init_vec;

        % --- 0K. Misc ---
        k_step = 1;
        R_OFF  = 1e10;

        % --- 0L. First call returns zeros (no f_d yet) ---
        f_d = zeros(3, 1);
        ekf_out = [a_x_init(1); a_x_init(3); a_x_init(2); 0];
        if nargout >= 3
            diag = empty_diag_4state();
            diag.f_d            = f_d;
            diag.a_hat          = a_x_init;
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
    a_hat = x_e_per_axis(4, :).';     % 3x1 [um/pN]  (slot 4 = a_x, or a_d on z-kfmeas)

    % NEW (kfmeas): slot 4 on z is a_d (desired-height gain); the control law
    % needs the ACTUAL-height gain a_x = a_d - a'*dxhat3.
    kfmeas_on = ap_on && strcmp(aprime_source, 'kfmeas');

    if has_override
        a_ctrl = a_ctrl_override;        % true-gain (or externally supplied) gain
    else
        a_ctrl = a_hat;                  % normal mode: EKF posterior[k-1]
        if kfmeas_on
            a_ctrl(3) = a_hat(3) - x_e_per_axis(5, 3) * x_e_per_axis(3, 3);   % a_d - a'*dxhat3
        end
    end

    % delta_x_m[k] = p_d[k-d] - p_m[k]
    if d_delay == 2
        pd_km_d = pd_km2;
    elseif d_delay == 1
        pd_km_d = pd_km1;
    else
        error('motion_control_law_eq17_4state:unsupportedDelay', ...
              'Only d=1 or d=2 supported, got d=%g.', d_delay);
    end
    delta_x_m = pd_km_d - p_m;           % 3x1 [um]
    pd_kp1    = pd + del_pd;             % 3x1 [um]

    % h_bar from current measurement (Guard 3 + wall functions; same as 5-state)
    if enable_wall
        h_bar = (dot(p_m, w_hat_n) - pz_wall) / R_radius;
    else
        h_bar = Inf;
    end

    one_minus_lc = 1 - lambda_c;

    % ------------------------------------------------------------------
    % Gain feedforward from the DESIRED trajectory (exogenous: uses p_d, NOT
    % the estimate). a_x_det[m] = a_nom / c(h_bar_d[m]).
    %   da_x_pred = a_x_det[k] - a_x_det[k-1]        -> predict increment
    %   sum_da_ff = a_x_det[k] - a_x_det[k-d]        -> measurement correction
    %               (= Sum_{i=1}^d delta_a_x[k-i], telescoped)
    % ------------------------------------------------------------------
    a_det_k   = local_a_x_det(pd,     w_hat_n, pz_wall, R_radius, enable_wall, a_nom_p);
    a_det_km1 = local_a_x_det(pd_km1, w_hat_n, pz_wall, R_radius, enable_wall, a_nom_p);
    da_x_pred = a_det_k - a_det_km1;     % 3x1 [um/pN]
    if d_delay == 2
        a_det_km2 = local_a_x_det(pd_km2, w_hat_n, pz_wall, R_radius, enable_wall, a_nom_p);
        sum_da_ff = a_det_k - a_det_km2; % 3x1
        a_det_kmd = a_det_km2;           % gain d-steps ago (de-blur normalization)
    else
        sum_da_ff = a_det_k - a_det_km1; % d=1
        a_det_kmd = a_det_km1;
    end

    % ------------------------------------------------------------------
    % Wall functions at measured h_bar (Q44/R22 random part; avoids bias loop)
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
    % [1] IIR a_xm (paper 2025 Eq.9-13) -- unchanged from 5-state
    % ------------------------------------------------------------------
    if y2_ar1 && (k_step <= d_delay)
        % Init-correct IIR hold (production, chat 2026-07-13): the first d
        % samples of delta_x_m are buffer-init artifacts (dx_r ~ 0 exactly);
        % feeding them drags sigma2_dxr_hat down ~d*a_cov and the ar1 level
        % learner reads the low a_xm as signal. Hold the prefill instead.
        dx_bar_m_new = dx_bar_m;
        dx_r = delta_x_m - dx_bar_m_new;
        sigma2_dxr_hat_new = sigma2_dxr_hat;
    else
        dx_bar_m_new = (1 - a_pd) * dx_bar_m + a_pd * delta_x_m;
        dx_r = delta_x_m - dx_bar_m_new;
        sigma2_dxr_hat_new = (1 - a_cov) * sigma2_dxr_hat + a_cov * dx_r.^2;
    end
    % C2 (kf_meas.tex §9): extract the deterministic level-error forcing from the
    % LP tracking residual. y3 = dx_bar_m[k] - lc*dx_bar_m[k-1] = -F_dh*e_ad + nu3.
    % dx_bar_m here is still [k-1] (pre-shift); dx_bar_m_new is [k].
    y3_c2 = dx_bar_m_new - lambda_c * dx_bar_m;   % 3x1 [um]

    a_xm = (sigma2_dxr_hat_new - C_n * sigma2_n_s) / (C_dpmr * 4 * kBT);   % 3x1 [um/pN]

    % --- de-blur (uses a' shape): normalize each squared residual by the KNOWN
    %     gain d-steps ago (a_det_kmd, the gain that generated dx_r), EWMA the
    %     shape-removed ratio kappa = a_true/a_det, then reconstruct at a_det_k.
    %     Removes the variance-window h-blur AND the d-step delay in one step
    %     (so no separate sum_da_ff correction is applied in the de-blur path).
    if use_deblur
        n_kappa       = (dx_r.^2 - C_n * sigma2_n_s) ./ (C_dpmr * 4 * kBT * a_det_kmd);
        kappa_hat_new = (1 - a_cov) * kappa_hat + a_cov * n_kappa;          % E = kappa (~1)
        a_xm_db       = kappa_hat_new .* a_det_k;                           % E = a_true[k]
    else
        kappa_hat_new = kappa_hat;
        a_xm_db       = a_xm;
    end

    a_m_det_new = (1 - a_det_lp) * a_m_det + a_det_lp * a_xm;   % 3x1 [um/pN]
    if use_am_lpf
        a_meas = a_m_det_new;
    elseif use_deblur
        a_meas = a_xm_db;
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
        kappa_hat      = kappa_hat_new;
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
            diag = empty_diag_4state();
            diag.f_d = f_d; diag.a_hat = a_hat_post;
            diag.a_ctrl_used = a_ctrl;
            diag.sigma2_dxr_hat = sigma2_dxr_hat_new; diag.a_xm = a_xm;
            diag.a_m_det = a_m_det_new;
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
    % [3b] Taylor-gain slope a' + desired height steps (4state_del_hd.tex taylor section)
    % ------------------------------------------------------------------
    if use_taylor_gain
        % Predict maps posterior[k-1] -> prior[k], so the gain-advance step is
        % the BACKWARD desired increment h_d[k]-h_d[k-1] (same convention as
        % da_x_pred). The d-step span aligns a_xm[k] (= a[k-d]) to a[k].
        dh_d_step  = dot(pd - pd_km1,  w_hat_n);      % h_d[k]   - h_d[k-1] [um]
        dH_d_dspan = dot(pd - pd_km_d, w_hat_n);      % h_d[k]   - h_d[k-d] [um]
        switch aprime_source
            case 'known'        % oracle: exogenous level (a_det) + known shape
                if aprime_eval_xhat
                    % EXPERIMENT (chat 2026-07-13): full a'(.) at the EKF-implied
                    % position x_hat = p_d - dxhat3_z (posterior [k-1]).
                    p_ap = pd - w_hat_n * x_e_per_axis(3, 3);
                    a_det_ap = local_a_x_det(p_ap, w_hat_n, pz_wall, R_radius, ...
                                             enable_wall, a_nom_p);
                    a_prime = local_a_prime_known(p_ap, w_hat_n, pz_wall, R_radius, ...
                                                  enable_wall, a_det_ap);
                else
                    a_prime = local_a_prime_known(pd, w_hat_n, pz_wall, R_radius, ...
                                                  enable_wall, a_det_k);
                end
            case 'ahat'         % known shape, level self-anchored to the estimate
                a_prime = local_a_prime_known(pd, w_hat_n, pz_wall, R_radius, ...
                                              enable_wall, a_hat);
            case {'state', 'kfmeas'}   % 5th-state slope (z, posterior[k-1]); x/y =
                                % known formula (their slot 5 is inert). 'kfmeas'
                                % updates a' via the y2 Jacobian H(2,5) (below);
                                % 'state' via the predict cross-coupling.
                a_prime = local_a_prime_known(pd, w_hat_n, pz_wall, R_radius, ...
                                              enable_wall, a_det_k);
                a_prime(3) = x_e_per_axis(5, 3);
            case 'varchan'      % TEMP (chat 2026-07-15): CLOSED-LOOP var-channel a'.
                                % Online var[k] EWMA chain on the entry (posterior
                                % [k-1]) a_hat_z and dxhat3_z, then a'_hd EWMA with
                                % warm-up freeze; feeds the z-axis a' (x/y stay
                                % known parallel). This is the a'-based feedforward
                                % path (predict/measurement use the 'otherwise'
                                % branch, so all 3 use points get a'_hd for z).
                ah_z  = a_hat(3);                 % posterior[k-1]
                dh3_z = x_e_per_axis(3, 3);       % posterior[k-1]
                vc_abar = (1 - vc_avar) * vc_abar + vc_avar * ah_z;   ar_v = ah_z  - vc_abar;
                vc_s2ar = (1 - vc_acov) * vc_s2ar + vc_acov * ar_v^2;
                vc_dbar = (1 - vc_avar) * vc_dbar + vc_avar * dh3_z;  dr_v = dh3_z - vc_dbar;
                vc_s2dh = (1 - vc_acov) * vc_s2dh + vc_acov * dr_v^2;
                if vc_s2dh > 0
                    var_k = sqrt(vc_s2ar / vc_s2dh);
                else
                    var_k = vc_ap_hat;
                end
                % fixed-step ignition guard (default 2 = generic convention)
                if k_step > vc_warm_steps && (k_step - 1) * Ts >= vc_warm
                    vc_ap_hat = vc_beta * vc_ap_hat + (1 - vc_beta) * var_k;
                end
                a_prime = local_a_prime_known(pd, w_hat_n, pz_wall, R_radius, ...
                                              enable_wall, a_det_k);
                a_prime(3) = vc_ap_hat;           % z-axis a' from the var channel
            case 'varchan_centered'  % TEMP (chat 2026-07-16): CLOSED-LOOP centered
                                % var-channel a' (5state_aprime_var_centered.tex C5).
                                % Maintained deterministic reference a_det (built
                                % from the current slope estimate) subtracted from
                                % the MEASUREMENT source a_xm (kappa=1 channel),
                                % d-step aligned; ahat'=sqrt(EWMA(s^2)/EWMA(dh_m^2)).
                dh_d_z = dot(pd - pd_km1, w_hat_n);          % h_d[k]-h_d[k-1] [um]
                vc_adet = vc_adet + vc_ap_hat * dh_d_z;      % maintain reference
                vc_adet_hist = [vc_adet_hist(2:end); vc_adet];
                adet_del = vc_adet_hist(1);                  % a_det[k-d]
                s_c = a_xm(3) - adet_del;                     % centered residual (measurement)
                vc_s2ar = (1 - vc_acov) * vc_s2ar + vc_acov * s_c^2;
                dhm_z   = delta_x_m(3);
                vc_s2dh = (1 - vc_acov) * vc_s2dh + vc_acov * dhm_z^2;
                if vc_s2dh > 0
                    var_k = sqrt(vc_s2ar / vc_s2dh);
                else
                    var_k = vc_ap_hat;
                end
                % fixed-step ignition guard (default 2 = generic convention)
                if k_step > vc_warm_steps && (k_step - 1) * Ts >= vc_warm
                    vc_ap_hat = vc_beta * vc_ap_hat + (1 - vc_beta) * var_k;
                end
                a_prime = local_a_prime_known(pd, w_hat_n, pz_wall, R_radius, ...
                                              enable_wall, a_det_k);
                a_prime(3) = vc_ap_hat;           % z-axis a' from the centered channel
            otherwise           % 'diff': model-free per-step self-diff, selfrw
                                % convention (chat 2026-07-07): backward raw
                                % slope (a_hat[k]-a_hat[k-1])/dh_d[k-1], gated
                                % by |dh_d| >= gamma, EWMA-smoothed with beta.
                dh_prev = dot(pd - pd_km1, w_hat_n);                  % h_d[k]-h_d[k-1] [um]
                if abs(dh_prev) >= ap_gate_um
                    raw = (a_hat - a_hat_km1) / dh_prev;              % 3x1 [um/pN per um]
                    if ap_pos_only
                        upd = raw >= 0;                               % input-side gate: skip negative raw
                        a_prime_diff(upd) = (1 - ap_beta) * a_prime_diff(upd) + ap_beta * raw(upd);
                    else
                        a_prime_diff = (1 - ap_beta) * a_prime_diff + ap_beta * raw;
                    end
                end                 % else: hold the previous slope (no excitation)
                if ap_clamp_pos
                    a_prime = max(a_prime_diff, 0);                   % a' > 0 prior (post-EWMA clamp)
                else
                    a_prime = a_prime_diff;
                end
        end
    else
        dh_d_step  = 0;
        dH_d_dspan = 0;
        a_prime    = zeros(3, 1);
    end

    % TEMP (chat 2026-07-15, E2): freeze the taylor-gain slope at scale x the
    % computed (oracle for 'known') value. All downstream uses -- predict
    % deviation coupling, F_e row 4, and the rank-1 Q(3:4) -- see the scaled a'.
    a_prime = ap_scale * a_prime;

    % ------------------------------------------------------------------
    % [4] Q (4x4 diagonal) and R (2x2) per axis
    %   Q33 = Var(epsilon); Q44 = var(delta_a_ram) (gain-level driving noise)
    % ------------------------------------------------------------------
    Q_per_axis = cell(3, 1);
    R_per_axis = cell(3, 1);
    gate_off   = false(3, 1);
    G_flags    = false(3, 3);
    var_da_ram = zeros(3, 1);   % Q44 driver (mode-dependent: RW / AR(1) / cap / taylor)
    var_da_inc = zeros(3, 1);   % true increment var Var(delta_a_ram), for Q33/R22
    Q33_vec    = zeros(3, 1);   % Var(epsilon) per axis (taylor Q needs Q33_z cross-axis)
    t_now = (k_step - 1) * Ts;
    learn_gate_on = ap_on && (t_now < ap_learn_t0);   % freeze a'-state (slot 5)
    for ax = 1:3
        a_hat_i = a_hat(ax);
        if use_q44_cap
            % Bounded-variance cap (4state_del_hd locate). The true gain
            % fluctuation a_x_ram is BOUNDED: Var(a_x_ram) = (a*K_h/R)^2*C_dx*sigma2_dh,
            % C_dx = 2+1/(1-lc^2). Setting Q44 = Var(a_x_ram)^2 / R22 makes the
            % random-walk steady state P_a = sqrt(Q44*R22) ~ Var(a_x_ram), so the
            % gain estimate does not wander beyond the true bounded fluctuation.
            C_dx_i    = 2 + 1 / (1 - lambda_c^2);
            Var_axram = (a_hat_i * K_h_axis(ax) / R_radius)^2 * C_dx_i * sigma2_dh;
            IF_cap    = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a_hat_i, sigma2_n_s(ax));
            R22_int   = amlpf_var_factor * K_var * IF_cap * (a_hat_i + xi_per_axis(ax))^2;
            var_da_ram(ax) = Var_axram^2 / R22_int;
        elseif use_q44_ar1
            % AR(1) reverting gain (F_e(4,4)=lc, predict reverts to a_det). Q44 is
            % the AR(1) innovation variance a'^2*Var(eps) = (3-2lc^2)(a*K_h/R)^2 sigma2_dh;
            % the F_e reversion (not a small Q) bounds the steady state to Var(a_x_ram).
            var_da_ram(ax) = (3 - 2*lambda_c^2) * (a_hat_i * K_h_axis(ax) / R_radius)^2 * sigma2_dh;
        elseif use_aprime_ff
            % (experimental, falsified) residual Q44 from the a'-feed-forward.
            var_da_ram(ax) = (a_hat_i * K_h_axis(ax) / R_radius)^2 ...
                             * 2 * one_minus_lc * P_per_axis{ax}(3, 3);
        else
            var_da_ram(ax) = var_da_inc_factor * (a_hat_i * K_h_axis(ax) / R_radius)^2 * sigma2_dh;
        end
        % True gain increment variance (always), for the Q33/R22 delay terms which
        % are functions of the REAL increment delta_a_ram, not the Q44 driver.
        var_da_inc(ax) = var_da_inc_factor * (a_hat_i * K_h_axis(ax) / R_radius)^2 * sigma2_dh;

        if d_delay == 2
            Q33_thermal  = 4 * kBT * (a_hat_i + one_minus_lc^2 * (a_hat_km1(ax) + a_hat_km2(ax)));
            Q33_randgain = one_minus_lc^2 * ( 4 * f_d_km1(ax)^2 * var_da_inc_km1(ax) ...
                                            + 1 * f_d_km2(ax)^2 * var_da_inc_km2(ax) );
        else
            Q33_thermal  = 4 * kBT * (a_hat_i + one_minus_lc^2 * a_hat_km1(ax));
            Q33_randgain = one_minus_lc^2 * (f_d_km1(ax)^2 * var_da_inc_km1(ax));
        end
        Q33_nx = one_minus_lc^2 * sigma2_n_s(ax);
        Q33_vec(ax) = Q33_thermal + Q33_randgain + Q33_nx;

        Q_i = zeros(4);
        Q_i(3, 3) = Q33_vec(ax);
        Q_i(4, 4) = var_da_ram(ax);                    % gain-level driving noise (slot 4)
        Q_per_axis{ax} = Q_i;

        R11_i = sigma2_n_s(ax);
        IF_eff_i = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a_hat_i, sigma2_n_s(ax));
        R2_intrinsic_i = amlpf_var_factor * K_var * IF_eff_i * (a_hat_i + xi_per_axis(ax))^2;
        if d_delay == 2
            % Telescoped delay-sum: var(delta_a_ram[k-1]+delta_a_ram[k-2]) =
            % 2*V_a*(1-rho2), not the independent 2-term sum (audit 2026-07-10).
            R22_delay_i = r22_delay_factor * (var_da_inc_km1(ax) + var_da_inc_km2(ax));
        else
            R22_delay_i = var_da_inc_km1(ax);       % d=1 single term is exact
        end
        if y2_ar1
            % colored-y2: (1-r2f) of the intrinsic variance rides the v state
            % (Qvv keeps its stationary variance tracked, time-varying with
            % a_hat); R22 keeps the F2 white floor + delay remainder.
            Q_per_axis{ax}(v_slot, v_slot) = (1 - phi_v^2) * (1 - r2f) * R2_intrinsic_i;
            R2_eff_i = r2f * R2_intrinsic_i + R22_delay_i;
        elseif kfmeas_on && ax == 3
            % kfmeas z: the d-step gain drift is modeled explicitly in h2
            % (via DH_d + dh1), NOT lumped into R. y2 = raw a_xm carries only
            % its intrinsic IIR variance -> drop the telescoped R22_delay term.
            R2_eff_i = R2_intrinsic_i;
        else
            R2_eff_i = R2_intrinsic_i + R22_delay_i;
        end

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

    % --- Taylor-gain rank-1 Q fixup (4state_del_hd.tex taylor section): q3 = -eps and
    %     q4 = a'*eps share the same sample -> Q(3:4,3:4) = s2eps*[1 -a'; -a' a'^2]
    %     on the z axis; x/y have q4 = a'_i*eps_z (independent of own q3), so
    %     Q34 = 0 and Q44 = a'_i^2 * Q33_z. Overrides the RW Q44 set above. ---
    if use_taylor_gain
        Q33_z = Q33_vec(3);
        for ax = 1:3
            var_da_ram(ax) = a_prime(ax)^2 * Q33_z;
            Q_per_axis{ax}(4, 4) = var_da_ram(ax);
        end
        Q_per_axis{3}(3, 4) = -a_prime(3) * Q33_z;
        Q_per_axis{3}(4, 3) = -a_prime(3) * Q33_z;
    end

    % --- a'-state process noise Q55 (5state_taylor_aprime.pdf):
    %     Q55[k] = kappa^2 * a_hat_z^2 / (h_d - h_wall)^4 * dh_d^2. Only the z
    %     axis has an active slope state; x/y slot 5 is inert (Q55 = 0). Frozen
    %     to 0 during the learn gate (Dh_d=0 => Q55=0 also holds during a hold). ---
    if ap_on
        hd_wall_dist = dot(pd, w_hat_n) - pz_wall;      % desired height above wall [um]
        Q55_z = 0;
        if ~learn_gate_on && hd_wall_dist > 0
            Q55_z = ap_kappa^2 * a_hat(3)^2 / hd_wall_dist^4 * dh_d_step^2;
        end
        % TEMP (chat 2026-07-19, AG): PLACEHOLDER forgetting floor (only when the
        % learn gate is open; kept 0 during the gate so the frozen a' stays exact).
        if ~learn_gate_on && ap_Q55_floor > 0
            Q55_z = max(Q55_z, ap_Q55_floor);
        end
        Q_per_axis{1}(5, 5) = 0;
        Q_per_axis{2}(5, 5) = 0;
        Q_per_axis{3}(5, 5) = Q55_z;
    end

    % ------------------------------------------------------------------
    % [5] EKF predict + update per axis
    % ------------------------------------------------------------------
    H_full = zeros(2, n_aug);   % [1 0 0 0 (0) (0); 0 0 0 1 (0) (1)] -- a' (slot 5) unobserved
    H_full(1, 1) = 1;
    H_full(2, 4) = 1;
    if y2_ar1
        H_full(2, v_slot) = 1;  % y2 = a_x + v + white remainder
    end
    H_y1   = H_full(1, :);

    K_a_y2_v  = zeros(3, 1);
    K_dx_y1_v = zeros(3, 1);
    innov_y2_v = zeros(3, 1);
    K_a_y1_v  = zeros(3, 1);        % L(4,1): y1 -> gain state (v-budget diag)
    K_dx_y2_v = zeros(3, 1);        % L(3,2): y2 -> dx3 state  (v-budget diag)
    innov_y1_v = zeros(3, 1);
    S_y1_v = zeros(3, 1);           % TEMP (chat 2026-07-20 honesty audit): predicted innovation var S(1,1)
    S_y2_v = zeros(3, 1);           % TEMP: S(2,2); 0 when y2 gated off

    % z-axis posterior[k-1] tracking-error estimate: the wall-normal deviation
    % that drives the taylor-gain deviation correction on ALL axes (w_hat = z).
    dxhat3_z_prev = x_e_per_axis(3, 3);

    for ax = 1:3
        % F_e (time-varying Row 3 via f_d history)
        if d_delay == 2
            F_1_i = f_d_km1(ax) + f_d_km2(ax);
        else
            F_1_i = f_d_km1(ax);
        end
        % 4-state F_e has no delta_a_x column, so dF_dx = (1-lc)*F_2 does not
        % appear; only F_1 (-> F_dx) is needed.
        if ap_on
            % 5-state a'-as-state (5state_taylor_aprime.pdf). z axis carries the
            % active slope with the sweep coupling F_e(4,5)=Dh_d; x/y keep an
            % inert slot 5 (row 4 = [0 0 0 1 0], row 5 = [0 0 0 0 1]).
            if ax == 3 && kfmeas_on
                % kfmeas z: Row 4 = [0 0 0 1 Dh_d] (a_d integrator + sweep
                % coupling only). a' enters the filter through the MEASUREMENT
                % (H2), NOT the predict -> fe43=0, a_pole=1.
                if learn_gate_on
                    fe45_i = 0;                 % freeze a'-state during learn gate
                else
                    fe45_i = dh_d_step;         % sweep coupling Dh_d[k]
                end
                F_e = build_F_e_5state(lambda_c, f_d(ax), F_1_i, 1, 0, fe45_i);
            elseif ax == 3
                F_dx_i   = f_d(ax) + one_minus_lc * F_1_i;
                fe43_i   = one_minus_lc * a_prime(3);
                a_pole_i = 1 + a_prime(3) * F_dx_i;
                if learn_gate_on
                    fe45_i = 0;                 % freeze a'-state during learn gate
                elseif ap_selfmod
                    % full Jacobian: sweep + deviation-correction couplings
                    fe45_i = dh_d_step + one_minus_lc * dxhat3_z_prev;
                else
                    fe45_i = dh_d_step;         % sweep coupling Dh_d[k]
                end
                F_e = build_F_e_5state(lambda_c, f_d(ax), F_1_i, a_pole_i, fe43_i, fe45_i);
            else
                F_e = build_F_e_5state(lambda_c, f_d(ax), F_1_i);   % x/y inert slot 5
            end
        elseif use_taylor_gain && ax == 3
            % Taylor-gain z row 4 = [0 0 (1-lc)a' 1+a'F_dx] (4state_del_hd.tex taylor section)
            F_dx_i   = f_d(ax) + one_minus_lc * F_1_i;
            fe43_i   = one_minus_lc * a_prime(3);
            a_pole_i = 1 + a_prime(3) * F_dx_i;
            if g4_selfdiff_ar1
                a_pole_i = lambda_c;   % TEMP G4 blue: AR1 revert pole for the covariance
            end
            F_e = build_F_e_4state(lambda_c, f_d(ax), F_1_i, a_pole_i, fe43_i);
        elseif use_q44_ar1
            if use_exact_fe44
                % exact pole lc + a'*F_dx, a' = -a_hat*K_h/R (known wall),
                % F_dx = f_d[k] + (1-lc)*sum f_d[k-i]
                a_prime_i = -a_hat(ax) * K_h_axis(ax) / R_radius;
                a_pole_i = lambda_c + a_prime_i * (f_d(ax) + (1 - lambda_c) * F_1_i);
            else
                a_pole_i = lambda_c;                                      % simplified (drops a'*F_dx)
            end
            F_e = build_F_e_4state(lambda_c, f_d(ax), F_1_i, a_pole_i);   % AR(1)
        else
            F_e = build_F_e_4state(lambda_c, f_d(ax), F_1_i);
        end
        if y2_ar1
            F_e = blkdiag(F_e, phi_v);   % v decoupled: v[k+1] = phi*v[k] + w_v
        end

        x_curr = x_e_per_axis(:, ax);
        P_curr = P_per_axis{ax};

        % --- Predict: deterministic map. Slot 4 (a_x) advances by the KNOWN
        %     feedforward increment da_x_pred (replaces the 5-state rate state). ---
        if use_aprime_ff
            % feed forward the predictable deviation-induced gain change:
            % -a'*Delta(dxhat3) = a'*(1-lc)*dxhat3,  a' = -a_hat*K_h/R.
            da_ram_pred_i = (a_hat(ax) * K_h_axis(ax) / R_radius) * (lambda_c - 1) * x_curr(3);
        else
            da_ram_pred_i = 0;
        end
        if use_taylor_gain
            % Taylor-gain predict: slope feedforward + (1-lc)*a'*dxhat3
            % deviation correction (dxhat3 = wall-normal = z-axis estimate for
            % all axes; for ax==3 this equals its own x_curr(3)).
            % Feedforward form per source (avoids the ~3.6%% Euler-integration
            % drift of a'*dh over the descent when the exact shape is known):
            %   known: exact a_det difference (= da_x_pred, telescopes exactly)
            %   ahat : shape-exact ratio, level self-anchored:
            %          a_hat*(a_det[k]/a_det[k-1] - 1) = a_hat*(c[k-1]/c[k] - 1)
            %   diff : a'*(h_d[k]-h_d[k-1]) (no shape available)
            switch aprime_source
                case 'known'
                    da_ff_i = da_x_pred(ax);
                case 'ahat'
                    da_ff_i = x_curr(4) * (a_det_k(ax) / a_det_km1(ax) - 1);
                otherwise
                    da_ff_i = a_prime(ax) * dh_d_step;
            end
            if kfmeas_on && ax == 3
                % kfmeas z: a_d advances by the DETERMINISTIC commanded sweep
                % only (a'*dh_d_step). No thermal deviation term -> no level
                % random walk. The thermal a_r part is carried by y2 (via dh1).
                x4_pred = x_curr(4) + da_ff_i;
            elseif g4_selfdiff_ar1
                % TEMP G4 blue: AR1 revert the LEVEL to the known a_det while a'
                % stays self-differenced (reconstructs the deleted selfrw
                % adet_known arm). da_ff_i (= a'*dh_d for 'diff') advances it.
                x4_pred = lambda_c * x_curr(4) + (1 - lambda_c) * a_det_k(ax) + da_ff_i ...
                          + one_minus_lc * a_prime(ax) * dxhat3_z_prev;
            else
                x4_pred = x_curr(4) + da_ff_i ...
                          + one_minus_lc * a_prime(ax) * dxhat3_z_prev;
            end
        elseif use_q44_ar1
            % AR(1) reverting gain: a_x reverts to a_det with pole lc.
            x4_pred = lambda_c * x_curr(4) + (1 - lambda_c) * a_det_k(ax) + da_x_pred(ax);
        else
            x4_pred = x_curr(4) + da_x_pred(ax) + da_ram_pred_i;
        end
        if ap_on
            % slot 5 (a') is a pure integrator in the mean (row 5 = [0 0 0 0 1]);
            % its wander is injected via Q55 only.
            x_pred = [x_curr(2); x_curr(3); lambda_c * x_curr(3); x4_pred; x_curr(5)];
        else
            x_pred = [x_curr(2); ...
                      x_curr(3); ...
                      lambda_c * x_curr(3); ...
                      x4_pred];
        end
        if y2_ar1
            x_pred(v_slot, 1) = phi_v * x_curr(v_slot);   % v: AR(1) decay
        end
        P_pred = F_e * P_curr * F_e' + Q_per_axis{ax};
        P_pred = 0.5 * (P_pred + P_pred');

        % --- Measurement: y_2 corrected by the KNOWN d-step gain drift so that
        %     the corrected a_xm directly measures a_x[k] (H row 2 = [0 0 0 1]). ---
        if use_deblur
            a_meas_corr = a_meas(ax);                    % de-blur already aligned to a_x[k]
        elseif use_taylor_gain
            % d-step gain-drift removal, source-consistent with the predict FF
            switch aprime_source
                case 'known'
                    a_meas_corr = a_meas(ax) + sum_da_ff(ax);
                case 'ahat'
                    a_meas_corr = a_meas(ax) + a_hat(ax) * (a_det_k(ax) / a_det_kmd(ax) - 1);
                otherwise
                    a_meas_corr = a_meas(ax) + a_prime(ax) * dH_d_dspan;
            end
        else
            a_meas_corr = a_meas(ax) + sum_da_ff(ax);    % a_xm + Sum_i delta_a_x[k-i]
        end

        if gate_off(ax)
            H_use = H_y1;
            y_use = delta_x_m(ax);
            R_use = sigma2_n_s(ax);
            y_pred = H_use * x_pred;
        elseif kfmeas_on && ax == 3
            % kfmeas z: bilinear y2 = a_d - a'*(DH_d + dh1) (nonlinear h2 for the
            % prediction; H2 Jacobian for the gain). a' is corrected DIRECTLY by
            % the y2 innovation via H2(5) = -(DH_d + dxhat1).
            DHd_z    = dH_d_dspan;                 % h_d[k]-h_d[k-d], scalar (wall-normal)
            dh1_pred = x_pred(1);
            ad_pred  = x_pred(4);
            ap_pred  = x_pred(5);
            h2_pred  = ad_pred - ap_pred * (DHd_z + dh1_pred);
            H2_row   = [-ap_pred, 0, 0, 1, -(DHd_z + dh1_pred)];   % 1x5 Jacobian
            H_use    = [H_y1; H2_row];             % 2x5
            y_use    = [delta_x_m(ax); a_xm(ax)];  % RAW a_xm (no sum_da_ff)
            R_use    = R_per_axis{ax};
            y_pred   = [x_pred(1); h2_pred];       % nonlinear prediction
            % --- C2 deterministic level channel (kf_meas.tex §9, route C white) ---
            if use_c2
                F_dx_z       = f_d(ax) + one_minus_lc * (f_d_km1(ax) + f_d_km2(ax));  % F_dh[k]
                F_dx_det(ax) = (1 - a_pd) * F_dx_det(ax) + a_pd * F_dx_z;             % endogeneity: LP
                c2_fire = false; z3_c2 = 0; R3_c2 = 0;
                if c2_window_s > 0
                    % multi-window weighted-LS: accumulate, emit one measurement / W.
                    % e_ad = -Sum(F_dh*y3)/Sum(F_dh^2); the F_dh^2 weight down-weights
                    % zero-crossings automatically (no hard gate needed).
                    c2_SFy = c2_SFy + F_dx_det(ax) * y3_c2(ax);
                    c2_SFF = c2_SFF + F_dx_det(ax)^2;
                    c2_count = c2_count + 1;
                    if c2_count >= c2_Wsteps && c2_SFF > 0
                        z3_c2   = x_pred(4) - c2_SFy / c2_SFF;                       % measures a_d
                        R3_c2   = c2_r3_scale * c2_knu3 * sigma2_dxr_hat_new(ax) / c2_SFF;
                        c2_fire = true;
                        c2_SFy = 0; c2_SFF = 0; c2_count = 0;
                    end
                elseif abs(F_dx_det(ax)) >= c2_Fmin
                    z3_c2   = x_pred(4) - y3_c2(ax) / F_dx_det(ax);                  % per-step
                    R3_c2   = c2_r3_scale * c2_knu3 * sigma2_dxr_hat_new(ax) / F_dx_det(ax)^2;
                    c2_fire = true;
                end
                if c2_fire
                    H_use  = [H_use; 0 0 0 1 0];
                    y_use  = [y_use; z3_c2];
                    R_use  = blkdiag(R_use, R3_c2);
                    y_pred = [y_pred; x_pred(4)];
                end
            end
        else
            H_use = H_full;
            y_use = [delta_x_m(ax); a_meas_corr];
            R_use = R_per_axis{ax};
            y_pred = H_use * x_pred;
        end

        innov  = y_use - y_pred;
        S_inn  = H_use * P_pred * H_use' + R_use;
        S_inn  = 0.5 * (S_inn + S_inn');
        K_kf   = (P_pred * H_use') / S_inn;

        % Warmup gate: freeze gain state during G1 (slot 4)
        if G_flags(1, ax)
            K_kf(4, :) = 0;
        end
        % NEW (kfmeas TEST 2): predict-only level test -- freeze the level
        % Kalman gain on z so slot 4 evolves by predict only (a' still updates).
        if freeze_level_kf && ax == 3
            K_kf(4, :) = 0;
        end
        % a'-state (slot 5): frozen for x/y always, and for z during the learn
        % gate (precedent: the G1 gain freeze zeroes K row 4).
        if ap_on && (ax ~= 3 || learn_gate_on)
            K_kf(5, :) = 0;
        end

        x_post = x_pred + K_kf * innov;
        ImKH   = eye(n_aug) - K_kf * H_use;
        P_post = ImKH * P_pred * ImKH' + K_kf * R_use * K_kf';   % Joseph form
        P_post = 0.5 * (P_post + P_post');

        % Diagnostics
        K_dx_y1_v(ax) = K_kf(3, 1);
        K_a_y1_v(ax)  = K_kf(4, 1);          % post-freeze (G1 zeroes row 4)
        innov_y1_v(ax) = innov(1);
        S_y1_v(ax) = S_inn(1, 1);
        if gate_off(ax)
            K_a_y2_v(ax)  = 0;
            K_dx_y2_v(ax) = 0;
            innov_y2_v(ax) = 0;
            S_y2_v(ax)    = 0;
        else
            K_a_y2_v(ax)  = K_kf(4, 2);
            K_dx_y2_v(ax) = K_kf(3, 2);
            innov_y2_v(ax) = innov(2);
            S_y2_v(ax)    = S_inn(2, 2);
        end

        x_e_per_axis(:, ax) = x_post;
        P_per_axis{ax} = P_post;
    end

    % ------------------------------------------------------------------
    % [6] Bookkeeping: shift delay buffers, IIR states, step counter
    % ------------------------------------------------------------------
    pd_km2 = pd_km1; pd_km1 = pd;
    f_d_km2 = f_d_km1; f_d_km1 = f_d;
    a_hat_km2 = a_hat_km1; a_hat_km1 = a_hat;   % entry posterior used during this step
                                                % (same object the taylor comments call posterior[k-1])
    a_ctrl_km2 = a_ctrl_km1; a_ctrl_km1 = a_ctrl;
    var_da_inc_km2 = var_da_inc_km1; var_da_inc_km1 = var_da_inc;
    dx_bar_m = dx_bar_m_new;
    sigma2_dxr_hat = sigma2_dxr_hat_new;
    kappa_hat = kappa_hat_new;
    a_m_det = a_m_det_new;
    k_step = k_step + 1;

    % ------------------------------------------------------------------
    % [7] Output
    % ------------------------------------------------------------------
    a_hat_post = x_e_per_axis(4, :).';
    a_d_post_z = x_e_per_axis(4, 3);          % z slot-4 posterior (a_d for kfmeas)
    if kfmeas_on
        % report the reconstructed ACTUAL-height gain a_x = a_d - a'*dxhat3 on z
        a_hat_post(3) = x_e_per_axis(4, 3) - x_e_per_axis(5, 3) * x_e_per_axis(3, 3);
    end
    h_bar_now = local_h_bar_out(enable_wall, h_bar);
    ekf_out = [a_hat_post(1); a_hat_post(3); a_hat_post(2); h_bar_now];

    if nargout >= 3
        P_a_v = zeros(3, 1); P_dx_v = zeros(3, 1); P_dx1_v = zeros(3, 1);
        P_aprime_v = zeros(3, 1);           % P(5,5): z only (zeros for x/y & 4-state)
        for ax = 1:3
            P_a_v(ax)   = P_per_axis{ax}(4, 4);
            P_dx_v(ax)  = P_per_axis{ax}(3, 3);
            P_dx1_v(ax) = P_per_axis{ax}(1, 1);
        end
        if ap_on
            P_aprime_v(3) = P_per_axis{3}(5, 5);
        end
        diag = empty_diag_4state();
        diag.sigma2_dxr_hat = sigma2_dxr_hat_new;
        diag.a_xm           = a_xm;
        diag.a_m_det        = a_m_det_new;
        diag.delta_x_m      = delta_x_m;
        diag.innovation_y2  = innov_y2_v;
        diag.S_y1           = S_y1_v;       % TEMP (honesty audit)
        diag.S_y2           = S_y2_v;
        diag.K_kf_a_y2      = K_a_y2_v;
        diag.K_kf_dx_y1     = K_dx_y1_v;
        diag.innovation_y1  = innov_y1_v;
        diag.K_kf_a_y1      = K_a_y1_v;
        diag.K_kf_dx_y2     = K_dx_y2_v;
        diag.P_a            = P_a_v;
        diag.P_dx           = P_dx_v;
        diag.x_D_hat              = zeros(3, 1);            % no disturbance state (driver compat)
        diag.delta_a_hat          = zeros(3, 1);           % no rate state (driver compat)
        diag.da_x_ff              = da_x_pred;             % feedforward increment used (predict)
        diag.a_x_det              = a_det_k;               % desired-trajectory gain at k
        diag.gate_active_per_axis = gate_off;
        diag.guards_individual    = G_flags;
        diag.h_bar                = h_bar;
        diag.f_d                  = f_d;
        diag.dx_r                 = dx_r;
        diag.a_hat                = a_hat_post;             % z = reconstructed a_x (kfmeas)
        diag.a_hat_d              = a_d_post_z;             % z slot-4 posterior (a_d for kfmeas) [scalar]
        diag.a_ctrl_used          = a_ctrl;
        diag.P77                  = zeros(3, 1);            % driver compat
        diag.Q77                  = zeros(3, 1);
        diag.var_da_ram           = var_da_ram;
        diag.delta_x_hat_1        = x_e_per_axis(1, :).';
        diag.delta_x_hat_3        = x_e_per_axis(3, :).';   % current tracking-error estimate
        diag.P_dx1                = P_dx1_v;
        diag.a_prime_used         = a_prime;                % taylor-gain slope (zeros if off)
        diag.P_aprime             = P_aprime_v;             % a'-state variance P(5,5) (z; zeros otherwise)
        % TEMP (chat 2026-07-17, E100 chain zoom): varchan internals (z axis,
        % post-update values of this step). vc_var_k is the raw statistic
        % a'_hdm[k] BEFORE the beta-EWMA (0 when source is not varchan*).
        diag.vc_s2ar  = vc_s2ar;                            % numerator EWMA sigma2_a_r
        diag.vc_s2dh  = vc_s2dh;                            % denominator EWMA sigma2_dh_r
        diag.vc_abar  = vc_abar;                            % a_hat_z mean tracker
        diag.vc_dbar  = vc_dbar;                            % dxhat3_z mean tracker
        diag.vc_ap_hat = vc_ap_hat;                         % slow-EWMA state a'_hd
        if exist('var_k', 'var')
            diag.vc_var_k = var_k;
        else
            diag.vc_var_k = 0;
        end
    end
end


%% =================== Local Helpers ===================

function F_e = build_F_e_4state(lambda_c, f_d_i, F_1_i, a_pole, fe43)
%BUILD_F_E_4STATE  4x4 error-dynamics matrix (per axis), rate state removed.
%   Row 3 = [0 0 lc -F_dx]   (cols: 1=dx1 2=dx2 3=dx3 4=a_x)
%       F_dx = f_d[k] + (1-lc)*F_1,   F_1 = sum_{i=1..d} f_d[k-i].
%   Row 4 = [0 0 fe43 a_pole]:
%     a_pole defaults to 1 (random walk; feedforward increment enters the
%     predict mean). Set a_pole=lc for the AR(1) reverting-gain model, or
%     a_pole = 1+a'*F_dx with fe43 = (1-lc)*a' for the taylor-gain model
%     (4state_del_hd.tex taylor section).
    if nargin < 4 || isempty(a_pole); a_pole = 1; end
    if nargin < 5 || isempty(fe43);   fe43 = 0;   end
    one_minus_lc = 1 - lambda_c;
    Fe3_a = -f_d_i - one_minus_lc * F_1_i;     % col 4 (a_x): -F_dx
    F_e = [0 1 0        0; ...
           0 0 1        0; ...
           0 0 lambda_c Fe3_a; ...
           0 0 fe43     a_pole];
end


function F_e = build_F_e_5state(lambda_c, f_d_i, F_1_i, a_pole, fe43, fe45)
%BUILD_F_E_5STATE  5x5 error-dynamics with a' promoted to state 5.
%   (5state_taylor_aprime.pdf). Cols: 1=dx1 2=dx2 3=dx3 4=a_x 5=a'_x.
%   Row 3 = [0 0 lc -F_dx 0]  (F_dx = f_d[k] + (1-lc)*F_1).
%   Row 4 = [0 0 fe43 a_pole fe45]:
%       fe43   = (1-lc)*a'      deviation coupling
%       a_pole = 1 + a'*F_dx    gain-error self-pole
%       fe45   = Dh_d[k]        sweep coupling (0 for x/y or during learn gate)
%   Row 5 = [0 0 0 0 1]        a' random walk (driven by Q55 only).
%   Defaults (x/y inert slot 5): a_pole=1, fe43=0, fe45=0 -> row 4 = [0 0 0 1 0].
    if nargin < 4 || isempty(a_pole); a_pole = 1; end
    if nargin < 5 || isempty(fe43);   fe43 = 0;   end
    if nargin < 6 || isempty(fe45);   fe45 = 0;   end
    one_minus_lc = 1 - lambda_c;
    Fe3_a = -f_d_i - one_minus_lc * F_1_i;     % col 4 (a_x): -F_dx
    F_e = [0 1 0        0      0; ...
           0 0 1        0      0; ...
           0 0 lambda_c Fe3_a  0; ...
           0 0 fe43     a_pole fe45; ...
           0 0 0        0      1];
end


function a_prime = local_a_prime_known(p_d, w_hat_n, pz_wall, R_radius, enable_wall, a_level)
%LOCAL_A_PRIME_KNOWN  Wall-model gain slope a' = -a_level * K_h(h_bar_d) / R.
%   Shape K_h evaluated at the DESIRED height (exogenous); the level comes
%   from the caller (a_det for 'known', a_hat for 'ahat'). [um/pN per um]
    if enable_wall
        h_bar_d = max((dot(p_d, w_hat_n) - pz_wall) / R_radius, 1.001);
        [~, ~, drv] = calc_correction_functions(h_bar_d, true);
        K_h_d = [drv.K_h_para; drv.K_h_para; drv.K_h_perp];
    else
        K_h_d = zeros(3, 1);
    end
    a_prime = -a_level .* K_h_d / R_radius;
end


function a_det = local_a_x_det(p_d, w_hat_n, pz_wall, R_radius, enable_wall, a_nom)
%LOCAL_A_X_DET  Desired-trajectory deterministic gain a_x_det = a_nom/c(h_bar_d).
%   Per axis: x,y use c_para; z uses c_perp (w_hat-normal). Exogenous (uses the
%   DESIRED position p_d, never the estimate). Returns a_nom for all axes when
%   the wall is disabled.
    if enable_wall
        h_bar_d = (dot(p_d, w_hat_n) - pz_wall) / R_radius;
        h_bar_d = max(h_bar_d, 1.001);
        [c_para_d, c_perp_d] = calc_correction_functions(h_bar_d);
        a_det = [a_nom / c_para_d; a_nom / c_para_d; a_nom / c_perp_d];
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


function d = empty_diag_4state()
%EMPTY_DIAG_4STATE  Zeroed diagnostic struct for 4-state controller.
    d = struct();
    d.sigma2_dxr_hat    = zeros(3, 1);
    d.a_xm              = zeros(3, 1);
    d.a_m_det           = zeros(3, 1);
    d.delta_x_m         = zeros(3, 1);
    d.innovation_y2     = zeros(3, 1);
    d.K_kf_a_y2         = zeros(3, 1);
    d.K_kf_dx_y1        = zeros(3, 1);
    d.innovation_y1     = zeros(3, 1);
    d.K_kf_a_y1         = zeros(3, 1);
    d.K_kf_dx_y2        = zeros(3, 1);
    d.P_a               = zeros(3, 1);
    d.P_dx              = zeros(3, 1);
    d.x_D_hat              = zeros(3, 1);   % no disturbance state (placeholder, driver compat)
    d.delta_a_hat          = zeros(3, 1);   % no rate state (placeholder, driver compat)
    d.da_x_ff              = zeros(3, 1);
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
    d.delta_x_hat_3        = zeros(3, 1);
    d.P_dx1                = zeros(3, 1);
    d.a_ctrl_used          = zeros(3, 1);
    d.a_prime_used         = zeros(3, 1);
    d.P_aprime             = zeros(3, 1);   % a'-state variance (driver compat)
    d.a_hat_d              = 0;             % z slot-4 posterior (a_d, kfmeas)
end
