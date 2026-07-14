function [f_d, ekf_out, diag] = motion_control_law_eq17_4state(del_pd, pd, p_m, params, ctrl_const, a_ctrl_override, a_true_k)
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
    if nargin < 7
        a_true_k = [];      % knife-2 oracle-y2 truth (chat 2026-07-14); [] when off
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
    persistent aprime_eval_xhat                                               % EXPERIMENT (chat 2026-07-13): a' at x_hat = p_d - dxhat3 instead of p_d
    persistent a_prime_diff                                                   % 'diff' EWMA slope state (selfrw convention)
    persistent y2_ar1 phi_v n_st r2f                                          % colored-y2 AR(1) augmentation + F2 floor (chat 2026-07-12)
    persistent use_c2 c2_phi c2_kappa c2_r3f Fbar_c2 FAbar_c2                 % C2 level channel y3 = p_md + v3 (chat 2026-07-13)
    persistent use_y2m y2m_sT y2m_cN                                          % y2 mirror-sensitivity H correction (chat 2026-07-13)
    persistent r22_adet                                                       % PROBE: R2_intrinsic eval at a_det (rectification test, chat 2026-07-13)
    persistent fe_eval_fdet                                                    % PROBE: build F_e from f_det instead of f_d (rectification test, chat 2026-07-13)
    persistent fdet_km1 fdet_km2                                              % C2 deterministic-force mirror recursion buffers
    persistent y2_oracle oracle_stream                                        % PROBE: knife-2 oracle y_2 measurement + isolated RNG (chat 2026-07-14)
    persistent a_true_km1 a_true_km2                                          % true-gain history for the oracle-y2 d-step delay
    persistent use_gamma_split sigma_gamma0 gamma_freeze_t0 gamma_freeze_t1   % z-axis ln level/shape split (gamma_split_draft.tex, chat 2026-07-14)
    persistent gamma_shared sigma_gamma0_vec shared_xy_y2_off                 % 3-axis shared-gamma federated fusion (gamma_split_draft §7, chat 2026-07-14)
    persistent gamma_hbar_min                                                % height gate: freeze gamma while h_bar_d < this (chat 2026-07-14)
    persistent init_from_anom                                                % a_nom-normalized init (c-unknown charter, chat 2026-07-14)
    persistent use_colored_eps m1_slot m2_slot                               % MA(2) thermal-history augmentation (colored_eps_draft.tex, chat 2026-07-14)
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
            assert(any(strcmp(aprime_source, {'known', 'ahat', 'diff'})), ...
                   'motion_control_law_eq17_4state:badAprimeSource', ...
                   'aprime_source must be ''known''|''ahat''|''diff'', got ''%s''.', aprime_source);
            assert(~use_q44_ar1 && ~use_q44_cap && ~use_aprime_ff && ~use_deblur, ...
                   'motion_control_law_eq17_4state:taylorComboUnsupported', ...
                   'use_taylor_gain cannot combine with use_q44_ar1/use_q44_cap/use_aprime_ff/use_deblur.');
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
        a_prime_diff = zeros(3, 1);
        % y2_noise_model (chat 2026-07-12): 'white' (default) keeps the current
        % behavior -- a_xm noise treated as white with R22 = intrinsic + delay.
        % 'ar1' augments a 5th state v per axis carrying the EWMA estimator
        % noise (AR(1), pole phi_v): H row 2 = [0 0 0 1 1], Q55 =
        % (1-phi^2)*R2_intrinsic[k] (time-varying), R22 white remainder =
        % delay term only. Fixes the KF over-harvesting of the colored y_2
        % (innovation rho(1) ~ 0.99 treated as white -> P44 ~20x overconfident).
        if isfield(ctrl_const, 'y2_noise_model') && ~isempty(ctrl_const.y2_noise_model)
            assert(any(strcmpi(ctrl_const.y2_noise_model, {'white', 'ar1'})), ...
                   'motion_control_law_eq17_4state:badY2NoiseModel', ...
                   'y2_noise_model must be ''white''|''ar1'', got ''%s''.', ...
                   ctrl_const.y2_noise_model);
            y2_ar1 = strcmpi(ctrl_const.y2_noise_model, 'ar1');
        else
            y2_ar1 = false;
        end
        if isfield(ctrl_const, 'y2_ar1_phi') && ~isempty(ctrl_const.y2_ar1_phi)
            phi_v = ctrl_const.y2_ar1_phi;
        else
            phi_v = 0.9864;      % lag-1 YW fit fallback (lc=0.7, a_pd=a_cov=0.05)
        end
        % F2 (chat 2026-07-12): white-remainder floor fraction. Splits the
        % intrinsic variance R2_int between the v state ((1-r2f), colored) and
        % the white R22 (r2f), total exactly R2_int. Prevents the R22 = 0
        % equality-constraint degeneracy far from the wall (K_h -> 0 makes the
        % delay term exactly zero: filter swallows y_2 whole; also the A4
        % RCOND ~ 1e-16 pathology). Engineering knob, not derived.
        if isfield(ctrl_const, 'y2_ar1_r2_floor_frac') && ~isempty(ctrl_const.y2_ar1_r2_floor_frac)
            r2f = ctrl_const.y2_ar1_r2_floor_frac;
        else
            r2f = 0.05;
        end
        % use_y2_mirror_h (chat 2026-07-13): derived y2 sensitivity correction.
        % The a_xm inversion assumes g = a/a_hat = 1; at g != 1 the closed
        % loop suppresses the very fluctuations being read, so
        %   dE[a_xm]/da |_{g=1} = s < 1   (s ~ 0.67 at lc=0.7, d=2, a_pd=0.05;
        % probe-verified slope -0.333). H(2,4) = 1 over-credits y2 by 1/s ->
        % level learning slower than booked + P44 overconfident by 1/s.
        % Fix: H(2,4) = s_i with the DERIVED per-axis sensitivity
        %   s_i = y2_mirror_sT + y2_mirror_cN * sigma2_n_i / (4*kBT*a_hat_i)
        % (constants from build_eq17_6state_constants compute_y2_mirror,
        % exact discrete-Lyapunov closed form -- no tuning). Default off.
        if isfield(ctrl_const, 'use_y2_mirror_h') && ~isempty(ctrl_const.use_y2_mirror_h)
            use_y2m = logical(ctrl_const.use_y2_mirror_h);
        else
            use_y2m = false;
        end
        if use_y2m
            assert(isfield(ctrl_const, 'y2_mirror_sT') && isfield(ctrl_const, 'y2_mirror_cN'), ...
                   'motion_control_law_eq17_4state:mirrorConstMissing', ...
                   'use_y2_mirror_h requires ctrl_const.y2_mirror_sT/_cN (rebuild constants).');
            y2m_sT = ctrl_const.y2_mirror_sT;
            y2m_cN = ctrl_const.y2_mirror_cN;
        else
            y2m_sT = 1;
            y2m_cN = 0;
        end
        % r22_eval_adet (PROBE, chat 2026-07-13): evaluate the R2_intrinsic
        % chain (IF_eff + R2_intrinsic, and Q55 in ar1 mode) at the ORACLE
        % a_det(h_bar_d) instead of the posterior a_hat. Breaks the
        % R22(a_hat) multiplicative feedback ("rectification") for diagnosis:
        % if the -3.5% ar1 bias vanishes, rectification is convicted.
        % Oracle wall knowledge -> diagnosis only, never production.
        if isfield(ctrl_const, 'r22_eval_adet') && ~isempty(ctrl_const.r22_eval_adet)
            r22_adet = logical(ctrl_const.r22_eval_adet);
        else
            r22_adet = false;
        end
        % fe_eval_fdet (PROBE, chat 2026-07-13): build the error-dynamics matrix
        % F_e (ONLY F_e) from the DETERMINISTIC mirror force f_det instead of the
        % actual noise-driven control force f_d. During hold f_d is pure noise
        % feedback (f_d correlates with the measurement noise), which correlates
        % the Kalman gain/covariance geometry with the innovations -> a
        % rectification of a_hat through F_e (not through R). If the -3.5% ar1
        % bias vanishes, the F_e endogeneity is convicted. Diagnosis only.
        if isfield(ctrl_const, 'fe_eval_fdet') && ~isempty(ctrl_const.fe_eval_fdet)
            fe_eval_fdet = logical(ctrl_const.fe_eval_fdet);
        else
            fe_eval_fdet = false;
        end
        % y2_oracle_meas (PROBE, chat 2026-07-14, "knife-2"): replace the real
        % IIR a_xm measurement with an ORACLE -- the TRUE delayed gain
        % a_true[k-d] plus synthetic white noise of the BOOKED variance
        % sqrt(R2_intrinsic[k]). The existing d-step drift correction still maps
        % [k-d] -> [k]. Isolates whether the real a_xm CONTENT is systematically
        % biased (hole-A) from the estimator machinery: if honest oracle content
        % removes the bias, the a_xm inversion content is convicted. Uses the
        % true gain (7th arg) + an ISOLATED RandStream so the global thermal /
        % measurement streams stay bit-identical to the knob-off baseline.
        % Oracle wall knowledge -> diagnosis only, never production. Default off.
        if isfield(ctrl_const, 'y2_oracle_meas') && ~isempty(ctrl_const.y2_oracle_meas)
            y2_oracle = logical(ctrl_const.y2_oracle_meas);
        else
            y2_oracle = false;
        end
        if isfield(ctrl_const, 'y2_oracle_seed') && ~isempty(ctrl_const.y2_oracle_seed)
            oracle_seed = ctrl_const.y2_oracle_seed;
        else
            oracle_seed = 7000000;
        end
        if y2_oracle
            % Independent stream (never draws from / seeds the global RNG).
            oracle_stream = RandStream('mt19937ar', 'Seed', oracle_seed);
        else
            oracle_stream = [];
        end
        % use_c2_level (chat 2026-07-13): C2 deterministic-residual level
        % channel. y_3 = p_md (= dx_bar_m, computed and previously discarded)
        % measures the gain level through the KNOWN force response:
        %   p_md ~ c3*a + b3 + v3,  c3 = -Fbar/(1-lc),  b3 = +FAbar/(1-lc)
        % with Fbar/FAbar the a_pd EWMAs of F_dx and F_dx*a_ctrl (same-filter
        % trick; both known signals). v3 is the colored p_md noise (6th state,
        % pole c2_phi; empirical D2: rho(1)=0.993, std ~9 nm during osc).
        % Requires y2_noise_model='ar1' (slot bookkeeping: 5=v2, 6=v3).
        if isfield(ctrl_const, 'use_c2_level') && ~isempty(ctrl_const.use_c2_level)
            use_c2 = logical(ctrl_const.use_c2_level);
        else
            use_c2 = false;
        end
        % use_c2 without 'ar1' is allowed as a CONTROL arm: with the white y_2
        % model P44 is dishonestly small, so y_3 is expected to be starved of
        % fusion weight (tests the honesty-is-a-prerequisite claim). v3 always
        % occupies the LAST state slot (5 without v2, 6 with v2).
        if isfield(ctrl_const, 'c2_phi') && ~isempty(ctrl_const.c2_phi)
            c2_phi = ctrl_const.c2_phi;
        else
            c2_phi = 0.993;      % empirical lag-1 rho of p_md noise (D2, z osc)
        end
        % c2_kappa_pmd: Var(n3) = kappa * (C_dpmr*4kBT*a_hat + C_n*sigma2_n)
        % (p_md noise tracks the same stationary dx_m scale as sigma2_dxr;
        %  kappa calibrated on f1Hz A2 white data: (9.08 nm)^2 / stat = 0.167).
        if isfield(ctrl_const, 'c2_kappa_pmd') && ~isempty(ctrl_const.c2_kappa_pmd)
            c2_kappa = ctrl_const.c2_kappa_pmd;
        else
            c2_kappa = 0.167;
        end
        if isfield(ctrl_const, 'c2_r3_floor_frac') && ~isempty(ctrl_const.c2_r3_floor_frac)
            c2_r3f = ctrl_const.c2_r3_floor_frac;
        else
            c2_r3f = 0.05;       % white floor split, same rationale as F2
        end
        % use_gamma_split (chat 2026-07-14, gamma_split_draft.tex): z-axis
        % log-gain level/shape split. Slot 4 becomes sigma = -ln c (per-axis
        % wall shape) and a new LAST slot gamma = ln(bulk level) is appended;
        % a_hat = exp(gamma + sigma). Only the z axis gets the real split; x/y
        % keep slot 4 = a_x (a-space) plus an INERT dummy gamma (zero coupling /
        % Q / H, F_e(g,g)=1) so their columns stay bit-identical. Requires
        % use_taylor_gain; incompatible with use_c2 (phase 1). Default off.
        if isfield(ctrl_const, 'use_gamma_split') && ~isempty(ctrl_const.use_gamma_split)
            use_gamma_split = logical(ctrl_const.use_gamma_split);
        else
            use_gamma_split = false;
        end
        if use_gamma_split
            assert(use_taylor_gain, ...
                   'motion_control_law_eq17_4state:gsplitNeedsTaylor', ...
                   'use_gamma_split requires use_taylor_gain.');
            assert(~use_c2, ...
                   'motion_control_law_eq17_4state:gsplitC2Unsupported', ...
                   'use_gamma_split cannot combine with use_c2_level (phase 1).');
        end
        % gamma_shared (chat 2026-07-14, gamma_split_draft §7): 3-axis SHARED
        % gamma via Carlson federated fusion. When on, x/y activate their own
        % ln-split (sigma_i z-driven, active gamma_i) and every step the three
        % gammas are information-fused and reset (federated approximation to the
        % joint arrowhead filter). Requires use_gamma_split. Default off ->
        % x/y stay inert, existing z-only gsplit UNCHANGED (bit-identical).
        if isfield(ctrl_const, 'gamma_shared') && ~isempty(ctrl_const.gamma_shared)
            gamma_shared = logical(ctrl_const.gamma_shared);
        else
            gamma_shared = false;
        end
        if gamma_shared
            assert(use_gamma_split, ...
                   'motion_control_law_eq17_4state:sharedNeedsGsplit', ...
                   'gamma_shared requires use_gamma_split.');
        end
        % shared_xy_y2_off (DIAGNOSTIC, chat 2026-07-14): force the x/y y_2
        % channel off (gate) while leaving z's y_2 intact, to isolate whether the
        % biased x/y a_xm is what drags the x/y sigma in the shared ln-split.
        % gamma_shared-only; default off.
        if isfield(ctrl_const, 'shared_xy_y2_off') && ~isempty(ctrl_const.shared_xy_y2_off)
            shared_xy_y2_off = logical(ctrl_const.shared_xy_y2_off);
        else
            shared_xy_y2_off = false;
        end
        % init_from_anom (chat 2026-07-14): seed ALL init quantities from the
        % bulk Stokes gain a_nom = Ts/gamma_N instead of the wall-aware
        % a_det(h_bar_init) = a_nom/c(h_bar_init). Charter-compliant: the wall
        % POSITION (h_bar_init) is known, but the drag correction c(h) is not,
        % so the init gain is the bulk a_nom and the wall shape (K_h) is unknown
        % at init. Only block 0E branches; every downstream init derives from it.
        % Default off -> wall-aware seed (bit-identical). See 0E for details.
        if isfield(ctrl_const, 'init_from_anom') && ~isempty(ctrl_const.init_from_anom)
            init_from_anom = logical(ctrl_const.init_from_anom);
        else
            init_from_anom = false;
        end
        % sigma_gamma0: diffuse gamma init std in ln-units (P_gamma(0)=sigma_gamma0^2).
        % Under init_from_anom (and no explicit override) it is DERIVED from the
        % known wall position in 0E as the leading-order |ln c| bound.
        if isfield(ctrl_const, 'sigma_gamma0') && ~isempty(ctrl_const.sigma_gamma0)
            sigma_gamma0 = ctrl_const.sigma_gamma0;
            sigma_gamma0_user = true;
        else
            sigma_gamma0 = 2.5;       % factor-e^2.5 diffuse; P_gamma(0)=6.25
            sigma_gamma0_user = false;
        end
        % gamma freeze WINDOW [gamma_freeze_t0, gamma_freeze_t1): the gamma
        % (level) Kalman-gain row is zeroed while t_now is inside the window
        % (descent gating, option ii). Defaults reproduce the original
        % "t < gamma_learn_start" gate EXACTLY: t0 = 0, t1 = gamma_learn_start
        % (which itself defaults to 0 -> empty window -> no gate). The
        % hold-learning variant sets t0 = t_hold, t1 = t_hold + t_descend so
        % gamma learns during the far-field hold, freezes only through descent.
        if isfield(ctrl_const, 'gamma_learn_start') && ~isempty(ctrl_const.gamma_learn_start)
            gamma_learn_start_bc = ctrl_const.gamma_learn_start;   % back-compat t1 default
        else
            gamma_learn_start_bc = 0;
        end
        if isfield(ctrl_const, 'gamma_freeze_t0') && ~isempty(ctrl_const.gamma_freeze_t0)
            gamma_freeze_t0 = ctrl_const.gamma_freeze_t0;
        else
            gamma_freeze_t0 = 0;
        end
        if isfield(ctrl_const, 'gamma_freeze_t1') && ~isempty(ctrl_const.gamma_freeze_t1)
            gamma_freeze_t1 = ctrl_const.gamma_freeze_t1;
        else
            gamma_freeze_t1 = gamma_learn_start_bc;
        end
        % gamma_hbar_min (chat 2026-07-14): near-wall HEIGHT gate. The gamma
        % (level) gain is also frozen whenever the DESIRED height h_bar_d falls
        % below this threshold, keeping the near-wall a_xm content burst out of
        % the level. h_bar_d (exogenous, from p_d) preserves the endogeneity
        % discipline. Default 0 -> never fires (bit-identical). Suggested 2.2 is
        % the documented near-wall a_xm content-deviation boundary (empirical
        % record: +14.6%% z content deviation for h_bar < 2, +34%% burst at
        % trough exit measured 2026-07-14, project_paper_nearwall_gap safe
        % region h_bar > 2.2); a first-principles derivation of the
        % content-deviation region remains an open obligation.
        if isfield(ctrl_const, 'gamma_hbar_min') && ~isempty(ctrl_const.gamma_hbar_min)
            gamma_hbar_min = ctrl_const.gamma_hbar_min;
        else
            gamma_hbar_min = 0;
        end
        % use_colored_eps (chat 2026-07-14, colored_eps_draft.tex): exact MA(2)
        % bookkeeping of the colored process noise eps. Appends TWO states per
        % axis, m1 = w[k-1], m2 = w[k-2] (w = a_x*f_T the per-step thermal kick),
        % so the lagged thermal kicks become DETERMINISTIC F_e coupling and only
        % the fresh white kick w[k] stays process noise. Requires use_taylor_gain;
        % incompatible with use_c2 (slot bookkeeping). Default off -> bit-identical.
        if isfield(ctrl_const, 'use_colored_eps') && ~isempty(ctrl_const.use_colored_eps)
            use_colored_eps = logical(ctrl_const.use_colored_eps);
        else
            use_colored_eps = false;
        end
        if use_colored_eps
            assert(use_taylor_gain, ...
                   'motion_control_law_eq17_4state:cepsNeedsTaylor', ...
                   'use_colored_eps requires use_taylor_gain.');
            assert(~use_c2, ...
                   'motion_control_law_eq17_4state:cepsC2Unsupported', ...
                   'use_colored_eps cannot combine with use_c2_level.');
        end
        % Slot order (fixed precedence after slot 4): v2, v3, m1, m2, gamma.
        % gamma stays LAST (== n_st) so all existing gsplit indexing is intact;
        % the m-states sit just before it.
        slot = 4;
        if y2_ar1; slot = slot + 1; end             % v2 = 5
        if use_c2; slot = slot + 1; end             % v3
        if use_colored_eps                          % m1, m2 (just before gamma)
            m1_slot = slot + 1;
            m2_slot = slot + 2;
            slot = slot + 2;
        else
            m1_slot = 0;
            m2_slot = 0;
        end
        if use_gamma_split; slot = slot + 1; end    % gamma = LAST slot
        n_st = slot;
        % gamma occupies the LAST slot (n_st) when use_gamma_split is on.
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

        % --- 0E. a_x[0] seeding: wall-aware c(h_bar_init) (default) OR a_nom bulk
        %     (init_from_anom). h_bar_init is the KNOWN wall position and stays
        %     legal (used for the derived sigma_gamma0). Under init_from_anom the
        %     drag correction c(h) and its shape K_h are UNKNOWN at init, so the
        %     gain seeds at the bulk Stokes a_nom and the wall-shape Q is zero.
        if enable_wall && isfield(params, 'common') && isfield(params.common, 'p0')
            p0_init    = params.common.p0(:);
            h_init_um  = dot(p0_init, w_hat_n) - pz_wall;
            h_bar_init = max(h_init_um / R_radius, 1.001);
            if init_from_anom
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
            h_bar_init  = Inf;
            a_x_init    = [a_nom_p; a_nom_p; a_nom_p];
            K_h_init    = zeros(3, 1);
            a_perp_init = a_nom_p;
        end
        % Derived diffuse-gamma init std under init_from_anom (charter-legal):
        % the init level error is gamma_true - gamma_hat = -ln c(h_bar_init).
        % The UNIVERSAL leading-order sphere-wall correction c_perp ~ 1 +
        % 9/(8*h_bar) gives |ln c| ~ 9/(8*h_bar_init) (Lorentz/lubrication --
        % this is NOT the repo c polynomial, so it is legal under the c-unknown
        % charter as an uncertainty BOUND). An optional hardware a_nom tolerance
        % (eta(T), R uncertainty in a_nom = Ts/(6*pi*eta*R)) would add in
        % quadrature; left at 0 by default. Explicit sigma_gamma0 overrides.
        if init_from_anom && use_gamma_split && ~sigma_gamma0_user && isfinite(h_bar_init)
            sigma_gamma0 = 9 / (8 * h_bar_init);
        end
        % Per-axis diffuse-gamma init std (shared-gamma): z uses c_perp leading
        % order 9/(8h), x/y use c_para leading order 9/(16h) (both universal
        % Lorentz terms). Defaults to the scalar sigma_gamma0 on all axes so the
        % z-only path is unchanged.
        sigma_gamma0_vec = [sigma_gamma0; sigma_gamma0; sigma_gamma0];
        if gamma_shared && init_from_anom && ~sigma_gamma0_user && isfinite(h_bar_init)
            sigma_gamma0_vec(1:2) = 9 / (16 * h_bar_init);
        end

        % --- 0F. EKF state init (a_x in slot 4; rest zero; slot 5 = v when ar1) ---
        x_e_per_axis = zeros(n_st, 3);
        x_e_per_axis(4, :) = a_x_init.';        % slot 4 = a_x (x/y keep; z reset below)
        if use_gamma_split
            % z gauge (f): sigma(0)=0, gamma(0)=ln(a_init) -> a_hat(0)=a_init.
            % x/y gamma dummy stays 0 (inert) unless gamma_shared (active ln-split).
            x_e_per_axis(4, 3)    = 0;
            x_e_per_axis(n_st, 3) = log(a_x_init(3));
            if gamma_shared
                x_e_per_axis(4, 1:2)    = 0;                       % sigma_x, sigma_y
                x_e_per_axis(n_st, 1:2) = log(a_x_init(1:2)).';    % gamma_x, gamma_y
            end
        end

        % --- 0G. Riccati Pf (DARE steady state at h_init, positioning f_d=0) ---
        %   The DARE runs on the pre-gsplit dimension nb = n_st - use_gamma_split
        %   (the a-space [dx1,dx2,dx3,a(,v2)] block, EXACTLY the knob-off system so
        %   x/y stay bit-identical). For z, the a-slot is re-read as sigma: the
        %   y_2 Jacobian becomes H(2,4)=a_init (da/dsigma) and Q_sigma = Q_a/a^2.
        %   gamma is then appended as a DIFFUSE row/col (excluded from the DARE:
        %   its stationary P would be the converged, NOT the diffuse-init value).
        one_minus_lc = 1 - lambda_c;
        nb = 4 + double(y2_ar1) + double(use_c2);   % DARE core dim (gamma + m appended)
        F_e_ss = build_F_e_4state(lambda_c, 0, 0);
        H_ss   = [1 0 0 0; 0 0 0 1];
        if y2_ar1
            F_e_ss = blkdiag(F_e_ss, phi_v);
            H_ss   = [1 0 0 0 0; 0 0 0 1 1];
        end
        if use_c2
            % y_3 row at the f_d = 0 seed point: c3 = 0 -> [0 ... 0 1]
            F_e_ss = blkdiag(F_e_ss, c2_phi);
            H_ss   = [H_ss, zeros(2, 1); zeros(1, size(H_ss, 2)), 1];
        end
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
            Q_ss = zeros(nb);
            Q_ss(3, 3) = Q33_ss;
            Q_ss(4, 4) = var_da_init;                      % Q on a_x slot (= 5-state Q44)
            IF_ss  = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a_init_ax, sigma2_n_s(ax));
            if y2_ar1
                R2_int_ss = amlpf_var_factor * K_var * IF_ss * (a_init_ax + xi_per_axis(ax))^2;
                Q_ss(5, 5) = (1 - phi_v^2) * (1 - r2f) * R2_int_ss;  % v stationary var = (1-r2f)*R2_int
                R22_ss = r2f * R2_int_ss + r22_delay_factor * d_delay * var_da_init;  % F2 floor + delay
            else
                R22_ss = amlpf_var_factor * K_var * IF_ss * (a_init_ax + xi_per_axis(ax))^2 ...
                         + r22_delay_factor * d_delay * var_da_init;   % telescoped delay-sum (audit 2026-07-10)
            end
            if use_c2
                var_n3_ss = c2_kappa * (C_dpmr * 4 * kBT * a_init_ax + C_n * sigma2_n_s(ax));
                Q_ss(nb, nb) = (1 - c2_phi^2) * (1 - c2_r3f) * var_n3_ss;
                R_ss = zeros(3);
                R_ss(1, 1) = sigma2_n_s(ax);
                R_ss(2, 2) = R22_ss;
                R_ss(3, 3) = c2_r3f * var_n3_ss;
            else
                R_ss = [sigma2_n_s(ax), 0; 0, R22_ss];
            end
            H_ss_ax = H_ss;
            if use_y2m
                H_ss_ax(2, 4) = y2m_sT + y2m_cN * sigma2_n_s(ax) / (4 * kBT * a_init_ax);
            end
            if use_gamma_split && (ax == 3 || gamma_shared)
                % ln-space seed (z always; x/y when shared): y_2 measures
                % a = exp(sigma+gamma), da/dsigma = a; variance in sigma = a/a^2.
                H_ss_ax(2, 4) = a_init_ax;
                Q_ss(4, 4)    = var_da_init / a_init_ax^2;
            end
            P_nb = solve_dare_kf_local(F_e_ss, H_ss_ax, Q_ss, R_ss);
            if use_gamma_split || use_colored_eps
                % append gamma (slot n_st, diffuse) and/or the m-states
                % (P_mm = sw(0) = 4kBT*a_init, zero cross; colored_eps_draft §5).
                P_full = zeros(n_st);
                P_full(1:nb, 1:nb) = P_nb;
                if use_gamma_split
                    P_full(n_st, n_st) = sigma_gamma0_vec(ax)^2;
                end
                if use_colored_eps
                    sw0 = 4 * kBT * a_x_init(ax);
                    P_full(m1_slot, m1_slot) = sw0;
                    P_full(m2_slot, m2_slot) = sw0;
                end
                P_per_axis{ax} = P_full;
            else
                P_per_axis{ax} = P_nb;
            end
        end

        % --- 0H. IIR states (prefill default) ---
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
        Fbar_c2 = zeros(3, 1);            FAbar_c2 = zeros(3, 1);   % C2 regressor EWMAs
        fdet_km1 = zeros(3, 1);           fdet_km2 = zeros(3, 1);   % C2 f_det recursion
        a_true_km1 = a_x_init;            a_true_km2 = a_x_init;    % oracle-y2 true-gain history

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
    a_hat = x_e_per_axis(4, :).';     % 3x1 [um/pN]  (slot 4 = a_x, x/y)
    if use_gamma_split
        % z: slot 4 = sigma, slot n_st = gamma, a_hat = exp(gamma + sigma).
        % x/y activate the same ln-split when gamma_shared (else slot 4 = a_x).
        a_hat(3) = local_exp_clamp(x_e_per_axis(4, 3) + x_e_per_axis(n_st, 3), a_nom_p);
        if gamma_shared
            a_hat(1) = local_exp_clamp(x_e_per_axis(4, 1) + x_e_per_axis(n_st, 1), a_nom_p);
            a_hat(2) = local_exp_clamp(x_e_per_axis(4, 2) + x_e_per_axis(n_st, 2), a_nom_p);
        end
    end

    if has_override
        a_ctrl = a_ctrl_override;        % true-gain (or externally supplied) gain
    else
        a_ctrl = a_hat;                  % normal mode: EKF posterior[k-1]
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
        h_bar_d = (dot(pd, w_hat_n) - pz_wall) / R_radius;   % desired height (exogenous, gamma height gate)
    else
        h_bar = Inf;
        h_bar_d = Inf;
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
        % Init-correct IIR hold (chat 2026-07-13): the first d samples of
        % delta_x_m are buffer-init artifacts, not data (pd_km_d and the
        % sensor delay buffer both return init values -> dx_r ~ 0 exactly).
        % Feeding them drags sigma2_dxr_hat down by ~d*a_cov (~10%) and the
        % ar1 level-learner reads the low a_xm as signal (the -10% a_hat
        % slug). Hold the IIR at its prefill for those d steps instead.
        % (Replaces the 0.2 s G1 blanket + F1 re-prefill of the first
        % attempt; white keeps the historical behavior, bit-identical.)
        dx_bar_m_new = dx_bar_m;
        dx_r = delta_x_m - dx_bar_m_new;
        sigma2_dxr_hat_new = sigma2_dxr_hat;
    else
        dx_bar_m_new = (1 - a_pd) * dx_bar_m + a_pd * delta_x_m;
        dx_r = delta_x_m - dx_bar_m_new;
        sigma2_dxr_hat_new = (1 - a_cov) * sigma2_dxr_hat + a_cov * dx_r.^2;
    end
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
        if use_gamma_split
            a_hat_post(3) = local_exp_clamp(x_e_per_axis(4, 3) + x_e_per_axis(n_st, 3), a_nom_p);
        end
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

    % ------------------------------------------------------------------
    % [3b'] Gamma-split z transport slope kappa (ln-space, level-free).
    %   known/ahat: kappa = -K_h(h_bar_d)/R (wall shape at the DESIRED height,
    %               contains NO a_hat -> the C3 self-anchor drift is structurally
    %               absent, gamma_split_draft.tex Sec.2).
    %   diff:       kappa = a'_hat / a_hat (a-space self-diff slope -> ln slope,
    %               model-free). Used for the z F_e coupling + sigma deviation
    %               correction + (ahat/diff) the deterministic FF.
    % ------------------------------------------------------------------
    kappa_tr = zeros(3, 1);
    if use_gamma_split
        switch aprime_source
            case {'known', 'ahat'}
                kappa_d_all = local_a_prime_known(pd, w_hat_n, pz_wall, R_radius, ...
                                                  enable_wall, ones(3, 1));   % -K_h/R (para,para,perp)
                kappa_tr = kappa_d_all;
            otherwise   % 'diff'
                kappa_tr = a_prime ./ max(a_hat, realmin);
        end
        if ~gamma_shared
            kappa_tr(1:2) = 0;   % z-only split: x/y are inert
        end
    end
    kappa_z_tr = kappa_tr(3);    % scalar alias used by the existing z-only code

    % ------------------------------------------------------------------
    % [3c] C2 regressor EWMAs (chat 2026-07-13): p_md ~ c3*a + b3 + v3 with
    %   c3 = -Fbar/(1-lc), b3 = +FAbar/(1-lc); Fbar/FAbar are the SAME a_pd
    %   EWMA applied to the DETERMINISTIC force F_dx_det and F_dx_det*a_ctrl
    %   (same-filter trick: p_md = EWMA(dx_m), dx_m_det ~ -F_dx_det*e_ax/(1-lc)).
    %   F_dx_det comes from a mirror of the control law with the noise-driven
    %   feedback removed (delta_x_m := 0) -- purely trajectory-driven, hence
    %   EXOGENOUS. Using the TOTAL F_dx here is an endogeneity bug: during
    %   hold f_d ~ (1-lc)/a*delta_x_m (pure noise feedback), so EWMA(F_dx)
    %   correlates with the p_md noise and the y_3 regression rectifies a_hat
    %   downward regardless of noise sign (-70%% crash, c2diag 2026-07-13).
    %   Known caveat: dx_m responds to F_dx_det[k-d]; using [k] misaligns by
    %   d = 2 steps inside a tau = 20-step EWMA (small lag attenuation).
    % ------------------------------------------------------------------
    if use_c2 || fe_eval_fdet
        % f_det: deterministic mirror of the control law with the noise-driven
        % feedback removed (delta_x_m := 0). Purely trajectory-driven (exogenous).
        % Runs for the C2 y_3 regressor AND the fe_eval_fdet PROBE (F_e-from-f_det);
        % the Fbar/FAbar EWMAs below stay C2-only (bit-identical use_c2 path).
        if d_delay == 2
            sum_af_det = a_ctrl_km1 .* fdet_km1 + a_ctrl_km2 .* fdet_km2;
        else
            sum_af_det = a_ctrl_km1 .* fdet_km1;
        end
        f_det = inv_a_ctrl .* (pd_kp1 - lambda_c * pd - one_minus_lc * pd_km_d ...
                               - one_minus_lc * sum_af_det);
        if use_c2
            if d_delay == 2
                F_dx_det = f_det + one_minus_lc * (fdet_km1 + fdet_km2);
            else
                F_dx_det = f_det + one_minus_lc * fdet_km1;
            end
            Fbar_c2  = (1 - a_pd) * Fbar_c2  + a_pd * F_dx_det;
            FAbar_c2 = (1 - a_pd) * FAbar_c2 + a_pd * (F_dx_det .* a_ctrl);
            c3_v = -Fbar_c2  / one_minus_lc;
            b3_v =  FAbar_c2 / one_minus_lc;
        else
            c3_v = zeros(3, 1);
            b3_v = zeros(3, 1);
        end
    else
        f_det = zeros(3, 1);
        c3_v = zeros(3, 1);
        b3_v = zeros(3, 1);
    end

    % ------------------------------------------------------------------
    % [4] Q (4x4 diagonal) and R (2x2) per axis
    %   Q33 = Var(epsilon); Q44 = var(delta_a_ram) (gain-level driving noise)
    % ------------------------------------------------------------------
    Q_per_axis = cell(3, 1);
    R_per_axis = cell(3, 1);
    gate_off   = false(3, 1);
    G_flags    = false(3, 3);
    R33_vec    = zeros(3, 1);   % C2 y_3 white-floor R (use_c2 only)
    s_y2_vec   = ones(3, 1);    % y2 mirror sensitivity H(2,4) (use_y2m only)
    var_da_ram = zeros(3, 1);   % Q44 driver (mode-dependent: RW / AR(1) / cap / taylor)
    var_da_inc = zeros(3, 1);   % true increment var Var(delta_a_ram), for Q33/R22
    Q33_vec    = zeros(3, 1);   % Var(epsilon) per axis (taylor Q needs Q33_z cross-axis)
    R2_int_vec = zeros(3, 1);   % booked R2_intrinsic per axis (oracle-y2 synthetic noise)
    t_now = (k_step - 1) * Ts;
    for ax = 1:3
        a_hat_i = a_hat(ax);
        if use_y2m
            s_y2_vec(ax) = y2m_sT + y2m_cN * sigma2_n_s(ax) / (4 * kBT * a_hat_i);
        end
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

        if use_colored_eps
            % Colored-eps: the beta^2 thermal-history lag now lives in the
            % m-states (Q66 injection + m-block coupling); Q33 keeps only the
            % fresh white kick (colored_eps_draft §3 reconciliation). randgain
            % and n_x terms are kept as-is (separate, minor colorings).
            Q33_thermal = 4 * kBT * a_hat_i;
            if d_delay == 2
                Q33_randgain = one_minus_lc^2 * ( 4 * f_d_km1(ax)^2 * var_da_inc_km1(ax) ...
                                                + 1 * f_d_km2(ax)^2 * var_da_inc_km2(ax) );
            else
                Q33_randgain = one_minus_lc^2 * (f_d_km1(ax)^2 * var_da_inc_km1(ax));
            end
        elseif d_delay == 2
            Q33_thermal  = 4 * kBT * (a_hat_i + one_minus_lc^2 * (a_hat_km1(ax) + a_hat_km2(ax)));
            Q33_randgain = one_minus_lc^2 * ( 4 * f_d_km1(ax)^2 * var_da_inc_km1(ax) ...
                                            + 1 * f_d_km2(ax)^2 * var_da_inc_km2(ax) );
        else
            Q33_thermal  = 4 * kBT * (a_hat_i + one_minus_lc^2 * a_hat_km1(ax));
            Q33_randgain = one_minus_lc^2 * (f_d_km1(ax)^2 * var_da_inc_km1(ax));
        end
        Q33_nx = one_minus_lc^2 * sigma2_n_s(ax);
        Q33_vec(ax) = Q33_thermal + Q33_randgain + Q33_nx;

        Q_i = zeros(n_st);
        Q_i(3, 3) = Q33_vec(ax);
        Q_i(4, 4) = var_da_ram(ax);                    % gain-level driving noise (slot 4)

        R11_i = sigma2_n_s(ax);
        if r22_adet
            a_r22_i = a_det_k(ax);      % PROBE: oracle eval point (breaks R22(a_hat) loop)
        else
            a_r22_i = a_hat_i;
        end
        IF_eff_i = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a_r22_i, sigma2_n_s(ax));
        R2_intrinsic_i = amlpf_var_factor * K_var * IF_eff_i * (a_r22_i + xi_per_axis(ax))^2;
        R2_int_vec(ax) = R2_intrinsic_i;       % booked intrinsic R (oracle-y2 noise scale)
        if d_delay == 2
            % Telescoped delay-sum: var(delta_a_ram[k-1]+delta_a_ram[k-2]) =
            % 2*V_a*(1-rho2), not the independent 2-term sum (audit 2026-07-10).
            R22_delay_i = r22_delay_factor * (var_da_inc_km1(ax) + var_da_inc_km2(ax));
        else
            R22_delay_i = var_da_inc_km1(ax);       % d=1 single term is exact
        end
        if y2_ar1
            % Colored y_2: the intrinsic EWMA noise lives in state v (slot 5,
            % stationary var tracked to (1-r2f)*R2_intrinsic[k]); R22 keeps the
            % F2 white floor (r2f*R2_int) + the short-memory delay remainder.
            % Total variance is exactly R2_intrinsic + delay (same as white).
            Q_i(5, 5) = (1 - phi_v^2) * (1 - r2f) * R2_intrinsic_i;
            R2_eff_i = r2f * R2_intrinsic_i + R22_delay_i;
        else
            R2_eff_i = R2_intrinsic_i + R22_delay_i;
        end
        if use_c2
            % C2: p_md noise scale tracks the stationary dx_m variance
            % (empirical kappa, D2); split (1-r3f) colored / r3f white.
            var_n3_i = c2_kappa * (C_dpmr * 4 * kBT * a_hat_i + C_n * sigma2_n_s(ax));
            Q_i(n_st, n_st) = (1 - c2_phi^2) * (1 - c2_r3f) * var_n3_i;
            R33_vec(ax) = c2_r3f * var_n3_i;
        end
        Q_per_axis{ax} = Q_i;

        G1 = (t_now < t_warmup_kf);
        G2 = ((sigma2_dxr_hat_new(ax) - C_n * sigma2_n_s(ax)) <= 0);
        G3 = (h_bar < h_bar_safe);
        G_flags(:, ax) = [G1; G2; G3];
        gate_off(ax) = G1 || G2 || G3;
        if shared_xy_y2_off && ax ~= 3
            gate_off(ax) = true;    % DIAGNOSTIC: drop x/y y_2 (z untouched)
        end

        if gate_off(ax)
            R22_i = R_OFF;
        else
            R22_i = R2_eff_i;
        end
        if use_c2
            R_ax = zeros(3);
            R_ax(1, 1) = R11_i;
            R_ax(2, 2) = R22_i;
            R_ax(3, 3) = R33_vec(ax);
        else
            R_ax = zeros(2);
            R_ax(1, 1) = R11_i;
            R_ax(2, 2) = R22_i;
        end
        R_per_axis{ax} = R_ax;
    end

    % --- Taylor-gain rank-1 Q fixup (4state_del_hd.tex taylor section): q3 = -eps and
    %     q4 = a'*eps share the same sample -> Q(3:4,3:4) = s2eps*[1 -a'; -a' a'^2]
    %     on the z axis; x/y have q4 = a'_i*eps_z (independent of own q3), so
    %     Q34 = 0 and Q44 = a'_i^2 * Q33_z. Overrides the RW Q44 set above. ---
    if use_taylor_gain && use_colored_eps
        % Colored-eps rank-1 thermal Q (colored_eps_draft §3): a single white
        % kick w[k] (variance sw = 4kBT*a) drives dx3(-1), the gain-row(+a' /
        % +kappa), and m1(+1). Q34/Q44 use sw (THERMAL only, no lag/randgain/nx);
        % the m-block Q36/Q46/Q66 carries the lag variance the filter can learn.
        Q33_z = Q33_vec(3);
        for ax = 1:3
            sw_i = 4 * kBT * a_hat(ax);
            if gamma_shared && ax ~= 3
                % x/y ln-split sigma is z-DRIVEN: Q_sigma = kappa_para^2*Q33_z
                % (the a-space a'^2*Q33_z divided by a_i^2; no dx3<->sigma cross).
                var_da_ram(ax) = kappa_tr(ax)^2 * Q33_z;
            else
                var_da_ram(ax) = a_prime(ax)^2 * Q33_z;   % x/y a-space (unshared); z overwritten
            end
            Q_per_axis{ax}(4, 4) = var_da_ram(ax);
            Q_per_axis{ax}(3, m1_slot) = -sw_i;           % dx3 <-> m1 (own-axis thermal)
            Q_per_axis{ax}(m1_slot, 3) = -sw_i;
            Q_per_axis{ax}(m1_slot, m1_slot) = sw_i;      % fresh kick into m1
        end
        sw_z = 4 * kBT * a_hat(3);
        if use_gamma_split
            kap = -K_h_axis(3) / R_radius;                % kappa_m (measured; ln shape)
            var_da_ram(3) = kap^2 * sw_z;
            Q_per_axis{3}(4, 4) = var_da_ram(3);
            Q_per_axis{3}(3, 4) = -kap * sw_z;  Q_per_axis{3}(4, 3) = -kap * sw_z;
            Q_per_axis{3}(4, m1_slot) = kap * sw_z;  Q_per_axis{3}(m1_slot, 4) = kap * sw_z;
        else
            ap = a_prime(3);
            var_da_ram(3) = ap^2 * sw_z;
            Q_per_axis{3}(4, 4) = var_da_ram(3);
            Q_per_axis{3}(3, 4) = -ap * sw_z;  Q_per_axis{3}(4, 3) = -ap * sw_z;
            Q_per_axis{3}(4, m1_slot) = ap * sw_z;  Q_per_axis{3}(m1_slot, 4) = ap * sw_z;
        end
    elseif use_taylor_gain
        Q33_z = Q33_vec(3);
        for ax = 1:3
            if gamma_shared && ax ~= 3
                var_da_ram(ax) = kappa_tr(ax)^2 * Q33_z;   % x/y ln sigma (z-driven)
            else
                var_da_ram(ax) = a_prime(ax)^2 * Q33_z;    % x/y a-space; z overwritten below
            end
            Q_per_axis{ax}(4, 4) = var_da_ram(ax);
        end
        if use_gamma_split
            % z ln-space rank-1 (q = eps*[0 0 -1 kappa_m 0], gamma_split_draft.tex Sec.4):
            % Q_sigma,sigma = kappa_m^2*Q33_z, Q3,sigma = -kappa_m*Q33_z; gamma-row 0.
            % kappa_m at the MEASURED h_bar (noise scale; avoids the bias loop).
            kappa_m_z = -K_h_axis(3) / R_radius;
            var_da_ram(3) = kappa_m_z^2 * Q33_z;
            Q_per_axis{3}(4, 4) = var_da_ram(3);
            Q_per_axis{3}(3, 4) = -kappa_m_z * Q33_z;
            Q_per_axis{3}(4, 3) = -kappa_m_z * Q33_z;
        else
            Q_per_axis{3}(3, 4) = -a_prime(3) * Q33_z;
            Q_per_axis{3}(4, 3) = -a_prime(3) * Q33_z;
        end
    end

    % ------------------------------------------------------------------
    % [4b] Oracle y_2 measurement (PROBE knife-2, chat 2026-07-14)
    %   Replace the real IIR a_xm content with the TRUE delayed gain
    %   a_true[k-d] plus synthetic white noise of the booked variance
    %   R2_intrinsic[k]. Drawn from the ISOLATED oracle_stream (global RNG
    %   untouched). The existing d-step drift correction (+ sum_da_ff) still
    %   maps a_meas from [k-d] to [k] in the EKF loop below, unchanged.
    % ------------------------------------------------------------------
    if y2_oracle
        assert(~isempty(a_true_k), ...
               'motion_control_law_eq17_4state:oracleNeedsTruth', ...
               'y2_oracle_meas=true requires the 7th arg a_true_k (3x1).');
        if d_delay == 2
            a_true_kmd = a_true_km2;      % gain d = 2 steps ago
        else
            a_true_kmd = a_true_km1;      % d = 1
        end
        a_meas = a_true_kmd(:) + sqrt(R2_int_vec) .* randn(oracle_stream, 3, 1);
    end

    % ------------------------------------------------------------------
    % [5] EKF predict + update per axis
    % ------------------------------------------------------------------
    if use_c2 && y2_ar1
        H_full = [1 0 0 0 0 0; 0 0 0 1 1 0];   % y_3 row appended per axis (time-varying c3)
    elseif use_c2
        H_full = [1 0 0 0 0; 0 0 0 1 0];   % white y_2 control arm: no v2 column
    elseif y2_ar1 && use_gamma_split
        % [dx1 dx2 dx3 sigma v2 gamma]; base = x/y (a-space slot4, inert gamma);
        % z overrides H(2,4)=H(2,gamma)=a_hat_pred in the loop (EKF exp Jacobian).
        H_full = [1 0 0 0 0 0; 0 0 0 1 1 0];
    elseif use_gamma_split
        % [dx1 dx2 dx3 sigma gamma]; z overrides H(2,4)=H(2,gamma)=a in the loop.
        H_full = [1 0 0 0 0; 0 0 0 1 0];
    elseif y2_ar1
        H_full = [1 0 0 0 0; 0 0 0 1 1];   % y_2 = a_x + v (v = EWMA estimator noise)
    else
        H_full = [1 0 0 0; 0 0 0 1];
    end
    if use_colored_eps
        % Scatter the core H into the full n_st, leaving the m-columns zero
        % (m-states are unmeasured; colored_eps_draft §2 H). gamma (if any) stays
        % at n_st, m at m1_slot/m2_slot just before it.
        core_cols = [1:(m1_slot - 1), (m2_slot + 1):n_st];
        H_scat = zeros(size(H_full, 1), n_st);
        H_scat(:, core_cols) = H_full;
        H_full = H_scat;
    end
    H_y1 = H_full(1, :);
    if use_c2
        H_y3_base = zeros(1, n_st);            % row(4) = c3 set per axis; v3 = last slot
        H_y3_base(n_st) = 1;
    end

    K_a_y2_v  = zeros(3, 1);
    K_dx_y1_v = zeros(3, 1);
    innov_y2_v = zeros(3, 1);
    K_a_y1_v  = zeros(3, 1);        % L(4,1): y1 -> gain state (v-budget diag)
    K_dx_y2_v = zeros(3, 1);        % L(3,2): y2 -> dx3 state  (v-budget diag)
    innov_y1_v = zeros(3, 1);
    gamma_frozen = false(3, 1);     % per-axis gamma-gate state (shared-gamma fusion)

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
        % PROBE fe_eval_fdet (chat 2026-07-13): substitute the deterministic
        % mirror force for the actual force in the F_e CONSTRUCTION ONLY (breaks
        % the f_d<->measurement-noise correlation in the covariance geometry;
        % Q33, R22, predict, and the measurement update keep the actual f_d).
        % Default off -> f_fe_i/F1_fe_i are textually the actual-force values.
        if fe_eval_fdet
            f_fe_i = f_det(ax);
            if d_delay == 2
                F1_fe_i = fdet_km1(ax) + fdet_km2(ax);
            else
                F1_fe_i = fdet_km1(ax);
            end
        else
            f_fe_i  = f_d(ax);
            F1_fe_i = F_1_i;
        end
        % 4-state F_e has no delta_a_x column, so dF_dx = (1-lc)*F_2 does not
        % appear; only F_1 (-> F_dx) is needed.
        if use_gamma_split && (ax == 3 || gamma_shared)
            % Ln-space F_e: z (self-driven sigma) always; x/y (z-driven sigma
            % integrator) when gamma_shared. gamma_split_draft §3/§7.
            F_dx_i = f_fe_i + one_minus_lc * F1_fe_i;
            F_e = build_F_e_gsplit(n_st, lambda_c, F_dx_i, kappa_tr(ax), a_hat(ax), ...
                                   y2_ar1, phi_v, ax == 3);
        else
            if use_taylor_gain && ax == 3
                % Taylor-gain z row 4 = [0 0 (1-lc)a' 1+a'F_dx] (4state_del_hd.tex taylor section)
                F_dx_i   = f_fe_i + one_minus_lc * F1_fe_i;
                fe43_i   = one_minus_lc * a_prime(3);
                a_pole_i = 1 + a_prime(3) * F_dx_i;
                F_e = build_F_e_4state(lambda_c, f_fe_i, F1_fe_i, a_pole_i, fe43_i);
            elseif use_q44_ar1
                if use_exact_fe44
                    % exact pole lc + a'*F_dx, a' = -a_hat*K_h/R (known wall),
                    % F_dx = f_d[k] + (1-lc)*sum f_d[k-i]
                    a_prime_i = -a_hat(ax) * K_h_axis(ax) / R_radius;
                    a_pole_i = lambda_c + a_prime_i * (f_fe_i + (1 - lambda_c) * F1_fe_i);
                else
                    a_pole_i = lambda_c;                                  % simplified (drops a'*F_dx)
                end
                F_e = build_F_e_4state(lambda_c, f_fe_i, F1_fe_i, a_pole_i);   % AR(1)
            else
                F_e = build_F_e_4state(lambda_c, f_fe_i, F1_fe_i);
            end
            if y2_ar1
                F_e = blkdiag(F_e, phi_v);       % v decoupled: v[k+1] = phi*v[k] + w_v
            end
            if use_c2
                F_e = blkdiag(F_e, c2_phi);      % v3 decoupled: v3[k+1] = phi3*v3[k] + w3
            end
            if use_colored_eps
                F_e = blkdiag(F_e, zeros(2));    % m1,m2 slots (couplings added below)
            end
            if use_gamma_split
                % x/y inert gamma (ax ~= 3 here; z handled above): F_e(g,g)=1,
                % zero coupling -> the level slot is decoupled and frozen.
                F_e = blkdiag(F_e, 1);
            end
        end
        % --- Colored-eps m-block couplings (all axes, colored_eps_draft §2/§2b).
        %     dx3 row: -beta on m1,m2 (the lagged thermal kicks); a/sigma-row (z
        %     only): +a'*beta / +kappa*beta; m2 = shift of m1; m1 row noise-only.
        if use_colored_eps
            F_e(3, m1_slot) = -one_minus_lc;
            F_e(3, m2_slot) = -one_minus_lc;
            F_e(m2_slot, m1_slot) = 1;                % m2[k+1] = m1[k] (shift)
            if ax == 3
                if use_gamma_split
                    arow_coup = kappa_z_tr * one_minus_lc;   % sigma-row (ln shape)
                else
                    arow_coup = a_prime(3) * one_minus_lc;   % a-row
                end
                F_e(4, m1_slot) = arow_coup;
                F_e(4, m2_slot) = arow_coup;
            end
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
            x4_pred = x_curr(4) + da_ff_i ...
                      + one_minus_lc * a_prime(ax) * dxhat3_z_prev;
        elseif use_q44_ar1
            % AR(1) reverting gain: a_x reverts to a_det with pole lc.
            x4_pred = lambda_c * x_curr(4) + (1 - lambda_c) * a_det_k(ax) + da_x_pred(ax);
        else
            x4_pred = x_curr(4) + da_x_pred(ax) + da_ram_pred_i;
        end
        if use_colored_eps
            % Index-based predict (colored-eps; composes with ar1 + gsplit; c2
            % excluded). m1[k+1] = w[k] (mean 0); m2[k+1] = m1[k] (shift).
            % sigma predict UNIFIES across z (self) and x/y (z-driven): both use
            % the axis' own kappa_tr + a_det ratio and z's dxhat3 (§7 known input).
            if use_gamma_split && (ax == 3 || gamma_shared)
                switch aprime_source
                    case 'known'
                        dsig_det = log(a_det_k(ax) / a_det_km1(ax));
                    otherwise
                        dsig_det = kappa_tr(ax) * dh_d_step;
                end
                slot4 = x_curr(4) + dsig_det + one_minus_lc * kappa_tr(ax) * dxhat3_z_prev;
            else
                slot4 = x4_pred;
            end
            x_pred = zeros(n_st, 1);
            x_pred(1) = x_curr(2);
            x_pred(2) = x_curr(3);
            x_pred(3) = lambda_c * x_curr(3);
            x_pred(4) = slot4;
            if y2_ar1; x_pred(5) = phi_v * x_curr(5); end
            if use_gamma_split; x_pred(n_st) = x_curr(n_st); end   % gamma identity
            x_pred(m1_slot) = 0;                  % m1[k+1] = w[k], mean 0
            x_pred(m2_slot) = x_curr(m1_slot);    % m2[k+1] = m1[k] shift
        elseif use_gamma_split
            % Slot 4: z (always) + x/y (when shared) -> sigma predict; else x/y
            % a-space x4_pred. gamma slot (last): identity (noiseless integrator).
            if ax == 3 || gamma_shared
                switch aprime_source
                    case 'known'     % exact ln ratio of the KNOWN levels
                        dsig_det = log(a_det_k(ax) / a_det_km1(ax));
                    otherwise        % ahat / diff: first-order ln transport
                        dsig_det = kappa_tr(ax) * dh_d_step;
                end
                s4_pred = x_curr(4) + dsig_det ...
                          + one_minus_lc * kappa_tr(ax) * dxhat3_z_prev;   % sigma predict
            else
                s4_pred = x4_pred;                                       % a-space (x/y unshared)
            end
            if y2_ar1
                x_pred = [x_curr(2); ...
                          x_curr(3); ...
                          lambda_c * x_curr(3); ...
                          s4_pred; ...
                          phi_v * x_curr(5); ...
                          x_curr(n_st)];       % gamma: identity
            else
                x_pred = [x_curr(2); ...
                          x_curr(3); ...
                          lambda_c * x_curr(3); ...
                          s4_pred; ...
                          x_curr(n_st)];        % gamma: identity
            end
        elseif use_c2 && y2_ar1
            x_pred = [x_curr(2); ...
                      x_curr(3); ...
                      lambda_c * x_curr(3); ...
                      x4_pred; ...
                      phi_v * x_curr(5); ...
                      c2_phi * x_curr(6)];
        elseif use_c2
            x_pred = [x_curr(2); ...
                      x_curr(3); ...
                      lambda_c * x_curr(3); ...
                      x4_pred; ...
                      c2_phi * x_curr(5)];
        elseif y2_ar1
            x_pred = [x_curr(2); ...
                      x_curr(3); ...
                      lambda_c * x_curr(3); ...
                      x4_pred; ...
                      phi_v * x_curr(5)];
        else
            x_pred = [x_curr(2); ...
                      x_curr(3); ...
                      lambda_c * x_curr(3); ...
                      x4_pred];
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
        elseif use_c2
            % y_3 = p_md - b3 = c3*a + v3 + white (known offset b3 subtracted)
            H_y3 = H_y3_base;
            H_y3(4) = c3_v(ax);
            H_use = [H_full; H_y3];
            y_use = [delta_x_m(ax); a_meas_corr; dx_bar_m_new(ax) - b3_v(ax)];
            R_use = R_per_axis{ax};
        else
            H_use = H_full;
            y_use = [delta_x_m(ax); a_meas_corr];
            R_use = R_per_axis{ax};
        end

        y_pred = H_use * x_pred;          % EKF h(x_pred): at the g=1 linearization
                                          % point E[y2|x_pred] = x4_pred + v_pred
        if use_gamma_split && (ax == 3 || gamma_shared) && ~gate_off(ax)
            % EKF nonlinearity (z always; x/y when shared): y_2 = a + v2 with
            % a = exp(sigma+gamma). Prediction is exp() (NOT H*x_pred); Jacobian
            % H(2,sigma)=H(2,gamma)=a_hat_pred (gamma_split_draft.tex Sec.3/§7).
            a_hat_pred_i = local_exp_clamp(x_pred(4) + x_pred(n_st), a_nom_p);
            if y2_ar1
                y_pred(2) = a_hat_pred_i + x_pred(5);   % + v2 colored-noise state
            else
                y_pred(2) = a_hat_pred_i;
            end
            H_use(2, 4)    = a_hat_pred_i;               % da/dsigma
            H_use(2, n_st) = a_hat_pred_i;               % da/dgamma
        elseif use_y2m && ~gate_off(ax)
            % Mirror correction enters the JACOBIAN only (gain/covariance
            % bookkeeping); y_pred stays h(x_pred) above.
            H_use(2, 4) = s_y2_vec(ax);
        end
        innov  = y_use - y_pred;
        S_inn  = H_use * P_pred * H_use' + R_use;
        S_inn  = 0.5 * (S_inn + S_inn');
        K_kf   = (P_pred * H_use') / S_inn;

        % Warmup gate: freeze gain state during G1 (slot 4)
        if G_flags(1, ax)
            K_kf(4, :) = 0;
            if use_gamma_split && (ax == 3 || gamma_shared)
                K_kf(n_st, :) = 0;      % also freeze the gamma (level) slot
            end
        end
        % Gamma freeze (option ii, gain-projection): freeze the LEVEL either
        % while t is inside the descent window [gamma_freeze_t0, gamma_freeze_t1)
        % OR while the desired height h_bar_d < gamma_hbar_min (near-wall gate),
        % so the curvature-corrupted descent AND the near-wall a_xm content
        % burst stay out of gamma (sigma/dx still update). Zeroing the gamma-row
        % of K leaves the Joseph P update a valid covariance (holds for any K).
        % Applies per axis (z always; x/y when shared -- respects the same window).
        if use_gamma_split && (ax == 3 || gamma_shared) && ...
                ((t_now >= gamma_freeze_t0 && t_now < gamma_freeze_t1) || (h_bar_d < gamma_hbar_min))
            K_kf(n_st, :) = 0;
            gamma_frozen(ax) = true;
        end
        if G_flags(1, ax) && use_gamma_split && (ax == 3 || gamma_shared)
            gamma_frozen(ax) = true;    % G1 also froze gamma above
        end

        x_post = x_pred + K_kf * innov;
        ImKH   = eye(n_st) - K_kf * H_use;
        P_post = ImKH * P_pred * ImKH' + K_kf * R_use * K_kf';   % Joseph form
        P_post = 0.5 * (P_post + P_post');

        % Diagnostics
        K_dx_y1_v(ax) = K_kf(3, 1);
        K_a_y1_v(ax)  = K_kf(4, 1);          % post-freeze (G1 zeroes row 4)
        innov_y1_v(ax) = innov(1);
        if gate_off(ax)
            K_a_y2_v(ax)  = 0;
            K_dx_y2_v(ax) = 0;
            innov_y2_v(ax) = 0;
        else
            K_a_y2_v(ax)  = K_kf(4, 2);
            K_dx_y2_v(ax) = K_kf(3, 2);
            innov_y2_v(ax) = innov(2);
        end

        x_e_per_axis(:, ax) = x_post;
        P_per_axis{ax} = P_post;
    end

    % ------------------------------------------------------------------
    % [5b] Carlson federated gamma fusion (gamma_split_draft §7; chat 2026-07-14)
    %   Stokes isotropy: one bulk gamma_true for all axes. Fuse the three
    %   per-axis gammas by information weight, reset each axis to the fused value.
    %   gamma_f = P_f * sum_i(gamma_i / P_gg_i),  1/P_f = sum_i(1/P_gg_i).
    %   Carlson information-sharing beta_i = 1/N = 1/3: reset P_gg_i = N*P_f = 3*P_f
    %   (guarantees federated consistency -- no double-counting -- zero tuning).
    %   Frozen axes are READ-ONLY consumers: they contribute their prior info and
    %   receive gamma_f, but keep their own P_gg (no reset). Gamma cross-covariance
    %   rows are ZEROED at reset -- a conservative choice: dropping the (sigma/dx,
    %   gamma) cross correlation can only make the axis filter treat gamma as more
    %   independent of its local states than it is, i.e. over-conservative; it
    %   cannot make P indefinite (the diagonal reset keeps each block PSD).
    % ------------------------------------------------------------------
    if use_gamma_split && gamma_shared
        g_slot = n_st;
        Pgg = [P_per_axis{1}(g_slot, g_slot); ...
               P_per_axis{2}(g_slot, g_slot); ...
               P_per_axis{3}(g_slot, g_slot)];
        gh  = x_e_per_axis(g_slot, :).';
        P_f = 1 / sum(1 ./ Pgg);
        gamma_f = P_f * sum(gh ./ Pgg);
        for ax = 1:3
            x_e_per_axis(g_slot, ax) = gamma_f;
            if ~gamma_frozen(ax)
                P_per_axis{ax}(g_slot, g_slot) = 3 * P_f;   % Carlson reset (active axes)
            end
            P_per_axis{ax}(g_slot, 1:g_slot-1) = 0;         % zero gamma cross (conservative)
            P_per_axis{ax}(1:g_slot-1, g_slot) = 0;
        end
    end

    % ------------------------------------------------------------------
    % [6] Bookkeeping: shift delay buffers, IIR states, step counter
    % ------------------------------------------------------------------
    pd_km2 = pd_km1; pd_km1 = pd;
    f_d_km2 = f_d_km1; f_d_km1 = f_d;
    fdet_km2 = fdet_km1; fdet_km1 = f_det;      % C2 deterministic-force mirror
    if y2_oracle
        a_true_km2 = a_true_km1; a_true_km1 = a_true_k(:);   % oracle-y2 true-gain history
    end
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
    if use_gamma_split
        a_hat_post(3) = local_exp_clamp(x_e_per_axis(4, 3) + x_e_per_axis(n_st, 3), a_nom_p);
        if gamma_shared
            a_hat_post(1) = local_exp_clamp(x_e_per_axis(4, 1) + x_e_per_axis(n_st, 1), a_nom_p);
            a_hat_post(2) = local_exp_clamp(x_e_per_axis(4, 2) + x_e_per_axis(n_st, 2), a_nom_p);
        end
    end
    h_bar_now = local_h_bar_out(enable_wall, h_bar);
    ekf_out = [a_hat_post(1); a_hat_post(3); a_hat_post(2); h_bar_now];

    if nargout >= 3
        P_a_v = zeros(3, 1); P_dx_v = zeros(3, 1); P_dx1_v = zeros(3, 1);
        for ax = 1:3
            P_a_v(ax)   = P_per_axis{ax}(4, 4);
            P_dx_v(ax)  = P_per_axis{ax}(3, 3);
            P_dx1_v(ax) = P_per_axis{ax}(1, 1);
        end
        if use_gamma_split
            % Report Var(a_hat) via the delta method so the P_a/Var(e_ax) honesty
            % metric stays comparable. P_a = a^2*(P_ss+P_gg+2 P_sg). z always;
            % x/y when shared (cross P_sg zeroed by fusion, so P_a = a^2(P_ss+P_gg)).
            split_axes = 3;
            if gamma_shared; split_axes = 1:3; end
            for ax = split_axes
                Psg = P_per_axis{ax}(4, n_st);
                P_a_v(ax) = a_hat_post(ax)^2 * (P_per_axis{ax}(4, 4) ...
                            + P_per_axis{ax}(n_st, n_st) + 2 * Psg);
            end
        end
        diag = empty_diag_4state();
        diag.sigma2_dxr_hat = sigma2_dxr_hat_new;
        diag.a_xm           = a_xm;
        diag.a_xm_fed       = a_meas;       % value actually fed to EKF y_2 (= a_xm when oracle off)
        diag.a_m_det        = a_m_det_new;
        diag.delta_x_m      = delta_x_m;
        diag.innovation_y2  = innov_y2_v;
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
        diag.a_hat                = a_hat_post;
        diag.a_ctrl_used          = a_ctrl;
        diag.P77                  = zeros(3, 1);            % driver compat
        diag.Q77                  = zeros(3, 1);
        diag.var_da_ram           = var_da_ram;
        diag.delta_x_hat_1        = x_e_per_axis(1, :).';
        diag.delta_x_hat_3        = x_e_per_axis(3, :).';   % current tracking-error estimate
        diag.P_dx1                = P_dx1_v;
        diag.a_prime_used         = a_prime;                % taylor-gain slope (zeros if off)
        if y2_ar1
            P_v_v = zeros(3, 1);
            for ax = 1:3
                P_v_v(ax) = P_per_axis{ax}(5, 5);
            end
            diag.v_hat = x_e_per_axis(5, :).';              % estimated y_2 colored noise
            diag.P_v   = P_v_v;
        else
            diag.v_hat = zeros(3, 1);
            diag.P_v   = zeros(3, 1);
        end
        if use_c2
            P_v3_v = zeros(3, 1);
            for ax = 1:3
                P_v3_v(ax) = P_per_axis{ax}(n_st, n_st);
            end
            diag.v3_hat  = x_e_per_axis(n_st, :).';         % estimated p_md colored noise (last slot)
            diag.P_v3    = P_v3_v;
            diag.c3_used = c3_v;                            % y_3 gain coefficient
        else
            diag.v3_hat  = zeros(3, 1);
            diag.P_v3    = zeros(3, 1);
            diag.c3_used = zeros(3, 1);
        end
        if use_gamma_split
            % z real; x/y report 0 (inert dummy gamma). P_gamma/P_sigma are the
            % raw covariance diagonals (z meaningful; x/y P_sigma = a-space P44).
            P_g_v   = zeros(3, 1);
            P_sig_v = zeros(3, 1);
            for ax = 1:3
                P_g_v(ax)   = P_per_axis{ax}(n_st, n_st);
                P_sig_v(ax) = P_per_axis{ax}(4, 4);
            end
            if gamma_shared
                diag.gamma_hat = x_e_per_axis(n_st, :).';   % all axes active + fused
                diag.sigma_hat = x_e_per_axis(4, :).';
            else
                diag.gamma_hat = [0; 0; x_e_per_axis(n_st, 3)];   % z real, x/y inert
                diag.sigma_hat = [0; 0; x_e_per_axis(4, 3)];
            end
            diag.P_gamma   = P_g_v;
            diag.P_sigma   = P_sig_v;
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


function F_e = build_F_e_gsplit(n_st, lambda_c, F_dx, kappa, a_hat_z, y2_ar1, phi_v, sigma_selfdriven)
%BUILD_F_E_GSPLIT  Full n_st x n_st ln-space error-dynamics matrix.
%   States [dx1, dx2, dx3, sigma, (v2), gamma] with a = exp(gamma+sigma).
%   Rows (gamma_split_draft.tex Sec.3/§7), gamma at slot n_st, v2 (if any) at 5:
%     dx3   : [0 0 lc            -F_dx*a           ... -F_dx*a  ]  (cols 4 & gamma)
%     sigma : self-driven (z): [0 0 (1-lc)*kappa 1+kappa*F_dx*a ... kappa*F_dx*a]
%             z-driven  (x/y): [0 0 0            1              ... 0          ]
%     gamma : [0 ...                                           1  ]  (noiseless)
%   sigma_selfdriven=true (z, default): the sigma row tracks the axis' own dx3.
%   =false (x/y): sigma is an integrator; its transport is a KNOWN predict input
%   driven by the wall-normal (z) motion (gamma_split_draft §7). kappa =
%   d(sigma)/dh (level-free); a_hat_z the posterior[k-1] gain level.
    if nargin < 8 || isempty(sigma_selfdriven); sigma_selfdriven = true; end
    one_minus_lc = 1 - lambda_c;
    g = n_st;                       % gamma slot (last)
    F_e = zeros(n_st);
    F_e(1, 2) = 1;
    F_e(2, 3) = 1;
    F_e(3, 3) = lambda_c;
    F_e(3, 4) = -F_dx * a_hat_z;
    F_e(3, g) = -F_dx * a_hat_z;
    if sigma_selfdriven
        F_e(4, 3) = one_minus_lc * kappa;
        F_e(4, 4) = 1 + kappa * F_dx * a_hat_z;
        F_e(4, g) = kappa * F_dx * a_hat_z;
    else
        F_e(4, 4) = 1;              % x/y: sigma integrator (transport = predict input)
    end
    F_e(g, g) = 1;                  % gamma integrator (noiseless)
    if y2_ar1
        F_e(5, 5) = phi_v;          % v2 colored-noise state (decoupled)
    end
end


function a = local_exp_clamp(log_a, a_nom)
%LOCAL_EXP_CLAMP  exp() of a ln-gain with the exponent clamped to a sane band
%   around ln(a_nom) (guards against Inf on a divergent seed; the physical
%   ln a spans only ~[ln(a_nom/10.5), ln(a_nom)] so the clamp never binds).
    L = 10;                          % factor-e^10 band each side of a_nom
    lo = log(a_nom) - L;
    hi = log(a_nom) + L;
    a = exp(min(max(log_a, lo), hi));
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
    d.a_xm_fed          = zeros(3, 1);   % value fed to EKF y_2 (oracle-y2 probe)
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
    d.v_hat                = zeros(3, 1);   % colored-y2 noise estimate (ar1 mode)
    d.P_v                  = zeros(3, 1);
    d.v3_hat               = zeros(3, 1);   % C2 p_md noise estimate (use_c2 mode)
    d.P_v3                 = zeros(3, 1);
    d.c3_used              = zeros(3, 1);   % C2 y_3 gain coefficient
    d.gamma_hat            = zeros(3, 1);   % gamma-split level estimate (z; ln units)
    d.sigma_hat            = zeros(3, 1);   % gamma-split shape estimate (z; ln units)
    d.P_gamma              = zeros(3, 1);   % filter var of gamma
    d.P_sigma              = zeros(3, 1);   % filter var of sigma
end
