function simOut = temp_run_pure_sim_centered(config, opts)
%TEMP_RUN_PURE_SIM_CENTERED  Copy of run_pure_simulation with the is_4state
%   branch dispatched to temp_motion_control_law_eq17_4state_centered (adds the
%   aprime_source='varchan_centered' closed-loop mode + aprime_scale passthrough).
%   TEMP diagnostic (chat 2026-07-16); delete after the var-centered verification.
%
%RUN_PURE_SIMULATION Pure-MATLAB time-stepping driver for the eq17_7state path.
%
%   simOut = run_pure_simulation(config)
%   simOut = run_pure_simulation(config, opts)
%
%   Replaces sim('system_model') for the eq17_7state controller. Mirrors
%   the Simulink ToWorkspace schema so downstream analysis scripts are
%   drop-in compatible (see agent_docs/dual-track-simulation-design.md
%   decision 5).
%
%   This driver supports ONLY controller_type=='eq17_7state'. type=7 /
%   type=23 stay on the Simulink path.
%
%   Inputs:
%       config - Same struct as run_simulation.m's config (after
%                user_config() + overrides). Must contain at minimum:
%                  config.T_sim
%                  config.lambda_c
%                  config.meas_noise_enable
%                  config.meas_noise_std (3x1 [um])
%                  config.a_cov
%                Plus all wall/traj/ctrl fields user_config() defines.
%       opts   - (optional) struct with fields (all defaulted):
%                  opts.seed         - RNG seed                  (default 42)
%                  opts.verbose      - print progress every 10%  (default false)
%                  opts.scheme       - integration scheme tag    (default 'ode4_step10us')
%                  opts.collect_diag - accumulate per-step diag
%                                      time series into simOut.diag (default false)
%                  opts.a_hat_freeze - 3x1 [um/pN] to lock the EKF
%                                      gain state per axis        (default [])
%                  opts.use_true_gain - feed the true time-varying gain to
%                                      the 6-state control law; requires
%                                      config.eq17_variant='6state' (default false)
%
%   Output simOut struct (mirrors Simulink ToWorkspace, all [Nx3] except
%   tout / ekf_out / meta):
%       simOut.p_d_out    - Desired position pd[k]      [N x 3, um]
%       simOut.f_d_out    - Control force f_d[k]        [N x 3, pN]
%       simOut.F_th_out   - Thermal force F_th[k]       [N x 3, pN]
%       simOut.p_m_out    - Measured position           [N x 3, um]
%                              (after measurement noise injection,
%                               before sensor delay — matches Simulink
%                               p_m_out tap point)
%       simOut.p_true_out - Noise-free true position    [N x 3, um]
%                              (ground-truth probe, logged AFTER the
%                               step's continuous integration)
%       simOut.a_true_out - True motion gain            [N x 3, um/pN]
%                              (at the PRE-integration p_curr, i.e. the
%                               position the controller acts on; always
%                               populated, one sample ahead of
%                               p_true_out's tap point)
%       simOut.tout       - Time vector                 [N x 1, sec]
%       simOut.ekf_out    - EKF diagnostic              [N x 4]
%                              [a_hat_x; a_hat_z; a_hat_y; h_bar]
%       simOut.meta       - struct(config, params_value, scheme, seed,
%                                  driver, driver_version)
%
%   See also: step_dynamics, motion_control_law_eq17_core,
%             trajectory_generator, calc_thermal_force, calc_gamma_inv

    % ------------------------------------------------------------------
    % 0. Defaults for opts
    % ------------------------------------------------------------------
    if nargin < 2 || isempty(opts); opts = struct(); end
    if ~isfield(opts, 'seed');         opts.seed         = 42;                 end
    if ~isfield(opts, 'verbose');      opts.verbose      = false;              end
    if ~isfield(opts, 'scheme');       opts.scheme       = 'ode4_step10us';    end
    % Phase 9 R(2,2) validation extensions:
    %   opts.a_hat_freeze  - 3x1 [um/pN] to lock state(6) per axis (default empty)
    %   opts.collect_diag  - bool, accumulate per-step diag time series
    if ~isfield(opts, 'a_hat_freeze'); opts.a_hat_freeze = [];                 end
    if ~isfield(opts, 'collect_diag'); opts.collect_diag = false;              end
    % True-gain A/B: feed true time-varying gain to the 6-state control law
    if ~isfield(opts, 'use_true_gain'); opts.use_true_gain = false;          end
    % TEMP (chat 2026-07-06, fixed-g probe; remove after use): scale factor c on
    % the true-gain override, a_ctrl = c*a_true => mismatch g := a_true/a_ctrl = 1/c.
    if ~isfield(opts, 'true_gain_scale'); opts.true_gain_scale = 1;          end

    % ------------------------------------------------------------------
    % 1. RNG seeding — MUST precede calc_simulation_params.
    %    calc_thermal_params:19 and calc_ctrl_params:31 each consume one
    %    randi(2^31-1) from the global rng to populate thermal.seed /
    %    meas_noise_seed. Seeding here makes those values a deterministic
    %    function of opts.seed, restoring full reproducibility of both the
    %    thermal-force stream (via calc_thermal_force's rng(thermal.seed))
    %    and the measurement-noise stream (which shares the global rng
    %    after that point).
    % ------------------------------------------------------------------
    rng(opts.seed);

    % ------------------------------------------------------------------
    % 2. Resolve params via existing builder (Simulink.Parameter wrapper)
    % ------------------------------------------------------------------
    params = calc_simulation_params(config);
    P = params.Value;   % unwrap to plain struct (used by all .m functions)

    % ------------------------------------------------------------------
    % 3. Clear controller / trajectory persistent state
    %    (mirrors run_simulation.m line 6 pattern)
    % ------------------------------------------------------------------
    clear motion_control_law motion_control_law_23state ...
          motion_control_law_eq6 motion_control_law_eq17 motion_control_law_eq17_core ...
          motion_control_law_eq17_6state motion_control_law_eq17_5state ...
          motion_control_law_eq17_5state_aprime ...
          motion_control_law_eq17_4state motion_control_law_eq17_gscalar ...
          temp_motion_control_law_eq17_4state_centered ...
          trajectory_generator calc_thermal_force;

    % ------------------------------------------------------------------
    % 4. Build offline constants for eq17_7state controller
    % ------------------------------------------------------------------
    eq17_opts.lambda_c   = config.lambda_c;
    eq17_opts.option     = 'A_MA2_full';
    eq17_opts.sigma2_n_s = (config.meas_noise_std(:)).^2;     % 3x1 [um^2]
    eq17_opts.kBT        = P.ctrl.k_B * P.ctrl.T;              % [pN*um]
    eq17_opts.t_warmup_kf = 0.2;
    if isfield(config, 't_warmup_kf') && ~isempty(config.t_warmup_kf)
        eq17_opts.t_warmup_kf = config.t_warmup_kf;   % Phase 3 sweep override
    end
    eq17_opts.h_bar_safe  = 1.5;
    if isfield(config, 'h_bar_safe') && ~isempty(config.h_bar_safe)
        eq17_opts.h_bar_safe = config.h_bar_safe;   % Round-2 gate-free override (design §12.1)
    end
    eq17_opts.d           = 2;
    eq17_opts.a_cov       = config.a_cov;
    % Wave 2D: separate a_pd (LP for δp_md mean) from a_cov (EWMA for σ²_δxr).
    eq17_opts.a_pd        = config.a_pd;
    % Wave 2D: f_D random-walk innovation variance (Phase 5 §5.4 Q55, baseline 0)
    if isfield(config, 'sigma2_w_fD')
        eq17_opts.sigma2_w_fD = config.sigma2_w_fD;
    else
        eq17_opts.sigma2_w_fD = 0;
    end
    % Phase 5 §5.5: a_x random-walk innovation variance (Q77 floor, baseline 0)
    if isfield(config, 'sigma2_w_fA')
        eq17_opts.sigma2_w_fA = config.sigma2_w_fA;
    else
        eq17_opts.sigma2_w_fA = 0;
    end
    % Stage 11 Option I: per-axis effective C_dpmr_eff / C_np_eff from calc_ctrl_params
    if isfield(P.ctrl, 'C_dpmr_eff_per_axis')
        eq17_opts.C_dpmr_eff_per_axis = P.ctrl.C_dpmr_eff_per_axis;
    end
    if isfield(P.ctrl, 'C_np_eff_per_axis')
        eq17_opts.C_np_eff_per_axis = P.ctrl.C_np_eff_per_axis;
    end
    % X2a: per-axis empirical IF_eff calibration (Phase 9 Stage I).
    if isfield(P.ctrl, 'IF_eff_calibrated_per_axis') && ...
            ~isempty(P.ctrl.IF_eff_calibrated_per_axis)
        eq17_opts.IF_eff_calibrated = P.ctrl.IF_eff_calibrated_per_axis;
    end
    % Wave 4 motion-ramp test pass-throughs
    if isfield(config, 'force_Q77_zero')
        eq17_opts.force_Q77_zero = config.force_Q77_zero;
    end
    if isfield(config, 'h_dot_max_override')
        eq17_opts.h_dot_max_override = config.h_dot_max_override;
    end
    if isfield(config, 'h_ddot_max_override')
        eq17_opts.h_ddot_max_override = config.h_ddot_max_override;
    end
    if isfield(config, 'Pf_init_slot7')
        eq17_opts.Pf_init_slot7 = config.Pf_init_slot7;
    end
    if isfield(config, 'sigma2_w_a_direct')
        eq17_opts.sigma2_w_a_direct = config.sigma2_w_a_direct;
    end
    if isfield(config, 'Q66_physical_mode')
        eq17_opts.Q66_physical_mode = config.Q66_physical_mode;
    end
    if isfield(config, 'Q66_OL_mode')
        eq17_opts.Q66_OL_mode = config.Q66_OL_mode;
    end
    if isfield(config, 'R22_include_Q66')
        eq17_opts.R22_include_Q66 = config.R22_include_Q66;
    end
    if isfield(config, 'Q36_cross_term')
        eq17_opts.Q36_cross_term = config.Q36_cross_term;
    end
    if isfield(config, 'use_joseph_form')
        eq17_opts.use_joseph_form = config.use_joseph_form;
    end
    if isfield(config, 'K_h_cap')
        eq17_opts.K_h_cap = config.K_h_cap;
    end
    if isfield(config, 'iir_warmup_mode') && ~isempty(config.iir_warmup_mode)
        eq17_opts.iir_warmup_mode = config.iir_warmup_mode;
    end
    % a_m LPF study (am_lpf_r22_design.md): post-LPF on a_xm -> a_m_det + R22
    % rescale. 6-state and 5-state (Vpersonal) both consume it via
    % build_eq17_6state_constants; 7-state and other paths leave
    % config.use_am_lpf unset (no-op).
    if isfield(config, 'use_am_lpf') && ~isempty(config.use_am_lpf)
        eq17_opts.use_am_lpf = config.use_am_lpf;
    end
    if isfield(config, 'a_det') && ~isempty(config.a_det)
        eq17_opts.a_det = config.a_det;
    end
    % ---- Dispatch A1: 6-state / 5-state / 4-state (Vpersonal) vs 7-state (eq17_core) ----
    is_6state = isfield(config, 'eq17_variant') && strcmpi(config.eq17_variant, '6state');
    is_5state = isfield(config, 'eq17_variant') && strcmpi(config.eq17_variant, '5state');
    is_5state_aprime = isfield(config, 'eq17_variant') && strcmpi(config.eq17_variant, '5state_aprime');
    is_4state = isfield(config, 'eq17_variant') && strcmpi(config.eq17_variant, '4state');
    is_4state_selfdet = isfield(config, 'eq17_variant') && strcmpi(config.eq17_variant, '4state_selfdet');   % TEMP diagnostic (chat 2026-07-05); remove after use
    is_4state_aprime_ff = isfield(config, 'eq17_variant') && strcmpi(config.eq17_variant, '4state_aprime_ff');   % TEMP diagnostic (chat 2026-07-06); remove after use
    is_4state_nocheat = isfield(config, 'eq17_variant') && strcmpi(config.eq17_variant, '4state_nocheat');   % TEMP diagnostic (chat 2026-07-06); remove after use
    is_4state_adet_only = isfield(config, 'eq17_variant') && strcmpi(config.eq17_variant, '4state_adet_only');   % TEMP diagnostic (chat 2026-07-06); remove after use
    is_4state_adet_only_newq = isfield(config, 'eq17_variant') && strcmpi(config.eq17_variant, '4state_adet_only_newq');   % TEMP diagnostic (chat 2026-07-06); remove after use
    is_4state_adet_only_gfix = isfield(config, 'eq17_variant') && strcmpi(config.eq17_variant, '4state_adet_only_gfix');   % TEMP oracle g-correction counter-side test (chat 2026-07-07); remove after use
    is_4state_deploy = isfield(config, 'eq17_variant') && strcmpi(config.eq17_variant, '4state_deploy');   % TEMP deployable model-free variant (chat 2026-07-07); remove after use
    is_4state_deploy_kf = isfield(config, 'eq17_variant') && strcmpi(config.eq17_variant, '4state_deploy_kf');   % TEMP KF-weighted deployable variant (chat 2026-07-07); remove after use
    is_4state_selfrw = isfield(config, 'eq17_variant') && strcmpi(config.eq17_variant, '4state_selfrw');   % TEMP a_det:=a_hat RW + fully self-diff a' (chat 2026-07-07); remove after use
    is_gscalar = isfield(config, 'eq17_variant') && strcmpi(config.eq17_variant, 'gscalar');
    is_vpersonal = is_6state || is_5state || is_5state_aprime || is_4state || is_4state_selfdet || is_4state_aprime_ff || is_4state_nocheat || is_4state_adet_only || is_4state_adet_only_newq || is_4state_adet_only_gfix || is_4state_deploy || is_4state_deploy_kf || is_4state_selfrw || is_gscalar;   % share prefill defaults + constants builder
    if opts.use_true_gain && ~is_vpersonal
        error('run_pure_simulation:useTrueGainUnsupported', ...
              'opts.use_true_gain=true requires config.eq17_variant=''6state'', ''5state'', ''5state_aprime'', ''4state'' or ''gscalar''.');
    end
    if is_vpersonal
        % Prefill three-pillar defaults (override 7-state warmup defaults
        % unless the caller explicitly set them). The 5-state controller
        % reuses build_eq17_6state_constants (offline scalars are
        % dimension-agnostic; only the EKF state dimension differs).
        if ~isfield(config, 't_warmup_kf') || isempty(config.t_warmup_kf)
            eq17_opts.t_warmup_kf = 0;
        end
        if ~isfield(config, 'iir_warmup_mode') || isempty(config.iir_warmup_mode)
            eq17_opts.iir_warmup_mode = 'prefill';
        end
        ctrl_const = build_eq17_6state_constants(eq17_opts);
    else
        ctrl_const = build_eq17_constants(eq17_opts);
    end

    % Phase 9 R(2,2) validation: optional a_hat freeze (locks state(6) per axis)
    if ~isempty(opts.a_hat_freeze)
        ctrl_const.a_hat_freeze = opts.a_hat_freeze(:);   % 3x1 [um/pN]
    end

    % Diagnostic: optional suppress x_D in control law (EKF still estimates
    % x_D in slot 4; only the term −x̂_D/â_x is zeroed in Eq.17 force law).
    if isfield(config, 'suppress_xD') && ~isempty(config.suppress_xD)
        ctrl_const.suppress_xD = logical(config.suppress_xD);
    end

    % 4-state de-blur: use a' shape to remove the a_xm variance-window h-blur
    % (motion_control_law_eq17_4state). Default off -> bit-identical to baseline.
    if isfield(config, 'use_deblur') && ~isempty(config.use_deblur)
        ctrl_const.use_deblur = logical(config.use_deblur);
    end

    % Override the gain process-noise increment factor (Q44 / var_da_ram scale).
    % With a' fed forward the deterministic gain swing is known, so the residual
    % gain process noise can be shrunk (toward 0) to stop a_hat wandering.
    if isfield(config, 'var_da_increment_factor') && ~isempty(config.var_da_increment_factor)
        ctrl_const.var_da_increment_factor = config.var_da_increment_factor;
    end

    % 4-state principled Q44: a' feed-forward of the deviation-induced gain change
    % (-a'*Delta(dxhat3)) + residual Q44 = a'^2*2(1-lc)*P33 (4state_del_hd Remark).
    if isfield(config, 'use_aprime_ff') && ~isempty(config.use_aprime_ff)
        ctrl_const.use_aprime_ff = logical(config.use_aprime_ff);
    end

    % 4-state bounded-variance cap: Q44 = Var(a_x_ram)^2/R22 (caps the random-walk
    % gain wander at the true bounded gain fluctuation). Q44-only, no F_e change.
    if isfield(config, 'use_q44_cap') && ~isempty(config.use_q44_cap)
        ctrl_const.use_q44_cap = logical(config.use_q44_cap);
    end

    % 4-state AR(1) reverting gain: F_e(4,4)=lc + predict reverts to a_det +
    % Q44 = (3-2lc^2)(a*K_h/R)^2 sigma2_dh (the structural "put the spring back").
    if isfield(config, 'use_q44_ar1') && ~isempty(config.use_q44_ar1)
        ctrl_const.use_q44_ar1 = logical(config.use_q44_ar1);
    end

    % TEMP (chat 2026-07-07): 4state_selfrw adet_known toggle (RW a_det:=a_hat vs
    % AR(1) reversion to KNOWN a_det, both with self-diff a'); remove with variant.
    if isfield(config, 'adet_known') && ~isempty(config.adet_known)
        ctrl_const.adet_known = logical(config.adet_known);
    end
    % TEMP (chat 2026-07-08): exact row-4 pole F_e(4,4)=lc+a'*F_dx in the AR(1)
    % covariance propagation (4state_del_hd.tex p.7); remove with variant.
    if isfield(config, 'use_exact_fe44') && ~isempty(config.use_exact_fe44)
        ctrl_const.use_exact_fe44 = logical(config.use_exact_fe44);
    end
    % Taylor-gain full suite (chat 2026-07-10, 4state_del_hd.tex taylor section): z-axis
    % F_e row 4 [0 0 (1-lc)a' 1+a'F_dx] + rank-1 Q(3:4,3:4) + predict deviation
    % correction; aprime_source = 'known' | 'ahat' | 'diff'.
    if isfield(config, 'use_taylor_gain') && ~isempty(config.use_taylor_gain)
        ctrl_const.use_taylor_gain = logical(config.use_taylor_gain);
    end
    if isfield(config, 'aprime_eval_xhat') && ~isempty(config.aprime_eval_xhat)
        ctrl_const.aprime_eval_xhat = logical(config.aprime_eval_xhat);   % EXPERIMENT (chat 2026-07-13)
    end
    if isfield(config, 'aprime_source') && ~isempty(config.aprime_source)
        ctrl_const.aprime_source = config.aprime_source;
    end
    % TEMP (chat 2026-07-15, E2): a'-freeze scale passthrough.
    if isfield(config, 'aprime_scale') && ~isempty(config.aprime_scale)
        ctrl_const.aprime_scale = config.aprime_scale;
    end
    % TEMP (chat 2026-07-16, D2): deliberately-wrong a_hat gain-state init scale.
    if isfield(config, 'a_hat_init_scale') && ~isempty(config.a_hat_init_scale)
        ctrl_const.a_hat_init_scale = config.a_hat_init_scale;
    end
    % TEMP (chat 2026-07-15, G4): AR1-known-level + self-diff a' passthrough.
    if isfield(config, 'g4_selfdiff_ar1') && ~isempty(config.g4_selfdiff_ar1)
        ctrl_const.g4_selfdiff_ar1 = config.g4_selfdiff_ar1;
    end
    % TEMP (chat 2026-07-15, V6): closed-loop var-channel a' EWMA knobs.
    if isfield(config, 'varchan_tau') && ~isempty(config.varchan_tau)
        ctrl_const.varchan_tau = config.varchan_tau;
    end
    if isfield(config, 'varchan_warm') && ~isempty(config.varchan_warm)
        ctrl_const.varchan_warm = config.varchan_warm;
    end
    % TEMP (chat 2026-07-16, E): scale the varchan a' init seed.
    if isfield(config, 'varchan_ap_init_scale') && ~isempty(config.varchan_ap_init_scale)
        ctrl_const.varchan_ap_init_scale = config.varchan_ap_init_scale;
    end
    % TEMP (chat 2026-07-17, E100): charter-legal init basis + P44 prior.
    if isfield(config, 'init_from_anom') && ~isempty(config.init_from_anom)
        ctrl_const.init_from_anom = config.init_from_anom;
    end
    if isfield(config, 'varchan_warm_steps') && ~isempty(config.varchan_warm_steps)
        ctrl_const.varchan_warm_steps = config.varchan_warm_steps;
    end
    % TEMP (chat 2026-07-17, E1SO): full F(4,5) Jacobian (selfmod coupling).
    if isfield(config, 'aprime_state_selfmod') && ~isempty(config.aprime_state_selfmod)
        ctrl_const.aprime_state_selfmod = config.aprime_state_selfmod;
    end
    if isfield(config, 'p44_prior_frac') && ~isempty(config.p44_prior_frac)
        ctrl_const.p44_prior_frac = config.p44_prior_frac;
    end
    if isfield(config, 'aprime_diff_beta') && ~isempty(config.aprime_diff_beta)
        ctrl_const.aprime_diff_beta = config.aprime_diff_beta;
    end
    if isfield(config, 'aprime_diff_gate_um') && ~isempty(config.aprime_diff_gate_um)
        ctrl_const.aprime_diff_gate_um = config.aprime_diff_gate_um;
    end
    if isfield(config, 'aprime_clamp_pos') && ~isempty(config.aprime_clamp_pos)
        ctrl_const.aprime_clamp_pos = config.aprime_clamp_pos;
    end
    if isfield(config, 'aprime_pos_only') && ~isempty(config.aprime_pos_only)
        ctrl_const.aprime_pos_only = config.aprime_pos_only;
    end
    % TEMP (chat 2026-07-10): positivity gate on the self-diff a' (physical
    % prior a' > 0; negative raw slopes are skipped); remove with variant.
    if isfield(config, 'selfdet_pos_only') && ~isempty(config.selfdet_pos_only)
        ctrl_const.selfdet_pos_only = logical(config.selfdet_pos_only);
    end
    % TEMP (chat 2026-07-10): a'>0 prior as a post-EWMA clamp instead
    % (max(a_prime_hat,0)); remove with variant.
    if isfield(config, 'selfdet_clamp_pos') && ~isempty(config.selfdet_clamp_pos)
        ctrl_const.selfdet_clamp_pos = logical(config.selfdet_clamp_pos);
    end
    % TEMP (chat 2026-07-10): oracle a' ablation (known-wall slope at h_bar_d,
    % a_det stays self-anchored); remove with variant.
    if isfield(config, 'use_true_aprime') && ~isempty(config.use_true_aprime)
        ctrl_const.use_true_aprime = logical(config.use_true_aprime);
    end

    % gscalar-only knobs: forgetting factor + h_bar source for a_det.
    if isfield(config, 'beta_s') && ~isempty(config.beta_s)
        ctrl_const.beta_s = config.beta_s;
    end
    if isfield(config, 'use_hbar_meas_for_adet') && ~isempty(config.use_hbar_meas_for_adet)
        ctrl_const.use_hbar_meas_for_adet = logical(config.use_hbar_meas_for_adet);
    end
    if isfield(config, 's_I_prior') && ~isempty(config.s_I_prior)
        ctrl_const.s_I_prior = config.s_I_prior;
    end

    % 5state_aprime knobs: slope process-noise scale, prior, freeze (L0).
    if isfield(config, 'Q_aprime_factor') && ~isempty(config.Q_aprime_factor)
        ctrl_const.Q_aprime_factor = config.Q_aprime_factor;
    end
    if isfield(config, 'Pf_aprime_scale') && ~isempty(config.Pf_aprime_scale)
        ctrl_const.Pf_aprime_scale = config.Pf_aprime_scale;
    end
    if isfield(config, 'freeze_aprime') && ~isempty(config.freeze_aprime)
        ctrl_const.freeze_aprime = logical(config.freeze_aprime);
    end
    % Self-modulation (hold-observability) + honest Q55 -- see
    % reference/eq17_analysis/derivation/5state_aprime_unified.tex Sec.3-5,7.
    if isfield(config, 'use_selfmod') && ~isempty(config.use_selfmod)
        ctrl_const.use_selfmod = logical(config.use_selfmod);
    end

    % 4state a'-as-state (aprime_source='state', 5state_taylor_aprime.pdf):
    % Q55 scale, a' prior variance, and the learn-gate start time. Default off
    % (only active with use_taylor_gain + aprime_source='state').
    if isfield(config, 'aprime_state_kappa') && ~isempty(config.aprime_state_kappa)
        ctrl_const.aprime_state_kappa = config.aprime_state_kappa;
    end
    if isfield(config, 'aprime_state_P0') && ~isempty(config.aprime_state_P0)
        ctrl_const.aprime_state_P0 = config.aprime_state_P0;
    end
    if isfield(config, 'aprime_learn_t0') && ~isempty(config.aprime_learn_t0)
        ctrl_const.aprime_learn_t0 = config.aprime_learn_t0;
    end
    % TEMP (chat 2026-07-19, AG): a'-state Q55 floor (PLACEHOLDER) passthrough.
    if isfield(config, 'aprime_state_Q55_floor') && ~isempty(config.aprime_state_Q55_floor)
        ctrl_const.aprime_state_Q55_floor = config.aprime_state_Q55_floor;
    end
    % TEMP (chat 2026-07-20, honesty audit): colored-y2 AR(1) v-state knobs.
    if isfield(config, 'y2_noise_model') && ~isempty(config.y2_noise_model)
        ctrl_const.y2_noise_model = config.y2_noise_model;
    end
    if isfield(config, 'y2_ar1_phi') && ~isempty(config.y2_ar1_phi)
        ctrl_const.y2_ar1_phi = config.y2_ar1_phi;
    end
    if isfield(config, 'y2_ar1_r2_floor_frac') && ~isempty(config.y2_ar1_r2_floor_frac)
        ctrl_const.y2_ar1_r2_floor_frac = config.y2_ar1_r2_floor_frac;
    end

    % TEMP (chat 2026-07-05): 4state_selfdet gate+EWMA knobs; remove with the variant.
    if isfield(config, 'selfdet_gate_thresh') && ~isempty(config.selfdet_gate_thresh)
        ctrl_const.selfdet_gate_thresh = config.selfdet_gate_thresh;
    end
    if isfield(config, 'selfdet_beta') && ~isempty(config.selfdet_beta)
        ctrl_const.selfdet_beta = config.selfdet_beta;
    end
    % TEMP (chat 2026-07-07): deploy-variant g-hat estimator knobs; remove with the variant.
    if isfield(config, 'g_gate_thresh') && ~isempty(config.g_gate_thresh)
        ctrl_const.g_gate_thresh = config.g_gate_thresh;
    end
    if isfield(config, 'g_beta') && ~isempty(config.g_beta)
        ctrl_const.g_beta = config.g_beta;
    end
    if isfield(config, 'g_decay') && ~isempty(config.g_decay)
        ctrl_const.g_decay = config.g_decay;
    end
    if isfield(config, 'kf_q44_scale') && ~isempty(config.kf_q44_scale)
        ctrl_const.kf_q44_scale = config.kf_q44_scale;
    end

    % ------------------------------------------------------------------
    % 5. Time grid (matches Simulink: discrete samples 0, Ts, 2Ts, ...)
    % ------------------------------------------------------------------
    Ts = P.common.Ts;
    T_sim = config.T_sim;
    N = round(T_sim / Ts) + 1;            % number of discrete samples
    tout = (0:N-1)' * Ts;                 % column vector [Nx1, sec]

    % ------------------------------------------------------------------
    % 6. Initialize state
    % ------------------------------------------------------------------
    p0 = P.common.p0;                     % 3x1 [um]
    p_curr = p0;                          % continuous state at sample boundary

    % Sensor-delay buffer of length d+1. Newest at end, oldest at index 1.
    % Initial condition matches Simulink Delay(d, Ts) IC=p0.
    %
    % Phase 8 Wave 2D fix (2026-04-29): a buffer of length d implements only
    % a (d-1)-step delay because the per-step ordering is
    %   (i)  read p_m_delayed = buffer(:,1)            <-- read BEFORE shift
    %   (ii) shift: buffer = [buffer(:,2:end), p_m_raw]
    % so on step k, buffer(:,1) = p_m at step (k - (d-1)).
    %
    % To realize a true d-step delay (matches design spec + Simulink Delay(d)),
    % buffer must hold (d+1) entries: the read at (i) returns p_m at step
    % (k - d). Agent B's controller has pd_km1/pd_km2 aligned to d=2 (Eq.17 v2).
    %
    % References: phase8_eq17_state_audit.md §K item 2; phase8_wave2D_driver.md
    d_delay = ctrl_const.d;
    p_m_buffer = repmat(p0, 1, d_delay + 1);  % [3 x (d+1)]

    % Trajectory unit-delay buffer:
    %   trajectory_generator returns p_d[k+1]; controller wants pd[k] and
    %   p_d_out logs pd[k]. Unit Delay IC = p0 → pd_for_ctrl[1] = p0.
    pd_for_ctrl = p0;                     % 3x1 [um]

    % True-gain support: a_true at the PRE-integration p_curr (the position
    % the controller acts on at step k; p_true_out logs post-integration).
    a_nom_drv   = P.common.Ts / P.common.gamma_N;
    wall_on_drv = isfield(P, 'wall') && P.wall.enable_wall_effect > 0.5;
    % True-gain h̄ clip floor: keep consistent with the physics h_min floor so
    % diverged excursions do not silently use a different singularity limit.
    if wall_on_drv && isfield(P.wall, 'h_bar_min')
        h_bar_floor_drv = P.wall.h_bar_min;
    else
        h_bar_floor_drv = 1.001;
    end

    % ------------------------------------------------------------------
    % 7. Allocate logs (mirror Simulink ToWorkspace schema, [N x 3])
    % ------------------------------------------------------------------
    p_d_out  = zeros(N, 3);
    f_d_out  = zeros(N, 3);
    F_th_out = zeros(N, 3);
    p_m_out  = zeros(N, 3);
    p_true_out = zeros(N, 3);    % ground-truth probe: noise-free true position [um]
    a_true_out = zeros(N, 3);    % true gain at PRE-integration p_curr (position the controller acts on) [um/pN]
    ekf_out  = zeros(N, 4);

    % Phase 9 R(2,2) validation: optional diag time-series accumulators
    if opts.collect_diag
        diag_log.dx_r              = zeros(N, 3);
        diag_log.sigma2_dxr_hat    = zeros(N, 3);
        diag_log.a_xm              = zeros(N, 3);
        diag_log.a_m_det           = zeros(N, 3);   % a_m LPF study: post-LPF gain measurement
        diag_log.a_x_det           = zeros(N, 3);   % deterministic gain a_nom/c(h_bar_d) from p_d
        diag_log.a_prime_hat       = zeros(N, 3);   % TEMP (chat 2026-07-06): self-differenced a' estimate; remove after use
        diag_log.g_hat             = zeros(N, 3);   % TEMP (chat 2026-07-07): p_md-based mismatch estimate; remove after use
        diag_log.a_xm_corr         = zeros(N, 3);   % TEMP (chat 2026-07-07): g-corrected a_xm; remove after use
        diag_log.delta_x_m         = zeros(N, 3);
        diag_log.innovation_y2     = zeros(N, 3);
        diag_log.S_y1              = zeros(N, 3);   % TEMP (chat 2026-07-20 honesty audit)
        diag_log.S_y2              = zeros(N, 3);
        diag_log.K_kf_a_y2         = zeros(N, 3);
        diag_log.K_kf_dx_y1        = zeros(N, 3);
        diag_log.innovation_y1     = zeros(N, 3);   % v-budget diag (4state taylor)
        diag_log.K_kf_a_y1         = zeros(N, 3);   % L(4,1)
        diag_log.K_kf_dx_y2        = zeros(N, 3);   % L(3,2)
        diag_log.P_a               = zeros(N, 3);
        diag_log.P_dx              = zeros(N, 3);
        diag_log.P77               = zeros(N, 3);
        diag_log.Q77               = zeros(N, 3);
        diag_log.var_da_ram        = zeros(N, 3);   % Q44 driver (gain process-noise) time series
        diag_log.a_prime_used      = zeros(N, 3);   % taylor-gain slope a' time series
        diag_log.P_aprime          = zeros(N, 3);   % a'-state variance P(5,5) (z; aprime_source='state')
        diag_log.a_hat             = zeros(N, 3);
        diag_log.x_D_hat           = zeros(N, 3);
        diag_log.delta_a_hat       = zeros(N, 3);
        diag_log.gate_active       = false(N, 3);
        diag_log.guards_individual = false(N, 3, 3);   % [N x guard x axis]
        diag_log.h_bar             = zeros(N, 1);
        diag_log.delta_x_hat_1     = zeros(N, 3);
        diag_log.delta_x_hat_3     = zeros(N, 3);   % current tracking-error estimate (slot 3)
        diag_log.P_dx1             = zeros(N, 3);
        diag_log.a_ctrl_used       = zeros(N, 3);
        diag_log.s_hat             = zeros(N, 1);   % gscalar: scalar gain estimate
        diag_log.var_s             = zeros(N, 1);   % gscalar: Var(s_hat) = 1/I_g
        % TEMP (chat 2026-07-17, E100 chain zoom): varchan internals (z, scalar)
        diag_log.vc_s2ar  = zeros(N, 1);
        diag_log.vc_s2dh  = zeros(N, 1);
        diag_log.vc_abar  = zeros(N, 1);
        diag_log.vc_dbar  = zeros(N, 1);
        diag_log.vc_ap_hat = zeros(N, 1);
        diag_log.vc_var_k  = zeros(N, 1);
    end

    % ------------------------------------------------------------------
    % 8. Time-stepping loop  (matches Simulink ordering)
    % ------------------------------------------------------------------
    if opts.verbose
        fprintf('[run_pure_simulation] N=%d steps, Ts=%.4e s, scheme=%s, seed=%d\n', ...
                N, Ts, opts.scheme, opts.seed);
    end

    for k = 1:N
        t_now = tout(k);

        % --- (a) Trajectory: returns pd[k+1] and del_pd[k] = pd[k+1]-pd[k]
        [pd_kp1, del_pd_k] = trajectory_generator(t_now, P);

        % --- (b) Controller input pd[k] is the unit-delayed trajectory
        pd_k = pd_for_ctrl;

        % --- (c) Sensor-delayed p_m for controller (head of buffer)
        p_m_delayed = p_m_buffer(:, 1);

        % --- (c2) Ground-truth gain at current (pre-integration) position
        if wall_on_drv
            h_bar_true_k = max((dot(p_curr, P.wall.w_hat) - P.wall.pz) / P.common.R, h_bar_floor_drv);
            [c_para_k, c_perp_k] = calc_correction_functions(h_bar_true_k);
            a_true_k = [a_nom_drv / c_para_k; a_nom_drv / c_para_k; a_nom_drv / c_perp_k];
        else
            a_true_k = a_nom_drv * ones(3, 1);
        end
        if opts.use_true_gain
            a_override_k = opts.true_gain_scale * a_true_k;   % TEMP fixed-g probe (chat 2026-07-06)
        else
            a_override_k = [];
        end

        % --- (d) Controller + EKF  (dispatch A1: 4-state / 5-state / 6-state vs 7-state)
        if is_4state
            if opts.collect_diag
                [f_d_k, ekf_k, diag_k] = temp_motion_control_law_eq17_4state_centered( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            else
                [f_d_k, ekf_k] = temp_motion_control_law_eq17_4state_centered( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            end
        elseif is_5state
            if opts.collect_diag
                [f_d_k, ekf_k, diag_k] = motion_control_law_eq17_5state( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            else
                [f_d_k, ekf_k] = motion_control_law_eq17_5state( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            end
        elseif is_5state_aprime
            if opts.collect_diag
                [f_d_k, ekf_k, diag_k] = motion_control_law_eq17_5state_aprime( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            else
                [f_d_k, ekf_k] = motion_control_law_eq17_5state_aprime( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            end
        elseif is_6state
            if opts.collect_diag
                [f_d_k, ekf_k, diag_k] = motion_control_law_eq17_6state( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            else
                [f_d_k, ekf_k] = motion_control_law_eq17_6state( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            end
        elseif is_gscalar
            if opts.collect_diag
                [f_d_k, ekf_k, diag_k] = motion_control_law_eq17_gscalar( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            else
                [f_d_k, ekf_k] = motion_control_law_eq17_gscalar( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            end
        elseif is_4state_selfdet   % TEMP diagnostic (chat 2026-07-05); remove after use
            if opts.collect_diag
                [f_d_k, ekf_k, diag_k] = temp_motion_control_law_eq17_4state_selfdet( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            else
                [f_d_k, ekf_k] = temp_motion_control_law_eq17_4state_selfdet( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            end
        elseif is_4state_aprime_ff   % TEMP diagnostic (chat 2026-07-06); remove after use
            if opts.collect_diag
                [f_d_k, ekf_k, diag_k] = temp_motion_control_law_eq17_4state_aprime_ff( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            else
                [f_d_k, ekf_k] = temp_motion_control_law_eq17_4state_aprime_ff( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            end
        elseif is_4state_nocheat   % TEMP diagnostic (chat 2026-07-06); remove after use
            if opts.collect_diag
                [f_d_k, ekf_k, diag_k] = temp_motion_control_law_eq17_4state_nocheat( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            else
                [f_d_k, ekf_k] = temp_motion_control_law_eq17_4state_nocheat( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            end
        elseif is_4state_adet_only   % TEMP diagnostic (chat 2026-07-06); remove after use
            if opts.collect_diag
                [f_d_k, ekf_k, diag_k] = temp_motion_control_law_eq17_4state_adet_only( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            else
                [f_d_k, ekf_k] = temp_motion_control_law_eq17_4state_adet_only( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            end
        elseif is_4state_adet_only_newq   % TEMP diagnostic (chat 2026-07-06); remove after use
            if opts.collect_diag
                [f_d_k, ekf_k, diag_k] = temp_motion_control_law_eq17_4state_adet_only_newq( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            else
                [f_d_k, ekf_k] = temp_motion_control_law_eq17_4state_adet_only_newq( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            end
        elseif is_4state_adet_only_gfix   % TEMP oracle g-correction counter-side test (chat 2026-07-07); remove after use
            if opts.collect_diag
                [f_d_k, ekf_k, diag_k] = temp_motion_control_law_eq17_4state_adet_only_gfix( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k, a_true_k);
            else
                [f_d_k, ekf_k] = temp_motion_control_law_eq17_4state_adet_only_gfix( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k, a_true_k);
            end
        elseif is_4state_deploy   % TEMP deployable model-free variant (chat 2026-07-07); remove after use
            if opts.collect_diag
                [f_d_k, ekf_k, diag_k] = temp_motion_control_law_eq17_4state_deploy( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            else
                [f_d_k, ekf_k] = temp_motion_control_law_eq17_4state_deploy( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            end
        elseif is_4state_deploy_kf   % TEMP KF-weighted deployable variant (chat 2026-07-07); remove after use
            if opts.collect_diag
                [f_d_k, ekf_k, diag_k] = temp_motion_control_law_eq17_4state_deploy_kf( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            else
                [f_d_k, ekf_k] = temp_motion_control_law_eq17_4state_deploy_kf( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            end
        elseif is_4state_selfrw   % TEMP a_det:=a_hat RW + fully self-diff a' (chat 2026-07-07); remove after use
            if opts.collect_diag
                [f_d_k, ekf_k, diag_k] = temp_motion_control_law_eq17_4state_selfrw( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            else
                [f_d_k, ekf_k] = temp_motion_control_law_eq17_4state_selfrw( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const, a_override_k);
            end
        else
            if opts.collect_diag
                [f_d_k, ekf_k, diag_k] = motion_control_law_eq17_core( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const);
            else
                [f_d_k, ekf_k] = motion_control_law_eq17_core( ...
                                    del_pd_k, pd_k, p_m_delayed, P, ctrl_const);
            end
        end

        % --- (e) Thermal force at current continuous-state position
        %   NOTE: calc_thermal_force does NOT check P.thermal.enable internally;
        %   gate is at driver level (matches Simulink block-level enable).
        if P.thermal.enable > 0.5
            f_th_k = calc_thermal_force(p_curr, P);
        else
            f_th_k = zeros(3, 1);
        end

        % --- (f) Total force (ZOH constant over [t_k, t_k + Ts])
        F_total = f_d_k + f_th_k;

        % --- (g) Continuous dynamics: integrate one Ts using ode4 substeps
        p_curr = step_dynamics(p_curr, F_total, P, Ts);

        % --- (h) Measurement noise injection (matches Simulink Measurement_Noise)
        if config.meas_noise_enable
            n_meas = config.meas_noise_std(:) .* randn(3, 1);
        else
            n_meas = zeros(3, 1);
        end
        p_m_raw = p_curr + n_meas;        % tapped to p_m_out (before delay)

        % --- (i) Shift sensor-delay buffer: drop oldest, append newest
        p_m_buffer = [p_m_buffer(:, 2:end), p_m_raw];

        % --- (j) Logging
        p_d_out(k, :)  = pd_k.';
        f_d_out(k, :)  = f_d_k.';
        F_th_out(k, :) = f_th_k.';
        p_m_out(k, :)  = p_m_raw.';
        p_true_out(k, :) = p_curr.';     % ground-truth probe (noise-free position)
        a_true_out(k, :) = a_true_k.';
        ekf_out(k, :)  = ekf_k.';

        % Phase 9 R(2,2) validation: log diag time-series
        if opts.collect_diag
            diag_log.dx_r(k, :)              = diag_k.dx_r.';
            diag_log.sigma2_dxr_hat(k, :)    = diag_k.sigma2_dxr_hat.';
            diag_log.a_xm(k, :)              = diag_k.a_xm.';
            if isfield(diag_k, 'a_m_det'); diag_log.a_m_det(k, :) = diag_k.a_m_det.'; end
            if isfield(diag_k, 'a_x_det'); diag_log.a_x_det(k, :) = diag_k.a_x_det.'; end
            if isfield(diag_k, 'a_prime_hat'); diag_log.a_prime_hat(k, :) = diag_k.a_prime_hat.'; end   % TEMP (chat 2026-07-06); remove after use
            if isfield(diag_k, 'g_hat'); diag_log.g_hat(k, :) = diag_k.g_hat.'; end   % TEMP (chat 2026-07-07); remove after use
            if isfield(diag_k, 'a_xm_corr'); diag_log.a_xm_corr(k, :) = diag_k.a_xm_corr.'; end   % TEMP (chat 2026-07-07); remove after use
            diag_log.delta_x_m(k, :)         = diag_k.delta_x_m.';
            diag_log.innovation_y2(k, :)     = diag_k.innovation_y2.';
            if isfield(diag_k, 'S_y1'); diag_log.S_y1(k, :) = diag_k.S_y1.'; end
            if isfield(diag_k, 'S_y2'); diag_log.S_y2(k, :) = diag_k.S_y2.'; end
            diag_log.K_kf_a_y2(k, :)         = diag_k.K_kf_a_y2.';
            diag_log.K_kf_dx_y1(k, :)        = diag_k.K_kf_dx_y1.';
            if isfield(diag_k, 'innovation_y1'); diag_log.innovation_y1(k, :) = diag_k.innovation_y1.'; end
            if isfield(diag_k, 'K_kf_a_y1');     diag_log.K_kf_a_y1(k, :)     = diag_k.K_kf_a_y1.';     end
            if isfield(diag_k, 'K_kf_dx_y2');    diag_log.K_kf_dx_y2(k, :)    = diag_k.K_kf_dx_y2.';    end
            diag_log.P_a(k, :)               = diag_k.P_a.';
            diag_log.P_dx(k, :)              = diag_k.P_dx.';
            diag_log.P77(k, :)               = diag_k.P77.';
            diag_log.Q77(k, :)               = diag_k.Q77.';
            if isfield(diag_k, 'var_da_ram'); diag_log.var_da_ram(k, :) = diag_k.var_da_ram.'; end
            if isfield(diag_k, 'a_prime_used'); diag_log.a_prime_used(k, :) = diag_k.a_prime_used.'; end
            if isfield(diag_k, 'P_aprime'); diag_log.P_aprime(k, :) = diag_k.P_aprime.'; end
            diag_log.a_hat(k, :)             = diag_k.a_hat.';
            diag_log.x_D_hat(k, :)           = diag_k.x_D_hat.';
            diag_log.delta_a_hat(k, :)       = diag_k.delta_a_hat.';
            diag_log.gate_active(k, :)       = diag_k.gate_active_per_axis(:).';
            diag_log.guards_individual(k, :, :) = diag_k.guards_individual;
            diag_log.h_bar(k)                = diag_k.h_bar;
            diag_log.delta_x_hat_1(k, :)     = diag_k.delta_x_hat_1.';
            if isfield(diag_k, 'delta_x_hat_3'); diag_log.delta_x_hat_3(k, :) = diag_k.delta_x_hat_3.'; end
            diag_log.P_dx1(k, :)             = diag_k.P_dx1.';
            diag_log.a_ctrl_used(k, :)       = diag_k.a_ctrl_used.';
            if isfield(diag_k, 's_hat'); diag_log.s_hat(k) = diag_k.s_hat; end
            if isfield(diag_k, 'var_s'); diag_log.var_s(k) = diag_k.var_s; end
            if isfield(diag_k, 'vc_s2ar')
                diag_log.vc_s2ar(k)   = diag_k.vc_s2ar;
                diag_log.vc_s2dh(k)   = diag_k.vc_s2dh;
                diag_log.vc_abar(k)   = diag_k.vc_abar;
                diag_log.vc_dbar(k)   = diag_k.vc_dbar;
                diag_log.vc_ap_hat(k) = diag_k.vc_ap_hat;
                diag_log.vc_var_k(k)  = diag_k.vc_var_k;
            end
        end

        % --- (k) Update unit-delay for next step's controller input
        pd_for_ctrl = pd_kp1;

        % --- (l) Verbose progress
        if opts.verbose && N >= 10 && mod(k, max(1, round(N/10))) == 0
            fprintf('  step %d/%d (t=%.3fs)\n', k, N, t_now);
        end
    end

    % ------------------------------------------------------------------
    % 9. Pack output (mirror Simulink ToWorkspace)
    % ------------------------------------------------------------------
    simOut.p_d_out  = p_d_out;
    simOut.f_d_out  = f_d_out;
    simOut.F_th_out = F_th_out;
    simOut.p_m_out  = p_m_out;
    simOut.p_true_out = p_true_out;
    simOut.a_true_out = a_true_out;
    simOut.tout     = tout;
    simOut.ekf_out  = ekf_out;
    simOut.meta = struct( ...
        'config',         config, ...
        'params_value',   P, ...
        'scheme',         opts.scheme, ...
        'seed',           opts.seed, ...
        'driver',         'run_pure_simulation', ...
        'driver_version', '1.1');

    % Phase 9 R(2,2) validation: optional diag time-series + ctrl_const
    if opts.collect_diag
        simOut.diag = diag_log;
    end
    simOut.ctrl_const = ctrl_const;
end
