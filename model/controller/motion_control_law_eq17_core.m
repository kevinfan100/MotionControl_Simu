function [f_d, ekf_out, diag] = motion_control_law_eq17_core(del_pd, pd, p_m, params, ctrl_const)
%MOTION_CONTROL_LAW_EQ17_7STATE Per-axis 7-state EKF controller (paper 2023 Eq.17, v2)
%
%   [f_d, ekf_out]       = motion_control_law_eq17_core(del_pd, pd, p_m, params, ctrl_const)
%   [f_d, ekf_out, diag] = motion_control_law_eq17_core(del_pd, pd, p_m, params, ctrl_const)
%
%   The optional 3rd output `diag` returns a struct of per-step diagnostics
%   (IIR variance, KF gains, gate flags, posteriors, ...). Computation is
%   skipped entirely when nargout < 3 (zero perf cost on existing 2-output
%   call sites).
%
%   ctrl_const.a_hat_freeze (optional, 3x1) — if present and non-empty, the
%   posterior a_x state (slot 6) is overridden with this value after both the
%   predict and update steps, and P(6,:)/P(:,6) are zeroed before update so
%   the filter cannot move it. Used for testing / breakthrough sweeps.
%
%   Implements 3 independent 7-state EKF controllers (one per axis x, y, z)
%   following paper 2023 Eq.(17) with d-step delay-compensated control law,
%   Σ f_d[k-i] term retained outside the (1/â_x) bracket (Strategy 1),
%   x̂_D additive disturbance compensation, adaptive Q33/Q55/Q77, and
%   3-guard adaptive R_2.
%
%   v2 differences from v1 (per Phase 1 §4.2 + Phase 5 + Phase 6):
%       * Σ_{i=1..d} f_d[k-i] retained at paper position (outside 1/â_x bracket)
%       * F_e(3,4) = -(1 + d·(1-λ_c))  (= -1.6 for d=2, λ_c=0.7), Eq.19 form
%       * Q55 closed form (a_nom_axis² · σ²_w_fD), default 0
%       * Wall-aware â_x[0] seeding via h_bar_init clamp
%       * Per-axis Pf_init derived from â_x_init
%       * Warmup: 2-step counter (NOT 320 steps); f_d=0 + IIR-only; seed δp_hat at end
%
%   References:
%       reference/eq17_analysis/phase1_Fe_derivation.md  §4.2, §10 (control law + F_e)
%       reference/eq17_analysis/phase2_C_dpmr_C_n_derivation.md  (C_dpmr/C_n/IF_var/ξ)
%       reference/eq17_analysis/phase5_Q_matrix_derivation.md   (Q33/Q55/Q77)
%       reference/eq17_analysis/phase6_R_matrix_derivation.md   (R(1,1)/R(2,2)/3-guard)
%       reference/eq17_analysis/design_v2.md §4 (control law) + §6 (a_cov=0.05)
%
%   State vector per axis (paper convention):
%       x_e[k] = [delta_x_1; delta_x_2; delta_x_3; x_D; delta_x_D; a_x; delta_a_x]
%
%       delta_x_1 = δx[k-2]  (oldest, matches y_1 direct measurement)
%       delta_x_2 = δx[k-1]
%       delta_x_3 = δx[k]    (current)
%       x_D       = lumped disturbance [um]
%       delta_x_D = x_D rate (random-walk velocity)
%       a_x       = motion gain [um/pN]
%       delta_a_x = a_x rate
%
%   Measurement (per axis):
%       y_1 = delta_x_m  = p_d[k-2] - p_m[k]      (delayed tracking error)
%       y_2 = a_xm       = (sigma2_dxr_hat - C_n*sigma2_n_s) / (C_dpmr * 4 kBT)
%
%   Inputs:
%       del_pd     - p_d[k+1] - p_d[k]              [3x1, um]
%       pd         - p_d[k]                          [3x1, um]
%       p_m        - p_m[k]   (already includes d-step sensor delay) [3x1, um]
%       params     - From calc_simulation_params (params.Value or params struct)
%       ctrl_const - From build_eq17_constants (offline, scalar constants)
%
%   Outputs:
%       f_d        - Control force f_d[k]            [3x1, pN]
%       ekf_out    - Diagnostic [4x1]:
%                       [a_hat_x; a_hat_z; a_hat_y; h_bar_current]
%                    (matches run_simulation.m extraction: idx 1,2)
%
%   See also: motion_control_law_7state, build_eq17_constants,
%             calc_correction_functions

    % ------------------------------------------------------------------
    % Open-loop bypass
    % ------------------------------------------------------------------
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = [1; 1; 1; 0];
        if nargout >= 3
            diag = empty_diag();
            diag.f_d = f_d;
        end
        return;
    end

    % ------------------------------------------------------------------
    % Persistent state declarations
    % ------------------------------------------------------------------

    % EKF state per axis: 7x3 matrix (column i = axis i)
    persistent x_e_per_axis

    % Covariance per axis: cell{3} of 7x7 (cleaner than 7x7x3)
    persistent P_per_axis

    % IIR states (per axis, 3x1 vectors)
    persistent dx_bar_m       % LP mean of delta_x_m         [3x1, um]
    persistent sigma2_dxr_hat % EWMA variance of dx_r        [3x1, um^2]

    % Trajectory delay buffers — to access pd[k-1], pd[k-2]
    persistent pd_km1 pd_km2

    % Past control buffers — for Σ_{i=1..d} f_d[k-i]  (Phase 1 §4.2)
    persistent f_d_km1 f_d_km2

    % Warmup step counter (Phase 8 §A: 2-step, f_d=0, IIR runs, EKF skip)
    persistent warmup_count

    % Step counter (1-based, increments after first init call)
    persistent k_step

    % Cached scalars / vectors (extracted once at init)
    persistent initialized
    persistent lambda_c d_delay Ts kBT R_radius gamma_N_p
    persistent a_pd a_var a_cov sigma2_w_fD sigma2_w_fA sigma2_w_a_direct
    persistent C_dpmr C_n IF_var IF_eff IF_eff_per_axis R22_prefactor xi_per_axis delay_R2_factor
    persistent C_dpmr_eff_per_axis C_np_eff_per_axis  % Stage 11 Option I: per-axis
    persistent t_warmup_kf h_bar_safe
    persistent sigma2_n_s          % 3x1 [um^2]
    persistent h_dot_max h_ddot_max
    persistent control_law_ch4 lambda_f_p F_state_ch4   % Meng Ch4 replication arm
    persistent ch4_fdet f_det_km1                       % ch4 exogenous-regressor fix (ledger 19)
    persistent ch4_stale_ff x34_prev                    % stale-consumption known input (ledger 25)
    persistent y2_whiten a_xm_prev                      % y2 AR(1) whitening (ledger 29; family knob, expgain SSOT)
    persistent y2_ma2 ma2_th ma2_s ma2_w1 ma2_w2        % u-serial-correlation recovery (ledger 40; MA(2), rho measured ledger 39)
    persistent lf_schedule lf_sched_scale               % height-scheduled forgetting (ledger 31; cube-root law)
    persistent lf_f2_avg lf_alpha_cyc                   % command-period-averaged f_det^2 (ledger 32)
    persistent enable_wall          % logical, fall back to flat (h_bar=inf) if false
    persistent w_hat_n pz_wall      % wall geometry
    persistent R_OFF                % large-R fallback for guarded y_2
    persistent a_x_init             % 3x1 wall-aware initial a_x [um/pN]
    persistent a_nom_per_axis       % 3x1 nominal a_x per axis (used in Q55) [um/pN]

    % ------------------------------------------------------------------
    % [0] Initialization on first call
    % ------------------------------------------------------------------
    if isempty(initialized)
        initialized = true;

        % --- 0A. Pull constants from params (matches existing 7-state) ---
        Ts        = params.ctrl.Ts;
        kBT       = params.ctrl.k_B * params.ctrl.T;     % [pN*um]
        R_radius  = params.common.R;                      % particle radius [um]
        gamma_N   = params.ctrl.gamma;                    % [pN*sec/um]
        gamma_N_p = gamma_N;                              % persistent copy for OL Q66 (run-time)
        sigma2_n_s = params.ctrl.sigma2_noise;            % 3x1 [um^2]

        % --- 0B. Pull constants from ctrl_const (offline scalar bundle) ---
        lambda_c        = ctrl_const.lambda_c;
        d_delay         = ctrl_const.d;                   % d-step delay (typically 2)
        C_dpmr          = ctrl_const.C_dpmr;
        C_n             = ctrl_const.C_n;
        IF_var          = ctrl_const.IF_var;
        % Phase 9 fix: s-weighted IF_eff(s) and R22_prefactor=2·a_cov/(2-a_cov)
        % replace small-α approximation `a_cov · IF_var`. Fall back if missing.
        if isfield(ctrl_const, 'IF_eff') && ~isempty(ctrl_const.IF_eff)
            IF_eff = ctrl_const.IF_eff;
        else
            IF_eff = IF_var;        % legacy: equivalent to s→1 limit
        end
        % X2a: per-axis empirical IF_eff calibration (Phase 9 Stage I).
        % If ctrl_const.IF_eff_per_axis is present (3x1), use it; else
        % fall back to the scalar IF_eff replicated 3x for backward compat.
        if isfield(ctrl_const, 'IF_eff_per_axis') && ...
                ~isempty(ctrl_const.IF_eff_per_axis)
            IF_eff_per_axis = ctrl_const.IF_eff_per_axis(:);
        else
            IF_eff_per_axis = IF_eff * ones(3, 1);
        end
        if isfield(ctrl_const, 'R22_prefactor') && ~isempty(ctrl_const.R22_prefactor)
            R22_prefactor = ctrl_const.R22_prefactor;
        else
            R22_prefactor = ctrl_const.a_cov;  % legacy: small-α limit
        end
        % Stage 11 Option I: per-axis effective C_dpmr / C_n (3x1 each).
        % Used in a_xm formula instead of paper closed-form scalars.
        % Falls back to scalar C_dpmr/C_n replicated if not present.
        if isfield(ctrl_const, 'C_dpmr_eff') && ~isempty(ctrl_const.C_dpmr_eff)
            C_dpmr_eff_per_axis = ctrl_const.C_dpmr_eff(:);
        else
            C_dpmr_eff_per_axis = C_dpmr * ones(3, 1);
        end
        if isfield(ctrl_const, 'C_np_eff') && ~isempty(ctrl_const.C_np_eff)
            C_np_eff_per_axis = ctrl_const.C_np_eff(:);
        else
            C_np_eff_per_axis = C_n * ones(3, 1);
        end
        xi_per_axis     = ctrl_const.xi_per_axis;         % 3x1 [um/pN]
        delay_R2_factor = ctrl_const.delay_R2_factor;     % = 5 for d=2
        t_warmup_kf     = ctrl_const.t_warmup_kf;
        h_bar_safe      = ctrl_const.h_bar_safe;
        a_cov           = ctrl_const.a_cov;
        % --- Meng Ch4 replication flags (spec + provenance:
        %     reference/eq17_analysis/meng_ch4_spec_ledger.md §4-§7) ---
        %   ctrl_const.control_law = 'ch4': thesis (4.4) law, and a two-matrix
        %     predict — state F = thesis (4.10) (row 3 = lambda_c, no input:
        %     the exact conditional mean, error terms are zero-mean), error
        %     covariance F = thesis (4.11) with f_d[k-1] (the force that acted
        %     over the k-1 -> k interval this predict bridges).
        %   ctrl_const.lambda_f: thesis (4.15) P <- P/lambda_f after update.
        %     1 (default) = journal (20) form (no forgetting).
        %   ctrl_const.Pf_init_lambda_f: init-only lambda_f for the Pf DARE
        %     (defaults to lambda_f). Needed because with Q66=Q77=0 and
        %     lambda_f=1 the DARE fixed point has P(6:7,6:7)=0 — a dead gain
        %     estimator; the paper never states its P[0].
        control_law_ch4 = isfield(ctrl_const, 'control_law') && ...
                          strcmpi(ctrl_const.control_law, 'ch4');
        if isfield(ctrl_const, 'lambda_f') && ~isempty(ctrl_const.lambda_f)
            lambda_f_p = ctrl_const.lambda_f(:);
            if isscalar(lambda_f_p); lambda_f_p = lambda_f_p * ones(3, 1); end
            if numel(lambda_f_p) ~= 3 || any(~isfinite(lambda_f_p)) ...
                    || any(lambda_f_p <= 0) || any(lambda_f_p > 1)
                error('motion_control_law_eq17_core:invalidLambdaF', ...
                      'ctrl_const.lambda_f must be scalar or 3x1 in (0,1].');
            end
        else
            lambda_f_p = ones(3, 1);
        end
        % ch4_fdet (ledger 19): use the FEEDFORWARD deterministic force
        % f_det = (x_d[k+1]-x_d[k])/a_hat in F_err(3,6) instead of the
        % realized f_d, which contains the estimate-feedback terms and is
        % noise-correlated with the innovation (the Ljung/endogeneity bias;
        % same disease and same fdet cure as eq17-4state 06-18 and formB
        % 08-01). Default off.
        ch4_fdet = isfield(ctrl_const, 'ch4_fdet') && ctrl_const.ch4_fdet;
        f_det_km1 = zeros(3, 1);
        % ch4_stale_ff (ledger 25): the (4.4) law consumes the PREVIOUS
        % posterior, so the true slot-3 dynamics carry the known input
        %   u3[k] = (1-lc)*(x3[k]-x3[k-1]) - (x4[k]-x4[k-1])
        % built purely from the filter's own history. Adding it to the
        % slot-3 predict removes a self-inflicted innovation component
        % (measured 2.5x of believed S1) and makes the covariance model
        % match the true error dynamics. Zero free numbers. Default off.
        ch4_stale_ff = isfield(ctrl_const, 'ch4_stale_ff') && ctrl_const.ch4_stale_ff;
        x34_prev = zeros(2, 3);
        % y2_whiten (ledger 29): a_xm is an EXACT AR(1) filter of the
        % single-sample gain readout u[k] with pole 1-a_cov, so feeding it
        % every step over-states its information by (2-a_cov)/a_cov (= 39 at
        % a_cov = 0.05) and its chi-square low-frequency fluctuation drives
        % the slow a_hat wander (ledger 28 arm-Y conviction). Whitened form
        % (family SSOT = motion_control_law_5state_expgain.m):
        %   y2' = a_xm[k] - (1-a_cov)*a_xm[k-1] = a_cov*u[k]  (white),
        %   H2' = a_cov*H2,  R22' = a_cov*(2-a_cov)*R22.
        % Zero new constants. Default off (bit-identical legacy).
        y2_whiten = isfield(ctrl_const, 'y2_whiten') && ctrl_const.y2_whiten;
        % y2_ma2 (ledger 40): the whitened u-samples are MA(2)-correlated
        % (rho = [0.60 0.17], zero beyond lag 2 -- the d = 2 loop entangles
        % adjacent increments; measured ledger 39). Recover the booked-away
        % 2.7x by subtracting the two lagged noise ESTIMATES (residual-
        % feedback MA inverter) and booking only the white innovation
        % variance sigma_w^2 = (R_booked/IF)/(1+th1^2+th2^2). Default off.
        y2_ma2 = isfield(ctrl_const, 'y2_ma2') && ctrl_const.y2_ma2;
        if isfield(ctrl_const, 'y2_ma2_rho') && ~isempty(ctrl_const.y2_ma2_rho)
            rho_u = ctrl_const.y2_ma2_rho(:);
        else
            rho_u = [0.596; 0.176];        % pooled measurement (ledger 39)
        end
        th1 = 0.8; th2 = 0.3;              % MA(2) coefficients from rho (fixed point)
        for it_ma = 1:50
            s_ma = 1 + th1^2 + th2^2;
            th2 = rho_u(2) * s_ma;
            th1 = rho_u(1) * s_ma / (1 + th2);
        end
        ma2_th = [th1; th2];
        ma2_s  = 1 + th1^2 + th2^2;
        ma2_w1 = zeros(3, 1);  ma2_w2 = zeros(3, 1);
        % lf_schedule (ledger 31): replace the constant forgetting factor by
        % the tracking-optimal memory (Ljung cube-root law)
        %   T*(t) = (2 * J_ln_rate * r_drift^2)^(-1/3),  lambda_f = 1 - Ts/T*,
        % with r_drift = |d ln a/dt| from the COMMANDED height (published
        % asymptotic log-slope 1/(hb*(hb-1)), parallel axes scaled by the
        % reflection ratio (9/16)/(9/8) = 1/2) and J_ln_rate the filter's own
        % booked relative Fisher rate. Zero free numbers; clamps are wide
        % safety rails only. Default off (bit-identical).
        lf_schedule = isfield(ctrl_const, 'lf_schedule') && ctrl_const.lf_schedule;
        % ledger 32 refinements: (i) J uses f_det^2 averaged over the KNOWN
        % command period (Fisher of the memory window, not the instant --
        % instantaneous J made lambda_f breathe at 2x the command frequency
        % and cut memory at every force peak); (ii) parallel axes use the
        % Goldman log-slope composite instead of the 1/2 reflection bound
        % (curvature test showed the bound overstates parallel drift ~2x);
        % (iii) T* is floored at the small-error validity of the cube-root
        % law: sigma_rel <= 10% (measured V(gamma) curvature keeps the
        % variance inflation J(sigma) <= 1.1 up to sigma ~ 0.10, 08-15 v3).
        if isfield(params, 'traj') && isfield(params.traj, 'frequency') ...
                && params.traj.frequency > 0
            lf_alpha_cyc = min(Ts * params.traj.frequency, 1);
        else
            lf_alpha_cyc = 1;               % no periodic command: instantaneous
        end
        lf_f2_avg = zeros(3, 1);
        if isfield(ctrl_const, 'lf_sched_scale') && ~isempty(ctrl_const.lf_sched_scale)
            lf_sched_scale = ctrl_const.lf_sched_scale(:);  % T* scale (scalar or 3x1 per-axis)
            if isscalar(lf_sched_scale); lf_sched_scale = lf_sched_scale * ones(3, 1); end
        else
            lf_sched_scale = ones(3, 1);
        end
        % Thesis (4.10) state-transition (constant; shared by all axes)
        F_state_ch4 = [0 1 0        0 0 0 0; ...
                       0 0 1        0 0 0 0; ...
                       0 0 lambda_c 0 0 0 0; ...
                       0 0 0        1 1 0 0; ...
                       0 0 0        0 1 0 0; ...
                       0 0 0        0 0 1 1; ...
                       0 0 0        0 0 0 1];
        % Wave 2D §5.6 fix: a_pd (LP for δp_md mean) is now separate from a_cov
        % (EWMA for σ²_δxr). If ctrl_const.a_pd is missing (legacy callers),
        % fall back to a_cov for backward compatibility.
        if isfield(ctrl_const, 'a_pd') && ~isempty(ctrl_const.a_pd)
            a_pd = ctrl_const.a_pd;
        else
            a_pd = a_cov;
        end
        a_var = a_pd;                                       % alias used in IIR LP update
        if isfield(ctrl_const, 'sigma2_w_fD') && ~isempty(ctrl_const.sigma2_w_fD)
            sigma2_w_fD = ctrl_const.sigma2_w_fD;          % Phase 5 §5.4 baseline 0
        else
            sigma2_w_fD = 0;
        end
        if isfield(ctrl_const, 'sigma2_w_fA') && ~isempty(ctrl_const.sigma2_w_fA)
            sigma2_w_fA = ctrl_const.sigma2_w_fA;          % Phase 5 §5.5 baseline 0
        else
            sigma2_w_fA = 0;
        end
        % Q66 direct random-walk variance (alternative to Q77 floor mechanism).
        % If > 0: Q66 = a_nom_axis^2 * sigma2_w_a_direct, modeling a_x as
        % direct random walk (Type 2 fudge analogous to sigma2_w_fA but on slot 6).
        % Default 0 (preserves Phase 5 §6.8 Q66 = 0 baseline).
        if isfield(ctrl_const, 'sigma2_w_a_direct') && ~isempty(ctrl_const.sigma2_w_a_direct)
            sigma2_w_a_direct = ctrl_const.sigma2_w_a_direct;
        else
            sigma2_w_a_direct = 0;
        end

        % --- 0C. Wall geometry ---
        if isfield(params, 'wall')
            w_hat_n  = params.wall.w_hat;
            pz_wall  = params.wall.pz;
            enable_wall = params.wall.enable_wall_effect > 0.5;
        else
            w_hat_n  = [0; 0; 1];
            pz_wall  = 0;
            enable_wall = false;
        end

        % --- 0D. Trajectory derivative bounds for Q77 (sinusoidal) ---
        %   h_dot_max  = A * (2 pi f)
        %   h_ddot_max = A * (2 pi f)^2
        amp_um = params.traj.amplitude;     % [um]
        freq_hz = params.traj.frequency;    % [Hz]
        omega = 2 * pi * freq_hz;
        h_dot_max  = amp_um * omega;        % [um/s]
        h_ddot_max = amp_um * omega^2;      % [um/s^2]

        % Optional overrides (Wave 4 motion ramp): when amplitude=0 but the
        % trajectory still has nonzero h_dot (e.g. ramp_descent), the
        % sinusoidal-derived h_dot_max / h_ddot_max are wrong. Allow caller
        % to inject scenario-specific bounds via build_eq17_constants.
        if isfield(ctrl_const, 'h_dot_max_override') && ~isempty(ctrl_const.h_dot_max_override)
            h_dot_max = ctrl_const.h_dot_max_override;
        end
        if isfield(ctrl_const, 'h_ddot_max_override') && ~isempty(ctrl_const.h_ddot_max_override)
            h_ddot_max = ctrl_const.h_ddot_max_override;
        end

        % --- 0E. Wall-aware â_x[0] init (Phase 8 §G, sigma-ratio-filter pattern) ---
        a_nom = Ts / gamma_N;               % [um/pN] free-space nominal motion gain
        if enable_wall && isfield(params, 'common') && isfield(params.common, 'p0')
            p0_init = params.common.p0(:);
            h_init_um  = dot(p0_init, w_hat_n) - pz_wall;          % [um]
            h_bar_init = max(h_init_um / R_radius, 1.001);          % clamp to avoid c blowup
            [c_para_init, c_perp_init] = calc_correction_functions(h_bar_init);
            % Per-axis: x,y use c_para; z uses c_perp (under standard w_hat = [0;0;1]).
            % For arbitrary w_hat, this still maps tangential-tangential-normal axes.
            a_x_init = [a_nom / c_para_init; ...
                        a_nom / c_para_init; ...
                        a_nom / c_perp_init];
        else
            a_x_init = [a_nom; a_nom; a_nom];
        end
        a_nom_per_axis = a_x_init;           % nominal Category-B a per axis (for Q55)
        a_xm_prev = a_x_init;                % y2_whiten chain seed: E[a_xm] at init (ledger 29)

        % --- 0F. Initialize EKF state ---
        x_e_per_axis = zeros(7, 3);
        x_e_per_axis(6, :) = a_x_init.';    % seed a_x per axis (wall-aware)

        % --- 0G. Pf_init per-axis (Riccati P_inf at h_init positioning) ---
        % Replaces Phase 8 §7 ad-hoc diag init with the steady-state DARE
        % solution at the known operating point (h_init, a_x_init,
        % positioning conditions f_d=0, h_dot=0, h_ddot=0).
        %
        % Symmetric counterpart to iir_warmup_mode='prefill': IIR state seeded
        % to its steady-state value, and Pf seeded to the KF's own steady-state
        % posterior covariance. Together: estimator starts at its equilibrium,
        % no transient.
        %
        % Per axis, solve P_post_inf such that:
        %   P_pred = F_e_ss * P_post * F_e_ss' + Q_ss
        %   K      = P_pred * H' * (H*P_pred*H' + R_ss)^{-1}
        %   P_post = (I - K*H) * P_pred
        % with F_e_ss = build_F_e(lambda_c, d, f_d=0) (Eq.19 form, positioning).
        % Q_ss / R_ss reflect h_init steady-state values:
        %   Q33_ss = 4*kBT*a_x_init(ax)
        %   Q55_ss = a_nom^2 * sigma2_w_fD               (default 0)
        %   Q66_ss = a_nom^2 * sigma2_w_a_direct         (Q66 floor if enabled)
        %   Q77_ss = a_nom^2 * sigma2_w_fA               (positioning floor)
        %   R11_ss = sigma2_n_s(ax)
        %   R22_ss = R22_prefactor * IF_eff_per_axis(ax) * (a_x_init + xi)^2
        %
        % NOTE: output var `diag` shadows MATLAB's diag() builtin, so we
        % construct diagonal matrices via assignment.
        % Pf_init Riccati steady state uses positioning limit:
        %   f_d = 0, F_1 = sum f_d_past = 0, F_2 = sum i*f_d_past = 0.
        % Captures the limiting structural F_e form with no control-history
        % accumulation (positioning Δp_d = 0 + KF-unbiased estimates).
        if control_law_ch4
            % ch4 arm: DARE under the thesis-(4.11) error F at f_d = 0, with
            % the lambda_f-sustained recursion so init = the filter's own
            % equilibrium (same rationale as the eq19 branch).
            F_e_ss = F_state_ch4;
            F_e_ss(3, :) = [0 0 1 -1 0 0 0];
        else
            F_e_ss = build_F_e(lambda_c, d_delay, 0, 0, 0, false);
        end
        if isfield(ctrl_const, 'Pf_init_lambda_f') ...
                && ~isempty(ctrl_const.Pf_init_lambda_f)
            lambda_f_init = ctrl_const.Pf_init_lambda_f(:);
            if isscalar(lambda_f_init); lambda_f_init = lambda_f_init * ones(3, 1); end
        else
            lambda_f_init = lambda_f_p;
        end
        H_ss   = [1 0 0 0 0 0 0; 0 0 0 0 0 1 -d_delay];

        P_per_axis = cell(3, 1);
        for ax = 1:3
            a_init_ax = a_x_init(ax);
            Q_ss_ax = zeros(7);
            Q_ss_ax(3, 3) = 4 * kBT * a_init_ax;
            Q_ss_ax(5, 5) = a_nom_per_axis(ax)^2 * sigma2_w_fD;
            Q_ss_ax(6, 6) = a_nom_per_axis(ax)^2 * sigma2_w_a_direct;
            Q_ss_ax(7, 7) = a_nom_per_axis(ax)^2 * sigma2_w_fA;
            R22_ss_ax = R22_prefactor * IF_eff_per_axis(ax) ...
                        * (a_init_ax + xi_per_axis(ax))^2;
            R_ss_ax = [sigma2_n_s(ax), 0; 0, R22_ss_ax];
            P_per_axis{ax} = solve_dare_kf_local(F_e_ss, H_ss, ...
                                                  Q_ss_ax, R_ss_ax, ...
                                                  lambda_f_init(ax));
        end

        % --- 0H. IIR states (mode-dependent) ---
        % iir_warmup_mode (default 'legacy' for backward compat):
        %   'legacy' : dx_bar_m=0, sigma2_dxr_hat=0 (Phase 8 §A original)
        %   'prefill': sigma2_dxr_hat seeded to per-axis steady-state at known
        %              h_init using inverse a_xm formula (design.md §9.4):
        %                sigma2_dxr_ss = 4*kBT*a_x_init.*C_dpmr_eff
        %                              + C_np_eff.*sigma2_n_s
        %              Requires fixed initial h; valid for positioning and
        %              motion (motion lag is <= 1 IIR time constant ≈ 1/a_cov).
        iir_warmup_mode = 'legacy';
        if isfield(ctrl_const, 'iir_warmup_mode') && ~isempty(ctrl_const.iir_warmup_mode)
            iir_warmup_mode = ctrl_const.iir_warmup_mode;
        end

        dx_bar_m = zeros(3, 1);
        if strcmpi(iir_warmup_mode, 'prefill')
            sigma2_dxr_hat = 4 * kBT * a_x_init .* C_dpmr_eff_per_axis ...
                           + C_np_eff_per_axis .* sigma2_n_s;
        else
            sigma2_dxr_hat = zeros(3, 1);
        end

        % --- 0I. Trajectory + control delay buffers ---
        pd_km1  = pd;
        pd_km2  = pd;
        f_d_km1 = zeros(3, 1);
        f_d_km2 = zeros(3, 1);

        % --- 0J. Warmup counter ---
        % legacy:  warmup_count = 2 (Phase 8 §A — controller emits f_d=0 for
        %          first 3 calls total, IIR accumulates real measurements)
        % prefill: warmup_count = 0 (sigma2_dxr_hat already at steady state;
        %          first call still returns 0 for structural delay-buffer
        %          reasons, but call 2 onward emits real control)
        if strcmpi(iir_warmup_mode, 'prefill')
            warmup_count = 0;
        else
            warmup_count = 2;
        end

        % --- 0K. Misc ---
        k_step = 1;                          % first user call counts as k=1
        R_OFF  = 1e10;                       % gate-off variance (Phase 6 §5)

        % --- 0L. First call returns zeros (no f_d yet) ---
        f_d = zeros(3, 1);
        ekf_out = [a_x_init(1); a_x_init(3); a_x_init(2); 0];
        if nargout >= 3
            diag = empty_diag();
            diag.f_d = f_d;
            % Populate diag with init persistent state so log step 1 matches
            % what the EKF actually starts with (avoids spurious zero-spike at
            % t=0 in plots, especially under iir_warmup_mode='prefill').
            diag.sigma2_dxr_hat = sigma2_dxr_hat;
            den_axm_init = C_dpmr_eff_per_axis * 4 * kBT;
            diag.a_xm  = (sigma2_dxr_hat - C_np_eff_per_axis .* sigma2_n_s) ...
                         ./ den_axm_init;
            diag.a_hat = a_x_init;
            P77_init_v = zeros(3, 1);
            P_a_v      = zeros(3, 1);
            P_dx_v     = zeros(3, 1);
            for ax_init = 1:3
                P77_init_v(ax_init) = P_per_axis{ax_init}(7, 7);
                P_a_v(ax_init)      = P_per_axis{ax_init}(6, 6);
                P_dx_v(ax_init)     = P_per_axis{ax_init}(3, 3);
            end
            diag.P77 = P77_init_v;
            diag.P_a = P_a_v;
            diag.P_dx = P_dx_v;
        end
        return;
    end

    % ------------------------------------------------------------------
    % Per-step variables — extract per-axis state (k step starts here)
    % ------------------------------------------------------------------
    a_hat = x_e_per_axis(6, :)';   % 3x1 [um/pN]
    xD_hat = x_e_per_axis(4, :)';  % 3x1 [um]
    % Optional: suppress x_D_hat in Eq.17 control law (testing — diagnose
    % whether x_D state is acting as thermal compensator)
    if isfield(ctrl_const, 'suppress_xD') && ctrl_const.suppress_xD
        xD_hat_for_ctrl = zeros(3, 1);
    else
        xD_hat_for_ctrl = xD_hat;
    end

    % delta_x_m[k] = p_d[k-d] - p_m[k]   (per axis)
    %   For d=2: pd_km2 already holds p_d[k-2] from previous step's buffer shift.
    if d_delay == 2
        pd_km_d = pd_km2;
    elseif d_delay == 1
        pd_km_d = pd_km1;
    else
        % Generic d would require a longer delay buffer; not supported here.
        error('motion_control_law_eq17_core:unsupportedDelay', ...
              'Only d=1 or d=2 supported, got d=%g.', d_delay);
    end
    delta_x_m = pd_km_d - p_m;        % 3x1 [um]

    % p_d[k+1] from increment
    pd_kp1 = pd + del_pd;             % 3x1 [um]

    % h_bar from current p_m  (for Q77 wall coupling and Guard 3)
    if enable_wall
        h_um = dot(p_m, w_hat_n) - pz_wall;  % [um]
        h_bar = h_um / R_radius;
    else
        h_bar = Inf;                          % flat / isotropic Stokes
    end

    one_minus_lc = 1 - lambda_c;

    % ------------------------------------------------------------------
    % [1] IIR a_xm  (paper 2025 Eq.9-13)  — per axis
    %   Always run during warmup (Phase 8 §A: IIR LP / EWMA stay alive)
    %
    %   dx_bar_m[k+1] = (1 - a_var) * dx_bar_m[k] + a_var * delta_x_m[k]
    %   dx_r[k]       = delta_x_m[k] - dx_bar_m[k]              (centered)
    %   sigma2_dxr_hat[k+1] = (1 - a_cov) * sigma2_dxr_hat[k]
    %                       + a_cov * (dx_r^2[k] - dx_bar_r_sq[k])
    %     (steady-state dx_bar_r ≈ 0; we keep dx_r^2 directly since
    %      dx_r is already centered, matching design_v2.md §3.1.)
    %
    %   a_xm[k] = (sigma2_dxr_hat[k] - C_n * sigma2_n_s) / (C_dpmr * 4 kBT)
    % ------------------------------------------------------------------
    % Update LP mean using current measurement (post-update form)
    dx_bar_m_new = (1 - a_var) * dx_bar_m + a_var * delta_x_m;
    dx_r = delta_x_m - dx_bar_m_new;                          % centered residual

    % Variance EWMA — δ̄x_r ~ 0 in steady state (~2.5% bias OK)
    sigma2_dxr_hat_new = (1 - a_cov) * sigma2_dxr_hat + a_cov * dx_r.^2;

    % Stage 11 Option I: per-axis effective C_dpmr_eff / C_np_eff
    % (replaces paper closed-form scalars 3.96 / 1.18 with v2-actual values)
    den_axm_per_axis = C_dpmr_eff_per_axis * 4 * kBT;          % 3x1 [pN*um]
    a_xm = (sigma2_dxr_hat_new - C_np_eff_per_axis .* sigma2_n_s) ./ den_axm_per_axis;  % 3x1 [um/pN]

    % v3 Jensen correction (Meng Ch4 ledger §12, diagnostic flag): the loop
    % variance inflates as E[V(a/â)] > V(1) when â fluctuates; divide the
    % readout by J(σ_ε), σ_ε = sqrt(P66)/â from the filter's own posterior.
    % ctrl_const.jensen_sigma_grid / jensen_J from calc_cdpmr_ch4 (derived,
    % no free numbers). Default absent = off.
    if isfield(ctrl_const, 'jensen_J') && ~isempty(ctrl_const.jensen_J) ...
            && ~isempty(P_per_axis)
        for jax = 1:3
            sig_eps = sqrt(max(P_per_axis{jax}(6, 6), 0)) ...
                      / max(x_e_per_axis(6, jax), 1e-12);
            Jf = interp1(ctrl_const.jensen_sigma_grid, ctrl_const.jensen_J, ...
                         min(sig_eps, ctrl_const.jensen_sigma_grid(end)), 'linear');
            a_xm(jax) = a_xm(jax) / Jf;
        end
    end

    % ------------------------------------------------------------------
    % [2] Warmup gate (Phase 8 §A: 2-step; f_d=0; EKF skipped; IIR active)
    % ------------------------------------------------------------------
    in_warmup = (warmup_count > 0);

    if in_warmup
        % --- During warmup: NO control output, NO EKF update ---
        f_d = zeros(3, 1);

        % If this is the LAST warmup step, seed EKF δp_hat states (slots 1,2,3)
        % from current IIR LP mean estimate (dx_bar_m_new). At the moment of
        % warmup-end, dx_bar_m_new already holds the converged LP mean.
        if warmup_count == 1
            for ax = 1:3
                x_e_per_axis(1, ax) = dx_bar_m_new(ax);    % δp_1 = δx[k-2] seed
                x_e_per_axis(2, ax) = dx_bar_m_new(ax);    % δp_2 = δx[k-1] seed
                x_e_per_axis(3, ax) = dx_bar_m_new(ax);    % δp_3 = δx[k] seed
            end
        end

        % Update IIR persistent state
        dx_bar_m       = dx_bar_m_new;
        sigma2_dxr_hat = sigma2_dxr_hat_new;

        % Shift trajectory delay buffers (controller sees pd[k] this call)
        pd_km2 = pd_km1;
        pd_km1 = pd;

        % Shift past-f_d buffers (f_d=0 during warmup, so just propagate zeros)
        f_d_km2 = f_d_km1;
        f_d_km1 = f_d;

        % Decrement warmup
        warmup_count = warmup_count - 1;
        k_step = k_step + 1;

        % Output
        a_hat_post = x_e_per_axis(6, :)';
        if enable_wall
            h_bar_now = h_bar;
        else
            h_bar_now = 0;
        end
        ekf_out = [a_hat_post(1); a_hat_post(3); a_hat_post(2); h_bar_now];

        if nargout >= 3
            diag = empty_diag();
            diag.f_d                  = f_d;
            diag.sigma2_dxr_hat       = sigma2_dxr_hat_new;
            diag.a_xm                 = a_xm;
            diag.delta_x_m            = delta_x_m;
            diag.h_bar                = h_bar;
            diag.x_D_hat              = x_e_per_axis(4, :).';
            diag.delta_a_hat          = x_e_per_axis(7, :).';
            diag.dx_r                 = dx_r;                          % Phase 9
            diag.a_hat                = a_hat_post;                    % Phase 9 (slot 6)
            % P77 from current persistent P_per_axis (no EKF update this step)
            P77_warm = zeros(3, 1);
            for ax_w = 1:3
                P77_warm(ax_w) = P_per_axis{ax_w}(7, 7);
            end
            diag.P77                  = P77_warm;                      % Phase 9
            % Q77 not yet computed (EKF skipped during warmup) — leave zeros
            % gate flags / KF outputs left as default zeros (no EKF this step)
        end
        return;
    end

    % ------------------------------------------------------------------
    % [3] Eq.17 control law (per axis)  — Phase 1 §4.2 paper form
    %
    %   f_d[k] = (1/â_x[k]) · { x_d[k+1] − λ_c·x_d[k] − (1−λ_c)·x_d[k−d]
    %                          + (1−λ_c)·δx_m[k] }
    %          − (1−λ_c) · Σ_{i=1..d} f_d[k−i]            ← OUTSIDE 1/â_x bracket
    %          − x̂_D[k] / â_x[k]                          ← additive disturbance comp.
    %
    %   For d=2: Σ = f_d_km1 + f_d_km2.
    %
    %   x̂_D placement: as separate term −x̂_D/â_x outside the kinematic bracket
    %   (Phase 0 §4.2 algebraic equivalence). â_x carries over from previous
    %   posterior; on first post-warmup step uses warmup-end posterior.
    % ------------------------------------------------------------------
    if d_delay == 2
        sum_fd_past = f_d_km1 + f_d_km2;
    elseif d_delay == 1
        sum_fd_past = f_d_km1;
    else
        sum_fd_past = zeros(3, 1);
    end

    inv_a_hat = 1 ./ a_hat;
    if isfield(params.ctrl, 'a_true_oracle') && ~isempty(params.ctrl.a_true_oracle)
        % DIAGNOSTIC ONLY (ledger 27 arm O): sever every 1/a-hat feedback in
        % the law and the fdet regressor (estimator untouched) -- convicts or
        % acquits the nonlinear self-interaction as the slow-wander driver.
        inv_a_hat = 1 ./ params.ctrl.a_true_oracle(:);
    end
    if control_law_ch4
        % Thesis (4.4) = journal (6):
        %   f_d = (1/â_x[k])·{ x_d[k+1] − x_d[k] + (1−λ_c)·δx̂[k] − x̂_D[k] }
        % Consumed estimates = the stored posterior. Correspondence: the
        % thesis (4.14) object X̂[k+1] = F·X̂[k] + K·innov(y[k]) is EXACTLY
        % what our predict+update wrote back last call — no further F
        % advance (an extra F·(·) cuts the position feedback to λ_c·δx̂₃,
        % measured as open-loop thermal wander at λ_c ≈ 0).
        % Closed-loop spectrum of THIS faithful pairing (10-state companion,
        % ledger §8): unstable for λ_c ≲ 0.35 (1.173/step at λ_c = 0),
        % stable at the journal's λ_c = 0.4 — the thesis' own λ = 0
        % simulation claim is not reproducible under d = 2 as written.
        dx3_hat = x_e_per_axis(3, :)';
        f_d = inv_a_hat .* ( ...
                    pd_kp1 ...
                  - pd ...
                  + one_minus_lc * dx3_hat ...
                  - xD_hat_for_ctrl );
        f_det_cur = inv_a_hat .* (pd_kp1 - pd);   % feedforward-only (exogenous)
    else
        f_d = inv_a_hat .* ( ...
                    pd_kp1 ...
                  - lambda_c * pd ...
                  - one_minus_lc * pd_km_d ...
                  + one_minus_lc * delta_x_m ) ...
              - one_minus_lc * sum_fd_past ...
              - xD_hat_for_ctrl .* inv_a_hat;
    end

    % ------------------------------------------------------------------
    % [4] Adaptive Q and R (per axis)  — Phase 5 / Phase 6
    % ------------------------------------------------------------------
    % Per-axis K_h, K_h' from h_bar
    %   x, y axes  ->  K_h_para, K_h_prime_para
    %   z axis     ->  K_h_perp, K_h_prime_perp
    if enable_wall && isfinite(h_bar) && h_bar > 1
        [c_para_h, c_perp_h, derivs] = calc_correction_functions(h_bar, true);
        K_h_axis      = [derivs.K_h_para; derivs.K_h_para; derivs.K_h_perp];
        K_h_pr_axis   = [derivs.K_h_prime_para; derivs.K_h_prime_para; derivs.K_h_prime_perp];
        % Optional saturation: cap |K_h| at K_h_cap when ctrl_const.K_h_cap > 0.
        % Used to prevent Q66 / F_e blow-up at h_bar -> 1 (numerical robustness).
        if isfield(ctrl_const, 'K_h_cap') && ctrl_const.K_h_cap > 0
            cap = ctrl_const.K_h_cap;
            K_h_axis    = sign(K_h_axis)    .* min(abs(K_h_axis),    cap);
            K_h_pr_axis = sign(K_h_pr_axis) .* min(abs(K_h_pr_axis), cap^2);
        end
    else
        % Far-field / wall-disabled: K_h ~ 0, K_h' ~ 0 → Q77 ~ 0
        c_para_h = 1; c_perp_h = 1;
        K_h_axis    = zeros(3, 1);
        K_h_pr_axis = zeros(3, 1);
    end

    R2_inv = R_radius^(-2);
    R4_inv = R2_inv^2;
    Ts4 = Ts^4;

    % Build Q (7x7) and R (2x2) per axis
    Q_per_axis = cell(3, 1);
    R_per_axis = cell(3, 1);
    Q77_per_axis = zeros(3, 1);     % cached for R(2,2) assembly
    for ax = 1:3
        a_hat_i = a_hat(ax);
        K_h_i   = K_h_axis(ax);
        K_h_p_i = K_h_pr_axis(ax);

        % Q33,i = 4 kBT * a_hat,i  (Path C strict, Phase 5 §4.2)
        Q33_i = 4 * kBT * a_hat_i;

        % Q55,i = a_nom_axis^2 * sigma2_w_fD  (Phase 5 §5.4 + §6.2 a_nom Cat. B)
        % Baseline σ²_w_fD = 0 → Q55 = 0.
        Q55_i = a_nom_per_axis(ax)^2 * sigma2_w_fD;

        % Q77,i = dt^4 * a_hat^2 * { (K_h^2 - K_h')^2 * h_dot_max^4 / (8 R^4)
        %                          + K_h^2          * h_ddot_max^2 / (2 R^2) }
        % (Phase 5 §6.5 Term A + Term B)
        term_A = (K_h_i^2 - K_h_p_i)^2 * h_dot_max^4 / 8 * R4_inv;
        term_B = K_h_i^2               * h_ddot_max^2 / 2 * R2_inv;
        Q77_i = Ts4 * a_hat_i^2 * (term_A + term_B);

        % Phase 5 §5.5 — σ²_w_fA random walk regularization (analogous to §5.4 σ²_w_fD on Q55)
        % Q77 floor from a_nom² · σ²_w_fA, ensures KF P77 doesn't degenerate
        Q77_phase5_floor = a_nom_per_axis(ax)^2 * sigma2_w_fA;
        Q77_i = max(Q77_i, Q77_phase5_floor);

        % Diagnostic: optional Q77_floor / Q77_floor_schedule injection.
        % Schedule form: Q77(t) = init * exp(-t/tau) + steady (struct fields).
        % If both present, schedule wins.
        Q77_floor_eff = 0;
        if isfield(ctrl_const, 'Q77_floor_schedule') && ~isempty(ctrl_const.Q77_floor_schedule)
            sched = ctrl_const.Q77_floor_schedule;
            t_now_local = (k_step - 1) * Ts;
            Q77_floor_eff = sched.init * exp(-t_now_local / sched.tau) + sched.steady;
        elseif isfield(ctrl_const, 'Q77_floor') && ~isempty(ctrl_const.Q77_floor)
            if numel(ctrl_const.Q77_floor) == 3
                Q77_floor_eff = ctrl_const.Q77_floor(ax);
            else
                Q77_floor_eff = ctrl_const.Q77_floor;
            end
        end
        if Q77_floor_eff > 0
            Q77_i = max(Q77_i, Q77_floor_eff);
        end

        % Optional hard-zero override for testing (Wave 4 motion ramp)
        if isfield(ctrl_const, 'force_Q77_zero') && ctrl_const.force_Q77_zero
            Q77_i = 0;
        end
        Q77_per_axis(ax) = Q77_i;

        % Q66 mode selection. Three modes (mutually exclusive in priority):
        %   OL:       Q66 = (a·K_h/R)^2 · σ²_δh_thermal           (Layer 1 physical)
        %             σ²_δh_thermal = 4·kBT·Ts/(γ_N·c_perp(h̄))    (paper Eq.11 convention, matches Q33 = 4·kBT·a)
        %             Pure thermal-motion-driven a noise; no control-loop coupling.
        %             + optional sigma2_w_a_direct floor.
        %   physical: Q66 = (a·K_h/R)^2 · (σ²_δxr_hat(z) - σ²_n_proj)   (Phase 5 §6.9 NEW)
        %             tracking-residual driven; circular w/ control loop.
        %             + optional sigma2_w_a_direct floor.
        %   legacy:   Q66 = a_nom^2 · sigma2_w_a_direct (pure engineering margin)
        if isfield(ctrl_const, 'Q66_OL_mode') && ctrl_const.Q66_OL_mode
            % Wall-normal Brownian increment per Ts (paper Eq.11 convention,
            % sigma^2_dx_T = 4*k_B T * a, matches Q33 = 4*k_B T * a_x scaling)
            sigma2_dh_thermal = 4 * kBT * Ts / (gamma_N_p * c_perp_h);
            % δa = -a·K_h/R · δh  (chain rule on a = a_nom/c, K_h = (1/c)·dc/dh̄)
            Q66_OL_i = (a_hat_i * K_h_i / R_radius)^2 * sigma2_dh_thermal;
            % Optional fixed floor on top (covers safe region where OL ≪ Q77 budget)
            Q66_i = Q66_OL_i + a_nom_per_axis(ax)^2 * sigma2_w_a_direct;
        elseif isfield(ctrl_const, 'Q66_physical_mode') && ctrl_const.Q66_physical_mode
            % Compute sigma2_n_proj (wall-normal projection of measurement noise)
            % Avoid diag() builtin (shadowed by output var). Use elementwise sum.
            sigma2_n_proj = sum((w_hat_n .^ 2) .* sigma2_n_s);
            % Use z-axis (= wall-normal for default w_hat=[0;0;1]) sigma2_dxr estimate
            sigma2_delta_xr_real = max(sigma2_dxr_hat_new(3) - sigma2_n_proj, 0);
            Q66_i = (a_hat_i * K_h_i / R_radius)^2 * sigma2_delta_xr_real;
            % Optionally add engineering margin on top
            Q66_i = Q66_i + a_nom_per_axis(ax)^2 * sigma2_w_a_direct;
        else
            % Legacy: pure engineering margin only
            Q66_i = a_nom_per_axis(ax)^2 * sigma2_w_a_direct;
        end

        Q_i = zeros(7);
        Q_i(3, 3) = Q33_i;
        Q_i(5, 5) = Q55_i;
        Q_i(6, 6) = Q66_i;
        Q_i(7, 7) = Q77_i;

        % Optional Q(3,6)=Q(6,3) cross term. Physics:
        %   w_3_ax ∝ F_th projected on axis ê_ax       (Q33 source)
        %   w_6_ax ∝ -(a·K_h/R) · F_th projected on ŵ  (Q66 source via δh chain)
        %   Cov(w_3, w_6) = (ê_ax · ŵ) · (-K_h_ax sign) · sqrt(Q33·Q66)
        % For default w_hat=[0;0;1]: only z-axis has non-zero contribution.
        if isfield(ctrl_const, 'Q36_cross_term') && ctrl_const.Q36_cross_term
            weight = w_hat_n(ax);
            if abs(weight) > 1e-12 && Q33_i > 0 && Q66_i > 0
                rho_signed = -sign(K_h_axis(ax)) * weight;
                Q36_val = rho_signed * sqrt(Q33_i * Q66_i);
                Q_i(3, 6) = Q36_val;
                Q_i(6, 3) = Q36_val;
            end
        end

        Q_per_axis{ax} = Q_i;

        % R(1,1) = sigma2_n_s,i  (Phase 6 §3)
        R11_i = sigma2_n_s(ax);

        % R(2,2) intrinsic = R22_prefactor * IF_eff_per_axis(ax) * (a_hat + xi)^2
        % Phase 9 fix: corrected from small-α approximation `a_cov · IF_var`
        % to finite-α exact form `(2·a_cov/(2-a_cov)) · IF_eff(1-a_cov)`.
        % See design.md:880-897 and Phase 9 Wave 1 (commit f618f37) Path C.
        % X2a: IF_eff is per-axis (calibrated from empirical ρ_δx if present)
        % to absorb the ~9% deviation between Phase 1 closed form and
        % production-conditions ρ_δx (Phase 9 Stage I diagnosis).
        % Diagnostic (Meng Ch4 ledger §11): ctrl_const.R22_a_fixed (3x1)
        % evaluates R(2,2) at a FIXED gain instead of a_hat, separating the
        % R22(a_hat) rectification route from the 1/a_hat loop route. Not a
        % production knob.
        if isfield(ctrl_const, 'R22_a_fixed') && ~isempty(ctrl_const.R22_a_fixed)
            a_for_R22 = ctrl_const.R22_a_fixed(ax);
        else
            a_for_R22 = a_hat_i;
        end
        R2_intrinsic_i = R22_prefactor * IF_eff_per_axis(ax) ...
                         * (a_for_R22 + xi_per_axis(ax))^2;
        % R(2,2) eff = intrinsic + delay_R2_factor * Q77  (Phase 6 §4.3)
        R2_eff_i = R2_intrinsic_i + delay_R2_factor * Q77_i;

        % Optional: include d_delay * Q66 term (full innovation-noise
        % variance from backward state expansion: Var(v_2) =
        % delay_R2_factor*Q77 + d*Q66 + Var(n_a)). Default false (legacy).
        if isfield(ctrl_const, 'R22_include_Q66') && ctrl_const.R22_include_Q66
            R2_eff_i = R2_eff_i + d_delay * Q66_i;
        end

        % --- 3-guard adaptive R_2 (Phase 6 §5) ----------------------
        t_now = (k_step - 1) * Ts;                  % real time at step k
        G1 = (t_now < t_warmup_kf);                 % warm-up
        G2 = ((sigma2_dxr_hat_new(ax) - C_np_eff_per_axis(ax) * sigma2_n_s(ax)) <= 0);  % low SNR
        G3 = (h_bar < h_bar_safe);                  % near-wall

        if G1 || G2 || G3
            R22_i = R_OFF;
        else
            R22_i = R2_eff_i;
        end

        % NOTE: avoid MATLAB diag() builtin (shadowed by output var `diag`)
        R_axis_mat = zeros(2);
        R_axis_mat(1, 1) = R11_i;
        R_axis_mat(2, 2) = R22_i;
        R_per_axis{ax} = R_axis_mat;
    end

    % ------------------------------------------------------------------
    % [5] EKF predict + update per axis  — Phase 1 §10
    %
    %   F_e (7x7), Eq.19 form (Phase 1 §10.4):
    %     Row 3: [0, 0, λ_c, -(1+d·(1-λ_c)), 0, -f_d[k], 0]
    %     other rows: structural (shift / RW Jordan blocks)
    %
    %   H (2x7), (2,7) = -d_delay
    % ------------------------------------------------------------------
    H_full = [1 0 0 0 0 0       0; ...
              0 0 0 0 0 1 -d_delay];
    H_y1   = H_full(1, :);   % y_1 only

    % a_hat freeze override config
    has_freeze = isfield(ctrl_const, 'a_hat_freeze') && ~isempty(ctrl_const.a_hat_freeze);
    if has_freeze
        a_hat_frz = ctrl_const.a_hat_freeze(:);   % 3x1
    end

    % Diag storage for the EKF loop (used only if nargout >= 3, but cheap)
    K_a_y2_per_axis  = zeros(3, 1);    % K_kf(6, 2) per axis (0 if y_2 gated)
    K_dx_y1_per_axis = zeros(3, 1);    % K_kf(3, 1) per axis
    innov_y1_per_axis = zeros(3, 1);   % y_1 innovation
    K_a_y1_per_axis  = zeros(3, 1);    % K_kf(6, 1) per axis
    S1_pred_per_axis = zeros(3, 1);    % believed Var(innov_y1)
    x34_used = zeros(2, 3);            % slots 3,4 consumed this call (ledger 25)
    lf_used_per_axis = ones(3, 1);     % actual forgetting applied (ledger 31 diag)
    innov_y2_per_axis = zeros(3, 1);   % y_2 innovation (0 if y_2 gated)
    gate_y2_off_per_axis = false(3, 1);
    G_per_axis = false(3, 3);          % rows: G1/G2/G3, cols: axes

    % Re-compute gate flags per axis (also computed in Q/R loop above; kept
    % consistent here). Sequential 1D updates avoid joint S-matrix
    % conditioning issues when R(2,2) = R_OFF.
    t_now = (k_step - 1) * Ts;
    % Optional debug flag — F_e Row 3 form switch (default 'eq19' = v2 spec)
    %   'eq19' (default): F_e(3,1)=0, (3,3)=λ_c, (3,4)=-(1+d·(1-λ_c))
    %   'eq18'           : F_e(3,1)=-(1-λ_c), (3,3)=1, (3,4)=-1
    use_eq18 = isfield(ctrl_const, 'F_e_form') && strcmpi(ctrl_const.F_e_form, 'eq18');
    for ax = 1:3
        % Exact F_e (Fe_H_derivation.tex) — Row 3 columns (3,5)/(3,6)/(3,7)
        % carry control-history sums:
        %   F_1[k] = sum_{i=1..d} f_d[k-i]
        %   F_2[k] = sum_{i=1..d} i * f_d[k-i]
        % For d=2: F_1 = f_d_km1 + f_d_km2, F_2 = f_d_km1 + 2*f_d_km2.
        % f_d_km1 / f_d_km2 are 3x1 persistent buffers; they are 0 on the
        % first two steps (correct initial condition: no prior control).
        if d_delay == 2
            F_1_i = f_d_km1(ax) + f_d_km2(ax);
            F_2_i = f_d_km1(ax) + 2 * f_d_km2(ax);
        elseif d_delay == 1
            F_1_i = f_d_km1(ax);
            F_2_i = f_d_km1(ax);    % i=1: 1*f_d[k-1]
        else
            F_1_i = 0;
            F_2_i = 0;
        end
        F_e = build_F_e(lambda_c, d_delay, f_d(ax), F_1_i, F_2_i, use_eq18);

        x_curr = x_e_per_axis(:, ax);
        P_curr = P_per_axis{ax};
        x34_used(:, ax) = x_curr(3:4);   % pre-update slots 3,4 (ledger 25 buffer)

        % --- Predict ---
        if control_law_ch4
            % Two-matrix predict (thesis Ch4): state by (4.10) — the exact
            % conditional mean (error terms are zero-mean, so no input) —
            % covariance by the (4.11) error F. The error transition this
            % predict bridges (k-1 -> k) was driven by f_d[k-1], i.e. the
            % f_d_km1 buffer BEFORE this step's shift (the formB-style
            % pairing; the thesis' own -f_d[k] indexing is its (4.12)
            % one-step-ahead label for the same object).
            F_err_ch4 = F_state_ch4;
            if ch4_fdet
                F_err_ch4(3, :) = [0 0 1 -1 0 -f_det_km1(ax) 0];
            else
                F_err_ch4(3, :) = [0 0 1 -1 0 -f_d_km1(ax) 0];
            end
            x_pred = F_state_ch4 * x_curr;
            if ch4_stale_ff
                x_pred(3) = x_pred(3) ...
                    + one_minus_lc * (x_curr(3) - x34_prev(1, ax)) ...
                    - (x_curr(4) - x34_prev(2, ax));
            end
            P_pred = F_err_ch4 * P_curr * F_err_ch4' + Q_per_axis{ax};
        else
            x_pred = F_e * x_curr;
            P_pred = F_e * P_curr * F_e' + Q_per_axis{ax};
        end
        P_pred = 0.5 * (P_pred + P_pred');

        % --- Optional a_hat freeze: lock state(6) and zero P row/col 6 ---
        if has_freeze
            x_pred(6)    = a_hat_frz(ax);
            P_pred(6, :) = 0;
            P_pred(:, 6) = 0;
        end

        % --- Determine if y_2 channel is gated off ---
        G1 = (t_now < t_warmup_kf);
        G2 = ((sigma2_dxr_hat_new(ax) - C_np_eff_per_axis(ax) * sigma2_n_s(ax)) <= 0);
        G3 = (h_bar < h_bar_safe);
        gate_y2_off = G1 || G2 || G3;

        % Save gate flags and y_2-off flag for diag
        G_per_axis(:, ax) = [G1; G2; G3];
        gate_y2_off_per_axis(ax) = gate_y2_off;

        if gate_y2_off
            % Skip y_2 entirely (1D update with y_1 only)
            H_use = H_y1;
            y_use = delta_x_m(ax);
            R_use = sigma2_n_s(ax);
        else
            % Full 2D update
            H_use = H_full;
            y_use = [delta_x_m(ax); a_xm(ax)];
            R_use = R_per_axis{ax};
            if y2_whiten
                % ledger 29: whitened increment consumes only the fresh
                % sample: innov2 = a_cov*(u[k] - H2*x_hat) exactly.
                H_use(2, :) = a_cov * H_use(2, :);
                y_use(2)    = a_xm(ax) - (1 - a_cov) * a_xm_prev(ax);
                R_use(2, 2) = a_cov * (2 - a_cov) * R_use(2, 2);
                if y2_ma2
                    % ledger 40: subtract the two lagged noise estimates and
                    % book the white innovation only (IF_eff penalty removed
                    % -- the correlation is now modeled, not discounted).
                    y_use(2) = y_use(2) - ma2_th(1) * ma2_w1(ax) ...
                                        - ma2_th(2) * ma2_w2(ax);
                    R_use(2, 2) = (R_use(2, 2) / IF_eff_per_axis(ax)) / ma2_s;
                end
            end
        end

        % --- Update (sequential / conditional) ---
        y_pred = H_use * x_pred;
        innov  = y_use - y_pred;

        S = H_use * P_pred * H_use' + R_use;
        S1_pred_per_axis(ax) = S(1, 1);          % y1 innovation variance the filter believes
        S = 0.5 * (S + S');                     % symmetrize
        K_kf = (P_pred * H_use') / S;           % 7x{1,2}

        % --- Stage 10 Option A: gate slot 6 + 7 update during G1 (warm-up) ---
        % Wave 4 root cause: F_e(3,6) = -f_d accumulates P_pred(3,6) cross-cov
        % during warm-up, leaking through K_kf(6,1)·y_1 to drive â_x runaway
        % (â_x can be pushed past 0 → control law blowup at step ~122).
        % Gate row 6 + 7 of K_kf during G1 only — slot 6/7 evolve via Jordan
        % integration only (a_x = const, δa_x ≈ 0). G2/G3 do not gate slot 6/7
        % (R(2,2)=R_OFF / 1D y_1-only update already disables y_2 contribution).
        if G1
            K_kf(6, :) = 0;     % gate slot 6 (a_x) measurement update
            K_kf(7, :) = 0;     % gate slot 7 (δa_x) — Jordan-pair consistency
        end

        % Diagnostic (ledger 24/25): scale the gain-block rows of K in the
        % STATE update only (P keeps the unscaled K -- the ledger-24 version
        % contaminated P and let the filter self-compensate). Copied AFTER the
        % G1 gate so the warm-up gate reaches the state update (a copy taken
        % before the gate silently disabled Guard 1 -- unit T4 caught it).
        K_state = K_kf;
        if isfield(ctrl_const, 'K_gain_scale') && ~isempty(ctrl_const.K_gain_scale)
            K_state(6:7, :) = ctrl_const.K_gain_scale * K_state(6:7, :);
        end

        x_post = x_pred + K_state * innov;
        if y2_whiten && y2_ma2 && ~gate_y2_off
            w_fresh = y_use(2) - H_use(2, :) * x_post;   % post-fit noise estimate
            ma2_w2(ax) = ma2_w1(ax);
            ma2_w1(ax) = w_fresh;
        end
        % Joseph form (numerically stable) vs Standard form (legacy).
        % Joseph: P_post = (I-KH) P_pred (I-KH)' + K R K'   — symmetric PSD by construction
        % Standard: P_post = (I-KH) P_pred                  — needs explicit symmetrization
        % Use Joseph when ctrl_const.use_joseph_form is true (default false).
        if isfield(ctrl_const, 'use_joseph_form') && ctrl_const.use_joseph_form
            I_minus_KH = eye(7) - K_kf * H_use;
            P_post = I_minus_KH * P_pred * I_minus_KH' + K_kf * R_use * K_kf';
        else
            P_post = (eye(7) - K_kf * H_use) * P_pred;
            P_post = 0.5 * (P_post + P_post');
        end

        % --- Forgetting factor, thesis (4.15): P <- P / lambda_f ---
        % ctrl_const.lf_selective (ledger 21): inflate ONLY the gain block
        % (slots 6-7, the Q=0 states that need forgetting to stay adaptive):
        % P <- L P L with L = diag(1,..,1,1/sqrt(lf),1/sqrt(lf)). The
        % position chain (Q33 driven) and x_D pair keep their honest P.
        if lf_schedule && control_law_ch4
            % ledger 31: height-scheduled forgetting (cube-root law).
            h_d_cmd  = w_hat_n' * pd - pz_wall;
            hb_d     = max(h_d_cmd / R_radius, 1 + 1e-3);
            dhb_d    = abs(w_hat_n' * del_pd) / R_radius;
            % per-axis log-slope: perp = 1/(hb(hb-1)) (expgain b=1 family);
            % parallel = Goldman composite (9/16)/(hb(hb-1)*c_A) with
            % c_A = 1 + (8/15)ln(1+1/(hb-1)) -- hits the published log law
            % near wall (2.23 vs Goldman 2.135 at hb=1.111) and the 9/16
            % reflection slope far away (ledger 32).
            g_perp   = 1 / (hb_d * (hb_d - 1));
            c_A      = 1 + (8/15) * log(1 + 1 / (hb_d - 1));
            g_par    = (9/16) / (hb_d * (hb_d - 1) * c_A);
            wsq      = w_hat_n(ax)^2;
            g_ln     = wsq * g_perp + (1 - wsq) * g_par;
            r_drift  = g_ln * dhb_d / Ts;                  % |d ln a/dt| [1/s]
            % ledger 37: J must book the regressor the filter ACTUALLY runs.
            % With ch4_fdet the F_err pairing is f_det; without it the pairing
            % is the realized force f_d (16x larger on z) -- using f_det^2
            % here under-books z information ~250x and stretches T*.
            if ch4_fdet
                f_reg = f_det_cur(ax);
            else
                f_reg = f_d(ax);
            end
            if lf_f2_avg(ax) == 0
                lf_f2_avg(ax) = f_reg^2;                   % warm start
            end
            lf_f2_avg(ax) = (1 - lf_alpha_cyc) * lf_f2_avg(ax) ...
                            + lf_alpha_cyc * f_reg^2;
            J_y1     = lf_f2_avg(ax) / S1_pred_per_axis(ax);
            if gate_y2_off
                J_y2 = 0;
            else
                J_y2 = H_use(2, 6)^2 / R_use(2, 2);        % raw & whitened alike
            end
            J_ln_rate = x_curr(6)^2 * (J_y1 + J_y2) / Ts;  % relative Fisher [1/s]
            T_cube = (2 * J_ln_rate * max(r_drift, 1e-12)^2)^(-1/3) * lf_sched_scale(ax);
            T_min  = 1 / (0.10^2 * max(J_ln_rate, 1e-12)); % sigma_rel <= 10% validity floor
            T_star = max(T_cube, T_min);
            lf_use = min(max(1 - Ts / T_star, 0.95), 1 - 1e-5);
        else
            lf_use = lambda_f_p(ax);
        end
        lf_used_per_axis(ax) = lf_use;
        if lf_use < 1
            if isfield(ctrl_const, 'lf_selective') && ctrl_const.lf_selective
                s_lf = 1 / sqrt(lf_use);
                L_lf = ones(7, 1); L_lf(6) = s_lf; L_lf(7) = s_lf;
                P_post = (L_lf * L_lf') .* P_post;
            else
                P_post = P_post / lf_use;
            end
            P_post = 0.5 * (P_post + P_post');
        end

        % --- Re-force a_hat freeze post-update (defense in depth) ---
        if has_freeze
            x_post(6)    = a_hat_frz(ax);
            P_post(6, :) = 0;
            P_post(:, 6) = 0;
        end

        % Save K_kf entries needed by diag (always — cheap; 7x1 or 7x2 col)
        K_dx_y1_per_axis(ax) = K_kf(3, 1);    % gain on δx_3 from y_1 (col 1)
        innov_y1_per_axis(ax) = innov(1);     % y_1 innovation (ledger §17 diagnostic)
        K_a_y1_per_axis(ax)  = K_kf(6, 1);    % gain-slot update from y_1
        if gate_y2_off
            K_a_y2_per_axis(ax)   = 0;        % y_2 channel skipped
            innov_y2_per_axis(ax) = 0;
        else
            K_a_y2_per_axis(ax)   = K_kf(6, 2);
            innov_y2_per_axis(ax) = innov(2);
        end

        x_e_per_axis(:, ax) = x_post;
        P_per_axis{ax} = P_post;
    end

    % ------------------------------------------------------------------
    % [6] Bookkeeping: shift delay buffers, IIR states, k_step
    %     (Σf_d shift: f_d_km2 <- f_d_km1; f_d_km1 <- f_d_current)
    % ------------------------------------------------------------------
    if control_law_ch4
        f_det_km1 = f_det_cur;
        x34_prev = x34_used;   % the posteriors CONSUMED this call (k-1 vintage)
    end
    a_xm_prev = a_xm;   % raw AR(1) chain continues through gated steps (ledger 29)
    pd_km2 = pd_km1;
    pd_km1 = pd;

    f_d_km2 = f_d_km1;
    f_d_km1 = f_d;

    dx_bar_m       = dx_bar_m_new;
    sigma2_dxr_hat = sigma2_dxr_hat_new;

    k_step = k_step + 1;

    % ------------------------------------------------------------------
    % [7] Output
    % ------------------------------------------------------------------
    a_hat_post = x_e_per_axis(6, :)';   % updated a_hat (for diagnostic)
    if enable_wall
        h_bar_now = h_bar;
    else
        h_bar_now = 0;
    end
    ekf_out = [a_hat_post(1); ...   % a_hat_x  [um/pN]
               a_hat_post(3); ...   % a_hat_z
               a_hat_post(2); ...   % a_hat_y  (slot 3)
               h_bar_now];          % current h_bar (slot 4)

    % ------------------------------------------------------------------
    % [8] Optional diagnostic struct (only computed if requested)
    % ------------------------------------------------------------------
    if nargout >= 3
        % Per-axis posterior covariance entries
        P_a_per_axis   = zeros(3, 1);
        P_dx_per_axis  = zeros(3, 1);
        P77_per_axis   = zeros(3, 1);   % Phase 9 R(2,2) validation
        P_dx1_per_axis = zeros(3, 1);   % slot 1 (δx̂_1, Eq.17 row 1) cov
        for ax = 1:3
            P_a_per_axis(ax)   = P_per_axis{ax}(6, 6);
            P_dx_per_axis(ax)  = P_per_axis{ax}(3, 3);
            P77_per_axis(ax)   = P_per_axis{ax}(7, 7);
            P_dx1_per_axis(ax) = P_per_axis{ax}(1, 1);
        end

        diag = struct();
        diag.sigma2_dxr_hat       = sigma2_dxr_hat_new;            % 3x1
        diag.a_xm                 = a_xm;                          % 3x1
        diag.delta_x_m            = delta_x_m;                     % 3x1
        diag.innovation_y2        = innov_y2_per_axis;             % 3x1
        diag.K_kf_a_y2            = K_a_y2_per_axis;               % 3x1
        diag.K_kf_dx_y1           = K_dx_y1_per_axis;              % 3x1
        diag.innovation_y1        = innov_y1_per_axis;             % 3x1
        diag.K_kf_a_y1            = K_a_y1_per_axis;               % 3x1
        diag.S1_pred              = S1_pred_per_axis;              % 3x1
        diag.lambda_f_used = lf_used_per_axis;                     % 3x1 (ledger 31)
        if control_law_ch4
            diag.f_det = f_det_cur;                                % 3x1 (fdet regressor as consumed)
        else
            diag.f_det = nan(3, 1);
        end
        diag.P_a                  = P_a_per_axis;                  % 3x1
        diag.P_dx                 = P_dx_per_axis;                 % 3x1
        diag.x_D_hat              = x_e_per_axis(4, :).';          % 3x1
        diag.delta_a_hat          = x_e_per_axis(7, :).';          % 3x1
        diag.gate_active_per_axis = gate_y2_off_per_axis;          % 3x1 logical
        diag.guards_individual    = G_per_axis;                    % 3x3 logical
        diag.h_bar                = h_bar;                         % scalar
        diag.f_d                  = f_d;                           % 3x1
        diag.dx_r                 = dx_r;                          % 3x1 (Phase 9)
        diag.a_hat                = a_hat_post;                    % 3x1 (Phase 9, slot 6)
        diag.P77                  = P77_per_axis;                  % 3x1 (Phase 9)
        diag.Q77                  = Q77_per_axis;                  % 3x1 (Phase 9)
        diag.delta_x_hat_1        = x_e_per_axis(1, :).';          % 3x1 (Eq.17 slot 1, δx̂_1)
        diag.P_dx1                = P_dx1_per_axis;                % 3x1 (P(1,1), δx̂_1 cov)
    end

end


%% =================== Local Helper ===================

function F_e = build_F_e(lambda_c, d_delay, f_d_i, F_1_i, F_2_i, use_eq18)
%BUILD_F_E Build 7x7 augmented system matrix (per axis), v2.
%
%   See phase1_Fe_derivation.md §10 + Fe_H_derivation.tex (exact version).
%
%   Inputs:
%       lambda_c  - closed-loop pole (scalar)
%       d_delay   - measurement delay (steps, typically 2)
%       f_d_i     - current control force on this axis: f_d[k]
%       F_1_i     - F_1[k] = sum_{i=1..d} f_d[k-i] (per axis)
%       F_2_i     - F_2[k] = sum_{i=1..d} i * f_d[k-i] (per axis)
%       use_eq18  - optional flag (default false), selects Row 3 partition
%
%   Row 3 (Eq.19, v2 default — exact F_e Fe_H_derivation.tex):
%       [0, 0, λ_c, -(1+d·(1-λ_c)), (d(d+1)/2)·(1-λ_c),
%        -f_d_i - (1-λ_c)·F_1_i,  (1-λ_c)·F_2_i]
%
%   Row 3 (Eq.18 legacy direct partition):
%       [-(1-λ_c), 0, 1, -1, (d(d+1)/2)·(1-λ_c),
%        -f_d_i - (1-λ_c)·F_1_i,  (1-λ_c)·F_2_i]
%       Columns (3,5)/(3,6)/(3,7) use the same exact F_1/F_2 contribution
%       as Eq.19 (only (3,1), (3,3), (3,4) differ between forms).
%
%   Eq.19 form (default) is the v2 paper-aligned algebraic rearrangement
%   (Phase 1 §6) — Σf_d substitution introduces past x_D contribution that
%   scales F_e(3,4) by (1+d·(1-λ_c)) under Option I (slowly-varying x_D).
%
%   F_1_i / F_2_i closed forms (for d=2):
%       F_1_i = f_d[k-1] + f_d[k-2]
%       F_2_i = 1·f_d[k-1] + 2·f_d[k-2]
%   Passing F_1_i = F_2_i = 0 reproduces the legacy 4-argument call
%   (positioning steady-state Riccati limit, no control-history coupling).

    if nargin < 6 || isempty(use_eq18)
        use_eq18 = false;
    end
    if nargin < 5 || isempty(F_2_i)
        F_2_i = 0;
    end
    if nargin < 4 || isempty(F_1_i)
        F_1_i = 0;
    end

    one_minus_lc = 1 - lambda_c;
    Fe35 = (d_delay * (d_delay + 1) / 2) * one_minus_lc;
    Fe36 = -f_d_i - one_minus_lc * F_1_i;
    Fe37 = one_minus_lc * F_2_i;

    if use_eq18
        F_e = [0           1 0        0  0    0    0; ...
               0           0 1        0  0    0    0; ...
              -one_minus_lc 0 1       -1  Fe35 Fe36 Fe37; ...
               0           0 0        1  1    0    0; ...
               0           0 0        0  1    0    0; ...
               0           0 0        0  0    1    1; ...
               0           0 0        0  0    0    1];
    else
        % Eq.19 form (v2 default): F_e(3,4) = -(1 + d·(1-λ_c))
        % For d=2, λ_c=0.7  -> -(1 + 2·0.3) = -1.6
        Fe34 = -(1 + d_delay * one_minus_lc);
        F_e = [0 1 0        0     0    0    0; ...
               0 0 1        0     0    0    0; ...
               0 0 lambda_c Fe34  Fe35 Fe36 Fe37; ...
               0 0 0        1     1    0    0; ...
               0 0 0        0     1    0    0; ...
               0 0 0        0     0    1    1; ...
               0 0 0        0     0    0    1];
    end
end


function P_post = solve_dare_kf_local(F, H, Q, R, lambda_f)
%SOLVE_DARE_KF_LOCAL  Discrete-time KF Riccati steady-state solver.
%
%   P_post = solve_dare_kf_local(F, H, Q, R)             % plain KF
%   P_post = solve_dare_kf_local(F, H, Q, R, lambda_f)   % with (4.15) forgetting
%
%   Fixed-point iteration on the KF Riccati recursion:
%     P_pred[k+1] = F * P_post[k] * F' + Q
%     K           = P_pred * H' / (H * P_pred * H' + R)
%     P_post[k+1] = (I - K*H) * P_pred[k+1] / lambda_f
%   until ||P_post[k+1] - P_post[k]||_inf < tol or max_iter reached.
%   lambda_f < 1 gives the forgetting-sustained equilibrium (nonzero even
%   for states with zero process noise).
%
%   Used for Pf_init at known h_init operating point (positioning, f_d=0).
%   Self-contained — no Control System Toolbox dependency. ~10000 iter at
%   tol=1e-13 takes ~50 ms; called once per axis at controller init.
    if nargin < 5 || isempty(lambda_f)
        lambda_f = 1;   % plain KF recursion (legacy call sites)
    end
    n = size(F, 1);
    P_post = eye(n);
    max_iter = 10000;
    tol = 1e-13;
    for k = 1:max_iter
        P_pred = F * P_post * F' + Q;
        P_pred = 0.5 * (P_pred + P_pred');
        S = H * P_pred * H' + R;
        K = (P_pred * H') / S;
        P_new = (eye(n) - K * H) * P_pred / lambda_f;
        P_new = 0.5 * (P_new + P_new');
        if max(abs(P_new(:) - P_post(:))) < tol
            P_post = P_new;
            return;
        end
        P_post = P_new;
    end
end


function d = empty_diag()
%EMPTY_DIAG Zeroed diagnostic struct (used for bypass / init returns).
    d = struct();
    d.sigma2_dxr_hat       = zeros(3, 1);
    d.a_xm                 = zeros(3, 1);
    d.delta_x_m            = zeros(3, 1);
    d.innovation_y2        = zeros(3, 1);
    d.K_kf_a_y2            = zeros(3, 1);
    d.K_kf_dx_y1           = zeros(3, 1);
    d.P_a                  = zeros(3, 1);
    d.P_dx                 = zeros(3, 1);
    d.x_D_hat              = zeros(3, 1);
    d.delta_a_hat          = zeros(3, 1);
    d.gate_active_per_axis = false(3, 1);
    d.guards_individual    = false(3, 3);
    d.h_bar                = 0;
    d.f_d                  = zeros(3, 1);
    d.dx_r                 = zeros(3, 1);   % Phase 9 R(2,2) validation: IIR HP residual
    d.a_hat                = zeros(3, 1);   % Phase 9: per-axis slot-6 estimate
    d.P77                  = zeros(3, 1);   % Phase 9: per-axis slot-7 covariance
    d.Q77                  = zeros(3, 1);   % Phase 9: per-axis Q77
    d.delta_x_hat_1        = zeros(3, 1);   % Eq.17 slot 1 (δx̂_1)
    d.P_dx1                = zeros(3, 1);   % P(1,1) (δx̂_1 cov)
end
