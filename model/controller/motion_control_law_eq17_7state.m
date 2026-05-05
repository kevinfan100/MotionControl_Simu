function [f_d, ekf_out, diag] = motion_control_law_eq17_7state(del_pd, pd, p_m, params, ctrl_const)
%MOTION_CONTROL_LAW_EQ17_7STATE Per-axis 7-state EKF controller (paper 2023 Eq.17, v2)
%
%   [f_d, ekf_out]       = motion_control_law_eq17_7state(del_pd, pd, p_m, params, ctrl_const)
%   [f_d, ekf_out, diag] = motion_control_law_eq17_7state(del_pd, pd, p_m, params, ctrl_const)
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
    persistent lambda_c d_delay Ts kBT R_radius
    persistent a_pd a_var a_cov sigma2_w_fD sigma2_w_fA
    persistent C_dpmr C_n IF_var IF_eff IF_eff_per_axis R22_prefactor xi_per_axis delay_R2_factor
    persistent C_dpmr_eff_per_axis C_np_eff_per_axis  % Stage 11 Option I: per-axis
    persistent t_warmup_kf h_bar_safe
    persistent sigma2_n_s          % 3x1 [um^2]
    persistent h_dot_max h_ddot_max
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

        % --- 0F. Initialize EKF state ---
        x_e_per_axis = zeros(7, 3);
        x_e_per_axis(6, :) = a_x_init.';    % seed a_x per axis (wall-aware)

        % --- 0G. Pf_init per-axis (Phase 8 §7) ---
        % Phase 8 §7 spec: per-axis Pf_init derived from a_x_init.
        %   sigma2_dXT_axis_init = 4·k_B·T·a_x_init_axis
        %
        %   Pf_init = diag(0,                        slot 1 (δx[k-2] pre-drift)
        %                  σ²_dXT_axis_init,         slot 2 (δx[k-1], 1-step drift)
        %                  2·σ²_dXT_axis_init,       slot 3 (δx[k], 2-step drift)
        %                  0,                        slot 4 (x_D baseline)
        %                  0,                        slot 5 (δx_D baseline)
        %                  1e-5,                     slot 6 (a_x init uncertainty)
        %                  0)                        slot 7 (δa_x baseline)
        %
        % NOTE: output var `diag` shadows MATLAB's diag() builtin in this
        % function, so we construct diagonal matrices via an explicit loop.
        % Optional override for slot 7 (delta a_x) initial uncertainty.
        % Default 0 (matches Phase 8 baseline). Set > 0 to give the random-walk
        % rate state initial permission to drift even without sigma2_w_fA Q77 floor.
        if isfield(ctrl_const, 'Pf_init_slot7') && ~isempty(ctrl_const.Pf_init_slot7)
            Pf_init_slot7 = ctrl_const.Pf_init_slot7;
        else
            Pf_init_slot7 = 0;
        end

        P_per_axis = cell(3, 1);
        for ax = 1:3
            sigma2_dXT_ax = 4 * kBT * a_x_init(ax);
            Pf_ax = zeros(7);
            Pf_ax(2, 2) = sigma2_dXT_ax;
            Pf_ax(3, 3) = 2 * sigma2_dXT_ax;
            Pf_ax(6, 6) = 1e-5;
            Pf_ax(7, 7) = Pf_init_slot7;
            P_per_axis{ax} = Pf_ax;
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
        error('motion_control_law_eq17_7state:unsupportedDelay', ...
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
    f_d = inv_a_hat .* ( ...
                pd_kp1 ...
              - lambda_c * pd ...
              - one_minus_lc * pd_km_d ...
              + one_minus_lc * delta_x_m ) ...
          - one_minus_lc * sum_fd_past ...
          - xD_hat_for_ctrl .* inv_a_hat;

    % ------------------------------------------------------------------
    % [4] Adaptive Q and R (per axis)  — Phase 5 / Phase 6
    % ------------------------------------------------------------------
    % Per-axis K_h, K_h' from h_bar
    %   x, y axes  ->  K_h_para, K_h_prime_para
    %   z axis     ->  K_h_perp, K_h_prime_perp
    if enable_wall && isfinite(h_bar) && h_bar > 1
        [~, ~, derivs] = calc_correction_functions(h_bar, true);
        K_h_axis      = [derivs.K_h_para; derivs.K_h_para; derivs.K_h_perp];
        K_h_pr_axis   = [derivs.K_h_prime_para; derivs.K_h_prime_para; derivs.K_h_prime_perp];
    else
        % Far-field / wall-disabled: K_h ~ 0, K_h' ~ 0 → Q77 ~ 0
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

        Q_i = zeros(7);
        Q_i(3, 3) = Q33_i;
        Q_i(5, 5) = Q55_i;
        Q_i(7, 7) = Q77_i;
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
        R2_intrinsic_i = R22_prefactor * IF_eff_per_axis(ax) ...
                         * (a_hat_i + xi_per_axis(ax))^2;
        % R(2,2) eff = intrinsic + delay_R2_factor * Q77  (Phase 6 §4.3)
        R2_eff_i = R2_intrinsic_i + delay_R2_factor * Q77_i;

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
        % F_e takes the current f_d[k] computed above (only (3,6) varies)
        F_e = build_F_e(lambda_c, d_delay, f_d(ax), use_eq18);

        x_curr = x_e_per_axis(:, ax);
        P_curr = P_per_axis{ax};

        % --- Predict ---
        x_pred = F_e * x_curr;
        P_pred = F_e * P_curr * F_e' + Q_per_axis{ax};
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
        end

        % --- Update (sequential / conditional) ---
        y_pred = H_use * x_pred;
        innov  = y_use - y_pred;

        S = H_use * P_pred * H_use' + R_use;
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

        x_post = x_pred + K_kf * innov;
        P_post = (eye(7) - K_kf * H_use) * P_pred;
        P_post = 0.5 * (P_post + P_post');

        % --- Re-force a_hat freeze post-update (defense in depth) ---
        if has_freeze
            x_post(6)    = a_hat_frz(ax);
            P_post(6, :) = 0;
            P_post(:, 6) = 0;
        end

        % Save K_kf entries needed by diag (always — cheap; 7x1 or 7x2 col)
        K_dx_y1_per_axis(ax) = K_kf(3, 1);    % gain on δx_3 from y_1 (col 1)
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
        P_a_per_axis  = zeros(3, 1);
        P_dx_per_axis = zeros(3, 1);
        P77_per_axis  = zeros(3, 1);   % Phase 9 R(2,2) validation
        for ax = 1:3
            P_a_per_axis(ax)  = P_per_axis{ax}(6, 6);
            P_dx_per_axis(ax) = P_per_axis{ax}(3, 3);
            P77_per_axis(ax)  = P_per_axis{ax}(7, 7);
        end

        diag = struct();
        diag.sigma2_dxr_hat       = sigma2_dxr_hat_new;            % 3x1
        diag.a_xm                 = a_xm;                          % 3x1
        diag.delta_x_m            = delta_x_m;                     % 3x1
        diag.innovation_y2        = innov_y2_per_axis;             % 3x1
        diag.K_kf_a_y2            = K_a_y2_per_axis;               % 3x1
        diag.K_kf_dx_y1           = K_dx_y1_per_axis;              % 3x1
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
    end

end


%% =================== Local Helper ===================

function F_e = build_F_e(lambda_c, d_delay, f_d_i, use_eq18)
%BUILD_F_E Build 7x7 augmented system matrix (per axis), v2.
%
%   See phase1_Fe_derivation.md §10 — only (3,6) is time-varying.
%
%   use_eq18 (optional, default false) selects Row 3 form:
%       false (Eq.19, v2 default):
%           Row 3 = [0, 0, λ_c, -(1+d·(1-λ_c)), 0, -f_d_i, 0]
%           For d=2, λ_c=0.7: F_e(3,4) = -1.6
%       true  (Eq.18 direct):
%           Row 3 = [-(1-λ_c), 0, 1, -1, 0, -f_d_i, 0]
%
%   Eq.19 form (default) is the v2 paper-aligned algebraic rearrangement
%   (Phase 1 §6) — Σf_d substitution introduces past x_D contribution that
%   scales F_e(3,4) by (1+d·(1-λ_c)) under Option I (slowly-varying x_D).
%
%   Eq.18 is preserved for testing/comparison; same dynamics, different
%   partition.

    if nargin < 4 || isempty(use_eq18)
        use_eq18 = false;
    end

    if use_eq18
        F_e = [0           1 0        0  0   0       0; ...
               0           0 1        0  0   0       0; ...
              -(1-lambda_c) 0 1       -1  0  -f_d_i  0; ...
               0           0 0        1  1   0       0; ...
               0           0 0        0  1   0       0; ...
               0           0 0        0  0   1       1; ...
               0           0 0        0  0   0       1];
    else
        % Eq.19 form (v2 default): F_e(3,4) = -(1 + d·(1-λ_c))
        % For d=2, λ_c=0.7  -> -(1 + 2·0.3) = -1.6
        Fe34 = -(1 + d_delay * (1 - lambda_c));
        F_e = [0 1 0        0     0   0       0; ...
               0 0 1        0     0   0       0; ...
               0 0 lambda_c Fe34  0  -f_d_i   0; ...
               0 0 0        1     1   0       0; ...
               0 0 0        0     1   0       0; ...
               0 0 0        0     0   1       1; ...
               0 0 0        0     0   0       1];
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
end
