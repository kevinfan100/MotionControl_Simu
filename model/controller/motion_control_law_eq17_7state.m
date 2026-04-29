function [f_d, ekf_out] = motion_control_law_eq17_7state(del_pd, pd, p_m, params, ctrl_const)
%MOTION_CONTROL_LAW_EQ17_7STATE Per-axis 7-state EKF controller (paper 2023 Eq.17)
%
%   [f_d, ekf_out] = motion_control_law_eq17_7state(del_pd, pd, p_m, params, ctrl_const)
%
%   Implements 3 independent 7-state EKF controllers (one per axis x, y, z)
%   following paper 2023 Eq.(17) with d-step delay-compensated control law,
%   adaptive Q33/Q77, and 3-guard adaptive R_2.
%
%   References (see reference/eq17_analysis/design.md):
%       §3   Eq.17 control law
%       §5   F_e (only entry (3,6) = -f_d is time-varying)
%       §6   Measurement architecture (H matrix, d-step delay)
%       §8.2 Q33 = 4 kBT * a_hat (Path C strict)
%       §8.4 Q77 (Path B full Var(w_a) with K_h, K_h')
%       §9.4 R_2_intrinsic = a_cov * IF_var * (a_hat + xi)^2
%       §9.5 R_2_eff = R_2_intrinsic + 5 * Q77 (d=2)
%       §9.9 3-guard adaptive R_2
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
        return;
    end

    % ------------------------------------------------------------------
    % Persistent state declarations
    % ------------------------------------------------------------------

    % EKF state per axis: 7x3 matrix (column i = axis i)
    persistent x_e_per_axis

    % Covariance per axis: stored as cell {3} of 7x7 (cleaner than 7x7x3)
    persistent P_per_axis

    % IIR states (per axis, 3x1 vectors)
    persistent dx_bar_m       % LP mean of delta_x_m         [3x1, um]
    persistent sigma2_dxr_hat % EWMA variance of dx_r        [3x1, um^2]

    % Trajectory delay buffers — to access pd[k-1], pd[k-2]
    persistent pd_km1 pd_km2

    % Step counter (1-based, increments after first init call)
    persistent k_step

    % Cached scalars / vectors (extracted once at init)
    persistent initialized
    persistent lambda_c d_delay Ts kBT R_radius
    persistent a_var a_cov
    persistent C_dpmr C_n IF_var xi_per_axis delay_R2_factor
    persistent t_warmup_kf h_bar_safe
    persistent sigma2_n_s          % 3x1 [um^2]
    persistent h_dot_max h_ddot_max
    persistent enable_wall          % logical, fall back to flat (h_bar=inf) if false
    persistent w_hat_n pz_wall      % wall geometry
    persistent R_OFF                % large-R fallback for guarded y_2

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
        xi_per_axis     = ctrl_const.xi_per_axis;         % 3x1 [um/pN]
        delay_R2_factor = ctrl_const.delay_R2_factor;     % = 5 for d=2
        t_warmup_kf     = ctrl_const.t_warmup_kf;
        h_bar_safe      = ctrl_const.h_bar_safe;
        a_cov           = ctrl_const.a_cov;
        a_var           = a_cov;                           % §6: dx_bar_m EWMA uses same coefficient

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

        % --- 0E. Initialize EKF state ---
        a_nom = Ts / gamma_N;               % [um/pN] nominal motion gain
        x_e_per_axis = zeros(7, 3);
        x_e_per_axis(6, :) = a_nom;          % seed a_x ~ a_nom for all axes

        % Covariance init from existing config (3 independent 7x7)
        Pf_diag = params.ctrl.Pf_init_diag;  % 7x1
        P_per_axis = cell(3, 1);
        for ax = 1:3
            P_per_axis{ax} = diag(Pf_diag);
        end

        % --- 0F. IIR states ---
        dx_bar_m       = zeros(3, 1);
        sigma2_dxr_hat = zeros(3, 1);

        % --- 0G. Trajectory delay buffers ---
        pd_km1 = pd;
        pd_km2 = pd;

        % --- 0H. Misc ---
        k_step = 1;                          % first user call counts as k=1
        R_OFF  = 1e10;                       % gate-off variance (design.md §9.9)

        % --- 0I. First call returns zeros (no f_d yet) ---
        f_d = zeros(3, 1);
        ekf_out = [a_nom; a_nom; a_nom; 0];
        return;
    end

    % ------------------------------------------------------------------
    % Per-step variables — extract per-axis state (k step starts here)
    % ------------------------------------------------------------------
    a_hat = x_e_per_axis(6, :)';   % 3x1 [um/pN]
    xD_hat = x_e_per_axis(4, :)';  % 3x1 [um]

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

    % ------------------------------------------------------------------
    % [1] Eq.17 control law (per axis)  — see §3
    %
    %   f_d[k] = (1/a_hat) * { x_d[k+1] - lambda_c*x_d[k] - (1-lambda_c)*x_d[k-d]
    %                        + (1-lambda_c)*delta_x_m[k] - x_D_hat }
    %
    % Note: a_hat carried over from previous step's posterior update (a_hat[k|k-1]
    % from the integrated-random-walk forecast). On the very first non-init step
    % (k_step == 1 here), a_hat == a_nom seeded above.
    % ------------------------------------------------------------------
    one_minus_lc = 1 - lambda_c;
    f_d = (1 ./ a_hat) .* ( ...
            pd_kp1 ...
          - lambda_c * pd ...
          - one_minus_lc * pd_km_d ...
          + one_minus_lc * delta_x_m ...
          - xD_hat );

    % ------------------------------------------------------------------
    % [2] IIR a_xm  (paper 2025 Eq.9-13)  — per axis
    %
    %   dx_bar_m[k+1] = (1 - a_var) * dx_bar_m[k] + a_var * delta_x_m[k]
    %   dx_r[k]       = delta_x_m[k] - dx_bar_m[k]              (centered)
    %   sigma2_dxr_hat[k+1] = (1 - a_cov) * sigma2_dxr_hat[k]
    %                       + a_cov * (dx_r^2[k] - dx_bar_r_sq[k])
    %     (steady-state dx_bar_r ≈ 0; we keep dx_r^2 directly since
    %      dx_r is already centered, matching design.md §9.3.)
    %
    %   a_xm[k] = (sigma2_dxr_hat[k] - C_n * sigma2_n_s) / (C_dpmr * 4 kBT)
    % ------------------------------------------------------------------
    % Update LP mean using current measurement (post-update form)
    dx_bar_m_new = (1 - a_var) * dx_bar_m + a_var * delta_x_m;
    dx_r = delta_x_m - dx_bar_m_new;                          % centered residual

    % Variance EWMA — δ̄x_r ~ 0 in steady state (§9.3 caveat, ~2.5% bias OK)
    sigma2_dxr_hat_new = (1 - a_cov) * sigma2_dxr_hat + a_cov * dx_r.^2;

    den_axm = C_dpmr * 4 * kBT;                               % [pN*um] scaled
    a_xm = (sigma2_dxr_hat_new - C_n * sigma2_n_s) / den_axm; % 3x1 [um/pN]

    % ------------------------------------------------------------------
    % [3] Adaptive Q and R (per axis)  — see §8 and §9
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
    for ax = 1:3
        a_hat_i = a_hat(ax);
        K_h_i   = K_h_axis(ax);
        K_h_p_i = K_h_pr_axis(ax);

        % Q33,i = 4 kBT * a_hat,i  (Path C strict, §8.2)
        Q33_i = 4 * kBT * a_hat_i;

        % Q55 = 0  (simulation, §8.3)
        Q55_i = 0;

        % Q77,i = dt^4 * a_hat^2 * { (K_h^2 - K_h')^2 * h_dot_max^4 / (8 R^4)
        %                          + K_h^2          * h_ddot_max^2 / (2 R^2) }
        % (§8.4)
        term_A = (K_h_i^2 - K_h_p_i)^2 * h_dot_max^4 / 8 * R4_inv;
        term_B = K_h_i^2               * h_ddot_max^2 / 2 * R2_inv;
        Q77_i = Ts4 * a_hat_i^2 * (term_A + term_B);

        Q_i = zeros(7);
        Q_i(3, 3) = Q33_i;
        Q_i(5, 5) = Q55_i;
        Q_i(7, 7) = Q77_i;
        Q_per_axis{ax} = Q_i;

        % R(1,1) = sigma2_n_s,i  (§9.6)
        R11_i = sigma2_n_s(ax);

        % R(2,2) intrinsic = a_cov * IF_var * (a_hat + xi)^2  (§9.4)
        R2_intrinsic_i = a_cov * IF_var * (a_hat_i + xi_per_axis(ax))^2;
        % R(2,2) eff = intrinsic + delay_R2_factor * Q77  (§9.5)
        R2_eff_i = R2_intrinsic_i + delay_R2_factor * Q77_i;

        % --- 3-guard adaptive R_2 (§9.9) -------------------------------
        t_now = (k_step - 1) * Ts;                  % real time at step k
        G1 = (t_now < t_warmup_kf);                 % warm-up
        G2 = ((sigma2_dxr_hat_new(ax) - C_n * sigma2_n_s(ax)) <= 0);  % low SNR
        G3 = (h_bar < h_bar_safe);                  % near-wall

        if G1 || G2 || G3
            R22_i = R_OFF;
        else
            R22_i = R2_eff_i;
        end

        R_per_axis{ax} = diag([R11_i, R22_i]);
    end

    % ------------------------------------------------------------------
    % [4] EKF predict + update per axis  — see §5, §6
    %
    %   F_e (7x7), only (3,6) is time-varying:  F_e(3,6) = -f_d[k]
    %   H   (2x7), (2,7) = -d_delay
    % ------------------------------------------------------------------
    H = [1 0 0 0 0 0       0; ...
         0 0 0 0 0 1 -d_delay];

    for ax = 1:3
        % F_e takes the current f_d[k] computed above (only (3,6) varies)
        F_e = build_F_e(lambda_c, f_d(ax));

        x_curr = x_e_per_axis(:, ax);
        P_curr = P_per_axis{ax};

        % --- Predict ---
        x_pred = F_e * x_curr;
        P_pred = F_e * P_curr * F_e' + Q_per_axis{ax};
        P_pred = 0.5 * (P_pred + P_pred');

        % --- Update with [delta_x_m; a_xm] ---
        y_meas = [delta_x_m(ax); a_xm(ax)];
        y_pred = H * x_pred;
        innov  = y_meas - y_pred;

        S = H * P_pred * H' + R_per_axis{ax};
        S = 0.5 * (S + S') + 1e-20 * eye(2);   % numerical regularization
        K_kf = (P_pred * H') / S;               % 7x2 (avoid shadowing 'K')

        x_post = x_pred + K_kf * innov;
        P_post = (eye(7) - K_kf * H) * P_pred;
        P_post = 0.5 * (P_post + P_post');

        x_e_per_axis(:, ax) = x_post;
        P_per_axis{ax} = P_post;
    end

    % ------------------------------------------------------------------
    % [5] Bookkeeping: shift trajectory delay buffer, IIR states, k_step
    % ------------------------------------------------------------------
    pd_km2 = pd_km1;
    pd_km1 = pd;

    dx_bar_m       = dx_bar_m_new;
    sigma2_dxr_hat = sigma2_dxr_hat_new;

    k_step = k_step + 1;

    % ------------------------------------------------------------------
    % [6] Output
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

end


%% =================== Local Helper ===================

function F_e = build_F_e(lambda_c, f_d_i)
%BUILD_F_E Build 7x7 augmented system matrix (per axis).
%
%   See design.md §5 — only (3,6) is time-varying.

    F_e = [0 1 0        0  0   0       0; ...
           0 0 1        0  0   0       0; ...
           0 0 lambda_c -1 0  -f_d_i   0; ...
           0 0 0        1  1   0       0; ...
           0 0 0        0  1   0       0; ...
           0 0 0        0  0   1       1; ...
           0 0 0        0  0   0       1];
end
