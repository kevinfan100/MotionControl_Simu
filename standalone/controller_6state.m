function [f_d, ekf_out] = controller_6state(del_pd, pd, p_m, params)
%CONTROLLER_6STATE Per-axis 6-state EKF controller (RevisedControl_Vpersonal).
%
%   [f_d, ekf_out] = controller_6state(del_pd, pd, p_m, params)
%
%   PACKAGING STATUS: parts 5-6 build -- the full EKF is live (sections
%   [3]-[10], paper predictor form + Joseph, D6). No guards: unlike the
%   mother repo there is no G1/G2/G3 gating, no 1D-update collapse, no
%   R_OFF (PACKAGING_PLAN decision 6); the operating envelope is
%   h_bar > 1.2 with near-wall a_hat limitations documented (L2).
%   Reset between runs: `clear controller_6state` (the driver does this).
%
%   State (per axis, Vpersonal p.3):
%       x = [delta_x_1; delta_x_2; delta_x_3; xD_comb; a_x; delta_a_x]
%       delta_x_1 = delta_x[k-2]   (oldest; matches y_1)
%       delta_x_2 = delta_x[k-1]
%       delta_x_3 = delta_x[k]     (current)
%       xD_comb   = delta_x_D^d, pre-combined disturbance
%                   (= delta_x_D[k] + (1-lc)*sum delta_x_D[k-i])
%       a_x       = motion gain [um/pN],  delta_a_x = gain rate
%
%   Measurements (per axis):
%       y_1 = delta_x_m = p_d[k-2] - p_m[k]
%       y_2 = a_xm = (sigma2_dxr_hat - C_n*sigma2_nx) / (C_dpmr*4kBT)
%
%   D6 (kf_canonical_spec section 1b): persistent stores the A-PRIORI
%   pair (x_hat[k|k-1], P_f[k]); per-step cycle Eq.19 -> 16 -> 20(Joseph)
%   -> 21; merged one-line state update x+ = Phi_map(x) + L*innov (gain
%   position L per paper convention). Phi (mean map, Row3 = lc only)
%   differs from F_e (error couplings -1, -F_dx, dF_dx): the mean uses
%   Phi, the covariance uses F_e -- the deterministic-map predict that
%   fixes the 7-state a_hat bias. All outputs are a-priori values.
%
%   Inputs:
%       del_pd - Trajectory increment p_d[k+1] - p_d[k]  [3x1, um]
%       pd     - Current desired position p_d[k]          [3x1, um]
%       p_m    - Measured position (d-step delayed)       [3x1, um]
%       params - Nested parameter struct from config.m
%
%   Outputs:
%       f_d     - Control force f_d[k] [3x1, pN]
%       ekf_out - [a_hat_x; a_hat_y; a_hat_z; h_bar]  (natural axis order;
%                 a-priori estimates -> one-step shift vs a_true in logs)
%
%   Execution order:
%     [0]  Init (first call): offline constants (C_dpmr, C_n, K_var,
%          IF_abc + self-check, xi), wall-aware a_hat seed, DARE ->
%          a-priori P_f0, prefill IIR, buffers
%     [1]  Measurement: delta_x_m, h_bar, K_h(h_bar); IIR -> a_xm
%     [2]  Control law (Eq.17 ACTIVE form, a-priori a_hat / xD_comb)
%          (measurement BEFORE control: Eq.17 feeds back the RAW current
%           delta_x_m -- unlike the 23-state, whose law uses estimates)
%     ---- per-axis loop (x, y, z) ----
%     [3]  Innovations e = y - H*x_hat (against stored a-priori)
%     [4]  R[k]: R11 = sigma2_nx; R22 = K_var*IF_eff*(a_hat+xi)^2
%          + buffered delay sum {1,1}
%     [5]  Gain  L = P_f*H'/(H*P_f*H' + R)                       (Eq.19)
%     [6]  State x+ = Phi_map(x) + L*e                  (merged, Eq.16)
%     [7]  Posterior covariance, Joseph form            (Eq.20 -> Joseph)
%     [8]  F_e[k] (time-varying Row 3 via f_d history)
%     [9]  Q[k]: Q33 three-component ({4,1} randgain weights), Q55
%     [10] Forecast P_f+ = F_e*P*F_e' + Q                        (Eq.21)
%     ---- end loop ----
%     [11] Buffer shifts + ekf_out
%
%   Params fields read here (param-flow contract):
%       params.ctrl.enable / lambda_c / a_pd / a_cov / k_B / T /
%                   Ts / gamma / sigma2_noise
%       params.common.R / p0
%       params.wall.w_hat / pz / enable_wall_effect
%
%   d = 2 sensor-delay steps is HARDCODED (PACKAGING_PLAN decision 6):
%   the C_dpmr / C_n / IF constants are derived for d = 2 only.
%
%   See also: wall_corrections, run_simulation, config

    % ------------------------------------------------------------------
    % [0] Open-loop bypass
    % ------------------------------------------------------------------
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = [1; 1; 1; 0];      % mother-repo bypass sentinel (a placeholders)
        return;
    end

    % ------------------------------------------------------------------
    % Persistent state
    % ------------------------------------------------------------------
    persistent initialized
    persistent lambda_c a_pd a_cov kBT R_radius Ts gamma_N sigma2_nx
    persistent C_dpmr C_n K_var IF_abc xi_per_axis var_da_increment_factor
    persistent w_hat_n pz_wall enable_wall             % wall geometry
    persistent x_e_per_axis                            % 6x3 a-priori state x_hat[k|k-1] (D6)
    persistent P_per_axis                              % cell{3} of 6x6 forecast covariance P_f[k] (D6)
    persistent dx_bar_m sigma2_dxr_hat                 % IIR states (3x1)
    persistent pd_km1 pd_km2                           % trajectory delay buffers
    persistent f_d_km1 f_d_km2                         % past control forces (Sigma f_d[k-i])
    persistent a_hat_km1 a_hat_km2                     % past gain estimates (ACTIVE law + Q33)
    persistent var_da_ram_km1 var_da_ram_km2           % past var(delta_a_ram[k-i]) (Q33/R22)

    % ------------------------------------------------------------------
    % [0] Initialization on first call
    % ------------------------------------------------------------------
    if isempty(initialized)
        initialized = true;

        % --- 0A. params constants ---
        Ts        = params.ctrl.Ts;
        kBT       = params.ctrl.k_B * params.ctrl.T;     % [pN*um]
        R_radius  = params.common.R;
        gamma_N   = params.ctrl.gamma;                   % [pN*sec/um]
        sigma2_nx = params.ctrl.sigma2_noise;            % 3x1 [um^2]
        lambda_c  = params.ctrl.lambda_c;
        a_pd      = params.ctrl.a_pd;
        a_cov     = params.ctrl.a_cov;

        % --- 0B. Offline constants (kf_canonical_spec section 7) ---
        % C_dpmr / C_n: FULL a_pd-dependent closed form
        % (lambda_c=0.7, a_pd=0.05 -> C_dpmr = 3.1610, C_n = 1.1093)
        one_m_apd  = 1 - a_pd;
        denom_pole = 1 - one_m_apd * lambda_c;          % shared pole product
        C_dpmr = one_m_apd^2 * ( ...
                   2 * one_m_apd * (1 - lambda_c) / denom_pole ...
                 + (2 / (2 - a_pd)) * 1 / ((1 + lambda_c) * denom_pole) );
        C_n = (2 * one_m_apd^2 / (2 - a_pd)) * ( ...
                   1 ...
                 + one_m_apd^2 * a_pd * (1 - lambda_c) / denom_pole ...
                 + (1 - lambda_c)^2 / ((1 + lambda_c) * denom_pole) );

        % K_var (R22 prefactor): Isserlis Var(X^2) = 2 sigma^4 x EWMA gain
        K_var = 2 * a_cov / (2 - a_cov);

        % var(delta_a_ram) closed-form increment factor 2/(1+lc): the
        % closed-loop level amplification C_dx = 2+1/(1-lc^2) and the
        % increment smoothing (1-rho1) cancel to 2/(1+lc).
        var_da_increment_factor = 2 / (1 + lambda_c);

        % IF_abc: s-weighted autocorrelation sums for the exact per-step
        % IF_eff (R22_derivation S4-S6), with the R_fT(0)/R_fN(0)
        % self-check (an independent route to the same C_dpmr / C_n).
        [IF_A, IF_B, IF_C, RfT0, RfN0] = compute_if_abc(lambda_c, a_pd, a_cov);
        IF_abc = [IF_A; IF_B; IF_C];
        assert(abs(RfT0 - C_dpmr) <= 1e-5 * C_dpmr && abs(RfN0 - C_n) <= 1e-5 * C_n, ...
               'controller_6state:IFselfcheck', ...
               'IF self-check failed: R_fT(0)=%.6f vs C_dpmr=%.6f; R_fN(0)=%.6f vs C_n=%.6f', ...
               RfT0, C_dpmr, RfN0, C_n);

        % xi: per-axis sensor-noise floor offset
        xi_per_axis = (C_n / C_dpmr) * sigma2_nx / (4 * kBT);

        % --- 0C. Wall geometry ---
        w_hat_n     = params.wall.w_hat;
        pz_wall     = params.wall.pz;
        enable_wall = params.wall.enable_wall_effect > 0.5;

        % --- 0E. Wall-aware a_x[0] seeding ---
        a_nom = Ts / gamma_N;
        if enable_wall
            p0_init    = params.common.p0(:);
            h_init_um  = dot(p0_init, w_hat_n) - pz_wall;
            h_bar_init = max(h_init_um / R_radius, 1.001);
            [c_para0, c_perp0, derivs0] = wall_corrections(h_bar_init, true);
            a_x_init    = [a_nom / c_para0; a_nom / c_para0; a_nom / c_perp0];
            K_h_init    = [derivs0.K_h_para; derivs0.K_h_para; derivs0.K_h_perp];
            a_perp_init = a_nom / c_perp0;
        else
            a_x_init    = [a_nom; a_nom; a_nom];
            K_h_init    = zeros(3, 1);
            a_perp_init = a_nom;
        end

        % --- 0F. EKF state init (a_x seeded; rest zero) ---
        x_e_per_axis = zeros(6, 3);
        x_e_per_axis(5, :) = a_x_init.';        % slot 5 = a_x

        % --- 0G. Riccati P_f0 (DARE at h_init, positioning f_d = 0).
        %     D6 (spec 1b item 2): persistent stores the A-PRIORI
        %     covariance, so advance the DARE posterior one step:
        %     P_f0 = F*P_dare*F' + Q. ---
        one_minus_lc = 1 - lambda_c;
        F_e_ss = build_F_e(lambda_c, 0, 0, 0);
        H_ss   = [1 0 0 0 0 0; 0 0 0 0 1 -2];
        sigma2_dh_init  = 4 * kBT * a_perp_init;       % wall-normal thermal var
        var_da_init_vec = zeros(3, 1);
        P_per_axis = cell(3, 1);
        for ax = 1:3
            a_init_ax   = a_x_init(ax);
            var_da_init = var_da_increment_factor ...
                          * (a_init_ax * K_h_init(ax) / R_radius)^2 * sigma2_dh_init;
            var_da_init_vec(ax) = var_da_init;
            % Q33 at the f_d = 0 positioning point: thermal history
            % (d = 2 -> factor (1 + (1-lc)^2*2)) + n_x feedthrough;
            % the randgain term vanishes (f_d = 0).
            Q33_ss = 4 * kBT * a_init_ax * (1 + one_minus_lc^2 * 2) ...
                     + one_minus_lc^2 * sigma2_nx(ax);
            Q_ss = zeros(6);
            Q_ss(3, 3) = Q33_ss;
            Q_ss(5, 5) = var_da_init;
            IF_ss  = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a_init_ax, sigma2_nx(ax));
            R22_ss = K_var * IF_ss * (a_init_ax + xi_per_axis(ax))^2 ...
                     + 2 * var_da_init;                % d = 2 buffered delay at SS
            R_ss = [sigma2_nx(ax), 0; 0, R22_ss];
            P_dare = solve_dare_kf(F_e_ss, H_ss, Q_ss, R_ss);     % posterior SS
            P_per_axis{ax} = F_e_ss * P_dare * F_e_ss' + Q_ss;    % a-priori P_f0
        end

        % --- 0H. IIR prefill (three-pillar equilibrium init): seed the
        %     variance estimator at its closed-loop steady state; with
        %     pillar 2 (Riccati P_f0) this replaces the mother repo's
        %     warm-up guards, which this package removes (decision 6). ---
        dx_bar_m       = zeros(3, 1);
        sigma2_dxr_hat = 4 * kBT * a_x_init * C_dpmr + C_n * sigma2_nx;

        % --- 0I. Delay buffers ---
        pd_km1 = pd;
        pd_km2 = pd;
        f_d_km1 = zeros(3, 1);
        f_d_km2 = zeros(3, 1);
        a_hat_km1 = a_x_init;
        a_hat_km2 = a_x_init;
        var_da_ram_km1 = var_da_init_vec;
        var_da_ram_km2 = var_da_init_vec;

        % --- 0L. First call returns zeros (no f_d yet) ---
        f_d = zeros(3, 1);
        ekf_out = [a_x_init(1); a_x_init(2); a_x_init(3); 0];
        return;
    end

    % ------------------------------------------------------------------
    % [1] Measurement chain
    %     delta_x_m[k] = p_d[k-d] - p_m[k]  (d = 2, hardcoded)
    %     IIR (paper 2025 Eq.9-13): mean EWMA (a_pd) -> residual ->
    %     variance EWMA (a_cov) -> a_xm linear inversion.
    %     Wall functions at MEASURED h_bar (deterministic; avoids the
    %     bias loop of feeding the KF's own a_hat back into Q).
    % ------------------------------------------------------------------
    delta_x_m = pd_km2 - p_m;                  % 3x1 [um]

    if enable_wall
        h_bar = (dot(p_m, w_hat_n) - pz_wall) / R_radius;
    else
        h_bar = Inf;
    end
    if enable_wall && isfinite(h_bar) && h_bar > 1
        [~, c_perp_h, derivs] = wall_corrections(h_bar, true);
        K_h_axis = [derivs.K_h_para; derivs.K_h_para; derivs.K_h_perp];
        a_perp_meas = Ts / (gamma_N * c_perp_h);
    else
        % Mother-repo domain guard: degrade gracefully to the free-space
        % branch for h_bar <= 1 or non-finite (unreachable inside the
        % h_bar > 1.2 envelope; kept for fidelity).
        K_h_axis = zeros(3, 1);
        a_perp_meas = Ts / gamma_N;
    end
    sigma2_dh = 4 * kBT * a_perp_meas;         % wall-normal thermal var (shared 3 axes)

    dx_bar_m_new = (1 - a_pd) * dx_bar_m + a_pd * delta_x_m;
    dx_r = delta_x_m - dx_bar_m_new;
    sigma2_dxr_hat_new = (1 - a_cov) * sigma2_dxr_hat + a_cov * dx_r.^2;
    a_xm = (sigma2_dxr_hat_new - C_n * sigma2_nx) / (C_dpmr * 4 * kBT);   % 3x1 [um/pN]

    % ------------------------------------------------------------------
    % [2] Control law (Eq.17 ACTIVE form, Vpersonal p.2)
    %     f_d = a_hat^-1 { Delta_x_d[k;d] + (1-lc)[dx_m - sum a_hat[k-i]*f_d[k-i]]
    %                      - delta_x_D^d }
    %     Delta_x_d[k;d] = pd[k+1] - lc*pd[k] - (1-lc)*pd[k-d]
    %     Past forces weighted by PAST gain estimates a_hat[k-i].
    %     D6 semantics: a_hat and xD_comb are A-PRIORI estimates (no
    %     current-step measurement; one extra Phi map vs the posterior).
    % ------------------------------------------------------------------
    a_hat   = x_e_per_axis(5, :).';            % 3x1 [um/pN]  a-priori a_hat[k|k-1]
    xD_comb = x_e_per_axis(4, :).';            % 3x1 [um]     delta_x_D^d, a-priori

    pd_kp1 = pd + del_pd;                      % p_d[k+1]
    one_minus_lc = 1 - lambda_c;
    sum_a_fd_past = a_hat_km1 .* f_d_km1 + a_hat_km2 .* f_d_km2;   % d = 2
    inv_a_hat = 1 ./ a_hat;
    f_d = inv_a_hat .* (pd_kp1 - lambda_c * pd - one_minus_lc * pd_km2 ...
                        + one_minus_lc * delta_x_m ...
                        - one_minus_lc * sum_a_fd_past ...
                        - xD_comb);

    % ------------------------------------------------------------------
    % [3]-[10] EKF per axis -- paper predictor form + Joseph (D6, spec 1b)
    %     Persistent (x_e_per_axis, P_per_axis) hold the A-PRIORI pair
    %     (x_hat[k|k-1], P_f[k]). NO guards: H is always the full 2x2
    %     map; both measurements always enter (decision 6).
    % ------------------------------------------------------------------
    H = [1 0 0 0 0 0; 0 0 0 0 1 -2];           % y_1 -> delta_x_1; y_2 -> a_x - 2*delta_a_x
    var_da_ram = zeros(3, 1);

    for ax = 1:3
        x_curr  = x_e_per_axis(:, ax);         % a-priori x_hat[k|k-1]
        P_f     = P_per_axis{ax};              % forecast covariance P_f[k]
        a_hat_i = a_hat(ax);

        % --- [3] Innovations against the stored a-priori ---
        innov = [delta_x_m(ax); a_xm(ax)] - H * x_curr;

        % --- [4] R[k]: R11 = sensor spec; R22 = intrinsic + buffered
        %     delay sum {1,1} (r_2 = n_a - Sigma delta_a_ram[k-i],
        %     Vpersonal p.3 -- deliberately NOT the {4,1} weights of the
        %     Q33 randgain accumulation; see kf_canonical_spec sec 6) ---
        IF_eff_i = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a_hat_i, sigma2_nx(ax));
        R22_i = K_var * IF_eff_i * (a_hat_i + xi_per_axis(ax))^2 ...
                + var_da_ram_km1(ax) + var_da_ram_km2(ax);
        R_i = [sigma2_nx(ax), 0; 0, R22_i];

        % --- [5] Gain L = P_f*H' / (H*P_f*H' + R)                (Eq.19) ---
        S_inn = H * P_f * H' + R_i;
        S_inn = 0.5 * (S_inn + S_inn');
        L_kf  = (P_f * H') / S_inn;

        % --- [6] State: x+ = Phi_map(x_curr) + L*innov (Eq.16 merged).
        %     Phi (Vpersonal p.4) keeps Row3 = lc ONLY -- the couplings
        %     (-1, -F_dx, dF_dx) multiply zero-mean estimation errors and
        %     belong to F_e (covariance), not to the mean prediction. ---
        x_plus = [x_curr(2); ...
                  x_curr(3); ...
                  lambda_c * x_curr(3); ...
                  x_curr(4); ...
                  x_curr(5) + x_curr(6); ...
                  x_curr(6)] + L_kf * innov;

        % --- [7] Posterior covariance, Joseph form (Eq.20 -> Joseph) ---
        ImLH   = eye(6) - L_kf * H;
        P_post = ImLH * P_f * ImLH' + L_kf * R_i * L_kf';
        P_post = 0.5 * (P_post + P_post');

        % --- [8] F_e[k]: time-varying Row 3 from f_d[k] (just computed)
        %     and the PRE-shift buffers f_d[k-1], f_d[k-2] -- paper-strict
        %     timing for the forecast P[k] -> P_f[k+1] ---
        F_1_i = f_d_km1(ax) + f_d_km2(ax);
        F_2_i = f_d_km1(ax) + 2 * f_d_km2(ax);
        F_e = build_F_e(lambda_c, f_d(ax), F_1_i, F_2_i);

        % --- [9] Q[k] (D3 three-component Q33 + closed-form Q55):
        %     thermal : 4kBT*( a_hat[k] + (1-lc)^2*(a_hat[k-1]+a_hat[k-2]) )
        %     randgain: (1-lc)^2 * Sigma (d+1-j)^2 * f_d[k-j]^2
        %               * var(da_ram[k-j])          (weights {4,1}, d = 2)
        %     n_x     : (1-lc)^2 * sigma2_nx
        %     Q55 = var(delta_a_ram) = [2/(1+lc)]*(a_hat*K_h/R)^2*sigma2_dh ---
        var_da_ram(ax) = var_da_increment_factor ...
                         * (a_hat_i * K_h_axis(ax) / R_radius)^2 * sigma2_dh;
        Q33_thermal  = 4 * kBT * (a_hat_i + one_minus_lc^2 * (a_hat_km1(ax) + a_hat_km2(ax)));
        Q33_randgain = one_minus_lc^2 * ( 4 * f_d_km1(ax)^2 * var_da_ram_km1(ax) ...
                                        + 1 * f_d_km2(ax)^2 * var_da_ram_km2(ax) );
        Q33_nx = one_minus_lc^2 * sigma2_nx(ax);
        Q_i = zeros(6);
        Q_i(3, 3) = Q33_thermal + Q33_randgain + Q33_nx;
        Q_i(5, 5) = var_da_ram(ax);

        % --- [10] Forecast (Eq.21): P_f[k+1] = F_e[k]*P[k]*F_e' + Q[k] ---
        P_f_plus = F_e * P_post * F_e' + Q_i;
        P_f_plus = 0.5 * (P_f_plus + P_f_plus');

        x_e_per_axis(:, ax) = x_plus;          % next a-priori x_hat[k+1|k]
        P_per_axis{ax} = P_f_plus;             % next forecast P_f[k+1]
    end

    % ------------------------------------------------------------------
    % [11] Buffer shifts + output
    % ------------------------------------------------------------------
    pd_km2 = pd_km1;
    pd_km1 = pd;
    f_d_km2 = f_d_km1;
    f_d_km1 = f_d;
    a_hat_km2 = a_hat_km1;
    a_hat_km1 = a_hat;
    var_da_ram_km2 = var_da_ram_km1;
    var_da_ram_km1 = var_da_ram;
    dx_bar_m = dx_bar_m_new;
    sigma2_dxr_hat = sigma2_dxr_hat_new;

    a_hat_out = x_e_per_axis(5, :).';          % a-priori x_hat[k+1|k] (D6 logs)
    if enable_wall
        h_bar_out = h_bar;
    else
        h_bar_out = 0;
    end
    ekf_out = [a_hat_out(1); a_hat_out(2); a_hat_out(3); h_bar_out];
end


%% =================== Local functions ===================

function F_e = build_F_e(lambda_c, f_d_i, F_1_i, F_2_i)
%BUILD_F_E  6x6 error-dynamics matrix (per axis), Vpersonal p.5.
%   Row 3 = [0 0 lc -1 -F_dx dF_dx]
%       F_dx  = f_d[k] + (1-lc)*F_1   (col 5, a_x)
%       dF_dx = (1-lc)*F_2            (col 6, delta_a_x)
%   F_1 = sum_{i=1..d} f_d[k-i], F_2 = sum_{i=1..d} i*f_d[k-i].
%   Col 4 = -1 is d-INDEPENDENT in the combined-disturbance (delta_x_D^d)
%   coordinates -- the 7-state value -(1+d(1-lc)) belongs to the separate
%   (x_D, delta_x_D) coordinates. Do not "fix" it.
    one_minus_lc = 1 - lambda_c;
    Fe3_a  = -f_d_i - one_minus_lc * F_1_i;     % col 5 (a_x): -F_dx
    Fe3_da = one_minus_lc * F_2_i;              % col 6 (delta_a_x): dF_dx
    F_e = [0 1 0        0  0      0; ...
           0 0 1        0  0      0; ...
           0 0 lambda_c -1 Fe3_a  Fe3_da; ...
           0 0 0        1  0      0; ...
           0 0 0        0  1      1; ...
           0 0 0        0  0      1];
end


function IF = if_eff_eval(IF_abc, C_dpmr, C_n, kBT, a, sigma2_nx)
%IF_EFF_EVAL  Exact color-inflation factor IF_eff for R22 (R22_derivation S4-S6).
%   IF = 1 + 2*(sxT^2*A + 2*sxT*snx*B + snx^2*C) / (C_dpmr*sxT + C_n*snx)^2,
%   sxT = 4*kBT*a (thermal residual variance), snx = sigma2_nx, IF_abc=[A;B;C]
%   the offline s-weighted autocorrelation sums. Per-axis, time-varying via a.
    sxT = 4 * kBT * a;
    num = sxT^2 * IF_abc(1) + 2 * sxT * sigma2_nx * IF_abc(2) + sigma2_nx^2 * IF_abc(3);
    den = (C_dpmr * sxT + C_n * sigma2_nx)^2;
    IF  = 1 + 2 * num / den;
end


function [A, B, C, RfT0, RfN0] = compute_if_abc(lc, apd, a_cov)
%COMPUTE_IF_ABC  s-weighted autocorrelation sums for the exact IF_eff
%   (R22_derivation S4-S6), via F_T/F_N impulse responses (toolbox-free).
%
%   F_T = (1-apd)(1-q)q^3[1+(1-lc)q+(1-lc)q^2] / [(1-(1-apd)q)(1-lc q)],  q=z^-1
%   F_N = (1-apd)(1-q)^2[1+(1-lc)q+(1-lc)q^2] / [same den]   (z^3 cancels q^3)
%
%   Returns  A = sum_{tau>=1} R_fT(tau)^2 s^tau,  B = sum R_fT(tau) R_fN(tau) s^tau,
%   C = sum R_fN(tau)^2 s^tau  (s = 1-a_cov), plus RfT0=R_fT(0)=C_dpmr and
%   RfN0=R_fN(0)=C_n for an independent self-check. Poles (1-apd, lc) < 1, so the
%   impulse responses decay geometrically; N/Tmax give machine precision.
    s = 1 - a_cov;
    q1 = [1, -1]; q3 = [0, 0, 0, 1]; thnum = [1, (1 - lc), (1 - lc)];
    numFT = (1 - apd) * conv(conv(q1, q3), thnum);
    numFN = (1 - apd) * conv(conv(q1, q1), thnum);
    den   = conv([1, -(1 - apd)], [1, -lc]);
    N = 8000;
    imp = [1; zeros(N - 1, 1)];
    hFT = filter(numFT, den, imp);
    hFN = filter(numFN, den, imp);
    Tmax = 600;
    RfT0 = sum(hFT .^ 2);
    RfN0 = sum(hFN .^ 2);
    A = 0; B = 0; C = 0;
    for t = 1:Tmax
        RfT = sum(hFT(1:end - t) .* hFT(1 + t:end));
        RfN = sum(hFN(1:end - t) .* hFN(1 + t:end));
        st  = s ^ t;
        A = A + RfT ^ 2 * st;
        B = B + RfT * RfN * st;
        C = C + RfN ^ 2 * st;
    end
end


function P_post = solve_dare_kf(F, H, Q, R)
%SOLVE_DARE_KF  Discrete-time KF Riccati steady-state (fixed-point iteration).
%   Returns the POSTERIOR steady-state covariance. No toolbox dependency.
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
