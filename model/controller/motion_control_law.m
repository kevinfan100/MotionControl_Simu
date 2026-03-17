function [f_d, ekf_out] = motion_control_law(del_pd, pd, p_m, params)
%MOTION_CONTROL_LAW Discrete-time motion control with wall-effect compensation
%
%   [f_d, ekf_out] = motion_control_law(del_pd, pd, p_m, params)
%
%   Implements feedforward trajectory tracking with known-geometry lambda
%   computation for wall-effect drag compensation.
%
%   Full 23-state EKF with structural stability fixes:
%     - Leaky integrator for rate states (rho_f) prevents drift
%     - Kalman gain decoupling prevents position-parameter cross-talk
%     - Adaptive g_cov corrects for EMA variance reduction
%     - State-dependent R uses chi-squared noise model
%     - Theta epsilon=0.05 prevents noise amplification at low anisotropy
%
%   Inputs:
%       del_pd - Trajectory increment p_d[k+1] - p_d[k] [3x1, um]
%       pd     - Current desired position p_d[k] [3x1, um]
%       p_m    - Measured position p_m[k] [3x1, um]
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       f_d     - Control force f_d[k] [3x1, pN]
%       ekf_out - Diagnostic [4x1]: [lamda_hat(2x1); theta_hat(2x1)]

    % Check if control is enabled
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = [1; 1; 0; 0];
        return;
    end

    % --- Persistent declarations ---
    persistent initialized
    persistent gamma_N Ts lambda_c a_pd a_prd a_cov epsilon g_cov sigma2_deltaXT
    persistent V V_T                                                   % Rotation [k]
    persistent V_del_p1_hat V_del_p2_hat V_del_p3_hat                 % Pos err est. (wall) [k]
    persistent V_d_hat V_del_d_hat                                     % Disturb est. (wall) [k]
    persistent lamda_hat del_lamda_hat                                  % Drag ratio [k]
    persistent theta_hat del_theta_hat                                  % Wall orient. [k]
    persistent del_p1_hat del_p2_hat del_p3_hat d_hat del_d_hat        % World copies [k]
    persistent pd_k1 pd_k2                                              % Delay buffers
    persistent del_pmd_k1 del_pmrd_k1                                  % EMA states [k-1]
    persistent Pf H R                                                  % Kalman filter
    persistent alpha_f rho_f                                               % forgetting/leaky factors
    persistent step_count warmup_steps                                    % EMA settling gate
    persistent g_cov_sq                                                   % adaptive g_cov

    %% Step [0]: Parameters & Initialization
    if isempty(initialized)
        initialized = true;

        % 0A. Extract constants from params
        gamma_N = params.ctrl.gamma;
        Ts = params.ctrl.Ts;
        lambda_c = params.ctrl.lambda_c;
        a_pd = params.ctrl.a_pd;
        a_prd = params.ctrl.a_prd;
        a_cov = params.ctrl.a_cov;
        epsilon = params.ctrl.epsilon;
        sigma2_deltaXT = params.ctrl.sigma2_deltaXT;
        g_cov = params.ctrl.g_cov;

        % 0B. Rotation (theta[0]=[0;0] -> V=I)
        V = eye(3); V_T = eye(3);

        % 0C. EKF states [k=0] -- all zeros except lamda_hat=[1;1]
        V_del_p1_hat = zeros(3,1); V_del_p2_hat = zeros(3,1); V_del_p3_hat = zeros(3,1);
        V_d_hat = zeros(3,1); V_del_d_hat = zeros(3,1);
        lamda_hat = [1;1]; del_lamda_hat = [0;0];
        theta_hat = [0;0]; del_theta_hat = [0;0];
        del_p1_hat = zeros(3,1); del_p2_hat = zeros(3,1); del_p3_hat = zeros(3,1);
        d_hat = zeros(3,1); del_d_hat = zeros(3,1);

        % 0D. Delay buffers (init to current pd input)
        pd_k1   = pd;
        pd_k2   = pd;

        % 0E. EMA states (init to zero)
        del_pmd_k1  = zeros(3,1);
        del_pmrd_k1 = zeros(3,1);

        % 0F. Kalman filter
        H = zeros(7, 23);
        H(1:3, 1:3) = eye(3);
        H(4:5, 16:17) = eye(2);
        H(6:7, 20:21) = eye(2);

        Pf_diag = [1e-2*ones(1,9), 1e-4*ones(1,6), ...
                   1e-4*ones(1,4), 1e-4*ones(1,4)];
        Pf = diag(Pf_diag);

        % Adaptive g_cov: self-corrects for EMA variance reduction
        g_cov_sq = g_cov * g_cov;

        R = zeros(7,7);
        R(1:3, 1:3) = g_cov_sq * eye(3);
        R(4:5, 4:5) = a_cov^2 * eye(2);
        R(6:7, 6:7) = a_cov^2 * eye(2);

        % 0G. Forgetting factor and leaky factor
        alpha_f = params.ctrl.alpha_f;
        rho_f = params.ctrl.rho_f;

        % 0H. EMA measurement validity gate
        step_count = 0;
        warmup_steps = ceil(3/a_pd + 3/a_prd);

        f_d = zeros(3,1);
        ekf_out = [1; 1; 0; 0];
        return;
    end

    step_count = step_count + 1;

    %% Step [1]-[2]: Coordinate Transform & Control Law
    V_del_pd = V_T * del_pd;

    lam_T_safe = max(lamda_hat(1), 0.1);
    lam_N_safe = max(lamda_hat(2), 0.1);

    del_u = (1/lam_T_safe) * (V_del_pd(1) + (1-lambda_c)*V_del_p3_hat(1) - V_d_hat(1));
    del_v = (1/lam_T_safe) * (V_del_pd(2) + (1-lambda_c)*V_del_p3_hat(2) - V_d_hat(2));
    del_w = (1/lam_N_safe) * (V_del_pd(3) + (1-lambda_c)*V_del_p3_hat(3) - V_d_hat(3));

    fu = (gamma_N / Ts) * del_u;
    fv = (gamma_N / Ts) * del_v;
    fw = (gamma_N / Ts) * del_w;

    f_max = 1.0;                                                   % pN, actuator limit
    fu = max(min(fu, f_max), -f_max);
    fv = max(min(fv, f_max), -f_max);
    fw = max(min(fw, f_max), -f_max);

    f_d = V * [fu; fv; fw];

    %% Step [3]: Measurement Processing

    del_pm = pd_k2 - p_m;
    V_del_pm = V_T * del_pm;

    del_pmd  = (1 - a_pd)  * del_pmd_k1  + a_pd  * del_pm;
    del_pmr  = del_pm - del_pmd;
    del_pmrd = (1 - a_prd) * del_pmrd_k1 + a_prd * del_pmr;
    del_pmrr = del_pmr - del_pmrd;

    % Adaptive g_cov: track running variance of del_pmrr
    del_pmrr_sq = mean(del_pmrr.^2);
    g_cov_sq = (1 - a_cov) * g_cov_sq + a_cov * del_pmrr_sq;
    g_cov = sqrt(max(g_cov_sq, 1e-20));

    R(1:3, 1:3) = g_cov_sq * eye(3);

    del_pnr  = (1 / g_cov) * del_pmrr;
    V_del_nr = V_T * del_pnr;
    del_unr = V_del_nr(1); del_vnr = V_del_nr(2); del_wnr = V_del_nr(3);

    lamda_m = (1 - a_cov) * lamda_hat + a_cov * [(del_unr^2 + del_vnr^2)/2; del_wnr^2];

    del_lamda_m = lamda_m(1) - lamda_m(2);
    if abs(del_lamda_m / max(lamda_m(1), 0.01)) < epsilon
        theta_m = theta_hat;
    else
        theta_m = theta_hat + a_cov * [del_vnr*del_wnr / del_lamda_m;
                                       -del_wnr*del_unr / del_lamda_m];
    end

    % State-dependent R for lambda/theta
    % Base chi-squared variance scaled by 100x safety factor to match
    % the effective smoothing rate that stabilizes closed-loop (~a_cov^2).
    R_scale = 100;
    lam_safe = max(lamda_hat, [0.1; 0.1]);
    R(4,4) = R_scale * a_cov^2 * lam_safe(1)^2;
    R(5,5) = R_scale * a_cov^2 * 2 * lam_safe(2)^2;
    del_lam = max(abs(lamda_hat(1) - lamda_hat(2)), 0.01);
    R(6,6) = R_scale * a_cov^2 * lam_safe(1) * lam_safe(2) / del_lam^2;
    R(7,7) = R(6,6);

    %% Step [4]: Error Signals
    err_p     = V_del_pm - V_del_p1_hat;
    err_lamda = lamda_m  - lamda_hat;
    err_theta = theta_m  - theta_hat;
    err = [err_p; err_lamda; err_theta];

    %% Step [5]: Kalman Gain
    idx_obs = [1:3, 16:17, 20:21];
    Pf_HT   = Pf(:, idx_obs);                                    % 23x7
    HPf_HT  = Pf(idx_obs, idx_obs);                              % 7x7
    S       = HPf_HT + R;                                        % 7x7
    S       = (S + S') / 2;                                       % enforce symmetry
    L       = Pf_HT / S;                                         % 23x7 (mldivide)

    % NaN guard
    if any(isnan(L(:)))
        L = zeros(23, 7);
    end

    % Staged activation: gradually enable state groups
    % Phase durations scaled by warmup_steps (60 for a_pd=a_prd=0.1)
    if step_count <= 8 * warmup_steps
        L = zeros(23, 7);                                         % Phase 0: EMA settling (long)
    elseif step_count <= 12 * warmup_steps
        L(10:23, :) = 0;                                         % Phase 1: position only
    elseif step_count <= 16 * warmup_steps
        L(16:23, :) = 0;                                         % Phase 2: +disturbance
    end
    % Phase 3+: full EKF (with decoupling below)

    % Decouple: lambda/theta only respond to their own measurements
    L(16:19, [1:3, 6:7]) = 0;                                    % lambda: keep only cols 4:5
    L(20:23, [1:3, 4:5]) = 0;                                    % theta: keep only cols 6:7

    %% Step [6]: State Update (all 9 groups)

    % [6a] Wall-frame EKF states
    V_del_p1_hat_kA1  = V_del_p2_hat                    + L(1:3,:)   * err;
    V_del_p2_hat_kA1  = V_del_p3_hat                    + L(4:6,:)   * err;
    V_del_p3_hat_kA1  = lambda_c * V_del_p3_hat         + L(7:9,:)   * err;
    V_d_hat_kA1       = V_d_hat + V_del_d_hat           + L(10:12,:) * err;
    V_del_d_hat_kA1   = V_del_d_hat                     + L(13:15,:) * err;
    lamda_hat_kA1     = lamda_hat + del_lamda_hat        + L(16:17,:) * err;
    del_lamda_hat_kA1 = del_lamda_hat                    + L(18:19,:) * err;
    theta_hat_kA1     = theta_hat + del_theta_hat        + L(20:21,:) * err;
    del_theta_hat_kA1 = del_theta_hat                    + L(22:23,:) * err;

    % [6b] Convert to world frame
    del_p1_hat_kA1 = V * V_del_p1_hat_kA1;
    del_p2_hat_kA1 = V * V_del_p2_hat_kA1;
    del_p3_hat_kA1 = V * V_del_p3_hat_kA1;
    d_hat_kA1      = V * V_d_hat_kA1;
    del_d_hat_kA1  = V * V_del_d_hat_kA1;

    %% Step [7]: Posterior Covariance (Joseph form)
    ILH = eye(23) - L * H;
    P = ILH * Pf * ILH' + L * R * L';

    %% Step [8]: F[k] Update
    del_lamda_hat_scalar = lamda_hat_kA1(1) - lamda_hat_kA1(2);
    dL = del_lamda_hat_scalar;

    G_lamda = [del_u, 0; del_v, 0; 0, del_w];
    G_theta = [0, -dL*del_w; dL*del_w, 0; dL*del_v, -dL*del_u];

    F = zeros(23, 23);
    F(1:3,   4:6)   = eye(3);
    F(4:6,   7:9)   = eye(3);
    F(7:9,   7:9)   = eye(3);
    F(7:9,   10:12) = -eye(3);
    F(7:9,   16:17) = -G_lamda;
    F(7:9,   20:21) = -G_theta;
    F(10:12, 10:12) = eye(3);
    F(10:12, 13:15) = eye(3);
    F(13:15, 13:15) = rho_f * eye(3);                              % Row 5 (leaky)
    F(16:17, 16:17) = eye(2);
    F(16:17, 18:19) = eye(2);
    F(18:19, 18:19) = rho_f * eye(2);                              % Row 7 (leaky)
    F(20:21, 20:21) = eye(2);
    F(20:21, 22:23) = eye(2);
    F(22:23, 22:23) = rho_f * eye(2);                              % Row 9 (leaky)

    %% Step [9]: Q[k] Update
    Q = zeros(23, 23);
    Q33 = sigma2_deltaXT * diag([lamda_hat_kA1(1), lamda_hat_kA1(1), lamda_hat_kA1(2)]);
    Q(7:9, 7:9)     = Q33;
    Q(10:12, 10:12) = a_pd * a_prd * Q33;
    Q(13:15, 13:15) = a_pd * a_prd * Q33;
    Q66 = a_cov * sigma2_deltaXT * diag([lamda_hat_kA1(1), lamda_hat_kA1(2)]);
    Q(16:17, 16:17) = Q66;
    Q(18:19, 18:19) = Q66;
    Q(20:21, 20:21) = 0.01 * Q66;
    Q(22:23, 22:23) = 0.01 * Q66;

    %% Step [10]: Forecast Covariance (with forgetting factor)
    Pf = (1/alpha_f) * (F * P * F') + Q;
    Pf = (Pf + Pf') / 2;

    % Enforce Pf bounds while preserving positive-definiteness.
    % Scale entire matrix if max diagonal exceeds ceiling (preserves PD).
    % Then enforce floor on diagonals.
    pf_max = max(diag(Pf));
    if pf_max > 1e0
        Pf = Pf * (1e0 / pf_max);
    end
    pf_diag = diag(Pf);
    pf_diag = max(pf_diag, 1e-12);
    Pf = Pf - diag(diag(Pf)) + diag(pf_diag);

    %% Step [11]: V[k] Update & State Re-projection
    cx = cos(theta_hat_kA1(1)); sx = sin(theta_hat_kA1(1));
    cy = cos(theta_hat_kA1(2)); sy = sin(theta_hat_kA1(2));
    V_new = [ cy,  sy*sx, sy*cx;
              0,   cx,    -sx  ;
             -sy,  cy*sx, cy*cx];
    V_T_new = V_new';

    V_del_p1_hat_new = V_T_new * del_p1_hat_kA1;
    V_del_p2_hat_new = V_T_new * del_p2_hat_kA1;
    V_del_p3_hat_new = V_T_new * del_p3_hat_kA1;
    V_d_hat_new      = V_T_new * d_hat_kA1;
    V_del_d_hat_new  = V_T_new * del_d_hat_kA1;

    % --- Persistent Shifts ---
    V = V_new;  V_T = V_T_new;
    V_del_p1_hat = V_del_p1_hat_new;
    V_del_p2_hat = V_del_p2_hat_new;
    V_del_p3_hat = V_del_p3_hat_new;
    V_d_hat      = V_d_hat_new;
    V_del_d_hat  = V_del_d_hat_new;

    lamda_hat     = max(min(lamda_hat_kA1, [1.5; 1.5]), [0.05; 0.05]);
    del_lamda_hat = del_lamda_hat_kA1;
    theta_max = pi/6;
    theta_hat     = max(min(theta_hat_kA1, theta_max), -theta_max);
    del_theta_hat = del_theta_hat_kA1;

    del_p1_hat = del_p1_hat_kA1;
    del_p2_hat = del_p2_hat_kA1;
    del_p3_hat = del_p3_hat_kA1;
    d_hat      = d_hat_kA1;
    del_d_hat  = del_d_hat_kA1;

    pd_k2 = pd_k1;
    pd_k1 = pd;

    del_pmd_k1  = del_pmd;
    del_pmrd_k1 = del_pmr;

    ekf_out = [lamda_hat; theta_hat];

end
