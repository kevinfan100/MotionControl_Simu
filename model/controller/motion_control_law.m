function f_d = motion_control_law(pd_A1, p_m, params)
%MOTION_CONTROL_LAW Discrete-time motion control with EKF estimation
%
%   f_d = motion_control_law(pd_A1, p_m, params)
%
%   Implements feedforward trajectory tracking with EKF-based estimation
%   of wall-effect parameters (lambda, theta) and disturbance rejection.
%   Reference: Estimation_Control.pdf
%
%   Inputs:
%       pd_A1  - Desired position p_d[k+1] from trajectory generator [3x1, um]
%       p_m    - Measured position p_m[k] [3x1, um]
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       f_d    - Control force f_d[k] [3x1, pN]

    % Check if control is enabled
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        return;
    end

    %% Persistent declarations
    persistent initialized k_step
    persistent gamma_N Ts lambda_c a_pd a_prd a_cov epsilon g_cov sigma2_deltaXT
    persistent V V_T                                                   % Rotation [k]
    persistent V_del_p1_hat V_del_p2_hat V_del_p3_hat                 % Pos err est. (wall) [k]
    persistent V_d_hat V_del_d_hat                                     % Disturb est. (wall) [k]
    persistent lamda_hat del_lamda_hat                                  % Drag ratio [k]
    persistent theta_hat del_theta_hat                                  % Wall orient. [k]
    persistent del_p1_hat del_p2_hat del_p3_hat d_hat del_d_hat        % World copies [k]
    persistent pd pd_k1 pd_k2                                          % Delay buffers
    persistent del_pmd_k1 del_pmrd_k1                                  % EMA states [k-1]
    persistent P_f H R                                                 % Kalman filter

    %% Part 0: Parameters & Initialization
    if isempty(initialized)
        initialized = true;
        k_step = 0;

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

        % 0D. Delay buffers (init all to first input)
        pd      = pd_A1;  % p_d[k]
        pd_k1   = pd_A1;  % p_d[k-1]
        pd_k2   = pd_A1;  % p_d[k-2]

        % 0E. EMA states (PDF p.2: delta_pmd[k-1]=0, delta_pmr[k-1]=0)
        del_pmd_k1  = zeros(3,1);   % delta_pmd[k-1]
        del_pmrd_k1 = zeros(3,1);   % delta_pmr[k-1]

        % 0F. Kalman filter
        H = zeros(7, 23);
        H(1:3, 1:3) = eye(3);       % selects del_p1
        H(4:5, 16:17) = eye(2);     % selects lambda
        H(6:7, 20:21) = eye(2);     % selects theta

        P_f = 10 * eye(23);         % Large initial uncertainty

        g2_cov = g_cov * g_cov;
        R = zeros(7,7);
        R(1:3, 1:3) = g2_cov * eye(3);
        R(4:5, 4:5) = a_cov^2 * g2_cov * eye(2);
        R(6:7, 6:7) = a_cov^2 * g2_cov * eye(2);

        f_d = zeros(3,1);
        return;
    end

    k_step = k_step + 1;
    % NOTE: pd_A1 = p_d[k+1], pd = p_d[k], p_m = p_m[k]

    %% Part 1: Measurement Processing

    % 1A. 2-step delay position error (world frame)
    del_pm = pd_k2 - p_m;                          % p_d[k-2] - p_m[k]

    % 1B. Transform to wall frame
    V_del_pm = V_T * del_pm;                        % [del_um; del_vm; del_wm]

    % 1C. EMA decomposition (world frame, 3 layers)
    del_pmd   = (1 - a_pd)  * del_pmd_k1  + a_pd  * del_pm;    % Deterministic
    del_pmr = del_pm - del_pmd;                                % Random
    del_pmrd  = (1 - a_prd) * del_pmrd_k1 + a_prd * del_pmr; % Random-deterministic
    del_pmrr  = del_pmr - del_pmrd;                            % Residual

    % 1D. Normalized residual
    del_pnr  = (1 / g_cov) * del_pmrr;             % world frame
    V_del_nr = V_T * del_pnr;                       % wall frame
    del_unr = V_del_nr(1); del_vnr = V_del_nr(2); del_wnr = V_del_nr(3);

    % 1E. Lambda measurement
    lamda_m = (1 - a_cov) * lamda_hat + a_cov * [(del_unr^2 + del_vnr^2)/2; del_wnr^2];

    % 1F. Theta measurement (conditional on anisotropy)
    del_lamda_m = lamda_m(1) - lamda_m(2);
    if abs(del_lamda_m / lamda_m(1)) < epsilon
        theta_m = theta_hat;                         % isotropic -> keep previous
    else
        theta_m = theta_hat + a_cov * [del_vnr*del_wnr / del_lamda_m;
                                       -del_wnr*del_unr / del_lamda_m];
    end

    %% Part 2: EKF Update

    % 2A. Innovation errors (wall frame for position, scalar for lambda/theta)
    err_p     = V_del_pm - V_del_p1_hat;            % 3x1
    err_lamda = lamda_m  - lamda_hat;                % 2x1
    err_theta = theta_m  - theta_hat;                % 2x1
    err = [err_p; err_lamda; err_theta];             % 7x1

    % 2B. Kalman gain (exploiting H's sparse structure, PDF p.5)
    idx_obs = [1:3, 16:17, 20:21];
    P_f_HT  = P_f(:, idx_obs);                      % 23x7  (= P_f * H')
    HP_f_HT = P_f(idx_obs, idx_obs);                % 7x7   (= H * P_f * H')
    G       = inv(HP_f_HT + R);                      % 7x7
    L       = P_f_HT * G;                           % 23x7

    % 2C. State update: [k] -> [k+1] (PDF p.5, stored as _kA1)
    V_del_p1_hat_kA1  = V_del_p2_hat                    + L(1:3,:)   * err;
    V_del_p2_hat_kA1  = V_del_p3_hat                    + L(4:6,:)   * err;
    V_del_p3_hat_kA1  = lambda_c * V_del_p3_hat         + L(7:9,:)   * err;
    V_d_hat_kA1       = V_d_hat + V_del_d_hat           + L(10:12,:) * err;
    V_del_d_hat_kA1   = V_del_d_hat                     + L(13:15,:) * err;
    lamda_hat_kA1     = lamda_hat + del_lamda_hat        + L(16:17,:) * err;
    del_lamda_hat_kA1 = del_lamda_hat                    + L(18:19,:) * err;
    theta_hat_kA1     = theta_hat + del_theta_hat        + L(20:21,:) * err;
    del_theta_hat_kA1 = del_theta_hat                    + L(22:23,:) * err;

    % 2D. Posterior covariance
    P = (eye(23) - L * H) * P_f;

    %% Part 3: Coordinate Update & Control Law (NO persistent writes)

    % 3A. World-frame copies (using current V[k])
    del_p1_hat_kA1 = V * V_del_p1_hat_kA1;
    del_p2_hat_kA1 = V * V_del_p2_hat_kA1;
    del_p3_hat_kA1 = V * V_del_p3_hat_kA1;
    d_hat_kA1      = V * V_d_hat_kA1;
    del_d_hat_kA1  = V * V_del_d_hat_kA1;

    % 3B. Compute new V from theta_hat_kA1 (local, NOT persistent)
    cx = cos(theta_hat_kA1(1)); sx = sin(theta_hat_kA1(1));
    cy = cos(theta_hat_kA1(2)); sy = sin(theta_hat_kA1(2));
    V_new  = [cy, sy*sx, sy*cx; 0, cx, -sx; -sy, cy*sx, cy*cx];
    V_T_new = V_new';

    % 3C. Re-transform to new wall frame (local temps)
    V_del_p1_hat_new = V_T_new * del_p1_hat_kA1;
    V_del_p2_hat_new = V_T_new * del_p2_hat_kA1;
    V_del_p3_hat_new = V_T_new * del_p3_hat_kA1;
    V_d_hat_new      = V_T_new * d_hat_kA1;
    V_del_d_hat_new  = V_T_new * del_d_hat_kA1;

    % 3D. Control law (PDF p.3, all [k+1] values via temps)
    delta_pd = pd_A1 - pd;                           % p_d[k+1] - p_d[k], world frame
    V_del_pd = V_T_new * delta_pd;                  % wall frame (new V)

    del_u = (1/lamda_hat_kA1(1)) * (V_del_pd(1) + (1-lambda_c)*V_del_p3_hat_new(1) - V_d_hat_new(1));
    del_v = (1/lamda_hat_kA1(1)) * (V_del_pd(2) + (1-lambda_c)*V_del_p3_hat_new(2) - V_d_hat_new(2));
    del_w = (1/lamda_hat_kA1(2)) * (V_del_pd(3) + (1-lambda_c)*V_del_p3_hat_new(3) - V_d_hat_new(3));

    fu = (gamma_N / Ts) * del_u;
    fv = (gamma_N / Ts) * del_v;
    fw = (gamma_N / Ts) * del_w;

    f_d = V_new * [fu; fv; fw];                     % world frame output

    %% Part 4: State Iteration + ALL Persistent Shifts

    % 4A. Build G_lamda, G_theta (PDF p.6, uses [k+1] lamda_hat)
    del_lamda_hat_scalar = lamda_hat_kA1(1) - lamda_hat_kA1(2);
    G_lamda = [del_u, 0; del_v, 0; 0, del_w];             % 3x2
    G_theta = [0,                        -del_lamda_hat_scalar*del_w;
               del_lamda_hat_scalar*del_w, 0;
               del_lamda_hat_scalar*del_v, -del_lamda_hat_scalar*del_u];

    % 4B. Build F[k] (23x23, PDF p.6)
    F = zeros(23, 23);
    F(1:3, 4:6)     = eye(3);                       % Row 1
    F(4:6, 7:9)     = eye(3);                       % Row 2
    F(7:9, 7:9)     = eye(3);                       % Row 3
    F(7:9, 10:12)   = -eye(3);
    F(7:9, 16:17)   = -G_lamda;
    F(7:9, 20:21)   = -G_theta;
    F(10:12, 10:12) = eye(3);                       % Row 4
    F(10:12, 13:15) = eye(3);
    F(13:15, 13:15) = eye(3);                       % Row 5
    F(16:17, 16:17) = eye(2);                       % Row 6
    F(16:17, 18:19) = eye(2);
    F(18:19, 18:19) = eye(2);                       % Row 7
    F(20:21, 20:21) = eye(2);                       % Row 8
    F(20:21, 22:23) = eye(2);
    F(22:23, 22:23) = eye(2);                       % Row 9

    % 4C. Build Q[k] (23x23 block diagonal, PDF p.7, uses [k+1] lamda_hat)
    Q = zeros(23, 23);
    Q33 = sigma2_deltaXT * diag([lamda_hat_kA1(1), lamda_hat_kA1(1), lamda_hat_kA1(2)]);
    Q(7:9, 7:9)     = Q33;
    Q(10:12, 10:12) = a_pd * a_prd * Q33;
    Q(13:15, 13:15) = a_pd * a_prd * Q33;
    Q66 = a_cov * sigma2_deltaXT * diag([lamda_hat_kA1(1), lamda_hat_kA1(2)]);
    Q(16:17, 16:17) = Q66;
    Q(18:19, 18:19) = Q66;
    Q(20:21, 20:21) = 0.01 * Q66;    % placeholder (PDF: Q88 = ?)
    Q(22:23, 22:23) = 0.01 * Q66;    % placeholder (PDF: Q99 = ?)

    % 4D. Forecast covariance
    P_f = F * P * F' + Q;

    %% 4E. ALL persistent shifts (single location for all updates)

    % Rotation
    V = V_new;  V_T = V_T_new;

    % Wall-frame EKF states
    V_del_p1_hat = V_del_p1_hat_new;
    V_del_p2_hat = V_del_p2_hat_new;
    V_del_p3_hat = V_del_p3_hat_new;
    V_d_hat      = V_d_hat_new;
    V_del_d_hat  = V_del_d_hat_new;

    % Scalar EKF states
    lamda_hat     = lamda_hat_kA1;
    del_lamda_hat = del_lamda_hat_kA1;
    theta_hat     = theta_hat_kA1;
    del_theta_hat = del_theta_hat_kA1;

    % World-frame copies
    del_p1_hat = del_p1_hat_kA1;
    del_p2_hat = del_p2_hat_kA1;
    del_p3_hat = del_p3_hat_kA1;
    d_hat      = d_hat_kA1;
    del_d_hat  = del_d_hat_kA1;

    % Delay buffers
    pd_k2 = pd_k1;
    pd_k1 = pd;
    pd    = pd_A1;              % p_d[k+1] becomes p_d[k] for next step

    % EMA states
    del_pmd_k1  = del_pmd;      % delta_pmd[k-1] <- current delta_pmd[k]
    del_pmrd_k1 = del_pmr;    % delta_pmr[k-1] <- current delta_pmr[k]

end
