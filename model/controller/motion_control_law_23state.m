function [f_d, ekf_out] = motion_control_law_23state(del_pd, pd, p_m, params)
%MOTION_CONTROL_LAW_23STATE Discrete-time motion control with 23-state EKF
%
%   [f_d, ekf_out] = motion_control_law(del_pd, pd, p_m, params)
%
%   Implements feedforward trajectory tracking with EKF-based estimation
%   of wall-effect parameters (lambda, theta) and disturbance rejection.
%   Reference: Estimation_Control.pdf
%
%   Execution order follows Estimation_Control.pdf:
%     [0]  Init  ->  [1]-[2] Control law  ->  [3] Measurement  ->
%     [4]  Errors  ->  [5] Kalman gain  ->  [6] State update  ->
%     [7]  Posterior cov  ->  [8] F[k]  ->  [9] Q[k]  ->
%     [10] Forecast cov  ->  [11] V[k] update & re-projection
%
%   Inputs:
%       del_pd - Trajectory increment p_d[k+1] - p_d[k] [3x1, um]
%       pd     - Current desired position p_d[k] [3x1, um]
%       p_m    - Measured position p_m[k] [3x1, um]
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       f_d     - Control force f_d[k] [3x1, pN]
%       ekf_out - EKF diagnostic [4x1]: [lamda_hat(2x1); theta_hat(2x1)]

    % Check if control is enabled
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = [1; 1; 0; 0];     % lamda_hat=[1;1], theta_hat=[0;0]
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
        pd_k1   = pd;  % p_d[k-1]
        pd_k2   = pd;  % p_d[k-2]

        % 0E. EMA states (init to zero)
        del_pmd_k1  = zeros(3,1);   % delta_pmd[k-1]
        del_pmrd_k1 = zeros(3,1);   % delta_pmr[k-1]

        % 0F. Kalman filter
        H = zeros(7, 23);
        H(1:3, 1:3) = eye(3);       % selects del_p1
        H(4:5, 16:17) = eye(2);     % selects lambda
        H(6:7, 20:21) = eye(2);     % selects theta

        Pf = 10 * eye(23);          % PDF undefined; arbitrary large value, Riccati converges in a few steps

        g2_cov = g_cov * g_cov;
        
        % R: PDF undefined (has commented-out adaptive formula);
        %    using fixed approximation here
        R = zeros(7,7);
        R(1:3, 1:3) = g2_cov * eye(3);
        R(4:5, 4:5) = a_cov^2 * g2_cov * eye(2);
        R(6:7, 6:7) = a_cov^2 * g2_cov * eye(2);

        % 0G. Error signals (init to zero)
        err = zeros(7,1);

        f_d = zeros(3,1);
        ekf_out = [1; 1; 0; 0];     % lamda_hat=[1;1], theta_hat=[0;0]
        return;
    end

    % NOTE: del_pd = p_d[k+1] - p_d[k], pd = p_d[k], p_m = p_m[k]

    %% Step [1]-[2]: Coordinate Transform & Control Law
    %  Uses persistent [k] values: V_T, lamda_hat, V_del_p3_hat, V_d_hat
    %  Control law executes BEFORE EKF update (standard output-then-update)

    % [1] Transform desired increment to wall frame
    V_del_pd = V_T * del_pd;

    % [2] Position increments and control force
    del_u = (1/lamda_hat(1)) * (V_del_pd(1) + (1-lambda_c)*V_del_p3_hat(1) - V_d_hat(1));
    del_v = (1/lamda_hat(1)) * (V_del_pd(2) + (1-lambda_c)*V_del_p3_hat(2) - V_d_hat(2));
    del_w = (1/lamda_hat(2)) * (V_del_pd(3) + (1-lambda_c)*V_del_p3_hat(3) - V_d_hat(3));

    fu = (gamma_N / Ts) * del_u;
    fv = (gamma_N / Ts) * del_v;
    fw = (gamma_N / Ts) * del_w;

    f_d = V * [fu; fv; fw];                                     % world frame output

    %% Step [3]: Measurement Processing

    % [3a] 2-step delay position error (world frame)
    del_pm = pd_k2 - p_m;                                        % p_d[k-2] - p_m[k]

    % [3b] Transform to wall frame
    V_del_pm = V_T * del_pm;                                     % [del_um; del_vm; del_wm]

    % [3c] EMA decomposition (world frame, 3 layers)
    del_pmd  = (1 - a_pd)  * del_pmd_k1  + a_pd  * del_pm;     % Deterministic
    del_pmr  = del_pm - del_pmd;                                 % Random
    del_pmrd = (1 - a_prd) * del_pmrd_k1 + a_prd * del_pmr;    % Random-deterministic
    del_pmrr = del_pmr - del_pmrd;                               % Residual

    % [3d] Normalized residual
    del_pnr  = (1 / g_cov) * del_pmrr;                          % world frame
    V_del_nr = V_T * del_pnr;                                    % wall frame
    del_unr = V_del_nr(1); del_vnr = V_del_nr(2); del_wnr = V_del_nr(3);

    % [3e] Lambda measurement
    lamda_m = (1 - a_cov) * lamda_hat + a_cov * [(del_unr^2 + del_vnr^2)/2; del_wnr^2];

    % [3f] Theta measurement (conditional on anisotropy)
    del_lamda_m = lamda_m(1) - lamda_m(2);
    if abs(del_lamda_m / lamda_m(1)) < epsilon
        theta_m = theta_hat;                                     % isotropic -> keep previous
    else
        theta_m = theta_hat + a_cov * [del_vnr*del_wnr / del_lamda_m;
                                       -del_wnr*del_unr / del_lamda_m];
    end

    %% Step [4]: Error Signals
    err_p     = V_del_pm - V_del_p1_hat;                          % 3x1
    err_lamda = lamda_m  - lamda_hat;                              % 2x1
    err_theta = theta_m  - theta_hat;                              % 2x1
    err = [err_p; err_lamda; err_theta];                           % 7x1

    %% Step [5]: Kalman Gain
    idx_obs = [1:3, 16:17, 20:21];
    Pf_HT   = Pf(:, idx_obs);                                    % 23x7  (= Pf * H')
    HPf_HT  = Pf(idx_obs, idx_obs);                              % 7x7   (= H * Pf * H')
    G       = inv(HPf_HT + R);                                   % 7x7
    L       = Pf_HT * G;                                         % 23x7

    %% Step [6]: State Update

    % [6a] 9 groups in wall frame
    V_del_p1_hat_kA1  = V_del_p2_hat                    + L(1:3,:)   * err;
    V_del_p2_hat_kA1  = V_del_p3_hat                    + L(4:6,:)   * err;
    V_del_p3_hat_kA1  = lambda_c * V_del_p3_hat         + L(7:9,:)   * err;
    V_d_hat_kA1       = V_d_hat + V_del_d_hat           + L(10:12,:) * err;
    V_del_d_hat_kA1   = V_del_d_hat                     + L(13:15,:) * err;
    lamda_hat_kA1     = lamda_hat + del_lamda_hat        + L(16:17,:) * err;
    del_lamda_hat_kA1 = del_lamda_hat                    + L(18:19,:) * err;
    theta_hat_kA1     = theta_hat + del_theta_hat        + L(20:21,:) * err;
    del_theta_hat_kA1 = del_theta_hat                    + L(22:23,:) * err;

    % [6b] Convert to world frame using current V[k]
    del_p1_hat_kA1 = V * V_del_p1_hat_kA1;
    del_p2_hat_kA1 = V * V_del_p2_hat_kA1;
    del_p3_hat_kA1 = V * V_del_p3_hat_kA1;
    d_hat_kA1      = V * V_d_hat_kA1;
    del_d_hat_kA1  = V * V_del_d_hat_kA1;

    %% Step [7]: Posterior Covariance
    P = (eye(23) - L * H) * Pf;

    %% Step [8]: F[k] Update
    %  G_lamda, G_theta use del_u/v/w from Step [2]
    %  del_lamda_hat_scalar uses lamda_hat_kA1 (updated value)
    del_lamda_hat_scalar = lamda_hat_kA1(1) - lamda_hat_kA1(2);
    dL = del_lamda_hat_scalar;

    G_lamda = [del_u, 0    ;                                     % 3x2
               del_v, 0    ;
               0,     del_w];

    G_theta = [ 0,        -dL*del_w;                             % 3x2
                dL*del_w,  0       ;
                dL*del_v, -dL*del_u];

    F = zeros(23, 23);
    F(1:3,   4:6)   = eye(3);                                    % Row 1
    F(4:6,   7:9)   = eye(3);                                    % Row 2
    F(7:9,   7:9)   = eye(3);                                    % Row 3
    F(7:9,   10:12) = -eye(3);
    F(7:9,   16:17) = -G_lamda;
    F(7:9,   20:21) = -G_theta;
    F(10:12, 10:12) = eye(3);                                    % Row 4
    F(10:12, 13:15) = eye(3);
    F(13:15, 13:15) = eye(3);                                    % Row 5
    F(16:17, 16:17) = eye(2);                                    % Row 6
    F(16:17, 18:19) = eye(2);
    F(18:19, 18:19) = eye(2);                                    % Row 7
    F(20:21, 20:21) = eye(2);                                    % Row 8
    F(20:21, 22:23) = eye(2);
    F(22:23, 22:23) = eye(2);                                    % Row 9

    %% Step [9]: Q[k] Update
    %  Uses lamda_hat_kA1 (updated value)
    Q = zeros(23, 23);
    Q33 = sigma2_deltaXT * diag([lamda_hat_kA1(1), lamda_hat_kA1(1), lamda_hat_kA1(2)]);
    Q(7:9, 7:9)     = Q33;
    Q(10:12, 10:12) = a_pd * a_prd * Q33;
    Q(13:15, 13:15) = a_pd * a_prd * Q33;
    Q66 = a_cov * sigma2_deltaXT * diag([lamda_hat_kA1(1), lamda_hat_kA1(2)]);
    Q(16:17, 16:17) = Q66;
    Q(18:19, 18:19) = Q66;
    Q(20:21, 20:21) = 0.01 * Q66;                                 % PDF undefined (Q88); placeholder
    Q(22:23, 22:23) = 0.01 * Q66;                                 % PDF undefined (Q99); placeholder

    %% Step [10]: Forecast Covariance
    Pf = F * P * F' + Q;

    %% Step [11]: V[k] Update & State Re-projection

    % [11a] New rotation matrix from theta_hat_kA1
    cx = cos(theta_hat_kA1(1)); sx = sin(theta_hat_kA1(1));
    cy = cos(theta_hat_kA1(2)); sy = sin(theta_hat_kA1(2));
    V_new = [ cy,  sy*sx, sy*cx;
              0,   cx,    -sx  ;
             -sy,  cy*sx, cy*cx];
    V_T_new = V_new';

    % [11b] Re-project states to new wall frame
    V_del_p1_hat_new = V_T_new * del_p1_hat_kA1;
    V_del_p2_hat_new = V_T_new * del_p2_hat_kA1;
    V_del_p3_hat_new = V_T_new * del_p3_hat_kA1;
    V_d_hat_new      = V_T_new * d_hat_kA1;
    V_del_d_hat_new  = V_T_new * del_d_hat_kA1;

    % --- Persistent Shifts ---

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
    pd_k1 = pd;                              % pd is input (= p_d[k])

    % EMA states
    del_pmd_k1  = del_pmd;            % delta_pmd[k-1] <- delta_pmd[k]
    del_pmrd_k1 = del_pmr;            % delta_pmr[k-1] <- delta_pmr[k]

    % EKF diagnostic output (updated posterior values)
    ekf_out = [lamda_hat; theta_hat];  % 4x1: [lamda_para; lamda_perp; theta_x; theta_y]

end
