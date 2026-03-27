function [f_d, ekf_out] = motion_control_law_7state(del_pd, pd, p_m, params)
%MOTION_CONTROL_LAW_7STATE Per-axis 7-state EKF controller
%
%   [f_d, ekf_out] = motion_control_law_7state(del_pd, pd, p_m, params)
%
%   Implements 3 independent 7-state EKF controllers (one per axis) with
%   execution order: control -> measurement -> IIR -> EKF.
%
%   State vector per axis: [del_p1, del_p2, del_p3, d, del_d, a, del_a]
%     del_p1..3: tracking error delay chain
%     d, del_d:  disturbance pair
%     a, del_a:  motion gain pair
%
%   Inputs:
%       del_pd - Trajectory increment p_d[k+1] - p_d[k] [3x1, um]
%       pd     - Current desired position p_d[k] [3x1, um]
%       p_m    - Measured position p_m[k] [3x1, um]
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       f_d     - Control force f_d[k] [3x1, pN]
%       ekf_out - EKF diagnostic [4x1]: [a_hat_x; a_hat_z; 0; 0]

    % Check if control is enabled
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = [1; 1; 0; 0];
        return;
    end

    % --- Persistent declarations ---

    % 7-state estimates (3x1 vectors, one element per axis)
    persistent del_p1_hat del_p2_hat del_p3_hat
    persistent d_hat del_d_hat
    persistent a_hat del_a_hat

    % Covariance (3 independent 7x7 matrices)
    persistent Pf_1 Pf_2 Pf_3

    % Delay buffers
    persistent pd_k1 pd_k2

    % IIR states (3x1 vectors)
    persistent del_pmd_k1 del_pmrd_k1
    persistent del_pmr2_avg

    % Warm-up counter
    persistent warmup_count

    % Constants (extracted once)
    persistent initialized
    persistent gamma_N Ts lambda_c k_B T_temp
    persistent a_pd a_prd a_cov beta lamdaF
    persistent sigma2_noise sigma2_deltaXT
    persistent Q_1 Q_2 Q_3 Rz_scaling

    %% Step [0]: Initialization
    if isempty(initialized)
        initialized = true;

        % 0A. Extract constants
        gamma_N = params.ctrl.gamma;
        Ts      = params.ctrl.Ts;
        lambda_c = params.ctrl.lambda_c;
        a_pd    = params.ctrl.a_pd;
        a_prd   = params.ctrl.a_prd;
        a_cov   = params.ctrl.a_cov;
        k_B     = params.ctrl.k_B;
        T_temp  = params.ctrl.T;
        beta    = params.ctrl.beta;
        lamdaF  = params.ctrl.lamdaF;
        sigma2_noise    = params.ctrl.sigma2_noise;       % 3x1 [um^2]
        sigma2_deltaXT  = params.ctrl.sigma2_deltaXT;     % scalar [um^2]

        % 0B. Nominal motion gain a_nom = Ts / gamma_N
        a_nom = Ts / gamma_N;

        % 0C. EKF states [k=0]
        del_p1_hat = zeros(3,1);
        del_p2_hat = zeros(3,1);
        del_p3_hat = zeros(3,1);
        d_hat      = zeros(3,1);
        del_d_hat  = zeros(3,1);
        a_hat      = [a_nom; a_nom; a_nom];
        del_a_hat  = zeros(3,1);

        % 0D. Covariance initialization
        Pf_diag = params.ctrl.Pf_init_diag;  % 7x1
        Pf_1 = diag(Pf_diag);
        Pf_2 = diag(Pf_diag);
        Pf_3 = diag(Pf_diag);

        % 0E. Q and R matrices (per axis)
        Qz_scaling = params.ctrl.Qz_diag_scaling;  % 7x1
        Rz_scaling = params.ctrl.Rz_diag_scaling;   % 2x1

        Q_base = sigma2_deltaXT * diag(Qz_scaling);
        Q_1 = Q_base;  Q_2 = Q_base;  Q_3 = Q_base;

        % 0F. Delay buffers
        pd_k1 = pd;
        pd_k2 = pd;

        % 0G. IIR states
        del_pmd_k1  = zeros(3,1);
        del_pmrd_k1 = zeros(3,1);
        del_pmr2_avg = zeros(3,1);

        % 0H. IIR warm-up counter (0.2s at Ts rate)
        warmup_count = round(0.2 / Ts);

        % 0I. Return zeros on first call
        f_d = zeros(3, 1);
        ekf_out = [a_nom; a_nom; 0; 0];
        return;
    end

    %% Step [1]: Control Force (uses persistent [k] forecast estimates)
    %  f_d[k] = (1./a_hat[k|k-1]) .* (del_pd + (1-lc)*del_p3_hat[k|k-1] - d_hat[k|k-1])

    f_d = (1 ./ a_hat) .* ...
          (del_pd + (1 - lambda_c) * del_p3_hat - d_hat);

    %% Step [2]: Measurement Processing
    %  del_pm = p_d[k-2] - p_m[k]
    del_pm = pd_k2 - p_m;

    %% Step [3]: IIR Single-layer Highpass + Gain Recovery

    %  LP of measurement
    del_pmd  = (1 - a_pd) * del_pmd_k1 + a_pd * del_pm;
    del_pmr  = del_pm - del_pmd;                             % HP residual
    %  Running mean and mean-square of HP residual
    del_pmrd = (1 - a_prd) * del_pmrd_k1 + a_prd * del_pmr; % E[del_pmr]
    del_pmr2_avg = (1 - a_cov) * del_pmr2_avg + a_cov * del_pmr.^2; % E[del_pmr^2]
    %  Variance = E[X^2] - E[X]^2
    del_pmr_var = max(del_pmr2_avg - del_pmrd.^2, 0);       % 3x1 [um^2]

    %  Eq.13 Gain Recovery
    C_dx  = 2*(1-a_pd)*(1-lambda_c) / (1-(1-a_pd)*lambda_c) ...
          + (2/(2-a_pd)) / ((1+lambda_c)*(1-(1-a_pd)*lambda_c));
    den   = C_dx * 4 * k_B * T_temp;                        % [pN*um]
    noise_corr = (2 / (1 + lambda_c)) * sigma2_noise;       % 3x1 [um^2]
    a_m   = max((del_pmr_var - noise_corr) / den, 0);       % 3x1 [um/pN]

    %% IIR Warm-up gate: only run measurement + IIR + Eq.13, skip EKF
    if warmup_count > 0
        warmup_count = warmup_count - 1;

        % Update delay buffers and IIR states
        pd_k2 = pd_k1;
        pd_k1 = pd;
        del_pmd_k1  = del_pmd;
        del_pmrd_k1 = del_pmrd;

        % On last warmup step: seed EKF del_p states with IIR LP estimate
        if warmup_count == 0
            del_p1_hat = del_pmd;
            del_p2_hat = del_pmd;
            del_p3_hat = del_pmd;
        end

        f_d = zeros(3, 1);
        ekf_out = [a_hat(1); a_hat(3); 0; 0];
        return;
    end

    %% Step [4]: EKF Update

    % Pack per-axis state vectors
    states_x = [del_p1_hat(1); del_p2_hat(1); del_p3_hat(1); ...
                d_hat(1); del_d_hat(1); a_hat(1); del_a_hat(1)];
    states_y = [del_p1_hat(2); del_p2_hat(2); del_p3_hat(2); ...
                d_hat(2); del_d_hat(2); a_hat(2); del_a_hat(2)];
    states_z = [del_p1_hat(3); del_p2_hat(3); del_p3_hat(3); ...
                d_hat(3); del_d_hat(3); a_hat(3); del_a_hat(3)];

    % Measurements: [del_pm_i; a_m_i]
    meas_x = [del_pm(1); a_m(1)];
    meas_y = [del_pm(2); a_m(2)];
    meas_z = [del_pm(3); a_m(3)];

    % Adaptive R: when del_pmr_var is too small (no thermal excitation),
    % inflate gain channel noise to prevent EKF from trusting a_m = 0.
    var_threshold = sigma2_deltaXT * 0.001;
    r_pos_base = sigma2_deltaXT * Rz_scaling(1);
    r_gain_base = sigma2_deltaXT * Rz_scaling(2);
    R_i = cell(3,1);
    for i = 1:3
        if del_pmr_var(i) < var_threshold
            r_gain_i = 1e6;     % effectively ignore gain measurement
        else
            r_gain_i = r_gain_base;
        end
        R_i{i} = diag([r_pos_base, r_gain_i]);
    end

    % --- F[k] Error dynamics matrices ---
    F_x = [0 1 0  0  0  0        0; ...
           0 0 1  0  0  0        0; ...
           0 0 1 -1  0 -f_d(1)   0; ...
           0 0 0  1  1  0        0; ...
           0 0 0  0  1  0        0; ...
           0 0 0  0  0  1        1; ...
           0 0 0  0  0  0        1];
    F_y = [0 1 0  0  0  0        0; ...
           0 0 1  0  0  0        0; ...
           0 0 1 -1  0 -f_d(2)   0; ...
           0 0 0  1  1  0        0; ...
           0 0 0  0  1  0        0; ...
           0 0 0  0  0  1        1; ...
           0 0 0  0  0  0        1];
    % z axis: beta coupling
    F_z = [0 1 0  0        0     0        0; ...
           0 0 1  0        0     0        0; ...
           0 0 1 -1        0    -f_d(3)   0; ...
           0 0 0  1+beta  -beta  0        0; ...
           0 0 0  1        0     0        0; ...
           0 0 0  0        0     1+beta  -beta; ...
           0 0 0  0        0     1        0];

    % --- EKF injection + covariance ---
    % x-axis: lamdaF=1
    [inj_x, Pf_1] = ekf_update_7state( ...
        states_x, Pf_1, meas_x, Q_1, R_i{1}, F_x, 1);
    % y-axis: lamdaF=1
    [inj_y, Pf_2] = ekf_update_7state( ...
        states_y, Pf_2, meas_y, Q_2, R_i{2}, F_y, 1);
    % z-axis: beta coupling, lamdaF=lamdaF
    [inj_z, Pf_3] = ekf_update_7state( ...
        states_z, Pf_3, meas_z, Q_3, R_i{3}, F_z, lamdaF);

    %% Step [5]: State Update

    del_p1_hat_kA1 = zeros(3,1);
    del_p2_hat_kA1 = zeros(3,1);
    del_p3_hat_kA1 = zeros(3,1);
    d_hat_kA1      = zeros(3,1);
    del_d_hat_kA1  = zeros(3,1);
    a_hat_kA1      = zeros(3,1);
    del_a_hat_kA1  = zeros(3,1);

    for ax = 1:2   % x, y axes
        if ax == 1, inj = inj_x; else, inj = inj_y; end
        del_p1_hat_kA1(ax) = del_p2_hat(ax)                           + inj(1);
        del_p2_hat_kA1(ax) = del_p3_hat(ax)                           + inj(2);
        del_p3_hat_kA1(ax) = lambda_c * del_p3_hat(ax)                + inj(3);
        d_hat_kA1(ax)      = d_hat(ax) + del_d_hat(ax)                + inj(4);
        del_d_hat_kA1(ax)  = del_d_hat(ax)                            + inj(5);
        a_hat_kA1(ax)      = a_hat(ax) + del_a_hat(ax)                + inj(6);
        del_a_hat_kA1(ax)  = del_a_hat(ax)                            + inj(7);
    end

    % z-axis -- beta coupling
    inj = inj_z;
    del_p1_hat_kA1(3) = del_p2_hat(3)                                 + inj(1);
    del_p2_hat_kA1(3) = del_p3_hat(3)                                 + inj(2);
    del_p3_hat_kA1(3) = lambda_c * del_p3_hat(3)                      + inj(3);
    d_hat_kA1(3)      = (1+beta)*d_hat(3) - beta*del_d_hat(3)         + inj(4);
    del_d_hat_kA1(3)  = d_hat(3)                                      + inj(5);
    a_hat_kA1(3)      = (1+beta)*a_hat(3) - beta*del_a_hat(3)         + inj(6);
    del_a_hat_kA1(3)  = a_hat(3)                                      + inj(7);

    %% Step [6]: Persistent Shifts

    % Apply updated states
    del_p1_hat = del_p1_hat_kA1;
    del_p2_hat = del_p2_hat_kA1;
    del_p3_hat = del_p3_hat_kA1;
    d_hat      = d_hat_kA1;
    del_d_hat  = del_d_hat_kA1;
    a_hat      = a_hat_kA1;
    del_a_hat  = del_a_hat_kA1;

    % Delay buffers
    pd_k2 = pd_k1;
    pd_k1 = pd;

    % IIR states
    del_pmd_k1  = del_pmd;
    del_pmrd_k1 = del_pmrd;

    %% Step [7]: Output
    ekf_out = [a_hat(1); ...            % x-axis gain [um/pN]
               a_hat(3); ...            % z-axis gain [um/pN]
               0; 0];

end


%% ==================== Local Function ====================

function [inj, Pf_new] = ekf_update_7state( ...
    states, Pf, meas, Q, R, F, lamdaF_i)
%EKF_UPDATE_7STATE Single-axis 7-state EKF: injection + covariance
%
%   Split design per paper Eq.16-21:
%     - Returns injection vector (L * innovation), NOT updated states
%     - Caller performs explicit state update (Eq.16)
%     - Covariance uses F from error dynamics (Eq.18)
%
%   Inputs:
%       states:    7x1 = [del_p1; del_p2; del_p3; d; del_d; a; del_a]
%       Pf:        7x7 forecast covariance
%       meas:      2x1 = [del_pm_i; a_m_i]
%       Q:         7x7 process noise
%       R:         2x2 measurement noise
%       F:         7x7 error dynamics matrix (Eq.18)
%       lamdaF_i:  scalar, forgetting factor
%
%   Outputs:
%       inj:     7x1 injection vector (= L * innovation)
%       Pf_new:  7x7 forecast covariance for next step

    H = [1 0 0 0 0 0 0; ...
         0 0 0 0 0 1 0];

    % Innovation (Eq.16 feedback)
    err = meas - H * states;

    % Innovation covariance (Eq.19)
    S = H * Pf * H' + R;
    S = 0.5 * (S + S') + 1e-20 * eye(2);

    % Kalman gain (Eq.19)
    L = (Pf * H') / S;

    % Injection vector -- caller uses this for explicit state update
    inj = L * err;

    % Posterior covariance (Eq.20) with forgetting factor
    P = (1 / lamdaF_i) * (eye(7) - L * H) * Pf;
    P = 0.5 * (P + P');

    % Forecast covariance (Eq.21) -- uses F (error dynamics)
    Pf_new = F * P * F' + Q;
    Pf_new = 0.5 * (Pf_new + Pf_new');

end
