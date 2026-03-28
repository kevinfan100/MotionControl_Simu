function [f_d, ekf_out] = motion_control_law_4(del_pd, pd, p_m, params)
%MOTION_CONTROL_LAW_4 3-state Kalman filter observer controller
%
%   [f_d, ekf_out] = motion_control_law_4(del_pd, pd, p_m, params)
%
%   Implements a 3-state observer with steady-state Kalman filter gains.
%   Same structure as Controller 2 (Luenberger observer), but gains are
%   computed from the discrete algebraic Riccati equation (DARE) instead
%   of pole placement.
%
%   The DARE is solved once at initialization to obtain steady-state
%   forecast covariance Pf_ss, from which the Kalman gain L_ss is derived.
%
%   Plant model for Kalman filter:
%       F = [0, 1, 0; 0, 0, 1; 0, 0, lambda_c]
%       H = [1, 0, 0]
%       Q = sigma2_deltaXT * diag([0, 0, 1])  (thermal noise on state 3)
%       R = kf_R  (measurement noise variance)
%
%   Control law per axis (Observer-1.pdf Eq.3, no disturbance):
%       f_d[k] = (1/a_x) * (del_pd + (1-lc)*del_p3_hat)
%
%   Observer update (Observer-1.pdf Eq.5):
%       innov = del_pm - del_p1_hat
%       del_p1_hat[k+1] = del_p2_hat          + L1 * innov
%       del_p2_hat[k+1] = del_p3_hat          + L2 * innov
%       del_p3_hat[k+1] = lc*del_p3_hat       + L3 * innov
%
%   Inputs:
%       del_pd - Trajectory increment p_d[k+1] - p_d[k] [3x1, um]
%       pd     - Current desired position p_d[k] [3x1, um]
%       p_m    - Measured position p_m[k] [3x1, um]
%       params - Parameter structure from calc_simulation_params
%                params.ctrl.kf_R - KF measurement noise variance
%
%   Outputs:
%       f_d     - Control force f_d[k] [3x1, pN]
%       ekf_out - Diagnostic [4x1]: all zeros (no EKF)

    % Check if control is enabled
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = zeros(4, 1);
        return;
    end

    % --- Persistent declarations ---
    persistent initialized
    persistent lambda_c a_x
    persistent L1 L2 L3
    persistent pd_k1 pd_k2
    persistent del_p1_hat del_p2_hat del_p3_hat

    %% Step [0]: Initialization
    if isempty(initialized)
        initialized = true;

        % Extract constants
        lambda_c = params.ctrl.lambda_c;
        a_x      = params.ctrl.Ts / params.ctrl.gamma;  % [um/pN]

        % KF gain pre-computed in calc_ctrl_params via DARE
        % L1=L2=L3=kf_L for Fe-based KF (closed-loop error dynamics)
        L1 = params.ctrl.kf_L;
        L2 = params.ctrl.kf_L;
        L3 = params.ctrl.kf_L;

        % Delay buffers
        pd_k1 = pd;
        pd_k2 = pd;

        % Observer states
        del_p1_hat = zeros(3, 1);
        del_p2_hat = zeros(3, 1);
        del_p3_hat = zeros(3, 1);

        % Return zeros on first call
        f_d = zeros(3, 1);
        ekf_out = zeros(4, 1);
        return;
    end

    %% Step [1]: Measurement Processing
    % del_pm = p_d[k-2] - p_m[k]
    del_pm = pd_k2 - p_m;

    %% Step [2]: Innovation
    innov = del_pm - del_p1_hat;

    %% Step [3]: Control Law (Observer-1.pdf Eq.3, no disturbance)
    % f_d[k] = (1/a_x) * (del_pd + (1-lc)*del_p3_hat)
    f_d = (1/a_x) * (del_pd + (1 - lambda_c) * del_p3_hat);

    %% Step [4]: Observer Update
    del_p1_hat_new = del_p2_hat              + L1 * innov;
    del_p2_hat_new = del_p3_hat              + L2 * innov;
    del_p3_hat_new = lambda_c * del_p3_hat   + L3 * innov;

    del_p1_hat = del_p1_hat_new;
    del_p2_hat = del_p2_hat_new;
    del_p3_hat = del_p3_hat_new;

    %% Step [5]: Update delay buffers
    pd_k2 = pd_k1;
    pd_k1 = pd;

    %% Output
    ekf_out = zeros(4, 1);

end
