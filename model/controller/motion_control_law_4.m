function [f_d, ekf_out] = motion_control_law_4(del_pd, pd, p_m, params)
%MOTION_CONTROL_LAW_4 3-state iterative Kalman filter observer controller
%
%   [f_d, ekf_out] = motion_control_law_4(del_pd, pd, p_m, params)
%
%   Implements a 3-state observer with iterative Kalman filter.
%   Pf and L are updated every step (Predict -> Update cycle).
%   Uses Fe (closed-loop error dynamics, (3,3)=1) for covariance prediction.
%
%   KF cycle per step:
%       1. L = Pf(:,1) / (Pf(1,1) + R)         (Kalman gain from current Pf)
%       2. Observer update with L                 (estimate correction)
%       3. P = (I - L*H) * Pf                    (posterior covariance)
%       4. Pf = Fe * P * Fe' + Q                 (forecast for next step)
%
%   Inputs:
%       del_pd - Trajectory increment p_d[k+1] - p_d[k] [3x1, um]
%       pd     - Current desired position p_d[k] [3x1, um]
%       p_m    - Measured position p_m[k] [3x1, um]
%       params - Parameter structure from calc_simulation_params
%           params.ctrl.kf_R   - KF measurement noise variance R
%           params.ctrl.kf_L   - DARE steady-state gain (used for Pf_init)
%
%   Outputs:
%       f_d     - Control force f_d[k] [3x1, pN]
%       ekf_out - Diagnostic [4x1]: [L(1); 0; 0; 0]

    % Check if control is enabled
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = zeros(4, 1);
        return;
    end

    % --- Persistent declarations ---
    persistent initialized
    persistent lambda_c a_x R_kf
    persistent Pf  % 3x3 forecast covariance (updated every step)
    persistent Fe Q_kf  % system matrices (constant)
    persistent pd_k1 pd_k2
    persistent del_p1_hat del_p2_hat del_p3_hat

    %% Step [0]: Initialization
    if isempty(initialized)
        initialized = true;

        % Extract constants
        lambda_c = params.ctrl.lambda_c;
        a_x      = params.ctrl.Ts / params.ctrl.gamma;
        R_kf     = params.ctrl.kf_R;

        % Fe: closed-loop error dynamics (3,3)=1
        Fe = [0, 1, 0; 0, 0, 1; 0, 0, 1];

        % Q: process noise (only state 3 driven by thermal noise)
        sigma2_deltaXT = params.ctrl.sigma2_deltaXT;
        Q_kf = sigma2_deltaXT * [0,0,0; 0,0,0; 0,0,1];

        % Pf_init: use DARE steady-state solution as default
        % Pf_ss = [a, a, a; a, a+s, a+s; a, a+s, a+2s] where s=sigma2_deltaXT
        % From kf_L (pre-computed in calc_ctrl_params):
        L_ss = params.ctrl.kf_L;
        if L_ss > 0 && L_ss < 1
            a_ss = sigma2_deltaXT / L_ss;  % a = sigma2_deltaXT * (1/L) since L=1/a_normalized, a_physical = a_normalized*sigma2_deltaXT
        else
            a_ss = sigma2_deltaXT;  % fallback
        end
        % Actually, a (normalized) = 1/L, so a_physical = (1/L)*sigma2_deltaXT
        % But Pf is in physical units. Let me recompute properly.
        % In normalized form: a_norm = (1+sqrt(1+4r))/2, Pf_norm = [a,a,a;a,a+1,a+1;a,a+1,a+2]
        % Physical: Pf = Pf_norm * sigma2_deltaXT
        if R_kf > 0
            r = R_kf / sigma2_deltaXT;
            a_norm = (1 + sqrt(1 + 4*r)) / 2;
        else
            a_norm = 1;  % deadbeat
        end
        s = sigma2_deltaXT;
        Pf = s * [a_norm, a_norm, a_norm;
                  a_norm, a_norm+1, a_norm+1;
                  a_norm, a_norm+1, a_norm+2];

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
    del_pm = pd_k2 - p_m;

    %% Step [2]: Kalman Gain from current Pf
    % S = Pf(1,1) + R, L = Pf(:,1) / S
    S = Pf(1,1) + R_kf;
    L1 = Pf(1,1) / S;
    L2 = Pf(2,1) / S;
    L3 = Pf(3,1) / S;

    %% Step [3]: Innovation
    innov = del_pm - del_p1_hat;

    %% Step [4]: Control Law (before observer update, uses current estimates)
    f_d = (1/a_x) * (del_pd + (1 - lambda_c) * del_p3_hat);

    %% Step [5]: Observer Update with current L
    del_p1_hat_new = del_p2_hat              + L1 * innov;
    del_p2_hat_new = del_p3_hat              + L2 * innov;
    del_p3_hat_new = lambda_c * del_p3_hat   + L3 * innov;

    del_p1_hat = del_p1_hat_new;
    del_p2_hat = del_p2_hat_new;
    del_p3_hat = del_p3_hat_new;

    %% Step [6]: Update Pf (Predict -> Update cycle for next step)
    % Posterior: P = (I - L*H) * Pf = Pf - Pf(:,1)*Pf(1,:)/S
    P = Pf - Pf(:,1) * Pf(1,:) / S;

    % Forecast for next step: Pf_new = Fe * P * Fe' + Q
    Pf = Fe * P * Fe' + Q_kf;

    % Symmetry enforcement
    Pf = 0.5 * (Pf + Pf');

    %% Step [7]: Update delay buffers
    pd_k2 = pd_k1;
    pd_k1 = pd;

    %% Output
    ekf_out = [L1; 0; 0; 0];  % diagnostic: current L1 value

end
