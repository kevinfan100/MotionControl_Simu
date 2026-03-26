function [f_d, ekf_out] = motion_control_law_2(del_pd, pd, p_m, params)
%MOTION_CONTROL_LAW_2 3-state observer controller (deadbeat, le=0)
%
%   [f_d, ekf_out] = motion_control_law_2(del_pd, pd, p_m, params)
%
%   Implements a 3-state Luenberger observer with deadbeat pole placement
%   (lambda_e=0, L1=L2=L3=1). Three independent axes, no disturbance
%   estimation, no IIR.
%
%   Control law per axis (Observer-1.pdf Eq.3, no disturbance):
%       f_d[k] = (1/a_x) * (del_pd + (1-lc)*del_p3_hat)
%
%   Observer update (Observer-1.pdf Eq.5, L1=L2=L3=1):
%       innov = del_pm - del_p1_hat
%       del_p1_hat[k+1] = del_p2_hat + innov
%       del_p2_hat[k+1] = del_p3_hat + innov
%       del_p3_hat[k+1] = lc*del_p3_hat + innov
%
%   Inputs:
%       del_pd - Trajectory increment p_d[k+1] - p_d[k] [3x1, um]
%       pd     - Current desired position p_d[k] [3x1, um]
%       p_m    - Measured position p_m[k] [3x1, um]
%       params - Parameter structure from calc_simulation_params
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
    persistent pd_k1 pd_k2
    persistent del_p1_hat del_p2_hat del_p3_hat

    %% Step [0]: Initialization
    if isempty(initialized)
        initialized = true;

        % Extract constants
        lambda_c = params.ctrl.lambda_c;
        a_x      = params.ctrl.Ts / params.ctrl.gamma;  % [um/pN]

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

    %% Step [4]: Observer Update (deadbeat: L1=L2=L3=1)
    del_p1_hat_new = del_p2_hat          + innov;
    del_p2_hat_new = del_p3_hat          + innov;
    del_p3_hat_new = lambda_c * del_p3_hat + innov;

    del_p1_hat = del_p1_hat_new;
    del_p2_hat = del_p2_hat_new;
    del_p3_hat = del_p3_hat_new;

    %% Step [5]: Update delay buffers
    pd_k2 = pd_k1;
    pd_k1 = pd;

    %% Output
    ekf_out = zeros(4, 1);

end
