function [f_d, ekf_out] = motion_control_law_2(del_pd, pd, p_m, params)
%MOTION_CONTROL_LAW_2 3-state observer controller with configurable observer pole
%
%   [f_d, ekf_out] = motion_control_law_2(del_pd, pd, p_m, params)
%
%   Implements a 3-state Luenberger observer with configurable pole placement.
%   Observer pole lambda_e is read from params.ctrl.lambda_e:
%       lambda_e = 0  => deadbeat (L1=L2=L3=1)
%       0 < lambda_e < 1 => slower convergence, more noise filtering
%
%   Observer gains (from characteristic polynomial (z - lambda_e)^3):
%       L1 = 1 - 3*lambda_e
%       L2 = 1 - 3*lambda_e + 3*lambda_e^2
%       L3 = (1 - lambda_e)^3
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
%                params.ctrl.lambda_e - Observer pole (0 = deadbeat)
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

        % Observer pole placement gains
        if isfield(params.ctrl, 'lambda_e')
            lambda_e_val = params.ctrl.lambda_e;
        else
            lambda_e_val = 0;  % default deadbeat
        end
        L1 = 1 - 3*lambda_e_val;
        L2 = 1 - 3*lambda_e_val + 3*lambda_e_val^2;
        L3 = (1 - lambda_e_val)^3;

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
