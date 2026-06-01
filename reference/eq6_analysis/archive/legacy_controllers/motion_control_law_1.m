function [f_d, ekf_out] = motion_control_law_1(del_pd, pd, p_m, params)
%MOTION_CONTROL_LAW_1 Eq.17 delay compensation controller
%
%   [f_d, ekf_out] = motion_control_law_1(del_pd, pd, p_m, params)
%
%   Implements the d-step delay compensation control law (thesis Eq.17)
%   with d=2. Three independent axes, no observer, no IIR.
%
%   Control law per axis:
%       f_d[k] = (1/a_x) * (del_pd + (1-lc)*del_pm) - (1-lc)*(fd[k-1] + fd[k-2])
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
    persistent fd_k1 fd_k2

    %% Step [0]: Initialization
    if isempty(initialized)
        initialized = true;

        % Extract constants
        lambda_c = params.ctrl.lambda_c;
        a_x      = params.ctrl.Ts / params.ctrl.gamma;  % [um/pN]

        % Delay buffers
        pd_k1 = pd;
        pd_k2 = pd;

        % Previous control forces
        fd_k1 = zeros(3, 1);
        fd_k2 = zeros(3, 1);

        % Return zeros on first call
        f_d = zeros(3, 1);
        ekf_out = zeros(4, 1);
        return;
    end

    %% Step [1]: Measurement Processing
    % del_pm = p_d[k-2] - p_m[k]
    del_pm = pd_k2 - p_m;

    %% Step [2]: Control Law (Eq.17)
    % f_d[k] = (1/a_x)*(del_pd + (1-lc)*del_pm) - (1-lc)*(fd[k-1] + fd[k-2])
    f_d = (1/a_x) * (del_pd + (1 - lambda_c) * del_pm) ...
        - (1 - lambda_c) * (fd_k1 + fd_k2);

    %% Step [3]: Update persistent state
    fd_k2 = fd_k1;
    fd_k1 = f_d;
    pd_k2 = pd_k1;
    pd_k1 = pd;

    %% Output
    ekf_out = zeros(4, 1);

end
