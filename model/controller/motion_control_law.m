function [f_d, ekf_out] = motion_control_law(del_pd, pd, p_m, params)
%MOTION_CONTROL_LAW Dispatcher for motion control law
%
%   [f_d, ekf_out] = motion_control_law(del_pd, pd, p_m, params)
%
%   Dispatches to either 23-state or 7-state controller based on
%   params.ctrl.controller_type.
%
%   Inputs:
%       del_pd - Trajectory increment p_d[k+1] - p_d[k] [3x1, um]
%       pd     - Current desired position p_d[k] [3x1, um]
%       p_m    - Measured position p_m[k] [3x1, um]
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       f_d     - Control force f_d[k] [3x1, pN]
%       ekf_out - EKF diagnostic [4x1]

    switch params.ctrl.controller_type
        case 1
            [f_d, ekf_out] = motion_control_law_1(del_pd, pd, p_m, params);
        case 2
            [f_d, ekf_out] = motion_control_law_2(del_pd, pd, p_m, params);
        case 4
            [f_d, ekf_out] = motion_control_law_4(del_pd, pd, p_m, params);
        case 7
            [f_d, ekf_out] = motion_control_law_7state(del_pd, pd, p_m, params);
        case 23
            [f_d, ekf_out] = motion_control_law_23state(del_pd, pd, p_m, params);
        otherwise
            error('Unknown controller_type: %d', params.ctrl.controller_type);
    end

end
