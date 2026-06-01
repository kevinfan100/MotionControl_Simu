function [f_d, ekf_out] = motion_control_law(del_pd, pd, p_m, params)
%MOTION_CONTROL_LAW Dispatcher for the per-axis 7-state EKF controllers.
%
%   [f_d, ekf_out] = motion_control_law(del_pd, pd, p_m, params)
%
%   Dispatches on params.ctrl.controller_type:
%     6  -> motion_control_law_eq6   (Paper 2025 Eq.6 control law)
%     17 -> motion_control_law_eq17  (Paper 2023 Eq.17 d-step delay
%                                     compensated, Sigma f_d outside
%                                     1/a_hat bracket, x_D additive)
%     23 -> motion_control_law_23state (legacy 23-state EKF)
%
%   Both 6 and 17 share the same EKF state layout (per-axis 7-state:
%   [del_p1, del_p2, del_p3, d, del_d, a, del_a]); they differ in
%   F_e Row 3 and the control law f_d form.
%
%   The numeric encoding is used so that params.ctrl can stay a
%   Simulink Bus signal (all-double) without introducing a string field.
%
%   Inputs:
%       del_pd - Trajectory increment p_d[k+1] - p_d[k] [3x1, um]
%       pd     - Current desired position p_d[k] [3x1, um]
%       p_m    - Measured position p_m[k] [3x1, um]
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       f_d     - Control force f_d[k] [3x1, pN]
%       ekf_out - EKF diagnostic [4x1] (or richer struct depending on
%                 controller; first 4 entries are the canonical contract)

    switch params.ctrl.controller_type
        case 6
            [f_d, ekf_out] = motion_control_law_eq6(del_pd, pd, p_m, params);
        case 17
            [f_d, ekf_out] = motion_control_law_eq17(del_pd, pd, p_m, params);
        case 23
            [f_d, ekf_out] = motion_control_law_23state(del_pd, pd, p_m, params);
        otherwise
            error('motion_control_law:unknown_type', ...
                  ['Unknown controller_type %d. Expected:\n', ...
                   '  6  = eq6  (Paper 2025 Eq.6 per-axis 7-state EKF)\n', ...
                   '  17 = eq17 (Paper 2023 Eq.17 per-axis 7-state EKF)\n', ...
                   '  23 = legacy 23-state EKF\n', ...
                   'Legacy variants 1/2/4/5/7/8 archived to ', ...
                   'reference/qr_analysis/archive/legacy_controllers/.'], ...
                  params.ctrl.controller_type);
    end

end
