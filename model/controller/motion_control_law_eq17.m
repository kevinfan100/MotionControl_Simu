function [f_d, ekf_out] = motion_control_law_eq17(del_pd, pd, p_m, params)
%MOTION_CONTROL_LAW_EQ17 Dispatcher-friendly wrapper for the Eq.17 controller.
%
%   [f_d, ekf_out] = motion_control_law_eq17(del_pd, pd, p_m, params)
%
%   Builds ctrl_const lazily on first call (via build_eq17_constants from
%   params.ctrl and params.thermal), then delegates to
%   motion_control_law_eq17_core(del_pd, pd, p_m, params, ctrl_const).
%
%   This wrapper exists so the unified dispatcher motion_control_law.m
%   can call all three controller variants with the same 4-arg signature
%   even though eq17_core needs the offline scalar bundle ctrl_const as
%   its 5th input.
%
%   Inputs:
%       del_pd - Trajectory increment p_d[k+1] - p_d[k] [3x1, um]
%       pd     - Current desired position p_d[k] [3x1, um]
%       p_m    - Measured position p_m[k] [3x1, um]
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       f_d     - Control force f_d[k] [3x1, pN]
%       ekf_out - First 4 entries of the eq17_core diagnostic output
%                 (canonical contract; richer diag struct is dropped here).
%
%   To reset the cached ctrl_const between runs (e.g. when scenario
%   changes), call `clear motion_control_law_eq17` before run_simulation.

    persistent ctrl_const

    if isempty(ctrl_const)
        ctrl_const = build_ctrl_const_from_params(params);
    end

    [f_d, ekf_out, ~] = motion_control_law_eq17_core(del_pd, pd, p_m, params, ctrl_const);

end


function ctrl_const = build_ctrl_const_from_params(params)
%BUILD_CTRL_CONST_FROM_PARAMS  Pack the build_eq17_constants opts.
%
%   Reads what is currently available from params.ctrl / params.thermal /
%   params.wall and forwards to build_eq17_constants. Fields not present
%   take the build_eq17_constants defaults.

    opts = struct();
    opts.lambda_c   = params.ctrl.lambda_c;
    opts.sigma2_n_s = params.ctrl.sigma2_noise(:);
    opts.kBT        = params.thermal.k_B * params.thermal.T;
    opts.Ts         = 1 / 1600;
    opts.a_cov      = params.ctrl.a_cov;

    if isfield(params.ctrl, 'a_pd')
        opts.a_pd = params.ctrl.a_pd;
    end
    if isfield(params.ctrl, 'beta')
        opts.beta = params.ctrl.beta;
    end

    ctrl_const = build_eq17_constants(opts);
end
