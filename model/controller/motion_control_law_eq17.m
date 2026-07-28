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

    % Codegen note (2026-07-29): the struct must be built COMPLETE here --
    % build_eq17_constants' isfield-guarded default-fill adds fields, which
    % code generation forbids after a struct crosses a function boundary.
    % Values replicate build_eq17_constants' own defaults exactly (the old
    % 7-field version always fell back to these defaults anyway).
    opts = struct();
    opts.lambda_c   = params.ctrl.lambda_c;
    opts.sigma2_n_s = params.ctrl.sigma2_noise(:);
    opts.kBT        = params.thermal.k_B * params.thermal.T;
    opts.Ts         = 1 / 1600;
    opts.a_cov      = params.ctrl.a_cov;
    opts.a_pd       = params.ctrl.a_pd;
    opts.beta       = params.ctrl.beta;
    opts.option              = 'A_MA2_full';
    opts.t_warmup_kf         = 0.2;
    opts.h_bar_safe          = 1.5;
    opts.d                   = 2;
    opts.sigma2_w_fD         = 0;
    opts.sigma2_w_fA         = 0;
    opts.force_Q77_zero      = false;
    opts.h_dot_max_override  = [];
    opts.h_ddot_max_override = [];
    opts.Pf_init_slot7       = [];
    opts.sigma2_w_a_direct   = [];
    opts.Q66_physical_mode   = false;
    opts.Q66_OL_mode         = false;
    opts.R22_include_Q66     = false;
    opts.Q36_cross_term      = false;
    opts.use_joseph_form     = false;
    opts.K_h_cap             = 0;
    opts.iir_warmup_mode     = 'legacy';
    opts.C_dpmr_eff_per_axis = (2 + 1/(1 - params.ctrl.lambda_c^2)) * ones(3, 1);
    opts.C_np_eff_per_axis   = (2 / (1 + params.ctrl.lambda_c)) * ones(3, 1);
    opts.IF_eff_calibrated   = [];

    ctrl_const = build_eq17_constants(opts);
end
