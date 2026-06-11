function [f_d, ekf_out] = controller_6state(del_pd, pd, p_m, params) %#ok<INUSD> stub: inputs unused until parts 3-7
%CONTROLLER_6STATE Per-axis 6-state EKF controller (RevisedControl_Vpersonal).
%
%   [f_d, ekf_out] = controller_6state(del_pd, pd, p_m, params)
%
%   PACKAGING STATUS: part-2 STUB. The numbered skeleton below is the
%   final layout (23-state style); sections [1]-[11] are filled in
%   packaging parts 3-7. Only the open-loop bypass works today.
%
%   Inputs:
%       del_pd - Trajectory increment p_d[k+1] - p_d[k]  [3x1, um]
%       pd     - Current desired position p_d[k]          [3x1, um]
%       p_m    - Measured position (d-step delayed)       [3x1, um]
%       params - Nested parameter struct from config.m
%
%   Outputs:
%       f_d     - Control force f_d[k]                    [3x1, pN]
%       ekf_out - [a_hat_x; a_hat_y; a_hat_z; h_bar]      (natural axis order)
%
%   Planned execution order (paper predictor form + Joseph, D6 /
%   kf_canonical_spec section 1b):
%     [0]  Init (first call): offline constants (C_dpmr, C_n, K_var,
%          IF_abc, xi), wall-aware a_hat seed, DARE -> a-priori P_f0,
%          prefill IIR, buffers
%     [1]  Control law (Eq.17 ACTIVE form, a-priori a_hat / xD_comb)
%     [2]  Measurement: delta_x_m, h_bar, K_h(h_bar); IIR -> a_xm
%     ---- per-axis loop (x, y, z) ----
%     [3]  Innovations e_x1, e_ax (against stored a-priori)
%     [4]  R[k]  (R11; R22 = K_var*IF_eff*(a_hat+xi)^2 + buffered delay)
%     [5]  Gain  L = P_f*H'/(H*P_f*H' + R)                       (Eq.19)
%     [6]  State x+ = Phi_map(x) + L*e                  (merged, Eq.16)
%     [7]  Posterior covariance, Joseph form            (Eq.20 -> Joseph)
%     [8]  F_e[k] (time-varying Row 3 via f_d history)
%     [9]  Q[k]  (Q33 three-component, Q55 closed form)
%     [10] Forecast P_f+ = F_e*P*F_e' + Q                        (Eq.21)
%     ---- end loop ----
%     [11] Buffer shifts + ekf_out
%
%   Params fields read (full contract arrives with parts 3-7):
%       params.ctrl.enable  (today) ; later: params.ctrl.{Ts, k_B, T,
%       gamma, lambda_c, a_pd, a_cov, sigma2_noise}, params.common.{R, p0},
%       params.wall.{w_hat, pz, enable_wall_effect}
%
%   See also: wall_corrections, run_simulation

    % ------------------------------------------------------------------
    % [0] Open-loop bypass (and, in parts 3-7, first-call initialization)
    % ------------------------------------------------------------------
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = zeros(4, 1);
        return;
    end

    error('controller_6state:notImplemented', ...
          'Sections [1]-[11] arrive in packaging parts 3-7 (closed loop not available yet).');
end
