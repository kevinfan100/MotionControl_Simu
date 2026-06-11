function [f_d, ekf_out] = controller_6state(del_pd, pd, p_m, params) %#ok<INUSL> del_pd unused until part 4 (control law)
%CONTROLLER_6STATE Per-axis 6-state EKF controller (RevisedControl_Vpersonal).
%
%   [f_d, ekf_out] = controller_6state(del_pd, pd, p_m, params)
%
%   PACKAGING STATUS: part-3 build. Implemented: [0] init subset (constants
%   C_dpmr/C_n, wall-aware a_hat seed, IIR prefill, pd buffers), [2]
%   measurement chain (delta_x_m + IIR -> a_xm), [11] shifts. Sections
%   [1] control law (part 4) and [3]-[10] EKF (parts 5-6) are pending;
%   f_d is a TEMP zero placeholder, so the loop runs "open loop with a
%   live measurement chain". Reset between runs: `clear controller_6state`
%   (the driver does this).
%
%   Inputs:
%       del_pd - Trajectory increment p_d[k+1] - p_d[k]  [3x1, um]
%       pd     - Current desired position p_d[k]          [3x1, um]
%       p_m    - Measured position (d-step delayed)       [3x1, um]
%       params - Nested parameter struct from config.m
%
%   Outputs:
%       f_d     - Control force f_d[k] [3x1, pN]  (TEMP: zeros until part 4)
%       ekf_out - INTERIM probe [delta_x_m_z; sigma2_dxr_hat_z; a_xm_z; h_bar]
%                 (z-axis measurement-chain probes for gates_part3 -- z is
%                 the wall-normal axis, the only one the ramp trajectory
%                 moves, so the probe exercises BOTH the pd and p_m delay
%                 buffers; the
%                 final form [a_hat_x; a_hat_y; a_hat_z; h_bar] replaces
%                 this in part 5/7)
%
%   Final execution order (paper predictor form + Joseph, D6 /
%   kf_canonical_spec section 1b):
%     [0]  Init (first call): offline constants (C_dpmr, C_n, K_var,
%          IF_abc, xi), wall-aware a_hat seed, DARE -> a-priori P_f0,
%          prefill IIR, buffers
%     [1]  Control law (Eq.17 ACTIVE form, a-priori a_hat / xD_comb)
%     [2]  Measurement: delta_x_m, h_bar; IIR -> a_xm
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
%   Params fields read here (param-flow contract, part-3 subset):
%       params.ctrl.enable / lambda_c / a_pd / a_cov / k_B / T /
%                   Ts / gamma / sigma2_noise
%       params.common.R / p0
%       params.wall.w_hat / pz / enable_wall_effect
%
%   d = 2 sensor-delay steps is HARDCODED (PACKAGING_PLAN decision 6):
%   the C_dpmr / C_n / IF constants are derived for d = 2 only.
%
%   See also: wall_corrections, run_simulation, config

    % ------------------------------------------------------------------
    % [0] Open-loop bypass
    % ------------------------------------------------------------------
    if params.ctrl.enable < 0.5
        f_d = zeros(3, 1);
        ekf_out = zeros(4, 1);
        return;
    end

    % ------------------------------------------------------------------
    % Persistent state
    % ------------------------------------------------------------------
    persistent initialized
    persistent lambda_c a_pd a_cov kBT R_radius sigma2_nx % cached scalars
    persistent C_dpmr C_n                              % offline constants
    persistent w_hat_n pz_wall enable_wall             % wall geometry
    persistent dx_bar_m sigma2_dxr_hat                 % IIR states (3x1)
    persistent pd_km1 pd_km2                           % trajectory delay buffers

    % ------------------------------------------------------------------
    % [0] Initialization on first call
    % ------------------------------------------------------------------
    if isempty(initialized)
        initialized = true;

        % --- 0A. params constants ---
        Ts        = params.ctrl.Ts;
        kBT       = params.ctrl.k_B * params.ctrl.T;     % [pN*um]
        R_radius  = params.common.R;
        gamma_N   = params.ctrl.gamma;                   % [pN*sec/um]
        sigma2_nx = params.ctrl.sigma2_noise;            % 3x1 [um^2]
        lambda_c  = params.ctrl.lambda_c;
        a_pd      = params.ctrl.a_pd;
        a_cov     = params.ctrl.a_cov;

        % --- 0B. Offline constants: C_dpmr / C_n, FULL a_pd-dependent
        %     closed form (kf_canonical_spec section 7 /
        %     Cdpmr_Cn_derivation; lambda_c=0.7, a_pd=0.05 ->
        %     C_dpmr = 3.1610, C_n = 1.1093). Part 7 adds K_var, IF_abc,
        %     xi and the IF self-check assert here. ---
        one_m_apd  = 1 - a_pd;
        denom_pole = 1 - one_m_apd * lambda_c;          % shared pole product
        C_dpmr = one_m_apd^2 * ( ...
                   2 * one_m_apd * (1 - lambda_c) / denom_pole ...
                 + (2 / (2 - a_pd)) * 1 / ((1 + lambda_c) * denom_pole) );
        C_n = (2 * one_m_apd^2 / (2 - a_pd)) * ( ...
                   1 ...
                 + one_m_apd^2 * a_pd * (1 - lambda_c) / denom_pole ...
                 + (1 - lambda_c)^2 / ((1 + lambda_c) * denom_pole) );

        % --- 0C. Wall geometry ---
        w_hat_n     = params.wall.w_hat;
        pz_wall     = params.wall.pz;
        enable_wall = params.wall.enable_wall_effect > 0.5;

        % --- 0E. Wall-aware a_x[0] seeding (part 7 adds K_h_init /
        %     a_perp_init for the DARE P_f0) ---
        a_nom = Ts / gamma_N;
        if enable_wall
            p0_init    = params.common.p0(:);
            h_init_um  = dot(p0_init, w_hat_n) - pz_wall;
            h_bar_init = max(h_init_um / R_radius, 1.001);
            [c_para0, c_perp0] = wall_corrections(h_bar_init);
            a_x_init = [a_nom / c_para0; a_nom / c_para0; a_nom / c_perp0];
        else
            a_x_init = [a_nom; a_nom; a_nom];
        end

        % --- 0H. IIR prefill (three-pillar equilibrium init): seed the
        %     variance estimator at its closed-loop steady state so no
        %     warm-up gate is needed (mother repo prefill branch; the
        %     legacy warm-up mode is not carried over, decision 6). ---
        dx_bar_m       = zeros(3, 1);
        sigma2_dxr_hat = 4 * kBT * a_x_init * C_dpmr + C_n * sigma2_nx;

        % --- 0I. Delay buffers ---
        pd_km1 = pd;
        pd_km2 = pd;

        % --- 0L. First call returns zeros (no f_d yet) ---
        f_d = zeros(3, 1);
        ekf_out = [0; sigma2_dxr_hat(3); 0; 0];   % INTERIM probe (prefill visible)
        return;
    end

    % ------------------------------------------------------------------
    % [1] Control law -- TEMP (part 4): Eq.17 ACTIVE form arrives here.
    %     Until then f_d = 0 (open loop with live measurement chain).
    % ------------------------------------------------------------------
    f_d = zeros(3, 1);

    % ------------------------------------------------------------------
    % [2] Measurement chain
    %     delta_x_m[k] = p_d[k-d] - p_m[k]  (d = 2, hardcoded)
    %     IIR (paper 2025 Eq.9-13): mean EWMA (a_pd) -> residual ->
    %     variance EWMA (a_cov) -> a_xm linear inversion.
    %     (Part 6 adds K_h_axis / sigma2_dh from wall_corrections here.)
    % ------------------------------------------------------------------
    delta_x_m = pd_km2 - p_m;                  % 3x1 [um]

    if enable_wall
        h_bar = (dot(p_m, w_hat_n) - pz_wall) / R_radius;
    else
        h_bar = Inf;
    end

    dx_bar_m_new = (1 - a_pd) * dx_bar_m + a_pd * delta_x_m;
    dx_r = delta_x_m - dx_bar_m_new;
    sigma2_dxr_hat_new = (1 - a_cov) * sigma2_dxr_hat + a_cov * dx_r.^2;
    a_xm = (sigma2_dxr_hat_new - C_n * sigma2_nx) / (C_dpmr * 4 * kBT);   % 3x1 [um/pN]

    % ------------------------------------------------------------------
    % [3]-[10] EKF per axis -- pending (parts 5-6):
    %     [3] innovations  [4] R[k]  [5] gain  [6] state (Phi + L*e)
    %     [7] Joseph  [8] F_e[k]  [9] Q[k]  [10] forecast
    % ------------------------------------------------------------------

    % ------------------------------------------------------------------
    % [11] Buffer shifts + output
    % ------------------------------------------------------------------
    pd_km2 = pd_km1;
    pd_km1 = pd;
    dx_bar_m = dx_bar_m_new;
    sigma2_dxr_hat = sigma2_dxr_hat_new;

    % INTERIM probe output (z-axis measurement chain + h_bar) for
    % gates_part3; replaced by [a_hat_x; a_hat_y; a_hat_z; h_bar] in part 5/7.
    if enable_wall
        h_bar_out = h_bar;
    else
        h_bar_out = 0;
    end
    ekf_out = [delta_x_m(3); sigma2_dxr_hat_new(3); a_xm(3); h_bar_out];
end
