function [p_dot, Gamma_inv] = wall_effect_integrated(f, p, params)
%#codegen
% WALL_EFFECT_INTEGRATED Real-time wall effect calculation for Simulink
%
%   [p_dot, Gamma_inv] = wall_effect_integrated(f, p, params)
%
%   This function computes:
%       Part 1: Γ⁻¹(p) - inverse drag coefficient matrix
%       Part 2: p_dot = Γ⁻¹(p) * f - velocity
%
%   Note: Integration (1/s) is done externally in Simulink (continuous Integrator)
%
%   Inputs:
%       f      - Total force [pN], 3x1 vector (f = f_M + f_T, continuous after ZOH)
%       p      - Current position [um], 3x1 vector (feedback from Integrator)
%       params - Parameter struct from calc_wall_params() (in workspace)
%
%   Outputs:
%       p_dot     - Velocity [um/sec], 3x1 vector (to Integrator)
%       Gamma_inv - Inverse drag coefficient matrix (3x3)
%
%   Usage in Simulink MATLAB Function block:
%       function [p_dot, Gamma_inv] = fcn(f, p, params)
%           [p_dot, Gamma_inv] = wall_effect_integrated(f, p, params);
%       end
%
%   System equation:
%       p_dot = Γ⁻¹(p) * (f_M + f_T)
%       p = ∫ p_dot dt    (done in Simulink Integrator block)

    %% Extract pre-calculated fixed parameters
    w_hat   = params.w_hat;     % From calc_wall_params
    p_z     = params.p_z;
    R       = params.R;
    gamma_N = params.gamma_N;
    W       = params.W;         % From calc_wall_params
    I       = params.I;         % From calc_wall_params

    %% ===== Part 1: Calculate Γ⁻¹(p) =====

    % Step 1.1: Distance from particle to plane
    h = p' * w_hat - p_z;

    % Step 1.2: Normalized distance
    h_bar = h / R;

    % Step 1.3: Correction functions
    inv_h = 1 / h_bar;

    c_para = (1 - (9/16)*inv_h + (1/8)*inv_h^3 - (45/256)*inv_h^4 - (1/16)*inv_h^5)^(-1);

    c_perp = (1 - (9/8)*inv_h + (1/2)*inv_h^3 - (57/100)*inv_h^4 + (1/5)*inv_h^5 ...
              + (7/200)*inv_h^11 - (1/25)*inv_h^12)^(-1);

    % Step 1.4: Gamma_inv
    coeff = 1 / (gamma_N * c_para);
    correction = (c_perp - c_para) / c_perp;
    Gamma_inv = coeff * (I - correction * W);

    %% ===== Part 2: Calculate p_dot =====
    p_dot = Gamma_inv * f;

end
