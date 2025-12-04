function f_t = simulate_brownian_motion(h_bar, params)
%#codegen
% SIMULATE_BROWNIAN_MOTION Calculate thermal force variance based on h_bar
%
%   variance = simulate_brownian_motion(h_bar, params)
%
%   Inputs:
%       h_bar  - Normalized distance to wall (scalar) from wall_effect_integrated
%       params - Parameter struct from calc_wall_params()
%
%   Outputs:
%       f_T - Thermal force [pN], 3x1 vector (f_T = σ .* randn)
%
%   Formula:
%       σ² = (4 * k_B * T * γ_N / Δt) * [c_x²; c_y²; c_z²]
%       where c_∥(h̄)[û + v̂] + c_⊥(h̄)ŵ = [c_x; c_y; c_z]
%
%   Reference: image.png

    % Physical constants
    k_B = 1.3806503e-23;  % Boltzmann constant (J/K)
    T = 273.15 + 37;      % Temperature (K) = 310.15 K
    gamma_N = 4.25e-8;    % Nominal drag coefficient (N·s/m = kg/s)
    delta_t = 1/1606;     % Sampling period (s)

    % Extract w_hat from params
    w_hat = params.w_hat;  % 3x1 normal vector to wall

    % Calculate correction functions from h_bar
    inv_h = 1 / h_bar;

    c_para = (1 - (9/16)*inv_h + (1/8)*inv_h^3 - (45/256)*inv_h^4 - (1/16)*inv_h^5)^(-1);

    c_perp = (1 - (9/8)*inv_h + (1/2)*inv_h^3 - (57/100)*inv_h^4 + (1/5)*inv_h^5 ...
              + (7/200)*inv_h^11 - (1/25)*inv_h^12)^(-1);

    % Calculate c_vec = c_∥(h̄)[û + v̂] + c_⊥(h̄)ŵ
    % Since ŵ = w_hat (normal to wall), and [û + v̂] spans the parallel plane:
    % c_vec = c_para * (I - W) + c_perp * W, applied to unit vectors
    % Simplified: c_i = c_para + (c_perp - c_para) * w_hat_i²
    c_x = c_para + (c_perp - c_para) * w_hat(1)^2;
    c_y = c_para + (c_perp - c_para) * w_hat(2)^2;
    c_z = c_para + (c_perp - c_para) * w_hat(3)^2;

    % Calculate variance: σ² = (4 * k_B * T * γ_N / Δt) * c²
    coeff = 4 * k_B * T * gamma_N / delta_t;

    variance = coeff * [c_x^2; c_y^2; c_z^2];  % 3x1 vector, unit: N² → need to convert to pN²

    % Convert from N² to pN² (1 N = 1e12 pN, so 1 N² = 1e24 pN²)
    variance = variance * 1e24;  % unit: pN²

    % Generate thermal force: f_T = σ .* randn(3,1)
    sigma = sqrt(variance);  % standard deviation (pN)
    f_t = sigma .* randn(3, 1);  % thermal force (pN), 3x1 vector

end
