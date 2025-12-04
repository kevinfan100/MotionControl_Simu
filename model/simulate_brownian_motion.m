function f_T = simulate_brownian_motion(Gamma_inv)
%#codegen
% SIMULATE_BROWNIAN_MOTION Generate thermal force based on Gamma_inv
%
%   f_T = simulate_brownian_motion(Gamma_inv)
%
%   Inputs:
%       Gamma_inv - Inverse drag coefficient matrix (3x3) from wall_effect_integrated
%
%   Outputs:
%       f_T - Thermal force vector [pN], 3x1
%
%   The variance is computed as: sigma^2 = 4 * k_B * T * S_t / gamma
%   where gamma = inv(Gamma_inv) for each direction (diagonal elements)

    % Physical constants
    k_B = 1.380649e-23;  % Boltzmann constant (J/K)
    T = 300;             % Temperature (K)

    % Sampling parameters
    f_s = 1606;          % Sampling frequency (Hz)
    S_t = 1/f_s;         % Sampling period (s)

    % Calculate drag coefficient matrix from Gamma_inv
    gamma = inv(Gamma_inv);  % 3x3 drag coefficient matrix

    % Extract diagonal elements (direction-dependent drag coefficients)
    % Unit: pN·s/um, convert to SI (N·s/m) for calculation
    gamma_x_SI = gamma(1,1) * 1e-6;  % N·s/m
    gamma_y_SI = gamma(2,2) * 1e-6;  % N·s/m
    gamma_z_SI = gamma(3,3) * 1e-6;  % N·s/m

    % Calculate thermal force standard deviation for each direction (SI units)
    sigma_x_SI = sqrt(4 * k_B * T * S_t / gamma_x_SI);  % N
    sigma_y_SI = sqrt(4 * k_B * T * S_t / gamma_y_SI);  % N
    sigma_z_SI = sqrt(4 * k_B * T * S_t / gamma_z_SI);  % N

    % Convert to pN
    sigma_x_pN = sigma_x_SI * 1e12;  % pN
    sigma_y_pN = sigma_y_SI * 1e12;  % pN
    sigma_z_pN = sigma_z_SI * 1e12;  % pN

    % Generate thermal force (3D Gaussian white noise)
    f_T = [sigma_x_pN * randn;
           sigma_y_pN * randn;
           sigma_z_pN * randn];  % 3x1 vector, unit: pN

end
