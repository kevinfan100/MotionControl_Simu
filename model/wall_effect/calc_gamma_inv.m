function [Gamma_inv, h_bar] = calc_gamma_inv(p, params)
%CALC_GAMMA_INV Calculate inverse drag coefficient matrix (mobility matrix)
%
%   [Gamma_inv, h_bar] = calc_gamma_inv(p, params)
%
%   Inputs:
%       p      - Particle position [3x1 vector, um]
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       Gamma_inv - Inverse drag coefficient matrix [3x3, um/(pN*sec)]
%       h_bar     - Normalized distance from wall (h/R)
%
%   Required params fields:
%       params.wall.w_hat      - Wall normal vector [3x1]
%       params.wall.pz         - Wall displacement [um]
%       params.common.R        - Particle radius [um]
%       params.common.gamma_N  - Stokes drag coefficient [pN*sec/um]
%

    % Extract parameters
    w_hat = params.wall.w_hat;
    pz = params.wall.pz;
    R = params.common.R;
    gamma_N = params.common.gamma_N;

    % h = (p . w_hat) - pz
    % This is the signed distance along the wall normal
    h = dot(p, w_hat) - pz;

    % Normalize by particle radius
    h_bar = h / R;

    % Calculate correction coefficients
    [c_para, c_perp] = calc_correction_functions(h_bar);

    % W = w_hat * w_hat' is the projection onto the normal direction
    W = w_hat * w_hat';

    % coeff = (c_perp - c_para) / c_perp
    coeff = (c_perp - c_para) / c_perp;

    % Construct the inverse drag coefficient matrix
    % Gamma_inv = (1 / (gamma_N * c_para)) * (I - coeff * W)
    %
    % This can be derived as:
    % - Parallel component: 1 / (gamma_N * c_para)
    % - Perpendicular component: 1 / (gamma_N * c_perp)
    %
    % The formula combines these into a single matrix expression:
    % For any direction d:
    %   d' * Gamma_inv * d = (1/gamma_N) * [1/c_para * (d_para)^2 + 1/c_perp * (d_perp)^2]
    Gamma_inv = (1 / (gamma_N * c_para)) * (eye(3) - coeff * W);

end
