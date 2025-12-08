function [Gamma_inv, h_bar] = calc_gamma_inv(p, params)
%CALC_GAMMA_INV Calculate inverse drag coefficient matrix (mobility matrix)
%
%   [Gamma_inv, h_bar] = calc_gamma_inv(p, params)
%
%   Calculates the position-dependent inverse drag coefficient matrix
%   (mobility matrix) that accounts for wall effects.
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
%   Physical interpretation:
%       The mobility matrix Gamma_inv relates force to velocity:
%           p_dot = Gamma_inv * F
%
%       Near a wall, mobility is reduced (harder to move) and becomes
%       anisotropic (different in parallel vs perpendicular directions).
%
%   Reference:
%       Drag_MATLAB.pdf - Wall Effect mathematical model

    % Extract parameters
    w_hat = params.wall.w_hat;
    pz = params.wall.pz;
    R = params.common.R;
    gamma_N = params.common.gamma_N;

    % Calculate distance from wall to particle center
    % h = (p . w_hat) - pz
    % This is the signed distance along the wall normal
    h = dot(p, w_hat) - pz;

    % Normalize by particle radius
    h_bar = h / R;

    % Calculate correction coefficients
    [c_para, c_perp] = calc_correction_functions(h_bar);

    % Calculate projection matrix onto wall normal
    % W = w_hat * w_hat' is the projection onto the normal direction
    W = w_hat * w_hat';

    % Calculate mobility coefficient for the perpendicular direction
    % coeff = (c_perp - c_para) / c_perp
    % This represents the additional reduction in perpendicular mobility
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
