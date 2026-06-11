function [Gamma_inv, h_bar] = gamma_inv(p, params)
%GAMMA_INV Position-dependent mobility matrix (inverse drag) with wall effect.
%
%   [Gamma_inv, h_bar] = gamma_inv(p, params)
%
%   Plant-side mobility: p_dot = Gamma_inv(p) * F_total.
%
%   Inputs:
%       p      - Particle position [3x1, um]
%       params - Nested parameter struct (built by config.m)
%
%   Outputs:
%       Gamma_inv - Mobility matrix [3x3, um/(pN*sec)]
%       h_bar     - Normalized wall distance h/R
%
%   Params fields read here (param-flow contract):
%       params.wall.w_hat               - Wall normal vector [3x1]
%       params.wall.pz                  - Wall displacement [um]
%       params.wall.enable_wall_effect  - >0.5 = wall effect ON
%       params.common.R                 - Particle radius [um]
%       params.common.gamma_N           - Stokes drag coefficient [pN*sec/um]
%
%   Source: verbatim from model/wall_effect/calc_gamma_inv.m (packaging
%   part 1; logic unchanged, calls wall_corrections instead of
%   calc_correction_functions).

    % Extract parameters
    w_hat = params.wall.w_hat;
    pz = params.wall.pz;
    R = params.common.R;
    gamma_N = params.common.gamma_N;
    enable_wall = params.wall.enable_wall_effect;

    % h = (p . w_hat) - pz
    % This is the signed distance along the wall normal
    h = dot(p, w_hat) - pz;

    % Normalize by particle radius
    h_bar = h / R;

    if enable_wall > 0.5
        % Wall effect ON: anisotropic mobility matrix

        % Calculate correction coefficients
        [c_para, c_perp] = wall_corrections(h_bar);

        % W = w_hat * w_hat' is the projection onto the normal direction
        W = w_hat * w_hat';

        % coeff = (c_perp - c_para) / c_perp
        coeff = (c_perp - c_para) / c_perp;

        % Construct the mobility matrix
        % Gamma_inv = (1 / (gamma_N * c_para)) * (I - coeff * W)
        % Parallel eigenvalue:      1 / (gamma_N * c_para)   (x2)
        % Perpendicular eigenvalue: 1 / (gamma_N * c_perp)
        Gamma_inv = (1 / (gamma_N * c_para)) * (eye(3) - coeff * W);
    else
        % Wall effect OFF: isotropic Stokes drag (c_para = c_perp = 1)
        Gamma_inv = (1 / gamma_N) * eye(3);
    end

end
