function p0 = calc_initial_position(params)
%CALC_INITIAL_POSITION Calculate safe initial position based on wall parameters
%
%   p0 = calc_initial_position(params)
%
%   Calculates a safe starting position that is at least h_bar_min away
%   from the wall (normalized distance) plus an additional h_margin.
%
%   Inputs:
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       p0 - Initial position [3x1 vector, um]
%
%   Required params fields:
%       params.wall.w_hat      - Wall normal vector [3x1]
%       params.wall.pz         - Wall displacement [um]
%       params.wall.h_bar_min  - Minimum safe normalized distance
%       params.common.R        - Particle radius [um]
%       params.traj.h_margin   - Additional safety margin [um]

    % Extract parameters
    w_hat = params.wall.w_hat;
    pz = params.wall.pz;
    h_bar_min = params.wall.h_bar_min;
    R = params.common.R;
    h_margin = params.traj.h_margin;

    % Calculate safe distance from wall
    % h_safe = R * h_bar_min + h_margin
    %        = R * 1.5 + 5 = 2.25 * 1.5 + 5 = 8.375 um (default)
    h_safe = R * h_bar_min + h_margin;

    % Initial position: wall position + safe distance along normal
    % p0 = pz * w_hat + h_safe * w_hat = (pz + h_safe) * w_hat
    p0 = (pz + h_safe) * w_hat;

end
