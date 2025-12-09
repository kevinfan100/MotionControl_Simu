function p0 = calc_initial_position(params)
%CALC_INITIAL_POSITION Calculate initial position for trajectory
%
%   p0 = calc_initial_position(params)
%
%   Calculates the initial particle position based on wall parameters
%   and safety margin. The initial position is placed at a safe distance
%   from the wall along the wall normal direction.
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
%       params.wall.h_bar_min  - Minimum safe normalized distance h/R
%       params.common.R        - Particle radius [um]
%       params.traj.h_margin   - Additional safety margin [um]
%
%   Initial position calculation:
%       The initial position is set along the wall normal direction at:
%       p0 = (h_bar_min * R + h_margin + pz) * w_hat
%
%       This ensures the particle starts at a safe distance from the wall,
%       with h/R >= h_bar_min at t=0.

    % Extract parameters
    w_hat = params.wall.w_hat;
    pz = params.wall.pz;
    h_bar_min = params.wall.h_bar_min;
    R = params.common.R;
    h_margin = params.traj.h_margin;

    % Calculate initial distance from wall
    % h = h_bar * R, so actual distance = h_bar_min * R
    % Add safety margin for initial position
    h_initial = h_bar_min * R + h_margin;

    % Calculate initial position
    % Position along wall normal, offset by wall displacement pz
    p0 = (h_initial + pz) * w_hat;

    % Ensure p0 is a column vector
    p0 = p0(:);
end
