function p0 = calc_initial_position(params)
%CALC_INITIAL_POSITION Calculate initial position for trajectory
%
%   p0 = calc_initial_position(params)
%
%   Calculates the initial particle position based on wall parameters
%   and initial distance. The initial position is placed at a specified
%   distance from the wall along the wall normal direction.
%
%   Inputs:
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       p0 - Initial position [3x1 vector, um] in world coordinates
%
%   Required params fields:
%       params.wall.w_hat      - Wall normal vector [3x1, unitless]
%       params.wall.pz         - Wall displacement along w_hat [um]
%       params.traj.h_init     - Initial distance from wall [um]

    % Extract parameters
    w_hat = params.wall.w_hat;
    pz = params.wall.pz;
    h_init = params.traj.h_init;

    % Calculate initial position
    % Position along wall normal, offset by wall displacement pz
    p0 = (pz + h_init) * w_hat;

    % Ensure p0 is a column vector
    p0 = p0(:);
end
