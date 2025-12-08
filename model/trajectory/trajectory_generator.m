function p_d = trajectory_generator(t, p0, params)
%TRAJECTORY_GENERATOR Generate desired position at time t
%
%   p_d = trajectory_generator(t, p0, params)
%
%   Calculates the desired trajectory position based on the current time,
%   starting position, and trajectory parameters.
%
%   Inputs:
%       t      - Current time [sec]
%       p0     - Starting position [3x1 vector, um]
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       p_d    - Desired position [3x1 vector, um]
%
%   Required params fields:
%       params.traj.type       - Trajectory type ('z_move' or 'xy_circle')
%       params.wall.w_hat      - Wall normal vector [3x1]
%       params.wall.u_hat      - Wall parallel vector 1 [3x1]
%       params.wall.v_hat      - Wall parallel vector 2 [3x1]
%
%   For 'z_move' type:
%       params.traj.delta_z    - Travel distance [um]
%       params.traj.direction  - 'away' or 'toward'
%       params.traj.speed      - Travel speed [um/sec]
%
%   For 'xy_circle' type:
%       params.traj.radius     - Circle radius [um]
%       params.traj.period     - Circle period [sec]
%       params.traj.n_circles  - Number of circles
%
%   Trajectory descriptions:
%       'z_move': Linear motion along wall normal direction
%           - 'away': moves in +w_hat direction (away from wall)
%           - 'toward': moves in -w_hat direction (toward wall)
%           - Saturates at delta_z displacement
%
%       'xy_circle': Circular motion in plane parallel to wall
%           - Circle in u_hat-v_hat plane
%           - Starts at p0, circles around, returns to p0
%           - Total time = period * n_circles

    % Get trajectory type
    traj_type = params.traj.type;

    switch traj_type
        case 'z_move'
            p_d = trajectory_z_move(t, p0, params);

        case 'xy_circle'
            p_d = trajectory_xy_circle(t, p0, params);

        otherwise
            error('Unknown trajectory type: %s', traj_type);
    end
end


function p_d = trajectory_z_move(t, p0, params)
%TRAJECTORY_Z_MOVE Linear motion along wall normal direction
    w_hat = params.wall.w_hat;
    delta_z = params.traj.delta_z;
    direction = params.traj.direction;
    speed = params.traj.speed;

    % Determine direction sign
    if strcmp(direction, 'away')
        dir_sign = 1;    % Move away from wall (+w_hat)
    else  % 'toward'
        dir_sign = -1;   % Move toward wall (-w_hat)
    end

    % Calculate displacement with saturation
    displacement = min(speed * t, delta_z);

    % Calculate desired position
    p_d = p0 + dir_sign * displacement * w_hat;
end


function p_d = trajectory_xy_circle(t, p0, params)
%TRAJECTORY_XY_CIRCLE Circular motion in plane parallel to wall
    u_hat = params.wall.u_hat;
    v_hat = params.wall.v_hat;
    radius = params.traj.radius;
    period = params.traj.period;
    n_circles = params.traj.n_circles;

    % Total trajectory time
    T_total = period * n_circles;

    if t <= T_total
        % Angular frequency
        omega = 2 * pi / period;

        % Circular motion in u_hat-v_hat plane
        % At t=0: p_d = p0 (starts at origin of circle path)
        % Uses (cos(omega*t) - 1) to ensure p_d(0) = p0
        p_d = p0 + radius * (cos(omega * t) - 1) * u_hat ...
                 + radius * sin(omega * t) * v_hat;
    else
        % After completing all circles, stay at starting position
        p_d = p0;
    end
end
