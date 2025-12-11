function [is_safe, h_min_actual, t_critical] = check_trajectory_safety(p0, params)
%CHECK_TRAJECTORY_SAFETY Check if trajectory remains safe (h >= h_min)
%
%   [is_safe, h_min_actual, t_critical] = check_trajectory_safety(p0, params)
%
%   Checks the entire trajectory to ensure the particle never gets too
%   close to the wall. Samples the trajectory at discrete time points
%   and calculates distance h at each point.
%
%   Inputs:
%       p0     - Initial position [3x1 vector, um] in world coordinates
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       is_safe      - true if trajectory is safe, false otherwise
%       h_min_actual - Minimum distance h along trajectory [um]
%       t_critical   - Time at which minimum h occurs [sec]
%
%   Required params fields:
%       params.wall.w_hat    - Wall normal vector [3x1, unitless]
%       params.wall.pz       - Wall displacement along w_hat [um]
%       params.wall.h_min    - Minimum safe distance [um]
%       params.common.T_sim  - Simulation time [sec]
%       params.common.Ts     - Sampling period [sec]
%       params.traj.type     - Trajectory type (0=z_move, 1=xy_circle)
%       (plus trajectory-specific parameters)
%
%   Safety criterion:
%       h >= h_min for all t in [0, T_sim]
%       where h = p(t) . w_hat - pz is the distance from wall [um]
%
%   Coordinate system:
%       - Trajectory p_d(t) is in world coordinates
%       - Distance h is computed using wall normal projection

    % Extract parameters
    w_hat = params.wall.w_hat;
    pz = params.wall.pz;
    h_min_threshold = params.wall.h_min;
    T_sim = params.common.T_sim;
    Ts = params.common.Ts;

    % Generate time vector for checking
    t_check = 0:Ts:T_sim;
    N = length(t_check);

    % Calculate h at each time point
    h = zeros(1, N);

    for i = 1:N
        % Get desired position at time t (world coordinates)
        p_d = trajectory_generator(t_check(i), p0, params);

        % Calculate distance from wall [um]
        h(i) = dot(p_d, w_hat) - pz;
    end

    % Find minimum h
    [h_min_actual, idx_min] = min(h);
    t_critical = t_check(idx_min);

    % Check safety
    is_safe = h_min_actual >= h_min_threshold;
end
