function [is_safe, h_bar_min_actual, t_critical] = check_trajectory_safety(p0, params)
%CHECK_TRAJECTORY_SAFETY Check if trajectory remains safe (h/R > h_bar_min)
%
%   [is_safe, h_bar_min_actual, t_critical] = check_trajectory_safety(p0, params)
%
%   Checks the entire trajectory to ensure the particle never gets too
%   close to the wall. Samples the trajectory at discrete time points
%   and calculates h/R at each point.
%
%   Inputs:
%       p0     - Initial position [3x1 vector, um]
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       is_safe        - true if trajectory is safe, false otherwise
%       h_bar_min_actual - Minimum h/R value along trajectory
%       t_critical     - Time at which minimum h/R occurs [sec]
%
%   Required params fields:
%       params.wall.w_hat      - Wall normal vector [3x1]
%       params.wall.pz         - Wall displacement [um]
%       params.wall.h_bar_min  - Minimum safe normalized distance
%       params.common.R        - Particle radius [um]
%       params.common.T_sim    - Simulation time [sec]
%       params.common.Ts       - Sampling period [sec]
%       params.traj.type       - Trajectory type (0=z_move, 1=xy_circle)
%       (plus trajectory-specific parameters)
%
%   Safety criterion:
%       h/R >= h_bar_min for all t in [0, T_sim]
%       where h = (p(t) . w_hat - pz) is the distance from wall

    % Extract parameters
    w_hat = params.wall.w_hat;
    pz = params.wall.pz;
    h_bar_min_threshold = params.wall.h_bar_min;
    R = params.common.R;
    T_sim = params.common.T_sim;
    Ts = params.common.Ts;

    % Generate time vector for checking
    t_check = 0:Ts:T_sim;
    N = length(t_check);

    % Calculate h/R at each time point
    h_bar = zeros(1, N);

    for i = 1:N
        % Get desired position at time t
        p_d = trajectory_generator(t_check(i), p0, params);

        % Calculate h/R
        h = dot(p_d, w_hat) - pz;
        h_bar(i) = h / R;
    end

    % Find minimum h/R
    [h_bar_min_actual, idx_min] = min(h_bar);
    t_critical = t_check(idx_min);

    % Check safety
    is_safe = h_bar_min_actual >= h_bar_min_threshold;
end
