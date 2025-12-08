function [is_safe, h_bar_min_actual, t_critical] = check_trajectory_safety(p0, params)
%CHECK_TRAJECTORY_SAFETY Verify trajectory maintains safe distance from wall
%
%   [is_safe, h_bar_min_actual, t_critical] = check_trajectory_safety(p0, params)
%
%   Checks all points along the planned trajectory to ensure the normalized
%   distance from wall (h_bar) stays above the safety threshold.
%
%   Inputs:
%       p0     - Starting position [3x1 vector, um]
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       is_safe         - Logical: true if trajectory is safe
%       h_bar_min_actual- Minimum h_bar found along trajectory
%       t_critical      - Time when minimum h_bar occurs [sec]
%
%   Required params fields:
%       params.wall.w_hat      - Wall normal vector [3x1]
%       params.wall.pz         - Wall displacement [um]
%       params.wall.h_bar_min  - Minimum safe normalized distance
%       params.common.R        - Particle radius [um]
%       params.common.Ts       - Sampling period [sec]
%       params.common.T_sim    - Simulation time [sec]
%       params.traj.*          - Trajectory parameters
%
%   Safety check process:
%       1. Generate time vector from 0 to T_sim at sampling interval Ts
%       2. Calculate p_d at each time using trajectory_generator
%       3. Compute h_bar = (h - pz) / R for each point
%       4. Find minimum h_bar and check against h_bar_min
%       5. Issue warning if trajectory is unsafe

    % Extract parameters
    w_hat = params.wall.w_hat;
    pz = params.wall.pz;
    R = params.common.R;
    h_bar_min = params.wall.h_bar_min;
    Ts = params.common.Ts;
    T_sim = params.common.T_sim;

    % Generate time vector
    t_vec = 0:Ts:T_sim;
    N = length(t_vec);

    % Calculate h_bar for all trajectory points
    h_bar_vec = zeros(1, N);
    for i = 1:N
        % Get desired position at time t
        p_d = trajectory_generator(t_vec(i), p0, params);

        % Calculate distance from wall
        h = dot(p_d, w_hat) - pz;

        % Normalize by particle radius
        h_bar_vec(i) = h / R;
    end

    % Find minimum h_bar and its time
    [h_bar_min_actual, idx] = min(h_bar_vec);
    t_critical = t_vec(idx);

    % Check safety
    is_safe = (h_bar_min_actual >= h_bar_min);

    % Issue warning if unsafe
    if ~is_safe
        warning('Trajectory unsafe! Min h/R = %.2f at t = %.3f sec. Required h/R_min = %.2f', ...
            h_bar_min_actual, t_critical, h_bar_min);
    end
end
