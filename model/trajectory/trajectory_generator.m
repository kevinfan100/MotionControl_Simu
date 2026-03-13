function [p_d, del_pd] = trajectory_generator(t, params)
%TRAJECTORY_GENERATOR Generate desired position at time t
%
%   [p_d, del_pd] = trajectory_generator(t, params)
%
%   Generates sinusoidal trajectory along the wall normal direction (w_hat).
%   The trajectory is defined in h-space (wall-normal distance) but the
%   output p_d is in world coordinates.
%
%   Inputs:
%       t      - Current time [sec]
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       p_d    - Desired position p_d[k+1] [3x1 vector, um] in world coordinates
%       del_pd - Trajectory increment p_d[k+1] - p_d[k] [3x1 vector, um]
%
%   Required params fields:
%       params.common.p0       - Starting position [3x1, um, world coords]
%       params.wall.w_hat      - Wall normal vector [3x1, unitless]
%       params.traj.amplitude  - Oscillation amplitude in h direction [um]
%       params.traj.frequency  - Oscillation frequency [Hz]
%       params.traj.n_cycles   - Number of cycles
%
%   Trajectory (one-step ahead):
%       t_next = t + Ts
%       p_d[k+1] = p0 + amplitude * sin(2*pi*frequency * t_next) * w_hat
%       Total time = n_cycles / frequency
%       Returns to p0 after completion
%
%   Note: Output is p_d[k+1] (one sample ahead of current time t).
%         Use a Unit Delay (IC=p0) in Simulink to obtain p_d[k] for recording.

    persistent p_d_prev

    p0 = params.common.p0;
    w_hat = params.wall.w_hat;
    Ts = params.common.Ts;

    amplitude = params.traj.amplitude;
    frequency = params.traj.frequency;
    n_cycles = params.traj.n_cycles;

    % Total trajectory time
    T_total = n_cycles / frequency;

    % One-step ahead: evaluate trajectory at t + Ts
    t_next = t + Ts;

    if t_next <= T_total
        omega = 2 * pi * frequency;
        displacement = amplitude * sin(omega * t_next);
        p_d = p0 + displacement * w_hat;
    else
        % After completing all cycles, stay at starting position
        p_d = p0;
    end

    % Compute trajectory increment del_pd = p_d[k+1] - p_d[k]
    if isempty(p_d_prev)
        p_d_prev = p0;      % p_d[0] = p0
    end
    del_pd = p_d - p_d_prev;
    p_d_prev = p_d;
end
