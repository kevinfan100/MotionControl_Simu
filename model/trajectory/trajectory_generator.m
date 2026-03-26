function [p_d, del_pd] = trajectory_generator(t, params)
%TRAJECTORY_GENERATOR Generate desired position at time t
%
%   [p_d, del_pd] = trajectory_generator(t, params)
%
%   Four-phase trajectory along wall normal direction (w_hat):
%     Phase 1 (hold):        Stay at h_init for EKF warm-up
%     Phase 2 (descent):     Cosine ease-in/out from h_init to h_bottom
%     Phase 3 (oscillation): Cosine oscillation with h_bottom as trough
%     Phase 4 (hold):        Stay at h_bottom after oscillation ends
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
%       params.common.Ts       - Sample time [sec]
%       params.wall.w_hat      - Wall normal vector [3x1, unitless]
%       params.wall.pz         - Wall displacement along w_hat [um]
%       params.traj.t_hold     - Initial hold time at h_init [sec]
%       params.traj.h_init     - Initial height from wall [um]
%       params.traj.h_bottom   - Lowest point / oscillation trough [um]
%       params.traj.amplitude  - Oscillation half-amplitude [um]
%       params.traj.frequency  - Oscillation frequency [Hz]
%       params.traj.n_cycles   - Number of oscillation cycles
%
%   Timing:
%       t_hold    = t_hold (user-defined, e.g. 0.5 sec)
%       t_descend = 1/frequency
%       T_osc     = n_cycles/frequency
%
%   Note: Output is p_d[k+1] (one sample ahead of current time t).
%         Use a Unit Delay (IC=p0) in Simulink to obtain p_d[k] for recording.

    persistent p_d_prev

    p0 = params.common.p0;
    w_hat = params.wall.w_hat;
    pz = params.wall.pz;
    Ts = params.common.Ts;

    t_hold    = params.traj.t_hold;
    h_init    = params.traj.h_init;
    h_bottom  = params.traj.h_bottom;
    amplitude = params.traj.amplitude;
    frequency = params.traj.frequency;
    n_cycles  = params.traj.n_cycles;
    trajectory_type = params.traj.trajectory_type;

    % Positioning mode: hold at h_init for entire simulation
    if trajectory_type > 1.5
        h = h_init;
        p_d = (pz + h) * w_hat;

        if isempty(p_d_prev)
            p_d_prev = p0;
        end
        del_pd = p_d - p_d_prev;
        p_d_prev = p_d;
        return;
    end

    % Derived timing
    t_descend = 1 / frequency;
    T_osc = n_cycles / frequency;

    % Phase boundaries
    t1 = t_hold;                      % end of hold
    t2 = t1 + t_descend;             % end of descent
    t3 = t2 + T_osc;                 % end of oscillation

    % One-step ahead: evaluate trajectory at t + Ts
    t_next = t + Ts;

    if t_next <= t1
        % Phase 1: Hold at h_init (EKF warm-up)
        h = h_init;
    elseif t_next <= t2
        % Phase 2: Cosine descent from h_init to h_bottom
        t_desc = t_next - t1;
        h = h_bottom + (h_init - h_bottom) * (1 + cos(pi * t_desc / t_descend)) / 2;
    elseif t_next <= t3
        % Phase 3: Cosine oscillation with h_bottom as trough
        t_osc = t_next - t2;
        h = (h_bottom + amplitude) - amplitude * cos(2 * pi * frequency * t_osc);
    else
        % Phase 4: Hold at h_bottom
        h = h_bottom;
    end

    % Convert h to world coordinates
    p_d = (pz + h) * w_hat;

    % Compute trajectory increment del_pd = p_d[k+1] - p_d[k]
    if isempty(p_d_prev)
        p_d_prev = p0;      % p_d[0] = p0
    end
    del_pd = p_d - p_d_prev;
    p_d_prev = p_d;
end
