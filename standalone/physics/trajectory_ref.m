function [p_d, del_pd] = trajectory_ref(t, params)
%TRAJECTORY_REF Desired position p_d[k+1] along the wall normal.
%
%   [p_d, del_pd] = trajectory_ref(t, params)
%
%   Supported trajectory types (params.traj.trajectory_type):
%       1 = oscillation   (4-phase: hold -> cosine descent -> cosine osc
%                          with h_bottom as trough -> hold; the osc1hz
%                          scenario, re-added in packaging 9c)
%       2 = positioning   (hold at h_init for the whole run)
%       3 = ramp_descent  (linear descent h_init -> h_bottom over T_sim)
%
%   Outputs:
%       p_d    - Desired position p_d[k+1] [3x1, um, world coords]
%       del_pd - Increment p_d[k+1] - p_d[k] [3x1, um]
%
%   Note: output is one sample AHEAD of t; the driver keeps a unit-delay
%   buffer (IC = p0) to obtain p_d[k] for the controller and logging.
%   Persistent p_d_prev -> reset between runs via `clear trajectory_ref`
%   (the driver does this).
%
%   Params fields read here (param-flow contract):
%       params.common.p0 / Ts / T_sim
%       params.wall.w_hat / pz
%       params.traj.t_hold / h_init / h_bottom / trajectory_type
%       params.traj.amplitude / frequency / n_cycles / t_descend_override
%                                            (oscillation type 1 only)
%
%   Source: verbatim subset of model/trajectory/trajectory_generator.m
%   (types 1/2/3 logic unchanged; osc re-added in packaging 9c).

    persistent p_d_prev

    p0 = params.common.p0;
    w_hat = params.wall.w_hat;
    pz = params.wall.pz;
    Ts = params.common.Ts;

    h_init    = params.traj.h_init;
    h_bottom  = params.traj.h_bottom;
    trajectory_type = params.traj.trajectory_type;

    % Ramp descent mode: linear descent from h_init to h_bottom over T_sim
    %   h(t) = max(h_init - rate * t, h_bottom)
    %   rate = (h_init - h_bottom) / T_sim   (auto-fits descent end at T_sim)
    if trajectory_type > 2.5
        T_sim_local = params.common.T_sim;
        if T_sim_local <= 0
            rate = 0;
        else
            rate = (h_init - h_bottom) / T_sim_local;
        end
        t_next = t + Ts;
        h = max(h_init - rate * t_next, h_bottom);
        p_d = (pz + h) * w_hat;

        if isempty(p_d_prev)
            p_d_prev = p0;
        end
        del_pd = p_d - p_d_prev;
        p_d_prev = p_d;
        return;
    end

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

    % Oscillation mode (type 1): 4-phase hold -> cosine descent -> cosine
    % oscillation (h_bottom as trough) -> hold. Faithful port of the mother
    % trajectory_generator.m phases 1-4.
    t_hold    = params.traj.t_hold;
    amplitude = params.traj.amplitude;
    frequency = params.traj.frequency;
    n_cycles  = params.traj.n_cycles;
    if isfield(params.traj, 't_descend_override') && params.traj.t_descend_override > 0
        t_descend = params.traj.t_descend_override;
    else
        t_descend = 1 / frequency;
    end
    T_osc = n_cycles / frequency;

    t1 = t_hold;              % end of hold
    t2 = t1 + t_descend;     % end of descent
    t3 = t2 + T_osc;         % end of oscillation
    t_next = t + Ts;         % one-step-ahead, same as the other modes

    if t_next <= t1
        h = h_init;                                              % Phase 1: hold
    elseif t_next <= t2
        t_desc = t_next - t1;                                    % Phase 2: descend
        h = h_bottom + (h_init - h_bottom) * (1 + cos(pi * t_desc / t_descend)) / 2;
    elseif t_next <= t3
        t_osc = t_next - t2;                                     % Phase 3: osc
        h = (h_bottom + amplitude) - amplitude * cos(2 * pi * frequency * t_osc);
    else
        h = h_bottom;                                            % Phase 4: hold
    end
    p_d = (pz + h) * w_hat;

    if isempty(p_d_prev)
        p_d_prev = p0;
    end
    del_pd = p_d - p_d_prev;
    p_d_prev = p_d;
end
