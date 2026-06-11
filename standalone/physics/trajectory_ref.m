function [p_d, del_pd] = trajectory_ref(t, params)
%TRAJECTORY_REF Desired position p_d[k+1] along the wall normal.
%
%   [p_d, del_pd] = trajectory_ref(t, params)
%
%   Supported trajectory types (params.traj.trajectory_type):
%       2 = positioning   (hold at h_init for the whole run)
%       3 = ramp_descent  (linear descent h_init -> h_bottom over T_sim)
%   The mother repo's oscillation type (1) is outside the package scenario
%   set and intentionally not carried over (errors out if requested).
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
%
%   Source: verbatim subset of model/trajectory/trajectory_generator.m
%   (packaging part 2; types 2/3 logic unchanged, type 1 removed).

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

    error('trajectory_ref:unsupportedType', ...
          ['trajectory_type %g not in package scope (2 = positioning, ' ...
           '3 = ramp_descent; oscillation lives in the mother repo).'], ...
          trajectory_type);
end
