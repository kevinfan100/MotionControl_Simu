function p_next = step_dynamics(p, F_total, params, Ts)
%STEP_DYNAMICS Advance continuous dynamics by one discrete sample (Ts).
%
%   p_next = step_dynamics(p, F_total, params, Ts)
%
%   Integrates the continuous plant
%       p_dot(t) = Gamma_inv(p(t)) * F_total
%   from t to t+Ts with fixed-step ode4 (classical RK4), inner step
%   ~10 us (62-63 substeps per Ts = 625 us). F_total is held constant
%   across the Ts (ZOH input, matching the discrete controller rate).
%
%   Inputs:
%       p        - Position at start of interval [3x1, um]
%       F_total  - Total force, constant over the Ts [3x1, pN]
%       params   - Nested parameter struct (gamma_inv reads
%                  params.wall.*, params.common.*)
%       Ts       - Discrete sampling period [sec]
%
%   Output:
%       p_next   - Position at t+Ts [3x1, um]
%
%   Design note: inner step 10 us = 10x the Simulink reference (1 us)
%   with the same ode4 algorithm; RMS error vs the 1 us reference
%   < 1e-10 validated for f <= 1 Hz, A <= 10 um, h_bar >= 2 (dual-track
%   design doc). The package's near-wall tail (1.2 < h_bar < 2) is
%   exercised by the ramp scenario gates; no separate 1 us reference
%   comparison exists there.
%
%   Source: verbatim from model/dual_track/step_dynamics.m (packaging
%   part 1; logic unchanged, calls gamma_inv instead of calc_gamma_inv).

    inner_step_target = 10e-6;                         % [sec] design target
    N_inner = max(1, round(Ts / inner_step_target));   % integer substeps
    h = Ts / N_inner;                                   % actual inner step [sec]

    p_curr = p;
    for j = 1:N_inner
        % Classical RK4 (ode4) -- F_total held constant within the Ts
        k1 = local_p_dot(p_curr,                F_total, params);
        k2 = local_p_dot(p_curr + (h/2) * k1,   F_total, params);
        k3 = local_p_dot(p_curr + (h/2) * k2,   F_total, params);
        k4 = local_p_dot(p_curr +  h    * k3,   F_total, params);
        p_curr = p_curr + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
    end

    p_next = p_curr;
end


function p_dot = local_p_dot(p, F_total, params)
%LOCAL_P_DOT  Right-hand side: p_dot = Gamma_inv(p) * F_total.

    Gi = gamma_inv(p, params);     % 3x3 [um/(pN*sec)]
    p_dot = Gi * F_total;          % 3x1 [um/sec]
end
