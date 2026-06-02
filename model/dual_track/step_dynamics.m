function p_next = step_dynamics(p, F_total, params, Ts)
%STEP_DYNAMICS Advance continuous-dynamics state by one discrete sample (Ts).
%
%   p_next = step_dynamics(p, F_total, params, Ts)
%
%   Implements the continuous part of the Simulink model
%       p_dot(t) = Gamma_inv(p(t)) * F_total
%   integrated from t to t+Ts using fixed-step ode4 (Runge-Kutta 4),
%   with inner step ~10 us (62-63 substeps per Ts=625 us).
%
%   F_total is held constant across the Ts (matches Simulink ZOH input
%   to the Drag_coeff_matrix block).
%
%   Inputs:
%       p        - Particle position at start of interval [3x1, um]
%       F_total  - Total force, constant over the Ts [3x1, pN]
%       params   - Resolved parameter struct (params.Value form), needed
%                  for calc_gamma_inv: params.wall.*, params.common.*
%       Ts       - Discrete sampling period [sec]
%
%   Output:
%       p_next   - Particle position at t+Ts [3x1, um]
%
%   Design note (agent_docs/dual-track-simulation-design.md, decision 3):
%       Inner step = 10 us was chosen to be 10x larger than Simulink
%       (1 us) while keeping ode4 algorithm identity. RMS error vs
%       Simulink expected < 1e-10 for the typical trajectory parameters
%       (f<=1 Hz, A<=10 um, h_bar>=2).
%
%   See also: calc_gamma_inv, run_pure_simulation

    inner_step_target = 10e-6;                         % [sec] design target
    N_inner = max(1, round(Ts / inner_step_target));   % integer substeps
    h = Ts / N_inner;                                   % actual inner step [sec]

    p_curr = p;
    for j = 1:N_inner
        % Classical RK4 (ode4) — F_total held constant within the Ts
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

    Gamma_inv = calc_gamma_inv(p, params);   % 3x3 [um/(pN*sec)]
    p_dot = Gamma_inv * F_total;             % 3x1 [um/sec]
end
