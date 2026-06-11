function f_th = thermal_force(p, params)
%THERMAL_FORCE Random thermal (Brownian) force with wall-corrected variance.
%
%   f_th = thermal_force(p, params)
%
%   Per-step white thermal force, one-sided integration convention:
%   Var(f_th) = 4*k_B*T*gamma/Ts per axis, with gamma = gamma_N * c(h_bar)
%   anisotropic near the wall (free-space MSD ratio vs Einstein 2*D*Ts
%   is 2.0 by this convention -- see gates_part1 T4).
%
%   Inputs:
%       p      - Particle position [3x1, um]
%       params - Nested parameter struct (built by config.m)
%
%   Output:
%       f_th   - Thermal force sample [3x1, pN]
%
%   Params fields read here (param-flow contract):
%       params.wall.w_hat / u_hat / v_hat  - Wall frame unit vectors [3x1]
%       params.wall.pz                     - Wall displacement [um]
%       params.wall.enable_wall_effect     - >0.5 = wall effect ON
%       params.common.R                    - Particle radius [um]
%       params.common.gamma_N              - Stokes drag [pN*sec/um]
%       params.thermal.k_B                 - Boltzmann constant [pN*um/K]
%       params.thermal.T                   - Temperature [K]
%       params.thermal.Ts                  - Sampling period [sec]
%       params.thermal.seed                - Derived stream seed (see below)
%
%   RNG seeding chain (kept verbatim from the mother repo so that
%   equivalence gates stay bit-exact):
%       driver rng(seed)  ->  params builder draws thermal.seed = randi(...)
%       ->  first call here re-seeds the global stream with thermal.seed.
%   All in-loop randomness (thermal + measurement noise) then flows from
%   that single derived stream. Reset between runs via `clear thermal_force`.
%
%   Scope note: the anisotropic combination C below is exact only for the
%   default wall orientation (w_hat = [0;0;1]); tilted walls are outside
%   the package envelope (see KNOWN_ISSUES).
%
%   Source: verbatim from model/thermal_force/calc_thermal_force.m
%   (packaging part 1; logic unchanged).

    % Initialize random seed on first call (persistent variable)
    persistent rng_initialized
    if isempty(rng_initialized)
        if isfield(params.thermal, 'seed')
            rng(params.thermal.seed);
        end
        rng_initialized = true;
    end

    % Extract parameters
    w_hat = params.wall.w_hat;
    u_hat = params.wall.u_hat;
    v_hat = params.wall.v_hat;
    pz = params.wall.pz;
    R = params.common.R;
    gamma_N = params.common.gamma_N;
    k_B = params.thermal.k_B;
    T = params.thermal.T;
    Ts = params.thermal.Ts;
    enable_wall = params.wall.enable_wall_effect;

    if enable_wall > 0.5
        % Wall effect ON: anisotropic variance
        h = dot(p, w_hat) - pz;
        h_bar = h / R;

        % Calculate correction coefficients using the shared wall module
        [c_para, c_perp] = wall_corrections(h_bar);

        % C = c_para * (u_hat + v_hat) + c_perp * w_hat
        C = c_para * (u_hat + v_hat) + c_perp * w_hat;
    else
        % Wall effect OFF: isotropic variance (c_para = c_perp = 1)
        C = u_hat + v_hat + w_hat;
    end

    % Calculate variance for each direction
    % Variance = (4 * k_B * T * gamma_N / Ts) * C
    variance_coeff = 4 * k_B * T * gamma_N / Ts;
    Variance = variance_coeff * abs(C);

    % Generate random thermal force
    % f_th ~ N(0, Variance) => f_th = sqrt(Variance) .* randn(3,1)
    f_th = sqrt(Variance) .* randn(3, 1);

end
