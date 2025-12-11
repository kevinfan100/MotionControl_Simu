function F_th = calc_thermal_force(p, params)
%CALC_THERMAL_FORCE Generate random thermal force (Brownian motion)
%
%   F_th = calc_thermal_force(p, params)
%
%   Generates a random thermal force based on the particle position,
%   accounting for wall effects on the diffusion coefficient.
%
%   Inputs:
%       p      - Particle position [3x1 vector, um]
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       F_th   - Random thermal force [3x1 vector, pN]
%
%   Required params fields:
%       params.wall.w_hat      - Wall normal vector [3x1]
%       params.wall.u_hat      - Wall parallel vector 1 [3x1]
%       params.wall.v_hat      - Wall parallel vector 2 [3x1]
%       params.wall.pz         - Wall displacement [um]
%       params.common.R        - Particle radius [um]
%       params.common.gamma_N  - Stokes drag coefficient [pN*sec/um]
%       params.thermal.k_B     - Boltzmann constant [pN*um/K]
%       params.thermal.T       - Temperature [K]
%       params.thermal.Ts      - Sampling period [sec]
%       params.thermal.seed    - Random seed for reproducibility
%

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

    % Calculate normalized distance from wall
    h = dot(p, w_hat) - pz;
    h_bar = h / R;

    % Calculate correction coefficients using Wall Effect module
    [c_para, c_perp] = calc_correction_functions(h_bar);

    % C = c_para * (u_hat + v_hat) + c_perp * w_hat
    C = c_para * (u_hat + v_hat) + c_perp * w_hat;

    % Calculate variance for each direction
    % Variance = (4 * k_B * T * gamma_N / Ts) * C 
    variance_coeff = 4 * k_B * T * gamma_N / Ts;
    Variance = variance_coeff * abs(C);

    % Generate random thermal force
    % F_th ~ N(0, Variance) => F_th = sqrt(Variance) .* randn(3,1)
    F_th = sqrt(Variance) .* randn(3, 1);

end
