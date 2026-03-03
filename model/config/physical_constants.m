function c = physical_constants()
%PHYSICAL_CONSTANTS Return fixed physical constants for the simulation
%
%   c = physical_constants()
%
%   Returns a struct with fields:
%       R       - Particle radius [um]
%       gamma_N - Stokes drag coefficient [pN*sec/um]
%       Ts      - Sampling period [sec]
%       k_B     - Boltzmann constant [pN*um/K]
%       T       - Temperature [K] (37 C)

    c.R = 2.25;                    % Particle radius [um]
    c.gamma_N = 0.0425;            % Stokes drag coefficient [pN*sec/um]
    c.Ts = 1/1600;                 % Sampling period [sec]
    c.k_B = 1.3806503e-5;          % Boltzmann constant [pN*um/K]
    c.T = 310.15;                  % Temperature [K] (37 C)

end
