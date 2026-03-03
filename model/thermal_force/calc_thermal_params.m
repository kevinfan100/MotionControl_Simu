function thermal = calc_thermal_params(config, constants)
%CALC_THERMAL_PARAMS Calculate thermal force parameters
%
%   thermal = calc_thermal_params(config, constants)
%
%   Inputs:
%       config    - User config with field: thermal_enable
%       constants - Physical constants with fields: k_B, T, gamma_N, Ts
%
%   Outputs:
%       thermal - Struct with fields:
%           enable, k_B, T, Ts, variance_coeff, seed

    thermal.enable = double(config.thermal_enable);
    thermal.k_B = constants.k_B;
    thermal.T = constants.T;
    thermal.Ts = constants.Ts;
    thermal.variance_coeff = 4 * constants.k_B * constants.T * constants.gamma_N / constants.Ts;
    thermal.seed = randi(2^31-1);

end
