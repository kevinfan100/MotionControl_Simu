function ctrl = calc_ctrl_params(config, constants)
%CALC_CTRL_PARAMS Calculate controller parameters including filter coefficient
%
%   ctrl = calc_ctrl_params(config, constants)
%
%   Inputs:
%       config    - User config with fields: ctrl_enable, lambda_c,
%                   noise_filter_enable, noise_filter_cutoff,
%                   meas_noise_enable, meas_noise_std
%       constants - Physical constants with fields: gamma_N, Ts
%
%   Outputs:
%       ctrl - Struct with fields:
%           enable, lambda_c, gamma, Ts, noise_filter_enable,
%           noise_filter_cutoff, filter_alpha, meas_noise_enable,
%           meas_noise_std, meas_noise_seed

    Ts = constants.Ts;

    ctrl.enable = double(config.ctrl_enable);
    ctrl.lambda_c = config.lambda_c;
    ctrl.gamma = constants.gamma_N;
    ctrl.Ts = Ts;
    ctrl.noise_filter_enable = double(config.noise_filter_enable);
    ctrl.noise_filter_cutoff = config.noise_filter_cutoff;

    % Pre-calculate filter coefficient: alpha = Ts / (Ts + 1/(2*pi*fc))
    fc = config.noise_filter_cutoff;
    ctrl.filter_alpha = Ts / (Ts + 1/(2*pi*fc));

    % Measurement noise parameters
    ctrl.meas_noise_enable = double(config.meas_noise_enable);
    ctrl.meas_noise_std = config.meas_noise_std;
    ctrl.meas_noise_seed = randi(2^31-1);

end
