function traj = calc_traj_params(config)
%CALC_TRAJ_PARAMS Package trajectory parameters with numeric type encoding
%
%   traj = calc_traj_params(config)
%
%   Inputs:
%       config - User config with fields: traj_type, h_init, amplitude,
%                frequency, n_cycles, radius, period
%
%   Outputs:
%       traj - Struct with fields:
%           type (0=z_sine, 1=xy_circle), h_init, amplitude, frequency,
%           n_cycles, radius, period

    switch config.traj_type
        case 'z_sine'
            traj.type = 0;
        case 'xy_circle'
            traj.type = 1;
        otherwise
            error('Unknown trajectory type: %s', config.traj_type);
    end

    traj.h_init = config.h_init;
    traj.amplitude = config.amplitude;
    traj.frequency = config.frequency;
    traj.n_cycles = config.n_cycles;
    traj.radius = config.radius;
    traj.period = config.period;

end
