function traj = calc_traj_params(config)
%CALC_TRAJ_PARAMS Package trajectory parameters
%
%   traj = calc_traj_params(config)
%
%   Inputs:
%       config - User config with fields: h_init, amplitude,
%                frequency, n_cycles
%
%   Outputs:
%       traj - Struct with fields:
%           h_init, amplitude, frequency, n_cycles

    traj.h_init = config.h_init;
    traj.amplitude = config.amplitude;
    traj.frequency = config.frequency;
    traj.n_cycles = config.n_cycles;

end
