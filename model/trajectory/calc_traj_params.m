function traj = calc_traj_params(config)
%CALC_TRAJ_PARAMS Package trajectory parameters
%
%   traj = calc_traj_params(config)
%
%   Inputs:
%       config - User config with fields: t_hold, h_init, h_bottom,
%                amplitude, frequency, n_cycles
%
%   Outputs:
%       traj - Struct with fields:
%           t_hold, h_init, h_bottom, amplitude, frequency, n_cycles

    traj.t_hold = config.t_hold;
    traj.h_init = config.h_init;
    traj.h_bottom = config.h_bottom;
    traj.amplitude = config.amplitude;
    traj.frequency = config.frequency;
    traj.n_cycles = config.n_cycles;

    % Convert trajectory_type string to numeric for Simulink Bus
    %   1 = osc (hold -> descent -> oscillation -> hold)
    %   2 = positioning (hold at h_init)
    if isfield(config, 'trajectory_type') && strcmp(config.trajectory_type, 'positioning')
        traj.trajectory_type = 2;
    else
        traj.trajectory_type = 1;  % default: osc
    end

end
