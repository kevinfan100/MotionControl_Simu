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

    % Optional override for Phase 2 descent duration (osc trajectory).
    % Default: t_descend = 1/frequency (used if override is missing or empty).
    if isfield(config, 't_descend_override') && ~isempty(config.t_descend_override)
        traj.t_descend_override = config.t_descend_override;
    else
        traj.t_descend_override = 0;   % 0 = use default (1/frequency)
    end

    % Convert trajectory_type string to numeric for Simulink Bus
    %   1 = osc (hold -> descent -> oscillation -> hold)
    %   2 = positioning (hold at h_init)
    %   3 = ramp_descent (linear descent from h_init to h_bottom over T_sim)
    if isfield(config, 'trajectory_type')
        switch config.trajectory_type
            case 'positioning'
                traj.trajectory_type = 2;
            case 'ramp_descent'
                traj.trajectory_type = 3;
            otherwise
                traj.trajectory_type = 1;  % default: osc
        end
    else
        traj.trajectory_type = 1;  % default: osc
    end

end
