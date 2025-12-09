function params_slx = convert_params_for_simulink(params)
%CONVERT_PARAMS_FOR_SIMULINK Convert params to Simulink-compatible format
%
%   params_slx = convert_params_for_simulink(params)
%
%   Converts string fields to numeric encoding for Simulink Bus compatibility.
%
%   String to numeric encoding:
%       traj.type:      'z_move' -> 0, 'xy_circle' -> 1
%       traj.direction: 'away' -> 0, 'toward' -> 1
%       ctrl.enable:    false -> 0, true -> 1
%       thermal.enable: false -> 0, true -> 1
%
%   Inputs:
%       params     - Parameter structure from calc_simulation_params
%
%   Outputs:
%       params_slx - Simulink-compatible parameter structure

    % Copy structure
    params_slx = params;

    % Convert traj.type
    switch params.traj.type
        case 'z_move'
            params_slx.traj.type = 0;
        case 'xy_circle'
            params_slx.traj.type = 1;
        otherwise
            error('Unknown trajectory type: %s', params.traj.type);
    end

    % Convert traj.direction
    switch params.traj.direction
        case 'away'
            params_slx.traj.direction = 0;
        case 'toward'
            params_slx.traj.direction = 1;
        otherwise
            error('Unknown direction: %s', params.traj.direction);
    end

    % Convert ctrl.enable (logical to double)
    params_slx.ctrl.enable = double(params.ctrl.enable);

    % Convert thermal.enable (logical to double)
    params_slx.thermal.enable = double(params.thermal.enable);

end
