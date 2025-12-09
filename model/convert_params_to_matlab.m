function params_m = convert_params_to_matlab(params)
%CONVERT_PARAMS_TO_MATLAB Convert Simulink numeric parameters to MATLAB format
%
%   params_m = convert_params_to_matlab(params)
%
%   Converts numeric encodings back to MATLAB-native types:
%   - traj.type: 0 -> 'z_move', 1 -> 'xy_circle'
%   - traj.direction: 0 -> 'away', 1 -> 'toward'
%   - ctrl.enable: double -> logical
%   - thermal.enable: double -> logical

    params_m = params;

    % traj.type: 0 -> 'z_move', 1 -> 'xy_circle'
    if params.traj.type == 0
        params_m.traj.type = 'z_move';
    else
        params_m.traj.type = 'xy_circle';
    end

    % traj.direction: 0 -> 'away', 1 -> 'toward'
    if params.traj.direction == 0
        params_m.traj.direction = 'away';
    else
        params_m.traj.direction = 'toward';
    end

    % ctrl.enable: double -> logical
    params_m.ctrl.enable = logical(params.ctrl.enable);

    % thermal.enable: double -> logical
    params_m.thermal.enable = logical(params.thermal.enable);

end
