function wall = calc_wall_params(config, constants)
%CALC_WALL_PARAMS Calculate wall geometry parameters
%
%   wall = calc_wall_params(config, constants)
%
%   Inputs:
%       config    - User config with fields: theta [deg], phi [deg], pz, h_min
%       constants - Physical constants with field: R
%
%   Outputs:
%       wall - Struct with fields:
%           theta, phi (stored in rad), pz, h_min, h_bar_min, w_hat, u_hat, v_hat

    % Convert degrees to radians
    theta = deg2rad(config.theta);
    phi = deg2rad(config.phi);

    wall.theta = theta;
    wall.phi = phi;
    wall.pz = config.pz;
    wall.h_min = config.h_min;
    wall.h_bar_min = config.h_min / constants.R;
    wall.w_hat = [cos(theta)*sin(phi); sin(theta)*sin(phi); cos(phi)];
    wall.u_hat = [-cos(theta)*cos(phi); -sin(theta)*cos(phi); sin(phi)];
    wall.v_hat = [sin(theta); -cos(theta); 0];
    wall.enable_wall_effect = double(config.enable_wall_effect);

end
