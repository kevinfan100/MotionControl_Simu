function wall = calc_wall_params(config, constants)
%CALC_WALL_PARAMS Calculate wall geometry parameters
%
%   wall = calc_wall_params(config, constants)
%
%   Inputs:
%       config    - User config with fields: theta, phi, pz, h_min
%       constants - Physical constants with field: R
%
%   Outputs:
%       wall - Struct with fields:
%           theta, phi, pz, h_min, h_bar_min, w_hat, u_hat, v_hat

    theta = config.theta;
    phi = config.phi;

    wall.theta = theta;
    wall.phi = phi;
    wall.pz = config.pz;
    wall.h_min = config.h_min;
    wall.h_bar_min = config.h_min / constants.R;
    wall.w_hat = [cos(theta)*sin(phi); sin(theta)*sin(phi); cos(phi)];
    wall.u_hat = [-cos(theta)*cos(phi); -sin(theta)*cos(phi); sin(phi)];
    wall.v_hat = [sin(theta); -cos(theta); 0];

end
