function wall = calc_wall_params(config, constants)
%CALC_WALL_PARAMS Calculate wall geometry parameters
%
%   wall = calc_wall_params(config, constants)
%
%   Inputs:
%       config    - User config with fields: theta_x [deg], theta_y [deg], pz, h_min
%       constants - Physical constants with field: R
%
%   Outputs:
%       wall - Struct with fields:
%           theta_x, theta_y (stored in rad), pz, h_min, h_bar_min, V, u_hat, v_hat, w_hat
%
%   Wall orientation (2026-10-15, same convention as Estimator_and_Control__.pdf p.6): the local coordinate
%   (u, v along the wall, w along the normal) is the measurement coordinate (x, y, z) turned first by theta_x
%   about x, then by theta_y about y:  V = R_y(theta_y) * R_x(theta_x) = [u_hat v_hat w_hat].
%   V maps local to measurement (dp = V * dp_local); V' maps measurement to local.
%   Zero angles: u = x, v = y, w = z. theta_y > 0 tilts the normal toward +x, theta_x > 0 toward -y.
%   Replaces the azimuth/polar pair (theta, phi), whose u_hat, v_hat were -x, -y at zero angles; a config that
%   still sets a nonzero theta or phi is refused rather than silently reinterpreted.

    if (isfield(config, 'theta') && config.theta ~= 0) || (isfield(config, 'phi') && config.phi ~= 0)
        error('calc_wall_params:legacyAngles', ['config.theta / config.phi (azimuth, polar) were replaced on 2026-10-15 by ' ...
              'config.theta_x / config.theta_y (rotation about x, then y). Old (theta, phi) maps to ' ...
              'theta_x = -asind(sind(theta)*sind(phi)), theta_y = atan2d(cosd(theta)*sind(phi), cosd(phi)).']);
    end

    tx = deg2rad(config.theta_x);
    ty = deg2rad(config.theta_y);
    Rx = [1 0 0; 0 cos(tx) -sin(tx); 0 sin(tx) cos(tx)];
    Ry = [cos(ty) 0 sin(ty); 0 1 0; -sin(ty) 0 cos(ty)];
    V = Ry * Rx;

    wall.theta_x = tx;
    wall.theta_y = ty;
    wall.pz = config.pz;
    wall.h_min = config.h_min;
    wall.h_bar_min = config.h_min / constants.R;
    wall.w_hat = V(:, 3);                      % field order = WallBus element order (calc_simulation_params)
    wall.u_hat = V(:, 1);
    wall.v_hat = V(:, 2);
    wall.enable_wall_effect = double(config.enable_wall_effect);
    wall.V = V;

end
