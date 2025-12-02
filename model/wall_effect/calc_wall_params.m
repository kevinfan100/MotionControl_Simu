function params = calc_wall_params(theta, phi, p_z, Ts, p0)
%CALC_WALL_PARAMS Calculate offline/fixed parameters for wall effect model
%
%   params = calc_wall_params(theta, phi, p_z, Ts, p0)
%
%   Inputs:
%       theta - Inclined plane angle around z-axis [rad]
%       phi   - Inclined plane angle around x-axis [rad]
%       p_z   - Distance from origin to inclined plane [um]
%       Ts    - Sampling time [sec]
%       p0    - Initial position [um], 3x1 vector
%
%   Output:
%       params - Simulink.Parameter with Bus type for Simulink integration
%
%   Reference: Drag_MATLAB.pdf

    %% Fixed constants
    R       = 2.25;         % Radius of magnetic particle [um] (FIXED)
    gamma_N = 0.0425;       % Nominal drag coefficient [pN*sec/um] (FIXED)

    %% Store basic parameters in struct
    params_data.theta = theta;
    params_data.phi = phi;
    params_data.p_z = p_z;
    params_data.R = R;
    params_data.gamma_N = gamma_N;
    params_data.Ts = Ts;
    params_data.p0 = p0;

    %% Calculate plane normal vector w_hat (Eq. from PDF Step 2)
    % w_hat = [cos(theta)*sin(phi); sin(theta)*sin(phi); cos(phi)]
    params_data.w_hat = [cos(theta) * sin(phi);
                         sin(theta) * sin(phi);
                         cos(phi)];

    %% Calculate outer product matrix W (Eq. from PDF Step 3)
    % W = w_hat * w_hat^T
    params_data.W = params_data.w_hat * params_data.w_hat';

    %% Store identity matrix for convenience
    params_data.I = eye(3);

    %% ========== Create Bus Object for Simulink ==========
    WallEffectParamsBus = Simulink.Bus;
    WallEffectParamsBus.Description = 'Wall Effect Parameters Structure';

    % Define Bus Elements (10 fields)
    elems = Simulink.BusElement.empty(10, 0);

    % 1. theta (scalar)
    elems(1) = Simulink.BusElement;
    elems(1).Name = 'theta';
    elems(1).DataType = 'double';

    % 2. phi (scalar)
    elems(2) = Simulink.BusElement;
    elems(2).Name = 'phi';
    elems(2).DataType = 'double';

    % 3. p_z (scalar)
    elems(3) = Simulink.BusElement;
    elems(3).Name = 'p_z';
    elems(3).DataType = 'double';

    % 4. R (scalar)
    elems(4) = Simulink.BusElement;
    elems(4).Name = 'R';
    elems(4).DataType = 'double';

    % 5. gamma_N (scalar)
    elems(5) = Simulink.BusElement;
    elems(5).Name = 'gamma_N';
    elems(5).DataType = 'double';

    % 6. Ts (scalar)
    elems(6) = Simulink.BusElement;
    elems(6).Name = 'Ts';
    elems(6).DataType = 'double';

    % 7. p0 (3x1 vector)
    elems(7) = Simulink.BusElement;
    elems(7).Name = 'p0';
    elems(7).Dimensions = [3 1];
    elems(7).DataType = 'double';

    % 8. w_hat (3x1 vector)
    elems(8) = Simulink.BusElement;
    elems(8).Name = 'w_hat';
    elems(8).Dimensions = [3 1];
    elems(8).DataType = 'double';

    % 9. W (3x3 matrix)
    elems(9) = Simulink.BusElement;
    elems(9).Name = 'W';
    elems(9).Dimensions = [3 3];
    elems(9).DataType = 'double';

    % 10. I (3x3 matrix)
    elems(10) = Simulink.BusElement;
    elems(10).Name = 'I';
    elems(10).Dimensions = [3 3];
    elems(10).DataType = 'double';

    WallEffectParamsBus.Elements = elems;

    % Save Bus Object to base workspace
    assignin('base', 'WallEffectParamsBus', WallEffectParamsBus);

    %% ========== Wrap as Simulink.Parameter ==========
    params = Simulink.Parameter(params_data);
    params.DataType = 'Bus: WallEffectParamsBus';
    params.Description = 'Wall Effect Parameters';

    %% Display summary
    fprintf('=== Wall Effect Parameters ===\n');
    fprintf('Inclined plane: theta=%.4f rad, phi=%.4f rad, p_z=%.2f um\n', theta, phi, p_z);
    fprintf('Particle radius R=%.2f um\n', R);
    fprintf('Nominal drag coefficient gamma_N=%.4f pN*sec/um\n', gamma_N);
    fprintf('Sampling time Ts=%.6f sec\n', Ts);
    fprintf('w_hat = [%.4f; %.4f; %.4f]\n', params_data.w_hat(1), params_data.w_hat(2), params_data.w_hat(3));
    fprintf('Bus Object: WallEffectParamsBus created in workspace\n');
    fprintf('==============================\n');

end
