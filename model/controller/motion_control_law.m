function f_d = motion_control_law(p_d, p_m, params)
%MOTION_CONTROL_LAW Position-dependent discrete-time motion control
%
%   f_d = motion_control_law(p_d, p_m, params)
%
%   Implements the control law for magnetic bead position control.
%   Based on Meng & Menq (2023) "Ultra-Precise High-Speed Untethered
%   Manipulation", Equation 17 (simplified version).
%
%   Control Law:
%       f_d[k] = (gamma / Ts) * {p_d[k] - lambda_c * p_d[k-1] - (1-lambda_c) * p_m[k]}
%
%   Inputs:
%       p_d    - Current desired position [3x1 vector, um]
%       p_m    - Current measured position [3x1 vector, um]
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       f_d    - Control force [3x1 vector, pN]
%
%   Required params fields:
%       params.ctrl.enable   - Enable control (true/false or 1/0)
%       params.ctrl.gamma    - Drag coefficient [pN*sec/um]
%       params.ctrl.lambda_c - Closed-loop pole (0 < lambda_c < 1)
%       params.ctrl.Ts       - Sampling period [sec]
%
%   State management:
%       Uses persistent variable p_d_prev to store previous desired position.
%       On first call, initializes p_d_prev = p_d, which gives f_d[0] = 0
%       when p_d[0] = p_m[0] (system starts at rest at initial position).
%
%   Control modes:
%       enable = true (1):  Closed-loop control active
%       enable = false (0): Open-loop mode, returns f_d = [0; 0; 0]

    % Check if control is enabled (0 = disabled, 1 = enabled)
    if params.ctrl.enable < 0.5
        % Open-loop mode: no control force
        f_d = zeros(3, 1);
        return;
    end

    % Closed-loop mode
    persistent p_d_prev initialized

    % Initialize on first call
    if isempty(initialized)
        initialized = true;
        % Initialize p_d_prev = p_d (current desired position)
        % This ensures f_d[0] = (gamma/Ts) * {p_d - lambda_c*p_d - (1-lambda_c)*p_m}
        %                     = (gamma/Ts) * (1-lambda_c) * (p_d - p_m)
        % If p_d[0] = p_m[0] = p0, then f_d[0] = 0 (system at rest)
        p_d_prev = p_d;
    end

    % Extract control parameters
    gamma = params.ctrl.gamma;
    lambda_c = params.ctrl.lambda_c;
    Ts = params.ctrl.Ts;

    % Compute control force
    % f_d[k] = (gamma / Ts) * {p_d[k] - lambda_c * p_d[k-1] - (1-lambda_c) * p_m[k]}
    f_d = (gamma / Ts) * (p_d - lambda_c * p_d_prev - (1 - lambda_c) * p_m);

    % Update state for next iteration
    p_d_prev = p_d;
end
