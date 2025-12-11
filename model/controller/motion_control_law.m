function f_d = motion_control_law(p_d, p_m, params)
%MOTION_CONTROL_LAW Position-dependent discrete-time motion control
%
%   f_d = motion_control_law(p_d, p_m, params)
%
%   Control Law:
%       f_d[k] = (gamma / Ts) * {p_d[k] - lambda_c * p_d[k-1] - (1-lambda_c) * p_feedback[k]}
%
%   When noise_filter_enable = true:
%       p_feedback = p_m_filtered (low-pass filtered position)
%   When noise_filter_enable = false:
%       p_feedback = p_m (raw measured position)
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
%       params.ctrl.noise_filter_enable - Enable low-pass filter on feedback
%       params.ctrl.filter_alpha - IIR filter coefficient

    % Check if control is enabled (0 = disabled, 1 = enabled)
    if params.ctrl.enable < 0.5
        % Open-loop mode: no control force
        f_d = zeros(3, 1);
        return;
    end

    % Closed-loop mode
    persistent p_d_prev p_m_filtered_prev initialized

    % Initialize on first call
    if isempty(initialized)
        initialized = true;
        p_d_prev = p_d;
        p_m_filtered_prev = p_m;
    end

    % Extract control parameters
    gamma = params.ctrl.gamma;
    lambda_c = params.ctrl.lambda_c;
    Ts = params.ctrl.Ts;
    filter_enable = params.ctrl.noise_filter_enable > 0.5;
    alpha = params.ctrl.filter_alpha;

    % Low-pass IIR filter for p_m
    % p_m_filtered[k] = alpha * p_m[k] + (1 - alpha) * p_m_filtered[k-1]
    p_m_filtered = alpha * p_m + (1 - alpha) * p_m_filtered_prev;

    % Select feedback source
    if filter_enable
        p_feedback = p_m_filtered;  % Use filtered position
    else
        p_feedback = p_m;           % Use raw position
    end

    % Compute control force
    % f_d[k] = (gamma / Ts) * {p_d[k] - lambda_c * p_d[k-1] - (1-lambda_c) * p_feedback[k]}
    f_d = (gamma / Ts) * (p_d - lambda_c * p_d_prev - (1 - lambda_c) * p_feedback);

    % Update states for next iteration
    p_d_prev = p_d;
    p_m_filtered_prev = p_m_filtered;
end
