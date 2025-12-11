function p_d = trajectory_generator(t, p0, params)
%TRAJECTORY_GENERATOR Generate desired position at time t
%
%   p_d = trajectory_generator(t, p0, params)
%
%   Calculates the desired trajectory position based on the current time,
%   starting position, and trajectory parameters.
%
%   Inputs:
%       t      - Current time [sec]
%       p0     - Starting position [3x1 vector, um]
%       params - Parameter structure from calc_simulation_params
%
%   Outputs:
%       p_d    - Desired position [3x1 vector, um]
%
%   Required params fields:
%       params.traj.type       - Trajectory type ('z_sine' or 'xy_circle')
%
%   For 'z_sine' type:
%       params.traj.amplitude  - Oscillation amplitude [um]
%       params.traj.frequency  - Oscillation frequency [Hz]
%       params.traj.n_cycles   - Number of cycles
%
%   For 'xy_circle' type:
%       params.traj.radius     - Circle radius [um]
%       params.traj.period     - Circle period [sec]
%       params.traj.n_cycles   - Number of circles
%
%   Trajectory descriptions (WORLD COORDINATES):
%       'z_sine': Sinusoidal motion along world Z axis
%           - z(t) = p0_z + amplitude * sin(2*pi*frequency*t)
%           - Total time = n_cycles / frequency
%           - Returns to p0 after completion
%
%       'xy_circle': Circular motion in world XY plane
%           - Circle in X-Y plane
%           - Starts at p0, circles around, returns to p0
%           - Total time = period * n_cycles

    % Get trajectory type (0 = z_sine, 1 = xy_circle)
    traj_type = params.traj.type;

    if traj_type < 0.5  % z_sine
        p_d = trajectory_z_sine(t, p0, params);
    else  % xy_circle
        p_d = trajectory_xy_circle(t, p0, params);
    end
end


function p_d = trajectory_z_sine(t, p0, params)
%TRAJECTORY_Z_SINE Sinusoidal motion along world Z axis
    % World coordinate Z axis
    z_hat = [0; 0; 1];

    amplitude = params.traj.amplitude;
    frequency = params.traj.frequency;
    n_cycles = params.traj.n_cycles;

    % Total trajectory time
    T_total = n_cycles / frequency;

    if t <= T_total
        % Angular frequency
        omega = 2 * pi * frequency;

        % Sinusoidal displacement along Z axis
        displacement = amplitude * sin(omega * t);
        p_d = p0 + displacement * z_hat;
    else
        % After completing all cycles, stay at starting position
        p_d = p0;
    end
end


function p_d = trajectory_xy_circle(t, p0, params)
%TRAJECTORY_XY_CIRCLE Circular motion in world XY plane
    % World coordinate axes
    x_hat = [1; 0; 0];
    y_hat = [0; 1; 0];

    radius = params.traj.radius;
    period = params.traj.period;
    n_cycles = params.traj.n_cycles;

    % Total trajectory time
    T_total = period * n_cycles;

    if t <= T_total
        % Angular frequency
        omega = 2 * pi / period;

        % Circular motion in X-Y plane
        % At t=0: p_d = p0 (starts at origin of circle path)
        % Uses (cos(omega*t) - 1) to ensure p_d(0) = p0
        p_d = p0 + radius * (cos(omega * t) - 1) * x_hat ...
                 + radius * sin(omega * t) * y_hat;
    else
        % After completing all circles, stay at starting position
        p_d = p0;
    end
end
