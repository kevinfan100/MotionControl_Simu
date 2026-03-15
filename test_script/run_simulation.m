%% run_simulation.m - Main Simulation Script
%
% Runs the complete motion control simulation with thesis-style visualization.

clear; close all; clc;
clear motion_control_law trajectory_generator;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
cd(project_root);

%% SECTION 1: Load Defaults and Override Parameters

addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'thermal_force'));
addpath(fullfile(project_root, 'model', 'trajectory'));
addpath(fullfile(project_root, 'model', 'controller'));

config = user_config();

% --- Wall Geometry ---
config.theta = 0;                    % Wall azimuth angle [deg]
config.phi   = 0;                    % Wall elevation angle [deg]
config.pz    = 0;                    % Wall displacement along w_hat [um]
config.h_min = 1.1 * 2.25;           % Minimum safe distance [um]

% --- Trajectory ---
config.h_init    = 5;                % Initial distance from wall [um]
config.amplitude = 2.5;              % Oscillation amplitude [um]
config.frequency = 1;                % Oscillation frequency [Hz]
config.n_cycles  = 3;                % Number of cycles

% --- Controller ---
config.ctrl_enable = true;           % Enable closed-loop control
config.lambda_c = 0.7;              % Closed-loop pole (0 < lambda_c < 1)
config.a_pd  = 0.1;                 % EMA: deterministic smoothing
config.a_prd = 0.1;                 % EMA: random-deterministic smoothing
config.a_cov = 0.1;                 % EMA: covariance smoothing
config.epsilon = 0.01;              % Anisotropy threshold for theta_m

% --- Measurement Noise ---
config.meas_noise_enable = true;
config.meas_noise_std = [0.00062; 0.000057; 0.00331];  % [um]

% --- Thermal Force ---
config.thermal_enable = true;

% --- Simulation Time ---
T_margin = 0.3;
T_traj = config.n_cycles / config.frequency;
config.T_sim = T_traj + T_margin;
T_sim = config.T_sim;

% --- Analysis Parameters ---
openloop_cutoff_freq = 5;            % Deterministic/Random cutoff [Hz]

% --- Thesis Figure Style ---
AXIS_LW = 3.0;
FONT_SIZE = 32;
LEGEND_FS = 24;
LINE_REF = 4;                       % Reference/true/desired
LINE_OUT = 3;                       % Output/estimated/error
COL_REF = [0.0 0.6 0.0];            % Green
COL_OUT = [0.8 0.0 0.0];            % Red
COL_ERR = [0.0 0.2 0.8];            % Blue

% Export style (scaled for traditional figures)
EXP_FS = 18;
EXP_LW = 2.0;
EXP_LFS = 14;
EXP_LR = 3;
EXP_LO = 2;

%% SECTION 2: Calculate Parameters

params = calc_simulation_params(config);

%% SECTION 3: Extract Initial Position and Safety Check

p0 = params.Value.common.p0;

fprintf('Initial position: [%.3f, %.3f, %.3f] um\n', p0);
fprintf('Initial h/R: %.2f\n', (dot(p0, params.Value.wall.w_hat) - params.Value.wall.pz) / params.Value.common.R);

[is_safe, h_min_actual, t_critical] = check_trajectory_safety(params.Value);

if ~is_safe
    warning('Trajectory unsafe! Min h = %.2f um at t = %.3f sec (threshold: %.2f um)', ...
        h_min_actual, t_critical, params.Value.wall.h_min);
    fprintf('Continuing anyway for demonstration...\n');
end

%% SECTION 4: Run Simulation

fprintf('\nStarting simulation...\n');
is_closed_loop = params.Value.ctrl.enable > 0.5;
is_thermal = params.Value.thermal.enable > 0.5;

ctrl_mode_str = 'Open-loop';
if is_closed_loop; ctrl_mode_str = 'Closed-loop'; end
thermal_str = 'Disabled';
if is_thermal; thermal_str = 'Enabled'; end
traj_type_str = 'sine (along w_hat)';

fprintf('  Mode: %s\n', ctrl_mode_str);
fprintf('  Thermal: %s\n', thermal_str);
fprintf('  Trajectory: %s\n', traj_type_str);
fprintf('  Duration: %.1f sec\n', T_sim);

assignin('base', 'params', params);
assignin('base', 'p0', p0);
assignin('base', 'Ts', params.Value.common.Ts);

simOut = sim(fullfile(project_root, 'model', 'system_model'), 'StopTime', num2str(T_sim), ...
    'SaveTime', 'on', 'TimeSaveName', 'tout', ...
    'SaveOutput', 'on', 'OutputSaveName', 'yout');

% --- Data Extraction ---
Ts = params.Value.common.Ts;
N_discrete = size(simOut.p_d_out, 1);
t_sample = (0:(N_discrete-1)) * Ts;

p_d_log = simOut.p_d_out';
f_d_log = simOut.f_d_out';
f_th_log = simOut.f_th_out';

t_cont = simOut.tout;
p_m_cont = simOut.p_m_out;
p_m_log = zeros(3, N_discrete);
for i = 1:N_discrete
    [~, idx] = min(abs(t_cont - t_sample(i)));
    p_m_log(:, i) = p_m_cont(idx, :)';
end

w_hat = params.Value.wall.w_hat;
u_hat = params.Value.wall.u_hat;
v_hat = params.Value.wall.v_hat;
pz_wall = params.Value.wall.pz;
R = params.Value.common.R;

h_bar_log = (p_m_log' * w_hat - pz_wall) / R;
h_bar_log = h_bar_log';

error_x = (p_m_log(1,:) - p_d_log(1,:)) * 1000;  % nm
error_y = (p_m_log(2,:) - p_d_log(2,:)) * 1000;
error_z = (p_m_log(3,:) - p_d_log(3,:)) * 1000;
error_3d = vecnorm(p_m_log - p_d_log, 2, 1);

% EKF diagnostic extraction (closed-loop only)
if is_closed_loop
    ekf_log = simOut.ekf_out';             % [4 x N]
    lamda_hat_log = ekf_log(1:2, :);       % [2 x N] [para; perp]
    theta_hat_log = ekf_log(3:4, :);       % [2 x N] [theta_x; theta_y]

    % True lambda from known geometry
    lambda_true_log = zeros(2, N_discrete);
    for i = 1:N_discrete
        h_val = (p_m_log(:,i)' * w_hat - pz_wall) / R;
        [c_para_i, c_perp_i] = calc_correction_functions(h_val);
        lambda_true_log(:, i) = [c_para_i; c_perp_i];
    end
end

N_samples = length(t_sample);
fprintf('Simulation completed.\n');

%% SECTION 5: Results Visualization (Tabbed Figure)

fprintf('\nGenerating figures...\n');

fig = uifigure('Name', 'Simulation Results', 'Position', [100 100 1400 900]);
tabgroup = uitabgroup(fig);
tabgroup.Units = 'normalized';
tabgroup.Position = [0 0 1 1];

fig_w = 1400; fig_h = 900;
plot_left = 0.18 * fig_w;
plot_w = 0.72 * fig_w;

% 2x1 layout positions
plot_h2 = 0.30 * fig_h;
pos_2x1 = {[plot_left, 0.58*fig_h, plot_w, plot_h2], ...
            [plot_left, 0.12*fig_h, plot_w, plot_h2]};

% 3x1 layout positions
plot_h3 = 0.22 * fig_h;
pos_3x1 = {[plot_left, 0.70*fig_h, plot_w, plot_h3], ...
            [plot_left, 0.40*fig_h, plot_w, plot_h3], ...
            [plot_left, 0.08*fig_h, plot_w, plot_h3]};

% ==================== Tab 1: 3D Trajectory ====================
tab1 = uitab(tabgroup, 'Title', '3D Trajectory');
ax1 = uiaxes(tab1);
ax1.Units = 'normalized';
ax1.Position = [0.10 0.08 0.85 0.84];

plot3(ax1, p_d_log(1,:), p_d_log(2,:), p_d_log(3,:), '-', 'Color', COL_REF, 'LineWidth', LINE_REF);
hold(ax1, 'on');
plot3(ax1, p_m_log(1,:), p_m_log(2,:), p_m_log(3,:), '-', 'Color', COL_OUT, 'LineWidth', LINE_OUT);
plot3(ax1, p0(1), p0(2), p0(3), 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'LineWidth', 2);

% Wall plane
wall_center = pz_wall * w_hat;
all_pos = [p_d_log, p_m_log];
traj_range = max([range(all_pos(1,:)), range(all_pos(2,:)), range(all_pos(3,:))]);
plane_size = max(traj_range * 1.5, 10);
corners = [
    wall_center + plane_size/2 * u_hat + plane_size/2 * v_hat, ...
    wall_center - plane_size/2 * u_hat + plane_size/2 * v_hat, ...
    wall_center - plane_size/2 * u_hat - plane_size/2 * v_hat, ...
    wall_center + plane_size/2 * u_hat - plane_size/2 * v_hat ...
];
fill3(ax1, corners(1,:), corners(2,:), corners(3,:), ...
    [0.7 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 1.5);
hold(ax1, 'off');

legend(ax1, {'p_d', 'p_m', 'Start', 'Wall'}, ...
    'Location', 'best', 'FontSize', LEGEND_FS, 'FontWeight', 'bold');
xlabel(ax1, 'x (um)', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
ylabel(ax1, 'y (um)', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
zlabel(ax1, 'z (um)', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
set(ax1, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
ax1.Box = 'on'; grid(ax1, 'off');
view(ax1, 30, 30); axis(ax1, 'equal');
fprintf('  Tab 1: 3D Trajectory\n');

% ==================== Tabs 2-4: X/Y/Z Axis (2x1) ====================
axis_names = {'X', 'Y', 'Z'};
axis_errors = {error_x, error_y, error_z};

for ai = 1:3
    tab_ax = uitab(tabgroup, 'Title', sprintf('%s-Axis', axis_names{ai}));

    % Top: overlay tracking
    ax_top = uiaxes(tab_ax, 'Position', pos_2x1{1});
    ax_top.PositionConstraint = 'innerposition';
    plot(ax_top, t_sample, p_d_log(ai,:), '-', 'Color', COL_REF, 'LineWidth', LINE_REF);
    hold(ax_top, 'on');
    plot(ax_top, t_sample, p_m_log(ai,:), '-', 'Color', COL_OUT, 'LineWidth', LINE_OUT);
    hold(ax_top, 'off');
    legend(ax_top, {'Reference', 'Measured'}, ...
        'Location', 'northoutside', 'Orientation', 'horizontal', ...
        'FontSize', LEGEND_FS, 'FontWeight', 'bold');
    ylabel(ax_top, 'Position (um)', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    set(ax_top, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
    ax_top.Box = 'on'; grid(ax_top, 'off'); axis(ax_top, 'tight');
    ax_top.XTickLabel = [];

    % Bottom: error
    ax_bot = uiaxes(tab_ax, 'Position', pos_2x1{2});
    ax_bot.PositionConstraint = 'innerposition';
    plot(ax_bot, t_sample, axis_errors{ai}, '-', 'Color', COL_ERR, 'LineWidth', LINE_OUT);
    xlabel(ax_bot, 'Time (sec)', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    ylabel(ax_bot, 'Error (nm)', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    title(ax_bot, sprintf('%s-Axis Tracking Error', axis_names{ai}), ...
        'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    set(ax_bot, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
    ax_bot.Box = 'on'; grid(ax_bot, 'off'); axis(ax_bot, 'tight');

    fprintf('  Tab %d: %s-Axis\n', ai+1, axis_names{ai});
end

% ==================== Tab 5: Control Force (3x1) ====================
tab5 = uitab(tabgroup, 'Title', 'Control Force');
force_labels = {'f_x (pN)', 'f_y (pN)', 'f_z (pN)'};

for fi = 1:3
    ax_f = uiaxes(tab5, 'Position', pos_3x1{fi});
    ax_f.PositionConstraint = 'innerposition';
    plot(ax_f, t_sample, f_d_log(fi,:), '-', 'Color', COL_ERR, 'LineWidth', LINE_OUT);
    ylabel(ax_f, force_labels{fi}, 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    set(ax_f, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
    ax_f.Box = 'on'; grid(ax_f, 'off'); axis(ax_f, 'tight');
    if fi == 1
        title(ax_f, sprintf('Control Force (%s)', ctrl_mode_str), ...
            'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    end
    if fi < 3; ax_f.XTickLabel = []; end
    if fi == 3
        xlabel(ax_f, 'Time (sec)', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    end
end
fprintf('  Tab 5: Control Force\n');

% ==================== Closed-loop Tabs (6-8) ====================
if is_closed_loop
    % --- Tab 6: Lambda Estimation ---
    tab6 = uitab(tabgroup, 'Title', 'Lambda Est.');
    lam_labels = {'c_{para}', 'c_{perp}'};
    for li = 1:2
        ax_l = uiaxes(tab6, 'Position', pos_2x1{li});
        ax_l.PositionConstraint = 'innerposition';
        plot(ax_l, t_sample, lambda_true_log(li,:), '-', 'Color', COL_REF, 'LineWidth', LINE_REF);
        hold(ax_l, 'on');
        plot(ax_l, t_sample, lamda_hat_log(li,:), '-', 'Color', COL_OUT, 'LineWidth', LINE_OUT);
        hold(ax_l, 'off');
        legend(ax_l, {'True', 'Estimated'}, ...
            'Location', 'northoutside', 'Orientation', 'horizontal', ...
            'FontSize', LEGEND_FS, 'FontWeight', 'bold');
        ylabel(ax_l, lam_labels{li}, 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
        set(ax_l, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
        ax_l.Box = 'on'; grid(ax_l, 'off'); axis(ax_l, 'tight');
        if li == 1; ax_l.XTickLabel = []; end
        if li == 2
            xlabel(ax_l, 'Time (sec)', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
        end
    end
    fprintf('  Tab 6: Lambda Est.\n');

    % --- Tab 7: Theta Estimation ---
    tab7 = uitab(tabgroup, 'Title', 'Theta Est.');
    theta_labels = {'\theta_x (rad)', '\theta_y (rad)'};
    for ti = 1:2
        ax_t = uiaxes(tab7, 'Position', pos_2x1{ti});
        ax_t.PositionConstraint = 'innerposition';
        plot(ax_t, t_sample, theta_hat_log(ti,:), '-', 'Color', COL_OUT, 'LineWidth', LINE_OUT);
        ylabel(ax_t, theta_labels{ti}, 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
        set(ax_t, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
        ax_t.Box = 'on'; grid(ax_t, 'off'); axis(ax_t, 'tight');
        if ti == 1
            title(ax_t, 'Theta Estimation Convergence', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
            ax_t.XTickLabel = [];
        end
        if ti == 2
            xlabel(ax_t, 'Time (sec)', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
        end
    end
    fprintf('  Tab 7: Theta Est.\n');

    % --- Tab 8: Error Statistics ---
    tab8 = uitab(tabgroup, 'Title', 'Error Stats');
    COL_XYZ = {[0.0 0.2 0.8], [0.0 0.6 0.0], [0.8 0.0 0.0]};

    errors_nm = {error_x, error_y, error_z};
    running_mean = zeros(3, N_samples);
    running_std_val = zeros(3, N_samples);
    for ai = 1:3
        cs = cumsum(errors_nm{ai});
        cs2 = cumsum(errors_nm{ai}.^2);
        n_vec = 1:N_samples;
        running_mean(ai,:) = cs ./ n_vec;
        running_std_val(ai,:) = sqrt(max(cs2 ./ n_vec - (cs ./ n_vec).^2, 0));
    end

    % Top: Running mean
    ax9_top = uiaxes(tab8, 'Position', pos_2x1{1});
    ax9_top.PositionConstraint = 'innerposition';
    hold(ax9_top, 'on');
    for ai = 1:3
        plot(ax9_top, t_sample, running_mean(ai,:), '-', 'Color', COL_XYZ{ai}, 'LineWidth', LINE_OUT);
    end
    hold(ax9_top, 'off');
    legend(ax9_top, {'X', 'Y', 'Z'}, ...
        'Location', 'northoutside', 'Orientation', 'horizontal', ...
        'FontSize', LEGEND_FS, 'FontWeight', 'bold');
    ylabel(ax9_top, 'Running Mean (nm)', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    set(ax9_top, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
    ax9_top.Box = 'on'; grid(ax9_top, 'off'); axis(ax9_top, 'tight');
    ax9_top.XTickLabel = [];

    % Bottom: Running STD
    ax9_bot = uiaxes(tab8, 'Position', pos_2x1{2});
    ax9_bot.PositionConstraint = 'innerposition';
    hold(ax9_bot, 'on');
    for ai = 1:3
        plot(ax9_bot, t_sample, running_std_val(ai,:), '-', 'Color', COL_XYZ{ai}, 'LineWidth', LINE_OUT);
    end
    hold(ax9_bot, 'off');
    legend(ax9_bot, {'X', 'Y', 'Z'}, ...
        'Location', 'northoutside', 'Orientation', 'horizontal', ...
        'FontSize', LEGEND_FS, 'FontWeight', 'bold');
    xlabel(ax9_bot, 'Time (sec)', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    ylabel(ax9_bot, 'Running STD (nm)', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
    set(ax9_bot, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
    ax9_bot.Box = 'on'; grid(ax9_bot, 'off'); axis(ax9_bot, 'tight');

    fprintf('  Tab 8: Error Stats\n');
end

% ==================== Open-loop Thermal Tabs ====================
is_openloop_thermal = ~is_closed_loop && is_thermal;

if is_openloop_thermal
    Fs = 1 / Ts;

    % Theoretical f_th STD
    k_B = params.Value.thermal.k_B;
    T_temp = params.Value.thermal.T;
    gamma_N = params.Value.common.gamma_N;
    variance_coeff = 4 * k_B * T_temp * gamma_N / Ts;
    h_bar_init = (dot(p0, w_hat) - pz_wall) / R;
    [c_para, c_perp] = calc_correction_functions(h_bar_init);
    C_vec = c_para * (u_hat + v_hat) + c_perp * w_hat;
    std_fth_theory = sqrt(variance_coeff) * abs(C_vec);

    % FFT analysis
    p_m_x_nm = p_m_log(1,:)' * 1000;
    p_m_y_nm = p_m_log(2,:)' * 1000;
    p_m_z_nm = p_m_log(3,:)' * 1000;
    time_vec = t_sample';

    [std_pm_x, det_pm_x, ~, freq_pm_x, spectrum_pm_x] = ...
        fft_deterministic_random_separation(p_m_x_nm, time_vec, openloop_cutoff_freq);
    [std_pm_y, det_pm_y, ~, freq_pm_y, spectrum_pm_y] = ...
        fft_deterministic_random_separation(p_m_y_nm, time_vec, openloop_cutoff_freq);
    [std_pm_z, det_pm_z, ~, freq_pm_z, spectrum_pm_z] = ...
        fft_deterministic_random_separation(p_m_z_nm, time_vec, openloop_cutoff_freq);

    pp_pm_x = max(det_pm_x) - min(det_pm_x);
    pp_pm_y = max(det_pm_y) - min(det_pm_y);
    pp_pm_z = max(det_pm_z) - min(det_pm_z);

    N_fft = length(det_pm_x);
    time_plot = time_vec(1:N_fft);
    p_m_x_plot = p_m_x_nm(1:N_fft);
    p_m_y_plot = p_m_y_nm(1:N_fft);
    p_m_z_plot = p_m_z_nm(1:N_fft);
    duration_plot = time_plot(end);

    % Remove DC offset for visualization
    offset_x = det_pm_x(1); offset_y = det_pm_y(1); offset_z = det_pm_z(1);
    p_m_x_centered = p_m_x_plot - offset_x;
    p_m_y_centered = p_m_y_plot - offset_y;
    p_m_z_centered = p_m_z_plot - offset_z;
    det_x_centered = det_pm_x - offset_x;
    det_y_centered = det_pm_y - offset_y;
    det_z_centered = det_pm_z - offset_z;

    max_dev = max([max(abs(p_m_x_centered)), max(abs(p_m_y_centered)), max(abs(p_m_z_centered))]);
    y_limit = max_dev * 1.15;

    % --- Tab 6: Open-loop Time Response ---
    tab6_ol = uitab(tabgroup, 'Title', 'OL Time');
    ol_colors = {COL_ERR, COL_REF, COL_OUT};
    ol_data_raw = {p_m_x_centered, p_m_y_centered, p_m_z_centered};
    ol_data_det = {det_x_centered, det_y_centered, det_z_centered};
    ol_stds = [std_pm_x, std_pm_y, std_pm_z];
    ol_ylabels = {'\Deltap_{m,x} (nm)', '\Deltap_{m,y} (nm)', '\Deltap_{m,z} (nm)'};

    for oi = 1:3
        ax_ol = uiaxes(tab6_ol, 'Position', pos_3x1{oi});
        ax_ol.PositionConstraint = 'innerposition';
        h_raw = plot(ax_ol, time_plot, ol_data_raw{oi}, '-', 'Color', ol_colors{oi}, 'LineWidth', 1.2);
        h_raw.Color(4) = 0.5;
        hold(ax_ol, 'on');
        plot(ax_ol, time_plot, ol_data_det{oi}, 'k-', 'LineWidth', 1.8);
        hold(ax_ol, 'off');
        ylabel(ax_ol, ol_ylabels{oi}, 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
        set(ax_ol, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
        ax_ol.Box = 'on'; grid(ax_ol, 'off');
        xlim(ax_ol, [0, duration_plot]); ylim(ax_ol, [-y_limit, y_limit]);
        if oi == 1
            title(ax_ol, 'Open-loop Position Response (DC removed)', ...
                'FontSize', FONT_SIZE, 'FontWeight', 'bold');
        end
        if oi < 3; ax_ol.XTickLabel = []; end
        if oi == 3
            xlabel(ax_ol, 'Time (sec)', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
        end
        text(ax_ol, 0.98, 0.95, sprintf('STD = %.2f nm', ol_stds(oi)), ...
            'Units', 'normalized', 'HorizontalAlignment', 'right', ...
            'VerticalAlignment', 'top', 'FontSize', EXP_FS, 'FontWeight', 'bold', ...
            'BackgroundColor', 'white', 'EdgeColor', 'black', 'LineWidth', 1.5);
    end
    fprintf('  Tab 6: OL Time Response\n');

    % --- Tab 7: Open-loop FFT Spectrum ---
    tab7_ol = uitab(tabgroup, 'Title', 'OL FFT');
    all_spectrum = [spectrum_pm_x(2:end); spectrum_pm_y(2:end); spectrum_pm_z(2:end)];
    y_min_fft = min(all_spectrum(all_spectrum > 0)) * 0.5;
    y_max_fft = max(all_spectrum) * 2;
    ol_freqs = {freq_pm_x, freq_pm_y, freq_pm_z};
    ol_spectra = {spectrum_pm_x, spectrum_pm_y, spectrum_pm_z};
    fft_ylabels = {'X Amp. (nm)', 'Y Amp. (nm)', 'Z Amp. (nm)'};

    for fi = 1:3
        ax_fft = uiaxes(tab7_ol, 'Position', pos_3x1{fi});
        ax_fft.PositionConstraint = 'innerposition';
        loglog(ax_fft, ol_freqs{fi}(2:end), ol_spectra{fi}(2:end), '-', ...
            'Color', ol_colors{fi}, 'LineWidth', LINE_OUT);
        ylabel(ax_fft, fft_ylabels{fi}, 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
        set(ax_fft, 'FontSize', FONT_SIZE, 'FontWeight', 'bold', 'LineWidth', AXIS_LW);
        ax_fft.Box = 'on';
        xlim(ax_fft, [ol_freqs{fi}(2), Fs/2]);
        ylim(ax_fft, [y_min_fft, y_max_fft]);
        if fi == 1
            title(ax_fft, 'Position FFT Spectrum (Open-loop)', ...
                'FontSize', FONT_SIZE, 'FontWeight', 'bold');
        end
        if fi < 3; ax_fft.XTickLabel = []; end
        if fi == 3
            xlabel(ax_fft, 'Frequency (Hz)', 'FontSize', FONT_SIZE, 'FontWeight', 'bold');
        end
    end
    fprintf('  Tab 7: OL FFT Spectrum\n');
end

%% SECTION 6: Save Results

fprintf('\nSaving results...\n');

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
output_dir = fullfile(project_root, 'test_results', 'simulation', ['sim_' timestamp]);
mkdir(output_dir);

% --- PNG Export (traditional figures with export style) ---
fig_exp = figure('Visible', 'off', 'Position', [100 100 1400 900]);

% Export Tab 1: 3D Trajectory
ax_e = axes(fig_exp, 'Position', [0.12 0.12 0.82 0.80]);
plot3(ax_e, p_d_log(1,:), p_d_log(2,:), p_d_log(3,:), '-', 'Color', COL_REF, 'LineWidth', EXP_LR);
hold(ax_e, 'on');
plot3(ax_e, p_m_log(1,:), p_m_log(2,:), p_m_log(3,:), '-', 'Color', COL_OUT, 'LineWidth', EXP_LO);
plot3(ax_e, p0(1), p0(2), p0(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'LineWidth', 2);
fill3(ax_e, corners(1,:), corners(2,:), corners(3,:), ...
    [0.7 0.7 0.7], 'FaceAlpha', 0.3, 'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 1.5);
hold(ax_e, 'off');
legend(ax_e, {'p_d', 'p_m', 'Start', 'Wall'}, 'Location', 'best', 'FontSize', EXP_LFS, 'FontWeight', 'bold');
xlabel(ax_e, 'x (um)'); ylabel(ax_e, 'y (um)'); zlabel(ax_e, 'z (um)');
set(ax_e, 'FontSize', EXP_FS, 'FontWeight', 'bold', 'LineWidth', EXP_LW, 'Box', 'on');
view(ax_e, 30, 30); axis(ax_e, 'equal');
exportgraphics(fig_exp, fullfile(output_dir, '1_trajectory_3d.png'), 'Resolution', 150);
clf(fig_exp);

% Export Tabs 2-4: X/Y/Z Axis (2x1)
for ai = 1:3
    ax_top_e = subplot(2,1,1);
    plot(ax_top_e, t_sample, p_d_log(ai,:), '-', 'Color', COL_REF, 'LineWidth', EXP_LR);
    hold(ax_top_e, 'on');
    plot(ax_top_e, t_sample, p_m_log(ai,:), '-', 'Color', COL_OUT, 'LineWidth', EXP_LO);
    hold(ax_top_e, 'off');
    legend(ax_top_e, {'Reference', 'Measured'}, 'Location', 'northoutside', ...
        'Orientation', 'horizontal', 'FontSize', EXP_LFS, 'FontWeight', 'bold');
    ylabel(ax_top_e, 'Position (um)');
    set(ax_top_e, 'FontSize', EXP_FS, 'FontWeight', 'bold', 'LineWidth', EXP_LW, 'Box', 'on');
    axis(ax_top_e, 'tight');

    ax_bot_e = subplot(2,1,2);
    plot(ax_bot_e, t_sample, axis_errors{ai}, '-', 'Color', COL_ERR, 'LineWidth', EXP_LO);
    xlabel(ax_bot_e, 'Time (sec)'); ylabel(ax_bot_e, 'Error (nm)');
    title(ax_bot_e, sprintf('%s-Axis Tracking Error', axis_names{ai}));
    set(ax_bot_e, 'FontSize', EXP_FS, 'FontWeight', 'bold', 'LineWidth', EXP_LW, 'Box', 'on');
    axis(ax_bot_e, 'tight');

    exportgraphics(fig_exp, fullfile(output_dir, sprintf('%d_%s_axis.png', ai+1, lower(axis_names{ai}))), 'Resolution', 150);
    clf(fig_exp);
end

% Export Tab 5: Control Force (3x1)
for fi = 1:3
    ax_fe = subplot(3,1,fi);
    plot(ax_fe, t_sample, f_d_log(fi,:), '-', 'Color', COL_ERR, 'LineWidth', EXP_LO);
    ylabel(ax_fe, force_labels{fi});
    set(ax_fe, 'FontSize', EXP_FS, 'FontWeight', 'bold', 'LineWidth', EXP_LW, 'Box', 'on');
    axis(ax_fe, 'tight');
    if fi == 1; title(ax_fe, sprintf('Control Force (%s)', ctrl_mode_str)); end
    if fi == 3; xlabel(ax_fe, 'Time (sec)'); end
end
exportgraphics(fig_exp, fullfile(output_dir, '5_control_force.png'), 'Resolution', 150);
clf(fig_exp);

% Export closed-loop tabs (6-8)
if is_closed_loop
    % Tab 6: Lambda Est.
    for li = 1:2
        ax_le = subplot(2,1,li);
        plot(ax_le, t_sample, lambda_true_log(li,:), '-', 'Color', COL_REF, 'LineWidth', EXP_LR);
        hold(ax_le, 'on');
        plot(ax_le, t_sample, lamda_hat_log(li,:), '-', 'Color', COL_OUT, 'LineWidth', EXP_LO);
        hold(ax_le, 'off');
        legend(ax_le, {'True', 'Estimated'}, 'Location', 'northoutside', ...
            'Orientation', 'horizontal', 'FontSize', EXP_LFS, 'FontWeight', 'bold');
        ylabel(ax_le, lam_labels{li});
        set(ax_le, 'FontSize', EXP_FS, 'FontWeight', 'bold', 'LineWidth', EXP_LW, 'Box', 'on');
        axis(ax_le, 'tight');
        if li == 2; xlabel(ax_le, 'Time (sec)'); end
    end
    exportgraphics(fig_exp, fullfile(output_dir, '6_lambda_est.png'), 'Resolution', 150);
    clf(fig_exp);

    % Tab 7: Theta Est.
    for ti = 1:2
        ax_te = subplot(2,1,ti);
        plot(ax_te, t_sample, theta_hat_log(ti,:), '-', 'Color', COL_OUT, 'LineWidth', EXP_LO);
        ylabel(ax_te, theta_labels{ti});
        set(ax_te, 'FontSize', EXP_FS, 'FontWeight', 'bold', 'LineWidth', EXP_LW, 'Box', 'on');
        axis(ax_te, 'tight');
        if ti == 1; title(ax_te, 'Theta Estimation Convergence'); end
        if ti == 2; xlabel(ax_te, 'Time (sec)'); end
    end
    exportgraphics(fig_exp, fullfile(output_dir, '7_theta_est.png'), 'Resolution', 150);
    clf(fig_exp);

    % Tab 8: Error Stats
    COL_XYZ_arr = {[0.0 0.2 0.8], [0.0 0.6 0.0], [0.8 0.0 0.0]};
    ax_me = subplot(2,1,1);
    hold(ax_me, 'on');
    for ai = 1:3
        plot(ax_me, t_sample, running_mean(ai,:), '-', 'Color', COL_XYZ_arr{ai}, 'LineWidth', EXP_LO);
    end
    hold(ax_me, 'off');
    legend(ax_me, {'X', 'Y', 'Z'}, 'Location', 'northoutside', ...
        'Orientation', 'horizontal', 'FontSize', EXP_LFS, 'FontWeight', 'bold');
    ylabel(ax_me, 'Running Mean (nm)');
    set(ax_me, 'FontSize', EXP_FS, 'FontWeight', 'bold', 'LineWidth', EXP_LW, 'Box', 'on');
    axis(ax_me, 'tight');

    ax_se = subplot(2,1,2);
    hold(ax_se, 'on');
    for ai = 1:3
        plot(ax_se, t_sample, running_std_val(ai,:), '-', 'Color', COL_XYZ_arr{ai}, 'LineWidth', EXP_LO);
    end
    hold(ax_se, 'off');
    legend(ax_se, {'X', 'Y', 'Z'}, 'Location', 'northoutside', ...
        'Orientation', 'horizontal', 'FontSize', EXP_LFS, 'FontWeight', 'bold');
    xlabel(ax_se, 'Time (sec)'); ylabel(ax_se, 'Running STD (nm)');
    set(ax_se, 'FontSize', EXP_FS, 'FontWeight', 'bold', 'LineWidth', EXP_LW, 'Box', 'on');
    axis(ax_se, 'tight');

    exportgraphics(fig_exp, fullfile(output_dir, '8_error_stats.png'), 'Resolution', 150);
    clf(fig_exp);
end

% Export open-loop tabs
if is_openloop_thermal
    % Tab 7: OL Time Response
    for oi = 1:3
        ax_oe = subplot(3,1,oi);
        h_r = plot(ax_oe, time_plot, ol_data_raw{oi}, '-', 'Color', ol_colors{oi}, 'LineWidth', 1.2);
        h_r.Color(4) = 0.5;
        hold(ax_oe, 'on');
        plot(ax_oe, time_plot, ol_data_det{oi}, 'k-', 'LineWidth', 1.8);
        hold(ax_oe, 'off');
        ylabel(ax_oe, ol_ylabels{oi});
        set(ax_oe, 'FontSize', EXP_FS, 'FontWeight', 'bold', 'LineWidth', EXP_LW, 'Box', 'on');
        xlim(ax_oe, [0 duration_plot]); ylim(ax_oe, [-y_limit y_limit]);
        if oi == 1; title(ax_oe, 'Open-loop Position Response (DC removed)'); end
        if oi == 3; xlabel(ax_oe, 'Time (sec)'); end
        text(ax_oe, 0.98, 0.95, sprintf('STD = %.2f nm', ol_stds(oi)), ...
            'Units', 'normalized', 'HorizontalAlignment', 'right', ...
            'VerticalAlignment', 'top', 'BackgroundColor', 'white', 'EdgeColor', 'black');
    end
    exportgraphics(fig_exp, fullfile(output_dir, '6_openloop_time.png'), 'Resolution', 150);
    clf(fig_exp);

    % Tab 8: OL FFT
    for fi = 1:3
        ax_fe2 = subplot(3,1,fi);
        loglog(ax_fe2, ol_freqs{fi}(2:end), ol_spectra{fi}(2:end), '-', ...
            'Color', ol_colors{fi}, 'LineWidth', EXP_LO);
        ylabel(ax_fe2, fft_ylabels{fi});
        set(ax_fe2, 'FontSize', EXP_FS, 'FontWeight', 'bold', 'LineWidth', EXP_LW, 'Box', 'on');
        xlim(ax_fe2, [ol_freqs{fi}(2), Fs/2]); ylim(ax_fe2, [y_min_fft, y_max_fft]);
        if fi == 1; title(ax_fe2, 'Position FFT Spectrum (Open-loop)'); end
        if fi == 3; xlabel(ax_fe2, 'Frequency (Hz)'); end
    end
    exportgraphics(fig_exp, fullfile(output_dir, '7_openloop_fft.png'), 'Resolution', 150);
    clf(fig_exp);
end

close(fig_exp);
fprintf('  Figures saved (.png)\n');

% --- Save Data ---
result.t = t_sample;
result.p_m = p_m_log;
result.p_d = p_d_log;
result.f_d = f_d_log;
result.f_th = f_th_log;
result.h_bar = h_bar_log;
result.error_3d = error_3d;
result.error_x = error_x;
result.error_y = error_y;
result.error_z = error_z;
result.params = params.Value;
result.p0 = p0;
result.config = config;
result.tracking_error_rmse = rms(error_3d);
result.tracking_error_rmse_x = rms(error_x);
result.tracking_error_rmse_y = rms(error_y);
result.tracking_error_rmse_z = rms(error_z);
result.meta.timestamp = datestr(now);
result.meta.ctrl_mode = ctrl_mode_str;
result.meta.thermal_mode = thermal_str;
result.meta.traj_type = traj_type_str;

if is_closed_loop
    result.lamda_hat = lamda_hat_log;
    result.theta_hat = theta_hat_log;
    result.lambda_true = lambda_true_log;
end

save(fullfile(output_dir, 'result.mat'), 'result');
fprintf('  Data saved (.mat)\n');

%% SECTION 7: Print Summary

fprintf('\n');
fprintf('================================================================\n');
fprintf('                    Simulation Summary\n');
fprintf('================================================================\n');
fprintf('\n');
fprintf('Configuration:\n');
fprintf('  Mode: %s\n', ctrl_mode_str);
fprintf('  Thermal: %s\n', thermal_str);
fprintf('  Trajectory: %s\n', traj_type_str);
fprintf('  Duration: %.1f sec (%d samples)\n', T_sim, N_samples);
fprintf('\n');
fprintf('Tracking Performance (3D):\n');
fprintf('  Max error: %.4f um (%.2f nm)\n', max(error_3d), max(error_3d)*1000);
fprintf('\n');
fprintf('Tracking Performance (per axis):\n');
fprintf('  X-axis Max: %.2f nm\n', max(abs(error_x)));
fprintf('  Y-axis Max: %.2f nm\n', max(abs(error_y)));
fprintf('  Z-axis Max: %.2f nm\n', max(abs(error_z)));
fprintf('\n');
fprintf('Control Force:\n');
fprintf('  Max |f_x|: %.4f pN\n', max(abs(f_d_log(1,:))));
fprintf('  Max |f_y|: %.4f pN\n', max(abs(f_d_log(2,:))));
fprintf('  Max |f_z|: %.4f pN\n', max(abs(f_d_log(3,:))));
fprintf('\n');
fprintf('Wall Distance:\n');
fprintf('  Min h/R: %.2f (threshold: %.1f)\n', min(h_bar_log), params.Value.wall.h_bar_min);
fprintf('  Max h/R: %.2f\n', max(h_bar_log));
fprintf('\n');
fprintf('Results saved to: %s\n', output_dir);
fprintf('================================================================\n');

if is_openloop_thermal
    fprintf('\n');
    fprintf('========================================\n');
    fprintf('  Open-loop Thermal Analysis Results\n');
    fprintf('========================================\n\n');
    fprintf('Thermal Force (f_th) - Theoretical STD:\n');
    fprintf('  X: STD = %.4f pN\n', std_fth_theory(1));
    fprintf('  Y: STD = %.4f pN\n', std_fth_theory(2));
    fprintf('  Z: STD = %.4f pN\n\n', std_fth_theory(3));
    fprintf('Position Response (p_m) - Measured:\n');
    fprintf('  X: Random STD = %.2f nm, Det. P-P = %.2f nm\n', std_pm_x, pp_pm_x);
    fprintf('  Y: Random STD = %.2f nm, Det. P-P = %.2f nm\n', std_pm_y, pp_pm_y);
    fprintf('  Z: Random STD = %.2f nm, Det. P-P = %.2f nm\n\n', std_pm_z, pp_pm_z);
    fprintf('Analysis Parameters:\n');
    fprintf('  Cutoff Frequency: %.1f Hz\n', openloop_cutoff_freq);
    fprintf('  Sample Rate: %.0f Hz\n', Fs);
    fprintf('  Duration: %.2f sec\n', duration_plot);
    fprintf('  FFT Resolution: %.4f Hz\n', Fs / N_fft);
    fprintf('========================================\n');
end


%% ========== Local Functions ==========

function [std_random, data_deterministic, data_random, f, P1] = ...
    fft_deterministic_random_separation(data, time, cutoff_freq)
%FFT_DETERMINISTIC_RANDOM_SEPARATION Separate deterministic and random via FFT

    N = length(data);

    if mod(N, 2) == 1
        data = data(1:end-1);
        time = time(1:end-1);
        N = N - 1;
    end

    Fs = 1 / mean(diff(time));

    data_mean = mean(data);
    data_demean = data - data_mean;

    Y_original = fft(data_demean);

    f = Fs * (0:(N/2)) / N;
    f = f(:);

    P2 = abs(Y_original / N);
    P1 = P2(1:N/2+1);
    P1(2:end-1) = 2 * P1(2:end-1);

    cutoff_idx = find(f <= cutoff_freq, 1, 'last');
    if isempty(cutoff_idx)
        cutoff_idx = 1;
    end

    % Random (high-frequency)
    Y_highfreq = Y_original;
    Y_highfreq(1:cutoff_idx) = 0;
    Y_highfreq(N-cutoff_idx+2:N) = 0;
    data_random = ifft(Y_highfreq, 'symmetric');
    std_random = std(data_random);

    % Deterministic (low-frequency with extension + windowing)
    extend_len = round(N * 0.1);
    left_extend = linspace(0, data_demean(1), extend_len)';
    right_extend = linspace(data_demean(end), 0, extend_len)';
    data_extended = [left_extend; data_demean; right_extend];
    N_ext = length(data_extended);

    alpha = 0.1;
    window = tukeywin(N_ext, alpha);
    data_windowed = data_extended .* window;

    Y_det = fft(data_windowed);
    Y_det_lowfreq = Y_det;
    cutoff_idx_ext = round(cutoff_idx * N_ext / N);
    Y_det_lowfreq(cutoff_idx_ext+1:N_ext-cutoff_idx_ext+1) = 0;

    det_extended = ifft(Y_det_lowfreq, 'symmetric');
    data_deterministic = det_extended(extend_len+1:extend_len+N) + data_mean;

end
