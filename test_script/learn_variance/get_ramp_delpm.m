% get_ramp_delpm.m
% Run pure-MATLAB simulation in ramp_descent 50→5 mode (T_sim=20s) and
% extract delta_x_m time series. Saved to .mat for offline IIR sandbox use.
%
% Reference: run_pure_simulation.m + motion_control_law_eq17_core.m
% Production default iir_warmup_mode='prefill'.

clear; close all; clc;
clear motion_control_law motion_control_law_23state ...
      motion_control_law_7state motion_control_law_eq17_core ...
      trajectory_generator calc_thermal_force;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(script_dir));
cd(project_root);

addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'thermal_force'));
addpath(fullfile(project_root, 'model', 'trajectory'));
addpath(fullfile(project_root, 'model', 'controller'));
addpath(fullfile(project_root, 'model', 'dual_track'));

%% Config — ramp_descent 50→5, T_sim=20s
config = user_config();
config.trajectory_type   = 'ramp_descent';
config.h_init            = 50;
config.h_bottom          = 5;
config.T_sim             = 20;
config.controller_type   = 'eq17_7state';
config.iir_warmup_mode   = 'prefill';
config.thermal_enable    = true;
config.meas_noise_enable = true;
config.meas_noise_std    = [0.00062; 0.000057; 0.00331];

opts = struct('seed', 42, 'verbose', true, 'collect_diag', true);

%% Run pure-MATLAB simulation
fprintf('===== Running pure MATLAB simulation =====\n');
fprintf('  Scenario: ramp_descent %g → %g um, T_sim = %g s\n', ...
        config.h_init, config.h_bottom, config.T_sim);
fprintf('  thermal=%d, meas_noise=%d, seed=%d\n\n', ...
        config.thermal_enable, config.meas_noise_enable, opts.seed);

t0 = tic;
simOut = run_pure_simulation(config, opts);
fprintf('\nElapsed: %.2f s\n', toc(t0));

%% Extract delta_x_m and save
delta_x_m = simOut.diag.delta_x_m;   % [N x 3]
tout      = simOut.tout;             % [N x 1]

save_dir = fullfile(project_root, 'test_results', 'learn_variance');
if ~exist(save_dir, 'dir'); mkdir(save_dir); end
save_path = fullfile(save_dir, 'ramp_50to5_T20_delpm.mat');
save(save_path, 'delta_x_m', 'tout', 'config', 'opts', '-v7.3');
fprintf('Saved: %s\n', save_path);

%% Summary statistics
labels = {'X', 'Y', 'Z'};
fprintf('\n=== delta_x_m summary ===\n');
fprintf('  N = %d samples, fs = %.0f Hz\n', length(tout), 1/(tout(2)-tout(1)));
for ax = 1:3
    fprintf('  axis %s: mean = %+.4e, std = %.4e, range = [%+.4e, %+.4e]  [um]\n', ...
            labels{ax}, mean(delta_x_m(:,ax)), std(delta_x_m(:,ax)), ...
            min(delta_x_m(:,ax)), max(delta_x_m(:,ax)));
end

%% Plot time domain (preview)
fig = figure('Position', [100 100 1200 700], 'Color', 'w');
tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for ax = 1:3
    nexttile;
    plot(tout, delta_x_m(:, ax), 'k-', 'LineWidth', 0.4);
    grid on;
    ylabel(sprintf('\\delta x_{m,%s} [um]', labels{ax}));
    if ax == 1
        title(sprintf('delta\\_x\\_m time domain — ramp\\_descent 50\\to 5, T\\_sim=%gs, seed=%d', ...
                      config.T_sim, opts.seed));
    end
    if ax == 3
        xlabel('Time [sec]');
    end
end
plot_path = fullfile(save_dir, 'preview_ramp_50to5_T20_delpm.png');
exportgraphics(fig, plot_path, 'Resolution', 150);
fprintf('\nSaved preview plot: %s\n', plot_path);
