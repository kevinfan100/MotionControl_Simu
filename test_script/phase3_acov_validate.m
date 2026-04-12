%% phase3_acov_validate.m — Validate a_cov reduction in actual Simulink closed-loop
%
% Phase 3b sweep (offline) showed that reducing a_cov from 0.05 to 0.005-0.01
% improves statistical precision 2-3x. This script runs actual Simulink
% closed-loop simulations with different a_cov values and measures the
% resulting EKF a_hat precision and tracking error.

clear; close all; clc;
clear motion_control_law motion_control_law_7state ...
      motion_control_law_23state trajectory_generator calc_thermal_force;

[script_dir, ~, ~] = fileparts(mfilename('fullpath'));
project_root = fileparts(script_dir);
cd(project_root);

addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'model', 'config'));
addpath(fullfile(project_root, 'model', 'wall_effect'));
addpath(fullfile(project_root, 'model', 'thermal_force'));
addpath(fullfile(project_root, 'model', 'trajectory'));
addpath(fullfile(project_root, 'model', 'controller'));
addpath(script_dir);

fprintf('===== phase3_acov_validate =====\n\n');

constants = physical_constants();
k_B = constants.k_B; T_temp = constants.T;
Ts = constants.Ts; gamma_N = constants.gamma_N;
a_nom = Ts / gamma_N;

% Three a_cov values to compare
a_cov_list = [0.05, 0.01, 0.005];   % current / small / very small
T_sim = 15;   % seconds

results_all = struct();

model_path = fullfile(project_root, 'model', 'system_model');
load_system(model_path);

for i = 1:length(a_cov_list)
    a_cov = a_cov_list(i);
    fprintf('--- [%d/%d] a_cov = %.4f ---\n', i, length(a_cov_list), a_cov);

    clear motion_control_law motion_control_law_7state ...
          motion_control_law_23state trajectory_generator calc_thermal_force;

    % Free space, positioning, no wall, no noise
    config = user_config();
    config.h_init = 50;
    config.amplitude = 0;
    config.enable_wall_effect = false;
    config.trajectory_type = 'positioning';
    config.t_hold = 0;
    config.n_cycles = 1;
    config.frequency = 1;
    config.ctrl_enable = true;
    config.controller_type = 7;
    config.lambda_c = 0.7;
    config.thermal_enable = true;
    config.meas_noise_enable = false;
    config.meas_noise_std = [0; 0; 0];
    config.a_cov = a_cov;              % <- THE VARIABLE
    config.T_sim = T_sim;

    params = calc_simulation_params(config);
    assignin('base', 'params', params);
    assignin('base', 'p0', params.Value.common.p0);
    assignin('base', 'Ts', Ts);

    rng(500 + i);
    fprintf('  Running Simulink (T_sim=%.1f)...', T_sim);
    t0 = tic;
    simOut = sim(model_path, 'StopTime', num2str(T_sim), ...
        'SaveTime', 'on', 'TimeSaveName', 'tout', ...
        'SaveOutput', 'on', 'OutputSaveName', 'yout');
    fprintf(' done (%.1f sec)\n', toc(t0));

    p_d = simOut.p_d_out';
    p_m = simOut.p_m_out';
    ekf = simOut.ekf_out';
    N = size(p_d, 2);
    ss = round(T_sim * 0.5 / Ts):N;   % last 50%

    err = p_m - p_d;
    a_hat_x = ekf(1, ss);
    a_hat_z = ekf(2, ss);

    % Stats
    fprintf('  a_hat_x: mean = %.4e (%.2f%%), std = %.4e (%.1f%% rel)\n', ...
            mean(a_hat_x), 100*mean(a_hat_x)/a_nom, std(a_hat_x), 100*std(a_hat_x)/mean(a_hat_x));
    fprintf('  a_hat_z: mean = %.4e (%.2f%%), std = %.4e (%.1f%% rel)\n', ...
            mean(a_hat_z), 100*mean(a_hat_z)/a_nom, std(a_hat_z), 100*std(a_hat_z)/mean(a_hat_z));
    fprintf('  3D tracking RMSE = %.4e um (%.1f nm)\n', ...
            sqrt(mean(sum(err(:, ss).^2, 1))), sqrt(mean(sum(err(:, ss).^2, 1)))*1e3);

    results_all(i).a_cov = a_cov;
    results_all(i).a_hat_x_mean = mean(a_hat_x);
    results_all(i).a_hat_x_std  = std(a_hat_x);
    results_all(i).a_hat_x_rel  = std(a_hat_x)/mean(a_hat_x) * 100;
    results_all(i).a_hat_x_bias = 100*(mean(a_hat_x) - a_nom)/a_nom;
    results_all(i).a_hat_z_mean = mean(a_hat_z);
    results_all(i).a_hat_z_std  = std(a_hat_z);
    results_all(i).a_hat_z_rel  = std(a_hat_z)/mean(a_hat_z) * 100;
    results_all(i).a_hat_z_bias = 100*(mean(a_hat_z) - a_nom)/a_nom;
    results_all(i).rmse_3d_nm   = sqrt(mean(sum(err(:, ss).^2, 1))) * 1e3;
end

close_system(model_path, 0);

%% Summary
fprintf('\n==============================================\n');
fprintf('  Summary: a_cov impact on EKF a_hat precision\n');
fprintf('==============================================\n');
fprintf('  a_cov    a_hat_x_rel  a_hat_x_bias   a_hat_z_rel  a_hat_z_bias  RMSE3D\n');
for i = 1:length(a_cov_list)
    r = results_all(i);
    fprintf('  %.4f   %6.2f%%     %+6.2f%%      %6.2f%%     %+6.2f%%     %.1f nm\n', ...
        r.a_cov, r.a_hat_x_rel, r.a_hat_x_bias, ...
        r.a_hat_z_rel, r.a_hat_z_bias, r.rmse_3d_nm);
end

out_dir = fullfile(project_root, 'test_results', 'verify');
save(fullfile(out_dir, 'phase3_acov_validate.mat'), 'results_all');
fprintf('\nSaved: phase3_acov_validate.mat\n');
fprintf('\nDone.\n');
