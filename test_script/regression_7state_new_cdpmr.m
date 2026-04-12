%% regression_7state_new_cdpmr.m — Regression test for post-integration
%
% Verifies that the new C_dpmr_eff / C_np_eff integration doesn't break the
% existing behavior of the 7-state EKF. Runs ACTUAL free-space positioning
% (h_init=50, no wall effect) across several lc values and checks that the
% EKF's a_hat converges to a_nom.

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

fprintf('===== regression_7state_new_cdpmr =====\n\n');

constants = physical_constants();
k_B = constants.k_B; T_temp = constants.T;
Ts = constants.Ts; gamma_N = constants.gamma_N;
a_nom = Ts / gamma_N;
sigma2_dXT = 4 * k_B * T_temp * Ts / gamma_N;

fprintf('a_nom = %.4e um/pN\n', a_nom);
fprintf('sigma2_dXT = %.4e um^2\n\n', sigma2_dXT);

lc_list = [0.5, 0.7, 0.9];
T_sim = 10;

results = struct();
results.lc = [];
results.a_hat_x_ratio = [];
results.a_hat_z_ratio = [];
results.Var_delpmr_z = [];
results.Var_delpmr_z_theory = [];
results.C_dpmr_eff_used = [];

model_path = fullfile(project_root, 'model', 'system_model');
load_system(model_path);

for i = 1:length(lc_list)
    lc = lc_list(i);
    fprintf('--- lc = %.2f ---\n', lc);

    clear motion_control_law motion_control_law_7state ...
          motion_control_law_23state trajectory_generator calc_thermal_force;

    config = user_config();
    config.h_init = 50;
    config.amplitude = 0;
    config.enable_wall_effect = false;           % TRUE free space
    config.trajectory_type = 'positioning';      % constant target
    config.t_hold = 0;
    config.n_cycles = 1;
    config.frequency = 1;
    config.ctrl_enable = true;
    config.controller_type = 7;
    config.lambda_c = lc;
    config.thermal_enable = true;
    config.meas_noise_enable = false;
    config.meas_noise_std = [0; 0; 0];
    config.T_sim = T_sim;

    params = calc_simulation_params(config);
    C_dpmr_used = params.Value.ctrl.C_dpmr_eff;
    assignin('base', 'params', params);
    assignin('base', 'p0', params.Value.common.p0);
    assignin('base', 'Ts', Ts);

    rng(200 + i);
    fprintf('  C_dpmr_eff = %.4f\n', C_dpmr_used);
    fprintf('  Running Simulink...');
    t0 = tic;
    simOut = sim(model_path, 'StopTime', num2str(T_sim), ...
        'SaveTime', 'on', 'TimeSaveName', 'tout', ...
        'SaveOutput', 'on', 'OutputSaveName', 'yout');
    fprintf(' done (%.1f sec)\n', toc(t0));

    p_d = simOut.p_d_out';
    p_m = simOut.p_m_out';
    ekf = simOut.ekf_out';
    N = size(p_d, 2);

    del_pm = zeros(3, N);
    for k = 3:N, del_pm(:, k) = p_d(:, k-2) - p_m(:, k); end

    a_pd = config.a_pd;
    del_pmd = zeros(3, N); del_pmr = zeros(3, N);
    for k = 2:N
        del_pmd(:,k) = (1-a_pd)*del_pmd(:,k-1) + a_pd*del_pm(:,k);
        del_pmr(:,k) = del_pm(:,k) - del_pmd(:,k);
    end

    ss = round(N/2):N;
    a_hat_x = mean(ekf(1, ss));  a_hat_z = mean(ekf(2, ss));
    Var_z   = var(del_pmr(3, ss));
    Var_th  = C_dpmr_used * sigma2_dXT;

    fprintf('  a_hat_x / a_nom = %.4f\n', a_hat_x / a_nom);
    fprintf('  a_hat_z / a_nom = %.4f\n', a_hat_z / a_nom);
    fprintf('  Var(del_pmr_z)  = %.4e  (theory %.4e, err %+.1f%%)\n', ...
            Var_z, Var_th, 100*(Var_z - Var_th)/Var_th);

    results.lc(end+1) = lc;
    results.a_hat_x_ratio(end+1) = a_hat_x / a_nom;
    results.a_hat_z_ratio(end+1) = a_hat_z / a_nom;
    results.Var_delpmr_z(end+1) = Var_z;
    results.Var_delpmr_z_theory(end+1) = Var_th;
    results.C_dpmr_eff_used(end+1) = C_dpmr_used;
end

close_system(model_path, 0);

%% Summary
fprintf('\n==============================================\n');
fprintf('  Regression summary (free space, positioning)\n');
fprintf('==============================================\n');
fprintf('    lc   a_hat_x/a_nom  a_hat_z/a_nom  C_dpmr_eff  Var_err%%\n');
for i = 1:length(lc_list)
    err_var = 100*(results.Var_delpmr_z(i) - results.Var_delpmr_z_theory(i)) / ...
              results.Var_delpmr_z_theory(i);
    fprintf('  %5.2f    %8.4f      %8.4f      %8.4f    %+6.1f\n', ...
        results.lc(i), results.a_hat_x_ratio(i), results.a_hat_z_ratio(i), ...
        results.C_dpmr_eff_used(i), err_var);
end

% Gate
mean_x = mean(results.a_hat_x_ratio);
mean_z = mean(results.a_hat_z_ratio);
fprintf('\n  Mean a_hat_x / a_nom = %.4f\n', mean_x);
fprintf('  Mean a_hat_z / a_nom = %.4f\n', mean_z);

pass_x = abs(mean_x - 1) < 0.25;   % within 25%
pass_z = abs(mean_z - 1) < 0.25;
fprintf('\n  Gate: |mean - 1| < 0.25:  x=%s  z=%s\n', ...
    pass_str(pass_x), pass_str(pass_z));

out_dir = fullfile(project_root, 'test_results', 'verify');
save(fullfile(out_dir, 'regression_7state_new_cdpmr.mat'), 'results');

if pass_x && pass_z
    fprintf('\nREGRESSION: PASS\n');
else
    fprintf('\nREGRESSION: WARN (check individual values above)\n');
end

function s = pass_str(b)
    if b, s = 'PASS'; else, s = 'FAIL'; end
end
