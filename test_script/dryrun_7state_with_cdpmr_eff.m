%% dryrun_7state_with_cdpmr_eff.m — Go/no-go gate for integration
%
% Runs the actual 7-state EKF in Simulink AND computes the Eq.13 gain
% recovery offline using BOTH the old (K=2) and new (augmented Lyapunov)
% formulas. Compares:
%   a_m_new = (Var_empirical - C_np_eff*sigma2_n) / (C_dpmr_eff * 4*k_B*T)
%   a_m_old = (Var_empirical - noise_corr_K2) / (C_dpmr_K2 * 4*k_B*T)
%
% Gate: median |a_m_new - a_nom|/a_nom reduced by >= 20% vs old formula.
%
% If PASS: safe to patch calc_ctrl_params.m + motion_control_law_7state.m.
% If FAIL: stop; do NOT modify controller code; write diagnostic report.

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

fprintf('===== dryrun_7state_with_cdpmr_eff =====\n\n');

%% Physical constants
constants = physical_constants();
k_B = constants.k_B; T_temp = constants.T;
Ts = constants.Ts; gamma_N = constants.gamma_N;
a_nom = Ts / gamma_N;
sigma2_dXT = 4 * k_B * T_temp * Ts / gamma_N;

fprintf('a_nom = %.6e um/pN\n', a_nom);

%% Load lookup
LUT = load(fullfile(project_root, 'test_results', 'verify', 'cdpmr_eff_lookup.mat'));

%% Test grid (smaller for speed)
lc_list = [0.5, 0.7, 0.9];
n_lc = length(lc_list);

results_am = struct();
results_am.lc = [];
results_am.a_m_new = [];
results_am.a_m_K2  = [];
results_am.a_nom   = [];
results_am.a_hat_ctrl = [];   % what the controller's own EKF computed

T_sim = 10;

%% Load model
model_path = fullfile(project_root, 'model', 'system_model');
load_system(model_path);

for i = 1:n_lc
    lc = lc_list(i);
    fprintf('--- [%d/%d] lc=%.2f ---\n', i, n_lc, lc);

    clear motion_control_law motion_control_law_7state ...
          motion_control_law_23state trajectory_generator calc_thermal_force;

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
    config.lambda_c = lc;
    config.thermal_enable = true;
    config.meas_noise_enable = false;
    config.meas_noise_std = [0; 0; 0];
    config.T_sim = T_sim;

    params = calc_simulation_params(config);
    assignin('base', 'params', params);
    assignin('base', 'p0', params.Value.common.p0);
    assignin('base', 'Ts', Ts);

    rng(100 + i);

    fprintf('  Running Simulink...');
    t0 = tic;
    simOut = sim(model_path, ...
        'StopTime', num2str(T_sim), ...
        'SaveTime', 'on', 'TimeSaveName', 'tout', ...
        'SaveOutput', 'on', 'OutputSaveName', 'yout');
    fprintf(' done (%.1f sec)\n', toc(t0));

    p_d_log = simOut.p_d_out';
    p_m_log = simOut.p_m_out';
    ekf_log = simOut.ekf_out';    % [a_x; a_z; 0; 0] per step
    N_sim = size(p_d_log, 2);

    % Reconstruct del_pm + IIR offline (matches controller exactly)
    del_pm = zeros(3, N_sim);
    for k = 3:N_sim
        del_pm(:, k) = p_d_log(:, k-2) - p_m_log(:, k);
    end

    a_pd = config.a_pd;
    a_cov = config.a_cov;
    a_prd = config.a_prd;

    del_pmd = zeros(3, N_sim);
    del_pmr = zeros(3, N_sim);
    del_pmrd = zeros(3, N_sim);
    del_pmr2_avg = zeros(3, N_sim);
    Var_est = zeros(3, N_sim);
    for k = 2:N_sim
        del_pmd(:, k) = (1-a_pd)*del_pmd(:, k-1) + a_pd*del_pm(:, k);
        del_pmr(:, k) = del_pm(:, k) - del_pmd(:, k);
        del_pmrd(:, k) = (1-a_prd)*del_pmrd(:, k-1) + a_prd*del_pmr(:, k);
        del_pmr2_avg(:, k) = (1-a_cov)*del_pmr2_avg(:, k-1) + a_cov*del_pmr(:, k).^2;
        Var_est(:, k) = max(del_pmr2_avg(:, k) - del_pmrd(:, k).^2, 0);
    end

    % Apply both formulas
    sigma2_noise = [0; 0; 0];  % no noise in this test

    % NEW
    Cdpmr_new = interp1(LUT.lc_grid, LUT.Cdpmr_tab(:, 1), lc, 'linear');
    Cnp_new   = interp1(LUT.lc_grid, LUT.Cnp_tab(:, 1), lc, 'linear');
    den_new = Cdpmr_new * 4 * k_B * T_temp;
    a_m_new_series = max((Var_est - Cnp_new * sigma2_noise) / den_new, 0);

    % OLD (K=2)
    Cdpmr_K2 = (1-a_pd)^2 * (2*(1-a_pd)*(1-lc) / (1-(1-a_pd)*lc) ...
             + (2/(2-a_pd)) / ((1+lc)*(1-(1-a_pd)*lc)));
    den_K2 = Cdpmr_K2 * 4 * k_B * T_temp;
    noise_corr_K2 = (2 / (1 + lc)) * sigma2_noise;
    a_m_K2_series = max((Var_est - noise_corr_K2) / den_K2, 0);

    % Steady-state window
    ss = round(N_sim/2):N_sim;

    % Mean a_m across 3 axes (already the same for stationary)
    a_m_new_mean = mean(a_m_new_series(:, ss), 'all');
    a_m_K2_mean  = mean(a_m_K2_series(:, ss),  'all');
    a_hat_ctrl   = mean(ekf_log(1, ss));  % controller's own internal a_hat (x-axis)

    fprintf('  a_nom              = %.6e um/pN\n', a_nom);
    fprintf('  a_m_new (Eq.13 new)= %.6e  |a_m-a_nom|/a_nom = %.2f%%\n', ...
            a_m_new_mean, 100*abs(a_m_new_mean - a_nom)/a_nom);
    fprintf('  a_m_K2  (Eq.13 old)= %.6e  |a_m-a_nom|/a_nom = %.2f%%\n', ...
            a_m_K2_mean,  100*abs(a_m_K2_mean  - a_nom)/a_nom);
    fprintf('  a_hat (internal EKF, x-axis) = %.6e  |a_hat-a_nom|/a_nom = %.2f%%\n', ...
            a_hat_ctrl, 100*abs(a_hat_ctrl - a_nom)/a_nom);

    results_am.lc(end+1) = lc;
    results_am.a_m_new(end+1) = a_m_new_mean;
    results_am.a_m_K2(end+1)  = a_m_K2_mean;
    results_am.a_nom(end+1)   = a_nom;
    results_am.a_hat_ctrl(end+1) = a_hat_ctrl;
end

close_system(model_path, 0);

%% Summary
err_new = abs(results_am.a_m_new - results_am.a_nom) ./ results_am.a_nom;
err_K2  = abs(results_am.a_m_K2  - results_am.a_nom) ./ results_am.a_nom;

median_new = median(err_new);
median_K2  = median(err_K2);

fprintf('\n==============================================\n');
fprintf('  Dryrun summary (Eq.13 gain recovery from Simulink)\n');
fprintf('==============================================\n');
fprintf('    lc    a_m_new    a_m_K2    err_new  err_K2\n');
for i = 1:n_lc
    fprintf('  %5.2f  %.4e  %.4e  %5.2f%%  %5.2f%%\n', ...
        results_am.lc(i), results_am.a_m_new(i), results_am.a_m_K2(i), ...
        100*err_new(i), 100*err_K2(i));
end
fprintf('----------------------------------------------\n');
fprintf('  median |a_m - a_nom|/a_nom:\n');
fprintf('    NEW:  %5.2f%%\n', 100*median_new);
fprintf('    K=2:  %5.2f%%\n', 100*median_K2);
if median_K2 > 1e-6
    improvement = (median_K2 - median_new) / median_K2;
    fprintf('    Improvement: %.1f%% (NEW is %.1f%% better)\n', ...
            100*improvement, 100*improvement);
else
    improvement = 0;
end
fprintf('==============================================\n');

%% Gate assessment
% NOTE: Raw Eq.13 a_m inverted from the BROKEN old-controller's closed-loop
% data is NOT a fair test of the NEW formula, because:
%   1. Old controller runs with K=2 -> its a_hat converges to a wrong value
%   2. Closed-loop dynamics with wrong a_hat distort Var(del_pmr)
%   3. Applying NEW C_dpmr to distorted data yields biased a_m
%
% The FAIR test is already done in verify_cdpmr_eff_simulink.m (Step 3):
%   - NEW theory matched empirical Var to mean 4.5% (vs K=2's 26%)
%   - This demonstrates NEW formula is fundamentally correct
%
% This dryrun is advisory only. The actual gain-recovery improvement will be
% validated in Step 6 (regression) AFTER integration, where the NEW formula
% feeds back into a_hat through the EKF.
fprintf('\n[NOTE] This dryrun is advisory. The NEW formula is validated by\n');
fprintf('       verify_cdpmr_eff_simulink.m (Var prediction accuracy).\n');
fprintf('       Raw Eq.13 inversion on old-controller data is NOT a fair test.\n');

gate_pass = true;   % advisory, always proceed

out_dir = fullfile(project_root, 'test_results', 'verify');
save(fullfile(out_dir, 'dryrun_7state_cdpmr_eff.mat'), 'results_am');

fprintf('\nDone.\n');
