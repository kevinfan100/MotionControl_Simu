%% verify_cdpmr_eff_simulink.m — Verify C_dpmr_eff lookup against Simulink
%
% Runs 7-state EKF in free-space (positioning), measures empirical
% Var(del_pmr) from the raw tracking error via offline IIR HP filter,
% compares with:
%   (a) theoretical Var_theory_new  = C_dpmr_eff * sigma2_dXT + C_np_eff * sigma2_n
%   (b) theoretical Var_theory_K2   = C_dpmr_K2  * sigma2_dXT + noise_corr_K2 * sigma2_n
%
% Tests 6 points: 3 lc × 2 noise levels (rho ~ 0 and rho ~ 0.05)
% Gate: mean |error_new| < 5%, max |error_new| < 10%, and new better than K=2

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

fprintf('===== verify_cdpmr_eff_simulink =====\n\n');

%% Physical constants
constants = physical_constants();
k_B = constants.k_B; T_temp = constants.T;
Ts = constants.Ts; gamma_N = constants.gamma_N;
a_nom = Ts / gamma_N;
sigma2_dXT = 4 * k_B * T_temp * Ts / gamma_N;

fprintf('a_nom = %.6e um/pN\n', a_nom);
fprintf('sigma2_dXT = %.6e um^2\n\n', sigma2_dXT);

%% Load lookup table
lookup_path = fullfile(project_root, 'test_results', 'verify', 'cdpmr_eff_lookup.mat');
if ~exist(lookup_path, 'file')
    error('verify_cdpmr_eff_simulink:lookup_missing', ...
          'Run build_cdpmr_eff_lookup.m first to create %s', lookup_path);
end
LUT = load(lookup_path);
fprintf('Loaded lookup table (built %s)\n', LUT.build_timestamp);

%% Test grid
lc_test_list    = [0.5, 0.7, 0.9];                          % 3 lc values
% Noise levels: zero noise + typical project noise (a std = 0.003 um)
noise_std_list  = [0, 0.003];                                % [um]
n_lc    = length(lc_test_list);
n_noise = length(noise_std_list);
T_sim   = 10;            % long enough for variance statistics

results = struct();
results.lc = [];
results.noise_std = [];
results.rho = [];
results.Cdpmr_new = [];
results.Cnp_new = [];
results.Cdpmr_K2 = [];
results.Cnp_K2 = [];
results.Var_theory_new = [];
results.Var_theory_K2 = [];
results.Var_empirical_z = [];
results.Var_empirical_x = [];
results.Var_empirical_y = [];
results.err_new_pct = [];
results.err_K2_pct = [];

%% Load Simulink model once
model_path = fullfile(project_root, 'model', 'system_model');
load_system(model_path);
fprintf('Model loaded.\n\n');

idx = 0;
for i_lc = 1:n_lc
    for i_n = 1:n_noise
        idx = idx + 1;
        lc = lc_test_list(i_lc);
        noise_std = noise_std_list(i_n);
        sigma2_n = noise_std^2;
        rho = sigma2_n / sigma2_dXT;

        fprintf('--- [%d/%d] lc=%.2f, noise_std=%.4f (rho=%.3g) ---\n', ...
                idx, n_lc*n_noise, lc, noise_std, rho);

        % Clear persistents
        clear motion_control_law motion_control_law_7state ...
              motion_control_law_23state trajectory_generator calc_thermal_force;

        % Config: free space, positioning, thermal on, optional noise
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
        config.meas_noise_enable = (noise_std > 0);
        config.meas_noise_std = [noise_std; noise_std; noise_std];
        config.T_sim = T_sim;

        params = calc_simulation_params(config);
        assignin('base', 'params', params);
        assignin('base', 'p0', params.Value.common.p0);
        assignin('base', 'Ts', Ts);

        rng(42 + idx);   % reproducible seed per point

        fprintf('  Running Simulink (T_sim=%.1f)...', T_sim);
        t0 = tic;
        simOut = sim(model_path, ...
            'StopTime', num2str(T_sim), ...
            'SaveTime', 'on', 'TimeSaveName', 'tout', ...
            'SaveOutput', 'on', 'OutputSaveName', 'yout');
        fprintf(' done (%.1f sec)\n', toc(t0));

        p_d_log = simOut.p_d_out';       % 3 x N
        p_m_log = simOut.p_m_out';
        N_sim = size(p_d_log, 2);

        % Reconstruct del_pm = p_d[k-2] - p_m[k] (controller notation)
        del_pm = zeros(3, N_sim);
        for k = 3:N_sim
            del_pm(:, k) = p_d_log(:, k-2) - p_m_log(:, k);
        end

        % Offline IIR HP filter (same parameters as controller)
        a_pd = config.a_pd;
        del_pmd = zeros(3, N_sim);
        del_pmr = zeros(3, N_sim);
        for k = 2:N_sim
            del_pmd(:, k) = (1-a_pd) * del_pmd(:, k-1) + a_pd * del_pm(:, k);
            del_pmr(:, k) = del_pm(:, k) - del_pmd(:, k);
        end

        % Steady-state window: last 50%
        ss = round(N_sim/2):N_sim;

        Var_emp_x = var(del_pmr(1, ss));
        Var_emp_y = var(del_pmr(2, ss));
        Var_emp_z = var(del_pmr(3, ss));
        Var_emp_mean = mean([Var_emp_x, Var_emp_y, Var_emp_z]);

        % Theory (new) — interpolate over lc from lookup
        Cdpmr_new = interp1(LUT.lc_grid, LUT.Cdpmr_tab(:, 1), lc, 'linear');
        Cnp_new   = interp1(LUT.lc_grid, LUT.Cnp_tab(:, 1), lc, 'linear');
        Var_th_new = Cdpmr_new * sigma2_dXT + Cnp_new * sigma2_n;

        % Theory (old K=2 formula)
        Cdpmr_K2 = (1-a_pd)^2 * (2*(1-a_pd)*(1-lc) / (1-(1-a_pd)*lc) ...
                 + (2/(2-a_pd)) / ((1+lc)*(1-(1-a_pd)*lc)));
        noise_corr_K2 = 2 / (1 + lc);  % the old wrong formula divided by sigma2_n
        Var_th_K2 = Cdpmr_K2 * sigma2_dXT + noise_corr_K2 * sigma2_n;

        % Errors against empirical (use mean of 3 axes for robustness)
        err_new_pct = (Var_emp_mean - Var_th_new) / Var_th_new * 100;
        err_K2_pct  = (Var_emp_mean - Var_th_K2)  / Var_th_K2  * 100;

        fprintf('  Var_emp (xyz mean) = %.4e um^2\n', Var_emp_mean);
        fprintf('  Var_theory_new     = %.4e um^2  (C_dpmr=%.3f, C_np=%.3f)\n', ...
                Var_th_new, Cdpmr_new, Cnp_new);
        fprintf('  Var_theory_K2      = %.4e um^2  (C_dpmr=%.3f, C_np=%.3f)\n', ...
                Var_th_K2, Cdpmr_K2, noise_corr_K2);
        fprintf('  Error new = %+6.2f%%  |  Error K=2 = %+6.2f%%\n', ...
                err_new_pct, err_K2_pct);

        % Store
        results.lc(end+1) = lc;
        results.noise_std(end+1) = noise_std;
        results.rho(end+1) = rho;
        results.Cdpmr_new(end+1) = Cdpmr_new;
        results.Cnp_new(end+1) = Cnp_new;
        results.Cdpmr_K2(end+1) = Cdpmr_K2;
        results.Cnp_K2(end+1) = noise_corr_K2;
        results.Var_theory_new(end+1) = Var_th_new;
        results.Var_theory_K2(end+1) = Var_th_K2;
        results.Var_empirical_z(end+1) = Var_emp_z;
        results.Var_empirical_x(end+1) = Var_emp_x;
        results.Var_empirical_y(end+1) = Var_emp_y;
        results.err_new_pct(end+1) = err_new_pct;
        results.err_K2_pct(end+1) = err_K2_pct;
    end
end

close_system(model_path, 0);

%% Summary table
fprintf('\n');
fprintf('===================================================================\n');
fprintf('  Summary: theory vs empirical Var(del_pmr)\n');
fprintf('===================================================================\n');
fprintf('    lc    noise    rho    Var_emp     Var_new     err_new   err_K2\n');
fprintf('  ----------------------------------------------------------------\n');
n_pts = length(results.lc);
for i = 1:n_pts
    fprintf('  %5.2f  %6.4f  %6.3g  %.3e  %.3e  %+6.2f%%  %+6.2f%%\n', ...
        results.lc(i), results.noise_std(i), results.rho(i), ...
        mean([results.Var_empirical_x(i), results.Var_empirical_y(i), results.Var_empirical_z(i)]), ...
        results.Var_theory_new(i), results.err_new_pct(i), results.err_K2_pct(i));
end

%% Gate assessment
abs_err_new = abs(results.err_new_pct);
abs_err_K2  = abs(results.err_K2_pct);
mean_err_new = mean(abs_err_new);
max_err_new  = max(abs_err_new);
mean_err_K2  = mean(abs_err_K2);
max_err_K2   = max(abs_err_K2);

fprintf('\n===================================================================\n');
fprintf('  Statistics:\n');
fprintf('    NEW (lookup) :  mean |err| = %5.2f%%   max |err| = %5.2f%%\n', ...
        mean_err_new, max_err_new);
fprintf('    OLD  (K=2)   :  mean |err| = %5.2f%%   max |err| = %5.2f%%\n', ...
        mean_err_K2, max_err_K2);

% Primary gate: new formula accuracy
GATE_MEAN = 5.0;    % mean |err| < 5%
GATE_MAX  = 10.0;   % max  |err| < 10%
gate_mean = mean_err_new < GATE_MEAN;
gate_max  = max_err_new  < GATE_MAX;
gate_improvement = mean_err_new < mean_err_K2;  % new must be better than K=2
pass = gate_mean && gate_max && gate_improvement;

fprintf('\n  Gate: mean<%.1f%%:%s  max<%.1f%%:%s  improves_over_K2:%s\n', ...
        GATE_MEAN, pass_str(gate_mean), GATE_MAX, pass_str(gate_max), ...
        pass_str(gate_improvement));
fprintf('===================================================================\n');

%% Save results
out_dir = fullfile(project_root, 'test_results', 'verify');
save(fullfile(out_dir, 'verify_cdpmr_eff_simulink_results.mat'), 'results');
fprintf('\nSaved: verify_cdpmr_eff_simulink_results.mat\n');

%% Scatter figure
fig_dir = fullfile(project_root, 'reference', 'qr_analysis');
fig = figure('Position', [100 100 900 700]);
subplot(1,2,1);
plot(results.Var_theory_new, [results.Var_empirical_x; results.Var_empirical_y; results.Var_empirical_z]', ...
     'o', 'MarkerSize', 8, 'LineWidth', 1.5);
hold on;
lims = [min([results.Var_theory_new, results.Var_empirical_z])*0.9, ...
        max([results.Var_theory_new, results.Var_empirical_z])*1.1];
plot(lims, lims, 'k--', 'LineWidth', 1.5);
hold off;
xlabel('Var_{theory,new} [um^2]');
ylabel('Var_{empirical} [um^2]');
title('New (augmented Lyapunov)');
legend({'x','y','z','ideal'}, 'Location', 'northwest');
axis equal;
set(gca, 'FontSize', 12);

subplot(1,2,2);
plot(results.Var_theory_K2, [results.Var_empirical_x; results.Var_empirical_y; results.Var_empirical_z]', ...
     's', 'MarkerSize', 8, 'LineWidth', 1.5);
hold on;
lims = [min([results.Var_theory_K2, results.Var_empirical_z])*0.9, ...
        max([results.Var_theory_K2, results.Var_empirical_z])*1.1];
plot(lims, lims, 'k--', 'LineWidth', 1.5);
hold off;
xlabel('Var_{theory,K=2} [um^2]');
ylabel('Var_{empirical} [um^2]');
title('Old (K=2 approximation)');
legend({'x','y','z','ideal'}, 'Location', 'northwest');
axis equal;
set(gca, 'FontSize', 12);

saveas(fig, fullfile(fig_dir, 'fig_cdpmr_eff_verification.png'));
fprintf('Figure saved: fig_cdpmr_eff_verification.png\n');

if ~pass
    error('verify_cdpmr_eff_simulink:gate_fail', ...
          'Simulink gate failed: mean=%.2f%% max=%.2f%%', mean_err_new, max_err_new);
end

fprintf('\nDone.\n');

function s = pass_str(b)
    if b, s = 'PASS'; else, s = 'FAIL'; end
end
